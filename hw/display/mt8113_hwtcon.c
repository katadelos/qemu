/*
 * MediaTek MT8113 hardware timing controller
 *
 * Coloursoft uses MediaTek's HWTCON v2 display pipeline rather than the
 * i.MX EPDC block used by earlier Kindles.  Keep the register model separate
 * even while only the bootloader-visible register storage is implemented.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/display/mt8113_hwtcon.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "trace.h"

#define WF_LUT_INTEN             0x4004
#define WF_LUT_INTSTA            0x4008
#define WF_LUT_EN_0              0x4d04
#define WF_LUT_EN_1              0x4d08
#define WF_LUT_CON               0x4d0c
#define WF_LUT_END_IRQ_CLR0      0x4d50
#define WF_LUT_END_IRQ_CLR1      0x4d54
#define WF_LUT_END_IRQ_STA0      0x4d58
#define WF_LUT_END_IRQ_STA1      0x4d5c
#define WF_LUT_EN_STA0           0x4270
#define WF_LUT_EN_STA1           0x4274

#define WB_WDMA_INTEN            0xa000
#define WB_WDMA_INTSTA           0xa004

#define PAPER_TCTOP_UPD_CFG2     0xd028
#define PAPER_TCTOP_UPD_CFG3     0xd02c
#define PAPER_TCTOP_PANEL_SIZE   0xd00c

#define MDP_RDMA_SRC_CON         0x7030
#define MDP_RDMA_SRC_PITCH       0x7060
#define MDP_RDMA_SRC_SIZE        0x7070
#define MDP_RDMA_SRC_BASE        0x7f00
#define MDP_RDMA_FORMAT_MASK     0xf
#define MDP_RDMA_FORMAT_RGBA8888 0x2
#define MDP_RDMA_FORMAT_Y8       0x7

#define HWTCON_LUT_COMPLETE_NS   (10 * SCALE_MS)
#define HWTCON_SCANOUT_INTERVAL_MS 2000
#define HWTCON_IOMMU_SECTION_SIZE 0x100000
#define HWTCON_PANEL_WIDTH       1272
#define HWTCON_PANEL_HEIGHT      1696

static uint32_t *mt8113_hwtcon_reg(MT8113HWTCONState *s, hwaddr offset)
{
    return &s->bank[0].regs[offset / sizeof(uint32_t)];
}

static void mt8113_hwtcon_update_irq(MT8113HWTCONState *s)
{
    uint32_t wf_status = *mt8113_hwtcon_reg(s, WF_LUT_INTSTA);
    uint32_t wf_enable = *mt8113_hwtcon_reg(s, WF_LUT_INTEN);
    uint32_t release0 = *mt8113_hwtcon_reg(s, WF_LUT_END_IRQ_STA0);
    uint32_t release1 = *mt8113_hwtcon_reg(s, WF_LUT_END_IRQ_STA1);
    uint32_t wb_status = *mt8113_hwtcon_reg(s, WB_WDMA_INTSTA);
    uint32_t wb_enable = *mt8113_hwtcon_reg(s, WB_WDMA_INTEN);

    qemu_set_irq(s->irq[MT8113_HWTCON_WB_FRAME_DONE_IRQ],
                 (wb_status & wb_enable) != 0);
    qemu_set_irq(s->irq[MT8113_HWTCON_WF_LUT_FRAME_DONE_IRQ],
                 (wf_status & wf_enable) != 0);
    qemu_set_irq(s->irq[MT8113_HWTCON_WF_LUT_RELEASE_IRQ],
                 (release0 | release1) != 0);
}

static DisplaySurface *mt8113_hwtcon_prepare_surface(MT8113HWTCONState *s,
                                                      unsigned width,
                                                      unsigned height)
{
    DisplaySurface *surface = qemu_console_surface(s->console);

    if (!surface || surface_width(surface) != width ||
        surface_height(surface) != height || surface_is_placeholder(surface)) {
        qemu_console_resize(s->console, width, height);
        surface = qemu_console_surface(s->console);
        memset(surface_data(surface), 0xff,
               (size_t)surface_stride(surface) * height);
    }
    return surface;
}

static void mt8113_hwtcon_render_framebuffer(MT8113HWTCONState *s)
{
    DisplaySurface *surface;
    size_t buffer_size;
    unsigned bytes_per_pixel;

    if (!s->console || !qemu_console_is_visible(s->console) || !s->iommu ||
        !s->fb_iova || !s->fb_width ||
        !s->fb_height || s->fb_pitch < s->fb_width ||
        s->fb_width > 4096 || s->fb_height > 4096) {
        return;
    }

    bytes_per_pixel = s->fb_format == MDP_RDMA_FORMAT_RGBA8888 ? 4 : 1;
    buffer_size = (size_t)s->fb_pitch * s->fb_height * bytes_per_pixel;
    s->fb_buffer = g_realloc(s->fb_buffer, buffer_size);
    if (!mt8113_iommu_dma_read(s->iommu, s->fb_iova,
                               s->fb_buffer, buffer_size)) {
        return;
    }

    surface = mt8113_hwtcon_prepare_surface(s, s->fb_width, s->fb_height);
    for (unsigned y = 0; y < s->fb_height; y++) {
        uint32_t *pixels = (uint32_t *)((uint8_t *)surface_data(surface) +
                                        (size_t)y * surface_stride(surface));
        const uint8_t *source = s->fb_buffer +
                                (size_t)y * s->fb_pitch * bytes_per_pixel;

        for (unsigned x = 0; x < s->fb_width; x++) {
            uint8_t red, green, blue;

            if (bytes_per_pixel == 4) {
                red = source[x * 4];
                green = source[x * 4 + 1];
                blue = source[x * 4 + 2];
            } else {
                red = green = blue = source[x];
            }

            pixels[x] = 0xff000000U | red << 16 | green << 8 | blue;
        }
    }
    dpy_gfx_update_full(s->console);
}

static void mt8113_hwtcon_invalidate_display(void *opaque)
{
    MT8113HWTCONState *s = opaque;

    if (s->console) {
        dpy_gfx_update_full(s->console);
    }
}

static void mt8113_hwtcon_refresh_scanout(void *opaque)
{
    MT8113HWTCONState *s = opaque;

    mt8113_hwtcon_render_framebuffer(s);
    timer_mod(s->refresh_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
              HWTCON_SCANOUT_INTERVAL_MS);
}

static const GraphicHwOps mt8113_hwtcon_gfx_ops = {
    .invalidate = mt8113_hwtcon_invalidate_display,
    .gfx_update = mt8113_hwtcon_invalidate_display,
};

static void mt8113_hwtcon_complete_luts(void *opaque)
{
    MT8113HWTCONState *s = opaque;
    uint32_t released0 = s->pending_luts;
    uint32_t released1 = s->pending_luts >> 32;

    s->pending_luts = 0;
    *mt8113_hwtcon_reg(s, WF_LUT_EN_0) &= ~released0;
    *mt8113_hwtcon_reg(s, WF_LUT_EN_1) &= ~released1;
    *mt8113_hwtcon_reg(s, WF_LUT_EN_STA0) &= ~released0;
    *mt8113_hwtcon_reg(s, WF_LUT_EN_STA1) &= ~released1;
    *mt8113_hwtcon_reg(s, WF_LUT_END_IRQ_STA0) |= released0;
    *mt8113_hwtcon_reg(s, WF_LUT_END_IRQ_STA1) |= released1;
    *mt8113_hwtcon_reg(s, WF_LUT_INTSTA) |= BIT(1);
    mt8113_hwtcon_update_irq(s);
    qemu_irq_pulse(s->gce_frame_done);
}

static void mt8113_hwtcon_enable_luts(MT8113HWTCONState *s,
                                      unsigned half, uint32_t value)
{
    uint64_t enabled = (uint64_t)value << (half * 32);

    if (!enabled) {
        return;
    }

    s->pending_luts |= enabled;
    *mt8113_hwtcon_reg(s, WF_LUT_EN_STA0 + half * 4) |= value;
    timer_mod(s->lut_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              HWTCON_LUT_COMPLETE_NS);
}

static uint64_t mt8113_hwtcon_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    MT8113HWTCONBank *bank = opaque;

    return bank->regs[offset / sizeof(uint32_t)];
}

static void mt8113_hwtcon_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    MT8113HWTCONBank *bank = opaque;
    MT8113HWTCONState *s = bank->owner;
    uint32_t old = bank->regs[offset / sizeof(uint32_t)];

    bank->regs[offset / sizeof(uint32_t)] = value;

    if (bank->index != 0) {
        if (offset == MDP_RDMA_SRC_CON) {
            uint32_t size = bank->regs[MDP_RDMA_SRC_SIZE / 4];
            uint32_t width = size & 0x3fff;
            uint32_t height = (size >> 16) & 0x3fff;
            uint32_t panel = *mt8113_hwtcon_reg(s,
                                                 PAPER_TCTOP_PANEL_SIZE);
            uint32_t format = value & MDP_RDMA_FORMAT_MASK;

            bool captured = width && height &&
                            width == (panel & 0x3fff) &&
                            height == ((panel >> 16) & 0x3fff) &&
                            (format == MDP_RDMA_FORMAT_RGBA8888 ||
                             format == MDP_RDMA_FORMAT_Y8);

            trace_mt8113_hwtcon_mdp_source(
                bank->regs[MDP_RDMA_SRC_BASE / 4],
                format, width, height,
                bank->regs[MDP_RDMA_SRC_PITCH / 4] & 0x1fffff,
                panel & 0x3fff, (panel >> 16) & 0x3fff, captured);
            if (captured) {
                s->fb_width = width;
                s->fb_height = height;
                s->fb_pitch = bank->regs[MDP_RDMA_SRC_PITCH / 4] & 0x1fffff;
                if (format == MDP_RDMA_FORMAT_Y8) {
                    size_t fb_size = (size_t)width * height * 4 * 2;
                    size_t fb_iova_span = QEMU_ALIGN_UP(
                        fb_size, HWTCON_IOMMU_SECTION_SIZE);
                    uint32_t source = bank->regs[MDP_RDMA_SRC_BASE / 4];
                    hwaddr physical;

                    /*
                     * The CFA pipeline gives MDP the converted Y8 buffer,
                     * while userspace draws into the earlier RGBA fbdev
                     * allocation.  The MTK ARMv7s DMA domain allocates that
                     * two-frame buffer in a section-aligned IOVA span.
                     */
                    if (source > fb_iova_span &&
                        mt8113_iommu_translate(s->iommu,
                                               source - fb_iova_span,
                                               &physical)) {
                        s->fb_iova = source - fb_iova_span;
                        s->fb_format = MDP_RDMA_FORMAT_RGBA8888;
                    }
                } else {
                    s->fb_iova = bank->regs[MDP_RDMA_SRC_BASE / 4];
                    s->fb_format = format;
                }
            }
        }
        return;
    }

    switch (offset) {
    case WF_LUT_EN_0:
        mt8113_hwtcon_enable_luts(s, 0, value);
        break;
    case WF_LUT_EN_1:
        mt8113_hwtcon_enable_luts(s, 1, value);
        break;
    case WF_LUT_END_IRQ_CLR0:
        *mt8113_hwtcon_reg(s, WF_LUT_END_IRQ_STA0) &= ~(uint32_t)value;
        break;
    case WF_LUT_END_IRQ_CLR1:
        *mt8113_hwtcon_reg(s, WF_LUT_END_IRQ_STA1) &= ~(uint32_t)value;
        break;
    case WF_LUT_CON:
        if ((value & BIT(15)) && !(old & BIT(15))) {
            *mt8113_hwtcon_reg(s, WF_LUT_END_IRQ_STA0) = 0;
            *mt8113_hwtcon_reg(s, WF_LUT_END_IRQ_STA1) = 0;
        }
        break;
    case PAPER_TCTOP_UPD_CFG2:
        if ((value & BIT(0)) && !(old & BIT(0))) {
            *mt8113_hwtcon_reg(s, PAPER_TCTOP_UPD_CFG3) |= BIT(2) | BIT(6);
            *mt8113_hwtcon_reg(s, WB_WDMA_INTSTA) |= BIT(0);
        }
        break;
    default:
        break;
    }

    mt8113_hwtcon_update_irq(s);
}

static const MemoryRegionOps mt8113_hwtcon_ops = {
    .read = mt8113_hwtcon_read,
    .write = mt8113_hwtcon_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mt8113_hwtcon_reset(DeviceState *dev)
{
    MT8113HWTCONState *s = MT8113_HWTCON(dev);

    for (unsigned i = 0; i < ARRAY_SIZE(s->bank); i++) {
        memset(s->bank[i].regs, 0, sizeof(s->bank[i].regs));
    }
    s->pending_luts = 0;
    s->fb_iova = 0;
    s->fb_width = 0;
    s->fb_height = 0;
    s->fb_pitch = 0;
    s->fb_format = 0;
    timer_del(s->lut_timer);
    for (unsigned i = 0; i < MT8113_HWTCON_NUM_IRQS; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
}

static void mt8113_hwtcon_init(Object *obj)
{
    MT8113HWTCONState *s = MT8113_HWTCON(obj);

    for (unsigned i = 0; i < ARRAY_SIZE(s->bank); i++) {
        g_autofree char *name =
            g_strdup_printf(TYPE_MT8113_HWTCON ".bank%u", i);

        s->bank[i].owner = s;
        s->bank[i].index = i;
        memory_region_init_io(&s->bank[i].iomem, obj,
                              &mt8113_hwtcon_ops, &s->bank[i], name,
                              MT8113_HWTCON_MMIO_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->bank[i].iomem);
    }
    for (unsigned i = 0; i < MT8113_HWTCON_NUM_IRQS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), &s->gce_frame_done,
                             "gce-frame-done", 1);
    s->lut_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                mt8113_hwtcon_complete_luts, s);
}

static void mt8113_hwtcon_finalize(Object *obj)
{
    MT8113HWTCONState *s = MT8113_HWTCON(obj);

    timer_free(s->lut_timer);
}

static void mt8113_hwtcon_realize(DeviceState *dev, Error **errp)
{
    MT8113HWTCONState *s = MT8113_HWTCON(dev);

    if (!s->iommu) {
        error_setg(errp, "HWTCON requires an MT8113 IOMMU link");
        return;
    }
    s->console = graphic_console_init(dev, 0, &mt8113_hwtcon_gfx_ops, s);
    mt8113_hwtcon_prepare_surface(s, HWTCON_PANEL_WIDTH,
                                  HWTCON_PANEL_HEIGHT);
    s->refresh_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                    mt8113_hwtcon_refresh_scanout, s);
    timer_mod(s->refresh_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
              HWTCON_SCANOUT_INTERVAL_MS);
}

static void mt8113_hwtcon_unrealize(DeviceState *dev)
{
    MT8113HWTCONState *s = MT8113_HWTCON(dev);

    timer_free(s->refresh_timer);
    s->refresh_timer = NULL;
    g_free(s->fb_buffer);
    s->fb_buffer = NULL;
}

static const Property mt8113_hwtcon_properties[] = {
    DEFINE_PROP_LINK("iommu", MT8113HWTCONState, iommu,
                     TYPE_MT8113_IOMMU, MT8113IOMMUState *),
};

static void mt8113_hwtcon_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mt8113_hwtcon_realize;
    dc->unrealize = mt8113_hwtcon_unrealize;
    device_class_set_legacy_reset(dc, mt8113_hwtcon_reset);
    device_class_set_props(dc, mt8113_hwtcon_properties);
}

static const TypeInfo mt8113_hwtcon_type = {
    .name = TYPE_MT8113_HWTCON,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8113HWTCONState),
    .instance_init = mt8113_hwtcon_init,
    .instance_finalize = mt8113_hwtcon_finalize,
    .class_init = mt8113_hwtcon_class_init,
};

static void mt8113_hwtcon_register_types(void)
{
    type_register_static(&mt8113_hwtcon_type);
}
type_init(mt8113_hwtcon_register_types)
