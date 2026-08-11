/*
 * i.MX50 electrophoretic display controller
 *
 * This models the older 16-LUT EPDC contract used by the Freescale 2.6.31
 * framebuffer driver.  Waveform timing is collapsed, but working-buffer and
 * LUT completion remain separate because the driver uses them as two queue
 * ownership transitions.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/display/imx50_epdc.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/dma.h"
#include "trace.h"
#include "ui/console.h"

enum {
    EPDC_CTRL = 0x000,
    EPDC_RES = 0x040,
    EPDC_FORMAT = 0x050,
    EPDC_FIFOCTRL = 0x0a0,
    EPDC_UPD_ADDR = 0x100,
    EPDC_UPD_CORD = 0x120,
    EPDC_UPD_SIZE = 0x140,
    EPDC_UPD_CTRL = 0x160,
    EPDC_IRQ_MASK = 0x400,
    EPDC_IRQ = 0x420,
    EPDC_STATUS_LUTS = 0x440,
    EPDC_STATUS_NEXTLUT = 0x460,
    EPDC_STATUS = 0x4a0,
};

#define EPDC_CTRL_SFTRST        (1U << 31)
#define EPDC_CTRL_CLKGATE       (1U << 30)
#define EPDC_UPD_CTRL_USE_FIXED (1U << 31)
#define EPDC_UPD_CTRL_AUTOWV_PAUSE (1U << 3)
#define EPDC_IRQ_WB_CMPLT       (1U << 16)
#define EPDC_IRQ_UPD_DONE       (1U << 22)
#define EPDC_STATUS_WB_BUSY     (1U << 0)
#define EPDC_UPD_LUT_SHIFT      16
#define EPDC_UPD_LUT_MASK       0xf
#define EPDC_MAX_DIMENSION      2048
#define EPDC_WB_COMPLETE_NS     1000000
#define EPDC_LUT_COMPLETE_NS    10000000
#define EPDC_SCANOUT_REFRESH_MS 50

static inline uint32_t *epdc_reg(IMX50EPDCState *s, hwaddr offset)
{
    return &s->regs[offset >> 2];
}

static void imx50_epdc_update_irq(IMX50EPDCState *s)
{
    qemu_set_irq(s->irq,
                 (*epdc_reg(s, EPDC_IRQ_MASK) &
                  *epdc_reg(s, EPDC_IRQ)) != 0);
}

static bool imx50_epdc_geometry(IMX50EPDCState *s,
                                unsigned *width, unsigned *height)
{
    uint32_t res = *epdc_reg(s, EPDC_RES);

    *width = extract32(res, 0, 13);
    *height = extract32(res, 16, 13);
    return *width && *height &&
           *width <= EPDC_MAX_DIMENSION &&
           *height <= EPDC_MAX_DIMENSION;
}

static DisplaySurface *imx50_epdc_prepare_surface(IMX50EPDCState *s,
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

static bool imx50_epdc_render_direct(IMX50EPDCState *s,
                                     unsigned surface_width,
                                     unsigned surface_height)
{
    DisplaySurface *surface;
    size_t buffer_size;
    MemTxResult result;
    unsigned x, y;

    if (!s->direct_fb) {
        return false;
    }
    if (!s->direct_fb_addr || s->direct_fb_stride < surface_width) {
        warn_report_once("%s: invalid direct scanout 0x%" PRIx64
                         " stride %u for %ux%u",
                         TYPE_IMX50_EPDC, s->direct_fb_addr,
                         s->direct_fb_stride, surface_width, surface_height);
        return true;
    }
    s->scanout_refreshes++;

    /*
     * mxc_epdc_fb_check_var() aligns xres_virtual to 32 pixels and the
     * Lab126 setup selects 8-bit grayscale before rotating Celeste CCW.
     * Snapshot that complete software framebuffer, using fix.line_length
     * exactly as the stock driver does when it feeds PxP.
     */
    buffer_size = (size_t)s->direct_fb_stride * surface_height;
    s->direct_fb_buffer = g_realloc(s->direct_fb_buffer, buffer_size);
    result = dma_memory_read(&address_space_memory, s->direct_fb_addr,
                             s->direct_fb_buffer, buffer_size,
                             MEMTXATTRS_UNSPECIFIED);
    if (result != MEMTX_OK) {
        warn_report_once("%s: cannot read direct scanout at 0x%" PRIx64,
                         TYPE_IMX50_EPDC, s->direct_fb_addr);
        return true;
    }

    surface = imx50_epdc_prepare_surface(s, surface_width, surface_height);
    for (y = 0; y < surface_height; y++) {
        uint32_t *pixels;
        pixels = (uint32_t *)((uint8_t *)surface_data(surface) +
                              (size_t)y * surface_stride(surface));
        for (x = 0; x < surface_width; x++) {
            uint8_t gray = s->direct_fb_buffer[
                (size_t)y * s->direct_fb_stride + x];

            pixels[x] = 0xff000000U | gray * 0x00010101U;
        }
    }
    dpy_gfx_update_full(s->console);
    return true;
}

static void imx50_epdc_render_update(IMX50EPDCState *s,
                                     uint32_t source, uint32_t cord,
                                     uint32_t size)
{
    DisplaySurface *surface;
    g_autofree uint8_t *update = NULL;
    unsigned panel_width, panel_height;
    unsigned left = extract32(cord, 0, 13);
    unsigned top = extract32(cord, 16, 13);
    unsigned width = extract32(size, 0, 13);
    unsigned height = extract32(size, 16, 13);
    unsigned stride;
    unsigned display_left, display_top;
    unsigned display_width, display_height;
    uint32_t *pixels;
    MemTxResult result;
    int64_t started_ns;
    unsigned x, y;

    if (s->direct_fb) {
        if (s->direct_fb_width && s->direct_fb_height) {
            imx50_epdc_render_direct(s, s->direct_fb_width,
                                     s->direct_fb_height);
        } else if (imx50_epdc_geometry(s, &panel_width, &panel_height)) {
            imx50_epdc_render_direct(s, panel_height, panel_width);
        }
        return;
    }
    if (!imx50_epdc_geometry(s, &panel_width, &panel_height)) {
        return;
    }
    if (!source ||
        !width || !height || left + width > panel_width ||
        top + height > panel_height) {
        return;
    }

    /*
     * The i.MX50 block has no UPD_STRIDE register.  PxP emits update data in
     * 8-pixel blocks, while EPDC consumes only the requested rectangle.
     */
    stride = QEMU_ALIGN_UP(width, 8);
    trace_whitney_epdc_begin(source, left, top, width, height, stride);
    started_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    update = g_malloc((size_t)stride * height);
    result = dma_memory_read(&address_space_memory, source, update,
                             (size_t)stride * height,
                             MEMTXATTRS_UNSPECIFIED);
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: cannot read update buffer at 0x%08x\n",
                      TYPE_IMX50_EPDC, source);
        return;
    }

    /* The panel scan is landscape; boards select their physical mounting. */
    display_width = panel_height;
    display_height = panel_width;
    surface = imx50_epdc_prepare_surface(s, display_width, display_height);
    if (s->rotate_ccw) {
        display_left = top;
        display_top = panel_width - left - width;
    } else {
        display_left = panel_height - top - height;
        display_top = left;
    }

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            unsigned dest_x;
            unsigned dest_y;
            uint8_t gray = update[(size_t)y * stride + x];

            if (s->rotate_ccw) {
                dest_x = top + y;
                dest_y = panel_width - 1 - (left + x);
            } else {
                dest_x = panel_height - 1 - (top + y);
                dest_y = left + x;
            }
            pixels = (uint32_t *)((uint8_t *)surface_data(surface) +
                                  (size_t)dest_y * surface_stride(surface));
            pixels[dest_x] = 0xff000000U | gray * 0x00010101U;
        }
    }
    dpy_gfx_update(s->console, display_left, display_top, height, width);
    trace_whitney_epdc_render(width, height,
                              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) -
                              started_ns);
}

static void imx50_epdc_invalidate(void *opaque)
{
    IMX50EPDCState *s = opaque;
    unsigned panel_width, panel_height;

    if (s->direct_fb && s->direct_fb_width && s->direct_fb_height) {
        imx50_epdc_render_direct(s, s->direct_fb_width,
                                 s->direct_fb_height);
        return;
    }
    if (s->direct_fb &&
        imx50_epdc_geometry(s, &panel_width, &panel_height)) {
        imx50_epdc_render_direct(s, panel_height, panel_width);
        return;
    }

    if (s->console) {
        dpy_gfx_update_full(s->console);
    }
}

static void imx50_epdc_scanout_refresh(void *opaque)
{
    IMX50EPDCState *s = opaque;

    /* Keep the next refresh armed even if a display listener re-enters. */
    timer_mod(s->scanout_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
              EPDC_SCANOUT_REFRESH_MS);
    imx50_epdc_invalidate(s);
}

static const GraphicHwOps imx50_epdc_gfx_ops = {
    .invalidate = imx50_epdc_invalidate,
    .gfx_update = imx50_epdc_invalidate,
};

static void imx50_epdc_wb_complete(void *opaque)
{
    IMX50EPDCState *s = opaque;

    *epdc_reg(s, EPDC_STATUS) &= ~EPDC_STATUS_WB_BUSY;
    *epdc_reg(s, EPDC_IRQ) |= EPDC_IRQ_WB_CMPLT;
    trace_whitney_epdc_wb_complete(*epdc_reg(s, EPDC_IRQ));
    imx50_epdc_update_irq(s);
}

static void imx50_epdc_lut_complete(void *opaque)
{
    IMX50EPDCState *s = opaque;
    uint32_t completed = s->pending_luts;

    s->pending_luts = 0;
    *epdc_reg(s, EPDC_STATUS_LUTS) &= ~completed;
    *epdc_reg(s, EPDC_IRQ) |= completed;
    trace_whitney_epdc_lut_complete(completed, *epdc_reg(s, EPDC_IRQ));
    imx50_epdc_update_irq(s);
}

static uint64_t imx50_epdc_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX50EPDCState *s = opaque;

    if (offset == EPDC_STATUS_NEXTLUT) {
        uint32_t active = *epdc_reg(s, EPDC_STATUS_LUTS);
        unsigned lut;

        for (lut = 0; lut < 16 && (active & (1U << lut)); lut++) {
        }
        return lut < 16 ? 0x100 | lut : 0;
    }
    return *epdc_reg(s, offset);
}

static bool imx50_epdc_is_atomic(hwaddr base)
{
    switch (base) {
    case EPDC_CTRL:
    case EPDC_FORMAT:
    case EPDC_FIFOCTRL:
    case EPDC_IRQ_MASK:
    case EPDC_IRQ:
    case EPDC_STATUS_LUTS:
    case EPDC_STATUS:
        return true;
    default:
        return false;
    }
}

static void imx50_epdc_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    IMX50EPDCState *s = opaque;
    hwaddr base = offset & ~0xf;
    unsigned alias = offset & 0xc;
    uint32_t *reg;

    if (alias && imx50_epdc_is_atomic(base)) {
        reg = epdc_reg(s, base);
        if (alias == 4) {
            *reg |= value;
        } else if (alias == 8) {
            *reg &= ~(uint32_t)value;
        } else {
            *reg ^= value;
        }
    } else {
        *epdc_reg(s, offset) = value;
    }

    if (base == EPDC_CTRL && (*epdc_reg(s, EPDC_CTRL) & EPDC_CTRL_SFTRST)) {
        *epdc_reg(s, EPDC_CTRL) |= EPDC_CTRL_CLKGATE;
    }

    if (offset == EPDC_UPD_CTRL) {
        unsigned lut = extract32(value, EPDC_UPD_LUT_SHIFT,
                                 ctpop32(EPDC_UPD_LUT_MASK));

        /*
         * PxP has completed before the guest submits this update.  Snapshot
         * the DMA buffer now, before the driver can reuse the same LUT and
         * transient output buffer for a later rectangle.
         */
        if (!(value & EPDC_UPD_CTRL_USE_FIXED)) {
            imx50_epdc_render_update(s, *epdc_reg(s, EPDC_UPD_ADDR),
                                     *epdc_reg(s, EPDC_UPD_CORD),
                                     *epdc_reg(s, EPDC_UPD_SIZE));
        }
        *epdc_reg(s, EPDC_STATUS) |= EPDC_STATUS_WB_BUSY;
        *epdc_reg(s, EPDC_STATUS_LUTS) |= 1U << lut;
        s->pending_luts |= 1U << lut;
        trace_whitney_epdc_submit(lut,
                                  *epdc_reg(s, EPDC_STATUS_LUTS));

        /*
         * The i.MX50 presents working-buffer and LUT completion as distinct
         * ownership transitions.  Leave enough virtual time between them
         * for the guest to service WB_CMPLT before the LUT interrupt arrives;
         * otherwise both status bits are observed in one IRQ and Kobo's
         * 2.6.35 queue never reaches its final idle/powerdown check.
         */
        if (value & EPDC_UPD_CTRL_AUTOWV_PAUSE) {
            /*
             * Lab126 pauses AUTO waveform updates after histogram/working-
             * buffer processing.  Linux handles UPD_DONE, selects the final
             * waveform and VCOM, then rewrites UPD_CTRL with PAUSE clear.
             */
            *epdc_reg(s, EPDC_IRQ) |= EPDC_IRQ_UPD_DONE;
            imx50_epdc_update_irq(s);
        } else {
            timer_mod(s->wb_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      EPDC_WB_COMPLETE_NS);
            timer_mod(s->lut_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      EPDC_LUT_COMPLETE_NS);
        }
    }

    if (base == EPDC_IRQ_MASK || base == EPDC_IRQ) {
        imx50_epdc_update_irq(s);
    }
}

static const MemoryRegionOps imx50_epdc_ops = {
    .read = imx50_epdc_read,
    .write = imx50_epdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void imx50_epdc_reset(DeviceState *dev)
{
    IMX50EPDCState *s = IMX50_EPDC(dev);

    timer_del(s->wb_timer);
    timer_del(s->lut_timer);
    memset(s->regs, 0, sizeof(s->regs));
    *epdc_reg(s, EPDC_CTRL) = EPDC_CTRL_CLKGATE;
    s->pending_luts = 0;
    qemu_set_irq(s->irq, 0);
    if (s->direct_fb && s->scanout_timer) {
        timer_mod(s->scanout_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                  EPDC_SCANOUT_REFRESH_MS);
    }
}

static const VMStateDescription vmstate_imx50_epdc = {
    .name = TYPE_IMX50_EPDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMX50EPDCState,
                             IMX50_EPDC_MMIO_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32(pending_luts, IMX50EPDCState),
        VMSTATE_END_OF_LIST()
    },
};

static void imx50_epdc_realize(DeviceState *dev, Error **errp)
{
    IMX50EPDCState *s = IMX50_EPDC(dev);

    s->wb_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                               imx50_epdc_wb_complete, s);
    s->lut_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                imx50_epdc_lut_complete, s);
    s->scanout_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                    imx50_epdc_scanout_refresh, s);
    s->console = graphic_console_init(dev, 0, &imx50_epdc_gfx_ops, s);
    if (s->direct_fb) {
        timer_mod(s->scanout_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME));
    }
}

static void imx50_epdc_unrealize(DeviceState *dev)
{
    IMX50EPDCState *s = IMX50_EPDC(dev);

    timer_free(s->wb_timer);
    timer_free(s->lut_timer);
    timer_free(s->scanout_timer);
    g_free(s->direct_fb_buffer);
    s->direct_fb_buffer = NULL;
}

static void imx50_epdc_init(Object *obj)
{
    IMX50EPDCState *s = IMX50_EPDC(obj);

    memory_region_init_io(&s->iomem, obj, &imx50_epdc_ops, s,
                          TYPE_IMX50_EPDC, IMX50_EPDC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property imx50_epdc_properties[] = {
    DEFINE_PROP_BOOL("direct-framebuffer", IMX50EPDCState, direct_fb, false),
    DEFINE_PROP_UINT64("framebuffer-address", IMX50EPDCState,
                       direct_fb_addr, 0),
    DEFINE_PROP_UINT32("framebuffer-stride", IMX50EPDCState,
                       direct_fb_stride, 0),
    DEFINE_PROP_UINT32("framebuffer-width", IMX50EPDCState,
                       direct_fb_width, 0),
    DEFINE_PROP_UINT32("framebuffer-height", IMX50EPDCState,
                       direct_fb_height, 0),
    DEFINE_PROP_UINT64("scanout-refreshes", IMX50EPDCState,
                       scanout_refreshes, 0),
    DEFINE_PROP_BOOL("rotate-ccw", IMX50EPDCState, rotate_ccw, true),
};

static void imx50_epdc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = imx50_epdc_realize;
    dc->unrealize = imx50_epdc_unrealize;
    device_class_set_legacy_reset(dc, imx50_epdc_reset);
    dc->vmsd = &vmstate_imx50_epdc;
    dc->desc = "i.MX50 electrophoretic display controller";
    device_class_set_props(dc, imx50_epdc_properties);
}

static const TypeInfo imx50_epdc_info = {
    .name = TYPE_IMX50_EPDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX50EPDCState),
    .instance_init = imx50_epdc_init,
    .class_init = imx50_epdc_class_init,
};

static void imx50_epdc_register_types(void)
{
    type_register_static(&imx50_epdc_info);
}

type_init(imx50_epdc_register_types)
