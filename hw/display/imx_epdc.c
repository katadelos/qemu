/*
 * i.MX electrophoretic display controller (EPDC)
 *
 * This implements the software-visible update queue and interrupt contract
 * used by the Freescale 3.0.35 EPDC framebuffer driver.  Waveform timing and
 * the physical electrophoretic panel are intentionally collapsed into an
 * asynchronous completion event.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/display/imx_epdc.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "system/dma.h"
#include "ui/console.h"

enum {
    EPDC_CTRL = 0x000,
    EPDC_RES = 0x040,
    EPDC_FORMAT = 0x050,
    EPDC_UPD_ADDR = 0x100,
    EPDC_UPD_STRIDE = 0x110,
    EPDC_UPD_CORD = 0x120,
    EPDC_UPD_SIZE = 0x140,
    EPDC_UPD_CTRL = 0x160,
    EPDC_IRQ_MASK1 = 0x3c0,
    EPDC_IRQ_MASK2 = 0x3d0,
    EPDC_IRQ1 = 0x3e0,
    EPDC_IRQ2 = 0x3f0,
    EPDC_IRQ_MASK = 0x400,
    EPDC_IRQ = 0x420,
    EPDC_STATUS_LUTS = 0x440,
    EPDC_STATUS_LUTS2 = 0x450,
    EPDC_STATUS = 0x4a0,
    EPDC_VERSION = 0x7f0,
};

#define EPDC_CTRL_SFTRST        (1U << 31)
#define EPDC_CTRL_CLKGATE       (1U << 30)
#define EPDC_IRQ_WB_CMPLT       (1U << 16)
#define EPDC_IRQ_UPD_DONE       (1U << 22)
#define EPDC_UPD_LUT_SHIFT      16
#define EPDC_UPD_LUT_MASK       0x3f
#define EPDC_STATUS_WB_BUSY     (1U << 0)
#define EPDC_VERSION_2_1_0      0x02010000
#define EPDC_MAX_PANEL_DIMENSION 4096
#define EPDC_SCANOUT_INTERVAL_MS 100

static inline uint32_t *epdc_reg(IMXEPDCState *s, hwaddr offset)
{
    return &s->regs[offset >> 2];
}

static void imx_epdc_update_irq(IMXEPDCState *s)
{
    bool level = ((*epdc_reg(s, EPDC_IRQ1) &
                   *epdc_reg(s, EPDC_IRQ_MASK1)) != 0) ||
                 ((*epdc_reg(s, EPDC_IRQ2) &
                   *epdc_reg(s, EPDC_IRQ_MASK2)) != 0) ||
                 ((*epdc_reg(s, EPDC_IRQ) &
                   *epdc_reg(s, EPDC_IRQ_MASK)) != 0);

    qemu_set_irq(s->irq, level);
}

static bool imx_epdc_panel_geometry(IMXEPDCState *s,
                                    unsigned *width, unsigned *height)
{
    uint32_t res = *epdc_reg(s, EPDC_RES);

    *width = res & 0x1fff;
    *height = (res >> 16) & 0x1fff;
    return *width && *height &&
           *width <= EPDC_MAX_PANEL_DIMENSION &&
           *height <= EPDC_MAX_PANEL_DIMENSION;
}

static DisplaySurface *imx_epdc_prepare_surface(IMXEPDCState *s,
                                                unsigned width,
                                                unsigned height)
{
    DisplaySurface *surface;

    surface = qemu_console_surface(s->console);
    if (!surface || surface_width(surface) != width ||
        surface_height(surface) != height || surface_is_placeholder(surface)) {
        qemu_console_resize(s->console, width, height);
        surface = qemu_console_surface(s->console);
        memset(surface_data(surface), 0xff,
               (size_t)surface_stride(surface) * height);
    }
    return surface;
}

static bool imx_epdc_has_framebuffer(IMXEPDCState *s)
{
    return s->fb_addr && s->fb_width && s->fb_height &&
           (s->fb_bpp == 1 || s->fb_bpp == 2) &&
           s->fb_width <= EPDC_MAX_PANEL_DIMENSION &&
           s->fb_height <= EPDC_MAX_PANEL_DIMENSION &&
           s->fb_stride >= s->fb_width &&
           s->fb_stride <= EPDC_MAX_PANEL_DIMENSION * 4;
}

/*
 * /dev/fb0 is the software-visible display on Wario.  EPDC_UPD_ADDR points
 * at a transient PxP output buffer whose geometry is in panel scan order;
 * it is not the framebuffer and must not be used as the QEMU scanout.
 */
static void imx_epdc_render_framebuffer(IMXEPDCState *s)
{
    DisplaySurface *surface;
    uint32_t *pixels;
    size_t buffer_size;
    MemTxResult result;
    unsigned x, y;

    if (!s->console || !imx_epdc_has_framebuffer(s)) {
        return;
    }

    buffer_size = (size_t)s->fb_stride * s->fb_height * s->fb_bpp;
    s->fb_buffer = g_realloc(s->fb_buffer, buffer_size);
    result = dma_memory_read(&address_space_memory, s->fb_addr,
                             s->fb_buffer, buffer_size,
                             MEMTXATTRS_UNSPECIFIED);
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "imx.epdc: cannot read framebuffer at 0x%08" PRIx64
                      "\n", s->fb_addr);
        return;
    }

    surface = imx_epdc_prepare_surface(s, s->fb_width, s->fb_height);
    for (y = 0; y < s->fb_height; y++) {
        pixels = (uint32_t *)((uint8_t *)surface_data(surface) +
                              (size_t)y * surface_stride(surface));
        for (x = 0; x < s->fb_width; x++) {
            size_t offset = ((size_t)y * s->fb_stride + x) * s->fb_bpp;
            uint8_t gray;

            if (s->fb_bpp == 2) {
                uint16_t rgb565 = lduw_le_p(s->fb_buffer + offset);
                unsigned red = ((rgb565 >> 11) & 0x1f) * 255 / 31;
                unsigned green = ((rgb565 >> 5) & 0x3f) * 255 / 63;
                unsigned blue = (rgb565 & 0x1f) * 255 / 31;

                gray = (red * 77 + green * 150 + blue * 29 + 128) >> 8;
            } else {
                gray = s->fb_buffer[offset];
            }

            pixels[x] = 0xff000000U | gray * 0x00010101U;
        }
    }
    dpy_gfx_update_full(s->console);
}

static void imx_epdc_render_update(IMXEPDCState *s)
{
    DisplaySurface *surface;
    g_autofree uint8_t *update = NULL;
    uint32_t *pixels;
    uint32_t cord = *epdc_reg(s, EPDC_UPD_CORD);
    uint32_t size = *epdc_reg(s, EPDC_UPD_SIZE);
    uint32_t source_addr = *epdc_reg(s, EPDC_UPD_ADDR);
    IMX6SLPXPFetch fetch;
    unsigned panel_width, panel_height;
    unsigned left = cord & 0x1fff;
    unsigned top = (cord >> 16) & 0x1fff;
    unsigned width = size & 0x1fff;
    unsigned height = (size >> 16) & 0x1fff;
    unsigned source_stride = *epdc_reg(s, EPDC_UPD_STRIDE);
    size_t update_size;
    MemTxResult result;
    unsigned display_width, display_height;
    unsigned display_left, display_top;
    unsigned display_update_width, display_update_height;
    bool rotate_cw;
    unsigned x, y;

    if (!source_addr && s->pxp) {
        if (imx6sl_pxp_get_wfe_a_fetch(s->pxp, &fetch) ||
            imx6sl_pxp_get_wfe_b_store(s->pxp, &fetch)) {
            source_addr = fetch.addr;
            source_stride = fetch.pitch;
            source_addr += (uint64_t)fetch.top * source_stride + fetch.left;
        }
    }

    if (!s->console || !source_addr ||
        !imx_epdc_panel_geometry(s, &panel_width, &panel_height) ||
        !width || !height || left + width > panel_width ||
        top + height > panel_height) {
        return;
    }

    if (!source_stride) {
        source_stride = QEMU_ALIGN_UP(width, 32);
    }
    if (source_stride < width) {
        return;
    }
    update_size = (size_t)source_stride * height;
    update = g_malloc(update_size);
    result = dma_memory_read(&address_space_memory, source_addr,
                             update, update_size,
                             MEMTXATTRS_UNSPECIFIED);
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "imx.epdc: cannot read update buffer at 0x%08x\n",
                      source_addr);
        return;
    }

    /*
     * Icewine's ED060TC1 is scanned by the EPDC in its native 1448x1072
     * landscape order, while the assembled Kindle is viewed as a 1072x1448
     * portrait device.  The stock PxP path has already rotated the update
     * contents into panel order by the time EPDC_UPD_ADDR is programmed.
     * Present that panel order clockwise to match the physical device.
     * Portrait-native panels need no additional presentation transform.
     */
    rotate_cw = panel_width > panel_height;
    display_width = rotate_cw ? panel_height : panel_width;
    display_height = rotate_cw ? panel_width : panel_height;
    surface = imx_epdc_prepare_surface(s, display_width, display_height);

    if (rotate_cw) {
        display_left = panel_height - top - height;
        display_top = left;
        display_update_width = height;
        display_update_height = width;
        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++) {
                unsigned dest_x = panel_height - 1 - (top + y);
                unsigned dest_y = left + x;
                uint8_t gray = update[(size_t)y * source_stride + x];

                pixels = (uint32_t *)((uint8_t *)surface_data(surface) +
                                      (size_t)dest_y *
                                      surface_stride(surface));
                pixels[dest_x] = 0xff000000U | gray * 0x00010101U;
            }
        }
    } else {
        display_left = left;
        display_top = top;
        display_update_width = width;
        display_update_height = height;
        for (y = 0; y < height; y++) {
            pixels = (uint32_t *)((uint8_t *)surface_data(surface) +
                                  (size_t)(top + y) *
                                  surface_stride(surface));
            for (x = 0; x < width; x++) {
                uint8_t gray = update[(size_t)y * source_stride + x];

                pixels[left + x] = 0xff000000U | gray * 0x00010101U;
            }
        }
    }
    dpy_gfx_update(s->console, display_left, display_top,
                   display_update_width, display_update_height);
}

static void imx_epdc_invalidate_display(void *opaque)
{
    IMXEPDCState *s = opaque;

    if (s->console) {
        dpy_gfx_update_full(s->console);
    }
}

static void imx_epdc_update_display(void *opaque)
{
    IMXEPDCState *s = opaque;

    if (imx_epdc_has_framebuffer(s)) {
        imx_epdc_render_framebuffer(s);
    } else {
        imx_epdc_invalidate_display(s);
    }
}

/*
 * The Kindle X driver renders through an mmap of /dev/fb0.  Those stores do
 * not cross the EPDC MMIO aperture, and modern KPP views can finish drawing
 * after their last explicit panel update.  Real hardware scans the backing
 * framebuffer as part of its update pipeline; keep the Cocoa scanout tied to
 * that same memory so it cannot retain an older blanket frame.
 */
static void imx_epdc_refresh_scanout(void *opaque)
{
    IMXEPDCState *s = opaque;

    imx_epdc_update_display(s);
    timer_mod(s->refresh_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
              EPDC_SCANOUT_INTERVAL_MS);
}

static const GraphicHwOps imx_epdc_gfx_ops = {
    .invalidate = imx_epdc_invalidate_display,
    .gfx_update = imx_epdc_update_display,
};

static void imx_epdc_complete(void *opaque)
{
    IMXEPDCState *s = opaque;
    uint32_t upd_ctrl;
    uint32_t lut;

    if (!s->update_pending) {
        return;
    }
    s->update_pending = false;
    upd_ctrl = *epdc_reg(s, EPDC_UPD_CTRL);
    lut = (upd_ctrl >> EPDC_UPD_LUT_SHIFT) & EPDC_UPD_LUT_MASK;

    /*
     * AUTOWV_PAUSE is a two-phase transaction.  Hardware first finishes
     * histogram processing and raises UPD_DONE while retaining both the
     * working buffer and LUT.  The driver chooses the waveform and rewrites
     * UPD_CTRL with AUTOWV_PAUSE clear; only that second write completes the
     * working buffer and LUT.  Completing everything on the first write
     * races the driver's queue bookkeeping and makes it resubmit forever.
     */
    if (upd_ctrl & (1U << 3)) {
        *epdc_reg(s, EPDC_IRQ) |= EPDC_IRQ_UPD_DONE;
        imx_epdc_update_irq(s);
        return;
    }

    /* Dry-run collision tests do not change the logical display. */
    if (!(upd_ctrl & (1U << 1))) {
        if (imx_epdc_has_framebuffer(s)) {
            imx_epdc_render_framebuffer(s);
        } else {
            imx_epdc_render_update(s);
        }
    }

    *epdc_reg(s, EPDC_STATUS) &= ~EPDC_STATUS_WB_BUSY;
    if (lut < 32) {
        *epdc_reg(s, EPDC_STATUS_LUTS) &= ~(1U << lut);
        *epdc_reg(s, EPDC_IRQ1) |= 1U << lut;
    } else {
        *epdc_reg(s, EPDC_STATUS_LUTS2) &= ~(1U << (lut - 32));
        *epdc_reg(s, EPDC_IRQ2) |= 1U << (lut - 32);
    }
    *epdc_reg(s, EPDC_IRQ) |= EPDC_IRQ_WB_CMPLT;
    qemu_log_mask(LOG_UNIMP,
                  "imx.epdc: complete lut=%u irq=%08x mask=%08x "
                  "irq1=%08x mask1=%08x\n",
                  lut, *epdc_reg(s, EPDC_IRQ),
                  *epdc_reg(s, EPDC_IRQ_MASK),
                  *epdc_reg(s, EPDC_IRQ1),
                  *epdc_reg(s, EPDC_IRQ_MASK1));
    imx_epdc_update_irq(s);
}

static uint64_t imx_epdc_read(void *opaque, hwaddr offset, unsigned size)
{
    IMXEPDCState *s = opaque;

    if (offset >= IMX_EPDC_MMIO_SIZE || (offset & 3)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: invalid read at 0x%" HWADDR_PRIx "\n",
                      TYPE_IMX_EPDC, offset);
        return 0;
    }
    return *epdc_reg(s, offset);
}

static bool imx_epdc_is_atomic(hwaddr base)
{
    switch (base) {
    case EPDC_CTRL:
    case 0x050: /* FORMAT */
    case 0x0a0: /* FIFOCTRL */
    case EPDC_IRQ_MASK1:
    case EPDC_IRQ_MASK2:
    case EPDC_IRQ1:
    case EPDC_IRQ2:
    case EPDC_IRQ_MASK:
    case EPDC_IRQ:
    case EPDC_STATUS_LUTS:
    case EPDC_STATUS_LUTS2:
    case EPDC_STATUS:
        return true;
    default:
        return false;
    }
}

static void imx_epdc_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    IMXEPDCState *s = opaque;
    hwaddr base = offset & ~0xf;
    unsigned alias = offset & 0xc;
    uint32_t *reg;
    uint32_t lut;

    if (offset >= IMX_EPDC_MMIO_SIZE || (offset & 3)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: invalid write at 0x%" HWADDR_PRIx "\n",
                      TYPE_IMX_EPDC, offset);
        return;
    }

    if (alias && imx_epdc_is_atomic(base)) {
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

    if (base == EPDC_CTRL) {
        reg = epdc_reg(s, EPDC_CTRL);
        if (*reg & EPDC_CTRL_SFTRST) {
            *reg |= EPDC_CTRL_CLKGATE;
        }
    }

    if (offset == EPDC_RES && s->console) {
        unsigned width, height;

        if (imx_epdc_panel_geometry(s, &width, &height)) {
            /* Landscape EPDC scan order is portrait on the assembled unit. */
            if (width > height) {
                unsigned tmp = width;

                width = height;
                height = tmp;
            }
            imx_epdc_prepare_surface(s, width, height);
            dpy_gfx_update_full(s->console);
        }
    }

    if (offset == EPDC_UPD_CTRL) {
        lut = ((uint32_t)value >> EPDC_UPD_LUT_SHIFT) & EPDC_UPD_LUT_MASK;
        *epdc_reg(s, EPDC_STATUS) |= EPDC_STATUS_WB_BUSY;
        if (lut < 32) {
            *epdc_reg(s, EPDC_STATUS_LUTS) |= 1U << lut;
        } else {
            *epdc_reg(s, EPDC_STATUS_LUTS2) |= 1U << (lut - 32);
        }
        s->update_pending = true;
        qemu_log_mask(LOG_UNIMP,
                      "imx.epdc: submit value=%08" PRIx64 " lut=%u "
                      "addr=%08x stride=%u cord=%08x size=%08x res=%08x "
                      "format=%08x\n",
                      value, lut, *epdc_reg(s, EPDC_UPD_ADDR),
                      *epdc_reg(s, EPDC_UPD_STRIDE),
                      *epdc_reg(s, EPDC_UPD_CORD),
                      *epdc_reg(s, EPDC_UPD_SIZE),
                      *epdc_reg(s, EPDC_RES),
                      *epdc_reg(s, EPDC_FORMAT));
        /*
         * The physical panel latency is irrelevant to the guest's queue
         * contract.  Complete synchronously so early userspace cannot park
         * the only runnable vCPU while waiting for a virtual-time bottom
         * half to deliver the level interrupt.
         */
        imx_epdc_complete(s);
    }

    if (base == EPDC_IRQ_MASK1 || base == EPDC_IRQ_MASK2 ||
        base == EPDC_IRQ1 || base == EPDC_IRQ2 ||
        base == EPDC_IRQ_MASK || base == EPDC_IRQ) {
        imx_epdc_update_irq(s);
    }
}

static const MemoryRegionOps imx_epdc_ops = {
    .read = imx_epdc_read,
    .write = imx_epdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void imx_epdc_reset(DeviceState *dev)
{
    IMXEPDCState *s = IMX_EPDC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    *epdc_reg(s, EPDC_CTRL) = EPDC_CTRL_CLKGATE;
    *epdc_reg(s, EPDC_VERSION) = EPDC_VERSION_2_1_0;
    s->update_pending = false;
    qemu_set_irq(s->irq, 0);
}

static const VMStateDescription vmstate_imx_epdc = {
    .name = TYPE_IMX_EPDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMXEPDCState,
                             IMX_EPDC_MMIO_SIZE / sizeof(uint32_t)),
        VMSTATE_BOOL(update_pending, IMXEPDCState),
        VMSTATE_END_OF_LIST()
    },
};

static void imx_epdc_realize(DeviceState *dev, Error **errp)
{
    IMXEPDCState *s = IMX_EPDC(dev);

    if ((s->fb_addr || s->fb_width || s->fb_height || s->fb_stride) &&
        !imx_epdc_has_framebuffer(s)) {
        error_setg(errp, "invalid EPDC framebuffer configuration");
        return;
    }
    s->complete_bh = qemu_bh_new(imx_epdc_complete, s);
    s->console = graphic_console_init(dev, 0, &imx_epdc_gfx_ops, s);
    s->refresh_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                    imx_epdc_refresh_scanout, s);
    timer_mod(s->refresh_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
              EPDC_SCANOUT_INTERVAL_MS);
}

static void imx_epdc_unrealize(DeviceState *dev)
{
    IMXEPDCState *s = IMX_EPDC(dev);

    qemu_bh_delete(s->complete_bh);
    timer_free(s->refresh_timer);
    g_free(s->fb_buffer);
    s->fb_buffer = NULL;
}

static const Property imx_epdc_properties[] = {
    DEFINE_PROP_LINK("pxp", IMXEPDCState, pxp, TYPE_IMX6SL_PXP,
                     IMX6SLPXPState *),
    DEFINE_PROP_UINT64("fb-addr", IMXEPDCState, fb_addr, 0),
    DEFINE_PROP_UINT32("fb-width", IMXEPDCState, fb_width, 0),
    DEFINE_PROP_UINT32("fb-height", IMXEPDCState, fb_height, 0),
    DEFINE_PROP_UINT32("fb-stride", IMXEPDCState, fb_stride, 0),
    DEFINE_PROP_UINT8("fb-bpp", IMXEPDCState, fb_bpp, 1),
};

static void imx_epdc_init(Object *obj)
{
    IMXEPDCState *s = IMX_EPDC(obj);

    memory_region_init_io(&s->iomem, obj, &imx_epdc_ops, s,
                          TYPE_IMX_EPDC, IMX_EPDC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void imx_epdc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = imx_epdc_realize;
    dc->unrealize = imx_epdc_unrealize;
    device_class_set_legacy_reset(dc, imx_epdc_reset);
    dc->vmsd = &vmstate_imx_epdc;
    dc->desc = "i.MX electrophoretic display controller";
    device_class_set_props(dc, imx_epdc_properties);
}

static const TypeInfo imx_epdc_info = {
    .name = TYPE_IMX_EPDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXEPDCState),
    .instance_init = imx_epdc_init,
    .class_init = imx_epdc_class_init,
};

static void imx_epdc_register_types(void)
{
    type_register_static(&imx_epdc_info);
}

type_init(imx_epdc_register_types)
