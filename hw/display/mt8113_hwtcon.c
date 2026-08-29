/*
 * MediaTek MT8113 hardware timing controller
 *
 * Bellatrix4 uses MediaTek's HWTCON v2 display pipeline rather than the
 * i.MX EPDC block used by earlier Kindles.  Keep the register model separate
 * even while only the bootloader-visible register storage is implemented.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/display/mt8113_hwtcon.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "exec/cpu-common.h"
#include "exec/target_page.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/cpus.h"
#include "system/hw_accel.h"
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

#define MDP_RDMA_EN              0x7000
#define MDP_RDMA_SRC_CON         0x7030
#define MDP_RDMA_SRC_PITCH       0x7060
#define MDP_RDMA_SRC_SIZE        0x7070
#define MDP_RDMA_SRC_END         0x7100
#define MDP_RDMA_SRC_OFFSET      0x7118
#define MDP_RDMA_SRC_BASE        0x7f00
#define MDP_RDMA_FORMAT_MASK     0xf
#define MDP_RDMA_FORMAT_RGBA8888 0x2
#define MDP_RDMA_FORMAT_Y8       0x7

/* The MM display bank also contains the MT8512 SMI common block. */
#define SMI_COMMON_OFFSET        0x2000
#define SMI_DEBUG_MISC           0x0440
#define SMI_DEBUG_MISC_IDLE      BIT(0)

#define HWTCON_LUT_COMPLETE_NS   (10 * SCALE_MS)
#define HWTCON_SCANOUT_INTERVAL_MS 50
#define HWTCON_PANEL_WIDTH       1272
#define HWTCON_PANEL_HEIGHT      1696
#define MT8113_IOMMU_PAGE_SIZE   0x1000
#define MT8113_IOMMU_IOVA_LIMIT  0x10000000

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

    if (!s->console || !s->iommu || !s->scanout_dirty ||
        (!s->fb_iova && !s->fb_guest_va) || !s->fb_width ||
        !s->fb_height || !s->fb_source_width || !s->fb_source_height ||
        s->fb_width > 4096 || s->fb_height > 4096 ||
        s->fb_source_width > 4096 || s->fb_source_height > 4096) {
        return;
    }
    bytes_per_pixel = s->fb_format == MDP_RDMA_FORMAT_RGBA8888 ? 4 : 1;
    if (s->fb_pitch < s->fb_source_width * bytes_per_pixel) {
        return;
    }
    buffer_size = (size_t)s->fb_pitch * s->fb_source_height;
    s->fb_buffer = g_realloc(s->fb_buffer, buffer_size);
    if (!s->fb_buffer_ready) {
        if (!mt8113_iommu_dma_read(s->iommu, s->fb_iova,
                                   s->fb_buffer, buffer_size)) {
            s->scanout_read_failures++;
            s->scanout_dirty = false;
            return;
        }
        s->scanout_captures++;
    }

    surface = mt8113_hwtcon_prepare_surface(s, s->fb_width, s->fb_height);
    for (unsigned y = 0; y < s->fb_height; y++) {
        uint32_t *pixels = (uint32_t *)((uint8_t *)surface_data(surface) +
                                        (size_t)y * surface_stride(surface));

        for (unsigned x = 0; x < s->fb_width; x++) {
            unsigned source_x;
            unsigned source_y;
            const uint8_t *source;
            uint8_t red, green, blue;

            switch (s->fb_rotation) {
            case 1:
                source_x = y;
                source_y = s->fb_source_height - 1 - x;
                break;
            case 2:
                source_x = s->fb_source_width - 1 - x;
                source_y = s->fb_source_height - 1 - y;
                break;
            case 3:
                source_x = s->fb_source_width - 1 - y;
                source_y = x;
                break;
            default:
                source_x = x;
                source_y = y;
                break;
            }
            source = s->fb_buffer + (size_t)source_y * s->fb_pitch +
                     (size_t)source_x * bytes_per_pixel;
            if (bytes_per_pixel == 4) {
                red = source[0];
                green = source[1];
                blue = source[2];
            } else {
                red = green = blue = source[0];
            }

            pixels[x] = 0xff000000U | red << 16 | green << 8 | blue;
        }
    }
    s->scanout_dirty = false;
    s->fb_buffer_ready = false;
    s->scanout_refreshes++;
    dpy_gfx_update_full(s->console);
}

static void mt8113_hwtcon_invalidate_display(void *opaque)
{
    MT8113HWTCONState *s = opaque;

    if (s->console) {
        dpy_gfx_update_full(s->console);
    }
}

static void mt8113_hwtcon_update_display(void *opaque)
{
    MT8113HWTCONState *s = opaque;

    mt8113_hwtcon_render_framebuffer(s);
}

static bool mt8113_hwtcon_iova_matches(MT8113HWTCONState *s,
                                        hwaddr iova, hwaddr physical,
                                        hwaddr next_physical)
{
    hwaddr candidate;

    if (!mt8113_iommu_translate(s->iommu, iova, &candidate) ||
        (candidate & -MT8113_IOMMU_PAGE_SIZE) != physical) {
        return false;
    }
    if (next_physical != (hwaddr)-1 &&
        (!mt8113_iommu_translate(s->iommu,
                                 iova + MT8113_IOMMU_PAGE_SIZE,
                                 &candidate) ||
         (candidate & -MT8113_IOMMU_PAGE_SIZE) != next_physical)) {
        return false;
    }
    return true;
}

/*
 * The CFA module receives the CPU's vmalloc view of the fbdev allocation,
 * while HWTCON consumes the same scatterlist through a contiguous IOVA.  Find
 * that IOVA by matching the first two backing pages.  This follows the live
 * IOMMU tables and therefore also handles fbdev's unaligned alternate page;
 * no framebuffer address or allocation-order pattern is assumed.
 */
static bool mt8113_hwtcon_find_cfa_iova(MT8113HWTCONState *s, CPUState *cpu,
                                         vaddr source, hwaddr *result,
                                         hwaddr *first_physical)
{
    MemTxAttrs attrs;
    vaddr page = source & -MT8113_IOMMU_PAGE_SIZE;
    hwaddr physical = cpu_get_phys_page_attrs_debug(cpu, page, &attrs);
    hwaddr next_physical = cpu_get_phys_page_attrs_debug(
        cpu, page + MT8113_IOMMU_PAGE_SIZE, &attrs);
    hwaddr page_offset = source & (MT8113_IOMMU_PAGE_SIZE - 1);
    hwaddr preferred;

    if (physical == (hwaddr)-1) {
        return false;
    }
    physical &= -MT8113_IOMMU_PAGE_SIZE;
    if (next_physical != (hwaddr)-1) {
        next_physical &= -MT8113_IOMMU_PAGE_SIZE;
    }

    if (s->fb_iova && s->fb_guest_va) {
        preferred = s->fb_iova + (uint32_t)(source - s->fb_guest_va);
        if (preferred < MT8113_IOMMU_IOVA_LIMIT &&
            mt8113_hwtcon_iova_matches(
                s, preferred & -MT8113_IOMMU_PAGE_SIZE,
                physical, next_physical)) {
            *result = preferred;
            *first_physical = physical + page_offset;
            return true;
        }
    }

    for (hwaddr iova = 0; iova < MT8113_IOMMU_IOVA_LIMIT;
         iova += MT8113_IOMMU_PAGE_SIZE) {
        if (mt8113_hwtcon_iova_matches(s, iova, physical, next_physical)) {
            *result = iova + page_offset;
            *first_physical = physical + page_offset;
            return true;
        }
    }
    return false;
}

static void mt8113_hwtcon_capture_cfa_source(MT8113HWTCONState *s)
{
    CPUState *cpu = current_cpu;
    uint32_t source = s->cfa_mailbox_regs[0];
    uint32_t source_width = s->cfa_mailbox_regs[1];
    uint32_t source_height = s->cfa_mailbox_regs[2];
    uint32_t left = s->cfa_mailbox_regs[3];
    uint32_t top = s->cfa_mailbox_regs[4];
    uint32_t update_width = s->cfa_mailbox_regs[5];
    uint32_t update_height = s->cfa_mailbox_regs[6];
    uint32_t rotation = s->cfa_mailbox_regs[7];
    uint32_t panel;
    uint32_t panel_width;
    uint32_t panel_height;
    hwaddr source_iova;
    hwaddr first_physical = -1;
    size_t pitch;

    if (!source) {
        return;
    }

    panel = *mt8113_hwtcon_reg(s, PAPER_TCTOP_PANEL_SIZE);
    panel_width = panel & 0x3fff;
    panel_height = (panel >> 16) & 0x3fff;
    if (!panel_width || !panel_height || !source_width || !source_height ||
        !update_width || !update_height ||
        (uint64_t)left + update_width > source_width ||
        (uint64_t)top + update_height > source_height ||
        rotation > 3 ||
        ((rotation & 1) &&
         (source_width != panel_height || source_height != panel_width)) ||
        (!(rotation & 1) &&
         (source_width != panel_width || source_height != panel_height))) {
        return;
    }

    s->cfa_source_reports++;
    trace_mt8113_hwtcon_cfa_source(source, source_width, source_height,
                                    rotation, panel_width, panel_height);

    if (!cpu) {
        s->scanout_read_failures++;
        return;
    }
    cpu_synchronize_state(cpu);
    if (!mt8113_hwtcon_find_cfa_iova(s, cpu, source, &source_iova,
                                      &first_physical)) {
        s->cfa_fault_va = source;
        s->cfa_fault_cpu = cpu->cpu_index;
        s->fb_buffer_ready = false;
        s->scanout_dirty = false;
        s->scanout_read_failures++;
        trace_mt8113_hwtcon_cfa_fault(source, source, cpu->cpu_index, 0);
        trace_mt8113_hwtcon_cfa_capture(source, 0, first_physical, 0,
                                         false);
        return;
    }

    pitch = (size_t)source_width * 4;
    trace_mt8113_hwtcon_cfa_capture(source, source_iova, first_physical,
                                     0, true);

    s->fb_iova = source_iova;
    s->fb_guest_va = source;
    s->fb_width = panel_width;
    s->fb_height = panel_height;
    s->fb_source_width = source_width;
    s->fb_source_height = source_height;
    s->fb_pitch = pitch;
    s->fb_format = MDP_RDMA_FORMAT_RGBA8888;
    s->fb_rotation = rotation;
    s->cfa_fault_va = 0;
    s->cfa_fault_cpu = 0;
    s->fb_buffer_ready = false;
    s->cfa_active = true;
    s->scanout_dirty = true;
}

static uint64_t mt8113_hwtcon_cfa_mailbox_read(void *opaque, hwaddr offset,
                                                unsigned size)
{
    MT8113HWTCONState *s = opaque;

    return s->cfa_mailbox_regs[offset / sizeof(uint32_t)];
}

static void mt8113_hwtcon_cfa_mailbox_write(void *opaque, hwaddr offset,
                                             uint64_t value, unsigned size)
{
    MT8113HWTCONState *s = opaque;

    if (offset == 0) {
        memset(s->cfa_mailbox_regs, 0, sizeof(s->cfa_mailbox_regs));
    }
    s->cfa_mailbox_regs[offset / sizeof(uint32_t)] = value;
    if (offset == MT8113_HWTCON_CFA_MAILBOX_SIZE - sizeof(uint32_t)) {
        mt8113_hwtcon_capture_cfa_source(s);
    }
}

static const MemoryRegionOps mt8113_hwtcon_cfa_mailbox_ops = {
    .read = mt8113_hwtcon_cfa_mailbox_read,
    .write = mt8113_hwtcon_cfa_mailbox_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

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
    .gfx_update = mt8113_hwtcon_update_display,
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
        if (offset == MDP_RDMA_EN && (value & BIT(0))) {
            uint32_t size = bank->regs[MDP_RDMA_SRC_SIZE / 4];
            uint32_t width = size & 0x3fff;
            uint32_t height = (size >> 16) & 0x3fff;
            uint32_t panel = *mt8113_hwtcon_reg(s,
                                                 PAPER_TCTOP_PANEL_SIZE);
            uint32_t panel_width = panel & 0x3fff;
            uint32_t panel_height = (panel >> 16) & 0x3fff;
            uint32_t format = bank->regs[MDP_RDMA_SRC_CON / 4] &
                              MDP_RDMA_FORMAT_MASK;
            uint32_t pitch = bank->regs[MDP_RDMA_SRC_PITCH / 4] &
                             0x1fffff;
            uint32_t source = bank->regs[MDP_RDMA_SRC_BASE / 4];
            uint32_t source_end = bank->regs[MDP_RDMA_SRC_END / 4];
            unsigned bytes_per_pixel =
                format == MDP_RDMA_FORMAT_RGBA8888 ? 4 : 1;
            bool transaction_valid = width && height && panel_width &&
                                     panel_height && width <= panel_width &&
                                     height <= panel_height &&
                                     (format == MDP_RDMA_FORMAT_RGBA8888 ||
                                      format == MDP_RDMA_FORMAT_Y8);
            bool full_source = false;
            bool refresh_existing = false;
            bool queued = false;

            /*
             * RDMA folds the crop coordinates into SRC_BASE for partial
             * updates.  Treating that tile address as a framebuffer origin
             * shifts every following row and wraps the cropped pixels onto
             * the opposite edge.  Only a panel-sized transaction can define
             * a new plane.  Partial transactions merely tell us to refresh
             * the stable full-frame source learned from a full transaction or
             * from the CFA input hook.
             */
            if (transaction_valid &&
                width == panel_width && height == panel_height &&
                pitch >= panel_width * bytes_per_pixel) {
                full_source = true;
            }

            if (full_source) {
                s->fb_iova = source;
                s->fb_guest_va = 0;
                s->fb_width = panel_width;
                s->fb_height = panel_height;
                s->fb_source_width = panel_width;
                s->fb_source_height = panel_height;
                s->fb_pitch = pitch;
                s->fb_format = format;
                s->fb_rotation = 0;
                s->cfa_active = false;
            } else if (transaction_valid && s->fb_iova &&
                       s->fb_width == panel_width &&
                       s->fb_height == panel_height &&
                       s->fb_source_width == panel_width &&
                       s->fb_source_height == panel_height &&
                       s->fb_pitch == pitch && s->fb_format == format) {
                refresh_existing = true;
            }

            if (full_source || refresh_existing) {
                s->fb_buffer_ready = false;
                s->scanout_dirty = true;
                queued = true;
            }

            s->mdp_transactions++;
            trace_mt8113_hwtcon_mdp_source(
                source, source_end, s->fb_iova, format, width,
                height, pitch, panel_width, panel_height, queued);
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
    /*
     * mtk-smi treats a clear SMI_DEBUG_MISC idle bit as an outstanding bus
     * transaction and runs its expensive hang-dump notifier on every runtime
     * suspend.  No emulated display engine leaves an SMI request pending, so
     * expose the hardware's idle state instead of the zero-filled reset value.
     */
    s->bank[0].regs[(SMI_COMMON_OFFSET + SMI_DEBUG_MISC) / 4] =
        SMI_DEBUG_MISC_IDLE;
    s->pending_luts = 0;
    s->fb_iova = 0;
    s->fb_guest_va = 0;
    s->fb_width = 0;
    s->fb_height = 0;
    s->fb_source_width = 0;
    s->fb_source_height = 0;
    s->fb_pitch = 0;
    s->fb_format = 0;
    s->fb_rotation = 0;
    s->fb_buffer_ready = false;
    s->cfa_active = false;
    s->scanout_dirty = false;
    memset(s->cfa_mailbox_regs, 0, sizeof(s->cfa_mailbox_regs));
    s->mdp_transactions = 0;
    s->cfa_source_reports = 0;
    s->cfa_fault_va = 0;
    s->cfa_fault_cpu = 0;
    s->scanout_captures = 0;
    s->scanout_refreshes = 0;
    s->scanout_read_failures = 0;
    timer_del(s->lut_timer);
    for (unsigned i = 0; i < MT8113_HWTCON_NUM_IRQS; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
}

static void mt8113_hwtcon_init(Object *obj)
{
    MT8113HWTCONState *s = MT8113_HWTCON(obj);

    object_property_add_uint32_ptr(obj, "scanout-iova", &s->fb_iova,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "scanout-guest-va",
                                   &s->fb_guest_va,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "scanout-width", &s->fb_width,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "scanout-height", &s->fb_height,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "scanout-source-width",
                                   &s->fb_source_width,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "scanout-source-height",
                                   &s->fb_source_height,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "scanout-pitch", &s->fb_pitch,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "scanout-format", &s->fb_format,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "scanout-rotation",
                                   &s->fb_rotation,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "mdp-transactions",
                                   &s->mdp_transactions,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "cfa-source-reports",
                                   &s->cfa_source_reports,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "cfa-fault-va", &s->cfa_fault_va,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "cfa-fault-cpu", &s->cfa_fault_cpu,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "scanout-captures",
                                   &s->scanout_captures,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "scanout-refreshes",
                                   &s->scanout_refreshes,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "scanout-read-failures",
                                   &s->scanout_read_failures,
                                   OBJ_PROP_FLAG_READ);

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
    memory_region_init_io(&s->cfa_mailbox, obj,
                          &mt8113_hwtcon_cfa_mailbox_ops, s,
                          TYPE_MT8113_HWTCON ".cfa-mailbox",
                          MT8113_HWTCON_CFA_MAILBOX_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->cfa_mailbox);
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
