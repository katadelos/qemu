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
#include "qemu/units.h"
#include "system/cpus.h"
#include "system/hw_accel.h"
#include "ui/console.h"
#include "trace.h"

#define WF_LUT_INTEN             0x4004
#define WF_LUT_INTSTA            0x4008
#define WF_LUT_EN                0x400c
#define WF_LUT_SHADOW_UP         0x4204
#define WF_LUT_EN_0              0x4d04
#define WF_LUT_EN_1              0x4d08
#define WF_LUT_CON               0x4d0c
#define WF_LUT_DUAL_MODE         0x4d00
#define WF_LUT_END_IRQ_CLR0      0x4d50
#define WF_LUT_END_IRQ_CLR1      0x4d54
#define WF_LUT_END_IRQ_STA0      0x4d58
#define WF_LUT_END_IRQ_STA1      0x4d5c
#define WF_LUT_INFO_ID_CFG       0x4dc8
#define WF_LUT_EN_STA0           0x4270
#define WF_LUT_EN_STA1           0x4274

#define WB_WDMA_INTEN            0xa000
#define WB_WDMA_INTSTA           0xa004

#define PAPER_TCTOP_UPD_CFG0     0xd020
#define PAPER_TCTOP_UPD_CFG1     0xd024
#define PAPER_TCTOP_UPD_CFG2     0xd028
#define PAPER_TCTOP_UPD_CFG3     0xd02c
#define PAPER_TCTOP_VOID_LUT     0xd034
#define PAPER_TCTOP_PANEL_SIZE   0xd00c
#define PAPER_TCTOP_WB_ST_ADDR0  0xdf00
#define PAPER_TCTOP_WB_ST_ADDR1  0xdf04
#define PAPER_TCTOP_IMG_ST_ADDR  0xdf08
#define PAPER_TCTOP_PIPELINE_FLAG 0xdf0c
#define PAPER_TCTOP_PIPELINE_CLEAR BIT(15)

#define MDP_RDMA_EN              0x7000
#define MDP_RDMA_SRC_CON         0x7030
#define MDP_RDMA_Y4_MODE_CFG     0x7038
#define MDP_RDMA_SRC_PITCH       0x7060
#define MDP_RDMA_SRC_SIZE        0x7070
#define MDP_RDMA_SRC_END         0x7100
#define MDP_RDMA_SRC_OFFSET      0x7118
#define MDP_RDMA_SRC_BASE        0x7f00
#define MDP_RDMA_FORMAT_MASK     0xf
#define MDP_RDMA_FORMAT_RGBA8888 0x2
#define MDP_RDMA_FORMAT_Y8       0x7
#define MDP_MUTEX0_EN            0x1020
#define MDP_MUTEX6_EN            0x10e0
#define MDP_WROT_CTRL            0xa000
#define MDP_WROT_MAIN_BUF_SIZE   0xa008
#define MDP_WROT_INT_EN          0xa018
#define MDP_WROT_INT_STATUS      0xa01c
#define MDP_WROT_CROP_OFFSET     0xa020
#define MDP_WROT_TARGET_SIZE     0xa024
#define MDP_WROT_OFFSET_ADDR     0xa02c
#define MDP_WROT_STRIDE          0xa030
#define MDP_WROT_INPUT_SIZE      0xa078
#define MDP_WROT_ROT_EN          0xa07c
#define MDP_WROT_Y_MODE          0xa088
#define MDP_WROT_BASE            0xaf00
#define MDP_WROT_ROTATION_SHIFT  20
#define MDP_WROT_ROTATION_MASK   0x3
#define MDP_WROT_Y8              0
#define MDP_WROT_Y4_M0           1
#define MDP_MAX_DMA_BYTES        (64 * MiB)

/* The MM display bank also contains the MT8512 SMI common block. */
#define SMI_COMMON_OFFSET        0x2000
#define SMI_DEBUG_MISC           0x0440
#define SMI_DEBUG_MISC_IDLE      BIT(0)

#define HWTCON_LUT_COMPLETE_NS   (10 * SCALE_MS)
#define HWTCON_SCANOUT_DEBOUNCE_MS 50
#define HWTCON_BOOT_HANDOFF_MS      60000
#define HWTCON_PANEL_WIDTH       1272
#define HWTCON_PANEL_HEIGHT      1696
#define MT8113_IOMMU_PAGE_SIZE   0x1000
#define MT8113_IOMMU_IOVA_LIMIT  0x10000000

static void mt8113_hwtcon_schedule_scanout(MT8113HWTCONState *s)
{
    if (s->refresh_timer) {
        /* A single fbdev update is tiled into several MDP transactions.  Wait
         * for the burst to settle before presenting it as one e-ink frame. */
        timer_mod(s->refresh_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                  HWTCON_SCANOUT_DEBOUNCE_MS);
    }
}

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

static void mt8113_hwtcon_update_mdp_irq(MT8113HWTCONState *s)
{
    uint32_t status = s->bank[1].regs[MDP_WROT_INT_STATUS / 4];
    uint32_t enable = s->bank[1].regs[MDP_WROT_INT_EN / 4];

    qemu_set_irq(s->mdp_wrot_irq, (status & enable) != 0);
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

static bool mt8113_hwtcon_capture_pending_frame(MT8113HWTCONState *s)
{
    size_t buffer_size;
    unsigned bytes_per_pixel;
    uint8_t frame_min = UINT8_MAX;
    uint8_t frame_max = 0;

    if (!s->iommu || !s->fb_iova || !s->fb_width || !s->fb_height ||
        !s->fb_source_width || !s->fb_source_height ||
        s->fb_width > 4096 || s->fb_height > 4096 ||
        s->fb_source_width > 4096 || s->fb_source_height > 4096) {
        return false;
    }
    bytes_per_pixel = s->fb_format == MDP_RDMA_FORMAT_RGBA8888 ? 4 : 1;
    if (s->fb_pitch < s->fb_source_width * bytes_per_pixel) {
        return false;
    }
    buffer_size = (size_t)s->fb_pitch * s->fb_source_height;
    s->pending_fb_buffer = g_realloc(s->pending_fb_buffer, buffer_size);
    if (!mt8113_iommu_dma_read(s->iommu, s->fb_iova,
                               s->pending_fb_buffer, buffer_size)) {
        s->pending_fb_buffer_ready = false;
        s->scanout_read_failures++;
        return false;
    }
    s->pending_fb_buffer_ready = true;
    s->scanout_captures++;
    if (s->last_update_width && s->last_update_height &&
        (uint64_t)s->last_update_x + s->last_update_width <=
            s->fb_source_width &&
        (uint64_t)s->last_update_y + s->last_update_height <=
            s->fb_source_height) {
        for (uint32_t y = 0; y < s->last_update_height; y++) {
            size_t offset = (size_t)(s->last_update_y + y) * s->fb_pitch +
                            (size_t)s->last_update_x * bytes_per_pixel;

            for (uint32_t x = 0; x < s->last_update_width; x++) {
                uint8_t value = s->pending_fb_buffer[
                    offset + (size_t)x * bytes_per_pixel];

                frame_min = MIN(frame_min, value);
                frame_max = MAX(frame_max, value);
            }
        }
        s->last_frame_min = frame_min;
        s->last_frame_max = frame_max;
    }
    return true;
}

static bool mt8113_hwtcon_update_working_image(MT8113HWTCONState *s,
                                                bool report_void,
                                                bool commit)
{
    size_t buffer_size;
    bool update_void = true;

    if (!s->pending_fb_buffer_ready || s->fb_format != MDP_RDMA_FORMAT_Y8 ||
        !s->last_update_width || !s->last_update_height ||
        (uint64_t)s->last_update_x + s->last_update_width >
            s->fb_source_width ||
        (uint64_t)s->last_update_y + s->last_update_height >
            s->fb_source_height) {
        return false;
    }

    buffer_size = (size_t)s->fb_pitch * s->fb_source_height;
    if (s->working_image_buffer_size != buffer_size) {
        s->working_image_buffer = g_realloc(s->working_image_buffer,
                                            buffer_size);
        memset(s->working_image_buffer, 0, buffer_size);
        s->working_image_buffer_size = buffer_size;
    }

    /*
     * The HWTCON working buffer stores the current five-bit gray value as
     * well as waveform LUT ownership.  The guest initializes a new working
     * buffer to black while the physical e-ink panel keeps the bootloader's
     * image.  Compare the Y5 image input against that independent state so a
     * zero-valued Linux initialization pass is reported as VOID and does not
     * erase the retained splash.
     */
    for (uint32_t y = 0; y < s->last_update_height && update_void; y++) {
        size_t offset = (size_t)(s->last_update_y + y) * s->fb_pitch +
                        s->last_update_x;

        for (uint32_t x = 0; x < s->last_update_width; x++) {
            if ((s->pending_fb_buffer[offset + x] & 0xf8) !=
                s->working_image_buffer[offset + x]) {
                update_void = false;
                break;
            }
        }
    }

    if (update_void && report_void) {
        s->pending_fb_buffer_ready = false;
        s->pipeline_voids++;
        return true;
    }

    if (commit) {
        for (uint32_t y = 0; y < s->last_update_height; y++) {
            size_t offset = (size_t)(s->last_update_y + y) * s->fb_pitch +
                            s->last_update_x;

            for (uint32_t x = 0; x < s->last_update_width; x++) {
                s->working_image_buffer[offset + x] =
                    s->pending_fb_buffer[offset + x] & 0xf8;
            }
        }
    }
    return false;
}

static bool mt8113_hwtcon_dual_pipeline_second_pass(
    MT8113HWTCONState *s, uint32_t img_addr, uint32_t wb_addr0,
    uint32_t wb_addr1, uint32_t position, uint32_t extent,
    uint32_t pipeline_flag, uint32_t lut)
{
    uint64_t plane_size = (uint64_t)s->fb_source_width *
                          s->fb_source_height;
    uint32_t expected_lut = (s->dual_pipeline_lut + 1) & 0x3f;

    if ((pipeline_flag & PAPER_TCTOP_PIPELINE_CLEAR) &&
        s->dual_pipeline_lut == 0x3f) {
        expected_lut = 0x3f;
    }
    return s->dual_pipeline_pending && plane_size <= UINT32_MAX &&
        img_addr == s->dual_pipeline_img_addr &&
        (uint64_t)s->dual_pipeline_wb_addr0 + plane_size == wb_addr0 &&
        (uint64_t)s->dual_pipeline_wb_addr1 + plane_size == wb_addr1 &&
        position == s->dual_pipeline_position &&
        extent == s->dual_pipeline_extent &&
        pipeline_flag == s->dual_pipeline_flags && lut == expected_lut;
}

static void mt8113_hwtcon_remember_dual_pipeline_first_pass(
    MT8113HWTCONState *s, uint32_t img_addr, uint32_t wb_addr0,
    uint32_t wb_addr1, uint32_t position, uint32_t extent,
    uint32_t pipeline_flag, uint32_t lut)
{
    s->dual_pipeline_pending = true;
    s->dual_pipeline_img_addr = img_addr;
    s->dual_pipeline_wb_addr0 = wb_addr0;
    s->dual_pipeline_wb_addr1 = wb_addr1;
    s->dual_pipeline_position = position;
    s->dual_pipeline_extent = extent;
    s->dual_pipeline_flags = pipeline_flag;
    s->dual_pipeline_lut = lut;
}

static bool mt8113_hwtcon_retain_boot_blank(MT8113HWTCONState *s,
                                             bool full_panel_update,
                                             bool nonuniform_frame,
                                             bool uniform_blank)
{
    int64_t now_ms = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);

    if (s->retain_boot_splash && !s->boot_handoff_active &&
        !s->boot_handoff_arms && full_panel_update && nonuniform_frame) {
        s->boot_handoff_active = true;
        s->boot_handoff_deadline_ms = now_ms + HWTCON_BOOT_HANDOFF_MS;
        s->boot_handoff_arms++;
    }
    if (s->boot_handoff_active && now_ms > s->boot_handoff_deadline_ms) {
        s->boot_handoff_active = false;
    }
    if (s->boot_handoff_active && full_panel_update && uniform_blank &&
        (s->last_pipeline_flags & BIT(0))) {
        s->boot_handoff_active = false;
        s->boot_blank_retentions++;
        return true;
    }
    if (s->boot_handoff_active && !full_panel_update) {
        s->boot_handoff_active = false;
    }
    return false;
}

static bool mt8113_hwtcon_publish_pending_frame(MT8113HWTCONState *s)
{
    size_t buffer_size;
    unsigned bytes_per_pixel;
    bool full_panel_update;
    bool nonuniform_frame;
    bool uniform_white;

    if (!s->pending_fb_buffer_ready || !s->last_update_width ||
        !s->last_update_height ||
        (uint64_t)s->last_update_x + s->last_update_width >
            s->fb_source_width ||
        (uint64_t)s->last_update_y + s->last_update_height >
            s->fb_source_height) {
        return false;
    }
    full_panel_update = s->last_update_x == 0 && s->last_update_y == 0 &&
        s->last_update_width == s->fb_source_width &&
        s->last_update_height == s->fb_source_height;
    nonuniform_frame = s->last_frame_min != s->last_frame_max;
    uniform_white = s->fb_format == MDP_RDMA_FORMAT_Y8 &&
        s->last_frame_min == s->last_frame_max &&
        (s->last_frame_min & 0xf8) == 0xf0;
    if (mt8113_hwtcon_retain_boot_blank(s, full_panel_update,
                                         nonuniform_frame, uniform_white)) {
        /*
         * MT8110's Linux display stack submits a full white staging image
         * after U-Boot has painted the physical e-ink panel and before the
         * loading artwork.  The controller's logical working buffer must
         * accept that update, but presenting it as an LCD frame erases the
         * retained splash for the whole userspace startup.
         */
        s->pending_fb_buffer_ready = false;
        return false;
    }
    bytes_per_pixel = s->fb_format == MDP_RDMA_FORMAT_RGBA8888 ? 4 : 1;
    buffer_size = (size_t)s->fb_pitch * s->fb_source_height;
    s->fb_buffer = g_realloc(s->fb_buffer, buffer_size);
    if (!s->fb_buffer_ready) {
        memset(s->fb_buffer, 0xff, buffer_size);
    }
    for (uint32_t y = 0; y < s->last_update_height; y++) {
        size_t offset = (size_t)(s->last_update_y + y) * s->fb_pitch +
                        (size_t)s->last_update_x * bytes_per_pixel;

        memcpy(s->fb_buffer + offset, s->pending_fb_buffer + offset,
               (size_t)s->last_update_width * bytes_per_pixel);
    }
    s->pending_fb_buffer_ready = false;
    s->fb_buffer_ready = true;
    return true;
}

static void mt8113_hwtcon_render_framebuffer(MT8113HWTCONState *s)
{
    DisplaySurface *surface;
    size_t buffer_size;
    unsigned bytes_per_pixel;
    bool full_panel_update;
    uint8_t frame_min = UINT8_MAX;
    uint8_t frame_max = 0;

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

    full_panel_update = s->last_update_x == 0 && s->last_update_y == 0 &&
        s->last_update_width == s->fb_source_width &&
        s->last_update_height == s->fb_source_height;
    if (s->retain_boot_splash &&
        (!s->boot_handoff_arms || s->boot_handoff_active) &&
        !s->image_scanout_active &&
        s->last_update_width && s->last_update_height &&
        (uint64_t)s->last_update_x + s->last_update_width <=
            s->fb_source_width &&
        (uint64_t)s->last_update_y + s->last_update_height <=
            s->fb_source_height) {
        for (uint32_t y = 0; y < s->last_update_height; y++) {
            size_t offset = (size_t)(s->last_update_y + y) * s->fb_pitch +
                            (size_t)s->last_update_x * bytes_per_pixel;

            for (uint32_t x = 0; x < s->last_update_width; x++) {
                const uint8_t *pixel = s->fb_buffer + offset +
                                       (size_t)x * bytes_per_pixel;
                unsigned channels = bytes_per_pixel == 4 ? 3 : 1;

                for (unsigned channel = 0; channel < channels; channel++) {
                    frame_min = MIN(frame_min, pixel[channel]);
                    frame_max = MAX(frame_max, pixel[channel]);
                }
            }
        }
        s->last_frame_min = frame_min;
        s->last_frame_max = frame_max;
        if (mt8113_hwtcon_retain_boot_blank(
                s, full_panel_update, frame_min != frame_max,
                frame_min == frame_max &&
                (frame_min == 0 || frame_min == UINT8_MAX))) {
            s->scanout_dirty = false;
            s->fb_buffer_ready = false;
            return;
        }
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
    if (!s->image_scanout_active) {
        s->fb_buffer_ready = false;
    }
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
    bool needs_initial_scanout;

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

    needs_initial_scanout = s->scanout_refreshes == 0;
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
    if (needs_initial_scanout) {
        s->scanout_dirty = true;
        mt8113_hwtcon_schedule_scanout(s);
    }
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
    bool scanout_dirty = s->scanout_dirty;

    mt8113_hwtcon_render_framebuffer(s);
    if (!scanout_dirty && s->console) {
        /*
         * An e-ink panel retains its last image while the display pipeline is
         * reset or has no readable source.  Re-present the retained surface
         * after those hand-offs so a newly recreated Cocoa texture cannot
         * remain black until the guest submits another frame.
         */
        dpy_gfx_update_full(s->console);
    }
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
    *mt8113_hwtcon_reg(s, WF_LUT_EN_0 + half * 4) |= value;
    *mt8113_hwtcon_reg(s, WF_LUT_EN_STA0 + half * 4) |= value;
    timer_mod(s->lut_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              HWTCON_LUT_COMPLETE_NS);
}

static bool mt8113_hwtcon_use_rotated_fb_plane(MT8113HWTCONState *s,
                                                uint32_t source,
                                                uint32_t pitch,
                                                uint32_t panel_width,
                                                uint32_t panel_height,
                                                uint32_t rotation)
{
    uint64_t plane_size = (uint64_t)pitch * panel_width;
    uint64_t second_plane;
    uint32_t active;

    if (s->cfa_active || !(rotation & 1) || pitch < panel_height ||
        plane_size > UINT32_MAX) {
        return false;
    }

    if (!s->fb_base_iova ||
        (source < s->fb_base_iova ||
         source >= (uint64_t)s->fb_base_iova + 2 * plane_size)) {
        /*
         * fbdev exposes two vertically stacked portrait pages.  The DMA
         * allocation begins on an IOMMU page, while the alternate page begins
         * exactly one portrait frame later and is intentionally unaligned.
         * A full-screen MDP update starts with crop (0, 0), so its first RDMA
         * transaction identifies either page without assuming an address.  A
         * Linux plane may replace the bootloader plane already being scanned
         * out, so an aligned first tile also starts a new allocation.
         */
        if (!(source & (MT8113_IOMMU_PAGE_SIZE - 1))) {
            s->fb_base_iova = source;
        } else if (source >= plane_size &&
                   !((source - plane_size) &
                     (MT8113_IOMMU_PAGE_SIZE - 1))) {
            s->fb_base_iova = source - plane_size;
        } else {
            return false;
        }
    }

    second_plane = (uint64_t)s->fb_base_iova + plane_size;
    if (source >= s->fb_base_iova && source < second_plane) {
        active = s->fb_base_iova;
    } else if (source >= second_plane && source < second_plane + plane_size) {
        active = second_plane;
    } else {
        return false;
    }

    s->fb_iova = active;
    s->fb_guest_va = 0;
    /* Cocoa presents the fbdev/user orientation, before WROT turns it to the
     * controller's landscape panel orientation. */
    s->fb_width = panel_height;
    s->fb_height = panel_width;
    s->fb_source_width = panel_height;
    s->fb_source_height = panel_width;
    s->fb_pitch = pitch;
    s->fb_format = MDP_RDMA_FORMAT_Y8;
    s->fb_rotation = 0;
    return true;
}

static bool mt8113_hwtcon_mdp_writeback(MT8113HWTCONState *s,
                                         MT8113HWTCONBank *bank)
{
    uint32_t source_size = bank->regs[MDP_RDMA_SRC_SIZE / 4];
    uint32_t width = source_size & 0x3fff;
    uint32_t height = (source_size >> 16) & 0x3fff;
    uint32_t source_pitch = bank->regs[MDP_RDMA_SRC_PITCH / 4] & 0x1fffff;
    uint32_t source = bank->regs[MDP_RDMA_SRC_BASE / 4];
    uint32_t source_offset = bank->regs[MDP_RDMA_SRC_OFFSET / 4];
    uint32_t source_format = bank->regs[MDP_RDMA_SRC_CON / 4] &
                             MDP_RDMA_FORMAT_MASK;
    uint32_t source_y4 = bank->regs[MDP_RDMA_Y4_MODE_CFG / 4] & 0xf;
    uint32_t target_size = bank->regs[MDP_WROT_TARGET_SIZE / 4];
    uint32_t input_size = bank->regs[MDP_WROT_INPUT_SIZE / 4];
    uint32_t destination = bank->regs[MDP_WROT_BASE / 4];
    uint32_t destination_pitch = bank->regs[MDP_WROT_STRIDE / 4] & 0x3fff;
    uint32_t destination_mode = bank->regs[MDP_WROT_Y_MODE / 4] & 0xf;
    uint32_t rotation =
        (bank->regs[MDP_WROT_CTRL / 4] >> MDP_WROT_ROTATION_SHIFT) &
        MDP_WROT_ROTATION_MASK;
    uint32_t destination_offset =
        bank->regs[MDP_WROT_OFFSET_ADDR / 4] & 0x0fffffff;
    uint32_t output_width = rotation & 1 ? height : width;
    uint32_t output_height = rotation & 1 ? width : height;
    uint64_t source_top = (uint64_t)source + source_offset;
    uint64_t destination_top = destination;
    uint64_t source_bytes;
    uint64_t output_bytes;
    g_autofree uint8_t *source_pixels = NULL;
    g_autofree uint8_t *output_pixels = NULL;

    if (!width || !height || width > 4096 || height > 4096 ||
        source_format != MDP_RDMA_FORMAT_Y8 || source_y4 != 0 ||
        source_pitch < width || destination_pitch < output_width ||
        (destination_mode != MDP_WROT_Y8 &&
         destination_mode != MDP_WROT_Y4_M0) ||
        (target_size & 0x3fff) != width ||
        ((target_size >> 16) & 0x3fff) != height ||
        (input_size & 0x3fff) != width ||
        ((input_size >> 16) & 0x3fff) != height) {
        return false;
    }

    switch (rotation) {
    case 0:
        destination_top += destination_offset;
        break;
    case 1:
        if (destination_offset != height - 1) {
            return false;
        }
        break;
    case 2:
        destination_top -= (uint64_t)(height - 1) * destination_pitch +
                           width - 1;
        break;
    case 3:
        destination_top -= (uint64_t)(width - 1) * destination_pitch;
        break;
    default:
        break;
    }
    if (destination_top > UINT32_MAX) {
        return false;
    }

    source_bytes = (uint64_t)(height - 1) * source_pitch + width;
    output_bytes = (uint64_t)(output_height - 1) * destination_pitch +
                   output_width;
    if (source_bytes > MDP_MAX_DMA_BYTES ||
        output_bytes > MDP_MAX_DMA_BYTES ||
        source_top + source_bytes > (uint64_t)UINT32_MAX + 1 ||
        destination_top + output_bytes > (uint64_t)UINT32_MAX + 1) {
        return false;
    }

    source_pixels = g_malloc(source_bytes);
    output_pixels = g_malloc(output_bytes);
    if (!mt8113_iommu_dma_read(s->iommu, source_top, source_pixels,
                               source_bytes) ||
        !mt8113_iommu_dma_read(s->iommu, destination_top, output_pixels,
                               output_bytes)) {
        return false;
    }
    for (uint32_t y = 0; y < output_height; y++) {
        for (uint32_t x = 0; x < output_width; x++) {
            uint32_t source_x;
            uint32_t source_y;
            uint8_t pixel;

            switch (rotation) {
            case 1:
                source_x = y;
                source_y = height - 1 - x;
                break;
            case 2:
                source_x = width - 1 - x;
                source_y = height - 1 - y;
                break;
            case 3:
                source_x = width - 1 - y;
                source_y = x;
                break;
            default:
                source_x = x;
                source_y = y;
                break;
            }
            pixel = source_pixels[(size_t)source_y * source_pitch +
                                  source_x];
            output_pixels[(size_t)y * destination_pitch + x] =
                destination_mode == MDP_WROT_Y4_M0 ? pixel & 0xf0 : pixel;
        }
    }
    return mt8113_iommu_dma_write(s->iommu, destination_top, output_pixels,
                                  output_bytes);
}

static bool mt8113_hwtcon_select_image_buffer(MT8113HWTCONState *s,
                                               MT8113HWTCONBank *bank,
                                               uint32_t panel_width,
                                               uint32_t panel_height)
{
    uint32_t image = *mt8113_hwtcon_reg(s, PAPER_TCTOP_IMG_ST_ADDR);
    uint32_t size = bank->regs[MDP_RDMA_SRC_SIZE / 4];
    uint32_t width = size & 0x3fff;
    uint32_t height = (size >> 16) & 0x3fff;
    uint32_t pitch = bank->regs[MDP_WROT_STRIDE / 4] & 0x3fff;
    uint32_t destination = bank->regs[MDP_WROT_BASE / 4];
    uint32_t rotation =
        (bank->regs[MDP_WROT_CTRL / 4] >> MDP_WROT_ROTATION_SHIFT) &
        MDP_WROT_ROTATION_MASK;
    uint32_t destination_offset =
        bank->regs[MDP_WROT_OFFSET_ADDR / 4] & 0x0fffffff;
    uint32_t output_width = rotation & 1 ? height : width;
    uint32_t output_height = rotation & 1 ? width : height;
    uint64_t destination_top = destination;
    uint64_t destination_end;
    uint64_t image_end;

    if (!s->scanout_image_buffer || !image || !panel_width || !panel_height ||
        !width || !height || pitch < panel_width) {
        return false;
    }

    switch (rotation) {
    case 0:
        destination_top += destination_offset;
        break;
    case 2:
        if (destination_top <
            (uint64_t)(height - 1) * pitch + width - 1) {
            return false;
        }
        destination_top -= (uint64_t)(height - 1) * pitch + width - 1;
        break;
    case 3:
        if (destination_top < (uint64_t)(width - 1) * pitch) {
            return false;
        }
        destination_top -= (uint64_t)(width - 1) * pitch;
        break;
    default:
        break;
    }
    destination_end = destination_top +
        (uint64_t)(output_height - 1) * pitch + output_width;
    image_end = (uint64_t)image + (uint64_t)pitch * panel_height;
    if (destination_top < image || destination_end > image_end) {
        return false;
    }

    /*
     * On monochrome HWTCON, fbdev is only the MDP input.  WROT accumulates
     * converted updates in the image buffer and the pipeline consumes that
     * full panel image.  Present the same buffer, undoing WROT's orientation
     * so Cocoa remains in the fbdev/user orientation.
     */
    if (!s->image_scanout_active) {
        s->fb_base_iova = image;
        s->fb_iova = image;
        s->fb_guest_va = 0;
        s->fb_width = rotation & 1 ? panel_height : panel_width;
        s->fb_height = rotation & 1 ? panel_width : panel_height;
        s->fb_source_width = panel_width;
        s->fb_source_height = panel_height;
        s->fb_pitch = pitch;
        s->fb_format = MDP_RDMA_FORMAT_Y8;
        s->fb_rotation = (-rotation) & MDP_WROT_ROTATION_MASK;
        s->cfa_active = false;
        s->image_scanout_active = true;
    }
    return true;
}

static void mt8113_hwtcon_mdp_transaction(MT8113HWTCONState *s,
                                           MT8113HWTCONBank *bank)
{
    uint32_t size = bank->regs[MDP_RDMA_SRC_SIZE / 4];
    uint32_t width = size & 0x3fff;
    uint32_t height = (size >> 16) & 0x3fff;
    uint32_t panel = *mt8113_hwtcon_reg(s, PAPER_TCTOP_PANEL_SIZE);
    uint32_t panel_width = panel & 0x3fff;
    uint32_t panel_height = (panel >> 16) & 0x3fff;
    uint32_t format = bank->regs[MDP_RDMA_SRC_CON / 4] &
                      MDP_RDMA_FORMAT_MASK;
    uint32_t pitch = bank->regs[MDP_RDMA_SRC_PITCH / 4] & 0x1fffff;
    uint32_t source = bank->regs[MDP_RDMA_SRC_BASE / 4];
    uint32_t source_end = bank->regs[MDP_RDMA_SRC_END / 4];
    uint32_t rotation =
        (bank->regs[MDP_WROT_CTRL / 4] >> MDP_WROT_ROTATION_SHIFT) &
        MDP_WROT_ROTATION_MASK;
    unsigned bytes_per_pixel =
        format == MDP_RDMA_FORMAT_RGBA8888 ? 4 : 1;
    bool transaction_valid = width && height && panel_width && panel_height &&
                             width <= panel_width && height <= panel_height &&
                             (format == MDP_RDMA_FORMAT_RGBA8888 ||
                              format == MDP_RDMA_FORMAT_Y8);
    bool full_source = false;
    bool refresh_existing = false;
    bool image_scanout = false;
    bool source_scanout;
    bool queued = false;

    if (format != MDP_RDMA_FORMAT_Y8) {
        s->mdp_writeback_skips++;
    } else if (mt8113_hwtcon_mdp_writeback(s, bank)) {
        s->mdp_writebacks++;
        image_scanout = mt8113_hwtcon_select_image_buffer(
            s, bank, panel_width, panel_height);
    } else {
        s->mdp_writeback_failures++;
    }

    if (transaction_valid && width == panel_width &&
        height == panel_height && pitch >= panel_width * bytes_per_pixel) {
        full_source = true;
    }

    /*
     * On Colorsoft, MDP consumes the CFA conversion result as an intermediate
     * Y8 HWTCON image.  Cocoa presents the original RGBA fbdev source reported
     * by the CFA hook, so an ensuing MDP transaction must not replace it.
     * MT8110 has no CFA source report and continues to present MDP output.
     */
    source_scanout = !s->cfa_active && !s->image_scanout_active;
    if (!source_scanout) {
        full_source = false;
    }

    if (image_scanout) {
        full_source = false;
    } else if (source_scanout && full_source) {
        s->fb_base_iova = source;
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
    } else if (source_scanout && format == MDP_RDMA_FORMAT_Y8 &&
               width && height &&
               panel_width && panel_height && width <= panel_height &&
               height <= panel_width &&
               mt8113_hwtcon_use_rotated_fb_plane(
                   s, source, pitch, panel_width, panel_height, rotation)) {
        refresh_existing = true;
    } else if (source_scanout && transaction_valid && s->fb_iova &&
               s->fb_width == panel_width && s->fb_height == panel_height &&
               s->fb_source_width == panel_width &&
               s->fb_source_height == panel_height &&
               s->fb_pitch == pitch && s->fb_format == format) {
        refresh_existing = true;
    }

    if (!image_scanout && (full_source || refresh_existing)) {
        s->fb_buffer_ready = false;
        s->scanout_dirty = true;
        mt8113_hwtcon_schedule_scanout(s);
        queued = true;
    }

    s->mdp_transactions++;
    trace_mt8113_hwtcon_mdp_source(
        source, source_end, s->fb_iova, format, width, height, pitch,
        panel_width, panel_height, queued);
}

static void mt8113_hwtcon_complete_mdp_wrot(void *opaque)
{
    MT8113HWTCONState *s = opaque;

    s->bank[1].regs[MDP_WROT_INT_STATUS / 4] |= BIT(0);
    mt8113_hwtcon_update_mdp_irq(s);
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

    if (bank->index != 0) {
        if (offset == MDP_WROT_INT_STATUS) {
            bank->regs[offset / sizeof(uint32_t)] = old & ~(uint32_t)value;
            mt8113_hwtcon_update_mdp_irq(s);
            return;
        }

        bank->regs[offset / sizeof(uint32_t)] = value;
        /*
         * The in-kernel fallback path owns image mutex 0, while the MT8512
         * userspace MDP service programs the same pipeline through mutex 6.
         * Both enable writes start the already-configured RDMA/WROT graph.
         */
        if ((offset == MDP_MUTEX0_EN || offset == MDP_MUTEX6_EN) &&
            (value & BIT(0))) {
            mt8113_hwtcon_mdp_transaction(s, bank);
            /* The writeback above is synchronous, so its frame-done status is
             * already observable when the trigger write completes. */
            mt8113_hwtcon_complete_mdp_wrot(s);
        } else if (offset == MDP_WROT_INT_EN) {
            mt8113_hwtcon_update_mdp_irq(s);
        }
        return;
    }

    bank->regs[offset / sizeof(uint32_t)] = value;

    switch (offset) {
    case WF_LUT_SHADOW_UP:
        if ((value & BIT(0)) &&
            (*mt8113_hwtcon_reg(s, WF_LUT_EN) & BIT(0))) {
            uint32_t info = *mt8113_hwtcon_reg(s, WF_LUT_INFO_ID_CFG);
            unsigned lut = extract32(info, 4, 6);

            s->waveform_triggers++;
            if (s->image_scanout_active &&
                mt8113_hwtcon_publish_pending_frame(s)) {
                s->scanout_dirty = true;
            } else if (s->cfa_active) {
                /* CFA memory becomes visible on the physical panel only when
                 * the waveform engine commits the accepted pipeline update. */
                s->fb_buffer_ready = false;
                s->scanout_dirty = true;
            }
            /* A waveform commit updates or retains the physical panel. */
            mt8113_hwtcon_schedule_scanout(s);
            mt8113_hwtcon_enable_luts(s, lut / 32, BIT(lut % 32));
        }
        break;
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
            uint32_t position =
                *mt8113_hwtcon_reg(s, PAPER_TCTOP_UPD_CFG0);
            uint32_t extent =
                *mt8113_hwtcon_reg(s, PAPER_TCTOP_UPD_CFG1);
            uint32_t pipeline_flag =
                *mt8113_hwtcon_reg(s, PAPER_TCTOP_PIPELINE_FLAG);
            uint32_t img_addr =
                *mt8113_hwtcon_reg(s, PAPER_TCTOP_IMG_ST_ADDR);
            uint32_t wb_addr0 =
                *mt8113_hwtcon_reg(s, PAPER_TCTOP_WB_ST_ADDR0);
            uint32_t wb_addr1 =
                *mt8113_hwtcon_reg(s, PAPER_TCTOP_WB_ST_ADDR1);
            uint32_t lut = extract32(value, 4, 6);
            uint32_t *void_lut =
                mt8113_hwtcon_reg(s, PAPER_TCTOP_VOID_LUT);
            bool working_buffer_update = wb_addr0 == wb_addr1;
            bool dual_mode =
                *mt8113_hwtcon_reg(s, WF_LUT_DUAL_MODE) & BIT(0);
            bool dual_second = dual_mode && working_buffer_update &&
                mt8113_hwtcon_dual_pipeline_second_pass(
                    s, img_addr, wb_addr0, wb_addr1, position, extent,
                    pipeline_flag, lut);

            s->pipeline_triggers++;
            s->last_update_x = position & 0x3fff;
            s->last_update_y = (position >> 16) & 0x3fff;
            s->last_update_width = extent & 0x3fff;
            s->last_update_height = (extent >> 16) & 0x3fff;
            s->last_pipeline_flags = pipeline_flag;
            s->last_pipeline_lut = lut;
            *void_lut = pipeline_flag & PAPER_TCTOP_PIPELINE_CLEAR ?
                BIT(1) : 0;
            if (s->image_scanout_active &&
                working_buffer_update &&
                mt8113_hwtcon_capture_pending_frame(s) &&
                mt8113_hwtcon_update_working_image(
                    s, !(pipeline_flag & PAPER_TCTOP_PIPELINE_CLEAR),
                    !dual_mode || dual_second)) {
                *void_lut |= BIT(0);
            }
            if (dual_second) {
                s->dual_pipeline_pending = false;
            } else if (dual_mode && working_buffer_update) {
                mt8113_hwtcon_remember_dual_pipeline_first_pass(
                    s, img_addr, wb_addr0, wb_addr1, position, extent,
                    pipeline_flag, lut);
            } else {
                s->dual_pipeline_pending = false;
            }
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
    s->fb_base_iova = 0;
    s->fb_guest_va = 0;
    s->fb_width = 0;
    s->fb_height = 0;
    s->fb_source_width = 0;
    s->fb_source_height = 0;
    s->fb_pitch = 0;
    s->fb_format = 0;
    s->fb_rotation = 0;
    s->last_update_x = 0;
    s->last_update_y = 0;
    s->last_update_width = 0;
    s->last_update_height = 0;
    s->last_pipeline_flags = 0;
    s->last_pipeline_lut = 0;
    s->last_frame_min = 0;
    s->last_frame_max = 0;
    s->dual_pipeline_pending = false;
    /* The physical e-ink panel and its composed image survive a SoC reset. */
    s->pending_fb_buffer_ready = false;
    s->working_image_buffer_size = 0;
    s->cfa_active = false;
    s->scanout_dirty = false;
    s->image_scanout_active = false;
    s->boot_handoff_active = false;
    s->boot_handoff_deadline_ms = 0;
    memset(s->cfa_mailbox_regs, 0, sizeof(s->cfa_mailbox_regs));
    s->mdp_transactions = 0;
    s->mdp_writebacks = 0;
    s->mdp_writeback_skips = 0;
    s->mdp_writeback_failures = 0;
    s->pipeline_triggers = 0;
    s->pipeline_voids = 0;
    s->waveform_triggers = 0;
    s->boot_handoff_arms = 0;
    s->boot_blank_retentions = 0;
    s->cfa_source_reports = 0;
    s->cfa_fault_va = 0;
    s->cfa_fault_cpu = 0;
    s->scanout_captures = 0;
    s->scanout_refreshes = 0;
    s->scanout_read_failures = 0;
    timer_del(s->lut_timer);
    if (s->refresh_timer) {
        timer_del(s->refresh_timer);
        mt8113_hwtcon_schedule_scanout(s);
    }
    for (unsigned i = 0; i < MT8113_HWTCON_NUM_IRQS; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
    qemu_set_irq(s->mdp_wrot_irq, 0);
}

static void mt8113_hwtcon_init(Object *obj)
{
    MT8113HWTCONState *s = MT8113_HWTCON(obj);

    object_property_add_uint32_ptr(obj, "scanout-iova", &s->fb_iova,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "scanout-base-iova",
                                   &s->fb_base_iova,
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
    object_property_add_uint32_ptr(obj, "last-update-x", &s->last_update_x,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "last-update-y", &s->last_update_y,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "last-update-width",
                                   &s->last_update_width,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "last-update-height",
                                   &s->last_update_height,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "last-pipeline-flags",
                                   &s->last_pipeline_flags,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "last-pipeline-lut",
                                   &s->last_pipeline_lut,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "last-frame-min",
                                   &s->last_frame_min,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "last-frame-max",
                                   &s->last_frame_max,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "mdp-transactions",
                                   &s->mdp_transactions,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "mdp-writebacks",
                                   &s->mdp_writebacks,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "mdp-writeback-skips",
                                   &s->mdp_writeback_skips,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "mdp-writeback-failures",
                                   &s->mdp_writeback_failures,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "pipeline-triggers",
                                   &s->pipeline_triggers,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "pipeline-voids",
                                   &s->pipeline_voids,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "waveform-triggers",
                                   &s->waveform_triggers,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "boot-handoff-arms",
                                   &s->boot_handoff_arms,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "boot-blank-retentions",
                                   &s->boot_blank_retentions,
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
    qdev_init_gpio_out_named(DEVICE(obj), &s->mdp_wrot_irq,
                             "mdp-wrot-irq", 1);
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
    DisplaySurface *surface;

    if (!s->iommu) {
        error_setg(errp, "HWTCON requires an MT8113 IOMMU link");
        return;
    }
    s->console = graphic_console_init(dev, 0, &mt8113_hwtcon_gfx_ops, s);
    surface = mt8113_hwtcon_prepare_surface(s, HWTCON_PANEL_WIDTH,
                                            HWTCON_PANEL_HEIGHT);
    memset(surface_data(surface), 0xff,
           (size_t)surface_stride(surface) * HWTCON_PANEL_HEIGHT);
    dpy_gfx_update_full(s->console);
    s->refresh_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                    mt8113_hwtcon_refresh_scanout, s);
}

static void mt8113_hwtcon_unrealize(DeviceState *dev)
{
    MT8113HWTCONState *s = MT8113_HWTCON(dev);

    timer_free(s->refresh_timer);
    s->refresh_timer = NULL;
    g_free(s->fb_buffer);
    s->fb_buffer = NULL;
    g_free(s->pending_fb_buffer);
    s->pending_fb_buffer = NULL;
    g_free(s->working_image_buffer);
    s->working_image_buffer = NULL;
    s->working_image_buffer_size = 0;
}

static const Property mt8113_hwtcon_properties[] = {
    DEFINE_PROP_LINK("iommu", MT8113HWTCONState, iommu,
                     TYPE_MT8113_IOMMU, MT8113IOMMUState *),
    DEFINE_PROP_BOOL("scanout-image-buffer", MT8113HWTCONState,
                     scanout_image_buffer, false),
    DEFINE_PROP_BOOL("retain-boot-splash", MT8113HWTCONState,
                     retain_boot_splash, false),
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
