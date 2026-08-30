/*
 * MediaTek MT8113 hardware timing controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_MT8113_HWTCON_H
#define HW_DISPLAY_MT8113_HWTCON_H

#include "hw/core/sysbus.h"
#include "hw/misc/mt8113_iommu.h"
#include "qemu/timer.h"
#include "ui/console.h"

#define TYPE_MT8113_HWTCON "mt8113.hwtcon"
OBJECT_DECLARE_SIMPLE_TYPE(MT8113HWTCONState, MT8113_HWTCON)

#define MT8113_HWTCON_MMIO_SIZE 0x10000
#define MT8113_HWTCON_MMIO_BANKS 2
#define MT8113_HWTCON_CFA_MAILBOX_ADDR 0x400000d0
#define MT8113_HWTCON_CFA_MAILBOX_SIZE 0x20

enum {
    MT8113_HWTCON_WB_FRAME_DONE_IRQ,
    MT8113_HWTCON_WF_LUT_FRAME_DONE_IRQ,
    MT8113_HWTCON_WF_LUT_RELEASE_IRQ,
    MT8113_HWTCON_DPI_UPDATE_DONE_IRQ,
    MT8113_HWTCON_TCON_END_IRQ,
    MT8113_HWTCON_PIXEL_LUT_COLLISION_IRQ,
    MT8113_HWTCON_DPI_SOF_IRQ,
    MT8113_HWTCON_NUM_IRQS,
};

typedef struct MT8113HWTCONBank {
    MemoryRegion iomem;
    MT8113HWTCONState *owner;
    unsigned index;
    uint32_t regs[MT8113_HWTCON_MMIO_SIZE / sizeof(uint32_t)];
} MT8113HWTCONBank;

struct MT8113HWTCONState {
    SysBusDevice parent_obj;
    MT8113HWTCONBank bank[MT8113_HWTCON_MMIO_BANKS];
    MemoryRegion cfa_mailbox;
    uint32_t cfa_mailbox_regs[MT8113_HWTCON_CFA_MAILBOX_SIZE /
                              sizeof(uint32_t)];
    qemu_irq irq[MT8113_HWTCON_NUM_IRQS];
    qemu_irq gce_frame_done;
    qemu_irq mdp_wrot_irq;
    QEMUTimer *lut_timer;
    QEMUTimer *refresh_timer;
    QemuConsole *console;
    MT8113IOMMUState *iommu;
    uint8_t *fb_buffer;
    uint8_t *pending_fb_buffer;
    uint8_t *working_image_buffer;
    size_t working_image_buffer_size;
    uint32_t fb_iova;
    uint32_t fb_base_iova;
    uint32_t fb_guest_va;
    uint32_t fb_width;
    uint32_t fb_height;
    uint32_t fb_source_width;
    uint32_t fb_source_height;
    uint32_t fb_pitch;
    uint32_t fb_format;
    uint32_t fb_rotation;
    uint32_t last_update_x;
    uint32_t last_update_y;
    uint32_t last_update_width;
    uint32_t last_update_height;
    uint32_t last_pipeline_flags;
    uint32_t last_pipeline_lut;
    uint32_t last_frame_min;
    uint32_t last_frame_max;
    uint32_t dual_pipeline_img_addr;
    uint32_t dual_pipeline_wb_addr0;
    uint32_t dual_pipeline_wb_addr1;
    uint32_t dual_pipeline_position;
    uint32_t dual_pipeline_extent;
    uint32_t dual_pipeline_flags;
    uint32_t dual_pipeline_lut;
    bool fb_buffer_ready;
    bool pending_fb_buffer_ready;
    bool dual_pipeline_pending;
    bool cfa_active;
    bool scanout_dirty;
    bool scanout_image_buffer;
    bool retain_boot_splash;
    bool image_scanout_active;
    bool boot_handoff_active;
    int64_t boot_handoff_deadline_ms;
    uint64_t mdp_transactions;
    uint64_t mdp_writebacks;
    uint64_t mdp_writeback_skips;
    uint64_t mdp_writeback_failures;
    uint64_t pipeline_triggers;
    uint64_t pipeline_voids;
    uint64_t waveform_triggers;
    uint64_t boot_handoff_arms;
    uint64_t boot_blank_retentions;
    uint64_t cfa_source_reports;
    uint32_t cfa_fault_va;
    uint32_t cfa_fault_cpu;
    uint64_t scanout_captures;
    uint64_t scanout_refreshes;
    uint64_t scanout_read_failures;
    uint64_t pending_luts;
};

#endif
