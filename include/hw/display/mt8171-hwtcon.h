/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_DISPLAY_MT8171_HWTCON_H
#define HW_DISPLAY_MT8171_HWTCON_H
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "hw/misc/mt8171-m4u.h"
#define TYPE_MT8171_HWTCON "mt8171-hwtcon"
OBJECT_DECLARE_SIMPLE_TYPE(MT8171HWTCONState, MT8171_HWTCON)
typedef struct MT8171HWTCONBank {
    MemoryRegion iomem;
    MT8171HWTCONState *owner;
    unsigned index;
    uint32_t regs[0x1e000 / 4];
} MT8171HWTCONBank;
typedef struct MT8171WaveJob {
    uint16_t x, y, width, height, slot, frame, frames;
    bool active;
} MT8171WaveJob;
struct MT8171HWTCONState {
    SysBusDevice parent_obj;
    MT8171HWTCONBank bank[2];
    MT8171M4UState *m4u;
    uint32_t *larb3_regs;
    uint32_t *larb8_regs;
    /* Same order as the signed DT's nine interrupts. */
    qemu_irq irq[9];
    qemu_irq event[3]; /* WB WDMA331, waveform333, DPI169 */
    uint64_t pipeline_triggers;
    uint64_t waveform_triggers;
    uint64_t clear_writebacks;
    uint64_t normal_writebacks;
    uint64_t numerical_regal_jobs;
    uint64_t dma_faults;
    uint64_t waveform_jobs;
    uint64_t completed_frames;
    uint64_t completed_waveforms;
    uint64_t drive_pixels;
    QEMUTimer *frame_timer;
    MT8171WaveJob job[256];
    /* Last emitted source-driver nibble per pixel; no optical calibration. */
    uint8_t *drive_frame;
    int16_t *drive_accumulator;
    size_t drive_frame_size;
};
/* Decode supported byte-addressed image precision with uncompressed WB24.
 * The mask retains the valid most significant image bits. */
bool mt8171_hwtcon_image_mask(uint32_t format, uint8_t *mask);

#endif
