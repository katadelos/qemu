/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_DISPLAY_MT8171_MDP_H
#define HW_DISPLAY_MT8171_MDP_H
#include "hw/core/sysbus.h"
#include "hw/misc/mt8171-m4u.h"
#include "qemu/timer.h"
#include "hw/display/mt8171-panel.h"

#define TYPE_MT8171_MDP "mt8171-mdp"
OBJECT_DECLARE_SIMPLE_TYPE(MT8171MDPState, MT8171_MDP)
struct MT8171MDPState {
    SysBusDevice parent_obj;
    MemoryRegion control, engines;
    uint32_t regs[0x16000 / 4];
    MT8171M4UState *m4u;
    MT8171PanelState *panel;
    /* Board's larb2 register storage, including NONSEC_CON[0..31]. */
    uint32_t *larb_regs;
    qemu_irq irq[20]; /* mutex, RDMA0..DITHER: SPI 584..603 */
    qemu_irq events[1024];
    QEMUTimer *lut_timer;
    uint8_t lut[8][0x17640];
    uint8_t lut_valid;
    bool color_busy, color_pending;
    unsigned lut_mode;
    hwaddr lut_address;
    uint64_t completed_tiles, companion_tiles;
};
#endif
