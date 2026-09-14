/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_MT8171_GCE_H
#define HW_MISC_MT8171_GCE_H
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#define TYPE_MT8171_GCE "mt8171-gce"
OBJECT_DECLARE_SIMPLE_TYPE(MT8171GCEState, MT8171_GCE)
struct MT8171GCEState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[0x4000 / 4];
    uint32_t gpr[65536];
    uint32_t spr[32][4];
    uint32_t mask[32];
    bool tokens[1024];
    int64_t compare_deadline[16];
    uint16_t compare_armed;
    uint32_t pending;
    uint32_t waiting;
    uint16_t token[32];
    uint16_t selected;
    unsigned executing;
};
#endif
