/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MT6577_AUXADC_H
#define HW_MT6577_AUXADC_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"

#define TYPE_MT6577_AUXADC "mt6577-auxadc"
OBJECT_DECLARE_SIMPLE_TYPE(MT6577AuxADCState, MT6577_AUXADC)

struct MT6577AuxADCState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QEMUTimer *timer;
    uint32_t regs[0x1000 / 4];
    uint32_t input_mv[16];
    uint32_t sampled_raw[16];
    int64_t complete_ns[16];
    int64_t power_ready_ns;
    uint16_t pending;
    uint64_t conversions;
};

#endif
