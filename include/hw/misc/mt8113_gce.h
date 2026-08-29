/*
 * MediaTek MT8113 Global Command Engine
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MT8113_GCE_H
#define HW_MISC_MT8113_GCE_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_MT8113_GCE "mt8113.gce"
OBJECT_DECLARE_SIMPLE_TYPE(MT8113GCEState, MT8113_GCE)

#define MT8113_GCE_MMIO_SIZE 0x4000
#define MT8113_GCE_THREADS   32

struct MT8113GCEState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[MT8113_GCE_MMIO_SIZE / sizeof(uint32_t)];
    uint32_t gpr[UINT16_MAX + 1];
    bool tokens[1024];
    bool waiting[MT8113_GCE_THREADS];
    uint16_t wait_token[MT8113_GCE_THREADS];
    uint16_t selected_token;
    QEMUTimer *exec_timer;
    uint32_t pending_threads;
};

#endif
