/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_IMX_SDMA_H
#define HW_IMX_SDMA_H
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#define TYPE_IMX_SDMA "imx-sdma"
OBJECT_DECLARE_SIMPLE_TYPE(IMXSDMAState, IMX_SDMA)
struct IMXSDMAState {
    SysBusDevice parent_obj;
    MemoryRegion memory;
    qemu_irq irq;
    QEMUTimer timer;
    uint32_t regs[0x10000 / 4], active;
    uint8_t data[0x4000], program[0x20000];
    uint32_t progress[32];
};
#endif
