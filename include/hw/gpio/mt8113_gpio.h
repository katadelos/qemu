/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_GPIO_MT8113_GPIO_H
#define HW_GPIO_MT8113_GPIO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_MT8113_GPIO "mt8113-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(MT8113GPIOState, MT8113_GPIO)

#define MT8113_GPIO_MMIO_SIZE 0x1000
#define MT8113_GPIO_NUM_PINS  124

struct MT8113GPIOState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion eint_iomem;
    qemu_irq irq;
    uint32_t regs[MT8113_GPIO_MMIO_SIZE / sizeof(uint32_t)];
    uint32_t eint_regs[MT8113_GPIO_MMIO_SIZE / sizeof(uint32_t)];
    uint32_t input_levels[DIV_ROUND_UP(MT8113_GPIO_NUM_PINS, 32)];
    qemu_irq output[MT8113_GPIO_NUM_PINS];
};

#endif
