/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_IMX7_DDRC_H
#define HW_MISC_IMX7_DDRC_H

#include "hw/core/sysbus.h"

#define TYPE_IMX7_DDRC "imx7-ddrc"
OBJECT_DECLARE_SIMPLE_TYPE(IMX7DDRCState, IMX7_DDRC)

#define IMX7_DDRC_REG_SIZE 0x1000

struct IMX7DDRCState {
    SysBusDevice parent_obj;
    MemoryRegion controller;
    MemoryRegion phy;
    uint32_t controller_regs[IMX7_DDRC_REG_SIZE / sizeof(uint32_t)];
    uint32_t phy_regs[IMX7_DDRC_REG_SIZE / sizeof(uint32_t)];
};

#endif
