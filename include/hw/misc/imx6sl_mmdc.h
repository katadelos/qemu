/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_IMX6SL_MMDC_H
#define HW_MISC_IMX6SL_MMDC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_IMX6SL_MMDC "imx6sl-mmdc"
OBJECT_DECLARE_SIMPLE_TYPE(IMX6SLMMDCState, IMX6SL_MMDC)

struct IMX6SLMMDCState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[0x1000];
};

#endif
