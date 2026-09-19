/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_IMX7_DAP_H
#define HW_MISC_IMX7_DAP_H

#include "hw/core/sysbus.h"

#define TYPE_IMX7_DAP "imx7.dap"
OBJECT_DECLARE_SIMPLE_TYPE(IMX7DAPState, IMX7_DAP)

#define IMX7_DAP_COMPONENTS 7

struct IMX7DAPState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[IMX7_DAP_COMPONENTS][0x1000 / 4];
};

#endif
