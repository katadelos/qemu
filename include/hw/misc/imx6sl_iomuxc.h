#ifndef HW_MISC_IMX6SL_IOMUXC_H
#define HW_MISC_IMX6SL_IOMUXC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_IMX6SL_IOMUXC "imx6sl-iomuxc"
OBJECT_DECLARE_SIMPLE_TYPE(IMX6SLIOMUXCState, IMX6SL_IOMUXC)

#define IMX6SL_IOMUXC_SIZE 0x4000

struct IMX6SLIOMUXCState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[IMX6SL_IOMUXC_SIZE / sizeof(uint32_t)];
};

#endif
