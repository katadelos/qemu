/* Freescale i.MX TrustZone Interrupt Controller */
#ifndef HW_INTC_IMX_TZIC_H
#define HW_INTC_IMX_TZIC_H

#include "hw/core/sysbus.h"

#define TYPE_IMX_TZIC "imx.tzic"
OBJECT_DECLARE_SIMPLE_TYPE(IMXTZICState, IMX_TZIC)

#define IMX_TZIC_NUM_IRQS 128

struct IMXTZICState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t enabled[4];
    uint32_t pending[4];
    uint32_t priority[32];
    uint32_t intcntl;
    qemu_irq irq;
};
#endif
