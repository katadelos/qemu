#ifndef IMX_GPCV2_H
#define IMX_GPCV2_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

enum IMXGPCv2Registers {
    GPC_NUM        = 0xE00 / sizeof(uint32_t),
    GPC_NUM_IRQS   = 128,
    GPC_NUM_CPUS   = 2,
};

struct IMXGPCv2State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    uint32_t     regs[GPC_NUM];
    uint32_t     irq_levels[GPC_NUM_IRQS / 32];
    qemu_irq     irq_out[GPC_NUM_IRQS];
    qemu_irq     wake[GPC_NUM_CPUS];
};

#define TYPE_IMX_GPCV2 "imx-gpcv2"
OBJECT_DECLARE_SIMPLE_TYPE(IMXGPCv2State, IMX_GPCV2)

#endif /* IMX_GPCV2_H */
