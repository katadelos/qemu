/* Allwinner B300 / sun8iw15p1. SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_ALLWINNER_B300_H
#define HW_ARM_ALLWINNER_B300_H

#include "hw/core/sysbus.h"
#include "hw/i2c/allwinner-i2c.h"
#include "hw/intc/arm_gic.h"
#include "hw/sd/allwinner-sdhost.h"
#include "target/arm/cpu.h"

#define TYPE_AW_B300 "allwinner-b300"
OBJECT_DECLARE_SIMPLE_TYPE(AwB300State, AW_B300)

#define AW_B300_NUM_CPUS 4
#define AW_B300_NUM_MMC 3
#define AW_B300_NUM_I2C 3
#define AW_B300_NUM_SPI 160
#define AW_B300_NUM_GPIO (12 * 32)
#define AW_B300_SDRAM_BASE 0x40000000

struct AwB300State {
    SysBusDevice parent_obj;
    ARMCPU cpus[AW_B300_NUM_CPUS];
    GICState gic;
    AwSdHostState mmc[AW_B300_NUM_MMC];
    AWI2CState i2c[AW_B300_NUM_I2C];
    DeviceState *display;
    DeviceState *power;
    DeviceState *iommu;
    MemoryRegion brom;
    MemoryRegion sram_a1;
    MemoryRegion sram_a2;
    MemoryRegion sram_c;
};

#endif
