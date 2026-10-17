/* Allwinner B288 / sun8iw10p1 SoC. SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_ALLWINNER_B288_H
#define HW_ARM_ALLWINNER_B288_H

#include "hw/core/sysbus.h"
#include "hw/i2c/allwinner-i2c.h"
#include "hw/intc/arm_gic.h"
#include "hw/sd/allwinner-sdhost.h"
#include "target/arm/cpu.h"

#define TYPE_AW_B288 "allwinner-b288"
OBJECT_DECLARE_SIMPLE_TYPE(AwB288State, AW_B288)

#define AW_B288_NUM_CPUS 2
#define AW_B288_NUM_MMC 4
#define AW_B288_NUM_I2C 3
#define AW_B288_NUM_SPI 128
#define AW_B288_SDRAM_BASE 0x40000000

struct AwB288State {
    SysBusDevice parent_obj;
    ARMCPU cpus[AW_B288_NUM_CPUS];
    GICState gic;
    AwSdHostState mmc[AW_B288_NUM_MMC];
    AWI2CState i2c[AW_B288_NUM_I2C];
    DeviceState *display;
    MemoryRegion sram_a;
    MemoryRegion sram_c;
};

#endif
