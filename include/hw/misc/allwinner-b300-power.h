/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_ALLWINNER_B300_POWER_H
#define HW_MISC_ALLWINNER_B300_POWER_H

#include "hw/core/sysbus.h"

#define TYPE_AW_B300_POWER "allwinner-b300-power"
OBJECT_DECLARE_SIMPLE_TYPE(AWB300PowerState, AW_B300_POWER)

struct AWB300PowerState {
    SysBusDevice parent_obj;
    MemoryRegion rsb_mmio;
    MemoryRegion msgbox_mmio;
    MemoryRegion spinlock_mmio;
    qemu_irq rsb_irq;
    qemu_irq msgbox_irq;
    qemu_irq pmic_irq;
    qemu_irq wifi_power;
    qemu_irq lcd_power;
    uint32_t rsb[0x304 / 4];
    uint32_t msgbox[0x1000 / 4];
    uint32_t fifo[8][8];
    uint8_t head[8];
    uint8_t count[8];
    uint32_t locks;
    uint32_t lock_irq_enable;
    uint32_t lock_irq_pending;
    uint8_t pmu[256];
    uint8_t bmu[256];
    uint8_t bmu_extended[256];
    uint8_t runtime_address[2];
    bool scp_started;
    bool scp_notified;
};

/* Release the SCP after boot0 has installed its SRAM and shared-memory images. */
void aw_b300_power_scp_start(AWB300PowerState *s);

#endif
