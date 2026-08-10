/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_MAX44009_H
#define HW_MISC_MAX44009_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_MAX44009 "max44009"
OBJECT_DECLARE_SIMPLE_TYPE(MAX44009State, MAX44009)

struct MAX44009State {
    I2CSlave parent_obj;
    qemu_irq irq;
    uint8_t regs[16];
    uint8_t pointer;
    uint8_t tx_len;
    uint32_t lux_millilux;
    bool interrupt;
};

#endif
