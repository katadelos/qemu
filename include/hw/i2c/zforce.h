/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_I2C_ZFORCE_H
#define HW_I2C_ZFORCE_H

#include "hw/i2c/i2c.h"

#define TYPE_ZFORCE "neonode-zforce"
OBJECT_DECLARE_SIMPLE_TYPE(ZForceState, ZFORCE)

#define TYPE_KINDLE_ZFORCE "kindle-neonode-zforce"
#define TYPE_KINDLE_ZFORCE2 "kindle-neonode-zforce2"

#define TYPE_TPS65185 "tps65185"
OBJECT_DECLARE_SIMPLE_TYPE(TPS65185State, TPS65185)

#define TYPE_KOBOTOUCH_MSP430 "kobotouch-msp430"
OBJECT_DECLARE_SIMPLE_TYPE(KoboTouchMSP430State, KOBOTOUCH_MSP430)

#endif
