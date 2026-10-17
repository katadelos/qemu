/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_I2C_CYTTSP5_H
#define HW_I2C_CYTTSP5_H
#include "hw/core/qdev.h"
#define TYPE_CYTTSP5 "cyttsp5"
/* Coordinates use QEMU's absolute input range (0..0x7fff). */
void cyttsp5_set_input(DeviceState *dev, int x, int y, bool down);
#endif
