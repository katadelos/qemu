/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_I2C_WACOM_EMR_H
#define HW_I2C_WACOM_EMR_H
#include "hw/i2c/i2c.h"
#define TYPE_WACOM_EMR "wacom-emr"
#define TYPE_WACOM_W9013 "wacom-w9013"
OBJECT_DECLARE_SIMPLE_TYPE(WacomEMRState, WACOM_EMR)
void wacom_emr_set_input(DeviceState *dev, uint16_t x, uint16_t y,
                         uint16_t pressure, bool proximity, bool eraser);
#endif
