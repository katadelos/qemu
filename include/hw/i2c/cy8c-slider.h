/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_I2C_CY8C_SLIDER_H
#define HW_I2C_CY8C_SLIDER_H

#include "hw/core/qdev.h"

#define TYPE_CY8C_SLIDER "cy8c-slider"

void cy8c_slider_set_position(DeviceState *dev, uint8_t position);
void cy8c_slider_step(DeviceState *dev, int delta);

#endif
