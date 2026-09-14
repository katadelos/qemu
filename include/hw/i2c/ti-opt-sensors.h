/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_I2C_TI_OPT_SENSORS_H
#define HW_I2C_TI_OPT_SENSORS_H
#define TYPE_TI_OPT3001 "ti-opt3001"
#define TYPE_TI_OPT4001 "ti-opt4001"
/* I2C slaves, GPIO output0 is physical INT (active-low by default).
 * millilux: explicit ambient illumination, defaults to100000 (100lux).
 * OPT4001 picostar=true (default): scale312.5ulux/count, no INT pin.
 * picostar=false: SOT-5X3 scale437.5ulux/count and INT output.
 */
#endif
