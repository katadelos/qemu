/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_I2C_FOCALTECH_FT3A81_H
#define HW_I2C_FOCALTECH_FT3A81_H
#define TYPE_FOCALTECH_FT3A81 "focaltech-ft3a81"
/* I2C application/bootloader read transport and two-contact host input.
 * Required firmware-image: original FT3A81 update asset.
 * vendor-id defaults0x5d, an explicit virtual Huizhou color profile.
 * GPIO output0=physical active-low INT; named reset[0]=active-low RESET.
 * Cocoa absolute pointer + left button uses contact0; QMP MTT supports0/1.
 * input-invert-x/y default true for the CS8 sensor mounting/DT flips.
 * Touch report delivery/read/press/release/overrun counters are read-only QOM.
 */
#endif
