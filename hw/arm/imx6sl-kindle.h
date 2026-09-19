/*
 * Shared helpers for Lab126 i.MX6SoloLite Kindle boards.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_IMX6SL_KINDLE_H
#define HW_ARM_IMX6SL_KINDLE_H

#include "hw/arm/boot.h"
#include "hw/arm/fsl-imx6.h"
#include "hw/core/boards.h"

typedef struct KindleIMX6SLIdme {
    char *serial;
    char *mac;
    char *mfg;
    char *pcbsn;
    char *bootmode;
    char *postmode;
} KindleIMX6SLIdme;

void kindle_imx6sl_populate_idme(DeviceState *card,
                                const KindleIMX6SLIdme *idme);
int kindle_imx6sl_write_extra_atags(const struct arm_boot_info *info,
                                   void *opaque, void *buffer,
                                   size_t max_size);
hwaddr kindle_imx6sl_load_firmware(MachineState *machine);
void kindle_imx6sl_attach_panel_flash(FslIMX6State *s,
                                     const uint8_t *barcode);

#endif /* HW_ARM_IMX6SL_KINDLE_H */
