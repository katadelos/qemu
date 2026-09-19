/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_SD_BCM4343W_SDIO_H
#define HW_SD_BCM4343W_SDIO_H

#include "hw/sd/sd.h"

/* Standard NIC mac/netdev properties. GPIO power is WL_REG_ON; irq is the
 * in-band SDIO interrupt, and oob-irq is the separately enabled host-wake pin.
 */
OBJECT_DECLARE_SIMPLE_TYPE(BCM4343WState, BCM4343W_SDIO)

#endif
