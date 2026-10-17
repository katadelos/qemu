/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef REMARKABLE_HWMON_H
#define REMARKABLE_HWMON_H
#include "chardev/char.h"
#include "hw/core/irq.h"
Chardev *remarkable_hwmon_create(uint32_t firmware_id);
qemu_irq remarkable_hwmon_reset_input(Chardev *chr);
qemu_irq remarkable_hwmon_wakeup_input(Chardev *chr);
void remarkable_hwmon_connect_wakeup(Chardev *chr, qemu_irq output);
#endif
