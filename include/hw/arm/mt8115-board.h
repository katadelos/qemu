/* MT8115 fitted-board descriptions, independent of the shared SoC model.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_MT8115_BOARD_H
#define HW_ARM_MT8115_BOARD_H

typedef struct MT8115I2CFittedDevice {
    const char *type;
    unsigned bus, address;
    int eint[2];
    const char *irq_name;
    int pmic_enable_gpio;
} MT8115I2CFittedDevice;

typedef struct MT8115BoardConfig {
    const char *name;
    const char *product_name, *board_id, *serial, *device_type_id;
    const char *manufacturing;
    unsigned hwid_pins[3];
    unsigned panel_width, panel_height;
    unsigned touch_width, touch_height;
    unsigned touch_bus, touch_eint, touch_reset_pin, touch_id_pin;
    uint8_t touch_vendor_id;
    const MT8115I2CFittedDevice *i2c_devices;
    unsigned num_i2c_devices;
    uint64_t mdp_shared_dma_ports;
} MT8115BoardConfig;

const MT8115BoardConfig *mt8115_board_config(const char *name);
int mt8115_board_profile_hwid(const char *name);

#endif
