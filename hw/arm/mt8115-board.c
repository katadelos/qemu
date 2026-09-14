/* MT8115 fitted-board descriptions.
 * Values originate in the shipped board DT and U-Boot; synthetic identities
 * and virtual secure-monitor policy are explicitly identified below.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/mt8115-board.h"

/* PA6 and CS8 ship the same signed DT and describe the same peripheral bus
 * wiring. Product selection remains firmware/driver behavior, not DT edits. */
static const MT8115I2CFittedDevice pa6_cs8_i2c[] = {
    { "fp9967", 3, 0x49, {96, -1}, NULL, -1 },
    { "fp9967", 3, 0x26, {92, -1}, NULL, -1 },
    { "fp9967", 4, 0x26, {88, -1}, NULL, -1 },
    { "max20342", 4, 0x35, {0, -1}, NULL, -1 },
    { "sgm66075", 5, 0x76, {-1, -1}, NULL, -1 },
    { "ti-opt4001", 5, 0x45, {-1, -1}, NULL, -1 },
    { "ti-opt3001", 9, 0x44, {118, -1}, NULL, -1 },
    { "ti-opt3001", 9, 0x45, {119, -1}, NULL, -1 },
    { "lsm6dso16is", 4, 0x6b, {55, 56}, NULL, -1 },
    { "fp9936", 9, 0x38, {135, -1}, "power-good", 5 },
};

static const MT8115BoardConfig calvados = {
    .name = "calvados",
    .product_name = "cs8",
    /* 4VX is a supported serial device code, not a captured factory tattoo. */
    .board_id = "0004VX0000000000",
    .serial = "G004VX0000000000",
    .device_type_id = "4VX",
    .manufacturing = "ScribeColorsoftQEMU",
    .hwid_pins = {52, 54, 53},
    .panel_width = 2648, .panel_height = 1986,
    .touch_width = 1986, .touch_height = 2648,
    .touch_bus = 2, .touch_eint = 140,
    .touch_reset_pin = 143, .touch_id_pin = 141,
    .touch_vendor_id = 0x5d,
    .i2c_devices = pa6_cs8_i2c,
    .num_i2c_devices = ARRAY_SIZE(pa6_cs8_i2c),
    /* Virtual monitor policy for the DT's shared MDPSYS_CONFIG client. */
    .mdp_shared_dma_ports = 0xdf,
};

static const MT8115BoardConfig paloma = {
    .name = "paloma",
    .product_name = "pa6",
    /* Supported KS3 serial code; these are synthetic emulator identities. */
    .board_id = "0004PG0000000000",
    .serial = "G004PG0000000000",
    .device_type_id = "4PG",
    .manufacturing = "Scribe3QEMU",
    .hwid_pins = {52, 54, 53},
    .panel_width = 2648, .panel_height = 1986,
    .touch_width = 1986, .touch_height = 2648,
    .touch_bus = 2, .touch_eint = 140,
    .touch_reset_pin = 143, .touch_id_pin = 141,
    .touch_vendor_id = 0x5c,
    .i2c_devices = pa6_cs8_i2c,
    .num_i2c_devices = ARRAY_SIZE(pa6_cs8_i2c),
    .mdp_shared_dma_ports = 0xdf,
};

const MT8115BoardConfig *mt8115_board_config(const char *name)
{
    if (!strcmp(name, calvados.name)) {
        return &calvados;
    }
    if (!strcmp(name, paloma.name)) {
        return &paloma;
    }
    return NULL;
}

/* The identical shipped PA6/CS8 U-Boot labels GPIO HWIDs 0/1/2 as
 * HVT1.1/EVT/DVT. Production is an emulator alias for its latest named phase;
 * it does not modify the separate SBC/AR fuse policy. */
int mt8115_board_profile_hwid(const char *name)
{
    static const struct { const char *name; int hwid; } profiles[] = {
        { "production", 2 }, { "dvt", 2 }, { "evt", 1 }, { "hvt1.1", 0 },
    };
    for (unsigned i = 0; i < ARRAY_SIZE(profiles); i++) {
        if (!strcmp(name, profiles[i].name)) {
            return profiles[i].hwid;
        }
    }
    return -1;
}
