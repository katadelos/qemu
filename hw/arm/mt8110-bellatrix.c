/*
 * Amazon/Lab126 Bellatrix platform emulation (MediaTek MT8110)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <libfdt.h>
#include "hw/arm/mt8113.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/bd71828.h"
#include "hw/i2c/fp9935.h"
#include "hw/i2c/goodix_gtx8.h"
#include "hw/i2c/max20342.h"
#include "hw/sd/sd.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/reset.h"

#define BELLATRIX_HANDOFF_ADDR      0x40001000
#define BELLATRIX_BOOTARGS_ADDR     0x40000100
#define BELLATRIX_UBOOT_ADDR        0x44e00000

#define BELLATRIX_BL2_GFH_OFFSET    0x800
#define BELLATRIX_BL2_LOAD_ADDR     0x00110d00
#define BELLATRIX_BL2_ENTRY         0x00111000

#define BELLATRIX_IDME_SIZE         (10 * 512)
#define BELLATRIX_IDME_VERSION      "2.1"

#define TYPE_BELLATRIX_MACHINE MACHINE_TYPE_NAME("mt8110-bellatrix")
OBJECT_DECLARE_SIMPLE_TYPE(BellatrixMachineState, BELLATRIX_MACHINE)

typedef struct BellatrixMachineState {
    MachineState parent_obj;
    MT8113State *soc;
    char *board;
    char *device_profile;
    char *bl2;
    char *tee;
    char *quickboot;
    char *boot_stage;
    char *idme_board_id;
    char *idme_serial;
    char *idme_mac;
    char *idme_mfg;
    char *idme_product_name;
    char *idme_device_type;
    char *idme_bootmode;
    char *idme_postmode;
    char *idme_fos_flags;
    char *idme_hwid;
    char *idme_bcm_stress;
} BellatrixMachineState;

typedef struct BellatrixIdmeField {
    const char *name;
    const char *value;
    size_t size;
    bool exportable;
} BellatrixIdmeField;

typedef struct BellatrixDeviceProfile {
    const char *name;
    unsigned hwid;
} BellatrixDeviceProfile;

typedef struct BellatrixBoard {
    const char *name;
    const char *board_id;
    const char *serial;
    const char *device_type;
    ram_addr_t ram_size;
} BellatrixBoard;

/* The board tattoo selects Cava or Rossini; HWID pins select its phase. */
static const BellatrixDeviceProfile bellatrix_device_profiles[] = {
    { .name = "production", .hwid = 4 },
    { .name = "dvt",        .hwid = 3 },
    { .name = "evt",        .hwid = 2 },
    { .name = "hvt",        .hwid = 1 },
    { .name = "proto",      .hwid = 0 },
};

static const BellatrixBoard bellatrix_boards[] = {
    {
        .name = "cava",
        .board_id = "0002R5000000000",
        .serial = "G0022D0000000000",
        .device_type = "22D",
        .ram_size = 512 * MiB,
    }, {
        .name = "rossini",
        .board_id = "0003M5000000000",
        .serial = "G003KM0000000000",
        .device_type = "3KM",
        .ram_size = 1 * GiB,
    },
};

static const BellatrixDeviceProfile *
bellatrix_find_device_profile(const char *name)
{
    for (size_t i = 0; i < ARRAY_SIZE(bellatrix_device_profiles); i++) {
        if (!strcmp(bellatrix_device_profiles[i].name, name)) {
            return &bellatrix_device_profiles[i];
        }
    }

    error_report("invalid Bellatrix device-profile '%s' (expected "
                 "production, dvt, evt, hvt, or proto)", name);
    exit(EXIT_FAILURE);
}

static const BellatrixBoard *bellatrix_find_board(const char *name)
{
    for (size_t i = 0; i < ARRAY_SIZE(bellatrix_boards); i++) {
        if (!strcmp(bellatrix_boards[i].name, name)) {
            return &bellatrix_boards[i];
        }
    }

    error_report("invalid Bellatrix board '%s' (expected cava or rossini)",
                 name);
    exit(EXIT_FAILURE);
}

static void bellatrix_apply_identity(BellatrixMachineState *bms)
{
    const BellatrixBoard *board = bellatrix_find_board(bms->board);
    const BellatrixDeviceProfile *profile =
        bellatrix_find_device_profile(bms->device_profile);

    g_free(bms->idme_board_id);
    g_free(bms->idme_serial);
    g_free(bms->idme_mfg);
    g_free(bms->idme_product_name);
    g_free(bms->idme_device_type);
    g_free(bms->idme_hwid);
    bms->idme_board_id = g_strdup(board->board_id);
    bms->idme_serial = g_strdup(board->serial);
    bms->idme_mfg = g_strdup("KINDLE11QEMU000000");
    bms->idme_product_name = g_strdup("");
    bms->idme_device_type = g_strdup(board->device_type);
    bms->idme_hwid = g_strdup_printf("%u", profile->hwid);
}

static void bellatrix_set_board_identity(BellatrixMachineState *bms,
                                          const char *name)
{
    bellatrix_find_board(name);
    g_free(bms->board);
    bms->board = g_strdup(name);
    bellatrix_apply_identity(bms);
}

static void bellatrix_set_device_profile_identity(
    BellatrixMachineState *bms, const char *name)
{
    bellatrix_find_device_profile(name);
    g_free(bms->device_profile);
    bms->device_profile = g_strdup(name);
    bellatrix_apply_identity(bms);
}

static void bellatrix_firmware_reset(void *opaque)
{
    ARMCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

static void bellatrix_profile_reset(void *opaque)
{
    BellatrixMachineState *bms = opaque;
    const BellatrixDeviceProfile *profile =
        bellatrix_find_device_profile(bms->device_profile);
    static const unsigned hwid_pins[] = { 14, 15, 16, 17 };

    for (size_t bit = 0; bit < ARRAY_SIZE(hwid_pins); bit++) {
        qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&bms->soc->gpio),
                                            "gpio-in", hwid_pins[bit]),
                     profile->hwid & BIT(bit));
    }
}

/* The real BootROM enters the shipped 32-bit BL2/U-Boot from AArch64 EL3. */
static const uint32_t bellatrix_handoff[] = {
    0x58000180, /* ldr x0, bootargs */
    0xd2803a61, /* mov x1, #0x1d3 (AArch32 SVC, AIF masked) */
    0xd51e4001, /* msr spsr_el3, x1 */
    0xd53e1101, /* mrs x1, scr_el3 */
    0x9275f821, /* bic x1, x1, #SCR_RW */
    0xb2400021, /* orr x1, x1, #SCR_NS */
    0xd51e1101, /* msr scr_el3, x1 */
    0x580000e1, /* ldr x1, entry */
    0xd51e4021, /* msr elr_el3, x1 */
    0xd5033fdf, /* isb */
    0xd69f03e0, /* eret */
    0xd503201f, /* nop/alignment */
    BELLATRIX_BOOTARGS_ADDR, 0,
    BELLATRIX_UBOOT_ADDR, 0,
};

static void bellatrix_load_fit_image(const char *filename,
                                      const char *node, const char *name)
{
    g_autofree uint8_t *fit = NULL;
    gsize fit_size;
    const void *data;
    const fdt32_t *load_prop;
    int data_size;
    int load_size;
    int image;
    hwaddr load;

    if (!g_file_get_contents(filename, (char **)&fit, &fit_size, NULL) ||
        fdt_check_header(fit)) {
        error_report("%s is not a valid FIT image", filename);
        exit(EXIT_FAILURE);
    }
    image = fdt_path_offset(fit, node);
    data = fdt_getprop(fit, image, "data", &data_size);
    load_prop = fdt_getprop(fit, image, "load", &load_size);
    if (image < 0 || !data || data_size <= 0 || !load_prop || load_size != 4) {
        error_report("%s has no loadable %s payload", filename, node);
        exit(EXIT_FAILURE);
    }
    load = fdt32_to_cpu(*load_prop);
    if (load < MT8113_RAM_BASE ||
        load + data_size > MT8113_RAM_BASE + MT8113_RAM_MAX) {
        error_report("%s payload has invalid load range", name);
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed(name, data, data_size, load);
}

static void bellatrix_load_bl2(BellatrixMachineState *bms)
{
    g_autofree uint8_t *image = NULL;
    gsize size;

    if (!bms->bl2) {
        error_report("boot-stage=bl2 requires the bl2 property");
        exit(EXIT_FAILURE);
    }
    if (!g_file_get_contents(bms->bl2, (char **)&image, &size, NULL) ||
        size <= BELLATRIX_BL2_GFH_OFFSET) {
        error_report("unable to load Bellatrix BL2 '%s'", bms->bl2);
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("bellatrix.bl2",
                       image + BELLATRIX_BL2_GFH_OFFSET,
                       size - BELLATRIX_BL2_GFH_OFFSET,
                       BELLATRIX_BL2_LOAD_ADDR);
}

static void bellatrix_load_firmware(BellatrixMachineState *bms,
                                     MachineState *machine)
{
    uint32_t handoff[ARRAY_SIZE(bellatrix_handoff)];
    g_autofree uint8_t *bootargs = g_malloc0(0x2dc);
    hwaddr entry = BELLATRIX_UBOOT_ADDR;

    if (!machine->firmware) {
        error_report("Bellatrix requires -bios with the stock u-boot.bin FIT");
        exit(EXIT_FAILURE);
    }

    bellatrix_load_fit_image(machine->firmware, "/images/kernel",
                             "bellatrix.u-boot");
    if (bms->tee) {
        bellatrix_load_fit_image(bms->tee, "/images/tee", "bellatrix.tee");
        bellatrix_load_fit_image(bms->tee, "/images/trustedos",
                                 "bellatrix.trustedos");
    }
    if (bms->quickboot) {
        bellatrix_load_fit_image(bms->quickboot, "/images/fbios",
                                 "bellatrix.fbios");
        bellatrix_load_fit_image(bms->quickboot, "/images/sbios",
                                 "bellatrix.sbios");
    }

    if (!strcmp(bms->boot_stage, "bl2")) {
        bellatrix_load_bl2(bms);
        entry = BELLATRIX_BL2_ENTRY;
    } else if (strcmp(bms->boot_stage, "u-boot")) {
        error_report("invalid boot-stage '%s' (expected u-boot or bl2)",
                     bms->boot_stage);
        exit(EXIT_FAILURE);
    }

    stl_le_p(bootargs, 0x504c504c);
    stl_le_p(bootargs + 4, machine->ram_size);
    stl_le_p(bootargs + 0x254, 0x504c504c);
    stl_le_p(bootargs + 0x2d8, 0x504c504c);
    rom_add_blob_fixed("bellatrix.bootargs", bootargs, 0x2dc,
                       BELLATRIX_BOOTARGS_ADDR);

    memcpy(handoff, bellatrix_handoff, sizeof(handoff));
    handoff[14] = entry;
    rom_add_blob_fixed("bellatrix.handoff", handoff, sizeof(handoff),
                       BELLATRIX_HANDOFF_ADDR);
}

static void bellatrix_populate_idme(DeviceState *card,
                                     BellatrixMachineState *bms)
{
    g_autofree uint8_t *contents = g_malloc0(BELLATRIX_IDME_SIZE);
    BellatrixIdmeField fields[] = {
        { "board_id",       bms->idme_board_id,   16, true },
        { "serial",         bms->idme_serial,     16, true },
        { "mac_addr",       bms->idme_mac,        16, true },
        { "mac_sec",        "0",                 32, true },
        { "bt_mac_addr",    "0",                 16, true },
        { "product_name",   bms->idme_product_name, 32, true },
        { "productid",      "0",                 32, true },
        { "productid2",     "0",                 32, true },
        { "region",         "US",                 4, true },
        { "bootmode",       bms->idme_bootmode,    4, true },
        { "postmode",       bms->idme_postmode,    4, true },
        { "bootcount",      "0",                  8, true },
        { "manufacturing",  bms->idme_mfg,       512, true },
        { "unlock_code",    "",                1024, true },
        { "device_type_id", bms->idme_device_type, 32, true },
        { "dev_flags",      "0",                  8, true },
        { "fos_flags",      bms->idme_fos_flags,   8, true },
        { "usr_flags",      "0",                  8, true },
        { "hwid",           bms->idme_hwid,        4, true },
        { "bcm_stress",     bms->idme_bcm_stress, 1024, true },
    };
    uint8_t *cursor = contents;

    memcpy(cursor, "beefdeed", 8);
    memcpy(cursor + 8, BELLATRIX_IDME_VERSION,
           sizeof(BELLATRIX_IDME_VERSION));
    stl_le_p(cursor + 12, ARRAY_SIZE(fields));
    cursor += 16;

    for (size_t i = 0; i < ARRAY_SIZE(fields); i++) {
        BellatrixIdmeField *field = &fields[i];
        size_t length = strlen(field->value);
        size_t item_size = ROUND_UP(28 + field->size, 4);

        if (strlen(field->name) > 16 || !g_str_is_ascii(field->value) ||
            length > field->size ||
            cursor + item_size > contents + BELLATRIX_IDME_SIZE) {
            error_report("IDME %s must contain at most %zu ASCII bytes",
                         field->name, field->size);
            exit(EXIT_FAILURE);
        }
        memcpy(cursor, field->name, strlen(field->name));
        stl_le_p(cursor + 16, field->size);
        stl_le_p(cursor + 20, field->exportable);
        stl_le_p(cursor + 24, 0444);
        memcpy(cursor + 28, field->value, length);
        cursor += item_size;
    }

    emmc_boot_partition_write(card, 2, 0, contents, BELLATRIX_IDME_SIZE,
                              &error_fatal);
}

static void bellatrix_write_boot_component(DeviceState *card,
                                            unsigned int partition,
                                            uint64_t start_block,
                                            uint64_t block_count,
                                            const char *filename,
                                            const char *name)
{
    g_autofree uint8_t *contents = NULL;
    gsize size;

    if (!filename) {
        return;
    }
    if (!g_file_get_contents(filename, (char **)&contents, &size, NULL)) {
        error_report("unable to load Bellatrix %s '%s'", name, filename);
        exit(EXIT_FAILURE);
    }
    if (size > block_count * 512) {
        error_report("Bellatrix %s exceeds its eMMC boot-area slot", name);
        exit(EXIT_FAILURE);
    }
    emmc_boot_partition_write(card, partition, start_block * 512,
                              contents, size, &error_fatal);
}

static void bellatrix_attach_emmc(BellatrixMachineState *bms)
{
    MachineState *machine = MACHINE(bms);
    DriveInfo *di = drive_get(IF_SD, 0, 0);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus = qdev_get_child_bus(DEVICE(&bms->soc->msdc0), "sd-bus");
    DeviceState *card = qdev_new(TYPE_EMMC);

    qdev_prop_set_uint64(card, "boot-partition-size", 4 * MiB);
    qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);

    /* Rossini's fixed boot-area slots from stock g_custom_partitions. */
    bellatrix_write_boot_component(card, 1, 0x000, 0x200,
                                   bms->bl2, "BL2");
    bellatrix_write_boot_component(card, 1, 0x400, 0x800,
                                   machine->firmware, "U-Boot");
    bellatrix_write_boot_component(card, 2, 0x400, 0x400,
                                   bms->quickboot, "quickboot");
    bellatrix_populate_idme(card, bms);
    object_unref(OBJECT(card));
}

static void bellatrix_init(MachineState *machine)
{
    BellatrixMachineState *bms = BELLATRIX_MACHINE(machine);
    const BellatrixBoard *board = bellatrix_find_board(bms->board);
    MT8113State *soc;
    I2CSlave *fp9930;
    I2CSlave *bd71828;
    I2CSlave *gtx8;

    if (machine->ram_size > board->ram_size) {
        error_report("%s RAM exceeds its %" PRIu64 " MiB hardware "
                     "configuration", board->name,
                     board->ram_size / MiB);
        exit(EXIT_FAILURE);
    }

    soc = MT8113(object_new(TYPE_MT8110));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_property_set_uint(OBJECT(soc), "reset-vector",
                             BELLATRIX_HANDOFF_ADDR, &error_fatal);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);
    bms->soc = soc;
    for (int cpu = 0; cpu < MT8113_NUM_CPUS; cpu++) {
        qemu_register_reset(bellatrix_firmware_reset, &soc->cpu[cpu]);
    }

    fp9930 = i2c_slave_create_simple(soc->i2c[0].bus, TYPE_FP9935, 0x18);
    i2c_slave_create_simple(soc->i2c[0].bus, TYPE_MAX20342, 0x35);
    bd71828 = i2c_slave_create_simple(soc->i2c[1].bus, TYPE_BD71828, 0x4b);
    i2c_slave_create_simple(soc->i2c[1].bus, TYPE_BD71828, 0x4d);
    gtx8 = i2c_slave_create_simple(soc->i2c[2].bus, TYPE_GOODIX_GTX8, 0x5d);

    qdev_connect_gpio_out_named(DEVICE(bd71828), "gpio", 1,
        qdev_get_gpio_in_named(DEVICE(fp9930), "enable", 0));
    qdev_connect_gpio_out_named(DEVICE(fp9930), "power-good", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 10));
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 10),
                 0);
    qdev_connect_gpio_out_named(DEVICE(gtx8), "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 0));
    qdev_connect_gpio_out_named(DEVICE(&soc->gpio), "gpio-out", 1,
        qdev_get_gpio_in_named(DEVICE(gtx8), "reset", 0));
    bellatrix_profile_reset(bms);

    memory_region_add_subregion(get_system_memory(), MT8113_RAM_BASE,
                                machine->ram);
    bellatrix_attach_emmc(bms);
    bellatrix_load_firmware(bms, machine);
    qemu_register_reset(bellatrix_profile_reset, bms);
}

static char *bellatrix_get_string(char **value)
{
    return g_strdup(*value);
}

static void bellatrix_set_string(char **field, const char *value)
{
    g_free(*field);
    *field = g_strdup(value);
}

#define BELLATRIX_STRING_PROPERTY(_member)                              \
    static char *bellatrix_get_##_member(Object *obj, Error **errp)     \
    {                                                                   \
        return bellatrix_get_string(&BELLATRIX_MACHINE(obj)->_member);  \
    }                                                                   \
    static void bellatrix_set_##_member(Object *obj, const char *value, \
                                        Error **errp)                   \
    {                                                                   \
        bellatrix_set_string(&BELLATRIX_MACHINE(obj)->_member, value);  \
    }

BELLATRIX_STRING_PROPERTY(bl2)
BELLATRIX_STRING_PROPERTY(tee)
BELLATRIX_STRING_PROPERTY(quickboot)
BELLATRIX_STRING_PROPERTY(boot_stage)
BELLATRIX_STRING_PROPERTY(idme_board_id)
BELLATRIX_STRING_PROPERTY(idme_serial)
BELLATRIX_STRING_PROPERTY(idme_mac)
BELLATRIX_STRING_PROPERTY(idme_mfg)
BELLATRIX_STRING_PROPERTY(idme_product_name)
BELLATRIX_STRING_PROPERTY(idme_device_type)
BELLATRIX_STRING_PROPERTY(idme_bootmode)
BELLATRIX_STRING_PROPERTY(idme_postmode)
BELLATRIX_STRING_PROPERTY(idme_fos_flags)
BELLATRIX_STRING_PROPERTY(idme_hwid)
BELLATRIX_STRING_PROPERTY(idme_bcm_stress)

static char *bellatrix_get_board(Object *obj, Error **errp)
{
    return g_strdup(BELLATRIX_MACHINE(obj)->board);
}

static void bellatrix_set_board(Object *obj, const char *value, Error **errp)
{
    bellatrix_set_board_identity(BELLATRIX_MACHINE(obj), value);
}

static char *bellatrix_get_device_profile(Object *obj, Error **errp)
{
    return g_strdup(BELLATRIX_MACHINE(obj)->device_profile);
}

static void bellatrix_set_device_profile(Object *obj, const char *value,
                                          Error **errp)
{
    bellatrix_set_device_profile_identity(BELLATRIX_MACHINE(obj), value);
}

static void bellatrix_machine_instance_init(Object *obj)
{
    BellatrixMachineState *bms = BELLATRIX_MACHINE(obj);

    bms->boot_stage = g_strdup("u-boot");
    bms->board = g_strdup("rossini");
    bms->device_profile = g_strdup("production");
    bellatrix_apply_identity(bms);
    bms->idme_mac = g_strdup("020000000006");
    bms->idme_bootmode = g_strdup("1");
    bms->idme_postmode = g_strdup("0");
    bms->idme_fos_flags = g_strdup("0");
    bms->idme_bcm_stress = g_strdup("");
}

static void bellatrix_machine_instance_finalize(Object *obj)
{
    BellatrixMachineState *bms = BELLATRIX_MACHINE(obj);

    if (bms->soc) {
        for (int cpu = 0; cpu < MT8113_NUM_CPUS; cpu++) {
            qemu_unregister_reset(bellatrix_firmware_reset,
                                  &bms->soc->cpu[cpu]);
        }
    }
    qemu_unregister_reset(bellatrix_profile_reset, bms);
    g_free(bms->board);
    g_free(bms->device_profile);
    g_free(bms->bl2);
    g_free(bms->tee);
    g_free(bms->quickboot);
    g_free(bms->boot_stage);
    g_free(bms->idme_board_id);
    g_free(bms->idme_serial);
    g_free(bms->idme_mac);
    g_free(bms->idme_mfg);
    g_free(bms->idme_product_name);
    g_free(bms->idme_device_type);
    g_free(bms->idme_bootmode);
    g_free(bms->idme_postmode);
    g_free(bms->idme_fos_flags);
    g_free(bms->idme_hwid);
    g_free(bms->idme_bcm_stress);
}

#define BELLATRIX_ADD_PROPERTY(_name, _member)                         \
    object_class_property_add_str(oc, _name,                           \
                                  bellatrix_get_##_member,             \
                                  bellatrix_set_##_member)

static void bellatrix_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Amazon/Lab126 Bellatrix platform (MediaTek MT8110)";
    mc->init = bellatrix_init;
    mc->max_cpus = MT8113_NUM_CPUS;
    mc->default_cpus = MT8113_NUM_CPUS;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a53");
    mc->default_ram_size = 1 * GiB;
    mc->default_ram_id = "bellatrix.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;

    object_class_property_add_str(oc, "board", bellatrix_get_board,
                                  bellatrix_set_board);
    object_class_property_add_str(oc, "device-profile",
                                  bellatrix_get_device_profile,
                                  bellatrix_set_device_profile);
    BELLATRIX_ADD_PROPERTY("bl2", bl2);
    BELLATRIX_ADD_PROPERTY("tee", tee);
    BELLATRIX_ADD_PROPERTY("quickboot", quickboot);
    BELLATRIX_ADD_PROPERTY("boot-stage", boot_stage);
    BELLATRIX_ADD_PROPERTY("idme-board-id", idme_board_id);
    BELLATRIX_ADD_PROPERTY("idme-serial", idme_serial);
    BELLATRIX_ADD_PROPERTY("idme-mac", idme_mac);
    BELLATRIX_ADD_PROPERTY("idme-mfg", idme_mfg);
    BELLATRIX_ADD_PROPERTY("idme-product-name", idme_product_name);
    BELLATRIX_ADD_PROPERTY("idme-device-type", idme_device_type);
    BELLATRIX_ADD_PROPERTY("idme-bootmode", idme_bootmode);
    BELLATRIX_ADD_PROPERTY("idme-postmode", idme_postmode);
    BELLATRIX_ADD_PROPERTY("idme-fos-flags", idme_fos_flags);
    BELLATRIX_ADD_PROPERTY("idme-hwid", idme_hwid);
    BELLATRIX_ADD_PROPERTY("idme-bcm-stress", idme_bcm_stress);
}

static const TypeInfo bellatrix_machine_type = {
    .name = TYPE_BELLATRIX_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(BellatrixMachineState),
    .instance_init = bellatrix_machine_instance_init,
    .instance_finalize = bellatrix_machine_instance_finalize,
    .class_init = bellatrix_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void bellatrix_machine_register_types(void)
{
    type_register_static(&bellatrix_machine_type);
}
type_init(bellatrix_machine_register_types)
