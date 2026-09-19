/*
 * Amazon/Lab126 Duet (Kindle Oasis 1 / Whisky) board emulation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/arm/fsl-imx6.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/imx_epdc.h"
#include "hw/i2c/cyttsp4.h"
#include "hw/i2c/max77696.h"
#include "hw/input/wario-keyboard.h"
#include "hw/misc/imx6sl_iomuxc.h"
#include "hw/misc/imx6sl_mmdc.h"
#include "hw/misc/imx6sl_pxp.h"
#include "hw/misc/unimp.h"
#include "hw/sd/sd.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/qtest.h"
#include "system/reset.h"
#include "imx6sl-kindle.h"

#define DUET_RAM_BASE       0x80000000
#define DUET_RAM_MAX        (2 * GiB)
#define DUET_IRAM_STACK     0x00920000
#define DUET_MACHINE_ID     4091
#define DUET_EPDC_ADDR      0x020f4000
#define DUET_EPDC_GIC_IRQ   97
#define DUET_PXP_ADDR       0x020f0000
#define DUET_PXP_GIC_IRQ    98
#define DUET_FB_ADDR        0x81000000
#define DUET_FB_WIDTH       1072
#define DUET_FB_HEIGHT      1448
#define DUET_FB_STRIDE      1088
#define DUET_SBIOS_ADDR     0x9f5e0000
#define DUET_FBIOS_ADDR     0x9f5f0000
#define DUET_SBIOS_SIZE     0x10000
#define DUET_FBIOS_SIZE     0x20000

#define TYPE_DUET_MACHINE MACHINE_TYPE_NAME("imx6sl-duet")
OBJECT_DECLARE_SIMPLE_TYPE(DuetMachineState, DUET_MACHINE)

struct DuetMachineState {
    MachineState parent_obj;
    KindleIMX6SLIdme idme;
    char *falcon_bios;
    char *storage_bios;
    FslIMX6State *soc;
    struct arm_boot_info boot_info;
};

static void duet_load_bios(DeviceState *card, const char *path,
                           const char *name, hwaddr address,
                           uint64_t offset, size_t limit)
{
    g_autofree char *contents = NULL;
    gsize size;

    if (!g_file_get_contents(path, &contents, &size, NULL) || size > limit) {
        error_report("Unable to load Duet %s '%s'", name, path);
        exit(EXIT_FAILURE);
    }
    emmc_boot_partition_write(card, 2, offset, (uint8_t *)contents,
                              size, &error_fatal);
    rom_add_blob_fixed(name, contents, size, address);
}

static void duet_attach_emmc(DuetMachineState *dms)
{
    DriveInfo *di = drive_get(IF_SD, 0, 1);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus = qdev_get_child_bus(DEVICE(&dms->soc->usdhc[1]), "sd-bus");
    DeviceState *card = qdev_new(TYPE_EMMC);

    if (!dms->storage_bios || !dms->falcon_bios) {
        error_report("Duet requires storage-bios and falcon-bios");
        exit(EXIT_FAILURE);
    }

    qdev_prop_set_uint64(card, "boot-partition-size", 2 * MiB);
    qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);
    kindle_imx6sl_populate_idme(card, &dms->idme);

    /* Falcon's storage and main BIOS occupy eMMC boot partition 2. */
    duet_load_bios(card, dms->storage_bios, "duet.sbios",
                   DUET_SBIOS_ADDR, 0, DUET_SBIOS_SIZE);
    duet_load_bios(card, dms->falcon_bios, "duet.fbios",
                   DUET_FBIOS_ADDR, DUET_SBIOS_SIZE, DUET_FBIOS_SIZE);
    object_unref(OBJECT(card));
}

static void duet_firmware_reset(void *opaque)
{
    DuetMachineState *dms = opaque;
    ARMCPU *cpu = &dms->soc->cpu[0];

    cpu_reset(CPU(cpu));
    if (dms->boot_info.entry >= DUET_RAM_BASE) {
        cpu->env.regs[13] = DUET_IRAM_STACK;
    }
    cpu_set_pc(CPU(cpu), dms->boot_info.entry);
}

static void duet_attach_wifi(FslIMX6State *s)
{
    BusState *bus = qdev_get_child_bus(DEVICE(&s->usdhc[2]), "sd-bus");
    DeviceState *wifi = qdev_new(TYPE_BCM4343W_SDIO);

    qdev_realize(wifi, bus, &error_fatal);
    /* Whisky's WL_REG_ON drives the Broadcom module on USDHC3. */
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 15,
                          qdev_get_gpio_in_named(wifi, "power", 0));
    qdev_connect_gpio_out_named(wifi, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&s->usdhc[2]), "sdio-irq", 0));
    qdev_connect_gpio_out_named(wifi, "oob-irq", 0,
                                qdev_get_gpio_in(DEVICE(&s->gpio[2]), 31));
    object_unref(OBJECT(wifi));
}

static void duet_init(MachineState *machine)
{
    DuetMachineState *dms = DUET_MACHINE(machine);
    FslIMX6State *s;
    DeviceState *pmic;
    DeviceState *touch;
    DeviceState *epdc;
    DeviceState *mmdc;
    DeviceState *iomuxc;
    DeviceState *pxp;
    DeviceState *keyboard;
    /* EC3 selects Whisky's native portrait EN060TC1-1CE panel. */
    static const uint8_t panel_barcode[] = { 0xe9, 0xe7, 0x03 };

    if (machine->ram_size > DUET_RAM_MAX) {
        error_report("RAM size " RAM_ADDR_FMT " exceeds i.MX6SL maximum",
                     machine->ram_size);
        exit(EXIT_FAILURE);
    }

    dms->boot_info = (struct arm_boot_info) {
        .loader_start = DUET_RAM_BASE,
        .board_id = DUET_MACHINE_ID,
        .ram_size = machine->ram_size,
        .write_extra_atags = kindle_imx6sl_write_extra_atags,
        .write_extra_atags_opaque = &dms->idme,
    };

    s = dms->soc = FSL_IMX6(object_new(TYPE_FSL_IMX6));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(s), "sololite", true, &error_fatal);
    /* Stock Duet Linux owns the GIC directly. */
    object_property_set_bool(OBJECT(s), "has-el3", false, &error_fatal);
    object_property_set_uint(OBJECT(&s->src), "sbmr1", 0x60, &error_fatal);
    object_property_set_uint(OBJECT(s), "fec-phy-num", 0, &error_fatal);
    /* Hall sensor: cover open. Page switches: released (active low). */
    object_property_set_uint(OBJECT(&s->gpio[3]), "reset-psr",
                             BIT(7), &error_fatal);
    object_property_set_uint(OBJECT(&s->gpio[4]), "reset-psr",
                             BIT(11) | BIT(14), &error_fatal);
    qdev_realize(DEVICE(s), NULL, &error_fatal);

    memory_region_add_subregion(get_system_memory(), DUET_RAM_BASE,
                                machine->ram);
    duet_attach_emmc(dms);
    duet_attach_wifi(s);
    kindle_imx6sl_attach_panel_flash(s, panel_barcode);

    i2c_slave_create_simple(s->i2c[0].bus, TYPE_MAX77696, 0x34);
    i2c_slave_create_simple(s->i2c[0].bus, TYPE_MAX77696, 0x35);
    pmic = qdev_new(TYPE_MAX77696);
    qdev_prop_set_uint8(pmic, "address", 0x3c);
    qdev_connect_gpio_out(pmic, 0,
                          qdev_get_gpio_in(DEVICE(&s->gpio[3]), 20));
    /* EPD power-good has its own rising-edge signal on GPIO2_13. */
    qdev_connect_gpio_out(pmic, 1,
                          qdev_get_gpio_in(DEVICE(&s->gpio[1]), 13));
    qdev_realize_and_unref(pmic, BUS(s->i2c[0].bus), &error_fatal);
    i2c_slave_create_simple(s->i2c[0].bus, TYPE_MAX77696, 0x68);

    touch = qdev_new(TYPE_CYTTSP4);
    qdev_prop_set_uint8(touch, "address", 0x24);
    qdev_prop_set_uint16(touch, "x-resolution", DUET_FB_WIDTH);
    qdev_prop_set_uint16(touch, "y-resolution", DUET_FB_HEIGHT);
    /* Whisky's stock touch driver flips the native panel Y axis. */
    qdev_prop_set_bit(touch, "invert-y", true);
    qdev_realize(touch, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out(touch, 0,
                          qdev_get_gpio_in(DEVICE(&s->gpio[3]), 3));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 5,
                          qdev_get_gpio_in(touch, 0));
    object_unref(OBJECT(touch));

    epdc = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(machine), "epdc", OBJECT(epdc));
    qdev_prop_set_uint64(epdc, "fb-addr", DUET_FB_ADDR);
    qdev_prop_set_uint32(epdc, "fb-width", DUET_FB_WIDTH);
    qdev_prop_set_uint32(epdc, "fb-height", DUET_FB_HEIGHT);
    qdev_prop_set_uint32(epdc, "fb-stride", DUET_FB_STRIDE);
    qdev_prop_set_uint8(epdc, "fb-bpp", 1);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(epdc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(epdc), 0, DUET_EPDC_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(epdc), 0,
                       qdev_get_gpio_in(DEVICE(&s->a9mpcore), DUET_EPDC_GIC_IRQ));

    mmdc = qdev_new(TYPE_IMX6SL_MMDC);
    object_property_add_child(OBJECT(machine), "mmdc", OBJECT(mmdc));
    qdev_prop_set_uint64(mmdc, "ram-size", machine->ram_size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(mmdc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mmdc), 0, 0x021b0000);

    iomuxc = qdev_new(TYPE_IMX6SL_IOMUXC);
    object_property_add_child(OBJECT(machine), "iomuxc", OBJECT(iomuxc));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(iomuxc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(iomuxc), 0, 0x020e0000);

    pxp = qdev_new(TYPE_IMX6SL_PXP);
    object_property_add_child(OBJECT(machine), "pxp", OBJECT(pxp));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pxp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pxp), 0, DUET_PXP_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(pxp), 0,
                       qdev_get_gpio_in(DEVICE(&s->a9mpcore), DUET_PXP_GIC_IRQ));

    keyboard = qdev_new(TYPE_WARIO_KEYBOARD);
    qdev_prop_set_bit(keyboard, "page-buttons", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(keyboard), &error_fatal);
    qdev_connect_gpio_out_named(keyboard, "power-button", 0,
        qdev_get_gpio_in_named(pmic, "power-button", 0));
    qdev_connect_gpio_out_named(keyboard, "page-button", 0,
                                qdev_get_gpio_in(DEVICE(&s->gpio[4]), 14));
    qdev_connect_gpio_out_named(keyboard, "page-button", 1,
                                qdev_get_gpio_in(DEVICE(&s->gpio[4]), 11));
    sysbus_mmio_map(SYS_BUS_DEVICE(keyboard), 0, 0x020fc000);
    create_unimplemented_device("imx6sl.dcp-reserved", 0x020fd000, 0x3000);

    dms->boot_info.entry = kindle_imx6sl_load_firmware(machine);
    if (machine->firmware) {
        qemu_register_reset(duet_firmware_reset, dms);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&s->cpu[0], machine, &dms->boot_info);
    }
}

#define DUET_STRING_PROPERTY(_name, _member)                              \
    static char *duet_get_##_name(Object *obj, Error **errp)               \
    {                                                                    \
        return g_strdup(DUET_MACHINE(obj)->_member);                     \
    }                                                                    \
    static void duet_set_##_name(Object *obj, const char *value,          \
                                 Error **errp)                           \
    {                                                                    \
        DuetMachineState *dms = DUET_MACHINE(obj);                        \
        g_free(dms->_member);                                            \
        dms->_member = g_strdup(value);                                  \
    }

DUET_STRING_PROPERTY(idme_serial, idme.serial)
DUET_STRING_PROPERTY(idme_mac, idme.mac)
DUET_STRING_PROPERTY(idme_mfg, idme.mfg)
DUET_STRING_PROPERTY(idme_pcbsn, idme.pcbsn)
DUET_STRING_PROPERTY(idme_bootmode, idme.bootmode)
DUET_STRING_PROPERTY(idme_postmode, idme.postmode)
DUET_STRING_PROPERTY(falcon_bios, falcon_bios)
DUET_STRING_PROPERTY(storage_bios, storage_bios)

static void duet_machine_instance_init(Object *obj)
{
    DuetMachineState *dms = DUET_MACHINE(obj);

    dms->idme.serial = g_strdup("G0B0GC0000000001");
    dms->idme.mac = g_strdup("020000000001");
    dms->idme.mfg = g_strdup("00000000000000000000");
    dms->idme.pcbsn = g_strdup("0790700000000001");
    dms->idme.bootmode = g_strdup("main");
    dms->idme.postmode = g_strdup("normal");
}

static void duet_machine_instance_finalize(Object *obj)
{
    DuetMachineState *dms = DUET_MACHINE(obj);

    g_free(dms->idme.serial);
    g_free(dms->idme.mac);
    g_free(dms->idme.mfg);
    g_free(dms->idme.pcbsn);
    g_free(dms->idme.bootmode);
    g_free(dms->idme.postmode);
    g_free(dms->falcon_bios);
    g_free(dms->storage_bios);
}

static void duet_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Kindle Oasis 1 / Duet (i.MX6SoloLite, Cortex-A9, EPDC)";
    mc->init = duet_init;
    mc->max_cpus = 1;
    mc->default_cpus = 1;
    mc->minimum_page_bits = 12;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "duet.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;

    object_class_property_add_str(oc, "idme-serial", duet_get_idme_serial,
                                  duet_set_idme_serial);
    object_class_property_add_str(oc, "idme-mac", duet_get_idme_mac,
                                  duet_set_idme_mac);
    object_class_property_add_str(oc, "idme-mfg", duet_get_idme_mfg,
                                  duet_set_idme_mfg);
    object_class_property_add_str(oc, "idme-pcbsn", duet_get_idme_pcbsn,
                                  duet_set_idme_pcbsn);
    object_class_property_add_str(oc, "idme-bootmode", duet_get_idme_bootmode,
                                  duet_set_idme_bootmode);
    object_class_property_add_str(oc, "idme-postmode", duet_get_idme_postmode,
                                  duet_set_idme_postmode);
    object_class_property_add_str(oc, "falcon-bios", duet_get_falcon_bios,
                                  duet_set_falcon_bios);
    object_class_property_add_str(oc, "storage-bios", duet_get_storage_bios,
                                  duet_set_storage_bios);
}

static const TypeInfo duet_machine_type = {
    .name = TYPE_DUET_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(DuetMachineState),
    .instance_init = duet_machine_instance_init,
    .instance_finalize = duet_machine_instance_finalize,
    .class_init = duet_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void duet_machine_register_types(void)
{
    type_register_static(&duet_machine_type);
}

type_init(duet_machine_register_types)
