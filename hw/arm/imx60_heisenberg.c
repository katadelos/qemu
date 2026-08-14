/*
 * Amazon/Lab126 Heisenberg (Eanab) board emulation
 *
 * The Lab126 source names the board imx60_heisenberg even though the SoC is
 * an i.MX6SoloLite (i.MX6SL), not an i.MX6SLL.
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
#include "hw/i2c/bd71815.h"
#include "hw/i2c/fp9928.h"
#include "hw/i2c/zforce.h"
#include "hw/input/wario-keyboard.h"
#include "hw/misc/unimp.h"
#include "hw/misc/imx6sl_iomuxc.h"
#include "hw/misc/imx6sl_mmdc.h"
#include "hw/misc/imx6sl_pxp.h"
#include "hw/sd/sd.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/reset.h"
#include "system/system.h"
#include "system/qtest.h"

#define HEISENBERG_RAM_BASE       0x80000000
#define HEISENBERG_RAM_MAX        (2 * GiB)
#define HEISENBERG_UBOOT_ADDR     0x00981000
#define HEISENBERG_UBOOT_MAX      0x0007f000
#define HEISENBERG_MACHINE_ID     4307
#define HEISENBERG_EPDC_ADDR      0x020f4000
#define HEISENBERG_EPDC_GIC_IRQ   97
#define HEISENBERG_PXP_ADDR       0x020f0000
#define HEISENBERG_PXP_GIC_IRQ    98
#define HEISENBERG_FB_ADDR        0x9c100000
#define HEISENBERG_FB_WIDTH       600
#define HEISENBERG_FB_HEIGHT      800
#define HEISENBERG_FB_STRIDE      608
#define HEISENBERG_IVT_OFFSET     0x400
#define HEISENBERG_IVT_ENTRY_OFF  0x04
#define HEISENBERG_IDME_BASE      0x80000

#define TYPE_HEISENBERG_MACHINE MACHINE_TYPE_NAME("imx6sl-eanab")
OBJECT_DECLARE_SIMPLE_TYPE(HeisenbergMachineState, HEISENBERG_MACHINE)

struct HeisenbergMachineState {
    MachineState parent_obj;
    char *idme_serial;
    char *idme_mac;
    char *idme_mfg;
    char *idme_pcbsn;
    char *idme_bootmode;
    char *idme_postmode;
};

typedef struct HeisenbergIdmeField {
    const char *name;
    char *value;
    uint32_t offset;
    size_t size;
} HeisenbergIdmeField;

static struct arm_boot_info heisenberg_boot_info;

static void heisenberg_firmware_reset(void *opaque)
{
    ARMCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
    cpu_set_pc(CPU(cpu), heisenberg_boot_info.entry);
}

static void heisenberg_populate_idme(DeviceState *card,
                                     HeisenbergMachineState *hms)
{
    HeisenbergIdmeField fields[] = {
        { "serial", hms->idme_serial, 0x0000, 16 },
        { "mac", hms->idme_mac, 0x0030, 12 },
        { "mfg", hms->idme_mfg, 0x0040, 20 },
        { "pcbsn", hms->idme_pcbsn, 0x0060, 16 },
        { "bootmode", hms->idme_bootmode, 0x1000, 16 },
        { "postmode", hms->idme_postmode, 0x1010, 16 },
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(fields); i++) {
        HeisenbergIdmeField *field = &fields[i];
        g_autofree uint8_t *contents = g_malloc0(field->size);
        size_t len = strlen(field->value);

        if (!g_str_is_ascii(field->value) || len > field->size) {
            error_report("IDME %s must contain at most %zu ASCII bytes",
                         field->name, field->size);
            exit(EXIT_FAILURE);
        }
        memcpy(contents, field->value, len);
        emmc_boot_partition_write(card, 1,
                                  HEISENBERG_IDME_BASE + field->offset,
                                  contents, field->size, &error_fatal);
    }
}

static void heisenberg_attach_emmc(FslIMX6State *s,
                                   HeisenbergMachineState *hms)
{
    DriveInfo *di = drive_get(IF_SD, 0, 1);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus;
    DeviceState *card;

    /* The Heisenberg DT wires its 8-bit, non-removable eMMC to USDHC2. */
    bus = qdev_get_child_bus(DEVICE(&s->usdhc[1]), "sd-bus");
    card = qdev_new(TYPE_EMMC);
    qdev_prop_set_uint64(card, "boot-partition-size", 2 * MiB);
    qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);
    heisenberg_populate_idme(card, hms);
    object_unref(OBJECT(card));
}

static void heisenberg_attach_wifi(FslIMX6State *s)
{
    BusState *bus;
    DeviceState *wifi;

    /* Heisenberg uses the BCM43430 function on USDHC1. */
    bus = qdev_get_child_bus(DEVICE(&s->usdhc[0]), "sd-bus");
    wifi = qdev_new(TYPE_BCM43430_SDIO);
    qdev_realize(wifi, bus, &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 15,
                          qdev_get_gpio_in_named(wifi, "power", 0));
    qdev_connect_gpio_out_named(
        wifi, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&s->usdhc[0]), "sdio-irq", 0));
    object_unref(OBJECT(wifi));
}

static void heisenberg_load_firmware(MachineState *machine)
{
    g_autofree uint8_t *image = NULL;
    gsize image_size;
    uint32_t entry;
    ssize_t size;

    if (!machine->firmware) {
        return;
    }

    /*
     * Unlike the older Wario images, Heisenberg's IVT self pointer is
     * 0x00981400 while the IVT remains at file offset 0x400.  The i.MX boot
     * ROM therefore places byte zero at 0x00981000; loading it at Wario's
     * 0x00980000 base makes the 0x00982000 entry land 0x1000 bytes late.
     */
    size = load_image_targphys(machine->firmware, HEISENBERG_UBOOT_ADDR,
                               HEISENBERG_UBOOT_MAX, NULL);
    if (size < 0) {
        error_report("Unable to load Heisenberg firmware '%s'",
                     machine->firmware);
        exit(EXIT_FAILURE);
    }

    if (!g_file_get_contents(machine->firmware, (char **)&image,
                             &image_size, NULL) ||
        image_size < HEISENBERG_IVT_OFFSET + HEISENBERG_IVT_ENTRY_OFF +
                     sizeof(entry)) {
        error_report("Heisenberg firmware has no readable IVT");
        exit(EXIT_FAILURE);
    }
    entry = ldl_le_p(image + HEISENBERG_IVT_OFFSET +
                     HEISENBERG_IVT_ENTRY_OFF);
    if (entry < HEISENBERG_UBOOT_ADDR ||
        entry >= HEISENBERG_UBOOT_ADDR + size) {
        error_report("Heisenberg firmware IVT entry 0x%08x is outside image",
                     entry);
        exit(EXIT_FAILURE);
    }
    heisenberg_boot_info.entry = entry;
}

static void heisenberg_init(MachineState *machine)
{
    HeisenbergMachineState *hms = HEISENBERG_MACHINE(machine);
    FslIMX6State *s;
    DeviceState *epdc;
    DeviceState *mmdc;
    DeviceState *iomuxc;
    DeviceState *pxp;
    DeviceState *keyboard;
    DeviceState *pmic;
    DeviceState *touch;

    if (machine->ram_size > HEISENBERG_RAM_MAX) {
        error_report("RAM size " RAM_ADDR_FMT " exceeds i.MX6SL maximum",
                     machine->ram_size);
        exit(EXIT_FAILURE);
    }

    heisenberg_boot_info = (struct arm_boot_info) {
        .loader_start = HEISENBERG_RAM_BASE,
        .board_id = HEISENBERG_MACHINE_ID,
        .ram_size = machine->ram_size,
    };

    s = FSL_IMX6(object_new(TYPE_FSL_IMX6));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(s), "sololite", true, &error_fatal);
    object_property_set_bool(OBJECT(s), "has-el3", false, &error_fatal);
    object_property_set_uint(OBJECT(&s->src), "sbmr1", 0x60,
                             &error_fatal);
    object_property_set_uint(OBJECT(s), "fec-phy-num", 0,
                             &error_fatal);
    /* GPIO4_16 is the active-low hall sensor; an open cover reads high. */
    object_property_set_uint(OBJECT(&s->gpio[3]), "reset-psr", BIT(16),
                             &error_fatal);
    /*
     * Heisenberg's 2014.04 USDHC driver waits for transfer completion before
     * acknowledging command completion.  Run USDHC2's SDMA phase without the
     * command-ack deferral used by newer Lab126 kernels.
     */
    object_property_set_bool(OBJECT(&s->usdhc[1]), "defer-data-transfer",
                             false, &error_fatal);
    qdev_realize(DEVICE(s), NULL, &error_fatal);

    memory_region_add_subregion(get_system_memory(), HEISENBERG_RAM_BASE,
                                machine->ram);

    heisenberg_attach_emmc(s, hms);
    heisenberg_attach_wifi(s);

    /* BD71815 and the FP9928 panel supply are on I2C1. */
    pmic = qdev_new(TYPE_BD71815);
    qdev_prop_set_uint8(pmic, "address", 0x4b);
    qdev_realize(pmic, BUS(s->i2c[0].bus), &error_fatal);
    qdev_connect_gpio_out_named(pmic, "irq", 0,
                                qdev_get_gpio_in(DEVICE(&s->gpio[3]), 20));
    object_unref(OBJECT(pmic));
    i2c_slave_create_simple(s->i2c[0].bus, TYPE_FP9928, 0x48);

    /* Eanab zForce2 TI protocol: I2C2 address 0x51 and GPIO4 controls. */
    touch = qdev_new(TYPE_KINDLE_ZFORCE2_TI);
    qdev_prop_set_uint8(touch, "address", 0x51);
    qdev_realize(touch, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out(touch, 0,
                          qdev_get_gpio_in(DEVICE(&s->gpio[3]), 3));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 5,
                          qdev_get_gpio_in(touch, 0));
    object_unref(OBJECT(touch));

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
    sysbus_mmio_map(SYS_BUS_DEVICE(pxp), 0, HEISENBERG_PXP_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(pxp), 0,
                       qdev_get_gpio_in(DEVICE(&s->a9mpcore),
                                       HEISENBERG_PXP_GIC_IRQ));

    /*
     * The stock 512 MiB kernel reserves CMA at 0x9c000000 and allocates its
     * 6 MiB EPDC framebuffer at 0x9c100000.  X rotates the native 800x600
     * panel into a 600x800, 8-bit surface with 608-byte scanlines.  Periodic
     * scanout keeps Cocoa synchronized with mmap writes between update
     * ioctls, as on the other Lab126 machines.
     */
    epdc = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(machine), "epdc", OBJECT(epdc));
    object_property_set_link(OBJECT(epdc), "pxp", OBJECT(pxp), &error_fatal);
    qdev_prop_set_uint64(epdc, "fb-addr", HEISENBERG_FB_ADDR);
    qdev_prop_set_uint32(epdc, "fb-width", HEISENBERG_FB_WIDTH);
    qdev_prop_set_uint32(epdc, "fb-height", HEISENBERG_FB_HEIGHT);
    qdev_prop_set_uint32(epdc, "fb-stride", HEISENBERG_FB_STRIDE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(epdc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(epdc), 0, HEISENBERG_EPDC_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(epdc), 0,
                       qdev_get_gpio_in(DEVICE(&s->a9mpcore),
                                       HEISENBERG_EPDC_GIC_IRQ));

    keyboard = qdev_new(TYPE_WARIO_KEYBOARD);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(keyboard), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(keyboard), 0, 0x020fc000);
    create_unimplemented_device("imx6sl.dcp-reserved", 0x020fd000, 0x3000);

    heisenberg_load_firmware(machine);
    if (machine->firmware) {
        qemu_register_reset(heisenberg_firmware_reset, &s->cpu[0]);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&s->cpu[0], machine, &heisenberg_boot_info);
    }
}

static char *heisenberg_idme_get(char **value)
{
    return g_strdup(*value);
}

static void heisenberg_idme_set(char **field, const char *value)
{
    g_free(*field);
    *field = g_strdup(value);
}

#define HEISENBERG_IDME_PROPERTY(_member)                              \
    static char *heisenberg_get_##_member(Object *obj, Error **errp)   \
    {                                                                  \
        return heisenberg_idme_get(                                    \
            &HEISENBERG_MACHINE(obj)->_member);                         \
    }                                                                  \
    static void heisenberg_set_##_member(Object *obj,                  \
                                          const char *value,            \
                                          Error **errp)                 \
    {                                                                  \
        heisenberg_idme_set(&HEISENBERG_MACHINE(obj)->_member, value);  \
    }

HEISENBERG_IDME_PROPERTY(idme_serial)
HEISENBERG_IDME_PROPERTY(idme_mac)
HEISENBERG_IDME_PROPERTY(idme_mfg)
HEISENBERG_IDME_PROPERTY(idme_pcbsn)
HEISENBERG_IDME_PROPERTY(idme_bootmode)
HEISENBERG_IDME_PROPERTY(idme_postmode)

static void heisenberg_machine_instance_init(Object *obj)
{
    HeisenbergMachineState *hms = HEISENBERG_MACHINE(obj);

    hms->idme_serial = g_strdup("G000000000000001");
    hms->idme_mac = g_strdup("020000000006");
    hms->idme_mfg = g_strdup("00000000000000000000");
    hms->idme_pcbsn = g_strdup("0001180400000001");
    hms->idme_bootmode = g_strdup("main");
    hms->idme_postmode = g_strdup("normal");
}

static void heisenberg_machine_instance_finalize(Object *obj)
{
    HeisenbergMachineState *hms = HEISENBERG_MACHINE(obj);

    g_free(hms->idme_serial);
    g_free(hms->idme_mac);
    g_free(hms->idme_mfg);
    g_free(hms->idme_pcbsn);
    g_free(hms->idme_bootmode);
    g_free(hms->idme_postmode);
}

static void heisenberg_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Lab126 Eanab / Kindle Basic 2 (i.MX6SL, Heisenberg)";
    mc->init = heisenberg_init;
    mc->max_cpus = 1;
    mc->default_cpus = 1;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "heisenberg.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;

    object_class_property_add_str(oc, "idme-serial",
                                  heisenberg_get_idme_serial,
                                  heisenberg_set_idme_serial);
    object_class_property_add_str(oc, "idme-mac", heisenberg_get_idme_mac,
                                  heisenberg_set_idme_mac);
    object_class_property_add_str(oc, "idme-mfg", heisenberg_get_idme_mfg,
                                  heisenberg_set_idme_mfg);
    object_class_property_add_str(oc, "idme-pcbsn",
                                  heisenberg_get_idme_pcbsn,
                                  heisenberg_set_idme_pcbsn);
    object_class_property_add_str(oc, "idme-bootmode",
                                  heisenberg_get_idme_bootmode,
                                  heisenberg_set_idme_bootmode);
    object_class_property_add_str(oc, "idme-postmode",
                                  heisenberg_get_idme_postmode,
                                  heisenberg_set_idme_postmode);
}

static const TypeInfo heisenberg_machine_type = {
    .name = TYPE_HEISENBERG_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(HeisenbergMachineState),
    .instance_init = heisenberg_machine_instance_init,
    .instance_finalize = heisenberg_machine_instance_finalize,
    .class_init = heisenberg_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void heisenberg_machine_register_types(void)
{
    type_register_static(&heisenberg_machine_type);
}
type_init(heisenberg_machine_register_types)
