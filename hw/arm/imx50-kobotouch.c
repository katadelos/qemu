/*
 * Kobo Touch (Trilogy) board emulation.
 *
 * The first-generation Kobo Touch stores U-Boot, its environment, the
 * Netronix hardware configuration, kernel, waveform and filesystems on an
 * internal SD card.  The ROM-loaded U-Boot image is supplied separately with
 * -bios while the reconstructed, partitioned card is attached to eSDHC1.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/arm/fsl-imx50.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/zforce.h"
#include "hw/sd/sd.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/qtest.h"
#include "system/reset.h"

#define KOBOTOUCH_RAM_BASE       0x70000000
#define KOBOTOUCH_RAM_MAX        (512 * MiB)
#define KOBOTOUCH_MACHINE_ID     2988 /* MACH_TYPE_MX50_RDP */
#define KOBOTOUCH_UBOOT_ADDR     0x77800000
#define KOBOTOUCH_UBOOT_ENTRY    0x77800a20
#define KOBOTOUCH_UBOOT_MAX      0x000c0000

#define TYPE_KOBOTOUCH_MACHINE MACHINE_TYPE_NAME("imx50-kobotouch")
OBJECT_DECLARE_SIMPLE_TYPE(KoboTouchMachineState, KOBOTOUCH_MACHINE)

struct KoboTouchMachineState {
    MachineState parent_obj;
};

static struct arm_boot_info kobotouch_binfo;

static void kobotouch_firmware_reset(void *opaque)
{
    ARMCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
    cpu_set_pc(CPU(cpu), KOBOTOUCH_UBOOT_ENTRY);
}

static void kobotouch_load_firmware(MachineState *machine)
{
    ssize_t size;

    if (!machine->firmware) {
        return;
    }
    size = load_image_targphys(machine->firmware, KOBOTOUCH_UBOOT_ADDR,
                               KOBOTOUCH_UBOOT_MAX, NULL);
    if (size < 0) {
        error_report("Unable to load Kobo Touch firmware '%s'",
                     machine->firmware);
        exit(EXIT_FAILURE);
    }
    kobotouch_binfo.entry = KOBOTOUCH_UBOOT_ENTRY;
}

static void kobotouch_attach_sd(FslIMX50State *soc)
{
    DriveInfo *di = drive_get(IF_SD, 0, 0);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus;
    DeviceState *card;

    if (!blk) {
        error_report("Kobo Touch requires its internal SD image");
        exit(EXIT_FAILURE);
    }

    bus = qdev_get_child_bus(DEVICE(&soc->esdhc[0]), "sd-bus");
    card = qdev_new(TYPE_SD_CARD);
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);
    object_unref(OBJECT(card));
}

static void kobotouch_create_peripherals(FslIMX50State *soc)
{
    I2CSlave *zforce;
    I2CSlave *tps65185;
    I2CSlave *msp430;

    /*
     * The production Trilogy kernel's direct TPS65185 driver uses I2C2,
     * while zForce remains on the board-info I2C1 bus.
     */
    tps65185 = i2c_slave_create_simple(soc->i2c[1].bus,
                                       TYPE_TPS65185, 0x68);
    qdev_connect_gpio_out_named(DEVICE(tps65185), "pwrgood", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[2]), 28));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[2]), 29,
        qdev_get_gpio_in_named(DEVICE(tps65185), "powerup", 0));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[2]), 30,
        qdev_get_gpio_in_named(DEVICE(tps65185), "power-enable", 0));

    zforce = i2c_slave_create_simple(soc->i2c[0].bus, TYPE_ZFORCE, 0x50);
    qdev_connect_gpio_out(DEVICE(zforce), 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[5]), 11));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[5]), 10,
        qdev_get_gpio_in(DEVICE(zforce), 0));

    /* The Netronix board controller is a 16-bit register device on I2C3. */
    msp430 = i2c_slave_create_simple(soc->i2c[2].bus,
                                     TYPE_KOBOTOUCH_MSP430, 0x43);
    (void)msp430;

}

static void kobotouch_init(MachineState *machine)
{
    FslIMX50State *soc;

    if (machine->ram_size > KOBOTOUCH_RAM_MAX) {
        error_report("RAM size exceeds Kobo Touch's 512 MiB addressable SDRAM");
        exit(EXIT_FAILURE);
    }

    soc = FSL_IMX50(object_new(TYPE_FSL_IMX50));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    /* Trilogy stores RGB888 pixels in fb0's 32-bit XRGB container. */
    object_property_set_bool(OBJECT(soc), "pxp-rgb888-xrgb32", true,
                             &error_fatal);
    /* PxP already rotates Kobo's portrait framebuffer into panel scan order. */
    object_property_set_bool(OBJECT(&soc->epdc), "rotate-ccw", false,
                             &error_fatal);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);
    kobotouch_create_peripherals(soc);
    memory_region_add_subregion(get_system_memory(), KOBOTOUCH_RAM_BASE,
                                machine->ram);

    /*
     * The Trilogy environment boots /dev/mmcblk0p1.  Its Netronix SD-number
     * straps read zero, making eSDHC1 the internal card and Linux mmcblk0.
     */
    kobotouch_attach_sd(soc);

    kobotouch_binfo = (struct arm_boot_info) {
        .loader_start = KOBOTOUCH_RAM_BASE,
        .ram_size = machine->ram_size,
        .board_id = KOBOTOUCH_MACHINE_ID,
    };
    kobotouch_load_firmware(machine);
    if (machine->firmware) {
        qemu_register_reset(kobotouch_firmware_reset, &soc->cpu);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&soc->cpu, machine, &kobotouch_binfo);
    }
}

static void kobotouch_machine_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Kobo Touch Trilogy (i.MX508)";
    mc->init = kobotouch_init;
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "kobotouch.ram";
    mc->auto_create_sdcard = true;
    mc->ignore_memory_transaction_failures = true;
}

static const TypeInfo kobotouch_machine_type = {
    .name = TYPE_KOBOTOUCH_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(KoboTouchMachineState),
    .class_init = kobotouch_machine_init,
    .interfaces = arm_machine_interfaces,
};

static void kobotouch_machine_register_types(void)
{
    type_register_static(&kobotouch_machine_type);
}

type_init(kobotouch_machine_register_types)
