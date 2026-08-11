/*
 * Kobo Touch (Trilogy) and Kobo Mini board emulation.
 *
 * The first-generation Kobo Touch stores U-Boot, its environment, the
 * Netronix hardware configuration, kernel, waveform and filesystems on an
 * internal SD card.  The ROM-loaded U-Boot image is supplied separately with
 * -bios while the reconstructed, partitioned card is attached to the board's
 * strapped eSDHC controller.
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
#include "hw/ssi/mc13892.h"
#include "hw/ssi/ssi.h"
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
#define TYPE_KOBOMINI_MACHINE MACHINE_TYPE_NAME("imx50-kobomini")
OBJECT_DECLARE_SIMPLE_TYPE(KoboTouchMachineState, KOBOTOUCH_MACHINE)

struct KoboTouchMachineState {
    MachineState parent_obj;
    bool mini;
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
        error_report("Unable to load Kobo firmware '%s'",
                     machine->firmware);
        exit(EXIT_FAILURE);
    }
    kobotouch_binfo.entry = KOBOTOUCH_UBOOT_ENTRY;
}

static void kobotouch_attach_sd(FslIMX50State *soc,
                                KoboTouchMachineState *tms)
{
    DriveInfo *di = drive_get(IF_SD, 0, 0);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus;
    DeviceState *card;

    if (!blk) {
        error_report("Kobo machine requires its internal SD image");
        exit(EXIT_FAILURE);
    }

    bus = qdev_get_child_bus(DEVICE(&soc->esdhc[tms->mini ? 2 : 0]),
                             "sd-bus");
    card = qdev_new(TYPE_SD_CARD);
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);
    object_unref(OBJECT(card));
}

static void kobomini_attach_pmic(FslIMX50State *soc)
{
    SSIBus *bus;
    DeviceState *pmic;

    bus = (SSIBus *)qdev_get_child_bus(DEVICE(&soc->spi[2]), "spi");
    pmic = qdev_new(TYPE_MC13892);
    ssi_realize_and_unref(pmic, bus, &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(&soc->spi[2]), 1,
        qemu_irq_invert(qdev_get_gpio_in_named(pmic, SSI_GPIO_CS, 0)));
    qdev_connect_gpio_out_named(
        pmic, "irq", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[5]), 8));
}

static void kobotouch_create_peripherals(FslIMX50State *soc,
                                         KoboTouchMachineState *tms)
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
    if (tms->mini) {
        qdev_connect_gpio_out(DEVICE(zforce), 0,
            qdev_get_gpio_in(DEVICE(&soc->gpio[4]), 15));
        qdev_connect_gpio_out(DEVICE(&soc->gpio[4]), 26,
            qdev_get_gpio_in(DEVICE(zforce), 0));
    } else {
        qdev_connect_gpio_out(DEVICE(zforce), 0,
            qdev_get_gpio_in(DEVICE(&soc->gpio[5]), 11));
        qdev_connect_gpio_out(DEVICE(&soc->gpio[5]), 10,
            qdev_get_gpio_in(DEVICE(zforce), 0));
    }

    /* The Netronix board controller is a 16-bit register device on I2C3. */
    msp430 = i2c_slave_create_simple(soc->i2c[2].bus,
                                     TYPE_KOBOTOUCH_MSP430, 0x43);
    (void)msp430;

}

static void kobotouch_init(MachineState *machine)
{
    KoboTouchMachineState *tms = KOBOTOUCH_MACHINE(machine);
    FslIMX50State *soc;

    if (machine->ram_size > KOBOTOUCH_RAM_MAX) {
        error_report("RAM size exceeds Kobo's 512 MiB addressable SDRAM");
        exit(EXIT_FAILURE);
    }

    soc = FSL_IMX50(object_new(TYPE_FSL_IMX50));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    /* PxP already rotates Kobo's portrait framebuffer into panel scan order. */
    object_property_set_bool(OBJECT(&soc->epdc), "rotate-ccw", false,
                             &error_fatal);
    if (tms->mini) {
        /* SD boot with controller 2 selected for U-Boot's environment. */
        object_property_set_uint(OBJECT(soc), "src-sbmr", 0x00200040,
                                 &error_fatal);
        /* E50610 straps EIM_WAIT:EIM_EB1 to binary 2 (internal SD3). */
        qdev_prop_set_uint32(DEVICE(&soc->gpio[0]), "reset-psr", 1U << 21);
        /* TLE4913 reads high while the magnetic cover is open. */
        qdev_prop_set_uint32(DEVICE(&soc->gpio[4]), "reset-psr", 1U << 25);
    }
    qdev_realize(DEVICE(soc), NULL, &error_fatal);
    kobotouch_create_peripherals(soc, tms);
    if (tms->mini) {
        kobomini_attach_pmic(soc);
    }
    memory_region_add_subregion(get_system_memory(), KOBOTOUCH_RAM_BASE,
                                machine->ram);

    /* Trilogy uses eSDHC1; Mini straps eSDHC3. Both become Linux mmcblk0. */
    kobotouch_attach_sd(soc, tms);

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

static void kobomini_machine_instance_init(Object *obj)
{
    KOBOTOUCH_MACHINE(obj)->mini = true;
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

static void kobomini_machine_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Kobo Mini N705 (i.MX508)";
    mc->default_ram_id = "kobomini.ram";
}

static const TypeInfo kobotouch_machine_type = {
    .name = TYPE_KOBOTOUCH_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(KoboTouchMachineState),
    .class_init = kobotouch_machine_init,
    .interfaces = arm_machine_interfaces,
};

static const TypeInfo kobomini_machine_type = {
    .name = TYPE_KOBOMINI_MACHINE,
    .parent = TYPE_KOBOTOUCH_MACHINE,
    .instance_init = kobomini_machine_instance_init,
    .class_init = kobomini_machine_init,
};

static void kobotouch_machine_register_types(void)
{
    type_register_static(&kobotouch_machine_type);
    type_register_static(&kobomini_machine_type);
}

type_init(kobotouch_machine_register_types)
