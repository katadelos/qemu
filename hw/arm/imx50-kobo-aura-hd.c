/*
 * Kobo Aura HD N204B (Netronix E606C0, i.MX507) board.
 *
 * Wiring follows kobolabs/imx507-aurahd in linux-ereader and u-boot-ereader.
 * The internal card contains the Netronix hardware configuration, kernel,
 * waveform and filesystems.  -bios supplies the card's complete U-Boot image,
 * including its ROM plug-in and main IVTs.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
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
#include "hw/usb/imx-usb-misc.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/qtest.h"
#include "system/reset.h"
#include "ui/input.h"

#define AURA_HD_RAM_BASE       0x70000000
#define AURA_HD_RAM_SIZE       (512 * MiB)
#define AURA_HD_MACHINE_ID     2988 /* MACH_TYPE_MX50_RDP */
#define AURA_HD_IVT_OFFSET     0x400
#define AURA_HD_IVT_HEADER     0x402000d1

#define TYPE_AURA_HD_MACHINE MACHINE_TYPE_NAME("imx50-kobo-aura-hd")
OBJECT_DECLARE_SIMPLE_TYPE(AuraHDMachineState, AURA_HD_MACHINE)

struct AuraHDMachineState {
    MachineState parent_obj;
    FslIMX50State *soc;
    struct arm_boot_info boot;
};

#define TYPE_AURA_HD_BUTTONS "kobo-aura-hd-buttons"
OBJECT_DECLARE_SIMPLE_TYPE(AuraHDButtonsState, AURA_HD_BUTTONS)

struct AuraHDButtonsState {
    DeviceState parent_obj;
    QemuInputHandlerState *input;
    qemu_irq gpio[2];
    uint8_t pressed;
};

static void aurahd_buttons_update(AuraHDButtonsState *s)
{
    for (unsigned i = 0; i < ARRAY_SIZE(s->gpio); i++) {
        qemu_set_irq(s->gpio[i], !(s->pressed & (1U << i)));
    }
}

static void aurahd_buttons_event(DeviceState *dev, QemuConsole *src,
                                 InputEvent *evt)
{
    AuraHDButtonsState *s = AURA_HD_BUTTONS(dev);
    InputKeyEvent *key = evt->u.key.data;
    unsigned button;

    switch (qemu_input_key_value_to_qcode(key->key)) {
    case Q_KEY_CODE_POWER:
    case Q_KEY_CODE_P:
        button = 0;
        break;
    case Q_KEY_CODE_F:
        button = 1;
        break;
    default:
        return;
    }
    s->pressed = deposit32(s->pressed, button, 1, key->down);
    aurahd_buttons_update(s);
}

static const QemuInputHandler aurahd_buttons_handler = {
    .name = "Kobo Aura HD buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = aurahd_buttons_event,
};

static void aurahd_buttons_reset(DeviceState *dev)
{
    AuraHDButtonsState *s = AURA_HD_BUTTONS(dev);

    s->pressed = 0;
    aurahd_buttons_update(s);
}

static int aurahd_buttons_post_load(void *opaque, int version_id)
{
    aurahd_buttons_update(opaque);
    return 0;
}

static const VMStateDescription aurahd_buttons_vmstate = {
    .name = TYPE_AURA_HD_BUTTONS,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = aurahd_buttons_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(pressed, AuraHDButtonsState),
        VMSTATE_END_OF_LIST()
    },
};

static void aurahd_buttons_realize(DeviceState *dev, Error **errp)
{
    AURA_HD_BUTTONS(dev)->input = qemu_input_handler_register(
        dev, &aurahd_buttons_handler);
}

static void aurahd_buttons_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(AURA_HD_BUTTONS(dev)->input);
}

static void aurahd_buttons_init(Object *obj)
{
    qdev_init_gpio_out(DEVICE(obj), AURA_HD_BUTTONS(obj)->gpio, 2);
}

static void aurahd_buttons_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = aurahd_buttons_realize;
    dc->unrealize = aurahd_buttons_unrealize;
    dc->vmsd = &aurahd_buttons_vmstate;
    device_class_set_legacy_reset(dc, aurahd_buttons_reset);
}

static void aurahd_firmware_reset(void *opaque)
{
    AuraHDMachineState *s = opaque;
    ARMCPU *cpu = &s->soc->cpu;

    cpu_reset(CPU(cpu));
    cpu->env.regs[13] = FSL_IMX50_IRAM_ADDR + FSL_IMX50_IRAM_SIZE;
    cpu_set_pc(CPU(cpu), s->boot.entry);
}

static size_t aurahd_boot_data_offset(const uint8_t *image, size_t image_size,
                                    size_t ivt_offset)
{
    uint32_t self, boot_data;

    if (ivt_offset > image_size || image_size - ivt_offset < 0x20 ||
        ldl_le_p(image + ivt_offset) != AURA_HD_IVT_HEADER) {
        error_report("Aura HD firmware has an invalid IVT at 0x%zx",
                     ivt_offset);
        exit(EXIT_FAILURE);
    }
    self = ldl_le_p(image + ivt_offset + 0x14);
    boot_data = ldl_le_p(image + ivt_offset + 0x10);
    if (boot_data < self ||
        boot_data - self > image_size - ivt_offset - 12) {
        error_report("Aura HD firmware boot-data pointer is outside the image");
        exit(EXIT_FAILURE);
    }
    return ivt_offset + boot_data - self;
}

static void aurahd_load_firmware(AuraHDMachineState *s)
{
    MachineState *machine = MACHINE(s);
    g_autofree uint8_t *image = NULL;
    gsize image_size;
    size_t ivt_offset = AURA_HD_IVT_OFFSET;
    size_t boot_data;
    uint32_t self, entry, start, length;
    hwaddr ram_end = AURA_HD_RAM_BASE + machine->ram_size;

    if (!g_file_get_contents(machine->firmware, (char **)&image,
                             &image_size, NULL)) {
        error_report("Unable to read Aura HD firmware '%s'", machine->firmware);
        exit(EXIT_FAILURE);
    }

    boot_data = aurahd_boot_data_offset(image, image_size, ivt_offset);
    if (ldl_le_p(image + boot_data + 8) == 1) {
        /* The ROM plug-in programs physical DDR. Enter its following SDRAM
         * image directly, since the emulated SDRAM is already available. */
        ivt_offset = boot_data + 12;
        boot_data = aurahd_boot_data_offset(image, image_size, ivt_offset);
    }
    self = ldl_le_p(image + ivt_offset + 0x14);
    entry = ldl_le_p(image + ivt_offset + 4);
    start = ldl_le_p(image + boot_data);
    length = ldl_le_p(image + boot_data + 4);
    if (ldl_le_p(image + boot_data + 8) != 0 ||
        self < ivt_offset || start != self - ivt_offset ||
        start < AURA_HD_RAM_BASE || start >= ram_end ||
        image_size > ram_end - start || !length || length > image_size ||
        entry < start || entry - start >= length) {
        error_report("Aura HD firmware main IVT does not describe an SDRAM image");
        exit(EXIT_FAILURE);
    }
    if (load_image_targphys(machine->firmware, start, ram_end - start,
                            NULL) < 0) {
        error_report("Unable to load Aura HD firmware '%s'", machine->firmware);
        exit(EXIT_FAILURE);
    }
    s->boot.entry = entry;
}

static void aurahd_attach_card(FslIMX50State *soc, unsigned controller,
                               unsigned unit)
{
    DriveInfo *di = drive_get(IF_SD, 0, unit);
    DeviceState *card;

    if (!di) {
        if (!unit && !qtest_enabled()) {
            error_report("Aura HD requires its internal SD image at index 0");
            exit(EXIT_FAILURE);
        }
        return;
    }
    card = qdev_new(TYPE_SD_CARD);
    qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                            &error_fatal);
    qdev_realize_and_unref(card,
        qdev_get_child_bus(DEVICE(&soc->esdhc[controller]), "sd-bus"),
        &error_fatal);
}

static void aurahd_create_peripherals(FslIMX50State *soc)
{
    DeviceState *usb_misc = qdev_new(TYPE_IMX_USB_MISC);
    I2CSlave *touch;
    I2CSlave *tps65185;
    I2CSlave *msp430;
    DeviceState *pmic;
    DeviceState *wifi;
    DeviceState *buttons;

    sysbus_realize_and_unref(SYS_BUS_DEVICE(usb_misc), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(usb_misc), 0, 0x53f80800, 2);
    sysbus_connect_irq(SYS_BUS_DEVICE(usb_misc), 0,
        qdev_get_gpio_in_named(DEVICE(&soc->usb_otg), "wakeup", 0));
    qdev_connect_gpio_out_named(DEVICE(&soc->usb_otg), "resume", 0,
                                qdev_get_gpio_in(usb_misc, 0));
    tps65185 = i2c_slave_create_simple(soc->i2c[1].bus, TYPE_TPS65185, 0x68);
    qdev_connect_gpio_out_named(DEVICE(tps65185), "pwrgood", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[2]), 28));
    qdev_connect_gpio_out_named(DEVICE(tps65185), "irq", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 15));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[2]), 29,
        qdev_get_gpio_in_named(DEVICE(tps65185), "powerup", 0));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[2]), 30,
        qdev_get_gpio_in_named(DEVICE(tps65185), "power-enable", 0));

    touch = i2c_slave_new(TYPE_ZFORCE, 0x50);
    qdev_prop_set_uint16(DEVICE(touch), "width", 1080);
    qdev_prop_set_uint16(DEVICE(touch), "height", 1440);
    i2c_slave_realize_and_unref(touch, soc->i2c[0].bus, &error_fatal);
    qdev_connect_gpio_out(DEVICE(touch), 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[4]), 15));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[4]), 26,
        qdev_get_gpio_in(DEVICE(touch), 0));

    msp430 = i2c_slave_create_simple(soc->i2c[2].bus,
                                     TYPE_KOBOTOUCH_MSP430, 0x43);
    qdev_connect_gpio_out_named(DEVICE(msp430), "irq", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 11));

    pmic = qdev_new(TYPE_MC13892);
    ssi_realize_and_unref(pmic,
        (SSIBus *)qdev_get_child_bus(DEVICE(&soc->spi[2]), "spi"),
        &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(&soc->spi[2]), 1,
        qemu_irq_invert(qdev_get_gpio_in_named(pmic, SSI_GPIO_CS, 0)));
    qdev_connect_gpio_out_named(pmic, "irq", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[5]), 8));

    wifi = qdev_new(TYPE_BCM43362_SDIO);
    qemu_configure_nic_device(wifi, true, "bcm43362");
    qdev_realize_and_unref(wifi,
        qdev_get_child_bus(DEVICE(&soc->esdhc[1]), "sd-bus"), &error_fatal);
    qdev_connect_gpio_out(DEVICE(&soc->gpio[4]), 14,
        qdev_get_gpio_in_named(wifi, "power", 0));
    qdev_connect_gpio_out_named(wifi, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->esdhc[1]), "sdio-irq", 0));
    qdev_connect_gpio_out_named(wifi, "oob-irq", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 8));

    buttons = qdev_new(TYPE_AURA_HD_BUTTONS);
    qdev_realize_and_unref(buttons, NULL, &error_fatal);
    qdev_connect_gpio_out(buttons, 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 10));
    qdev_connect_gpio_out(buttons, 1,
        qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 1));
}

static void aurahd_init(MachineState *machine)
{
    AuraHDMachineState *s = AURA_HD_MACHINE(machine);
    FslIMX50State *soc;

    if (machine->ram_size != AURA_HD_RAM_SIZE) {
        error_report("Kobo Aura HD requires 512 MiB RAM");
        exit(EXIT_FAILURE);
    }

    soc = s->soc = FSL_IMX50(object_new(TYPE_FSL_IMX50));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_property_set_uint(OBJECT(&soc->usb_otg),
                             "gadget-host-autosuspend-ms", 2000, &error_fatal);
    /* The skipped ROM plug-in configures 512 MiB of mobile DDR. Linux uses
     * DATAbahn's DDR class to initialize its IRAM idle/frequency routines. */
    object_property_set_uint(OBJECT(soc), "ddr-type", 0x100, &error_fatal);
    /* ED068OG1 mounts opposite the Touch panel. The stock EPDC driver
     * reverses all rotations for its 1440x1080 native scan geometry. */
    object_property_set_bool(OBJECT(&soc->epdc), "rotate-ccw", true,
                             &error_fatal);
    object_property_set_uint(OBJECT(soc), "src-sbmr", 0x00200040,
                             &error_fatal);
    /* EIM_WAIT:EIM_EB1 select physical eSDHC3 for the internal card. */
    qdev_prop_set_uint32(DEVICE(&soc->gpio[0]), "reset-psr", 1U << 21);
    /* Released keys, inactive MSP430 IRQ and disconnected charger inputs. */
    qdev_prop_set_uint32(DEVICE(&soc->gpio[3]), "reset-psr",
                         (1U << 1) | (1U << 10) | (1U << 11) |
                         (1U << 18) | (1U << 19) | (1U << 23));
    /* External card absent, magnetic cover open, touch reset released. */
    qdev_prop_set_uint32(DEVICE(&soc->gpio[4]), "reset-psr",
                         (drive_get(IF_SD, 0, 1) ? 0 : (1U << 17)) |
                         (1U << 25) | (1U << 26));
    /* All three I2C buses retain their pull-ups during GPIO bus recovery. */
    qdev_prop_set_uint32(DEVICE(&soc->gpio[5]), "reset-psr", 0x00fc0000);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), AURA_HD_RAM_BASE,
                                machine->ram);
    aurahd_create_peripherals(soc);
    aurahd_attach_card(soc, 2, 0);
    aurahd_attach_card(soc, 0, 1);

    s->boot = (struct arm_boot_info) {
        .loader_start = AURA_HD_RAM_BASE,
        .ram_size = machine->ram_size,
        .board_id = AURA_HD_MACHINE_ID,
    };
    if (machine->firmware) {
        aurahd_load_firmware(s);
        qemu_register_reset(aurahd_firmware_reset, s);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&soc->cpu, machine, &s->boot);
    }
}

static void aurahd_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Kobo Aura HD N204B (i.MX507)";
    mc->init = aurahd_init;
    mc->default_ram_size = AURA_HD_RAM_SIZE;
    mc->default_ram_id = "kobo-aura-hd.ram";
    mc->auto_create_sdcard = true;
    mc->ignore_memory_transaction_failures = true;
}

static const TypeInfo aurahd_types[] = {
    {
        .name = TYPE_AURA_HD_BUTTONS,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(AuraHDButtonsState),
        .instance_init = aurahd_buttons_init,
        .class_init = aurahd_buttons_class_init,
    }, {
        .name = TYPE_AURA_HD_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(AuraHDMachineState),
        .class_init = aurahd_machine_class_init,
        .interfaces = arm_machine_interfaces,
    },
};

DEFINE_TYPES(aurahd_types)
