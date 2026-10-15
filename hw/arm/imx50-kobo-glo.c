/*
 * Kobo Glo N613 (Netronix E606B0, i.MX507) board.
 *
 * Wiring follows kobolabs/imx507 in linux-ereader and u-boot-ereader.
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

#define GLO_RAM_BASE       0x70000000
#define GLO_RAM_SIZE       (256 * MiB)
#define GLO_MACHINE_ID     2988 /* MACH_TYPE_MX50_RDP */
#define GLO_IVT_OFFSET     0x400
#define GLO_IVT_HEADER     0x402000d1

#define TYPE_GLO_MACHINE MACHINE_TYPE_NAME("imx50-kobo-glo")
OBJECT_DECLARE_SIMPLE_TYPE(GloMachineState, GLO_MACHINE)

struct GloMachineState {
    MachineState parent_obj;
    FslIMX50State *soc;
    struct arm_boot_info boot;
};

#define TYPE_GLO_BUTTONS "kobo-glo-buttons"
OBJECT_DECLARE_SIMPLE_TYPE(GloButtonsState, GLO_BUTTONS)

struct GloButtonsState {
    SysBusDevice parent_obj;
    QemuInputHandlerState *input;
    MemoryRegion keypad;
    qemu_irq power;
    qemu_irq keypad_irq;
    uint16_t kpcr, kpsr, kddr, kpdr;
    uint8_t pressed;
};

/* The Glo frontlight switch connects KPP column 0 to row 0. Linux's
 * mxc_keyb driver scans columns through KPDR and acknowledges KPSR W1C
 * flags; the power slider remains a separate GPIO input. */
static void glo_buttons_update(GloButtonsState *s)
{
    qemu_set_irq(s->power, !(s->pressed & 1));
    qemu_set_irq(s->keypad_irq, (s->kpsr & (s->kpsr >> 8) & 3) != 0);
}

static uint64_t glo_keypad_read(void *opaque, hwaddr offset, unsigned size)
{
    GloButtonsState *s = opaque;
    uint16_t rows;

    switch (offset) {
    case 0: return s->kpcr;
    case 2: return s->kpsr;
    case 4: return s->kddr;
    case 6:
        rows = 0xff;
        if ((s->pressed & 2) && (s->kpcr & 1) &&
            (s->kddr & BIT(8)) && !(s->kpdr & BIT(8))) {
            rows &= ~1;
        }
        return (s->kpdr & s->kddr) | (rows & ~s->kddr);
    default: return 0;
    }
}

static void glo_keypad_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    GloButtonsState *s = opaque;

    switch (offset) {
    case 0: s->kpcr = value; break;
    case 2:
        /* KPKD/KPKR are W1C; synchronizer reset requests self-clear. */
        s->kpsr = (s->kpsr & 3 & ~value) | (value & 0x700);
        break;
    case 4: s->kddr = value; break;
    case 6: s->kpdr = value; break;
    }
    glo_buttons_update(s);
}

static const MemoryRegionOps glo_keypad_ops = {
    .read = glo_keypad_read,
    .write = glo_keypad_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 2 },
};

static void glo_buttons_event(DeviceState *dev, QemuConsole *src,
                              InputEvent *evt)
{
    GloButtonsState *s = GLO_BUTTONS(dev);
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
    if (button == 1 && !!(s->pressed & 2) != key->down) {
        s->kpsr |= key->down ? BIT(0) : BIT(1);
    }
    s->pressed = deposit32(s->pressed, button, 1, key->down);
    glo_buttons_update(s);
}

static const QemuInputHandler glo_buttons_handler = {
    .name = "Kobo Glo buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = glo_buttons_event,
};

static void glo_buttons_reset(DeviceState *dev)
{
    GloButtonsState *s = GLO_BUTTONS(dev);

    s->pressed = 0;
    s->kpcr = s->kpsr = s->kddr = s->kpdr = 0;
    glo_buttons_update(s);
}

static int glo_buttons_post_load(void *opaque, int version_id)
{
    glo_buttons_update(opaque);
    return 0;
}

static const VMStateDescription glo_buttons_vmstate = {
    .name = TYPE_GLO_BUTTONS,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = glo_buttons_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(pressed, GloButtonsState),
        VMSTATE_UINT16(kpcr, GloButtonsState),
        VMSTATE_UINT16(kpsr, GloButtonsState),
        VMSTATE_UINT16(kddr, GloButtonsState),
        VMSTATE_UINT16(kpdr, GloButtonsState),
        VMSTATE_END_OF_LIST()
    },
};

static void glo_buttons_realize(DeviceState *dev, Error **errp)
{
    GLO_BUTTONS(dev)->input = qemu_input_handler_register(
        dev, &glo_buttons_handler);
}

static void glo_buttons_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(GLO_BUTTONS(dev)->input);
}

static void glo_buttons_init(Object *obj)
{
    GloButtonsState *s = GLO_BUTTONS(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->power, 1);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->keypad_irq);
    memory_region_init_io(&s->keypad, obj, &glo_keypad_ops, s,
                          "kobo-glo.keypad", 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->keypad);
}

static void glo_buttons_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = glo_buttons_realize;
    dc->unrealize = glo_buttons_unrealize;
    dc->vmsd = &glo_buttons_vmstate;
    device_class_set_legacy_reset(dc, glo_buttons_reset);
}

static void glo_firmware_reset(void *opaque)
{
    GloMachineState *s = opaque;
    ARMCPU *cpu = &s->soc->cpu;

    cpu_reset(CPU(cpu));
    cpu->env.regs[13] = FSL_IMX50_IRAM_ADDR + FSL_IMX50_IRAM_SIZE;
    cpu_set_pc(CPU(cpu), s->boot.entry);
}

static size_t glo_boot_data_offset(const uint8_t *image, size_t image_size,
                                    size_t ivt_offset)
{
    uint32_t self, boot_data;

    if (ivt_offset > image_size || image_size - ivt_offset < 0x20 ||
        ldl_le_p(image + ivt_offset) != GLO_IVT_HEADER) {
        error_report("Glo firmware has an invalid IVT at 0x%zx",
                     ivt_offset);
        exit(EXIT_FAILURE);
    }
    self = ldl_le_p(image + ivt_offset + 0x14);
    boot_data = ldl_le_p(image + ivt_offset + 0x10);
    if (boot_data < self ||
        boot_data - self > image_size - ivt_offset - 12) {
        error_report("Glo firmware boot-data pointer is outside the image");
        exit(EXIT_FAILURE);
    }
    return ivt_offset + boot_data - self;
}

static void glo_load_firmware(GloMachineState *s)
{
    MachineState *machine = MACHINE(s);
    g_autofree uint8_t *image = NULL;
    gsize image_size;
    size_t ivt_offset = GLO_IVT_OFFSET;
    size_t boot_data;
    uint32_t self, entry, start, length;
    hwaddr ram_end = GLO_RAM_BASE + machine->ram_size;

    if (!g_file_get_contents(machine->firmware, (char **)&image,
                             &image_size, NULL)) {
        error_report("Unable to read Glo firmware '%s'", machine->firmware);
        exit(EXIT_FAILURE);
    }

    boot_data = glo_boot_data_offset(image, image_size, ivt_offset);
    if (ldl_le_p(image + boot_data + 8) == 1) {
        /* The ROM plug-in programs physical DDR. Enter its following SDRAM
         * image directly, since the emulated SDRAM is already available. */
        ivt_offset = boot_data + 12;
        boot_data = glo_boot_data_offset(image, image_size, ivt_offset);
    }
    self = ldl_le_p(image + ivt_offset + 0x14);
    entry = ldl_le_p(image + ivt_offset + 4);
    start = ldl_le_p(image + boot_data);
    length = ldl_le_p(image + boot_data + 4);
    if (ldl_le_p(image + boot_data + 8) != 0 ||
        self < ivt_offset || start != self - ivt_offset ||
        start < GLO_RAM_BASE || start >= ram_end ||
        image_size > ram_end - start || !length || length > image_size ||
        entry < start || entry - start >= length) {
        error_report("Glo firmware main IVT does not describe an SDRAM image");
        exit(EXIT_FAILURE);
    }
    if (load_image_targphys(machine->firmware, start, ram_end - start,
                            NULL) < 0) {
        error_report("Unable to load Glo firmware '%s'", machine->firmware);
        exit(EXIT_FAILURE);
    }
    s->boot.entry = entry;
}

static void glo_attach_card(FslIMX50State *soc, unsigned controller,
                               unsigned unit)
{
    DriveInfo *di = drive_get(IF_SD, 0, unit);
    DeviceState *card;

    if (!di) {
        if (!unit && !qtest_enabled()) {
            error_report("Glo requires its internal SD image at index 0");
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

static void glo_create_peripherals(FslIMX50State *soc)
{
    DeviceState *usb_misc = qdev_new(TYPE_IMX_USB_MISC);
    I2CSlave *touch;
    I2CSlave *tps65185;
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
    qdev_prop_set_uint16(DEVICE(touch), "width", 758);
    qdev_prop_set_uint16(DEVICE(touch), "height", 1024);
    i2c_slave_realize_and_unref(touch, soc->i2c[0].bus, &error_fatal);
    qdev_connect_gpio_out(DEVICE(touch), 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[4]), 15));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[4]), 26,
        qdev_get_gpio_in(DEVICE(touch), 0));

    i2c_slave_create_simple(soc->i2c[2].bus, TYPE_KOBOTOUCH_MSP430, 0x43);

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
        qdev_get_child_bus(DEVICE(&soc->esdhc[2]), "sd-bus"), &error_fatal);
    qdev_connect_gpio_out(DEVICE(&soc->gpio[4]), 14,
        qdev_get_gpio_in_named(wifi, "power", 0));
    qdev_connect_gpio_out_named(wifi, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->esdhc[2]), "sdio-irq", 0));
    qdev_connect_gpio_out_named(wifi, "oob-irq", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 8));

    buttons = qdev_new(TYPE_GLO_BUTTONS);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(buttons), &error_fatal);
    qdev_connect_gpio_out(buttons, 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 10));
    sysbus_mmio_map(SYS_BUS_DEVICE(buttons), 0, 0x53f94000);
    sysbus_connect_irq(SYS_BUS_DEVICE(buttons), 0,
        qdev_get_gpio_in(DEVICE(&soc->tzic), 60));
}

static void glo_init(MachineState *machine)
{
    GloMachineState *s = GLO_MACHINE(machine);
    FslIMX50State *soc;

    if (machine->ram_size != GLO_RAM_SIZE) {
        error_report("Kobo Glo requires 256 MiB RAM");
        exit(EXIT_FAILURE);
    }

    soc = s->soc = FSL_IMX50(object_new(TYPE_FSL_IMX50));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_property_set_uint(OBJECT(&soc->usb_otg),
                             "gadget-host-autosuspend-ms", 2000, &error_fatal);
    /* The skipped ROM plug-in configures 256 MiB of mobile DDR. Linux uses
     * DATAbahn's DDR class to initialize its IRAM idle/frequency routines. */
    object_property_set_uint(OBJECT(soc), "ddr-type", 0x100, &error_fatal);
    /* The 1024x758 native panel is rotated clockwise for portrait input. */
    object_property_set_bool(OBJECT(&soc->epdc), "rotate-ccw", false,
                             &error_fatal);
    object_property_set_uint(OBJECT(soc), "src-sbmr", 0x00000040,
                             &error_fatal);
    /* EIM_WAIT:EIM_EB1 select physical eSDHC1 for the internal card. */
    qdev_prop_set_uint32(DEVICE(&soc->gpio[0]), "reset-psr", 0);
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
    memory_region_add_subregion(get_system_memory(), GLO_RAM_BASE,
                                machine->ram);
    glo_create_peripherals(soc);
    glo_attach_card(soc, 0, 0);
    glo_attach_card(soc, 1, 1);

    s->boot = (struct arm_boot_info) {
        .loader_start = GLO_RAM_BASE,
        .ram_size = machine->ram_size,
        .board_id = GLO_MACHINE_ID,
    };
    if (machine->firmware) {
        glo_load_firmware(s);
        qemu_register_reset(glo_firmware_reset, s);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&soc->cpu, machine, &s->boot);
    }
}

static void glo_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Kobo Glo N613 (i.MX507)";
    mc->init = glo_init;
    mc->default_ram_size = GLO_RAM_SIZE;
    mc->default_ram_id = "kobo-glo.ram";
    mc->auto_create_sdcard = true;
    mc->ignore_memory_transaction_failures = true;
}

static const TypeInfo glo_types[] = {
    {
        .name = TYPE_GLO_BUTTONS,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(GloButtonsState),
        .instance_init = glo_buttons_init,
        .class_init = glo_buttons_class_init,
    }, {
        .name = TYPE_GLO_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(GloMachineState),
        .class_init = glo_machine_class_init,
        .interfaces = arm_machine_interfaces,
    },
};

DEFINE_TYPES(glo_types)
