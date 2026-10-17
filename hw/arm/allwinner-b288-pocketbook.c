/*
 * PocketBook B288 boards, with the original vendor boot package.
 * Boot0's package-loading and DRAM handoff are supplied by the board loader.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/arm/allwinner-b288.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/elan-ktf2k.h"
#include "hw/i2c/zforce.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "ui/input.h"

#define B288_UBOOT_BASE 0x4a000000
#define TYPE_B288PB_MACHINE MACHINE_TYPE_NAME("allwinner-b288-pocketbook")
OBJECT_DECLARE_TYPE(B288PBMachineState, B288PBMachineClass, B288PB_MACHINE)

#define TYPE_B288PB_BUTTONS "pocketbook-b288-buttons"
OBJECT_DECLARE_SIMPLE_TYPE(B288PBButtonsState, B288PB_BUTTONS)

struct B288PBButtonsState {
    DeviceState parent_obj;
    QemuInputHandlerState *input;
    qemu_irq output[3];
    uint8_t pressed;
    uint16_t home_mv;
};

static void b288_pb_buttons_update(B288PBButtonsState *s)
{
    qemu_set_irq(s->output[0], !(s->pressed & BIT(0)));
    qemu_set_irq(s->output[1], !(s->pressed & BIT(1)));
    qemu_set_irq(s->output[2], (s->pressed & BIT(2)) ? s->home_mv :
                              (s->pressed & BIT(3)) ? 1100 : 2000);
}

static void b288_pb_buttons_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    B288PBButtonsState *s = B288PB_BUTTONS(dev);
    InputKeyEvent *key = evt->u.key.data;
    unsigned button;

    switch (qemu_input_key_value_to_qcode(key->key)) {
    case Q_KEY_CODE_PGDN:
    case Q_KEY_CODE_RIGHT: button = 0; break;
    case Q_KEY_CODE_PGUP:
    case Q_KEY_CODE_LEFT: button = 1; break;
    case Q_KEY_CODE_HOME: button = 2; break;
    case Q_KEY_CODE_ESC:
    case Q_KEY_CODE_BACKSPACE: button = 3; break;
    default: return;
    }
    s->pressed = deposit32(s->pressed, button, 1, key->down);
    b288_pb_buttons_update(s);
}

static const QemuInputHandler b288_pb_buttons_handler = {
    .name = "PocketBook B288 buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = b288_pb_buttons_event,
};

static void b288_pb_buttons_reset(DeviceState *dev)
{
    B288PBButtonsState *s = B288PB_BUTTONS(dev);
    s->pressed = 0;
    b288_pb_buttons_update(s);
}

static int b288_pb_buttons_post_load(void *opaque, int version_id)
{
    b288_pb_buttons_update(opaque);
    return 0;
}

static const VMStateDescription b288_pb_buttons_vmstate = {
    .name = TYPE_B288PB_BUTTONS,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = b288_pb_buttons_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(pressed, B288PBButtonsState),
        VMSTATE_END_OF_LIST()
    },
};

static void b288_pb_buttons_realize(DeviceState *dev, Error **errp)
{
    B288PB_BUTTONS(dev)->input = qemu_input_handler_register(dev, &b288_pb_buttons_handler);
}

static void b288_pb_buttons_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(B288PB_BUTTONS(dev)->input);
}

static void b288_pb_buttons_init(Object *obj)
{
    qdev_init_gpio_out(DEVICE(obj), B288PB_BUTTONS(obj)->output, 3);
}

static void b288_pb_buttons_class_init(ObjectClass *oc, const void *data)
{
    static const Property props[] = {
        DEFINE_PROP_UINT16("home-millivolts", B288PBButtonsState, home_mv, 700),
    };
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = b288_pb_buttons_realize;
    dc->unrealize = b288_pb_buttons_unrealize;
    dc->vmsd = &b288_pb_buttons_vmstate;
    device_class_set_props(dc, props);
    device_class_set_legacy_reset(dc, b288_pb_buttons_reset);
}

typedef struct B288PBProfile {
    const char *description;
    const char *ram_id;
    uint64_t ram_size;
    uint32_t width, height, rotation, color_filter;
    uint16_t touch_width, touch_height;
    unsigned pmic_wakeup, pmic_powerup, pmic_powergood, pmic_irq;
    unsigned page_next, page_previous;
    uint16_t home_mv;
    bool touch_swap_axes, touch_invert_x, touch_invert_y;
    bool elan_touch;
    bool touch_only;
    const char *focaltech_touch;
} B288PBProfile;

struct B288PBMachineClass {
    MachineClass parent_class;
    const B288PBProfile *profile;
};

struct B288PBMachineState {
    MachineState parent_obj;
    AwB288State *soc;
    DeviceState *touch;
    DeviceState *frontlight;
    struct arm_boot_info boot;
};

static void b288_pb_reset(void *opaque)
{
    B288PBMachineState *s = opaque;
    const B288PBProfile *p = B288PB_MACHINE_GET_CLASS(s)->profile;
    /* Released keys, touchscreen IRQ and EPD power-good/power-fault pins. */
    const unsigned high_pins[] = { p->pmic_irq, p->pmic_powergood, 32 + 11,
                                  32 + 14, 32 + 15, 32 + 16, 5 * 32 + 6 };
    for (unsigned i = 0; i < ARRAY_SIZE(high_pins); i++) {
        qemu_set_irq(qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in",
                                            high_pins[i]), 1);
    }
    if (p->elan_touch) {
        qemu_set_irq(qdev_get_gpio_in_named(s->touch, "power", 0), 1);
        qemu_set_irq(qdev_get_gpio_in_named(s->touch, "reset", 0), 1);
    } else if (p->focaltech_touch) {
        qemu_set_irq(qdev_get_gpio_in_named(s->touch, "reset", 0), 1);
    } else {
        qemu_set_irq(qdev_get_gpio_in(s->touch, 0), 1);
    }
    qemu_set_irq(qdev_get_gpio_in_named(s->frontlight, "enable", 0), 1);
    if (MACHINE(s)->firmware) {
        cpu_reset(CPU(&s->soc->cpus[0]));
        s->soc->cpus[0].env.regs[13] = 0xd000;
        cpu_set_pc(CPU(&s->soc->cpus[0]), B288_UBOOT_BASE);
    }
}

static void b288_pb_load_boot_package(MachineState *machine)
{
    g_autofree char *package = NULL;
    gsize length;
    uint32_t count, offset, size;
    bool have_uboot = false, have_dtb = false, have_config = false;

    if (!g_file_get_contents(machine->firmware, &package, &length, NULL) ||
        length < 64 || memcmp(package, "sunxi-package", 13) ||
        ldl_le_p(package + 16) != 0x89119800 ||
        ldl_le_p(package + 36) != length) {
        error_report("PocketBook B288 requires an original Sunxi U-Boot package");
        exit(EXIT_FAILURE);
    }
    count = ldl_le_p(package + 32);
    if (!count || count > (length - 64) / 368) {
        error_report("PocketBook B288 boot package has an invalid item table");
        exit(EXIT_FAILURE);
    }
    for (unsigned i = 0; i < count; i++) {
        char *item = package + 64 + i * 368;
        offset = ldl_le_p(item + 64);
        size = ldl_le_p(item + 68);
        if (offset > length || size > length - offset || !size) {
            error_report("PocketBook B288 boot package item is truncated");
            exit(EXIT_FAILURE);
        }
        if (!strncmp(item, "u-boot", 64)) {
            g_autofree uint8_t *uboot = g_memdup2(package + offset, size);
            if (size < 0x600 || memcmp(uboot + 4, "uboot\0\0\0", 8)) {
                error_report("PocketBook B288 package has an invalid U-Boot header");
                exit(EXIT_FAILURE);
            }
            /* Values boot0 supplies after DRAM setup and package loading. */
            stl_le_p(uboot + 0xe4, 4); /* STORAGE_EMMC3 */
            stl_le_p(uboot + 0x4f4, length);
            stl_le_p(uboot + 0x4f8, machine->ram_size / MiB);
            rom_add_blob_fixed("b288_pb.u-boot", uboot, size, B288_UBOOT_BASE);
            have_uboot = true;
        } else if (!strncmp(item, "dtb", 64)) {
            rom_add_blob_fixed("b288_pb.dtb", package + offset, size, 0x4a200000);
            have_dtb = true;
        } else if (!strncmp(item, "soc-cfg", 64)) {
            rom_add_blob_fixed("b288_pb.soc-cfg", package + offset, size, 0x4a300000);
            rom_add_blob_fixed("b288_pb.script", package + offset, size, 0x43000000);
            have_config = true;
        }
    }
    if (!have_uboot || !have_dtb || !have_config) {
        error_report("PocketBook B288 package must contain U-Boot, DTB and SoC configuration");
        exit(EXIT_FAILURE);
    }
}

static void b288_pb_init(MachineState *machine)
{
    B288PBMachineState *s = B288PB_MACHINE(machine);
    const B288PBProfile *p = B288PB_MACHINE_GET_CLASS(s)->profile;
    if (machine->ram_size != p->ram_size) {
        error_report("%s requires %" PRIu64 " MiB RAM",
                     p->description, p->ram_size / MiB);
        exit(EXIT_FAILURE);
    }
    s->soc = AW_B288(object_new(TYPE_AW_B288));
    object_property_add_child(OBJECT(s), "soc", OBJECT(s->soc));
    object_unref(OBJECT(s->soc));
    qdev_prop_set_uint32(s->soc->display, "width", p->width);
    qdev_prop_set_uint32(s->soc->display, "height", p->height);
    qdev_prop_set_uint32(s->soc->display, "rotation", p->rotation);
    qdev_prop_set_uint32(s->soc->display, "color-filter", p->color_filter);
    sysbus_realize(SYS_BUS_DEVICE(s->soc), &error_fatal);
    memory_region_add_subregion(get_system_memory(), AW_B288_SDRAM_BASE,
                                 machine->ram);
    if (!p->touch_only) {
        DeviceState *buttons = qdev_new(TYPE_B288PB_BUTTONS);
        object_property_add_child(OBJECT(s), "buttons", OBJECT(buttons));
        qdev_prop_set_uint16(buttons, "home-millivolts", p->home_mv);
        qdev_realize_and_unref(buttons, NULL, &error_fatal);
        qdev_connect_gpio_out(buttons, 0,
            qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", p->page_next));
        qdev_connect_gpio_out(buttons, 1,
            qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", p->page_previous));
        qdev_connect_gpio_out(buttons, 2,
            qdev_get_gpio_in_named(DEVICE(s->soc), "lradc-in", 0));
    }
    for (unsigned i = 0; i < AW_B288_NUM_MMC; i++) {
        DriveInfo *di = drive_get(IF_SD, 0, i);
        if (di) {
            DeviceState *card = qdev_new(i == 3 ? TYPE_EMMC : TYPE_SD_CARD);
            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card,
                qdev_get_child_bus(DEVICE(&s->soc->mmc[i]), "sd-bus"),
                &error_fatal);
        }
    }
    DeviceState *wifi = qdev_new("xradio-sdio");
    qemu_configure_nic_device(wifi, true, "xradio");
    qdev_realize_and_unref(wifi,
        qdev_get_child_bus(DEVICE(&s->soc->mmc[1]), "sd-bus"), &error_fatal);
    qdev_connect_gpio_out_named(DEVICE(s->soc), "wlan-power", 0,
        qdev_get_gpio_in_named(wifi, "power", 0));
    qdev_connect_gpio_out_named(wifi, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->soc->gic), 107));
    i2c_slave_create_simple(s->soc->i2c[0].bus, "axp227_pmu", 0x34);
    DeviceState *pmic = qdev_new(TYPE_TPS65185);
    qdev_prop_set_bit(pmic, "gpio-rails", true);
    qdev_prop_set_uint8(pmic, "address", 0x68);
    object_property_add_child(OBJECT(s), "display-pmic", OBJECT(pmic));
    qdev_realize_and_unref(pmic, BUS(s->soc->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out_named(DEVICE(pmic), "pwrgood", 0,
        qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", p->pmic_powergood));
    qdev_connect_gpio_out_named(DEVICE(pmic), "irq", 0,
        qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", p->pmic_irq));
    qdev_connect_gpio_out_named(DEVICE(s->soc), "gpio-out", p->pmic_wakeup,
        qdev_get_gpio_in_named(DEVICE(pmic), "powerup", 0));
    qdev_connect_gpio_out_named(DEVICE(s->soc), "gpio-out", p->pmic_powerup,
        qdev_get_gpio_in_named(DEVICE(pmic), "power-enable", 0));
    s->frontlight = DEVICE(i2c_slave_create_simple(s->soc->i2c[1].bus, "lm3630a", 0x36));
    s->touch = qdev_new(p->elan_touch ? TYPE_ELAN_KTF2K :
                       p->focaltech_touch ? p->focaltech_touch : "cyttsp5");
    object_property_add_child(OBJECT(s), "touchscreen", OBJECT(s->touch));
    qdev_prop_set_uint16(s->touch, p->elan_touch ? "x-resolution" : "width",
                         p->touch_width);
    qdev_prop_set_uint16(s->touch, p->elan_touch ? "y-resolution" : "height",
                         p->touch_height);
    qdev_prop_set_bit(s->touch, "invert-x", p->touch_invert_x);
    qdev_prop_set_bit(s->touch, "invert-y", p->touch_invert_y);
    qdev_prop_set_bit(s->touch, "swap-axes", p->touch_swap_axes);
    qdev_prop_set_uint8(s->touch, "address", p->elan_touch ? 0x15 :
                        p->focaltech_touch ? 0x38 : 0x24);
    if (p->elan_touch) {
        /* The original XCD2 updater checks firmware version 0x12f8. */
        qdev_prop_set_uint8(s->touch, "contact-mask", 1);
        qdev_prop_set_uint16(s->touch, "firmware-version", 0x12f8);
    }
    qdev_realize_and_unref(s->touch, BUS(s->soc->i2c[2].bus), &error_fatal);
    if (p->elan_touch || p->focaltech_touch) {
        qdev_connect_gpio_out_named(s->touch, "irq", 0,
            qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", 32 + 15));
        qdev_connect_gpio_out_named(DEVICE(s->soc), "gpio-out", 32 + 16,
            qdev_get_gpio_in_named(s->touch, "reset", 0));
    } else {
        qdev_connect_gpio_out(s->touch, 0,
            qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", 32 + 15));
        qdev_connect_gpio_out_named(DEVICE(s->soc), "gpio-out", 32 + 16,
                                    qdev_get_gpio_in(s->touch, 0));
    }
    if (machine->firmware) {
        b288_pb_load_boot_package(machine);
    } else if (machine->kernel_filename) {
        s->boot.loader_start = AW_B288_SDRAM_BASE;
        s->boot.ram_size = machine->ram_size;
        s->boot.board_id = 4137;
        arm_load_kernel(&s->soc->cpus[0], machine, &s->boot);
    } else {
        error_report("PocketBook B288 requires a vendor boot package (-bios)");
        exit(EXIT_FAILURE);
    }
    qemu_register_reset(b288_pb_reset, s);
}

static void b288_pb_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    const B288PBProfile *p = data;
    B288PB_MACHINE_CLASS(oc)->profile = p;
    mc->desc = p->description;
    mc->init = b288_pb_init;
    mc->default_ram_size = p->ram_size;
    mc->default_ram_id = p->ram_id;
    mc->default_cpus = AW_B288_NUM_CPUS;
    mc->min_cpus = AW_B288_NUM_CPUS;
    mc->max_cpus = AW_B288_NUM_CPUS;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");
    mc->block_default_type = IF_SD;
    mc->units_per_default_bus = 4;
    mc->no_parallel = true;
}

static const B288PBProfile pb628_profile = {
    .description = "PocketBook Touch Lux 5 (628, Allwinner B288)",
    .ram_id = "pb628.ram",
    .ram_size = 512 * MiB,
    .width = 1024, .height = 758, .rotation = 90,
    .touch_width = 758, .touch_height = 1024,
    .elan_touch = true,
    .pmic_wakeup = 3 * 32 + 7, .pmic_powerup = 3 * 32 + 6,
    .pmic_powergood = 3 * 32 + 26, .pmic_irq = 3 * 32 + 25,
    .page_next = 32 + 11, .page_previous = 32 + 14,
    .home_mv = 700,
};

static const B288PBProfile pb632_profile = {
    .description = "PocketBook Touch HD 3 (632, Allwinner B288)",
    .ram_id = "pb632.ram",
    .ram_size = 512 * MiB,
    .width = 1448, .height = 1072, .rotation = 90,
    .touch_width = 1448, .touch_height = 1072,
    .pmic_wakeup = 3 * 32 + 7, .pmic_powerup = 3 * 32 + 6,
    .pmic_powergood = 3 * 32 + 26, .pmic_irq = 3 * 32 + 25,
    .page_next = 32 + 14, .page_previous = 32 + 11,
    .home_mv = 700,
    .touch_swap_axes = true,
    .touch_invert_y = true,
};

static const B288PBProfile pb634_profile = {
    .description = "PocketBook Verse Pro (634, Allwinner B288)",
    .ram_id = "pb634.ram",
    .ram_size = 512 * MiB,
    .width = 1448, .height = 1072, .rotation = 90,
    .touch_width = 1448, .touch_height = 1072,
    .pmic_wakeup = 3 * 32 + 7, .pmic_powerup = 3 * 32 + 6,
    .pmic_powergood = 3 * 32 + 26, .pmic_irq = 3 * 32 + 25,
    .page_next = 32 + 11, .page_previous = 32 + 14,
    .home_mv = 240,
    .touch_swap_axes = true,
    .touch_invert_y = true,
};

static const B288PBProfile pb700_profile = {
    .description = "PocketBook Era (700, Allwinner B288)",
    .ram_id = "pb700.ram",
    .ram_size = 1 * GiB,
    .width = 1264, .height = 1680,
    .touch_width = 1680, .touch_height = 1264,
    .focaltech_touch = "ft5436",
    .touch_swap_axes = true,
    .pmic_wakeup = 3 * 32 + 7, .pmic_powerup = 3 * 32 + 6,
    .pmic_powergood = 3 * 32 + 26, .pmic_irq = 3 * 32 + 25,
    .page_next = 32 + 14, .page_previous = 32 + 11,
    .home_mv = 500,
};

static const B288PBProfile pb740_profile = {
    .description = "PocketBook InkPad 3 (740, Allwinner B288)",
    .ram_id = "pb740.ram",
    .ram_size = 1 * GiB,
    .width = 1872, .height = 1404, .rotation = 90,
    .touch_width = 1872, .touch_height = 1404,
    .pmic_wakeup = 32 + 1, .pmic_powerup = 32,
    .pmic_powergood = 32 + 3, .pmic_irq = 32 + 2,
    .page_next = 32 + 11, .page_previous = 32 + 14,
    .home_mv = 700,
    .touch_swap_axes = true,
    .touch_invert_x = true,
};

static const B288PBProfile pb740_2_profile = {
    .description = "PocketBook InkPad 3 Pro (740-2, Allwinner B288)",
    .ram_id = "pb740-2.ram",
    .ram_size = 1 * GiB,
    .width = 1872, .height = 1404, .rotation = 90,
    .touch_width = 1872, .touch_height = 1404,
    /* UART2 uses the original 740's PB0/PB1 EPD power pins. */
    .pmic_wakeup = 3 * 32 + 7, .pmic_powerup = 3 * 32 + 6,
    .pmic_powergood = 3 * 32 + 26, .pmic_irq = 3 * 32 + 25,
    .page_next = 32 + 11, .page_previous = 32 + 14,
    .home_mv = 700,
    .touch_swap_axes = true,
    .touch_invert_x = true,
};

static const B288PBProfile pb1040_profile = {
    .description = "PocketBook InkPad X (1040, Allwinner B288)",
    .ram_id = "pb1040.ram",
    .ram_size = 1 * GiB,
    .width = 1872, .height = 1404, .rotation = 90,
    /* Touch type 20 uses portrait coordinates with identity calibration. */
    .touch_width = 1404, .touch_height = 1872,
    .pmic_wakeup = 3 * 32 + 7, .pmic_powerup = 3 * 32 + 6,
    .pmic_powergood = 3 * 32 + 26, .pmic_irq = 3 * 32 + 25,
    .page_next = 32 + 11, .page_previous = 32 + 14,
    .home_mv = 700,
};

static const B288PBProfile pb970_profile = {
    .description = "PocketBook InkPad Lite (970, Allwinner B288)",
    .ram_id = "pb970.ram",
    .ram_size = 512 * MiB,
    .width = 1200, .height = 825, .rotation = 90,
    .touch_width = 1200, .touch_height = 825,
    .focaltech_touch = "ft5726",
    .touch_swap_axes = true,
    .touch_invert_y = true,
    .pmic_wakeup = 3 * 32 + 7, .pmic_powerup = 3 * 32 + 6,
    .pmic_powergood = 3 * 32 + 26, .pmic_irq = 3 * 32 + 25,
    .page_next = 32 + 14, .page_previous = 32 + 11,
    .home_mv = 500,
};

static const TypeInfo b288_pb_types[] = {
    {
        .name = TYPE_B288PB_BUTTONS,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(B288PBButtonsState),
        .instance_init = b288_pb_buttons_init,
        .class_init = b288_pb_buttons_class_init,
    }, {
        .name = TYPE_B288PB_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(B288PBMachineState),
        .class_size = sizeof(B288PBMachineClass),
        .abstract = true,
        .interfaces = arm_machine_interfaces,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b288-pocketbook-628"),
        .parent = TYPE_B288PB_MACHINE,
        .class_init = b288_pb_class_init,
        .class_data = &pb628_profile,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b288-pocketbook-632"),
        .parent = TYPE_B288PB_MACHINE,
        .class_init = b288_pb_class_init,
        .class_data = &pb632_profile,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b288-pocketbook-634"),
        .parent = TYPE_B288PB_MACHINE,
        .class_init = b288_pb_class_init,
        .class_data = &pb634_profile,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b288-pocketbook-700"),
        .parent = TYPE_B288PB_MACHINE,
        .class_init = b288_pb_class_init,
        .class_data = &pb700_profile,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b288-pocketbook-740"),
        .parent = TYPE_B288PB_MACHINE,
        .class_init = b288_pb_class_init,
        .class_data = &pb740_profile,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b288-pocketbook-740-2"),
        .parent = TYPE_B288PB_MACHINE,
        .class_init = b288_pb_class_init,
        .class_data = &pb740_2_profile,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b288-pocketbook-970"),
        .parent = TYPE_B288PB_MACHINE,
        .class_init = b288_pb_class_init,
        .class_data = &pb970_profile,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b288-pocketbook-1040"),
        .parent = TYPE_B288PB_MACHINE,
        .class_init = b288_pb_class_init,
        .class_data = &pb1040_profile,
    },
};

DEFINE_TYPES(b288_pb_types)
