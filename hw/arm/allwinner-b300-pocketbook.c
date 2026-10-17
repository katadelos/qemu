/*
 * PocketBook B300 boards and the native boot0-to-OP-TEE handoff.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <libfdt.h>
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/allwinner-b300.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/allwinner-b300-power.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"
#include "ui/input.h"

#define B300_UBOOT_BASE 0x4a000000
#define B300_OPTEE_BASE 0x48600000
#define TYPE_B300PB_MACHINE MACHINE_TYPE_NAME("allwinner-b300-pocketbook")
OBJECT_DECLARE_TYPE(B300PBMachineState, B300PBMachineClass, B300PB_MACHINE)

#define TYPE_B300PB_BUTTONS "pocketbook-b300-buttons"
OBJECT_DECLARE_SIMPLE_TYPE(B300PBButtonsState, B300PB_BUTTONS)

struct B300PBButtonsState {
    DeviceState parent_obj;
    QemuInputHandlerState *input;
    qemu_irq output[4];
    uint8_t pressed;
};

static void b300_pb_buttons_update(B300PBButtonsState *s)
{
    for (unsigned i = 0; i < ARRAY_SIZE(s->output); i++) {
        qemu_set_irq(s->output[i], !(s->pressed & BIT(i)));
    }
}

static void b300_pb_buttons_event(DeviceState *dev, QemuConsole *src,
                                  InputEvent *evt)
{
    B300PBButtonsState *s = B300PB_BUTTONS(dev);
    InputKeyEvent *key = evt->u.key.data;
    unsigned button;

    switch (qemu_input_key_value_to_qcode(key->key)) {
    case Q_KEY_CODE_PGDN:
    case Q_KEY_CODE_RIGHT: button = 0; break;
    case Q_KEY_CODE_PGUP:
    case Q_KEY_CODE_LEFT: button = 1; break;
    case Q_KEY_CODE_HOME: button = 2; break;
    case Q_KEY_CODE_MENU:
    case Q_KEY_CODE_ESC:
    case Q_KEY_CODE_BACKSPACE: button = 3; break;
    default: return;
    }
    s->pressed = deposit32(s->pressed, button, 1, key->down);
    b300_pb_buttons_update(s);
}

static const QemuInputHandler b300_pb_buttons_handler = {
    .name = "PocketBook B300 buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = b300_pb_buttons_event,
};

static void b300_pb_buttons_reset(DeviceState *dev)
{
    B300PBButtonsState *s = B300PB_BUTTONS(dev);

    s->pressed = 0;
    b300_pb_buttons_update(s);
}

static int b300_pb_buttons_post_load(void *opaque, int version_id)
{
    b300_pb_buttons_update(opaque);
    return 0;
}

static const VMStateDescription b300_pb_buttons_vmstate = {
    .name = TYPE_B300PB_BUTTONS,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = b300_pb_buttons_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(pressed, B300PBButtonsState),
        VMSTATE_END_OF_LIST()
    },
};

static void b300_pb_buttons_realize(DeviceState *dev, Error **errp)
{
    B300PB_BUTTONS(dev)->input =
        qemu_input_handler_register(dev, &b300_pb_buttons_handler);
}

static void b300_pb_buttons_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(B300PB_BUTTONS(dev)->input);
}

static void b300_pb_buttons_init(Object *obj)
{
    qdev_init_gpio_out(DEVICE(obj), B300PB_BUTTONS(obj)->output, 4);
}

static void b300_pb_buttons_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = b300_pb_buttons_realize;
    dc->unrealize = b300_pb_buttons_unrealize;
    dc->vmsd = &b300_pb_buttons_vmstate;
    device_class_set_legacy_reset(dc, b300_pb_buttons_reset);
}

typedef struct B300PBProfile {
    const char *description;
    const char *ram_id;
    uint32_t width, height, rotation;
    uint16_t touch_width, touch_height;
    unsigned page_next, page_previous;
    bool touch_swap_axes, touch_invert_x, touch_invert_y;
    const char *touch_type;
} B300PBProfile;

struct B300PBMachineClass {
    MachineClass parent_class;
    const B300PBProfile *profile;
};

struct B300PBMachineState {
    MachineState parent_obj;
    AwB300State *soc;
    DeviceState *touch;
    DeviceState *frontlight;
};

static void b300_pb_reset(void *opaque)
{
    B300PBMachineState *s = opaque;
    const B300PBProfile *p = B300PB_MACHINE_GET_CLASS(s)->profile;
    const unsigned high_pins[] = {
        11 * 32 + 3, 11 * 32 + 9,
        11 * 32 + 10, 11 * 32 + 11, 11 * 32 + 12,
    };

    for (unsigned i = 0; i < ARRAY_SIZE(high_pins); i++) {
        qemu_set_irq(qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in",
                                            high_pins[i]), 1);
    }
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in",
                                        11 * 32 + 6), 0);
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in",
                                        7 * 32 + 9),
                  !strcmp(p->touch_type, "cyttsp5"));
    if (!strcmp(p->touch_type, "cyttsp5")) {
        qemu_set_irq(qdev_get_gpio_in(s->touch, 0), 1);
    } else {
        qemu_set_irq(qdev_get_gpio_in_named(s->touch, "reset", 0), 1);
    }
    qemu_set_irq(qdev_get_gpio_in_named(s->frontlight, "enable", 0), 1);

    /* Boot0 has installed the SCP SRAM/DRAM images before releasing reset. */
    uint32_t scp_reset = cpu_to_le32(1);
    address_space_write(&address_space_memory, 0x07000400,
                         MEMTXATTRS_UNSPECIFIED, &scp_reset, sizeof(scp_reset));
    cpu_reset(CPU(&s->soc->cpus[0]));
    s->soc->cpus[0].env.regs[13] = 0x00037ff0;
    s->soc->cpus[0].env.regs[14] = B300_UBOOT_BASE;
    cpu_set_pc(CPU(&s->soc->cpus[0]), B300_OPTEE_BASE);
}

static void b300_pb_dram_handoff(uint8_t *uboot, const void *fdt)
{
    static const char *const fields[] = {
        "dram_clk", "dram_type", "dram_dx_odt", "dram_dx_dri",
        "dram_ca_dri", "dram_odt_en", "dram_para1", "dram_para2",
        "dram_mr0", "dram_mr1", "dram_mr2", "dram_mr3",
        "dram_mr4", "dram_mr5", "dram_mr6", "dram_mr11",
        "dram_mr12", "dram_mr13", "dram_mr14", "dram_mr16",
        "dram_mr17", "dram_mr22", "dram_tpr0", "dram_tpr1",
        "dram_tpr2", "dram_tpr3", "dram_tpr6", "dram_tpr10",
        "dram_tpr11", "dram_tpr12", "dram_tpr13", "dram_tpr14",
    };
    int node = fdt_path_offset(fdt, "/dram");

    if (node < 0) {
        error_report("PocketBook B300 DTB has no native DRAM configuration");
        exit(EXIT_FAILURE);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(fields); i++) {
        int length;
        const fdt32_t *property = fdt_getprop(fdt, node, fields[i], &length);
        uint32_t value = 0;

        /* tpr14 is the native reserved final word, absent from these DTBs. */
        if (property && length == sizeof(*property)) {
            value = fdt32_to_cpu(*property);
        } else if (i != 31) {
            error_report("PocketBook B300 DTB has invalid %s", fields[i]);
            exit(EXIT_FAILURE);
        }
        if (i == 30) {
            /* Boot0 marks its completed DRAM training with tpr13 bit31. */
            value |= BIT(31);
        }
        stl_le_p(uboot + 0x30 + i * 4, value);
    }
}

static void b300_pb_load_boot_package(MachineState *machine)
{
    g_autofree char *package = NULL;
    gsize length;
    bool have_uboot = false, have_optee = false;
    bool have_dtb = false, have_config = false, have_scp = false;
    const void *fdt = NULL;

    if (!machine->firmware ||
        !g_file_get_contents(machine->firmware, &package, &length, NULL) ||
        length < 64 || memcmp(package, "sunxi-package", 13) ||
        ldl_le_p(package + 16) != 0x89119800 ||
        ldl_le_p(package + 36) != length) {
        error_report("PocketBook B300 requires an original Sunxi boot package");
        exit(EXIT_FAILURE);
    }
    uint32_t count = ldl_le_p(package + 32);

    if (!count || count > (length - 64) / 368) {
        error_report("PocketBook B300 boot package has an invalid item table");
        exit(EXIT_FAILURE);
    }
    for (unsigned i = 0; i < count; i++) {
        char *item = package + 64 + i * 368;
        uint32_t offset = ldl_le_p(item + 64);
        uint32_t size = ldl_le_p(item + 68);

        if (offset > length || size > length - offset || !size) {
            error_report("PocketBook B300 boot package item is truncated");
            exit(EXIT_FAILURE);
        }
        if (!strncmp(item, "dtb", 64)) {
            fdt = package + offset;
            if (size < sizeof(struct fdt_header) || fdt_check_header(fdt) ||
                fdt_totalsize(fdt) > size) {
                error_report("PocketBook B300 package has an invalid DTB");
                exit(EXIT_FAILURE);
            }
        }
    }
    if (!fdt) {
        error_report("PocketBook B300 package has no DTB");
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("b300_pb.package", package, length, 0x42e00000);
    for (unsigned i = 0; i < count; i++) {
        char *item = package + 64 + i * 368;
        uint32_t offset = ldl_le_p(item + 64);
        uint32_t size = ldl_le_p(item + 68);
        char *payload = package + offset;

        if (offset > length || size > length - offset || !size) {
            error_report("PocketBook B300 boot package item is truncated");
            exit(EXIT_FAILURE);
        }
        if (!strncmp(item, "u-boot", 64)) {
            g_autofree uint8_t *uboot = g_memdup2(payload, size);

            if (size < 0x600 || memcmp(uboot + 4, "uboot\0\0\0", 8)) {
                error_report("PocketBook B300 package has an invalid U-Boot header");
                exit(EXIT_FAILURE);
            }
            /* Values supplied by the native boot0 package/DRAM handoff. */
            b300_pb_dram_handoff(uboot, fdt);
            /* Native boot0's calibrated digital eMMC timing handoff. */
            memset(uboot + 0x3e8, 0, 12 * sizeof(uint32_t));
            stl_le_p(uboot + 0x420, 0x08000000); /* CARD_TYPE_MMC */
            stl_le_p(uboot + 0x424, 0x55000001); /* Tuning table valid. */
            stl_le_p(uboot + 0x428, 1); /* 1.8 V I/O bias. */
            stl_le_p(uboot + 0xe4, 2); /* STORAGE_EMMC: controller 2. */
            uboot[0x4e8] = 1; /* Secure monitor is present. */
            stl_le_p(uboot + 0x4f4, length);
            stl_le_p(uboot + 0x4f8, machine->ram_size / MiB);
            rom_add_blob_fixed("b300_pb.u-boot", uboot, size, B300_UBOOT_BASE);
            have_uboot = true;
        } else if (!strncmp(item, "optee", 64)) {
            if (size < 64 || memcmp(payload + 4, "optee\0\0\0", 8)) {
                error_report("PocketBook B300 package has an invalid OP-TEE header");
                exit(EXIT_FAILURE);
            }
            rom_add_blob_fixed("b300_pb.optee", payload, size, B300_OPTEE_BASE);
            have_optee = true;
        } else if (!strncmp(item, "dtb", 64)) {
            rom_add_blob_fixed("b300_pb.dtb", payload, size, 0x4a200000);
            have_dtb = true;
        } else if (!strncmp(item, "soc-cfg", 64)) {
            rom_add_blob_fixed("b300_pb.soc-cfg", payload, size, 0x4a300000);
            rom_add_blob_fixed("b300_pb.script", payload, size, 0x43000000);
            have_config = true;
        } else if (!strncmp(item, "scp", 64)) {
            if (size <= 0x1c000 || size > 0x20000) {
                error_report("PocketBook B300 package has an invalid SCP image");
                exit(EXIT_FAILURE);
            }
            rom_add_blob_fixed("b300_pb.scp-sram", payload, 0x1c000, 0x00100000);
            rom_add_blob_fixed("b300_pb.scp-dram", payload + 0x1c000,
                                size - 0x1c000, 0x48100000);
            have_scp = true;
        }
    }
    if (!have_uboot || !have_optee || !have_dtb || !have_config || !have_scp) {
        error_report("PocketBook B300 package needs U-Boot, OP-TEE, SCP, DTB and config");
        exit(EXIT_FAILURE);
    }
}

static void b300_pb_init(MachineState *machine)
{
    B300PBMachineState *s = B300PB_MACHINE(machine);
    const B300PBProfile *p = B300PB_MACHINE_GET_CLASS(s)->profile;
    DeviceState *dev;

    if (machine->ram_size != GiB) {
        error_report("%s requires 1024 MiB RAM", p->description);
        exit(EXIT_FAILURE);
    }
    s->soc = AW_B300(object_new(TYPE_AW_B300));
    object_property_add_child(OBJECT(s), "soc", OBJECT(s->soc));
    object_unref(OBJECT(s->soc));
    qdev_prop_set_uint32(s->soc->display, "width", p->width);
    qdev_prop_set_uint32(s->soc->display, "height", p->height);
    qdev_prop_set_uint32(s->soc->display, "rotation", p->rotation);
    qdev_prop_set_uint32(s->soc->display, "color-filter", 3);
    sysbus_realize(SYS_BUS_DEVICE(s->soc), &error_fatal);
    memory_region_add_subregion(get_system_memory(), AW_B300_SDRAM_BASE,
                                 machine->ram);

    dev = qdev_new(TYPE_B300PB_BUTTONS);
    object_property_add_child(OBJECT(s), "buttons", OBJECT(dev));
    qdev_realize_and_unref(dev, NULL, &error_fatal);
    const unsigned key_pins[] = {
        p->page_next, p->page_previous, 11 * 32 + 10, 11 * 32 + 3,
    };

    for (unsigned i = 0; i < ARRAY_SIZE(key_pins); i++) {
        qdev_connect_gpio_out(dev, i,
            qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", key_pins[i]));
    }
    for (unsigned i = 0; i < AW_B300_NUM_MMC; i++) {
        DriveInfo *di = drive_get(IF_SD, 0, i);

        if (di) {
            dev = qdev_new(i == 2 ? TYPE_EMMC : TYPE_SD_CARD);
            qdev_prop_set_drive_err(dev, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(dev,
                qdev_get_child_bus(DEVICE(&s->soc->mmc[i]), "sd-bus"),
                &error_fatal);
        }
    }
    dev = qdev_new("aic8800-sdio");
    qemu_configure_nic_device(dev, true, "aic8800");
    qdev_realize_and_unref(dev,
        qdev_get_child_bus(DEVICE(&s->soc->mmc[1]), "sd-bus"), &error_fatal);
    qdev_connect_gpio_out_named(s->soc->power, "wifi-power", 0,
        qdev_get_gpio_in_named(dev, "power", 0));
    qdev_connect_gpio_out_named(dev, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&s->soc->mmc[1]), "sdio-irq", 0));

    dev = qdev_new("sy7636");
    qdev_prop_set_uint8(dev, "address", 0x62);
    object_property_add_child(OBJECT(s), "display-pmic", OBJECT(dev));
    qdev_realize_and_unref(dev, BUS(s->soc->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out_named(dev, "power-good", 0,
        qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", 32 + 2));
    qdev_connect_gpio_out_named(s->soc->power, "lcd-power", 0,
        qdev_get_gpio_in_named(dev, "power", 0));
    qdev_connect_gpio_out_named(DEVICE(s->soc), "gpio-out", 32 + 3,
        qdev_get_gpio_in_named(dev, "enable", 0));
    s->frontlight = DEVICE(i2c_slave_create_simple(s->soc->i2c[1].bus,
                                                   "lm3630a", 0x36));

    s->touch = qdev_new(p->touch_type);
    object_property_add_child(OBJECT(s), "touchscreen", OBJECT(s->touch));
    qdev_prop_set_uint16(s->touch, "width", p->touch_width);
    qdev_prop_set_uint16(s->touch, "height", p->touch_height);
    qdev_prop_set_bit(s->touch, "invert-x", p->touch_invert_x);
    qdev_prop_set_bit(s->touch, "invert-y", p->touch_invert_y);
    qdev_prop_set_bit(s->touch, "swap-axes", p->touch_swap_axes);
    bool cypress = !strcmp(p->touch_type, "cyttsp5");

    qdev_prop_set_uint8(s->touch, "address", cypress ? 0x24 : 0x38);
    if (cypress) {
        qdev_prop_set_bit(s->touch, "packed-touch-reports", true);
    }
    qdev_realize_and_unref(s->touch, BUS(s->soc->i2c[0].bus), &error_fatal);
    if (cypress) {
        qdev_connect_gpio_out(s->touch, 0,
            qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", 11 * 32 + 9));
        qdev_connect_gpio_out_named(DEVICE(s->soc), "gpio-out", 7 * 32 + 9,
                                     qdev_get_gpio_in(s->touch, 0));
    } else {
        qdev_connect_gpio_out_named(s->touch, "irq", 0,
            qdev_get_gpio_in_named(DEVICE(s->soc), "gpio-in", 11 * 32 + 9));
        qdev_connect_gpio_out_named(DEVICE(s->soc), "gpio-out", 7 * 32 + 9,
            qemu_irq_invert(qdev_get_gpio_in_named(s->touch, "reset", 0)));
    }
    b300_pb_load_boot_package(machine);
    qemu_register_reset(b300_pb_reset, s);
}

static void b300_pb_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    const B300PBProfile *p = data;

    B300PB_MACHINE_CLASS(oc)->profile = p;
    mc->desc = p->description;
    mc->init = b300_pb_init;
    mc->default_ram_size = GiB;
    mc->default_ram_id = p->ram_id;
    mc->default_cpus = AW_B300_NUM_CPUS;
    mc->min_cpus = AW_B300_NUM_CPUS;
    mc->max_cpus = AW_B300_NUM_CPUS;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");
    mc->block_default_type = IF_SD;
    mc->units_per_default_bus = AW_B300_NUM_MMC;
    mc->no_parallel = true;
}

static const B300PBProfile pb743k3_profile = {
    .description = "PocketBook InkPad Color 3 (743K3, Allwinner B300)",
    .ram_id = "pb743k3.ram",
    .width = 1872, .height = 1404, .rotation = 90,
    .touch_type = "cyttsp5",
    .touch_width = 1872, .touch_height = 1404,
    .touch_swap_axes = true, .touch_invert_x = true,
    .page_next = 11 * 32 + 12, .page_previous = 11 * 32 + 11,
};

static const B300PBProfile pb700k3_profile = {
    .description = "PocketBook Era Color (700K3, Allwinner B300)",
    .ram_id = "pb700k3.ram",
    .width = 1264, .height = 1680, .rotation = 0,
    .touch_type = "ft3518",
    .touch_width = 1680, .touch_height = 1264,
    .touch_swap_axes = true,
    .page_next = 11 * 32 + 11, .page_previous = 11 * 32 + 12,
};

static const TypeInfo b300_pb_types[] = {
    {
        .name = TYPE_B300PB_BUTTONS,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(B300PBButtonsState),
        .instance_init = b300_pb_buttons_init,
        .class_init = b300_pb_buttons_class_init,
    }, {
        .name = TYPE_B300PB_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(B300PBMachineState),
        .class_size = sizeof(B300PBMachineClass),
        .abstract = true,
        .interfaces = arm_machine_interfaces,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b300-pocketbook-743k3"),
        .parent = TYPE_B300PB_MACHINE,
        .class_init = b300_pb_class_init,
        .class_data = &pb743k3_profile,
    }, {
        .name = MACHINE_TYPE_NAME("allwinner-b300-pocketbook-700k3"),
        .parent = TYPE_B300PB_MACHINE,
        .class_init = b300_pb_class_init,
        .class_data = &pb700k3_profile,
    },
};
DEFINE_TYPES(b300_pb_types)
