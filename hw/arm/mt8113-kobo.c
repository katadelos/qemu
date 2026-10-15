/* Kobo Libra Colour / Clara and Tolino MT8113 readers.
 * Wiring: Kobo-Reader Linux 4.9 E70T00, E60T00 and E60V00 device trees.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <libfdt.h>
#include "hw/arm/mt8113.h"
#include "hw/adc/mt6577_auxadc.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/split-irq.h"
#include "hw/i2c/bd71828.h"
#include "hw/i2c/max20342.h"
#include "hw/sd/sd.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "system/block-backend.h"
#include "system/reset.h"

#define TYPE_KOBO_MT8113_MACHINE "mt8113-kobo"
OBJECT_DECLARE_TYPE(KoboMT8113MachineState, KoboMT8113MachineClass,
                    KOBO_MT8113_MACHINE)

typedef struct KoboMT8113Profile {
    const char *description;
    uint64_t ram_size;
    unsigned cpus, width, height;
    bool cfa, cypress, page_keys;
    const char *epd_type, *frontlight_type;
    unsigned epd_address, epd_enable, epd_power, epd_power_good;
    unsigned touch_irq, touch_reset;
    unsigned frontlight_bus, frontlight_enable, hall;
} KoboMT8113Profile;

struct KoboMT8113MachineClass {
    MachineClass parent_class;
    const KoboMT8113Profile *profile;
};

struct KoboMT8113MachineState {
    MachineState parent_obj;
    MT8113State *soc;
};

static const KoboMT8113Profile libra_colour = {
    .description = "Kobo Libra Colour / Tolino Vision Color (MT8113 / E70T04)",
    .ram_size = GiB, .cpus = 2, .width = 1680, .height = 1264,
    .cfa = true, .page_keys = true,
    .epd_type = "jd9930", .epd_address = 0x18,
    .epd_enable = 11, .epd_power = 13, .epd_power_good = 10,
    .touch_irq = 20, .touch_reset = 14,
    .frontlight_type = "lm3630a", .frontlight_bus = 1,
    .frontlight_enable = 15, .hall = 30,
};

static const KoboMT8113Profile clara_bw = {
    .description = "Kobo Clara BW / Tolino Shine 5 (MT8113L / E60T04)",
    .ram_size = 512 * MiB, .cpus = 1, .width = 1448, .height = 1072,
    .cypress = true,
    .epd_type = "sy7636", .epd_address = 0x62,
    .epd_enable = 34, .epd_power = 13, .epd_power_good = 36,
    .touch_irq = 20, .touch_reset = 14,
    .frontlight_type = "lm3630a", .frontlight_bus = 1,
    .frontlight_enable = 15, .hall = 30,
};

static const KoboMT8113Profile clara_colour = {
    .description = "Kobo Clara Colour / Tolino Shine Color (MT8113T / E60T04)",
    .ram_size = 512 * MiB, .cpus = 2, .width = 1448, .height = 1072,
    .cfa = true, .cypress = true,
    .epd_type = "sy7636", .epd_address = 0x62,
    .epd_enable = 34, .epd_power = 13, .epd_power_good = 36,
    .touch_irq = 20, .touch_reset = 14,
    .frontlight_type = "lm3630a", .frontlight_bus = 1,
    .frontlight_enable = 15, .hall = 30,
};

static const KoboMT8113Profile clara_bw_tpv = {
    .description = "Kobo Clara BW / Tolino Shine 5 TPV (MT8113L / E60V00)",
    .ram_size = 512 * MiB, .cpus = 1, .width = 1448, .height = 1072,
    .cypress = true,
    .epd_type = "sy7636", .epd_address = 0x62,
    .epd_enable = 39, .epd_power = 40, .epd_power_good = 10,
    .touch_irq = 16, .touch_reset = 15,
    .frontlight_type = "lm3697", .frontlight_bus = 0,
    .frontlight_enable = 28, .hall = 29,
};

static void kobo_mt8113_reset(void *opaque)
{
    KoboMT8113MachineState *s = opaque;
    const KoboMT8113Profile *p = KOBO_MT8113_MACHINE_GET_CLASS(s)->profile;

    /* Hall sensor is inactive while the emulated cover is open. */
    qemu_irq_raise(qdev_get_gpio_in_named(DEVICE(&s->soc->gpio),
                                         "gpio-in", p->hall));
    for (unsigned i = 0; i < p->cpus; i++) {
        cpu_reset(CPU(&s->soc->cpu[i]));
    }
}

static void kobo_mt8113_load_firmware(MachineState *machine)
{
    g_autofree char *fit = NULL;
    gsize size;
    int length;
    int node;
    const void *payload;
    const fdt32_t *load;
    uint8_t bootargs[0x224] = { 0 };
    /* BL2 hands its boot arguments to the stock AArch32 U-Boot in r0. */
    static const uint32_t handoff[] = {
        0x58000180, 0xd2803a61, 0xd51e4001, 0xd53e1101,
        0x9275f821, 0xb2400021, 0xd51e1101, 0x580000e1,
        0xd51e4021, 0xd5033fdf, 0xd69f03e0, 0xd503201f,
        0x40000100, 0, 0x44e00000, 0,
    };

    if (!machine->firmware ||
        !g_file_get_contents(machine->firmware, &fit, &size, NULL) ||
        size < sizeof(struct fdt_header) || fdt_check_header(fit) ||
        fdt_totalsize(fit) > size) {
        error_report("Kobo MT8113 requires -bios with a decompressed U-Boot FIT image");
        exit(EXIT_FAILURE);
    }
    node = fdt_path_offset(fit, "/images/kernel@1");
    load = fdt_getprop(fit, node, "load", &length);
    if (!load || length != 4 || fdt32_to_cpu(*load) != 0x44e00000) {
        error_report("Invalid Kobo MT8113 U-Boot load address");
        exit(EXIT_FAILURE);
    }
    payload = fdt_getprop(fit, node, "data", &length);
    if (!payload || length <= 0 || length > 2 * MiB) {
        error_report("Invalid Kobo MT8113 U-Boot payload");
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("kobo_mt8113.u-boot", payload, length, 0x44e00000);
    /* mt8113_tp1_data.h: magic, DRAM size, 512 efuse bytes, end magic. */
    stl_le_p(bootargs, 0x504c504c);
    stl_le_p(bootargs + 4, machine->ram_size);
    stl_le_p(bootargs + 0x208, 0x504c504c);
    stl_le_p(bootargs + 0x210, 1); /* booted by power key */
    stl_le_p(bootargs + 0x218, 1); /* page-up released */
    stl_le_p(bootargs + 0x21c, 1); /* page-down released */
    rom_add_blob_fixed("kobo_mt8113.bootargs", bootargs, sizeof(bootargs),
                       0x40000100);
    rom_add_blob_fixed("kobo_mt8113.handoff", handoff, sizeof(handoff),
                       0x40001000);
}

static void kobo_mt8113_init(MachineState *machine)
{
    KoboMT8113MachineState *s = KOBO_MT8113_MACHINE(machine);
    const KoboMT8113Profile *p = KOBO_MT8113_MACHINE_GET_CLASS(s)->profile;
    MT8113State *soc = MT8113(object_new(TYPE_MT8113));
    DriveInfo *di = drive_get(IF_SD, 0, 0);
    DeviceState *card = qdev_new(TYPE_EMMC);
    DeviceState *adc = qdev_new(TYPE_MT6577_AUXADC);
    I2CSlave *pmic;
    I2CSlave *epd;

    if (machine->ram_size != p->ram_size) {
        error_report("%s requires %" PRIu64 " MiB RAM",
                     p->description, p->ram_size / MiB);
        exit(EXIT_FAILURE);
    }
    s->soc = soc;
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_property_set_uint(OBJECT(soc), "reset-vector", 0x40001000,
                             &error_fatal);
    qdev_prop_set_uint32(DEVICE(soc), "num-cpus", p->cpus);
    qdev_prop_set_bit(DEVICE(&soc->gce), "inclusive-end-address", false);
    qdev_prop_set_bit(DEVICE(&soc->hwtcon), "scanout-image-buffer", true);
    qdev_prop_set_bit(DEVICE(&soc->hwtcon), "cfa-panel", p->cfa);
    qdev_prop_set_uint32(DEVICE(&soc->hwtcon), "initial-width", p->width);
    qdev_prop_set_uint32(DEVICE(&soc->hwtcon), "initial-height", p->height);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), MT8113_RAM_BASE,
                                machine->ram);
    object_property_add_child(OBJECT(machine), "auxadc", OBJECT(adc));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(adc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(adc), 0, 0x11001000);

    pmic = i2c_slave_new(TYPE_BD71828, 0x4b);
    object_property_add_child(OBJECT(machine), "pmic", OBJECT(pmic));
    qdev_prop_set_bit(DEVICE(pmic), "power-button-support", true);
    i2c_slave_realize_and_unref(pmic, soc->i2c[1].bus, &error_fatal);
    i2c_slave_create_simple(soc->i2c[1].bus, TYPE_BD71828, 0x4c);
    qdev_connect_gpio_out_named(DEVICE(pmic), "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 62));
    epd = i2c_slave_create_simple(soc->i2c[0].bus, p->epd_type, p->epd_address);
    qdev_connect_gpio_out_named(DEVICE(&soc->gpio), "gpio-out", p->epd_enable,
        qdev_get_gpio_in_named(DEVICE(epd), "enable", 0));
    qdev_connect_gpio_out_named(DEVICE(&soc->gpio), "gpio-out", p->epd_power,
        qdev_get_gpio_in_named(DEVICE(epd), "power", 0));
    qdev_connect_gpio_out_named(DEVICE(epd), "power-good", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", p->epd_power_good));
    if (p->page_keys) {
        I2CSlave *detector = i2c_slave_create_simple(soc->i2c[0].bus,
                                                   TYPE_MAX20342, 0x35);
        DeviceState *vbus = qdev_new(TYPE_SPLIT_IRQ);

        qdev_connect_gpio_out(DEVICE(detector), 0,
            qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 39));
        object_property_add_child(OBJECT(machine), "usb-vbus", OBJECT(vbus));
        qdev_prop_set_uint16(vbus, "num-lines", 2);
        qdev_realize_and_unref(vbus, NULL, &error_fatal);
        qdev_connect_gpio_out(vbus, 0,
            qdev_get_gpio_in_named(DEVICE(detector), "vbus", 0));
        qdev_connect_gpio_out(vbus, 1,
            qdev_get_gpio_in_named(DEVICE(pmic), "vbus", 0));
        qdev_connect_gpio_out_named(DEVICE(soc), "usb-vbus", 0,
            qdev_get_gpio_in(vbus, 0));
    } else {
        qdev_connect_gpio_out_named(DEVICE(soc), "usb-vbus", 0,
            qdev_get_gpio_in_named(DEVICE(pmic), "vbus", 0));
    }

    I2CSlave *touch;
    if (p->cypress) {
        touch = i2c_slave_new("cyttsp5", 0x24);
        qdev_prop_set_uint16(DEVICE(touch), "width", p->width);
        qdev_prop_set_uint16(DEVICE(touch), "height", p->height);
        /* Qt rotates the native landscape sensor coordinates by 270 degrees. */
        qdev_prop_set_bit(DEVICE(touch), "swap-axes", true);
        qdev_prop_set_bit(DEVICE(touch), "invert-x", false);
        /* Portrait input needs Y inverted before Qt's 270-degree rotation.
         * The Colour device tree already supplies that inversion.
         */
        qdev_prop_set_bit(DEVICE(touch), "invert-y", !p->cfa);
    } else {
        touch = i2c_slave_new("elan-ekth3500", 0x10);
        /* The stock E70T04 driver inverts X, then Qt rotates by 270 degrees. */
        qdev_prop_set_bit(DEVICE(touch), "invert-x", false);
        qdev_prop_set_uint8(DEVICE(touch), "x-traces", 54);
        qdev_prop_set_uint8(DEVICE(touch), "y-traces", 41);
        qdev_prop_set_uint16(DEVICE(touch), "width-mm", 142);
        qdev_prop_set_uint16(DEVICE(touch), "height-mm", 107);
    }
    i2c_slave_realize_and_unref(touch, soc->i2c[2].bus, &error_fatal);
    qdev_connect_gpio_out_named(DEVICE(touch), p->cypress ? NULL : "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", p->touch_irq));
    qdev_connect_gpio_out_named(DEVICE(&soc->gpio), "gpio-out", p->touch_reset,
        qdev_get_gpio_in_named(DEVICE(touch), p->cypress ? NULL : "reset", 0));

    I2CSlave *frontlight = i2c_slave_create_simple(
        soc->i2c[p->frontlight_bus].bus, p->frontlight_type, 0x36);
    qdev_connect_gpio_out_named(DEVICE(&soc->gpio), "gpio-out",
        p->frontlight_enable,
        qdev_get_gpio_in_named(DEVICE(frontlight), "enable", 0));
    if (p->page_keys) {
        I2CSlave *accel = i2c_slave_create_simple(soc->i2c[2].bus,
                                                 "kx122", 0x1e);
        DeviceState *buttons = qdev_new("forma-buttons");

        qdev_connect_gpio_out_named(DEVICE(accel), "irq", 0,
            qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 17));
        qdev_connect_gpio_out_named(DEVICE(accel), "irq", 1,
            qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 18));
        object_property_add_child(OBJECT(machine), "buttons", OBJECT(buttons));
        qdev_realize_and_unref(buttons, NULL, &error_fatal);
        qdev_connect_gpio_out(buttons, 0,
            qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 28));
        qdev_connect_gpio_out(buttons, 1,
            qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 29));
    }

    qdev_prop_set_uint64(card, "boot-partition-size", 4 * MiB);
    qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    qdev_prop_set_drive_err(card, "drive", di ? blk_by_legacy_dinfo(di) : NULL,
                            &error_fatal);
    qdev_realize(card, qdev_get_child_bus(DEVICE(&soc->msdc0), "sd-bus"),
                 &error_fatal);
    object_unref(OBJECT(card));
    kobo_mt8113_load_firmware(machine);
    qemu_register_reset(kobo_mt8113_reset, s);
}

static void kobo_mt8113_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    KoboMT8113MachineClass *kmc = KOBO_MT8113_MACHINE_CLASS(oc);
    const KoboMT8113Profile *p = data;

    kmc->profile = p;
    mc->desc = p->description;
    mc->init = kobo_mt8113_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a53");
    mc->default_ram_size = p->ram_size;
    mc->default_ram_id = "kobo_mt8113.ram";
    mc->default_cpus = mc->min_cpus = mc->max_cpus = p->cpus;
    mc->no_parallel = true;
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;
}

static const TypeInfo kobo_mt8113_types[] = {
    {
        .name = TYPE_KOBO_MT8113_MACHINE,
        .parent = TYPE_MACHINE,
        .abstract = true,
        .instance_size = sizeof(KoboMT8113MachineState),
        .class_size = sizeof(KoboMT8113MachineClass),
        .interfaces = aarch64_machine_interfaces,
    }, {
        .name = MACHINE_TYPE_NAME("mt8113-kobo-libra-colour"),
        .parent = TYPE_KOBO_MT8113_MACHINE,
        .class_init = kobo_mt8113_class_init,
        .class_data = &libra_colour,
    }, {
        .name = MACHINE_TYPE_NAME("mt8113-kobo-clara-bw"),
        .parent = TYPE_KOBO_MT8113_MACHINE,
        .class_init = kobo_mt8113_class_init,
        .class_data = &clara_bw,
    }, {
        .name = MACHINE_TYPE_NAME("mt8113-kobo-clara-colour"),
        .parent = TYPE_KOBO_MT8113_MACHINE,
        .class_init = kobo_mt8113_class_init,
        .class_data = &clara_colour,
    }, {
        .name = MACHINE_TYPE_NAME("mt8113-kobo-clara-bw-tpv"),
        .parent = TYPE_KOBO_MT8113_MACHINE,
        .class_init = kobo_mt8113_class_init,
        .class_data = &clara_bw_tpv,
    },
};
DEFINE_TYPES(kobo_mt8113_types)
