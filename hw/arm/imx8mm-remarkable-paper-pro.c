/* reMarkable Paper Pro (Ferrari). SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/arm/fsl-imx8mm.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "hw/char/remarkable_hwmon.h"
#include "hw/sd/sd.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "system/blockdev.h"
#include "system/reset.h"

#define TYPE_FERRARI_MACHINE MACHINE_TYPE_NAME("imx8mm-remarkable-paper-pro")
OBJECT_DECLARE_SIMPLE_TYPE(FerrariMachineState, FERRARI_MACHINE)
struct FerrariMachineState {
    MachineState parent_obj;
    FslIMX8MMState *soc;
    uint64_t entry;
    char *factory_image;
    char *hwmon_firmware;
};

static char *ferrari_get_factory(Object *obj, Error **errp)
{
    return g_strdup(FERRARI_MACHINE(obj)->factory_image);
}
static void ferrari_set_factory(Object *obj, const char *value, Error **errp)
{
    FerrariMachineState *s = FERRARI_MACHINE(obj);
    g_free(s->factory_image);
    s->factory_image = g_strdup(value);
}
static char *ferrari_get_hwmon(Object *obj, Error **errp)
{
    return g_strdup(FERRARI_MACHINE(obj)->hwmon_firmware);
}
static void ferrari_set_hwmon(Object *obj, const char *value, Error **errp)
{
    FerrariMachineState *s = FERRARI_MACHINE(obj);
    g_free(s->hwmon_firmware);
    s->hwmon_firmware = g_strdup(value);
}
static void ferrari_reset(void *opaque)
{
    FerrariMachineState *s = opaque;
    for (unsigned i = 0; i < MACHINE(s)->smp.cpus; i++) {
        cpu_reset(CPU(&s->soc->cpu[i]));
    }
    cpu_set_pc(CPU(&s->soc->cpu[0]), s->entry);
}
static void ferrari_init(MachineState *ms)
{
    FerrariMachineState *f = FERRARI_MACHINE(ms);
    FslIMX8MMState *s = f->soc = FSL_IMX8MM(object_new(TYPE_FSL_IMX8MM));
    g_autofree uint8_t *boot = NULL;
    g_autofree uint8_t *factory = NULL;
    g_autofree uint8_t *mcu = NULL;
    gsize length;
    gsize factory_length, mcu_length;
    DeviceState *card;
    DriveInfo *di;
    if (ms->ram_size != 2 * GiB) {
        error_report("Paper Pro requires 2 GiB RAM");
        exit(EXIT_FAILURE);
    }
    if (!f->factory_image ||
        !g_file_get_contents(f->factory_image, (char **)&factory, &factory_length, NULL) ||
        factory_length != 4 * MiB) {
        error_report("Paper Pro requires a provisioned factory boot partition (-M factory-image=)");
        exit(EXIT_FAILURE);
    }
    if (!f->hwmon_firmware ||
        !g_file_get_contents(f->hwmon_firmware, (char **)&mcu, &mcu_length, NULL) ||
        mcu_length < 0x110) {
        error_report("Paper Pro requires the original hardware-monitor firmware (-M hwmon-firmware=)");
        exit(EXIT_FAILURE);
    }
    if (!ms->firmware ||
        !g_file_get_contents(ms->firmware, (char **)&boot, &length, NULL) ||
        length < 0x40 || ldl_le_p(boot) != 0x412000d1) {
        error_report("Paper Pro requires its original imx-boot package (-bios)");
        exit(EXIT_FAILURE);
    }
    object_property_add_child(OBJECT(ms), "soc", OBJECT(s));
    qdev_prop_set_bit(DEVICE(&s->analog), "rom-boot-clocks", true);
    qdev_prop_set_uint32(DEVICE(&s->lcdif), "pixel-clock", 54000000);
    qdev_prop_set_bit(DEVICE(&s->lcdif), "latch-masked-frame-events", true);
    Chardev *hwmon = remarkable_hwmon_create(ldl_le_p(mcu + 0x10c));
    qdev_prop_set_chr(DEVICE(&s->uart[0]), "chardev", hwmon);
    for (unsigned i = 0; i < 5; i++) {
        qdev_prop_set_uint32(DEVICE(&s->gpio[i]), "reset-psr",
                             i == 2 ? UINT32_MAX & ~BIT(5) : UINT32_MAX);
    }
    for (unsigned i = 0; i < 3; i++) {
        qdev_prop_set_bit(DEVICE(&s->usdhc[i]), "defer-data-transfer", false);
        qdev_prop_set_bit(DEVICE(&s->usdhc[i]), "preserve-interrupt-enables-on-reset", true);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s), &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->gpio[2]), 12,
                          remarkable_hwmon_reset_input(hwmon));
    qdev_connect_gpio_out(DEVICE(&s->gpio[2]), 10,
                          remarkable_hwmon_wakeup_input(hwmon));
    remarkable_hwmon_connect_wakeup(hwmon,
        qdev_get_gpio_in(DEVICE(&s->gpio[2]), 5));
    memory_region_add_subregion(get_system_memory(), 0x40000000, ms->ram);
    DeviceState *panel = qdev_new("remarkable3-panel");
    object_property_add_child(OBJECT(ms), "panel", OBJECT(panel));
    object_property_set_link(OBJECT(panel), "lcdif", OBJECT(&s->lcdif), &error_fatal);
    ssi_realize_and_unref(panel, s->spi[1].bus, &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->gpio[4]), 13,
        qdev_get_gpio_in_named(panel, SSI_GPIO_CS, 0));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 26,
        qdev_get_gpio_in_named(panel, "program-n", 0));
    qdev_connect_gpio_out_named(panel, "init-n", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[3]), 22));
    qdev_connect_gpio_out_named(panel, "done", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[3]), 27));
    qdev_connect_gpio_out(DEVICE(&s->gpio[0]), 4,
        qdev_get_gpio_in_named(panel, "pmu", 0));
    qdev_connect_gpio_out_named(panel, "pmu-feedback", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[0]), 7));
    DeviceState *elan = qdev_new("elants-spi");
    object_property_add_child(OBJECT(ms), "digitizer", OBJECT(elan));
    qdev_prop_set_uint8(elan, "cs", 1);
    ssi_realize_and_unref(elan, s->spi[1].bus, &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 19,
        qdev_get_gpio_in_named(elan, SSI_GPIO_CS, 0));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 17,
        qdev_get_gpio_in_named(elan, "reset-n", 0));
    qdev_connect_gpio_out_named(elan, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[3]), 18));
    qdev_connect_gpio_out_named(elan, "power-key", 0,
        qdev_get_gpio_in_named(DEVICE(&s->snvs), "power-key", 0));
    i2c_slave_create_simple(s->i2c[3].bus, "pca9450", 0x25);
    i2c_slave_create_simple(s->i2c[0].bus, "fusb303b", 0x21);
    DeviceState *accelerometer = DEVICE(i2c_slave_create_simple(
        s->i2c[0].bus, "lis2dw12", 0x19));
    qdev_connect_gpio_out_named(accelerometer, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[2]), 2));
    DeviceState *backlight = DEVICE(i2c_slave_create_simple(s->i2c[0].bus, "aw99703", 0x36));
    qdev_connect_gpio_out(DEVICE(&s->gpio[2]), 24,
        qdev_get_gpio_in_named(backlight, "enable", 0));
    i2c_slave_create_simple(s->i2c[1].bus, "max17261", 0x36);
    DeviceState *charger = DEVICE(i2c_slave_create_simple(s->i2c[1].bus, "max77963", 0x69));
    qdev_connect_gpio_out_named(charger, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[3]), 11));
    i2c_slave_create_simple(s->i2c[1].bus, "g2194", 0x48);
    i2c_slave_create_simple(s->i2c[2].bus, "cumulus-bridge-i2c", 0x22);
    di = drive_get(IF_SD, 0, 0);
    if (!di) {
        error_report("Paper Pro requires eMMC at drive index 0");
        exit(EXIT_FAILURE);
    }
    card = qdev_new(TYPE_EMMC);
    qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di), &error_fatal);
    qdev_prop_set_uint64(card, "boot-partition-size", 4 * MiB);
    qdev_prop_set_uint8(card, "boot-config", 0x08);
    qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    qdev_prop_set_uint64(card, "rpmb-partition-size", 4 * MiB);
    qdev_prop_set_bit(card, "rpmb-in-memory", true);
    qdev_realize_and_unref(card, qdev_get_child_bus(DEVICE(&s->usdhc[0]), "sd-bus"), &error_fatal);
    emmc_boot_partition_write(card, 2, 0, factory, factory_length, &error_fatal);
    emmc_boot_partition_write(card, 1, 4 * MiB - 2048,
                              factory + factory_length - 2048, 2048, &error_fatal);
    emmc_boot_partition_write(card, 1, 33 * KiB, boot, length, &error_fatal);
    emmc_boot_partition_write(card, 2, 33 * KiB, boot, length, &error_fatal);
    /* ROM loads the signed SPL into TCM; DDR and subsequent FIT stages are
     * initialized and loaded by that original SPL. */
    f->entry = ldl_le_p(boot + 4);
    uint32_t self = ldl_le_p(boot + 0x14);
    size_t spl_length = MIN(length, 0x30000);
    if (self < 0x7c0000 || self + spl_length > 0x840000 ||
        f->entry < self || f->entry >= self + spl_length) {
        error_report("Paper Pro SPL IVT is outside TCM");
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("ferrari.spl", boot, spl_length, self);
    uint32_t pointer = cpu_to_le32(0x1000);
    uint8_t info[32] = {0, 0, 2, 0};
    stl_le_p(info + 4, 1800000000);
    stl_le_p(info + 8, 400000000);
    stl_le_p(info + 12, 1500000000);
    stl_le_p(info + 16, 8000000);
    rom_add_blob_fixed("ferrari.rom-info-pointer", &pointer, sizeof(pointer), 0x9e8);
    rom_add_blob_fixed("ferrari.rom-info", info, sizeof(info), 0x1000);
    qemu_register_reset(ferrari_reset, f);
}
static void ferrari_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "reMarkable Paper Pro (i.MX8M Mini, Ferrari)";
    mc->init = ferrari_init;
    mc->min_cpus = mc->max_cpus = mc->default_cpus = 4;
    mc->default_ram_size = 2 * GiB;
    mc->default_ram_id = "ferrari.ram";
    mc->block_default_type = IF_SD;
    mc->units_per_default_bus = 4;
    mc->minimum_page_bits = 12;
    mc->no_parallel = true;
    mc->auto_create_sdcard = false;
    object_class_property_add_str(oc, "factory-image", ferrari_get_factory, ferrari_set_factory);
    object_class_property_add_str(oc, "hwmon-firmware", ferrari_get_hwmon, ferrari_set_hwmon);
}
static const TypeInfo ferrari_info = {
    .name = TYPE_FERRARI_MACHINE, .parent = TYPE_MACHINE,
    .instance_size = sizeof(FerrariMachineState), .class_init = ferrari_class_init,
    .interfaces = arm_machine_interfaces,
};
static void ferrari_register(void) { type_register_static(&ferrari_info); }
type_init(ferrari_register)
