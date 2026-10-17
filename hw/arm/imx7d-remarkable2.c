/* reMarkable 2 (zero-sugar). SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/arm/fsl-imx7.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/bd71815.h"
#include "hw/display/imx_lcdif.h"
#include "hw/i2c/cyttsp5.h"
#include "hw/i2c/wacom_emr.h"
#include "hw/misc/imx7_ddrc.h"
#include "hw/sd/sd.h"
#include "system/blockdev.h"
#include "system/reset.h"
#define RM2_RAM_BASE 0x80000000
#define RM2_RAM_SIZE (1 * GiB)
#define TYPE_RM2_MACHINE MACHINE_TYPE_NAME("imx7d-remarkable2")
OBJECT_DECLARE_SIMPLE_TYPE(RM2MachineState, RM2_MACHINE)
struct RM2MachineState {
    MachineState parent_obj;
    FslIMX7State *soc;
    uint32_t entry;
};
static void rm2_reset(void *opaque)
{
    RM2MachineState *s = opaque;
    cpu_reset(CPU(&s->soc->cpu[0]));
    cpu_set_pc(CPU(&s->soc->cpu[0]), s->entry);
    s->soc->src.regs[SRC_SBMR1] = 0x2810;
}
static void rm2_init(MachineState *ms)
{
    RM2MachineState *f = RM2_MACHINE(ms);
    FslIMX7State *s = f->soc = FSL_IMX7(object_new(TYPE_FSL_IMX7));
    DeviceState *dev, *lcdif, *pen, *touch;
    DriveInfo *di;
    g_autofree uint8_t *image = NULL;
    gsize size;
    uint32_t self, offset;
    if (ms->ram_size != RM2_RAM_SIZE) {
        error_report("reMarkable 2 requires 1 GiB RAM");
        exit(EXIT_FAILURE);
    }
    object_property_add_child(OBJECT(ms), "soc", OBJECT(s));
    qdev_prop_set_bit(DEVICE(&s->usb[0]), "gadget", true);
    qdev_prop_set_bit(DEVICE(&s->usb[1]), "gadget", true);
    qdev_prop_set_uint32(DEVICE(&s->gpio[0]), "reset-psr", BIT(1) | BIT(7) | BIT(13) | BIT(14));
    qdev_prop_set_uint32(DEVICE(&s->gpio[5]), "reset-psr", BIT(16) | BIT(17) | BIT(19) | BIT(20) | BIT(21));
    for (unsigned i = 0; i < FSL_IMX7_NUM_USDHCS; i++) {
        qdev_prop_set_bit(DEVICE(&s->usdhc[i]), "defer-data-transfer", false);
        qdev_prop_set_bit(DEVICE(&s->usdhc[i]), "preserve-interrupt-enables-on-reset", true);
    }
    qdev_realize(DEVICE(s), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), RM2_RAM_BASE, ms->ram);
    lcdif = DEVICE(&s->lcdif);
    dev = qdev_new("remarkable2-panel");
    object_property_add_child(OBJECT(ms), "panel", OBJECT(dev));
    object_property_set_link(OBJECT(dev), "lcdif", OBJECT(lcdif), &error_fatal);
    qdev_realize_and_unref(dev, NULL, &error_fatal);
    dev = qdev_new(TYPE_IMX7_DDRC);
    object_property_add_child(OBJECT(ms), "ddrc", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x307a0000);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, 0x30790000);
    dev = qdev_new("imx7-syscnt");
    object_property_add_child(OBJECT(ms), "system-counter", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, FSL_IMX7_SYSCNT_CTRL_ADDR);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, FSL_IMX7_SYSCNT_RD_ADDR);
    dev = DEVICE(i2c_slave_create_simple(s->i2c[1].bus, TYPE_BD71815, 0x4b));
    qdev_connect_gpio_out_named(dev, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[5]), 16));
    for (unsigned i = 0; i < 3; i++) {
        static const unsigned addresses[] = { 0x66, 0x69, 0x36 };
        dev = DEVICE(i2c_slave_create_simple(s->i2c[1].bus, "max77818", addresses[i]));
        if (!i) {
            qdev_connect_gpio_out_named(dev, "irq", 0,
                qdev_get_gpio_in(DEVICE(&s->gpio[0]), 7));
        }
    }
    dev = DEVICE(i2c_slave_create_simple(s->i2c[3].bus, "sy7636", 0x62));
    qdev_connect_gpio_out(DEVICE(&s->gpio[6]), 10,
        qdev_get_gpio_in_named(dev, "power", 0));
    qdev_connect_gpio_out(DEVICE(&s->gpio[6]), 11,
        qdev_get_gpio_in_named(dev, "enable", 0));
    qdev_connect_gpio_out_named(dev, "power-good", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[5]), 21));
    pen = dev = qdev_new(TYPE_WACOM_EMR);
    object_property_add_child(OBJECT(ms), "digitizer", OBJECT(dev));
    qdev_prop_set_uint8(dev, "address", 0x09);
    qdev_prop_set_uint16(dev, "max-x", 20966);
    qdev_prop_set_uint16(dev, "max-y", 15725);
    qdev_prop_set_uint16(dev, "product-id", 0x95);
    qdev_prop_set_uint16(dev, "firmware-version", 0x1231);
    qdev_realize_and_unref(dev, BUS(s->i2c[0].bus), &error_fatal);
    qdev_connect_gpio_out_named(dev, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[0]), 1));
    touch = dev = qdev_new(TYPE_CYTTSP5);
    object_property_add_child(OBJECT(ms), "touchscreen", OBJECT(dev));
    qdev_prop_set_uint8(dev, "address", 0x24);
    qdev_prop_set_uint16(dev, "width", 880);
    qdev_prop_set_uint16(dev, "height", 1280);
    qdev_prop_set_bit(dev, "packed-touch-reports", true);
    qdev_realize_and_unref(dev, BUS(s->i2c[2].bus), &error_fatal);
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(&s->gpio[0]), 14));
    qdev_connect_gpio_out(DEVICE(&s->gpio[0]), 13, qdev_get_gpio_in(dev, 0));
    dev = qdev_new("remarkable-input");
    object_property_add_child(OBJECT(ms), "input", OBJECT(dev));
    object_property_set_link(OBJECT(dev), "pen", OBJECT(pen), &error_fatal);
    object_property_set_link(OBJECT(dev), "touch", OBJECT(touch), &error_fatal);
    qdev_prop_set_uint8(dev, "generation", 2);
    qdev_realize_and_unref(dev, NULL, &error_fatal);
    qdev_connect_gpio_out(dev, 3,
        qdev_get_gpio_in_named(DEVICE(&s->snvs), "power-key", 0));
    di = drive_get(IF_SD, 0, 2);
    if (!di) {
        error_report("reMarkable 2 requires an eMMC drive at index 2");
        exit(EXIT_FAILURE);
    }
    dev = qdev_new(TYPE_EMMC);
    qdev_prop_set_drive_err(dev, "drive", blk_by_legacy_dinfo(di), &error_fatal);
    qdev_prop_set_uint64(dev, "boot-partition-size", 4 * MiB);
    qdev_prop_set_bit(dev, "boot-partitions-in-memory", true);
    qdev_realize_and_unref(dev,
        qdev_get_child_bus(DEVICE(&s->usdhc[2]), "sd-bus"), &error_fatal);
    {
        static const char *serials[] = {
            "RM11000000000000", "RM2QEMU00000000000000",
            "BATQEMU00000000000000", "ED103TC2M1AFAB210000-1.25",
        };
        uint8_t factory[512] = {0};
        size_t position = 0;
        for (unsigned i = 0; i < ARRAY_SIZE(serials); i++) {
            size_t length = strlen(serials[i]);
            stl_be_p(factory + position, length);
            memcpy(factory + position + 4, serials[i], length);
            position += 4 + length;
        }
        emmc_boot_partition_write(dev, 2, 0, factory, sizeof(factory), &error_fatal);
    }
    if (!ms->firmware ||
        !g_file_get_contents(ms->firmware, (char **)&image, &size, NULL) ||
        size < 0x2c || ldl_le_p(image) != 0x402000d1) {
        error_report("reMarkable 2 requires its vendor U-Boot image (-bios)");
        exit(EXIT_FAILURE);
    }
    /* Boot ROM's OCRAM plugin initializes DDR, then loads the second IVT. */
    for (offset = 4; offset + 32 <= size; offset += 4) {
        if (ldl_le_p(image + offset) == 0x402000d1 &&
            ldl_le_p(image + offset + 0x14) >= RM2_RAM_BASE) {
            break;
        }
    }
    if (offset + 32 > size) {
        error_report("reMarkable 2 firmware has no DDR U-Boot IVT");
        exit(EXIT_FAILURE);
    }
    self = ldl_le_p(image + offset + 0x14) - offset;
    f->entry = ldl_le_p(image + offset + 4);
    if (self < RM2_RAM_BASE || self >= RM2_RAM_BASE + RM2_RAM_SIZE ||
        size > RM2_RAM_BASE + RM2_RAM_SIZE - self ||
        f->entry < self || f->entry - self >= size) {
        error_report("reMarkable 2 U-Boot IVT is outside RAM");
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("remarkable2.uboot", image, size, self);
    {
        uint32_t pointer = cpu_to_le32(0x200);
        uint8_t info[32] = { 0, 2, 2, 0 };
        stl_le_p(info + 4, 1000000000);
        stl_le_p(info + 8, 332000000);
        stl_le_p(info + 12, 400000000);
        stl_le_p(info + 16, 24000000);
        rom_add_blob_fixed("remarkable2.rom-info-pointer", &pointer, 4, 0x1e8);
        rom_add_blob_fixed("remarkable2.rom-info", info, sizeof(info), 0x200);
    }
    qemu_register_reset(rm2_reset, f);
}
static void rm2_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "reMarkable 2 (i.MX7D, zero-sugar)";
    mc->init = rm2_init;
    mc->max_cpus = mc->default_cpus = 2;
    mc->default_ram_size = RM2_RAM_SIZE;
    mc->default_ram_id = "remarkable2.ram";
    mc->block_default_type = IF_SD;
    mc->units_per_default_bus = 4;
    mc->minimum_page_bits = 12;
    mc->no_parallel = true;
    mc->auto_create_sdcard = false;
}
static const TypeInfo rm2_info = {
    .name = TYPE_RM2_MACHINE, .parent = TYPE_MACHINE,
    .instance_size = sizeof(RM2MachineState), .class_init = rm2_class_init,
    .interfaces = arm_machine_interfaces,
};
static void rm2_register(void) { type_register_static(&rm2_info); }
type_init(rm2_register)
