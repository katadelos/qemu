/*
 * reMarkable 1 (zero-gravitas) board.
 * Original U-Boot IVT/DCD image, stock device tree and reMarkable OS.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "hw/arm/fsl-imx6.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/imx_epdc.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/cyttsp5.h"
#include "hw/i2c/wacom_emr.h"
#include "hw/misc/imx6sl_iomuxc.h"
#include "hw/misc/imx6sl_mmdc.h"
#include "hw/misc/imx6sl_pxp.h"
#include "hw/misc/imx6sll_ocotp.h"
#include "hw/misc/imx_rngc.h"
#include "hw/sd/sd.h"
#include "hw/usb/imx-usb-misc.h"
#include "migration/vmstate.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "ui/input.h"

#define RM1_RAM_BASE 0x80000000
#define RM1_RAM_SIZE (512 * MiB)
#define TYPE_RM1_MACHINE MACHINE_TYPE_NAME("imx6sl-remarkable1")
OBJECT_DECLARE_SIMPLE_TYPE(RM1MachineState, RM1_MACHINE)

struct RM1MachineState {
    MachineState parent_obj;
    FslIMX6State *soc;
    uint32_t entry;
};


static void rm1_reset(void *opaque)
{
    RM1MachineState *s = opaque;
    cpu_reset(CPU(&s->soc->cpu[0]));
    cpu_set_pc(CPU(&s->soc->cpu[0]), s->entry);
}

static void rm1_init(MachineState *ms)
{
    RM1MachineState *f = RM1_MACHINE(ms);
    FslIMX6State *s = f->soc = FSL_IMX6(object_new(TYPE_FSL_IMX6));
    DeviceState *dev, *pxp, *pen, *touch;
    DriveInfo *di;
    g_autofree uint8_t *image = NULL;
    gsize size;
    uint32_t self;

    if (ms->ram_size != RM1_RAM_SIZE) {
        error_report("reMarkable 1 requires 512 MiB RAM");
        exit(EXIT_FAILURE);
    }
    object_property_add_child(OBJECT(ms), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(s), "sololite", true, &error_fatal);
    object_property_set_bool(OBJECT(s), "has-el3", false, &error_fatal);
    object_property_set_uint(OBJECT(&s->src), "sbmr1", 0x2840, &error_fatal);
    qdev_prop_set_uint32(DEVICE(&s->gpio[1]), "reset-psr", BIT(10) | BIT(12));
    qdev_prop_set_uint32(DEVICE(&s->gpio[2]), "reset-psr",
                         BIT(12) | BIT(13) | BIT(24) | BIT(26) | BIT(28) | BIT(31));
    qdev_prop_set_uint32(DEVICE(&s->gpio[3]), "reset-psr",
                         BIT(3) | BIT(4) | BIT(5) | BIT(7) | BIT(9));
    for (unsigned i = 0; i < FSL_IMX6_NUM_USDHCS; i++) {
        qdev_prop_set_bit(DEVICE(&s->usdhc[i]), "defer-data-transfer", false);
        qdev_prop_set_bit(DEVICE(&s->usdhc[i]),
                          "preserve-interrupt-enables-on-reset", true);
    }
    qdev_realize(DEVICE(s), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), RM1_RAM_BASE, ms->ram);
    dev = qdev_new(TYPE_IMX6SL_MMDC);
    qdev_prop_set_uint64(dev, "ram-size", ms->ram_size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x021b0000);
    sysbus_create_simple(TYPE_IMX6SL_IOMUXC, 0x020e0000, NULL);
    dev = sysbus_create_simple(TYPE_IMX6SLL_OCOTP, 0x021bc000, NULL);
    imx6sll_ocotp_set_fuse(IMX6SLL_OCOTP(dev), 1, 6, 0x5f055a64);
    sysbus_create_simple(TYPE_IMX_RNGC, 0x021b4000,
        qdev_get_gpio_in(DEVICE(&s->a9mpcore), 5));
    pxp = sysbus_create_simple(TYPE_IMX6SL_PXP, 0x020f0000,
        qdev_get_gpio_in(DEVICE(&s->a9mpcore), 98));
    dev = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(ms), "epdc", OBJECT(dev));
    qdev_prop_set_uint16(dev, "landscape-rotation", 270);
    object_property_set_link(OBJECT(dev), "pxp", OBJECT(pxp), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x020f4000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
        qdev_get_gpio_in(DEVICE(&s->a9mpcore), 97));
    dev = qdev_new(TYPE_IMX_USB_MISC);
    qdev_prop_set_bit(dev, "imx6-layout", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, 0x02184800, 2);
    i2c_slave_create_simple(s->i2c[0].bus, "pfuze100", 0x08);
    i2c_slave_create_simple(s->i2c[0].bus, "bq27441", 0x55);
    dev = DEVICE(i2c_slave_create_simple(s->i2c[0].bus, "max17135", 0x48));
    qdev_connect_gpio_out_named(dev, "pwrgood", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[1]), 13));
    qdev_connect_gpio_out_named(dev, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[1]), 12));
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 14,
        qdev_get_gpio_in_named(dev, "wakeup", 0));
    pen = dev = qdev_new(TYPE_WACOM_W9013);
    object_property_add_child(OBJECT(ms), "digitizer", OBJECT(dev));
    qdev_prop_set_uint8(dev, "address", 0x09);
    qdev_realize_and_unref(dev, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out_named(dev, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[1]), 10));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 4,
        qdev_get_gpio_in_named(dev, "reset", 0));
    touch = dev = qdev_new(TYPE_CYTTSP5);
    object_property_add_child(OBJECT(ms), "touchscreen", OBJECT(dev));
    qdev_prop_set_uint16(dev, "width", 880);
    qdev_prop_set_uint16(dev, "height", 1280);
    qdev_prop_set_bit(dev, "invert-x", true);
    qdev_prop_set_uint8(dev, "address", 0x24);
    qdev_realize_and_unref(dev, BUS(s->i2c[2].bus), &error_fatal);
    qdev_connect_gpio_out(dev, 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[3]), 3));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 5, qdev_get_gpio_in(dev, 0));
    dev = qdev_new("remarkable-input");
    object_property_add_child(OBJECT(ms), "input", OBJECT(dev));
    object_property_set_link(OBJECT(dev), "pen", OBJECT(pen), &error_fatal);
    object_property_set_link(OBJECT(dev), "touch", OBJECT(touch), &error_fatal);
    qdev_realize_and_unref(dev, NULL, &error_fatal);
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(&s->gpio[2]), 24));
    qdev_connect_gpio_out(dev, 1, qdev_get_gpio_in(DEVICE(&s->gpio[2]), 26));
    qdev_connect_gpio_out(dev, 2, qdev_get_gpio_in(DEVICE(&s->gpio[2]), 28));
    qdev_connect_gpio_out(dev, 3, qdev_get_gpio_in(DEVICE(&s->gpio[3]), 9));
    di = drive_get(IF_SD, 0, 1);
    if (!di) {
        error_report("reMarkable 1 requires an eMMC drive at index 1");
        exit(EXIT_FAILURE);
    }
    dev = qdev_new(TYPE_EMMC);
    qdev_prop_set_drive_err(dev, "drive", blk_by_legacy_dinfo(di), &error_fatal);
    qdev_realize_and_unref(dev,
        qdev_get_child_bus(DEVICE(&s->usdhc[1]), "sd-bus"), &error_fatal);
    if (!ms->firmware ||
        !g_file_get_contents(ms->firmware, (char **)&image, &size, NULL) ||
        size < 0x2c || ldl_le_p(image) != 0x402000d1) {
        error_report("reMarkable 1 requires its vendor U-Boot image (-bios)");
        exit(EXIT_FAILURE);
    }
    self = ldl_le_p(image + 0x14);
    f->entry = ldl_le_p(image + 4);
    if (self < RM1_RAM_BASE || self >= RM1_RAM_BASE + RM1_RAM_SIZE ||
        size > RM1_RAM_BASE + RM1_RAM_SIZE - self ||
        f->entry < self || f->entry - self >= size) {
        error_report("reMarkable 1 U-Boot IVT is outside RAM");
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("remarkable1.uboot", image, size, self);
    qemu_register_reset(rm1_reset, f);
}
static void rm1_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "reMarkable 1 (i.MX6SoloLite, zero-gravitas)";
    mc->init = rm1_init;
    mc->max_cpus = mc->default_cpus = 1;
    mc->default_ram_size = RM1_RAM_SIZE;
    mc->default_ram_id = "remarkable1.ram";
    mc->block_default_type = IF_SD;
    mc->units_per_default_bus = 4;
    mc->no_parallel = true;
}
static const TypeInfo rm1_types[] = {
    {
        .name = TYPE_RM1_MACHINE, .parent = TYPE_MACHINE,
        .instance_size = sizeof(RM1MachineState), .class_init = rm1_class_init,
        .interfaces = arm_machine_interfaces,
    },
};
DEFINE_TYPES(rm1_types)
