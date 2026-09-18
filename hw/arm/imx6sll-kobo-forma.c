/* Kobo Forma (Netronix E80K02, i.MX6SLL).
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/arm/fsl-imx6.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/imx_epdc.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/imx6sll_ocotp.h"
#include "hw/misc/imx6sl_iomuxc.h"
#include "hw/misc/imx6sl_mmdc.h"
#include "hw/misc/imx6sl_pxp.h"
#include "hw/misc/imx_rngc.h"
#include "hw/sd/sd.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/dma.h"
#include "system/reset.h"

#define FORMA_RAM_BASE 0x80000000
#define FORMA_SDMA_SIZE 0x1000
#define FORMA_SDMA_C0PTR 0x00
#define FORMA_SDMA_INTR 0x04
#define FORMA_SDMA_START 0x0c
#define FORMA_SDMA_RESET 0x24
#define FORMA_DCP_SIZE 0x4000
#define FORMA_DCP_CTRL 0x000
#define FORMA_DCP_SFTRST BIT(31)
#define FORMA_DCP_CLKGATE BIT(30)
#define TYPE_FORMA_MACHINE MACHINE_TYPE_NAME("imx6sll-kobo-forma")
OBJECT_DECLARE_SIMPLE_TYPE(FormaMachineState, FORMA_MACHINE)

struct FormaMachineState {
    MachineState parent_obj;
    FslIMX6State *soc;
    MemoryRegion sdma_iomem;
    uint32_t sdma[FORMA_SDMA_SIZE / 4];
    qemu_irq sdma_irq;
    MemoryRegion dcp_iomem;
    uint32_t dcp[FORMA_DCP_SIZE / 4];
    uint32_t entry;
};
static void forma_sdma_complete_channel0(FormaMachineState *rms)
{
    uint32_t ccb_addr = rms->sdma[FORMA_SDMA_C0PTR / 4];
    uint32_t bd_addr;
    uint32_t mode;

    /* Complete the channel-zero host command descriptor synchronously. */
    if (dma_memory_read(&address_space_memory, ccb_addr + 4, &bd_addr,
                        sizeof(bd_addr), MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
        bd_addr = le32_to_cpu(bd_addr);
        if (bd_addr &&
            dma_memory_read(&address_space_memory, bd_addr, &mode,
                            sizeof(mode), MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
            mode = cpu_to_le32(le32_to_cpu(mode) & ~BIT(16));
            dma_memory_write(&address_space_memory, bd_addr, &mode,
                             sizeof(mode), MEMTXATTRS_UNSPECIFIED);
        }
    }

    rms->sdma[FORMA_SDMA_INTR / 4] |= BIT(0);
    qemu_set_irq(rms->sdma_irq, 1);
}

static uint64_t forma_sdma_read(void *opaque, hwaddr offset, unsigned size)
{
    FormaMachineState *rms = opaque;

    return rms->sdma[offset / 4];
}

static void forma_sdma_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    FormaMachineState *rms = opaque;

    switch (offset) {
    case FORMA_SDMA_INTR:
        rms->sdma[offset / 4] &= ~(uint32_t)value;
        qemu_set_irq(rms->sdma_irq, rms->sdma[offset / 4] != 0);
        break;
    case FORMA_SDMA_START:
        if (value & BIT(0)) {
            forma_sdma_complete_channel0(rms);
        }
        break;
    case FORMA_SDMA_RESET:
        memset(rms->sdma, 0, sizeof(rms->sdma));
        qemu_set_irq(rms->sdma_irq, 0);
        break;
    default:
        rms->sdma[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps forma_sdma_ops = {
    .read = forma_sdma_read,
    .write = forma_sdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t forma_dcp_read(void *opaque, hwaddr offset, unsigned size)
{
    FormaMachineState *rms = opaque;

    return rms->dcp[offset / 4];
}

static void forma_dcp_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    FormaMachineState *rms = opaque;
    hwaddr base = offset & ~0xfULL;
    unsigned alias = (offset & 0xc) >> 2;
    uint32_t current = rms->dcp[base / 4];

    switch (alias) {
    case 0: current = value; break;
    case 1: current |= value; break;
    case 2: current &= ~(uint32_t)value; break;
    case 3: current ^= value; break;
    }
    if (base == FORMA_DCP_CTRL && (current & FORMA_DCP_SFTRST)) {
        memset(rms->dcp, 0, sizeof(rms->dcp));
        current = FORMA_DCP_SFTRST | FORMA_DCP_CLKGATE;
    }
    rms->dcp[base / 4] = current;
}

static const MemoryRegionOps forma_dcp_ops = {
    .read = forma_dcp_read,
    .write = forma_dcp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4,
               .unaligned = false },
};
static void forma_reset(void *opaque)
{
    FormaMachineState *s = opaque;
    memset(s->sdma, 0, sizeof(s->sdma));
    memset(s->dcp, 0, sizeof(s->dcp));
    s->dcp[0] = FORMA_DCP_CLKGATE;
    qemu_set_irq(s->sdma_irq, 0);
    cpu_reset(CPU(&s->soc->cpu[0]));
    cpu_set_pc(CPU(&s->soc->cpu[0]), s->entry);
}

static void forma_load_firmware(FormaMachineState *s, MachineState *ms)
{
    g_autofree uint8_t *image = NULL;
    gsize size;
    uint32_t self, entry;
    if (!ms->firmware ||
        !g_file_get_contents(ms->firmware, (char **)&image, &size, NULL) ||
        size < 0x20 || size > MiB || ldl_le_p(image) != 0x402000d1) {
        error_report("Forma requires an E80K00 U-Boot .imx image (-bios)");
        exit(EXIT_FAILURE);
    }
    entry = ldl_le_p(image + 4);
    self = ldl_le_p(image + 0x14);
    if (self < FORMA_RAM_BASE || self + size > FORMA_RAM_BASE + ms->ram_size ||
        entry < self || entry >= self + size) {
        error_report("Forma U-Boot IVT points outside its RAM image");
        exit(EXIT_FAILURE);
    }
    /* Reproduce the Boot ROM copy. DDR is already initialized in QEMU. */
    rom_add_blob_fixed("forma.uboot", image, size, self);
    s->entry = entry;
}

static void forma_init(MachineState *ms)
{
    FormaMachineState *f = FORMA_MACHINE(ms);
    FslIMX6State *s = FSL_IMX6(object_new(TYPE_FSL_IMX6));
    DeviceState *dev, *pxp;
    DriveInfo *di;

    if (ms->ram_size != 512 * MiB) {
        error_report("Kobo Forma requires 512 MiB RAM");
        exit(EXIT_FAILURE);
    }
    f->soc = s;
    object_property_add_child(OBJECT(ms), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(s), "sololite", true, &error_fatal);
    object_property_set_bool(OBJECT(s), "sololite-lite", true, &error_fatal);
    object_property_set_bool(OBJECT(s), "has-el3", false, &error_fatal);
    object_property_set_bool(OBJECT(&s->usdhc[0]), "defer-data-transfer",
                             false, &error_fatal);
    object_property_set_uint(OBJECT(&s->src), "sbmr1", 0x60, &error_fatal);
    /* I2C pull-ups, released keys and open cover; EPDC power good. */
    for (int i = 0; i < FSL_IMX6_NUM_GPIOS; i++) {
        object_property_set_uint(OBJECT(&s->gpio[i]), "reset-psr",
                                 0xffffffff, &error_fatal);
    }
    object_property_set_uint(OBJECT(&s->gpio[1]), "reset-psr",
                             0x7fffffff, &error_fatal);
    object_property_set_uint(OBJECT(&s->gpio[2]), "reset-psr",
                             0xfffffffe, &error_fatal);
    qdev_realize(DEVICE(s), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), FORMA_RAM_BASE, ms->ram);

    f->sdma_irq = qdev_get_gpio_in(DEVICE(&s->a9mpcore), FSL_IMX6_SDMA_IRQ);
    memory_region_init_io(&f->sdma_iomem, OBJECT(ms), &forma_sdma_ops,
                          f, "forma.sdma", FORMA_SDMA_SIZE);
    memory_region_add_subregion(get_system_memory(), FSL_IMX6_SDMA_ADDR,
                                &f->sdma_iomem);
    memory_region_init_io(&f->dcp_iomem, OBJECT(ms), &forma_dcp_ops,
                          f, "forma.dcp", FORMA_DCP_SIZE);
    memory_region_add_subregion(get_system_memory(), 0x020fc000, &f->dcp_iomem);

    sysbus_create_simple(TYPE_IMX_RNGC, 0x021b4000,
                         qdev_get_gpio_in(DEVICE(&s->a9mpcore), 5));
    pxp = sysbus_create_simple(TYPE_IMX6SL_PXP, 0x020f0000,
                              qdev_get_gpio_in(DEVICE(&s->a9mpcore), 98));
    dev = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(ms), "epdc", OBJECT(dev));
    object_property_set_link(OBJECT(dev), "pxp", OBJECT(pxp), &error_fatal);
    /*
     * Stock Forma 4.1.15 reserves its mmap framebuffer at this address.
     * Boot starts in RGB565, then Nickel switches to a 32-bit surface.
     */
    qdev_prop_set_uint64(dev, "fb-addr", 0x9d000000);
    qdev_prop_set_uint32(dev, "fb-width", 1440);
    qdev_prop_set_uint32(dev, "fb-height", 1920);
    qdev_prop_set_uint32(dev, "fb-stride", 1440);
    qdev_prop_set_uint8(dev, "fb-bpp", 2);
    qdev_prop_set_bit(dev, "fb-follow-pxp", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x020f4000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                      qdev_get_gpio_in(DEVICE(&s->a9mpcore), 97));
    sysbus_create_simple(TYPE_IMX6SLL_OCOTP, FSL_IMX6_OCOTPCTRL_ADDR, NULL);
    sysbus_create_simple(TYPE_IMX6SL_MMDC, 0x021b0000, NULL);
    sysbus_create_simple(TYPE_IMX6SL_IOMUXC, 0x020e0000, NULL);

    di = drive_get(IF_SD, 0, 0);
    dev = qdev_new(TYPE_EMMC);
    qdev_prop_set_uint64(dev, "boot-partition-size", 4 * MiB);
    qdev_prop_set_bit(dev, "boot-partitions-in-memory", true);
    qdev_prop_set_drive_err(dev, "drive", di ? blk_by_legacy_dinfo(di) : NULL,
                            &error_fatal);
    qdev_realize_and_unref(dev,
                           qdev_get_child_bus(DEVICE(&s->usdhc[0]), "sd-bus"),
                           &error_fatal);

    i2c_slave_create_simple(s->i2c[2].bus, "forma-ricoh619", 0x32);
    i2c_slave_create_simple(s->i2c[1].bus, "forma-tps65185", 0x68);
    dev = qdev_new("cyttsp5");
    qdev_prop_set_uint8(dev, "address", 0x24);
    qdev_realize(dev, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(&s->gpio[3]), 24));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 18, qdev_get_gpio_in(dev, 0));
    object_unref(OBJECT(dev));
    dev = qdev_new("forma-buttons");
    qdev_realize(dev, NULL, &error_fatal);
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(&s->gpio[3]), 0));
    qdev_connect_gpio_out(dev, 1, qdev_get_gpio_in(DEVICE(&s->gpio[3]), 2));
    qdev_connect_gpio_out(dev, 2, qdev_get_gpio_in(DEVICE(&s->gpio[3]), 25));
    object_unref(OBJECT(dev));
    forma_load_firmware(f, ms);
    qemu_register_reset(forma_reset, f);
}

static void forma_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Kobo Forma (Netronix E80K02, i.MX6SLL)";
    mc->init = forma_init;
    mc->max_cpus = mc->default_cpus = 1;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "forma.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;
}

static const TypeInfo forma_type = {
    .name = TYPE_FORMA_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(FormaMachineState),
    .class_init = forma_class_init,
    .interfaces = arm_machine_interfaces,
};
static void forma_register_types(void)
{
    type_register_static(&forma_type);
}
type_init(forma_register_types)
