/* Kobo Clara HD (Netronix E60K02, i.MX6SLL).
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
#include "net/net.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/dma.h"
#include "system/reset.h"

#define CLARA_RAM_BASE 0x80000000
#define CLARA_SDMA_SIZE 0x1000
#define CLARA_SDMA_C0PTR 0x00
#define CLARA_SDMA_INTR 0x04
#define CLARA_SDMA_START 0x0c
#define CLARA_SDMA_RESET 0x24
#define CLARA_DCP_SIZE 0x4000
#define CLARA_DCP_CTRL 0x000
#define CLARA_DCP_SFTRST BIT(31)
#define CLARA_DCP_CLKGATE BIT(30)
#define TYPE_CLARA_MACHINE MACHINE_TYPE_NAME("imx6sll-kobo-clara-hd")
OBJECT_DECLARE_SIMPLE_TYPE(ClaraMachineState, CLARA_MACHINE)

struct ClaraMachineState {
    MachineState parent_obj;
    FslIMX6State *soc;
    MemoryRegion sdma_iomem;
    uint32_t sdma[CLARA_SDMA_SIZE / 4];
    qemu_irq sdma_irq;
    MemoryRegion dcp_iomem;
    uint32_t dcp[CLARA_DCP_SIZE / 4];
    uint32_t entry;
};
static void clara_sdma_complete_channel0(ClaraMachineState *rms)
{
    uint32_t ccb_addr = rms->sdma[CLARA_SDMA_C0PTR / 4];
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

    rms->sdma[CLARA_SDMA_INTR / 4] |= BIT(0);
    qemu_set_irq(rms->sdma_irq, 1);
}

static uint64_t clara_sdma_read(void *opaque, hwaddr offset, unsigned size)
{
    ClaraMachineState *rms = opaque;

    return rms->sdma[offset / 4];
}

static void clara_sdma_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    ClaraMachineState *rms = opaque;

    switch (offset) {
    case CLARA_SDMA_INTR:
        rms->sdma[offset / 4] &= ~(uint32_t)value;
        qemu_set_irq(rms->sdma_irq, rms->sdma[offset / 4] != 0);
        break;
    case CLARA_SDMA_START:
        if (value & BIT(0)) {
            clara_sdma_complete_channel0(rms);
        }
        break;
    case CLARA_SDMA_RESET:
        memset(rms->sdma, 0, sizeof(rms->sdma));
        qemu_set_irq(rms->sdma_irq, 0);
        break;
    default:
        rms->sdma[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps clara_sdma_ops = {
    .read = clara_sdma_read,
    .write = clara_sdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t clara_dcp_read(void *opaque, hwaddr offset, unsigned size)
{
    ClaraMachineState *rms = opaque;

    return rms->dcp[offset / 4];
}

static void clara_dcp_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    ClaraMachineState *rms = opaque;
    hwaddr base = offset & ~0xfULL;
    unsigned alias = (offset & 0xc) >> 2;
    uint32_t current = rms->dcp[base / 4];

    switch (alias) {
    case 0: current = value; break;
    case 1: current |= value; break;
    case 2: current &= ~(uint32_t)value; break;
    case 3: current ^= value; break;
    }
    if (base == CLARA_DCP_CTRL && (current & CLARA_DCP_SFTRST)) {
        memset(rms->dcp, 0, sizeof(rms->dcp));
        current = CLARA_DCP_SFTRST | CLARA_DCP_CLKGATE;
    }
    rms->dcp[base / 4] = current;
}

static const MemoryRegionOps clara_dcp_ops = {
    .read = clara_dcp_read,
    .write = clara_dcp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4,
               .unaligned = false },
};
static void clara_reset(void *opaque)
{
    ClaraMachineState *s = opaque;
    memset(s->sdma, 0, sizeof(s->sdma));
    memset(s->dcp, 0, sizeof(s->dcp));
    s->dcp[0] = CLARA_DCP_CLKGATE;
    qemu_set_irq(s->sdma_irq, 0);
    cpu_reset(CPU(&s->soc->cpu[0]));
    cpu_set_pc(CPU(&s->soc->cpu[0]), s->entry);
}

static void clara_load_firmware(ClaraMachineState *s, MachineState *ms)
{
    g_autofree uint8_t *image = NULL;
    gsize size;
    uint32_t self, entry;
    if (!ms->firmware ||
        !g_file_get_contents(ms->firmware, (char **)&image, &size, NULL) ||
        size < 0x20 || size > MiB || ldl_le_p(image) != 0x402000d1) {
        error_report("Clara HD requires an E60K02 U-Boot .imx image (-bios)");
        exit(EXIT_FAILURE);
    }
    entry = ldl_le_p(image + 4);
    self = ldl_le_p(image + 0x14);
    if (self < CLARA_RAM_BASE || self + size > CLARA_RAM_BASE + ms->ram_size ||
        entry < self || entry >= self + size) {
        error_report("Clara U-Boot IVT points outside its RAM image");
        exit(EXIT_FAILURE);
    }
    /* Reproduce the Boot ROM copy. DDR is already initialized in QEMU. */
    rom_add_blob_fixed("clara.uboot", image, size, self);
    s->entry = entry;
}

static void clara_init(MachineState *ms)
{
    ClaraMachineState *f = CLARA_MACHINE(ms);
    FslIMX6State *s = FSL_IMX6(object_new(TYPE_FSL_IMX6));
    DeviceState *dev, *pxp;
    DriveInfo *di;

    if (ms->ram_size != 512 * MiB) {
        error_report("Kobo Clara HD requires 512 MiB RAM");
        exit(EXIT_FAILURE);
    }
    f->soc = s;
    object_property_add_child(OBJECT(ms), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(s), "sololite", true, &error_fatal);
    object_property_set_bool(OBJECT(s), "sololite-lite", true, &error_fatal);
    object_property_set_bool(OBJECT(s), "has-el3", false, &error_fatal);
    object_property_set_bool(OBJECT(&s->usdhc[1]), "defer-data-transfer",
                             false, &error_fatal);
    for (int i = 1; i <= 2; i++) {
        object_property_set_bool(OBJECT(&s->usdhc[i]),
                                 "preserve-interrupt-enables-on-reset",
                                 true, &error_fatal);
    }
    object_property_set_uint(OBJECT(&s->src), "sbmr1", 0x860, &error_fatal);
    /* I2C pull-ups, released keys and open cover; EPDC power good. */
    for (int i = 0; i < FSL_IMX6_NUM_GPIOS; i++) {
        object_property_set_uint(OBJECT(&s->gpio[i]), "reset-psr",
                                 0xffffffff, &error_fatal);
    }
    object_property_set_uint(OBJECT(&s->gpio[2]), "reset-psr",
                             0xfffffffe, &error_fatal);
    qdev_realize(DEVICE(s), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), CLARA_RAM_BASE, ms->ram);

    f->sdma_irq = qdev_get_gpio_in(DEVICE(&s->a9mpcore), FSL_IMX6_SDMA_IRQ);
    memory_region_init_io(&f->sdma_iomem, OBJECT(ms), &clara_sdma_ops,
                          f, "clara.sdma", CLARA_SDMA_SIZE);
    memory_region_add_subregion(get_system_memory(), FSL_IMX6_SDMA_ADDR,
                                &f->sdma_iomem);
    memory_region_init_io(&f->dcp_iomem, OBJECT(ms), &clara_dcp_ops,
                          f, "clara.dcp", CLARA_DCP_SIZE);
    memory_region_add_subregion(get_system_memory(), 0x020fc000, &f->dcp_iomem);

    sysbus_create_simple(TYPE_IMX_RNGC, 0x021b4000,
                         qdev_get_gpio_in(DEVICE(&s->a9mpcore), 5));
    pxp = sysbus_create_simple(TYPE_IMX6SL_PXP, 0x020f0000,
                              qdev_get_gpio_in(DEVICE(&s->a9mpcore), 98));
    dev = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(ms), "epdc", OBJECT(dev));
    object_property_set_link(OBJECT(dev), "pxp", OBJECT(pxp), &error_fatal);
    /*
     * Stock Clara HD 4.1.15 allocates its mmap framebuffer here in CMA.
     * Boot starts in RGB565, then Nickel switches to a 32-bit surface.
     */
    qdev_prop_set_uint64(dev, "fb-addr", 0x9c800000);
    qdev_prop_set_uint32(dev, "fb-width", 1072);
    qdev_prop_set_uint32(dev, "fb-height", 1448);
    qdev_prop_set_uint32(dev, "fb-stride", 1088);
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
    dev = qdev_new(TYPE_SD_CARD);
    qdev_prop_set_drive_err(dev, "drive", di ? blk_by_legacy_dinfo(di) : NULL,
                            &error_fatal);
    qdev_realize_and_unref(dev,
                           qdev_get_child_bus(DEVICE(&s->usdhc[1]), "sd-bus"),
                           &error_fatal);

    dev = qdev_new("rtl8189fs-sdio");
    qemu_configure_nic_device(dev, true, "rtl8189fs");
    qdev_realize(dev,
                 qdev_get_child_bus(DEVICE(&s->usdhc[2]), "sd-bus"),
                 &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->gpio[4]), 0,
                          qdev_get_gpio_in_named(dev, "power", 0));
    qdev_connect_gpio_out_named(dev, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&s->usdhc[2]), "sdio-irq", 0));
    qdev_connect_gpio_out_named(dev, "oob-irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[3]), 31));
    object_unref(OBJECT(dev));

    i2c_slave_create_simple(s->i2c[2].bus, "forma-ricoh619", 0x32);
    i2c_slave_create_simple(s->i2c[1].bus, "forma-tps65185", 0x68);
    dev = qdev_new("cyttsp5");
    qdev_prop_set_uint8(dev, "address", 0x24);
    /* Nickel presents the landscape-mounted sensor clockwise. */
    qdev_prop_set_uint16(dev, "width", 1448);
    qdev_prop_set_uint16(dev, "height", 1072);
    qdev_prop_set_bit(dev, "swap-axes", true);
    qdev_prop_set_bit(dev, "invert-y", true);
    qdev_realize(dev, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(&s->gpio[4]), 6));
    qdev_connect_gpio_out(DEVICE(&s->gpio[4]), 13, qdev_get_gpio_in(dev, 0));
    object_unref(OBJECT(dev));
    dev = qdev_new("forma-buttons");
    qdev_realize(dev, NULL, &error_fatal);
    qdev_connect_gpio_out(dev, 2, qdev_get_gpio_in(DEVICE(&s->gpio[4]), 8));
    object_unref(OBJECT(dev));
    dev = DEVICE(i2c_slave_create_simple(s->i2c[0].bus, "lm3630a", 0x36));
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 10,
                         qdev_get_gpio_in_named(dev, "enable", 0));
    clara_load_firmware(f, ms);
    qemu_register_reset(clara_reset, f);
}

static void clara_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Kobo Clara HD (Netronix E60K02, i.MX6SLL)";
    mc->init = clara_init;
    mc->max_cpus = mc->default_cpus = 1;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "clara-hd.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;
}

static const TypeInfo clara_type = {
    .name = TYPE_CLARA_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(ClaraMachineState),
    .class_init = clara_class_init,
    .interfaces = arm_machine_interfaces,
};
static void clara_register_types(void)
{
    type_register_static(&clara_type);
}
type_init(clara_register_types)
