/* Kobo Glo HD (Netronix E60Q9X, i.MX6SL).
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
#include "hw/i2c/zforce.h"
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

#define GLOHD_RAM_BASE 0x80000000
#define GLOHD_SDMA_SIZE 0x1000
#define GLOHD_SDMA_C0PTR 0x00
#define GLOHD_SDMA_INTR 0x04
#define GLOHD_SDMA_START 0x0c
#define GLOHD_SDMA_RESET 0x24
#define GLOHD_DCP_SIZE 0x4000
#define GLOHD_DCP_CTRL 0x000
#define GLOHD_DCP_SFTRST BIT(31)
#define GLOHD_DCP_CLKGATE BIT(30)
#define TYPE_GLOHD_MACHINE MACHINE_TYPE_NAME("imx6sl-kobo-glo-hd")
OBJECT_DECLARE_SIMPLE_TYPE(GloHDMachineState, GLOHD_MACHINE)

struct GloHDMachineState {
    MachineState parent_obj;
    FslIMX6State *soc;
    MemoryRegion sdma_iomem;
    uint32_t sdma[GLOHD_SDMA_SIZE / 4];
    qemu_irq sdma_irq;
    MemoryRegion dcp_iomem;
    uint32_t dcp[GLOHD_DCP_SIZE / 4];
    uint32_t entry;
};
static void glohd_sdma_complete_channel0(GloHDMachineState *rms)
{
    uint32_t ccb_addr = rms->sdma[GLOHD_SDMA_C0PTR / 4];
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

    rms->sdma[GLOHD_SDMA_INTR / 4] |= BIT(0);
    qemu_set_irq(rms->sdma_irq, 1);
}

static uint64_t glohd_sdma_read(void *opaque, hwaddr offset, unsigned size)
{
    GloHDMachineState *rms = opaque;

    return rms->sdma[offset / 4];
}

static void glohd_sdma_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    GloHDMachineState *rms = opaque;

    switch (offset) {
    case GLOHD_SDMA_INTR:
        rms->sdma[offset / 4] &= ~(uint32_t)value;
        qemu_set_irq(rms->sdma_irq, rms->sdma[offset / 4] != 0);
        break;
    case GLOHD_SDMA_START:
        if (value & BIT(0)) {
            glohd_sdma_complete_channel0(rms);
        }
        break;
    case GLOHD_SDMA_RESET:
        memset(rms->sdma, 0, sizeof(rms->sdma));
        qemu_set_irq(rms->sdma_irq, 0);
        break;
    default:
        rms->sdma[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps glohd_sdma_ops = {
    .read = glohd_sdma_read,
    .write = glohd_sdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t glohd_dcp_read(void *opaque, hwaddr offset, unsigned size)
{
    GloHDMachineState *rms = opaque;

    return rms->dcp[offset / 4];
}

static void glohd_dcp_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    GloHDMachineState *rms = opaque;
    hwaddr base = offset & ~0xfULL;
    unsigned alias = (offset & 0xc) >> 2;
    uint32_t current = rms->dcp[base / 4];

    switch (alias) {
    case 0: current = value; break;
    case 1: current |= value; break;
    case 2: current &= ~(uint32_t)value; break;
    case 3: current ^= value; break;
    }
    if (base == GLOHD_DCP_CTRL && (current & GLOHD_DCP_SFTRST)) {
        memset(rms->dcp, 0, sizeof(rms->dcp));
        current = GLOHD_DCP_SFTRST | GLOHD_DCP_CLKGATE;
    }
    rms->dcp[base / 4] = current;
}

static const MemoryRegionOps glohd_dcp_ops = {
    .read = glohd_dcp_read,
    .write = glohd_dcp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4,
               .unaligned = false },
};
static void glohd_reset(void *opaque)
{
    GloHDMachineState *s = opaque;
    memset(s->sdma, 0, sizeof(s->sdma));
    memset(s->dcp, 0, sizeof(s->dcp));
    s->dcp[0] = GLOHD_DCP_CLKGATE;
    qemu_set_irq(s->sdma_irq, 0);
    cpu_reset(CPU(&s->soc->cpu[0]));
    cpu_set_pc(CPU(&s->soc->cpu[0]), s->entry);
}

static void glohd_load_firmware(GloHDMachineState *s, MachineState *ms)
{
    g_autofree uint8_t *image = NULL;
    gsize size;
    uint32_t self, entry;
    if (!ms->firmware ||
        !g_file_get_contents(ms->firmware, (char **)&image, &size, NULL) ||
        size < 0x20 || size > MiB || ldl_le_p(image) != 0x402000d1) {
        error_report("Glo HD requires an E60Q9X U-Boot .imx image (-bios)");
        exit(EXIT_FAILURE);
    }
    entry = ldl_le_p(image + 4);
    self = ldl_le_p(image + 0x14);
    if (self < GLOHD_RAM_BASE || self + size > GLOHD_RAM_BASE + ms->ram_size ||
        entry < self || entry >= self + size) {
        error_report("Glo HD U-Boot IVT points outside its RAM image");
        exit(EXIT_FAILURE);
    }
    /* Reproduce the Boot ROM copy. DDR is already initialized in QEMU. */
    rom_add_blob_fixed("glohd.uboot", image, size, self);
    s->entry = entry;
}

static void glohd_init(MachineState *ms)
{
    GloHDMachineState *f = GLOHD_MACHINE(ms);
    FslIMX6State *s = FSL_IMX6(object_new(TYPE_FSL_IMX6));
    DeviceState *dev, *pxp;
    DriveInfo *di;

    if (ms->ram_size != 512 * MiB) {
        error_report("Kobo Glo HD requires 512 MiB RAM");
        exit(EXIT_FAILURE);
    }
    f->soc = s;
    object_property_add_child(OBJECT(ms), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(s), "sololite", true, &error_fatal);
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
    /* Boot GPIOs select SD (GPIO2_26:25=10), on physical USDHC2
     * (GPIO3_0:GPIO2_31=01). U-Boot exposes the selected card as mmc 0. */
    object_property_set_uint(OBJECT(&s->gpio[1]), "reset-psr",
                             0xfdffffff, &error_fatal);
    object_property_set_uint(OBJECT(&s->gpio[2]), "reset-psr",
                             0xfffffffe, &error_fatal);
    qdev_realize(DEVICE(s), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), GLOHD_RAM_BASE, ms->ram);

    f->sdma_irq = qdev_get_gpio_in(DEVICE(&s->a9mpcore), FSL_IMX6_SDMA_IRQ);
    memory_region_init_io(&f->sdma_iomem, OBJECT(ms), &glohd_sdma_ops,
                          f, "glohd.sdma", GLOHD_SDMA_SIZE);
    memory_region_add_subregion(get_system_memory(), FSL_IMX6_SDMA_ADDR,
                                &f->sdma_iomem);
    memory_region_init_io(&f->dcp_iomem, OBJECT(ms), &glohd_dcp_ops,
                          f, "glohd.dcp", GLOHD_DCP_SIZE);
    memory_region_add_subregion(get_system_memory(), 0x020fc000, &f->dcp_iomem);

    sysbus_create_simple(TYPE_IMX_RNGC, 0x021b4000,
                         qdev_get_gpio_in(DEVICE(&s->a9mpcore), 5));
    pxp = sysbus_create_simple(TYPE_IMX6SL_PXP, 0x020f0000,
                              qdev_get_gpio_in(DEVICE(&s->a9mpcore), 98));
    dev = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(ms), "epdc", OBJECT(dev));
    object_property_set_link(OBJECT(dev), "pxp", OBJECT(pxp), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x020f4000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                      qdev_get_gpio_in(DEVICE(&s->a9mpcore), 97));
    dev = sysbus_create_simple(TYPE_IMX6SLL_OCOTP,
                               FSL_IMX6_OCOTPCTRL_ADDR, NULL);
    /* ANA1: counts 1520 at 25 C and 1370 at 100 C, matching ANATOP's
     * nominal sample and both vendor temperature-calibration formulas. */
    imx6sll_ocotp_set_fuse(IMX6SLL_OCOTP(dev), 1, 6, 0x5f055a64);
    sysbus_create_simple(TYPE_IMX6SL_MMDC, 0x021b0000, NULL);
    sysbus_create_simple(TYPE_IMX6SL_IOMUXC, 0x020e0000, NULL);

    di = drive_get(IF_SD, 0, 0);
    dev = qdev_new(TYPE_SD_CARD);
    qdev_prop_set_drive_err(dev, "drive", di ? blk_by_legacy_dinfo(di) : NULL,
                            &error_fatal);
    qdev_realize_and_unref(dev,
                           qdev_get_child_bus(DEVICE(&s->usdhc[1]), "sd-bus"),
                           &error_fatal);

    dev = qdev_new(TYPE_BCM43362_SDIO);
    qemu_configure_nic_device(dev, true, "bcm43362");
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
    i2c_slave_create_simple(s->i2c[0].bus, TYPE_KOBOTOUCH_MSP430, 0x43);
    dev = DEVICE(i2c_slave_create_simple(s->i2c[1].bus, TYPE_TPS65185, 0x68));
    qdev_connect_gpio_out_named(dev, "pwrgood", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[1]), 13));
    qdev_connect_gpio_out_named(dev, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[1]), 9));
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 7,
        qdev_get_gpio_in_named(dev, "powerup", 0));
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 8,
        qdev_get_gpio_in_named(dev, "power-enable", 0));

    dev = qdev_new(TYPE_ZFORCE3);
    qdev_prop_set_uint8(dev, "address", 0x50);
    qdev_prop_set_uint16(dev, "width", 1072);
    qdev_prop_set_uint16(dev, "height", 1448);
    qdev_realize(dev, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(&s->gpio[4]), 6));
    qdev_connect_gpio_out(DEVICE(&s->gpio[4]), 9, qdev_get_gpio_in(dev, 0));
    object_unref(OBJECT(dev));
    dev = qdev_new("forma-buttons");
    qdev_realize(dev, NULL, &error_fatal);
    qdev_connect_gpio_out(dev, 2, qdev_get_gpio_in(DEVICE(&s->gpio[4]), 8));
    object_unref(OBJECT(dev));
    glohd_load_firmware(f, ms);
    qemu_register_reset(glohd_reset, f);
}

static void glohd_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Kobo Glo HD (Netronix E60Q9X, i.MX6SL)";
    mc->init = glohd_init;
    mc->max_cpus = mc->default_cpus = 1;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "glo-hd.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;
}

static const TypeInfo glohd_type = {
    .name = TYPE_GLOHD_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(GloHDMachineState),
    .class_init = glohd_class_init,
    .interfaces = arm_machine_interfaces,
};
static void glohd_register_types(void)
{
    type_register_static(&glohd_type);
}
type_init(glohd_register_types)
