/*
 * Amazon/Lab126 Wario board emulation (Freescale i.MX6SoloLite)
 *
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
#include "hw/i2c/drv2667.h"
#include "hw/i2c/cyttsp4.h"
#include "hw/i2c/zforce.h"
#include "hw/input/wario-keyboard.h"
#include "hw/i2c/max77696.h"
#include "hw/misc/unimp.h"
#include "hw/misc/imx6sl_iomuxc.h"
#include "hw/misc/imx6sl_mmdc.h"
#include "hw/misc/imx6sl_pxp.h"
#include "hw/misc/max44009.h"
#include "hw/sd/sd.h"
#include "hw/ssi/ssi.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/reset.h"
#include "system/system.h"
#include "system/qtest.h"

#define WARIO_RAM_BASE       0x80000000
#define WARIO_RAM_MAX        (2 * GiB)
#define WARIO_UBOOT_ADDR     0x00980000
#define WARIO_UBOOT_MAX      0x00080000
#define WARIO_IRAM_STACK     0x00920000
#define WARIO_MACHINE_ID     4091
#define WARIO_EPDC_ADDR      0x020f4000
/* Linux IRQ 129 minus the GIC SPI base (32). */
#define WARIO_EPDC_GIC_IRQ   97
#define WARIO_FB_ADDR        0x81000000
#define WARIO_FB_WIDTH       1072
#define WARIO_FB_HEIGHT      1448
#define WARIO_FB_STRIDE      1088
#define BOURBON_FB_ADDR      0x80c00000
#define BOURBON_FB_WIDTH     600
#define BOURBON_FB_HEIGHT    800
#define BOURBON_FB_STRIDE    608
#define PINOT_FB_ADDR        0x80c00000
#define PINOT_FB_WIDTH       758
#define PINOT_FB_HEIGHT      1024
#define PINOT_FB_STRIDE      768
#define WARIO_PXP_ADDR       0x020f0000
/* Linux IRQ 130 minus the GIC SPI base (32). */
#define WARIO_PXP_GIC_IRQ    98
#define WARIO_IVT_OFFSET     0x400
#define WARIO_IVT_ENTRY_OFF  0x04
#define WARIO_IVT_SELF_OFF   0x14
#define WARIO_IDME_BASE      0x5e000

#define TYPE_WARIO_MACHINE MACHINE_TYPE_NAME("imx6sl-wario")
OBJECT_DECLARE_SIMPLE_TYPE(WarioMachineState, WARIO_MACHINE)

typedef struct WarioMachineState {
    MachineState parent_obj;
    char *idme_serial;
    char *idme_mac;
    char *idme_mfg;
    char *idme_pcbsn;
    char *idme_bootmode;
    char *idme_postmode;
    bool idme_boot_partitions;
} WarioMachineState;

typedef struct WarioIdmeField {
    const char *name;
    char *value;
    uint32_t offset;
    size_t size;
} WarioIdmeField;

static struct arm_boot_info wario_boot_info;

static bool wario_is_bourbon(const WarioMachineState *wms)
{
    /*
     * Lab126 board IDs occupy the first three bytes of the PCB serial.
     * 051 is production Bourbon and 062 is the pre-EVT2 Bourbon spin.
     */
    return g_str_has_prefix(wms->idme_pcbsn, "051") ||
           g_str_has_prefix(wms->idme_pcbsn, "062");
}

static bool wario_is_pinot(const WarioMachineState *wms)
{
    /* 027/02e are Wi-Fi Pinot boards; 02a/02f are WAN variants. */
    return g_str_has_prefix(wms->idme_pcbsn, "027") ||
           g_ascii_strncasecmp(wms->idme_pcbsn, "02a", 3) == 0 ||
           g_ascii_strncasecmp(wms->idme_pcbsn, "02e", 3) == 0 ||
           g_ascii_strncasecmp(wms->idme_pcbsn, "02f", 3) == 0;
}

static bool wario_is_muscat(const WarioMachineState *wms)
{
    /* 067/13G are Wi-Fi Muscat boards; 068 is the WAN variant. */
    return g_str_has_prefix(wms->idme_pcbsn, "067") ||
           g_str_has_prefix(wms->idme_pcbsn, "068") ||
           g_ascii_strncasecmp(wms->idme_pcbsn, "13g", 3) == 0;
}

static void wario_firmware_reset(void *opaque)
{
    ARMCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
    if (wario_boot_info.entry >= WARIO_RAM_BASE) {
        cpu->env.regs[13] = WARIO_IRAM_STACK;
    }
    cpu_set_pc(CPU(cpu), wario_boot_info.entry);
}

static void wario_populate_idme(DeviceState *card, WarioMachineState *wms)
{
    WarioIdmeField fields[] = {
        { "serial", wms->idme_serial, 0x0000, 16 },
        { "mac", wms->idme_mac, 0x0030, 12 },
        { "mfg", wms->idme_mfg, 0x0040, 20 },
        { "pcbsn", wms->idme_pcbsn, 0x0060, 16 },
        { "bootmode", wms->idme_bootmode, 0x1000, 16 },
        { "postmode", wms->idme_postmode, 0x1010, 16 },
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(fields); i++) {
        WarioIdmeField *field = &fields[i];
        g_autofree uint8_t *contents = g_malloc0(field->size);
        size_t len = strlen(field->value);

        if (!g_str_is_ascii(field->value) || len > field->size) {
            error_report("IDME %s must contain at most %zu ASCII bytes",
                         field->name, field->size);
            exit(EXIT_FAILURE);
        }
        memcpy(contents, field->value, len);
        emmc_boot_partition_write(card, 1, WARIO_IDME_BASE + field->offset,
                                  contents, field->size, &error_fatal);
    }
}

static void wario_attach_card(FslIMX6State *s, WarioMachineState *wms,
                              unsigned controller, unsigned drive_index,
                              bool emmc)
{
    DriveInfo *di = drive_get(IF_SD, 0, drive_index);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus;
    DeviceState *card;

    bus = qdev_get_child_bus(DEVICE(&s->usdhc[controller]), "sd-bus");
    card = qdev_new(emmc ? TYPE_EMMC : TYPE_SD_CARD);
    if (emmc && wms->idme_boot_partitions) {
        /* Wario keeps IDME in volatile eMMC boot partition 1. */
        qdev_prop_set_uint64(card, "boot-partition-size", 2 * MiB);
        qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    }
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);
    if (emmc && wms->idme_boot_partitions) {
        wario_populate_idme(card, wms);
    }
    object_unref(OBJECT(card));
}

static void wario_attach_panel_flash(FslIMX6State *s)
{
    SSIBus *bus;
    DeviceState *flash;
    DriveInfo *di = drive_get(IF_MTD, 0, 0);

    /*
     * A 4-Mbit panel NOR is physically populated on Icewine and must be
     * present even when no content backing file was supplied.  An unbacked
     * QEMU flash has erased (0xff) contents but still reports its JEDEC
     * ID, allowing the stock driver to distinguish an empty flash from a
     * missing SPI slave.  Panel waveform contents are supplied separately by
     * the hidden eMMC waveform store.
     */
    bus = (SSIBus *)qdev_get_child_bus(DEVICE(&s->spi[0]), "spi");
    /* AC-format panels use the 512 KiB address layout ending at 0x80000. */
    flash = qdev_new("mx25l4005a");
    if (di) {
        qdev_prop_set_drive_err(flash, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
    }
    qdev_realize_and_unref(flash, BUS(bus), &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 11,
                          qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0));
}

static void wario_attach_wifi(FslIMX6State *s)
{
    BusState *bus;
    DeviceState *wifi;

    bus = qdev_get_child_bus(DEVICE(&s->usdhc[2]), "sd-bus");
    wifi = qdev_new(TYPE_AR6003_SDIO);
    qdev_realize(wifi, bus, &error_fatal);

    /* Icewine WIFI_PWD: GPIO3_29, low at boot and high when enabled. */
    qdev_connect_gpio_out(DEVICE(&s->gpio[2]), 29,
                          qdev_get_gpio_in_named(wifi, "power", 0));
    qdev_connect_gpio_out_named(wifi, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&s->usdhc[2]), "sdio-irq", 0));
    object_unref(OBJECT(wifi));
}

static void wario_load_firmware(MachineState *machine)
{
    g_autofree uint8_t *image = NULL;
    gsize image_size;
    hwaddr addr = WARIO_UBOOT_ADDR;
    hwaddr max_size = WARIO_UBOOT_MAX;
    uint32_t entry;
    uint32_t self;
    ssize_t size;

    if (!machine->firmware) {
        return;
    }

    /*
     * The i.MX boot ROM consumes an IVT at offset 0x400 and jumps to its
     * entry pointer.  -bios supplies the post-ROM image, so reproduce that
     * last boot-ROM action instead of starting at the ARM reset vector.
     * Stock U-Boot runs from OCRAM, while current barebox images are loaded
     * directly into SDRAM; the IVT self pointer identifies either layout.
     */
    if (!g_file_get_contents(machine->firmware, (char **)&image,
                             &image_size, NULL) ||
        image_size < WARIO_IVT_OFFSET + WARIO_IVT_ENTRY_OFF + sizeof(entry)) {
        error_report("Wario firmware has no readable IVT");
        exit(EXIT_FAILURE);
    }
    entry = ldl_le_p(image + WARIO_IVT_OFFSET + WARIO_IVT_ENTRY_OFF);
    self = ldl_le_p(image + WARIO_IVT_OFFSET + WARIO_IVT_SELF_OFF);
    if (self >= WARIO_RAM_BASE + WARIO_IVT_OFFSET &&
        self < WARIO_RAM_BASE + machine->ram_size) {
        addr = self - WARIO_IVT_OFFSET;
        max_size = WARIO_RAM_BASE + machine->ram_size - addr;
    }
    if (entry < addr || entry >= addr + image_size) {
        error_report("Wario firmware IVT entry 0x%08x is outside image", entry);
        exit(EXIT_FAILURE);
    }

    size = load_image_targphys(machine->firmware, addr, max_size, NULL);
    if (size < 0) {
        error_report("Unable to load Wario firmware '%s'",
                     machine->firmware);
        exit(EXIT_FAILURE);
    }

    wario_boot_info.entry = entry;
}

static void wario_init(MachineState *machine)
{
    WarioMachineState *wms = WARIO_MACHINE(machine);
    FslIMX6State *s;
    DeviceState *epdc;
    DeviceState *mmdc;
    DeviceState *iomuxc;
    DeviceState *pxp;
    DeviceState *keyboard;
    DeviceState *pmic;
    I2CBus *i2c;
    bool bourbon = wario_is_bourbon(wms);
    bool pinot = wario_is_pinot(wms);
    bool muscat = wario_is_muscat(wms);

    if (machine->ram_size > WARIO_RAM_MAX) {
        error_report("RAM size " RAM_ADDR_FMT " exceeds i.MX6SL maximum",
                     machine->ram_size);
        exit(EXIT_FAILURE);
    }

    wario_boot_info = (struct arm_boot_info) {
        .loader_start = WARIO_RAM_BASE,
        .board_id = WARIO_MACHINE_ID,
        .ram_size = machine->ram_size,
    };

    s = FSL_IMX6(object_new(TYPE_FSL_IMX6));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(s), "sololite", true, &error_fatal);
    /* Stock Wario Linux owns the GIC directly; do not expose TrustZone. */
    object_property_set_bool(OBJECT(s), "has-el3", false, &error_fatal);
    object_property_set_uint(OBJECT(&s->src), "sbmr1", 0x60,
                             &error_fatal);
    object_property_set_uint(OBJECT(s), "fec-phy-num", 0,
                             &error_fatal);
    qdev_realize(DEVICE(s), NULL, &error_fatal);

    memory_region_add_subregion(get_system_memory(), WARIO_RAM_BASE,
                                machine->ram);

    /* USDHC2 is eMMC; USDHC3's Wi-Fi SDIO function is modeled separately. */
    wario_attach_card(s, wms, 1, 1, true);
    wario_attach_wifi(s);
    wario_attach_panel_flash(s);

    i2c = s->i2c[0].bus;
    i2c_slave_create_simple(i2c, TYPE_MAX77696, 0x34);
    i2c_slave_create_simple(i2c, TYPE_MAX77696, 0x35);
    pmic = qdev_new(TYPE_MAX77696);
    qdev_prop_set_uint8(pmic, "address", 0x3c);
    qdev_connect_gpio_out(pmic, 0,
                          qdev_get_gpio_in(DEVICE(&s->gpio[3]), 20));
    /* MAX77696 EPD POK is a separate rising-edge signal on GPIO2_13. */
    qdev_connect_gpio_out(pmic, 1,
                          qdev_get_gpio_in(DEVICE(&s->gpio[1]), 13));
    qdev_realize_and_unref(pmic, BUS(i2c), &error_fatal);
    i2c_slave_create_simple(i2c, TYPE_MAX77696, 0x68);

    /* Icewine/Wario 4 ambient-light sensor: I2C3, active-low GPIO4_14. */
    pmic = qdev_new(TYPE_MAX44009);
    qdev_prop_set_uint8(pmic, "address", 0x4a);
    qdev_realize(pmic, BUS(s->i2c[2].bus), &error_fatal);
    qdev_connect_gpio_out(pmic, 0,
                          qdev_get_gpio_in(DEVICE(&s->gpio[3]), 14));
    object_unref(OBJECT(pmic));

    /* Icewine's TI DRV2667 piezo haptic controller is on I2C3. */
    i2c_slave_create_simple(s->i2c[2].bus, TYPE_DRV2667, 0x59);

    /*
     * Touch is on I2C2 for these production variants.  Icewine, Pinot and
     * Muscat use Cypress TrueTouch Gen4 while the 256 MiB Bourbon board uses
     * Neonode zForce2.  They share the active-low GPIO4_3 interrupt and
     * GPIO4_5 reset lines.  Muscat's stock driver flips X, so pre-invert the
     * controller coordinates to keep host pointer input spatially aligned.
     */
    pmic = qdev_new(bourbon ? TYPE_KINDLE_ZFORCE2 : TYPE_CYTTSP4);
    qdev_prop_set_uint8(pmic, "address", bourbon ? 0x50 : 0x24);
    if (muscat) {
        qdev_prop_set_bit(pmic, "invert-x", true);
    }
    qdev_realize(pmic, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out(pmic, 0,
                          qdev_get_gpio_in(DEVICE(&s->gpio[3]), 3));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 5,
                          qdev_get_gpio_in(pmic, 0));
    object_unref(OBJECT(pmic));

    epdc = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(machine), "epdc", OBJECT(epdc));
    if (bourbon) {
        /*
         * Bourbon's 256 MiB stock kernel reserves its 608x5376 virtual
         * framebuffer at 0x80c00000.  Scan out the visible 600x800 page.
         */
        qdev_prop_set_uint64(epdc, "fb-addr", BOURBON_FB_ADDR);
        qdev_prop_set_uint32(epdc, "fb-width", BOURBON_FB_WIDTH);
        qdev_prop_set_uint32(epdc, "fb-height", BOURBON_FB_HEIGHT);
        qdev_prop_set_uint32(epdc, "fb-stride", BOURBON_FB_STRIDE);
    } else if (pinot) {
        /* Pinot's 256 MiB layout and 758x1024 E60 panel. */
        qdev_prop_set_uint64(epdc, "fb-addr", PINOT_FB_ADDR);
        qdev_prop_set_uint32(epdc, "fb-width", PINOT_FB_WIDTH);
        qdev_prop_set_uint32(epdc, "fb-height", PINOT_FB_HEIGHT);
        qdev_prop_set_uint32(epdc, "fb-stride", PINOT_FB_STRIDE);
    } else {
        /* Icewine's fixed 512 MiB layout places the framebuffer here. */
        qdev_prop_set_uint64(epdc, "fb-addr", WARIO_FB_ADDR);
        qdev_prop_set_uint32(epdc, "fb-width", WARIO_FB_WIDTH);
        qdev_prop_set_uint32(epdc, "fb-height", WARIO_FB_HEIGHT);
        qdev_prop_set_uint32(epdc, "fb-stride", WARIO_FB_STRIDE);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(epdc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(epdc), 0, WARIO_EPDC_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(epdc), 0,
                       qdev_get_gpio_in(DEVICE(&s->a9mpcore),
                                       WARIO_EPDC_GIC_IRQ));

    mmdc = qdev_new(TYPE_IMX6SL_MMDC);
    object_property_add_child(OBJECT(machine), "mmdc", OBJECT(mmdc));
    qdev_prop_set_uint64(mmdc, "ram-size", machine->ram_size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(mmdc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mmdc), 0, 0x021b0000);

    iomuxc = qdev_new(TYPE_IMX6SL_IOMUXC);
    object_property_add_child(OBJECT(machine), "iomuxc", OBJECT(iomuxc));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(iomuxc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(iomuxc), 0, 0x020e0000);

    pxp = qdev_new(TYPE_IMX6SL_PXP);
    object_property_add_child(OBJECT(machine), "pxp", OBJECT(pxp));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pxp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pxp), 0, WARIO_PXP_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(pxp), 0,
                       qdev_get_gpio_in(DEVICE(&s->a9mpcore),
                                       WARIO_PXP_GIC_IRQ));

    /*
     * The stock kernel has uinput built in.  A tiny userspace bridge reads
     * this mailbox from the otherwise unused DCP aperture, exposing QEMU key
     * events without a guest kernel module or a non-stock input driver.
     */
    keyboard = qdev_new(TYPE_WARIO_KEYBOARD);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(keyboard), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(keyboard), 0, 0x020fc000);
    create_unimplemented_device("imx6sl.dcp-reserved", 0x020fd000, 0x3000);

    wario_load_firmware(machine);
    if (machine->firmware) {
        qemu_register_reset(wario_firmware_reset, &s->cpu[0]);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&s->cpu[0], machine, &wario_boot_info);
    }
}

static char *wario_idme_get(char **value)
{
    return g_strdup(*value);
}

static void wario_idme_set(char **field, const char *value)
{
    g_free(*field);
    *field = g_strdup(value);
}

static bool wario_get_idme_boot_partitions(Object *obj, Error **errp)
{
    return WARIO_MACHINE(obj)->idme_boot_partitions;
}

static void wario_set_idme_boot_partitions(Object *obj, bool value,
                                           Error **errp)
{
    WARIO_MACHINE(obj)->idme_boot_partitions = value;
}

#define WARIO_IDME_PROPERTY(_member)                                     \
    static char *wario_get_##_member(Object *obj, Error **errp)          \
    {                                                                    \
        return wario_idme_get(&WARIO_MACHINE(obj)->_member);             \
    }                                                                    \
    static void wario_set_##_member(Object *obj, const char *value,      \
                                    Error **errp)                         \
    {                                                                    \
        wario_idme_set(&WARIO_MACHINE(obj)->_member, value);             \
    }

WARIO_IDME_PROPERTY(idme_serial)
WARIO_IDME_PROPERTY(idme_mac)
WARIO_IDME_PROPERTY(idme_mfg)
WARIO_IDME_PROPERTY(idme_pcbsn)
WARIO_IDME_PROPERTY(idme_bootmode)
WARIO_IDME_PROPERTY(idme_postmode)

static void wario_machine_instance_init(Object *obj)
{
    WarioMachineState *wms = WARIO_MACHINE(obj);

    wms->idme_serial = g_strdup("B054000000000001");
    wms->idme_mac = g_strdup("020000000002");
    wms->idme_mfg = g_strdup("00000000000000000000");
    wms->idme_pcbsn = g_strdup("0470000000000001");
    wms->idme_bootmode = g_strdup("main");
    wms->idme_postmode = g_strdup("normal");
    wms->idme_boot_partitions = true;
}

static void wario_machine_instance_finalize(Object *obj)
{
    WarioMachineState *wms = WARIO_MACHINE(obj);

    g_free(wms->idme_serial);
    g_free(wms->idme_mac);
    g_free(wms->idme_mfg);
    g_free(wms->idme_pcbsn);
    g_free(wms->idme_bootmode);
    g_free(wms->idme_postmode);
}

static void wario_machine_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Lab126 Wario (i.MX6SoloLite, Cortex-A9, EPDC)";
    mc->init = wario_init;
    mc->max_cpus = 1;
    mc->default_cpus = 1;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "wario.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;

    object_class_property_add_str(oc, "idme-serial", wario_get_idme_serial,
                                  wario_set_idme_serial);
    object_class_property_add_str(oc, "idme-mac", wario_get_idme_mac,
                                  wario_set_idme_mac);
    object_class_property_add_str(oc, "idme-mfg", wario_get_idme_mfg,
                                  wario_set_idme_mfg);
    object_class_property_add_str(oc, "idme-pcbsn", wario_get_idme_pcbsn,
                                  wario_set_idme_pcbsn);
    object_class_property_add_str(oc, "idme-bootmode",
                                  wario_get_idme_bootmode,
                                  wario_set_idme_bootmode);
    object_class_property_add_str(oc, "idme-postmode",
                                  wario_get_idme_postmode,
                                  wario_set_idme_postmode);
    object_class_property_add_bool(oc, "idme-boot-partitions",
                                   wario_get_idme_boot_partitions,
                                   wario_set_idme_boot_partitions);
}

static const TypeInfo wario_machine_type = {
    .name = TYPE_WARIO_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(WarioMachineState),
    .instance_init = wario_machine_instance_init,
    .instance_finalize = wario_machine_instance_finalize,
    .class_init = wario_machine_init,
    .interfaces = arm_machine_interfaces,
};

static void wario_machine_register_types(void)
{
    type_register_static(&wario_machine_type);
}

type_init(wario_machine_register_types)
