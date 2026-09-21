/*
 * Amazon/Lab126 Jaeger (Kindle 10th generation basic) board emulation
 *
 * Wiring follows imx6sll-jaeger-wfo.dts in Kindle GPL 5.18.1.1.1.
 * The vendor boot package calls the shared boot platform imx6sll_rex;
 * Jaeger has its own panel, touch orientation, identity and board wiring.
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
#include "hw/i2c/bd71827.h"
#include "hw/i2c/fp9929.h"
#include "hw/i2c/lm3695.h"
#include "hw/input/wario-keyboard.h"
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
#include "system/qtest.h"
#include "system/reset.h"

#define JAEGER_RAM_BASE        0x80000000
#define JAEGER_RAM_MAX         (2 * GiB)
#define JAEGER_UBOOT_MAX       (1 * MiB)
#define JAEGER_MACHINE_ID      5380
#define JAEGER_IDME_SIZE       0x4000
#define JAEGER_IDME_VERSION    "2.1"
#define JAEGER_SBIOS_OFFSET    0xc0000
#define JAEGER_SBIOS_SIZE      0x10000
#define JAEGER_FBIOS_OFFSET    (JAEGER_SBIOS_OFFSET + JAEGER_SBIOS_SIZE)
#define JAEGER_FBIOS_SIZE      0x30000
#define JAEGER_SBIOS_ADDR      0x9ffc0000
#define JAEGER_FBIOS_ADDR      0x9ffd0000
#define JAEGER_BIOS_MARKER     0x51494255
#define JAEGER_SDMA_SIZE       0x1000
#define JAEGER_SDMA_C0PTR      0x00
#define JAEGER_SDMA_INTR       0x04
#define JAEGER_SDMA_START      0x0c
#define JAEGER_SDMA_RESET      0x24
#define JAEGER_DCP_ADDR        0x020fc000
#define JAEGER_DCP_SIZE        0x4000
#define JAEGER_DCP_CTRL        0x000
#define JAEGER_DCP_SFTRST      BIT(31)
#define JAEGER_DCP_CLKGATE     BIT(30)
#define JAEGER_PXP_ADDR        0x020f0000
#define JAEGER_PXP_GIC_IRQ     98
#define JAEGER_EPDC_ADDR       0x020f4000
#define JAEGER_EPDC_GIC_IRQ    97
#define JAEGER_FB_ADDR         0x9d000000
#define JAEGER_FB_WIDTH        600
#define JAEGER_FB_HEIGHT       800
#define JAEGER_FB_STRIDE       608

#define JAEGER_FUSE_SEC_CONFIG_BANK  0
#define JAEGER_FUSE_SEC_CONFIG_WORD  6
#define JAEGER_FUSE_SRK_BANK         3
#define JAEGER_FUSE_PROD_BANK        4
#define JAEGER_FUSE_PROD_WORD        6

#define IMX_IVT_HEADER      0x402000d1
#define IMX_IVT_ENTRY       0x04
#define IMX_IVT_BOOT_DATA   0x10
#define IMX_IVT_SELF        0x14
#define IMX_IVT_SIZE        0x20
#define IMX_BOOT_PLUGIN     0x08
#define IMX_BOOT_DATA_SIZE  0x0c

#define TYPE_JAEGER_MACHINE MACHINE_TYPE_NAME("imx6sll-jaeger")
OBJECT_DECLARE_SIMPLE_TYPE(JaegerMachineState, JAEGER_MACHINE)

typedef struct JaegerMachineState {
    MachineState parent_obj;
    char *idme_serial;
    char *idme_mac;
    char *idme_mfg;
    char *idme_pcbsn;
    char *idme_bootmode;
    char *idme_postmode;
    char *device_profile;
    char *falcon_bios;
    char *storage_bios;
    char *boot_partitions_file;
    char *fuse_overrides;
    FslIMX6State *soc;
    IMX6SLLOCOTPState *ocotp;
    IMXRNGCState rngc;
    MemoryRegion sdma_iomem;
    uint32_t sdma[JAEGER_SDMA_SIZE / sizeof(uint32_t)];
    qemu_irq sdma_irq;
    MemoryRegion dcp_iomem;
    uint32_t dcp[JAEGER_DCP_SIZE / sizeof(uint32_t)];
} JaegerMachineState;

typedef struct JaegerIdmeField {
    const char *name;
    const char *value;
    size_t size;
    bool exportable;
} JaegerIdmeField;

static struct arm_boot_info jaeger_boot_info;

static void jaeger_sdma_complete_channel0(JaegerMachineState *rms)
{
    uint32_t ccb_addr = rms->sdma[JAEGER_SDMA_C0PTR / 4];
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

    rms->sdma[JAEGER_SDMA_INTR / 4] |= BIT(0);
    qemu_set_irq(rms->sdma_irq, 1);
}

static uint64_t jaeger_sdma_read(void *opaque, hwaddr offset, unsigned size)
{
    JaegerMachineState *rms = opaque;

    return rms->sdma[offset / 4];
}

static void jaeger_sdma_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    JaegerMachineState *rms = opaque;

    switch (offset) {
    case JAEGER_SDMA_INTR:
        rms->sdma[offset / 4] &= ~(uint32_t)value;
        qemu_set_irq(rms->sdma_irq, rms->sdma[offset / 4] != 0);
        break;
    case JAEGER_SDMA_START:
        if (value & BIT(0)) {
            jaeger_sdma_complete_channel0(rms);
        }
        break;
    case JAEGER_SDMA_RESET:
        memset(rms->sdma, 0, sizeof(rms->sdma));
        qemu_set_irq(rms->sdma_irq, 0);
        break;
    default:
        rms->sdma[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps jaeger_sdma_ops = {
    .read = jaeger_sdma_read,
    .write = jaeger_sdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t jaeger_dcp_read(void *opaque, hwaddr offset, unsigned size)
{
    JaegerMachineState *rms = opaque;

    return rms->dcp[offset / 4];
}

static void jaeger_dcp_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    JaegerMachineState *rms = opaque;
    hwaddr base = offset & ~0xfULL;
    unsigned alias = (offset & 0xc) >> 2;
    uint32_t current = rms->dcp[base / 4];

    switch (alias) {
    case 0: current = value; break;
    case 1: current |= value; break;
    case 2: current &= ~(uint32_t)value; break;
    case 3: current ^= value; break;
    }
    if (base == JAEGER_DCP_CTRL && (current & JAEGER_DCP_SFTRST)) {
        memset(rms->dcp, 0, sizeof(rms->dcp));
        current = JAEGER_DCP_SFTRST | JAEGER_DCP_CLKGATE;
    }
    rms->dcp[base / 4] = current;
}

static const MemoryRegionOps jaeger_dcp_ops = {
    .read = jaeger_dcp_read,
    .write = jaeger_dcp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4,
               .unaligned = false },
};

static const uint32_t jaeger_srk_hash[8] = {
    0xdce63a54, 0x4fbd4fd2, 0xa75404f6, 0xb76b5da5,
    0xa1930e3f, 0x800a3091, 0x877a46d1, 0x72d0c6be,
};

static unsigned jaeger_profile_hwid(const char *profile)
{
    if (!strcmp(profile, "production") || !strcmp(profile, "dvt")) {
        /* The vendor Jaeger board table labels the retail PCB straps DVT. */
        return 3;
    }
    error_report("invalid Jaeger device-profile '%s' (expected production or dvt)",
                 profile);
    exit(EXIT_FAILURE);
}

static void jaeger_configure_identity(FslIMX6State *s,
                                   IMX6SLLOCOTPState *ocotp,
                                   JaegerMachineState *rms)
{
    unsigned hwid;
    bool secure;
    bool production;
    size_t i;

    hwid = jaeger_profile_hwid(rms->device_profile);
    secure = production = !strcmp(rms->device_profile, "production");

    if (secure) {
        imx6sll_ocotp_set_fuse(ocotp, 0, 0, 1 << 14); /* SRK_LOCK */
        imx6sll_ocotp_set_fuse(ocotp, JAEGER_FUSE_SEC_CONFIG_BANK,
                               JAEGER_FUSE_SEC_CONFIG_WORD, 1 << 1);
        for (i = 0; i < ARRAY_SIZE(jaeger_srk_hash); i++) {
            imx6sll_ocotp_set_fuse(ocotp, JAEGER_FUSE_SRK_BANK, i,
                                   jaeger_srk_hash[i]);
        }
    }
    if (production) {
        imx6sll_ocotp_set_fuse(ocotp, JAEGER_FUSE_PROD_BANK,
                               JAEGER_FUSE_PROD_WORD, 1);
    }

    qemu_set_irq(qdev_get_gpio_in(DEVICE(&s->gpio[4]), 19), hwid >> 3 & 1);
    qemu_set_irq(qdev_get_gpio_in(DEVICE(&s->gpio[4]), 20), hwid >> 2 & 1);
    qemu_set_irq(qdev_get_gpio_in(DEVICE(&s->gpio[4]), 16), hwid >> 1 & 1);
    qemu_set_irq(qdev_get_gpio_in(DEVICE(&s->gpio[4]), 17), hwid & 1);
}

static void jaeger_firmware_reset(void *opaque)
{
    JaegerMachineState *rms = opaque;
    ARMCPU *cpu = &rms->soc->cpu[0];

    memset(rms->sdma, 0, sizeof(rms->sdma));
    memset(rms->dcp, 0, sizeof(rms->dcp));
    rms->dcp[JAEGER_DCP_CTRL / 4] = JAEGER_DCP_CLKGATE;
    qemu_set_irq(rms->sdma_irq, 0);
    cpu_reset(CPU(cpu));
    cpu_set_pc(CPU(cpu), jaeger_boot_info.entry);
}

static void jaeger_populate_idme(DeviceState *card, JaegerMachineState *rms)
{
    g_autofree uint8_t *contents = g_malloc0(JAEGER_IDME_SIZE);
    unsigned hwid = jaeger_profile_hwid(rms->device_profile);
    g_autofree char *hwid_string = g_strdup_printf("%u", hwid);
    JaegerIdmeField fields[] = {
        { "board_id",       rms->idme_pcbsn,   16, true  },
        { "serial",         rms->idme_serial,  16, true  },
        { "mac_addr",       rms->idme_mac,     16, true  },
        { "mac_sec",        "0",               32, true  },
        { "bt_mac_addr",    "0",               16, true  },
        { "bt_mfg",         "0",              128, true  },
        { "product_name",   "0",               32, true  },
        { "productid",      "0",               32, true  },
        /* Murata battery identity checked by stock production recovery. */
        { "productid2",     "1034",            32, true  },
        { "region",         "US",               4, true  },
        { "bootmode",       rms->idme_bootmode, 16, true  },
        { "postmode",       rms->idme_postmode, 16, true  },
        { "bootcount",      "0",                8, true  },
        { "manufacturing",  rms->idme_mfg,     512, true  },
        { "unlock_code",    "",              1024, true  },
        { "oldboot",        "0",               16, true  },
        { "qbcount",        "0",               16, true  },
        { "device_type_id", "0",               32, true  },
        { "vcom",           "-2500000",        16, true  },
        { "fsn",            "0",               16, true  },
        { "mfgdate",        "0",                8, true  },
        { "dev_flags",      "0",                8, true  },
        { "fos_flags",      "0",                8, true  },
        { "usr_flags",      "0",                8, true  },
        { "tz_keys",        "0",             8192, false },
        { "alscal1",        "0",               32, true  },
        { "alscal2",        "0",               32, true  },
        { "hwid",           hwid_string,          4, true  },
    };
    uint8_t *cursor = contents;
    size_t i;

    memcpy(cursor, "beefdeed", 8);
    memcpy(cursor + 8, JAEGER_IDME_VERSION, sizeof(JAEGER_IDME_VERSION));
    stl_le_p(cursor + 12, ARRAY_SIZE(fields));
    cursor += 16;

    for (i = 0; i < ARRAY_SIZE(fields); i++) {
        JaegerIdmeField *field = &fields[i];
        size_t len = strlen(field->value);

        if (strlen(field->name) > 16 || !g_str_is_ascii(field->value) ||
            len > field->size ||
            cursor + 28 + field->size > contents + JAEGER_IDME_SIZE) {
            error_report("IDME %s must contain at most %zu ASCII bytes",
                         field->name, field->size);
            exit(EXIT_FAILURE);
        }
        memcpy(cursor, field->name, strlen(field->name));
        stl_le_p(cursor + 16, field->size);
        stl_le_p(cursor + 20, field->exportable);
        stl_le_p(cursor + 24, 0444);
        memcpy(cursor + 28, field->value, len);
        cursor += 28 + field->size;
    }

    /* Jaeger stores the complete IDME 2.x table at block zero of boot area 2. */
    emmc_boot_partition_write(card, 2, 0, contents, JAEGER_IDME_SIZE,
                              &error_fatal);
}

static bool jaeger_idme_is_populated(DeviceState *card)
{
    uint8_t header[12];
    Error *err = NULL;

    emmc_boot_partition_read(card, 2, 0, header, sizeof(header), &err);
    if (err) {
        error_report_err(err);
        exit(EXIT_FAILURE);
    }
    return !memcmp(header, "beefdeed", 8) &&
           !memcmp(header + 8, JAEGER_IDME_VERSION, sizeof(JAEGER_IDME_VERSION));
}

static void jaeger_populate_falcon_image(DeviceState *card, const char *path,
                                      const char *name, uint64_t offset,
                                      size_t limit, size_t marker_offset,
                                      hwaddr load_addr)
{
    g_autofree uint8_t *contents = NULL;
    g_autoptr(GError) error = NULL;
    gsize size;

    if (!g_file_get_contents(path, (char **)&contents, &size, &error)) {
        error_report("unable to read Jaeger %s '%s': %s", name, path,
                     error->message);
        exit(EXIT_FAILURE);
    }
    if (size > limit || size < marker_offset + sizeof(uint32_t) ||
        ldl_le_p(contents + marker_offset) != JAEGER_BIOS_MARKER) {
        error_report("Jaeger %s '%s' is not a valid image", name, path);
        exit(EXIT_FAILURE);
    }

    emmc_boot_partition_write(card, 1, offset, contents, size, &error_fatal);
    /* The real DDR plugin leaves both images resident for U-Boot. */
    rom_add_blob_fixed(name, contents, size, load_addr);
    cpu_physical_memory_write(load_addr, contents, size);
}

static void jaeger_attach_emmc(FslIMX6State *s, JaegerMachineState *rms)
{
    DriveInfo *di = drive_get(IF_SD, 0, 1);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus = qdev_get_child_bus(DEVICE(&s->usdhc[1]), "sd-bus");
    DeviceState *card = qdev_new(TYPE_EMMC);

    qdev_prop_set_uint64(card, "boot-partition-size", 2 * MiB);
    qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    if (rms->boot_partitions_file) {
        qdev_prop_set_string(card, "boot-partitions-file",
                             rms->boot_partitions_file);
    }
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);
    if (!rms->falcon_bios || !rms->storage_bios) {
        error_report("Jaeger falcon-bios and storage-bios are required");
        exit(EXIT_FAILURE);
    }
    if (rms->storage_bios) {
        /* Jaeger keeps quickboot firmware in boot area 1; area 2 holds IDME. */
        jaeger_populate_falcon_image(card, rms->storage_bios, "storage BIOS",
                                  JAEGER_SBIOS_OFFSET, JAEGER_SBIOS_SIZE, 0x20,
                                  JAEGER_SBIOS_ADDR);
        jaeger_populate_falcon_image(card, rms->falcon_bios, "Falcon BIOS",
                                  JAEGER_FBIOS_OFFSET, JAEGER_FBIOS_SIZE, 0x30,
                                  JAEGER_FBIOS_ADDR);
    }
    if (!jaeger_idme_is_populated(card)) {
        jaeger_populate_idme(card, rms);
    }
    object_unref(OBJECT(card));
}

static void jaeger_attach_wifi(FslIMX6State *s)
{
    BusState *bus = qdev_get_child_bus(DEVICE(&s->usdhc[0]), "sd-bus");
    DeviceState *wifi = qdev_new(TYPE_BCM4343W_SDIO);

    /* Jaeger: BCM4343W/BCM43430 A1, USDHC1 and its real wake wiring. */
    qdev_realize(wifi, bus, &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 15,
                          qdev_get_gpio_in_named(wifi, "power", 0));
    qdev_connect_gpio_out_named(wifi, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&s->usdhc[0]), "sdio-irq", 0));
    qdev_connect_gpio_out_named(wifi, "oob-irq", 0,
                                qdev_get_gpio_in(DEVICE(&s->gpio[2]), 31));
    object_unref(OBJECT(wifi));
}

static size_t jaeger_find_uboot_ivt(const uint8_t *image, size_t size)
{
    size_t offset;

    for (offset = 4; offset + IMX_IVT_SIZE + IMX_BOOT_DATA_SIZE <= size;
         offset += 4) {
        uint32_t boot_ptr;
        uint32_t self;
        uint64_t boot_offset;

        if (ldl_le_p(image + offset) != IMX_IVT_HEADER) {
            continue;
        }
        boot_ptr = ldl_le_p(image + offset + IMX_IVT_BOOT_DATA);
        self = ldl_le_p(image + offset + IMX_IVT_SELF);
        if (boot_ptr < self) {
            continue;
        }
        boot_offset = (uint64_t)offset + boot_ptr - self;
        if (boot_offset + IMX_BOOT_DATA_SIZE <= size &&
            ldl_le_p(image + boot_offset + IMX_BOOT_PLUGIN) == 0) {
            return offset;
        }
    }
    return SIZE_MAX;
}

static void jaeger_load_firmware(MachineState *machine)
{
    g_autofree uint8_t *image = NULL;
    gsize image_size;
    size_t ivt_offset;
    uint32_t first_boot_ptr;
    uint32_t first_self;
    uint64_t first_boot_offset;
    uint32_t entry;
    uint32_t self;
    hwaddr load_addr;
    ssize_t loaded;

    if (!machine->firmware) {
        return;
    }
    if (!g_file_get_contents(machine->firmware, (char **)&image,
                             &image_size, NULL) ||
        image_size < IMX_IVT_SIZE + IMX_BOOT_DATA_SIZE ||
        image_size > JAEGER_UBOOT_MAX ||
        ldl_le_p(image) != IMX_IVT_HEADER) {
        error_report("Jaeger firmware is not a supported i.MX boot image");
        exit(EXIT_FAILURE);
    }

    first_boot_ptr = ldl_le_p(image + IMX_IVT_BOOT_DATA);
    first_self = ldl_le_p(image + IMX_IVT_SELF);
    if (first_boot_ptr < first_self) {
        error_report("Jaeger firmware has an invalid plugin boot-data pointer");
        exit(EXIT_FAILURE);
    }
    first_boot_offset = first_boot_ptr - first_self;
    if (first_boot_offset + IMX_BOOT_DATA_SIZE > image_size ||
        ldl_le_p(image + first_boot_offset + IMX_BOOT_PLUGIN) != 1) {
        error_report("Jaeger firmware does not contain the expected DDR plugin");
        exit(EXIT_FAILURE);
    }

    ivt_offset = jaeger_find_uboot_ivt(image, image_size);
    if (ivt_offset == SIZE_MAX) {
        error_report("Jaeger firmware has no second-stage U-Boot IVT");
        exit(EXIT_FAILURE);
    }
    entry = ldl_le_p(image + ivt_offset + IMX_IVT_ENTRY);
    self = ldl_le_p(image + ivt_offset + IMX_IVT_SELF);
    if (entry < self || entry - self > image_size - ivt_offset) {
        error_report("Jaeger U-Boot entry point is outside the firmware image");
        exit(EXIT_FAILURE);
    }

    /*
     * The real Boot ROM runs the OCRAM plugin, which initializes LPDDR and
     * asks the ROM to copy the complete image to DDR.  RAM is already usable
     * in QEMU, so reproduce the resulting copy and enter U-Boot's second IVT.
     */
    load_addr = self - ivt_offset;
    if (load_addr < JAEGER_RAM_BASE ||
        load_addr + image_size > JAEGER_RAM_BASE + machine->ram_size) {
        error_report("Jaeger U-Boot load range is outside guest RAM");
        exit(EXIT_FAILURE);
    }
    loaded = load_image_targphys(machine->firmware, load_addr,
                                 JAEGER_UBOOT_MAX, NULL);
    if (loaded != image_size) {
        error_report("Unable to load Jaeger firmware '%s'", machine->firmware);
        exit(EXIT_FAILURE);
    }
    jaeger_boot_info.entry = entry;
}

static void jaeger_init(MachineState *machine)
{
    JaegerMachineState *rms = JAEGER_MACHINE(machine);
    FslIMX6State *s;
    DeviceState *mmdc;
    DeviceState *iomuxc;
    DeviceState *ocotp_dev;
    DeviceState *epdc;
    DeviceState *pmic;
    DeviceState *pxp;
    DeviceState *touch;
    DeviceState *keyboard;
    IMX6SLLOCOTPState *ocotp;

    if (machine->ram_size > JAEGER_RAM_MAX) {
        error_report("RAM size " RAM_ADDR_FMT " exceeds i.MX6SLL maximum",
                     machine->ram_size);
        exit(EXIT_FAILURE);
    }

    jaeger_boot_info = (struct arm_boot_info) {
        .loader_start = JAEGER_RAM_BASE,
        .board_id = JAEGER_MACHINE_ID,
        .ram_size = machine->ram_size,
    };

    s = FSL_IMX6(object_new(TYPE_FSL_IMX6));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(s), "sololite", true, &error_fatal);
    object_property_set_bool(OBJECT(s), "sololite-lite", true, &error_fatal);
    object_property_set_bool(OBJECT(s), "has-el3", false, &error_fatal);
    /* A configured USB network backend represents the attached host. */
    if (!s->usb[0].gadget_nic_conf.peers.ncs[0]) {
        qdev_prop_set_bit(DEVICE(&s->usb[0]), "gadget-host-connected", false);
    }
    /* Stock U-Boot's synchronous EXT_CSD path is used only on USDHC2/eMMC. */
    object_property_set_bool(OBJECT(&s->usdhc[1]), "defer-data-transfer",
                             false, &error_fatal);
    /* External pull-ups leave all three Jaeger I2C buses idle high. */
    object_property_set_uint(OBJECT(&s->gpio[0]), "reset-psr",
                             BIT(0) | BIT(1), &error_fatal);
    object_property_set_uint(OBJECT(&s->gpio[2]), "reset-psr",
                             BIT(12) | BIT(13) | BIT(14) | BIT(15),
                             &error_fatal);
    /* INT_B is pulled high; active-high Hall input stays low with lid open. */
    object_property_set_uint(OBJECT(&s->gpio[3]), "reset-psr", BIT(24),
                             &error_fatal);
    {
        unsigned hwid = jaeger_profile_hwid(rms->device_profile);
        uint32_t psr = ((hwid >> 3 & 1) << 19) |
                       ((hwid >> 2 & 1) << 20) |
                       ((hwid >> 1 & 1) << 16) |
                       ((hwid & 1) << 17);

        object_property_set_uint(OBJECT(&s->gpio[4]), "reset-psr", psr,
                                 &error_fatal);
    }
    /* USDHC2 eMMC boot: BOOT_CFG1=0x60, BOOT_CFG2 port selector=1. */
    object_property_set_uint(OBJECT(&s->src), "sbmr1", 0x860,
                             &error_fatal);
    object_property_set_uint(OBJECT(s), "fec-phy-num", 0, &error_fatal);
    qdev_realize(DEVICE(s), NULL, &error_fatal);

    rms->sdma_irq = qdev_get_gpio_in(DEVICE(&s->a9mpcore), FSL_IMX6_SDMA_IRQ);
    memory_region_init_io(&rms->sdma_iomem, OBJECT(machine), &jaeger_sdma_ops,
                          rms, "jaeger.sdma", JAEGER_SDMA_SIZE);
    memory_region_add_subregion(get_system_memory(), FSL_IMX6_SDMA_ADDR,
                                &rms->sdma_iomem);

    memory_region_init_io(&rms->dcp_iomem, OBJECT(machine), &jaeger_dcp_ops,
                          rms, "jaeger.dcp", JAEGER_DCP_SIZE);
    memory_region_add_subregion(get_system_memory(), JAEGER_DCP_ADDR,
                                &rms->dcp_iomem);

    sysbus_realize(SYS_BUS_DEVICE(&rms->rngc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&rms->rngc), 0, 0x021b4000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&rms->rngc), 0,
                       qdev_get_gpio_in(DEVICE(&s->a9mpcore), 5));

    pxp = qdev_new(TYPE_IMX6SL_PXP);
    object_property_add_child(OBJECT(machine), "pxp", OBJECT(pxp));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pxp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pxp), 0, JAEGER_PXP_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(pxp), 0,
                       qdev_get_gpio_in(DEVICE(&s->a9mpcore),
                                       JAEGER_PXP_GIC_IRQ));

    epdc = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(machine), "epdc", OBJECT(epdc));
    object_property_set_link(OBJECT(epdc), "pxp", OBJECT(pxp), &error_fatal);
    qdev_prop_set_uint64(epdc, "fb-addr", JAEGER_FB_ADDR);
    qdev_prop_set_uint32(epdc, "fb-width", JAEGER_FB_WIDTH);
    qdev_prop_set_uint32(epdc, "fb-height", JAEGER_FB_HEIGHT);
    qdev_prop_set_uint32(epdc, "fb-stride", JAEGER_FB_STRIDE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(epdc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(epdc), 0, JAEGER_EPDC_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(epdc), 0,
                       qdev_get_gpio_in(DEVICE(&s->a9mpcore),
                                       JAEGER_EPDC_GIC_IRQ));

    ocotp_dev = qdev_new(TYPE_IMX6SLL_OCOTP);
    ocotp = IMX6SLL_OCOTP(ocotp_dev);
    object_property_add_child(OBJECT(machine), "ocotp", OBJECT(ocotp_dev));
    if (rms->fuse_overrides) {
        qdev_prop_set_string(ocotp_dev, "shadow-overrides",
                             rms->fuse_overrides);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(ocotp_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(ocotp_dev), 0, FSL_IMX6_OCOTPCTRL_ADDR);
    jaeger_configure_identity(s, ocotp, rms);
    imx6sll_ocotp_apply_shadow_overrides(ocotp);
    rms->soc = s;
    rms->ocotp = ocotp;

    memory_region_add_subregion(get_system_memory(), JAEGER_RAM_BASE,
                                machine->ram);
    jaeger_load_firmware(machine);
    jaeger_attach_emmc(s, rms);
    jaeger_attach_wifi(s);
    pmic = qdev_new(TYPE_BD71827);
    qdev_prop_set_uint8(pmic, "address", 0x4b);
    qdev_realize(pmic, BUS(s->i2c[0].bus), &error_fatal);
    qdev_connect_gpio_out_named(pmic, "irq", 0,
                                qdev_get_gpio_in(DEVICE(&s->gpio[3]), 24));
    object_unref(OBJECT(pmic));
    i2c_slave_create_simple(s->i2c[0].bus, TYPE_LM3695, 0x63);
    i2c_slave_create_simple(s->i2c[0].bus, TYPE_FP9929, 0x48);

    /* The landscape sensor is rotated by DT FLIP|INV_X into portrait. */
    touch = qdev_new("cyttsp5");
    qdev_prop_set_uint8(touch, "address", 0x24);
    qdev_prop_set_uint16(touch, "width", 800);
    qdev_prop_set_uint16(touch, "height", 600);
    qdev_prop_set_bit(touch, "invert-x", false);
    qdev_prop_set_bit(touch, "invert-y", true);
    qdev_prop_set_bit(touch, "swap-axes", true);
    qdev_realize(touch, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out(touch, 0,
                          qdev_get_gpio_in(DEVICE(&s->gpio[3]), 3));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 5,
                          qdev_get_gpio_in(touch, 0));
    object_unref(OBJECT(touch));

    keyboard = qdev_new(TYPE_WARIO_KEYBOARD);
    qdev_prop_set_bit(keyboard, "page-buttons", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(keyboard), &error_fatal);
    qdev_connect_gpio_out_named(keyboard, "power-button", 0,
                                qdev_get_gpio_in_named(pmic,
                                                      "power-button", 0));

    mmdc = qdev_new(TYPE_IMX6SL_MMDC);
    object_property_add_child(OBJECT(machine), "mmdc", OBJECT(mmdc));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(mmdc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mmdc), 0, 0x021b0000);

    iomuxc = qdev_new(TYPE_IMX6SL_IOMUXC);
    object_property_add_child(OBJECT(machine), "iomuxc", OBJECT(iomuxc));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(iomuxc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(iomuxc), 0, 0x020e0000);

    if (machine->firmware) {
        qemu_register_reset(jaeger_firmware_reset, rms);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&s->cpu[0], machine, &jaeger_boot_info);
    }
}

static char *jaeger_idme_get(char **value)
{
    return g_strdup(*value);
}

static void jaeger_idme_set(char **field, const char *value)
{
    g_free(*field);
    *field = g_strdup(value);
}

#define JAEGER_IDME_PROPERTY(_member)                                      \
    static char *jaeger_get_##_member(Object *obj, Error **errp)           \
    {                                                                   \
        return jaeger_idme_get(&JAEGER_MACHINE(obj)->_member);                \
    }                                                                   \
    static void jaeger_set_##_member(Object *obj, const char *value,       \
                                  Error **errp)                         \
    {                                                                   \
        jaeger_idme_set(&JAEGER_MACHINE(obj)->_member, value);                \
    }

JAEGER_IDME_PROPERTY(idme_serial)
JAEGER_IDME_PROPERTY(idme_mac)
JAEGER_IDME_PROPERTY(idme_mfg)
JAEGER_IDME_PROPERTY(idme_pcbsn)
JAEGER_IDME_PROPERTY(idme_bootmode)
JAEGER_IDME_PROPERTY(idme_postmode)

static char *jaeger_get_falcon_bios(Object *obj, Error **errp)
{
    return g_strdup(JAEGER_MACHINE(obj)->falcon_bios);
}

static void jaeger_set_falcon_bios(Object *obj, const char *value, Error **errp)
{
    jaeger_idme_set(&JAEGER_MACHINE(obj)->falcon_bios, value);
}

static char *jaeger_get_storage_bios(Object *obj, Error **errp)
{
    return g_strdup(JAEGER_MACHINE(obj)->storage_bios);
}

static void jaeger_set_storage_bios(Object *obj, const char *value, Error **errp)
{
    jaeger_idme_set(&JAEGER_MACHINE(obj)->storage_bios, value);
}

static char *jaeger_get_device_profile(Object *obj, Error **errp)
{
    return g_strdup(JAEGER_MACHINE(obj)->device_profile);
}

static void jaeger_set_device_profile(Object *obj, const char *value,
                                   Error **errp)
{
    jaeger_idme_set(&JAEGER_MACHINE(obj)->device_profile, value);
}

static char *jaeger_get_boot_partitions_file(Object *obj, Error **errp)
{
    return jaeger_idme_get(&JAEGER_MACHINE(obj)->boot_partitions_file);
}

static void jaeger_set_boot_partitions_file(Object *obj, const char *value,
                                         Error **errp)
{
    jaeger_idme_set(&JAEGER_MACHINE(obj)->boot_partitions_file, value);
}

static char *jaeger_get_fuse_overrides(Object *obj, Error **errp)
{
    return jaeger_idme_get(&JAEGER_MACHINE(obj)->fuse_overrides);
}

static void jaeger_set_fuse_overrides(Object *obj, const char *value,
                                   Error **errp)
{
    jaeger_idme_set(&JAEGER_MACHINE(obj)->fuse_overrides, value);
}

static void jaeger_machine_instance_init(Object *obj)
{
    JaegerMachineState *rms = JAEGER_MACHINE(obj);

    object_initialize_child(obj, "rngc", &rms->rngc, TYPE_IMX_RNGC);

    rms->idme_serial = g_strdup("G000WF0400000000");
    rms->idme_mac = g_strdup("020000000010");
    rms->idme_mfg = g_strdup("JAEGERQEMU0000000");
    rms->idme_pcbsn = g_strdup("0001M70000000000");
    rms->idme_bootmode = g_strdup("0");
    rms->idme_postmode = g_strdup("0");
    rms->device_profile = g_strdup("production");
}

static void jaeger_machine_instance_finalize(Object *obj)
{
    JaegerMachineState *rms = JAEGER_MACHINE(obj);

    g_free(rms->idme_serial);
    g_free(rms->idme_mac);
    g_free(rms->idme_mfg);
    g_free(rms->idme_pcbsn);
    g_free(rms->idme_bootmode);
    g_free(rms->idme_postmode);
    g_free(rms->device_profile);
    g_free(rms->falcon_bios);
    g_free(rms->storage_bios);
    g_free(rms->boot_partitions_file);
    g_free(rms->fuse_overrides);
}

static void jaeger_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Lab126 Jaeger / Kindle 10th generation basic (i.MX6SLL)";
    mc->init = jaeger_init;
    mc->max_cpus = 1;
    mc->default_cpus = 1;
    mc->minimum_page_bits = 12;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "jaeger.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;

    object_class_property_add_str(oc, "idme-serial", jaeger_get_idme_serial,
                                  jaeger_set_idme_serial);
    object_class_property_add_str(oc, "idme-mac", jaeger_get_idme_mac,
                                  jaeger_set_idme_mac);
    object_class_property_add_str(oc, "idme-mfg", jaeger_get_idme_mfg,
                                  jaeger_set_idme_mfg);
    object_class_property_add_str(oc, "idme-pcbsn", jaeger_get_idme_pcbsn,
                                  jaeger_set_idme_pcbsn);
    object_class_property_add_str(oc, "idme-bootmode", jaeger_get_idme_bootmode,
                                  jaeger_set_idme_bootmode);
    object_class_property_add_str(oc, "idme-postmode", jaeger_get_idme_postmode,
                                  jaeger_set_idme_postmode);
    object_class_property_add_str(oc, "device-profile",
                                  jaeger_get_device_profile,
                                  jaeger_set_device_profile);
    object_class_property_add_str(oc, "falcon-bios", jaeger_get_falcon_bios,
                                  jaeger_set_falcon_bios);
    object_class_property_add_str(oc, "storage-bios", jaeger_get_storage_bios,
                                  jaeger_set_storage_bios);
    object_class_property_add_str(oc, "boot-partitions-file",
                                  jaeger_get_boot_partitions_file,
                                  jaeger_set_boot_partitions_file);
    object_class_property_add_str(oc, "fuse-overrides",
                                  jaeger_get_fuse_overrides,
                                  jaeger_set_fuse_overrides);
}

static const TypeInfo jaeger_machine_type = {
    .name = TYPE_JAEGER_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(JaegerMachineState),
    .instance_init = jaeger_machine_instance_init,
    .instance_finalize = jaeger_machine_instance_finalize,
    .class_init = jaeger_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void jaeger_machine_register_types(void)
{
    type_register_static(&jaeger_machine_type);
}
type_init(jaeger_machine_register_types)
