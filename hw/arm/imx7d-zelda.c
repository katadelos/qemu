/*
 * Amazon/Lab126 Cognac (Kindle Oasis 2, Zelda firmware platform) board emulation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/arm/fsl-imx7.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/imx_epdc.h"
#include "hw/i2c/max77796.h"
#include "hw/i2c/bma2x2.h"
#include "hw/misc/imx6sl_pxp.h"
#include "hw/misc/imx7_ddrc.h"
#include "hw/misc/max44009.h"
#include "hw/sd/sd.h"
#include "hw/ssi/ssi.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/dma.h"
#include "system/qtest.h"
#include "system/reset.h"

#define ZELDA_RAM_BASE        0x80000000
#define ZELDA_UBOOT_MAX       (1 * MiB)
#define ZELDA_MACHINE_ID      -1
#define ZELDA_IDME_SIZE       0x4000
#define ZELDA_IDME_VERSION    "2.1"
#define ZELDA_SBIOS_OFFSET    0
#define ZELDA_SBIOS_SIZE      0x10000
#define ZELDA_FBIOS_OFFSET    (ZELDA_SBIOS_OFFSET + ZELDA_SBIOS_SIZE)
#define ZELDA_FBIOS_SIZE      0x20000
#define ZELDA_SBIOS_ADDR      0x9ffc0000
#define ZELDA_FBIOS_ADDR      0x9ffd0000
#define ZELDA_BIOS_MARKER     0x51494255
#define ZELDA_SDMA_SIZE       0x1000
#define ZELDA_SDMA_C0PTR      0x00
#define ZELDA_SDMA_INTR       0x04
#define ZELDA_SDMA_START      0x0c
#define ZELDA_SDMA_RESET      0x24
#define ZELDA_PXP_ADDR        0x30700000
#define ZELDA_PXP_GIC_IRQ     8
#define ZELDA_EPDC_ADDR       0x306f0000
#define ZELDA_EPDC_GIC_IRQ    117
#define ZELDA_FB_WIDTH        1264
#define ZELDA_FB_HEIGHT       1680

#define IMX_IVT_HEADER      0x402000d1
#define IMX_IVT_ENTRY       0x04
#define IMX_IVT_BOOT_DATA   0x10
#define IMX_IVT_SELF        0x14
#define IMX_IVT_SIZE        0x20
#define IMX_BOOT_PLUGIN     0x08
#define IMX_BOOT_DATA_SIZE  0x0c

#define TYPE_ZELDA_MACHINE MACHINE_TYPE_NAME("imx7d-zelda")
OBJECT_DECLARE_SIMPLE_TYPE(ZeldaMachineState, ZELDA_MACHINE)

typedef struct ZeldaMachineState {
    MachineState parent_obj;
    char *idme_serial;
    char *idme_mac;
    char *idme_mfg;
    char *idme_pcbsn;
    char *idme_bootmode;
    char *idme_postmode;
    char *falcon_bios;
    char *storage_bios;
    char *boot_partitions_file;
    FslIMX7State *soc;
    MemoryRegion ocotp_iomem;
    uint32_t ocotp[0x1000 / 4];
    MemoryRegion sdma_iomem;
    uint32_t sdma[ZELDA_SDMA_SIZE / sizeof(uint32_t)];
    qemu_irq sdma_irq;
    MemoryRegion adc_iomem;
    uint32_t adc[0x1000 / 4];
    MemoryRegion counter_iomem;
    uint32_t counter[0x1000 / 4];
} ZeldaMachineState;

typedef struct ZeldaIdmeField {
    const char *name;
    const char *value;
    size_t size;
    bool exportable;
} ZeldaIdmeField;

static struct arm_boot_info zelda_boot_info;

static void zelda_sdma_complete_channel0(ZeldaMachineState *rms)
{
    uint32_t ccb_addr = rms->sdma[ZELDA_SDMA_C0PTR / 4];
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

    rms->sdma[ZELDA_SDMA_INTR / 4] |= BIT(0);
    qemu_set_irq(rms->sdma_irq, 1);
}

static uint64_t zelda_sdma_read(void *opaque, hwaddr offset, unsigned size)
{
    ZeldaMachineState *rms = opaque;

    return rms->sdma[offset / 4];
}

static void zelda_sdma_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    ZeldaMachineState *rms = opaque;

    switch (offset) {
    case ZELDA_SDMA_INTR:
        rms->sdma[offset / 4] &= ~(uint32_t)value;
        qemu_set_irq(rms->sdma_irq, rms->sdma[offset / 4] != 0);
        break;
    case ZELDA_SDMA_START:
        if (value & BIT(0)) {
            zelda_sdma_complete_channel0(rms);
        }
        break;
    case ZELDA_SDMA_RESET:
        memset(rms->sdma, 0, sizeof(rms->sdma));
        qemu_set_irq(rms->sdma_irq, 0);
        break;
    default:
        rms->sdma[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps zelda_sdma_ops = {
    .read = zelda_sdma_read,
    .write = zelda_sdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t zelda_adc_read(void *opaque, hwaddr offset, unsigned size)
{
    ZeldaMachineState *s = opaque;
    unsigned hwid = 5; /* Cognac Wi-Fi DVT/PVT resistor straps. */
    if (offset == 0xf0 || offset == 0x100) {
        unsigned shift = offset == 0xf0 ? 0 : 2;
        return ((hwid & (1 << shift)) ? 0xfff : 0) |
               ((hwid & (2 << shift)) ? 0xfff0000 : 0);
    }
    if (offset == 0xe0) {
        return 0xf00; /* All four single conversions are complete. */
    }
    return s->adc[offset / 4];
}

static void zelda_adc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    ZeldaMachineState *s = opaque;
    s->adc[offset / 4] = value;
}

static const MemoryRegionOps zelda_adc_ops = {
    .read = zelda_adc_read,
    .write = zelda_adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t zelda_counter_read(void *opaque, hwaddr offset, unsigned size)
{
    ZeldaMachineState *s = opaque;
    uint64_t ticks = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 125;
    switch (offset) {
    case 0x08: return (uint32_t)ticks;
    case 0x0c: return ticks >> 32;
    case 0x20: return 8000000;
    default: return s->counter[offset / 4];
    }
}

static void zelda_counter_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    ZeldaMachineState *s = opaque;
    s->counter[offset / 4] = value;
}

static const MemoryRegionOps zelda_counter_ops = {
    .read = zelda_counter_read,
    .write = zelda_counter_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t zelda_ocotp_read(void *opaque, hwaddr offset, unsigned size)
{
    ZeldaMachineState *s = opaque;
    return s->ocotp[offset / 4];
}

static void zelda_ocotp_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    ZeldaMachineState *s = opaque;
    unsigned base = offset & ~0xf;
    unsigned bank = s->ocotp[0] & 0xf;
    unsigned i;
    switch (offset & 0xc) {
    case 0: s->ocotp[base / 4] = value; break;
    case 4: s->ocotp[base / 4] |= value; break;
    case 8: s->ocotp[base / 4] &= ~value; break;
    case 12: s->ocotp[base / 4] ^= value; break;
    }
    if (base == 0x60 && (value & 1)) {
        for (i = 0; i < 4; i++) {
            s->ocotp[(0x70 + i * 0x10) / 4] =
                s->ocotp[(0x400 + bank * 0x40 + i * 0x10) / 4];
        }
    }
    s->ocotp[0] &= ~(BIT(8) | BIT(9) | BIT(10));
}

static const MemoryRegionOps zelda_ocotp_ops = {
    .read = zelda_ocotp_read,
    .write = zelda_ocotp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void zelda_ocotp_reset(ZeldaMachineState *s)
{
    static const uint32_t srk[] = {
        0x7728d930, 0x840fe74c, 0x6b402fbb, 0x1112612a,
        0xa99305de, 0x40017071, 0x26e8623f, 0xe34e6f12,
    };
    unsigned i;

    memset(s->ocotp, 0, sizeof(s->ocotp));
    s->ocotp[0x140 / 4] = 0x10000;
    s->ocotp[0x400 / 4] = BIT(9);  /* SRK fuse lock. */
    s->ocotp[0x470 / 4] = BIT(25); /* Closed secure-boot configuration. */
    for (i = 0; i < ARRAY_SIZE(srk); i++) {
        s->ocotp[(0x580 + i * 0x10) / 4] = srk[i];
    }
}

static void zelda_firmware_reset(void *opaque)
{
    ZeldaMachineState *s = opaque;
    ARMCPU *cpu = &s->soc->cpu[0];

    zelda_ocotp_reset(s);
    memset(s->sdma, 0, sizeof(s->sdma));
    memset(s->adc, 0, sizeof(s->adc));
    memset(s->counter, 0, sizeof(s->counter));
    qemu_set_irq(s->sdma_irq, 0);
    /* The boot ROM has already selected USDHC3 and initialized DDR. */
    s->soc->src.regs[SRC_SBMR1] = 0x2810;
    cpu_reset(CPU(cpu));
    cpu_set_pc(CPU(cpu), zelda_boot_info.entry);
}

static void zelda_populate_idme(DeviceState *card, ZeldaMachineState *rms)
{
    g_autofree uint8_t *contents = g_malloc0(ZELDA_IDME_SIZE);
    ZeldaIdmeField fields[] = {
        { "board_id",       rms->idme_pcbsn,   16, true  },
        { "serial",         rms->idme_serial,  16, true  },
        { "mac_addr",       rms->idme_mac,     16, true  },
        { "mac_sec",        "0",               32, true  },
        { "bt_mac_addr",    "0",               16, true  },
        { "bt_mfg",         "0",              128, true  },
        { "product_name",   "0",               32, true  },
        { "productid",      "0",               32, true  },
        { "productid2",     "0",               32, true  },
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
    };
    uint8_t *cursor = contents;
    size_t i;

    memcpy(cursor, "beefdeed", 8);
    memcpy(cursor + 8, ZELDA_IDME_VERSION, sizeof(ZELDA_IDME_VERSION));
    stl_le_p(cursor + 12, ARRAY_SIZE(fields));
    cursor += 16;

    for (i = 0; i < ARRAY_SIZE(fields); i++) {
        ZeldaIdmeField *field = &fields[i];
        size_t len = strlen(field->value);

        if (strlen(field->name) > 16 || !g_str_is_ascii(field->value) ||
            len > field->size ||
            cursor + 28 + field->size > contents + ZELDA_IDME_SIZE) {
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

    /* IDME occupies 16 KiB at byte offset 1 MiB in eMMC boot area 1. */
    emmc_boot_partition_write(card, 1, 0x100000, contents, ZELDA_IDME_SIZE,
                              &error_fatal);
}

static bool zelda_idme_is_populated(DeviceState *card)
{
    uint8_t header[12];
    Error *err = NULL;

    emmc_boot_partition_read(card, 1, 0x100000, header, sizeof(header), &err);
    if (err) {
        error_report_err(err);
        exit(EXIT_FAILURE);
    }
    return !memcmp(header, "beefdeed", 8) &&
           !memcmp(header + 8, ZELDA_IDME_VERSION, sizeof(ZELDA_IDME_VERSION));
}

static void zelda_populate_falcon_image(DeviceState *card, const char *path,
                                      const char *name, uint64_t offset,
                                      size_t limit, size_t marker_offset,
                                      hwaddr load_addr)
{
    g_autofree uint8_t *contents = NULL;
    g_autoptr(GError) error = NULL;
    gsize size;

    if (!g_file_get_contents(path, (char **)&contents, &size, &error)) {
        error_report("unable to read Zelda %s '%s': %s", name, path,
                     error->message);
        exit(EXIT_FAILURE);
    }
    if (size > limit || size < marker_offset + sizeof(uint32_t) ||
        ldl_le_p(contents + marker_offset) != ZELDA_BIOS_MARKER) {
        error_report("Zelda %s '%s' is not a valid image", name, path);
        exit(EXIT_FAILURE);
    }

    emmc_boot_partition_write(card, 2, offset, contents, size, &error_fatal);
    /* The real DDR plugin leaves both images resident for U-Boot. */
    rom_add_blob_fixed(name, contents, size, load_addr);
    cpu_physical_memory_write(load_addr, contents, size);
}

static void zelda_attach_emmc(FslIMX7State *s, ZeldaMachineState *rms)
{
    DriveInfo *di = drive_get(IF_SD, 0, 2);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus = qdev_get_child_bus(DEVICE(&s->usdhc[2]), "sd-bus");
    DeviceState *card = qdev_new(TYPE_EMMC);

    qdev_prop_set_uint64(card, "boot-partition-size", 2 * MiB);
    qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    if (rms->boot_partitions_file) {
        qdev_prop_set_string(card, "boot-partitions-file",
                             rms->boot_partitions_file);
    }
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);
    if (!!rms->falcon_bios != !!rms->storage_bios) {
        error_report("Zelda falcon-bios and storage-bios must be supplied together");
        exit(EXIT_FAILURE);
    }
    if (rms->storage_bios) {
        /* Oasis 2 keeps the stock quickboot firmware in eMMC boot area 2. */
        zelda_populate_falcon_image(card, rms->storage_bios, "storage BIOS",
                                  ZELDA_SBIOS_OFFSET, ZELDA_SBIOS_SIZE, 0x20,
                                  ZELDA_SBIOS_ADDR);
        zelda_populate_falcon_image(card, rms->falcon_bios, "Falcon BIOS",
                                  ZELDA_FBIOS_OFFSET, ZELDA_FBIOS_SIZE, 0x30,
                                  ZELDA_FBIOS_ADDR);
    }
    if (!zelda_idme_is_populated(card)) {
        zelda_populate_idme(card, rms);
    }
    object_unref(OBJECT(card));
}

static void zelda_attach_wifi(FslIMX7State *s)
{
    BusState *bus = qdev_get_child_bus(DEVICE(&s->usdhc[0]), "sd-bus");
    DeviceState *wifi = qdev_new(TYPE_BCM43430_SDIO);

    /*
     * Cognac connects its BCM43430 Wi-Fi function to USDHC1.
     */
    qdev_realize(wifi, bus, &error_fatal);
    qemu_set_irq(qdev_get_gpio_in_named(wifi, "power", 0), 1);
    qdev_connect_gpio_out_named(
        wifi, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&s->usdhc[0]), "sdio-irq", 0));
    object_unref(OBJECT(wifi));
}

static void zelda_attach_spinor(FslIMX7State *s)
{
    SSIBus *bus = (SSIBus *)qdev_get_child_bus(DEVICE(&s->spi[0]), "spi");
    DeviceState *flash = qdev_new("mx25l4005a");

    qdev_realize_and_unref(flash, BUS(bus), &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 19,
                          qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0));
}

static size_t zelda_find_uboot_ivt(const uint8_t *image, size_t size)
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

static void zelda_load_firmware(MachineState *machine)
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
        image_size > ZELDA_UBOOT_MAX ||
        ldl_le_p(image) != IMX_IVT_HEADER) {
        error_report("Zelda firmware is not a supported i.MX boot image");
        exit(EXIT_FAILURE);
    }

    first_boot_ptr = ldl_le_p(image + IMX_IVT_BOOT_DATA);
    first_self = ldl_le_p(image + IMX_IVT_SELF);
    if (first_boot_ptr < first_self) {
        error_report("Zelda firmware has an invalid plugin boot-data pointer");
        exit(EXIT_FAILURE);
    }
    first_boot_offset = first_boot_ptr - first_self;
    if (first_boot_offset + IMX_BOOT_DATA_SIZE > image_size ||
        ldl_le_p(image + first_boot_offset + IMX_BOOT_PLUGIN) != 1) {
        error_report("Zelda firmware does not contain the expected DDR plugin");
        exit(EXIT_FAILURE);
    }

    ivt_offset = zelda_find_uboot_ivt(image, image_size);
    if (ivt_offset == SIZE_MAX) {
        error_report("Zelda firmware has no second-stage U-Boot IVT");
        exit(EXIT_FAILURE);
    }
    entry = ldl_le_p(image + ivt_offset + IMX_IVT_ENTRY);
    self = ldl_le_p(image + ivt_offset + IMX_IVT_SELF);
    if (entry < self || entry - self > image_size - ivt_offset) {
        error_report("Zelda U-Boot entry point is outside the firmware image");
        exit(EXIT_FAILURE);
    }

    /*
     * The real Boot ROM runs the OCRAM plugin, which initializes LPDDR and
     * asks the ROM to copy the complete image to DDR.  RAM is already usable
     * in QEMU, so reproduce the resulting copy and enter U-Boot's second IVT.
     */
    load_addr = self - ivt_offset;
    if (load_addr < ZELDA_RAM_BASE ||
        load_addr + image_size > ZELDA_RAM_BASE + machine->ram_size) {
        error_report("Zelda U-Boot load range is outside guest RAM");
        exit(EXIT_FAILURE);
    }
    loaded = load_image_targphys(machine->firmware, load_addr,
                                 ZELDA_UBOOT_MAX, NULL);
    if (loaded != image_size) {
        error_report("Unable to load Zelda firmware '%s'", machine->firmware);
        exit(EXIT_FAILURE);
    }
    zelda_boot_info.entry = entry;
}

static void zelda_init(MachineState *machine)
{
    ZeldaMachineState *rms = ZELDA_MACHINE(machine);
    FslIMX7State *s;
    DeviceState *epdc, *pxp, *pmic, *touch, *ddrc;
    int i;

    if (machine->ram_size != 512 * MiB) {
        error_report("Zelda requires 512 MiB RAM");
        exit(EXIT_FAILURE);
    }
    zelda_boot_info = (struct arm_boot_info) {
        .loader_start = ZELDA_RAM_BASE,
        .board_id = ZELDA_MACHINE_ID,
        .ram_size = machine->ram_size,
    };
    s = FSL_IMX7(object_new(TYPE_FSL_IMX7));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(&s->usdhc[2]), "defer-data-transfer",
                             false, &error_fatal);
    object_property_set_uint(OBJECT(&s->gpio[3]), "reset-psr", 0x3f00,
                             &error_fatal);
    object_property_set_uint(OBJECT(&s->gpio[0]), "reset-psr", BIT(4),
                             &error_fatal);
    object_property_set_uint(OBJECT(&s->gpio[2]), "reset-psr", BIT(20) | BIT(21),
                             &error_fatal);
    object_property_set_uint(OBJECT(&s->gpio[1]), "reset-psr", BIT(31) | BIT(22),
                             &error_fatal);
    qdev_realize(DEVICE(s), NULL, &error_fatal);
    rms->soc = s;
    ddrc = qdev_new(TYPE_IMX7_DDRC);
    object_property_add_child(OBJECT(machine), "ddrc", OBJECT(ddrc));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(ddrc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(ddrc), 0, 0x307a0000);
    sysbus_mmio_map(SYS_BUS_DEVICE(ddrc), 1, 0x30790000);
    rms->sdma_irq = qdev_get_gpio_in(DEVICE(&s->gpcv2), 2);
    memory_region_init_io(&rms->sdma_iomem, OBJECT(machine), &zelda_sdma_ops,
                          rms, "zelda.sdma", ZELDA_SDMA_SIZE);
    memory_region_add_subregion_overlap(get_system_memory(), FSL_IMX7_SDMA_ADDR,
                                        &rms->sdma_iomem, 1);
    memory_region_init_io(&rms->adc_iomem, OBJECT(machine), &zelda_adc_ops,
                          rms, "zelda.adc", sizeof(rms->adc));
    memory_region_add_subregion_overlap(get_system_memory(), FSL_IMX7_ADC1_ADDR,
                                        &rms->adc_iomem, 1);
    memory_region_init_io(&rms->counter_iomem, OBJECT(machine), &zelda_counter_ops,
                          rms, "zelda.counter", sizeof(rms->counter));
    memory_region_add_subregion(get_system_memory(), 0x306c0000,
                                &rms->counter_iomem);
    pxp = qdev_new(TYPE_IMX6SL_PXP);
    object_property_add_child(OBJECT(machine), "pxp", OBJECT(pxp));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pxp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pxp), 0, ZELDA_PXP_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(pxp), 0,
                       qdev_get_gpio_in(DEVICE(&s->gpcv2), ZELDA_PXP_GIC_IRQ));
    epdc = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(machine), "epdc", OBJECT(epdc));
    object_property_set_link(OBJECT(epdc), "pxp", OBJECT(pxp), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(epdc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(epdc), 0, ZELDA_EPDC_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(epdc), 0,
                       qdev_get_gpio_in(DEVICE(&s->gpcv2), ZELDA_EPDC_GIC_IRQ));
    memory_region_init_io(&rms->ocotp_iomem, OBJECT(machine), &zelda_ocotp_ops,
                          rms, "zelda.ocotp", sizeof(rms->ocotp));
    memory_region_add_subregion_overlap(get_system_memory(), FSL_IMX7_OCOTP_ADDR,
                                        &rms->ocotp_iomem, 1);
    memory_region_add_subregion(get_system_memory(), ZELDA_RAM_BASE, machine->ram);
    {
        uint32_t pointer = cpu_to_le32(0x200);
        uint8_t info[32] = { 0, 2, 2, 0 };

        stl_le_p(info + 4, 1000000000);
        stl_le_p(info + 8, 332000000);
        stl_le_p(info + 12, 400000000);
        stl_le_p(info + 16, 24000000);
        rom_add_blob_fixed("zelda.rom-sw-info-pointer", &pointer,
                           sizeof(pointer), 0x1e8);
        rom_add_blob_fixed("zelda.rom-sw-info", info, sizeof(info), 0x200);
    }
    zelda_load_firmware(machine);
    zelda_attach_emmc(s, rms);
    zelda_attach_wifi(s);
    zelda_attach_spinor(s);
    for (i = 0; i < 4; i++) {
        static const unsigned addresses[] = { 0x3c, 0x35, 0x34, 0x68 };
        pmic = qdev_new(TYPE_MAX77796);
        qdev_prop_set_uint8(pmic, "address", addresses[i]);
        qdev_realize(pmic, BUS(s->i2c[0].bus), &error_fatal);
        if (!i) {
            qdev_connect_gpio_out(pmic, 0,
                                  qdev_get_gpio_in(DEVICE(&s->gpio[0]), 4));
            qdev_connect_gpio_out(pmic, 1,
                                  qdev_get_gpio_in(DEVICE(&s->gpio[1]), 31));
            qdev_connect_gpio_out_named(pmic, "page-button", 0,
                                  qdev_get_gpio_in(DEVICE(&s->gpio[2]), 20));
            qdev_connect_gpio_out_named(pmic, "page-button", 1,
                                  qdev_get_gpio_in(DEVICE(&s->gpio[2]), 21));
        }
        object_unref(OBJECT(pmic));
    }
    touch = qdev_new("cyttsp5");
    qdev_prop_set_uint8(touch, "address", 0x24);
    qdev_prop_set_uint16(touch, "width", ZELDA_FB_WIDTH);
    qdev_prop_set_uint16(touch, "height", ZELDA_FB_HEIGHT);
    qdev_prop_set_bit(touch, "invert-x", true);
    qdev_prop_set_bit(touch, "invert-y", true);
    qdev_realize(touch, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out(touch, 0,
                          qdev_get_gpio_in(DEVICE(&s->gpio[0]), 10));
    qdev_connect_gpio_out(DEVICE(&s->gpio[0]), 11, qdev_get_gpio_in(touch, 0));
    object_unref(OBJECT(touch));
    touch = qdev_new(TYPE_BMA2X2);
    qdev_prop_set_uint8(touch, "address", 0x18);
    qdev_realize(touch, BUS(s->i2c[2].bus), &error_fatal);
    qdev_connect_gpio_out(touch, 0,
                          qdev_get_gpio_in(DEVICE(&s->gpio[2]), 5));
    object_unref(OBJECT(touch));
    i2c_slave_create_simple(s->i2c[2].bus, TYPE_MAX44009, 0x4a);
    i2c_slave_create_simple(s->i2c[2].bus, TYPE_MAX44009, 0x4b);
    if (machine->firmware) {
        qemu_register_reset(zelda_firmware_reset, rms);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&s->cpu[0], machine, &zelda_boot_info);
    }
}

static char *zelda_idme_get(char **value)
{
    return g_strdup(*value);
}

static void zelda_idme_set(char **field, const char *value)
{
    g_free(*field);
    *field = g_strdup(value);
}

#define ZELDA_IDME_PROPERTY(_member)                                      \
    static char *zelda_get_##_member(Object *obj, Error **errp)           \
    {                                                                   \
        return zelda_idme_get(&ZELDA_MACHINE(obj)->_member);                \
    }                                                                   \
    static void zelda_set_##_member(Object *obj, const char *value,       \
                                  Error **errp)                         \
    {                                                                   \
        zelda_idme_set(&ZELDA_MACHINE(obj)->_member, value);                \
    }

ZELDA_IDME_PROPERTY(idme_serial)
ZELDA_IDME_PROPERTY(idme_mac)
ZELDA_IDME_PROPERTY(idme_mfg)
ZELDA_IDME_PROPERTY(idme_pcbsn)
ZELDA_IDME_PROPERTY(idme_bootmode)
ZELDA_IDME_PROPERTY(idme_postmode)

static char *zelda_get_falcon_bios(Object *obj, Error **errp)
{
    return g_strdup(ZELDA_MACHINE(obj)->falcon_bios);
}

static void zelda_set_falcon_bios(Object *obj, const char *value, Error **errp)
{
    zelda_idme_set(&ZELDA_MACHINE(obj)->falcon_bios, value);
}

static char *zelda_get_storage_bios(Object *obj, Error **errp)
{
    return g_strdup(ZELDA_MACHINE(obj)->storage_bios);
}

static void zelda_set_storage_bios(Object *obj, const char *value, Error **errp)
{
    zelda_idme_set(&ZELDA_MACHINE(obj)->storage_bios, value);
}

static char *zelda_get_boot_partitions_file(Object *obj, Error **errp)
{
    return zelda_idme_get(&ZELDA_MACHINE(obj)->boot_partitions_file);
}

static void zelda_set_boot_partitions_file(Object *obj, const char *value,
                                         Error **errp)
{
    zelda_idme_set(&ZELDA_MACHINE(obj)->boot_partitions_file, value);
}

static void zelda_machine_instance_init(Object *obj)
{
    ZeldaMachineState *rms = ZELDA_MACHINE(obj);

    rms->idme_serial = g_strdup("G000SA0000000000");
    rms->idme_mac = g_strdup("020000000004");
    rms->idme_mfg = g_strdup("ZELDAQEMU0000000000000");
    /* PSN v2 prefix is also required by userspace's Wi-Fi-only detection. */
    rms->idme_pcbsn = g_strdup("P0013H0000000000");
    rms->idme_bootmode = g_strdup("0");
    rms->idme_postmode = g_strdup("0");
}

static void zelda_machine_instance_finalize(Object *obj)
{
    ZeldaMachineState *rms = ZELDA_MACHINE(obj);

    g_free(rms->idme_serial);
    g_free(rms->idme_mac);
    g_free(rms->idme_mfg);
    g_free(rms->idme_pcbsn);
    g_free(rms->idme_bootmode);
    g_free(rms->idme_postmode);
    g_free(rms->falcon_bios);
    g_free(rms->storage_bios);
    g_free(rms->boot_partitions_file);
}

static void zelda_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Lab126 Cognac / Kindle Oasis 2 (i.MX7D)";
    mc->init = zelda_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "zelda.ram";
    /* Cortex-A7 TLB invalidation operates on at least 4 KiB pages. */
    mc->minimum_page_bits = 12;
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;

    object_class_property_add_str(oc, "idme-serial", zelda_get_idme_serial,
                                  zelda_set_idme_serial);
    object_class_property_add_str(oc, "idme-mac", zelda_get_idme_mac,
                                  zelda_set_idme_mac);
    object_class_property_add_str(oc, "idme-mfg", zelda_get_idme_mfg,
                                  zelda_set_idme_mfg);
    object_class_property_add_str(oc, "idme-pcbsn", zelda_get_idme_pcbsn,
                                  zelda_set_idme_pcbsn);
    object_class_property_add_str(oc, "idme-bootmode", zelda_get_idme_bootmode,
                                  zelda_set_idme_bootmode);
    object_class_property_add_str(oc, "idme-postmode", zelda_get_idme_postmode,
                                  zelda_set_idme_postmode);
    object_class_property_add_str(oc, "falcon-bios", zelda_get_falcon_bios,
                                  zelda_set_falcon_bios);
    object_class_property_add_str(oc, "storage-bios", zelda_get_storage_bios,
                                  zelda_set_storage_bios);
    object_class_property_add_str(oc, "boot-partitions-file",
                                  zelda_get_boot_partitions_file,
                                  zelda_set_boot_partitions_file);
}

static const TypeInfo zelda_machine_type = {
    .name = TYPE_ZELDA_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(ZeldaMachineState),
    .instance_init = zelda_machine_instance_init,
    .instance_finalize = zelda_machine_instance_finalize,
    .class_init = zelda_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void zelda_machine_register_types(void)
{
    type_register_static(&zelda_machine_type);
}
type_init(zelda_machine_register_types)
