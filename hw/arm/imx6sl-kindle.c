/*
 * Shared helpers for Lab126 i.MX6SoloLite Kindle boards.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "imx6sl-kindle.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/ssi/ssi.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "system/block-backend.h"

#define KINDLE_RAM_BASE          0x80000000
#define KINDLE_UBOOT_ADDR        0x00980000
#define KINDLE_UBOOT_MAX         0x00080000
#define KINDLE_IVT_OFFSET        0x400
#define KINDLE_IVT_ENTRY_OFF     0x04
#define KINDLE_IVT_SELF_OFF      0x14
#define KINDLE_IDME_BASE         0x5e000

#define KINDLE_ATAG_SERIAL16     0x5441000a
#define KINDLE_ATAG_REVISION16   0x5441000b
#define KINDLE_ATAG_MACADDR      0x5441000d
#define KINDLE_ATAG_BOOTMODE     0x5441000f
#define KINDLE_ATAG_ID16_SIZE    16
#define KINDLE_ATAG_MAC_SIZE     32
#define KINDLE_ATAG_BOOT_SIZE    32

typedef struct KindleIMX6SLIdmeField {
    const char *name;
    const char *value;
    uint32_t offset;
    size_t size;
} KindleIMX6SLIdmeField;

void kindle_imx6sl_populate_idme(DeviceState *card,
                                const KindleIMX6SLIdme *idme)
{
    const KindleIMX6SLIdmeField fields[] = {
        { "serial", idme->serial, 0x0000, 16 },
        { "mac", idme->mac, 0x0030, 12 },
        { "mfg", idme->mfg, 0x0040, 20 },
        { "pcbsn", idme->pcbsn, 0x0060, 16 },
        { "bootmode", idme->bootmode, 0x1000, 16 },
        { "postmode", idme->postmode, 0x1010, 16 },
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(fields); i++) {
        const KindleIMX6SLIdmeField *field = &fields[i];
        g_autofree uint8_t *contents = g_malloc0(field->size);
        size_t len = strlen(field->value);

        if (!g_str_is_ascii(field->value) || len > field->size) {
            error_report("i.MX6SL IDME %s must contain at most %zu ASCII bytes",
                         field->name, field->size);
            exit(EXIT_FAILURE);
        }
        memcpy(contents, field->value, len);
        emmc_boot_partition_write(card, 1, KINDLE_IDME_BASE + field->offset,
                                  contents, field->size, &error_fatal);
    }
}

static uint8_t *kindle_imx6sl_append_string_atag(uint8_t *p, uint32_t tag,
                                               const char *value,
                                               size_t payload_size)
{
    stl_le_p(p, (payload_size + 8) / 4);
    stl_le_p(p + 4, tag);
    memset(p + 8, 0, payload_size);
    memcpy(p + 8, value, MIN(strlen(value), payload_size));
    return p + 8 + payload_size;
}

int kindle_imx6sl_write_extra_atags(const struct arm_boot_info *info,
                                   void *opaque, void *buffer,
                                   size_t max_size)
{
    const KindleIMX6SLIdme *idme = opaque;
    uint8_t *start = buffer;
    uint8_t *p = start;
    const size_t required =
        2 * (8 + KINDLE_ATAG_ID16_SIZE) +
        (8 + KINDLE_ATAG_MAC_SIZE) + (8 + KINDLE_ATAG_BOOT_SIZE);

    (void)info;

    if (max_size < required) {
        return -1;
    }

    p = kindle_imx6sl_append_string_atag(p, KINDLE_ATAG_SERIAL16,
                                        idme->serial, KINDLE_ATAG_ID16_SIZE);
    p = kindle_imx6sl_append_string_atag(p, KINDLE_ATAG_REVISION16,
                                        idme->pcbsn, KINDLE_ATAG_ID16_SIZE);

    stl_le_p(p, (KINDLE_ATAG_MAC_SIZE + 8) / 4);
    stl_le_p(p + 4, KINDLE_ATAG_MACADDR);
    memset(p + 8, 0, KINDLE_ATAG_MAC_SIZE);
    memcpy(p + 8, idme->mac, MIN(strlen(idme->mac), (size_t)12));
    memcpy(p + 20, idme->mfg, MIN(strlen(idme->mfg), (size_t)20));
    p += 8 + KINDLE_ATAG_MAC_SIZE;

    stl_le_p(p, (KINDLE_ATAG_BOOT_SIZE + 8) / 4);
    stl_le_p(p + 4, KINDLE_ATAG_BOOTMODE);
    memset(p + 8, 0, KINDLE_ATAG_BOOT_SIZE);
    memcpy(p + 8, idme->bootmode, MIN(strlen(idme->bootmode), (size_t)16));
    memcpy(p + 24, idme->postmode, MIN(strlen(idme->postmode), (size_t)16));
    p += 8 + KINDLE_ATAG_BOOT_SIZE;

    return (int)(p - start);
}

hwaddr kindle_imx6sl_load_firmware(MachineState *machine)
{
    g_autofree uint8_t *image = NULL;
    gsize image_size;
    hwaddr addr = KINDLE_UBOOT_ADDR;
    hwaddr max_size = KINDLE_UBOOT_MAX;
    uint32_t entry;
    uint32_t self;
    ssize_t size;

    if (!machine->firmware) {
        return 0;
    }

    /*
     * The boot ROM consumes an IVT at offset 0x400 and jumps to its entry.
     * Stock U-Boot runs from OCRAM; barebox can load directly into SDRAM.
     * The IVT self pointer identifies the SDRAM image's load address.
     */
    if (!g_file_get_contents(machine->firmware, (char **)&image,
                             &image_size, NULL) ||
        image_size < KINDLE_IVT_OFFSET + KINDLE_IVT_SELF_OFF + sizeof(self)) {
        error_report("i.MX6SL firmware has no readable IVT");
        exit(EXIT_FAILURE);
    }
    entry = ldl_le_p(image + KINDLE_IVT_OFFSET + KINDLE_IVT_ENTRY_OFF);
    self = ldl_le_p(image + KINDLE_IVT_OFFSET + KINDLE_IVT_SELF_OFF);
    if (self >= KINDLE_RAM_BASE + KINDLE_IVT_OFFSET &&
        self < KINDLE_RAM_BASE + machine->ram_size) {
        addr = self - KINDLE_IVT_OFFSET;
        max_size = KINDLE_RAM_BASE + machine->ram_size - addr;
    }
    if (entry < addr || entry >= addr + image_size) {
        error_report("i.MX6SL firmware IVT entry 0x%08x is outside image",
                     entry);
        exit(EXIT_FAILURE);
    }

    size = load_image_targphys(machine->firmware, addr, max_size, NULL);
    if (size < 0) {
        error_report("Unable to load i.MX6SL firmware '%s'",
                     machine->firmware);
        exit(EXIT_FAILURE);
    }

    return entry;
}

static void kindle_imx6sl_panel_flash_program(SSIBus *bus, qemu_irq cs,
                                             uint32_t address,
                                             const uint8_t *data, size_t length)
{
    size_t i;

    qemu_set_irq(cs, 1);
    qemu_set_irq(cs, 0);
    ssi_transfer(bus, 0x06); /* write enable */
    qemu_set_irq(cs, 1);
    qemu_set_irq(cs, 0);
    ssi_transfer(bus, 0x02); /* page program */
    ssi_transfer(bus, address >> 16);
    ssi_transfer(bus, address >> 8);
    ssi_transfer(bus, address);
    for (i = 0; i < length; i++) {
        ssi_transfer(bus, data[i]);
    }
    qemu_set_irq(cs, 1);
}

void kindle_imx6sl_attach_panel_flash(FslIMX6State *s,
                                     const uint8_t *barcode)
{
    SSIBus *bus;
    DeviceState *flash;
    qemu_irq cs;
    static const uint8_t ac_format = 0x4b;

    /*
     * Keep a synthetic 4-Mbit panel NOR present so the
     * stock driver can read its JEDEC ID.  Waveforms come from the hidden
     * eMMC store; the synthetic NOR provides the AC layout and panel ID.
     */
    bus = (SSIBus *)qdev_get_child_bus(DEVICE(&s->spi[0]), "spi");
    flash = qdev_new("mx25l4005a");
    qdev_realize_and_unref(flash, BUS(bus), &error_fatal);
    cs = qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0);
    kindle_imx6sl_panel_flash_program(bus, cs, 0x899, &ac_format, 1);
    if (barcode) {
        kindle_imx6sl_panel_flash_program(bus, cs, 0x70050, barcode, 3);
    }
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 11, cs);
}
