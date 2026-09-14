/*
 * MT8171 connectivity digital register fabric and MT6631 identity SPI.
 *
 * Register contracts reconstructed from the stock wmt_drv.ko functions
 * polling_consys_chipid_mt8171_gen, consys_read_reg_from_dts,
 * consys_adie_chipid_checking_flow, consys_hw_reset_bit_set_mt8171_gen,
 * consys_conn2ap_sw_irq_clear and consys_bus_timeout_config_mt8171_gen.
 * The driver itself is absent from the released connectivity source tree.
 * See fw-work/scribe-colorsoft/consys/register-contract.md.
 *
 * ROM bootstrap is a protocol model, gated on the complete stock MCU
 * download and real power/reset/remap configuration. Firmware instruction
 * execution and Wi-Fi/radio command transports are not implemented.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/misc/mt8171-consys.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define R(s, a) ((s)->regs[(a) / 4])

static bool powered(MT8171ConsysState *s)
{
    return s->spm_regs && (s->spm_regs[0x304 / 4] & 0xc) == 0xc;
}

static bool rom_boot_enabled(MT8171ConsysState *s)
{
    return powered(s) && !(s->reset_control & BIT(12)) &&
           (R(s, 0x7024) & 1) && (R(s, 0xc1168) & 1) &&
           R(s, 0x2504) == 0xf0170000 && R(s, 0x2508) == 0xf02a0000 &&
           s->infracfg_regs && (s->infracfg_regs[0x380 / 4] & 0xfffff);
}

bool mt8171_consys_boot_ready(MT8171ConsysState *s)
{
    return s && rom_boot_enabled(s) && R(s, 0x2600) == 0x1d1e;
}

static void rom_boot(void *opaque)
{
    MT8171ConsysState *s = opaque;
    uint8_t buffer[4096];
    hwaddr base;

    if (!rom_boot_enabled(s) || !s->mcu_image) { return; }
    /* consys_emi_set_remapping_reg encodes PA[35:16] in low20 bits.
     * mtk_wcn_soc_rom_patch_dwn strips the 0x30-byte container header
     * and copies the MCU image to EMI offset zero before releasing reset.
     * Compare every byte: a short, stale or missing guest DMA download
     * cannot advance this protocol to the ROM idle mailbox state. */
    base = (uint64_t)(s->infracfg_regs[0x380 / 4] & 0xfffff) << 16;
    for (size_t offset = 0; offset < s->mcu_image_size - 0x30; ) {
        size_t length = MIN(sizeof(buffer), s->mcu_image_size - 0x30 - offset);
        if (address_space_read(&address_space_memory, base + offset,
                               MEMTXATTRS_UNSPECIFIED, buffer, length) != MEMTX_OK ||
            memcmp(buffer, s->mcu_image + 0x30 + offset, length)) {
            s->firmware_mismatches++;
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt8171-consys: MCU download mismatch at EMI +0x%zx\n",
                          offset);
            return;
        }
        offset += length;
    }
    s->firmware_bytes_verified += s->mcu_image_size - 0x30;
    /* Exact host protocol value, consys_polling_goto_idle_mt8171_gen.
     * This completes only ROM bootstrap, not an RF/calibration command. */
    R(s, 0x2600) = 0x1d1e;
    s->rom_boots++;
}

static void maybe_start_rom(MT8171ConsysState *s)
{
    if (!s->rom_attempted && s->mcu_image && rom_boot_enabled(s)) {
        s->rom_attempted = true;
        /* Nominal protocol latency; no instruction-clock claim. */
        timer_mod(s->rom_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
    }
}

bool mt8171_consys_calibration_memory(MT8171ConsysState *s, void *data,
                                     unsigned size, bool write)
{
    hwaddr address;
    if (!s || !rom_boot_enabled(s) || R(s, 0x2600) != 0x1d1e ||
        !size || size > 0x1000) {
        return false;
    }
    address = ((uint64_t)(s->infracfg_regs[0x380 / 4] & 0xfffff) << 16) +
              MT8171_SYNTH_CAL_OFFSET;
    return address_space_rw(&address_space_memory, address,
                            MEMTXATTRS_UNSPECIFIED, data, size, write) == MEMTX_OK;
}

static void update_irq(MT8171ConsysState *s)
{
    /* Status is fed only by the modeled connectivity-side input. */
    qemu_set_irq(s->irq[0], powered(s) && (R(s, 0x2150) & 0x0f000000));
}

static void invalidate_boot(MT8171ConsysState *s)
{
    timer_del(s->spi_timer);
    timer_del(s->rom_timer);
    s->rom_attempted = false;
    R(s, 0x2600) = 0;
    R(s, 0x7024) &= ~1U;
    R(s, 0xc6000) = 0;
    R(s, 0x2150) = 0;
    update_irq(s);
}

static void update_subsystem_reset(MT8171ConsysState *s)
{
    qemu_set_irq(s->subsystem_reset,
                 !powered(s) || (s->reset_control & BIT(12)));
}

/* The board drives this on CONN_PWR_CON rail changes. Unlike a readiness
 * check at the next MMIO access, this immediately cancels pending work and
 * interrupts when the domain loses power, even if no CPU accesses it. */
static void power_input(void *opaque, int input, int level)
{
    MT8171ConsysState *s = opaque;
    if (!level) {
        invalidate_boot(s);
    }
    update_subsystem_reset(s);
}

static void mailbox_input(void *opaque, int input, int level)
{
    MT8171ConsysState *s = opaque;
    if (level && powered(s) && !(s->reset_control & BIT(12))) {
        R(s, 0x2150) |= BIT(24 + input);
    }
    update_irq(s);
}

static void spi_done(void *opaque)
{
    MT8171ConsysState *s = opaque;
    if (!powered(s) || !(R(s, 0xc1500) & 1)) { return; }
    /* Detection selects TOP SPI read0xb024 then writes dummy data at+54.
     * Return the real chip family identified by all stock firmware names.
     * This is a register read transaction, not successful RF initialization. */
    if (R(s, 0xc6050) == 0xb024) {
        R(s, 0xc6058) = 0x66310000;
        R(s, 0xc6000) &= ~(BIT(0) | BIT(5));
        s->spi_transactions++;
    } else {
        s->unsupported_spi++;
        qemu_log_mask(LOG_UNIMP,
                      "mt8171-consys: unsupported RF SPI command %08x data=%08x\n",
                      R(s, 0xc6050), R(s, 0xc6054));
    }
}

static bool identity_register(hwaddr address)
{
    return address == 0x2000 || address == 0x2004 ||
           address == 0xb1010 || address == 0xb101c;
}

bool mt8171_consys_mcu_adie(MT8171ConsysState *s, uint32_t *identity)
{
    if (!rom_boot_enabled(s) || R(s, 0x2600) != 0x1d1e) {
        return false;
    }
    /* Same soldered MT6631 as TOP SPI's ID register. The firmware's query
     * establishes identity/presence, not successful RF calibration. */
    *identity = 0x6631;
    return true;
}

static uint64_t read_reg(void *opaque, hwaddr address, unsigned size)
{
    MT8171ConsysState *s = opaque;
    uint32_t value;
    if ((address & ~3) == 0x2600) {
        if (!powered(s)) { R(s, 0x2600) = 0; }
        maybe_start_rom(s);
    }
    if (identity_register(address & ~3)) {
        s->identity_reads++;
        value = powered(s) ? R(s, address & ~3) : 0;
    } else {
        value = R(s, address & ~3);
    }
    return value >> ((address & 3) * 8);
}

static void write_reg(void *opaque, hwaddr address, uint64_t value, unsigned size)
{
    MT8171ConsysState *s = opaque;
    unsigned shift = (address & 3) * 8;
    uint32_t mask = size == 4 ? UINT32_MAX : (1U << (size * 8)) - 1;
    uint32_t written = (value & mask) << shift;
    address &= ~3;
    if (identity_register(address) || address == 0x2150 || address == 0x2600 ||
        address == 0xc6000 || address == 0xc6058) { return; }
    R(s, address) = (R(s, address) & ~(mask << shift)) | written;
    switch (address) {
    case 0x214c: /* CONN2AP software interrupt W1C, verified by read+150. */
        R(s, 0x2150) &= ~(written & 0x0f000000);
        R(s, address) = 0;
        update_irq(s);
        break;
    case 0xc6054: /* TOP RF SPI dummy write starts addressed read. */
        if (powered(s) && (R(s, 0xc1500) & 1)) {
            R(s, 0xc6000) |= BIT(0) | BIT(5);
            timer_mod(s->spi_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
        }
        break;
    case 0x2504: /* MCU EMI entry */
    case 0x2508: /* BT EMI entry */
    case 0x7024: /* CPU reset release companion */
    case 0xc1168: /* EMI hardware access */
        maybe_start_rom(s);
        break;
    }
}

bool mt8171_consys_mcu_reg(MT8171ConsysState *s, uint32_t address,
                          uint32_t *value, uint32_t mask, bool write)
{
    uint32_t offset;
    if (!rom_boot_enabled(s) || R(s, 0x2600) != 0x1d1e || (address & 3)) {
        return false;
    }
    /* The WMT protocol's identity view is used by wmt_core_stp_init and
     * wmt_ic_ops_soc: HW version, FW version, chip identity. This is a
     * protocol view, not a claim that the MCU physical bus aliases AP MMIO. */
    if (address >= 0x80000000 && address <= 0x80000008) {
        if (write) { return false; }
        *value = address == 0x80000008 ? 0x8171 :
                 R(s, 0x2000 + address - 0x80000000);
        *value &= mask;
        return true;
    }
    if (address < 0x18000000 || address >= 0x18100000) { return false; }
    offset = address - 0x18000000;
    if (identity_register(offset)) {
        if (write) { return false; }
        *value = read_reg(s, offset, 4) & mask;
        return true;
    }
    /* Only already-modeled digital controls may be accessed. Do not
     * acknowledge RF/calibration addresses using generic register RAM. */
    switch (offset) {
    case 0x2440: /* bus timeout */
    case 0xc0284:
    case 0xc1168: /* EMI access mode */
    case 0xc1500: /* identity SPI gate */
        if (write) {
            write_reg(s, offset, (R(s, offset) & ~mask) | (*value & mask), 4);
        } else {
            *value = read_reg(s, offset, 4) & mask;
        }
        return true;
    default:
        return false;
    }
}

static uint64_t reset_read(void *opaque, hwaddr address, unsigned size)
{
    return MT8171_CONSYS(opaque)->reset_control;
}

static void reset_write(void *opaque, hwaddr address, uint64_t value, unsigned size)
{
    MT8171ConsysState *s = opaque;
    /* RGU WDT_SWSYSRST requires key0x88 in its high byte. Only CONSYS
     * bit12 is owned here; the board retains all unrelated WDT registers. */
    if ((value >> 24) != 0x88) { return; }
    bool was_reset = s->reset_control & BIT(12);
    s->reset_control = value & 0xffffff;
    if ((value & BIT(12)) && !was_reset) {
        s->reset_assertions++;
        invalidate_boot(s);
    }
    update_subsystem_reset(s);
    if (!(value & BIT(12))) { maybe_start_rom(s); }
}

static const MemoryRegionOps ops = {
    .read = read_reg, .write = write_reg,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};
static const MemoryRegionOps reset_ops = {
    .read = reset_read, .write = reset_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void reset(DeviceState *dev)
{
    MT8171ConsysState *s = MT8171_CONSYS(dev);
    timer_del(s->spi_timer);
    timer_del(s->rom_timer);
    s->rom_attempted = false;
    memset(s->regs, 0, sizeof(s->regs));
    R(s, 0x2000) = R(s, 0x2004) = 0x8a00;
    R(s, 0xb1010) = 0x100e0100;
    R(s, 0xb101c) = 1;
    s->reset_control = BIT(12);
    s->identity_reads = s->spi_transactions = s->unsupported_spi = 0;
    s->reset_assertions = 0;
    s->rom_boots = s->firmware_mismatches = s->firmware_bytes_verified = 0;
    for (unsigned i = 0; i < 3; i++) { qemu_set_irq(s->irq[i], 0); }
    update_subsystem_reset(s);
}

static void init(Object *obj)
{
    MT8171ConsysState *s = MT8171_CONSYS(obj);
    s->spi_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, spi_done, s);
    s->rom_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rom_boot, s);
    memory_region_init_io(&s->iomem, obj, &ops, s,
                         "mt8171-consys", 0x100000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    memory_region_init_io(&s->reset_iomem, obj, &reset_ops, s,
                         "mt8171-consys-rgu-reset", 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->reset_iomem);
    for (unsigned i = 0; i < 3; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    qdev_init_gpio_in_named(DEVICE(obj), mailbox_input, "mailbox", 4);
    qdev_init_gpio_in_named(DEVICE(obj), power_input, "power", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->subsystem_reset, "reset", 1);
    object_property_add_uint64_ptr(obj, "identity-reads", &s->identity_reads, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "spi-transactions", &s->spi_transactions, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "unsupported-spi", &s->unsupported_spi, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "reset-assertions", &s->reset_assertions, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "rom-boots", &s->rom_boots, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "firmware-mismatches", &s->firmware_mismatches, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "firmware-bytes-verified", &s->firmware_bytes_verified, OBJ_PROP_FLAG_READ);
}
static void realize(DeviceState *dev, Error **errp)
{
    MT8171ConsysState *s = MT8171_CONSYS(dev);
    g_autoptr(GError) error = NULL;

    if (!s->mcu_firmware) { return; }
    if (!g_file_get_contents(s->mcu_firmware, (char **)&s->mcu_image,
                             &s->mcu_image_size, &error)) {
        error_setg(errp, "mt8171-consys: cannot read MCU firmware: %s", error->message);
        return;
    }
    if (s->mcu_image_size <= 0x30 || s->mcu_image_size > 0x170000 ||
        memcmp(s->mcu_image + 16, "ALPS", 4) ||
        memcmp(s->mcu_image + 20, "\x8a\x00\x8a\x00", 4)) {
        error_setg(errp, "mt8171-consys: expected MT8171 MCU image with 48-byte ALPS header");
    }
}
static void finalize(Object *obj)
{
    MT8171ConsysState *s = MT8171_CONSYS(obj);
    timer_free(s->spi_timer);
    timer_free(s->rom_timer);
    g_free(s->mcu_image);
}
static const Property properties[] = {
    DEFINE_PROP_STRING("mcu-firmware", MT8171ConsysState, mcu_firmware),
};
static void class_init(ObjectClass *oc, const void *data)
{
    DEVICE_CLASS(oc)->realize = realize;
    device_class_set_props(DEVICE_CLASS(oc), properties);
    device_class_set_legacy_reset(DEVICE_CLASS(oc), reset);
}
static const TypeInfo info = {
    .name = TYPE_MT8171_CONSYS, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8171ConsysState), .instance_init = init,
    .instance_finalize = finalize, .class_init = class_init,
};
static void register_types(void) { type_register_static(&info); }
type_init(register_types)
