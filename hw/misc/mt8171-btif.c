/*
 * MT8171 BTIF PIO and APDMA virtual FIFOs.
 * Register/IRQ contract: device_module/drivers/misc/mediatek/btif/common/
 * btif_plat.c, btif_dma_plat.c and plat_inc/{btif,btif_dma}_priv.h.
 * The bounded protocol companion handles digital bootstrap commands.
 * Unimplemented MCU commands are retained without a success response.
 * The hardware loopback bit routes actual bytes to RX for transport use.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/misc/mt8171-btif.h"
#include "hw/core/irq.h"
#include "system/address-spaces.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define B(s, b, a) ((s)->bank[b].regs[(a) / 4])

static bool running(MT8171BtifState *s)
{
    return s->consys && s->consys->spm_regs &&
           (s->consys->spm_regs[0x304 / 4] & 0xc) == 0xc &&
           !(s->consys->reset_control & (1 << 12)) &&
           s->consys->regs[0x2600 / 4] == 0x1d1e &&
           !(B(s, 0, 0x48) & 1);
}

static void sync_rom_boot(MT8171BtifState *s)
{
    if (s->consys && s->last_rom_boot != s->consys->rom_boots) {
        mt8171_btif_protocol_reset(s);
        s->last_rom_boot = s->consys->rom_boots;
    }
}

static unsigned ring_used(MT8171BtifState *s, unsigned bank)
{
    uint32_t len = B(s, bank, 0x24), wp = B(s, bank, 0x2c),
             rp = B(s, bank, 0x30);
    if (!len || len > 0xfff8 || (len & 7) ||
        (wp & 0xffff) >= len || (rp & 0xffff) >= len) {
        return UINT_MAX;
    }
    int distance = (wp & 0xffff) - (rp & 0xffff);
    if ((wp ^ rp) & 0x10000) { distance += len; }
    return distance < 0 || distance > len ? UINT_MAX : distance;
}

static uint32_t advance(uint32_t pointer, unsigned length, unsigned count)
{
    unsigned offset = (pointer & 0xffff) + count;
    if (offset >= length) { offset -= length; pointer ^= 0x10000; }
    return (pointer & 0x10000) | offset;
}

static bool accept_byte(MT8171BtifState *s, uint8_t byte)
{
    if (B(s, 0, 0x60) & 0x80) {
        if (s->loopback_used == sizeof(s->loopback)) { return false; }
        s->loopback[(s->loopback_head + s->loopback_used++) % sizeof(s->loopback)] = byte;
    } else {
        if (s->ingress_used == sizeof(s->ingress)) { return false; }
        s->ingress[s->ingress_used++] = byte;
    }
    s->tx_bytes++;
    return true;
}

static void update_irqs(MT8171BtifState *s)
{
    bool rx = s->loopback_used && !(B(s, 0, 0x4c) & 1);
    bool room = (B(s, 0, 0x60) & 0x80) ?
        s->loopback_used < sizeof(s->loopback) :
        s->ingress_used < sizeof(s->ingress);
    uint32_t interrupt = rx && (B(s, 0, 4) & 1) ? 4 :
                         room && (B(s, 0, 4) & 2) ? 2 : 1;
    B(s, 0, 8) = interrupt;
    B(s, 0, 0x14) = (rx ? 1 : 0) | (room ? 0x60 : 0);
    qemu_set_irq(s->irq[0], running(s) && interrupt != 1);
    for (unsigned i = 1; i < 3; i++) {
        unsigned used = ring_used(s, i);
        B(s, i, 0x3c) = used == UINT_MAX ? 0 : used;
        B(s, i, 0x40) = used == UINT_MAX ? 0 : B(s, i, 0x24) - used;
        qemu_set_irq(s->irq[i], running(s) && (B(s, i, 0) & B(s, i, 4)));
    }
}

static void transfer(MT8171BtifState *s)
{
    sync_rom_boot(s);
    if (!running(s)) { update_irqs(s); return; }
    mt8171_btif_protocol_hardware_wake(s);
    if ((B(s, 0, 0x4c) & 2) && (B(s, 1, 8) & 1)) {
        unsigned count = ring_used(s, 1);
        uint64_t previous_tx = s->tx_bytes;
        while (count != UINT_MAX && count--) {
            uint32_t rp = B(s, 1, 0x30);
            hwaddr base = ((uint64_t)B(s, 1, 0x54) << 32) | B(s, 1, 0x1c);
            uint8_t byte;
            if (address_space_read(&address_space_memory, base + (rp & 0xffff),
                                   MEMTXATTRS_UNSPECIFIED, &byte, 1) != MEMTX_OK) {
                s->dma_faults++;
                break;
            }
            if (!accept_byte(s, byte)) { break; }
            B(s, 1, 0x30) = advance(rp, B(s, 1, 0x24), 1);
        }
        unsigned left = ring_used(s, 1);
        if (previous_tx != s->tx_bytes && left != UINT_MAX &&
            left < B(s, 1, 0x28)) { B(s, 1, 0) |= 1; }
        if (!left) { B(s, 1, 0x14) = 0; }
    }
    if (!(B(s, 0, 0x60) & 0x80)) { mt8171_btif_protocol_process(s); }
    if ((B(s, 0, 0x4c) & 1) && (B(s, 2, 8) & 1)) {
        unsigned used = ring_used(s, 2);
        while (used != UINT_MAX && used < B(s, 2, 0x24) && s->loopback_used) {
            uint32_t wp = B(s, 2, 0x2c);
            hwaddr base = ((uint64_t)B(s, 2, 0x54) << 32) | B(s, 2, 0x1c);
            if (address_space_write(&address_space_memory, base + (wp & 0xffff),
                                    MEMTXATTRS_UNSPECIFIED,
                                    &s->loopback[s->loopback_head], 1) != MEMTX_OK) {
                s->dma_faults++;
                break;
            }
            s->loopback_head = (s->loopback_head + 1) % sizeof(s->loopback);
            s->loopback_used--;
            s->rx_bytes++;
            B(s, 2, 0x2c) = advance(wp, B(s, 2, 0x24), 1);
            used++;
            B(s, 2, 0) |= 2;
        }
        if (used != UINT_MAX && used && used >= B(s, 2, 0x28)) {
            B(s, 2, 0) |= 1;
        }
    }
    update_irqs(s);
}

static uint64_t read_reg(void *opaque, hwaddr address, unsigned size)
{
    MT8171BtifBank *bank = opaque;
    MT8171BtifState *s = bank->owner;
    sync_rom_boot(s);
    update_irqs(s);
    if (!bank->index && address == 0) {
        if (!s->loopback_used) { return 0; }
        uint8_t byte = s->loopback[s->loopback_head];
        s->loopback_head = (s->loopback_head + 1) % sizeof(s->loopback);
        s->loopback_used--;
        s->rx_bytes++;
        transfer(s);
        return byte;
    }
    return bank->regs[address / 4] >> ((address & 3) * 8);
}

static void write_reg(void *opaque, hwaddr address, uint64_t value, unsigned size)
{
    MT8171BtifBank *bank = opaque;
    MT8171BtifState *s = bank->owner;
    sync_rom_boot(s);
    unsigned i = bank->index;
    uint32_t old = bank->regs[address / 4];
    unsigned shift = (address & 3) * 8;
    uint32_t mask = size == 4 ? UINT32_MAX : (1U << (size * 8)) - 1;
    value = (old & ~(mask << shift)) | ((value & mask) << shift);
    address &= ~3;
    if (!i) {
        if (address == 0) {
            if (running(s)) { accept_byte(s, value); }
        } else if (address == 8) {
            if (value & 2) { s->loopback_head = s->loopback_used = 0; }
            /* Clearing the AP TX FIFO does not erase MCU ingress. */
        } else if (address == 0x64) {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            /* hal_btif_raise_wak_sig: low for at least one 32k period, then
             * high. The stock driver sleeps128..160us between these writes.
             * Wake only a sleeping, bootstrapped MCU; reset-time level writes
             * must not inject an unsolicited packet into STP initialization. */
            if (!(value & 1)) {
                s->wake_low_ns = now;
                s->wake_low_armed = true;
            } else if (!(old & 1) && s->wake_low_armed) {
                if (now - s->wake_low_ns >= 31250 &&
                    !s->firmware_awake && s->mcu_adie_id == 0x6631 &&
                    s->stp_active) {
                    s->hardware_wake_pending = true;
                }
                s->wake_low_armed = false;
            }
            bank->regs[address / 4] = value;
        } else if (address != 0x14) {
            bank->regs[address / 4] = value;
        }
    } else if (address == 0) {
        /* TX flag is cleared by zero; RX flags are write-one-to-clear. */
        bank->regs[0] = i == 1 ? old & value : old & ~value;
    } else if (address == 0xc && (value & 3)) {
        B(s, i, 8) = B(s, i, 0xc) = B(s, i, 0x10) = B(s, i, 0x14) = 0;
        B(s, i, 0) = B(s, i, 0x2c) = B(s, i, 0x30) = 0;
    } else if (address == 0x10 && (value & 1)) {
        B(s, i, 8) = B(s, i, 0x10) = 0;
    } else if (address != 0x3c && address != 0x40) {
        bank->regs[address / 4] = value;
    }
    transfer(s);
}

static const MemoryRegionOps ops = {
    .read = read_reg, .write = write_reg, .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void reset(DeviceState *dev)
{
    MT8171BtifState *s = MT8171_BTIF(dev);
    for (unsigned i = 0; i < 3; i++) {
        memset(s->bank[i].regs, 0, sizeof(s->bank[i].regs));
        qemu_set_irq(s->irq[i], 0);
    }
    s->ingress_used = s->loopback_head = s->loopback_used = 0;
    s->tx_bytes = s->rx_bytes = s->dma_faults = 0;
    mt8171_btif_protocol_reset(s);
    s->protocol_commands = s->protocol_rejections = s->protocol_resets = 0;
    s->stp_replays = s->stp_acks = 0;
    s->chip_id_sets = s->adie_queries = s->configuration_writes = 0;
    s->calibration_runs = s->calibration_restores = s->calibration_backups = 0;
    s->utc_syncs = s->blank_updates = s->sleep_commands = s->wake_commands = 0;
    s->function_control_requests = s->unavailable_function_requests = 0;
    s->hardware_wake_events = 0;
    s->last_rom_boot = 0;
    update_irqs(s);
}

static char *get_ingress(Object *obj, Error **errp)
{
    MT8171BtifState *s = MT8171_BTIF(obj);
    GString *text = g_string_new(NULL);
    for (unsigned i = 0; i < MIN(s->ingress_used, 512U); i++) {
        g_string_append_printf(text, "%02x", s->ingress[i]);
    }
    return g_string_free(text, false);
}

static char *get_last_request(Object *obj, Error **errp)
{
    MT8171BtifState *s = MT8171_BTIF(obj);
    GString *text = g_string_new(NULL);
    for (unsigned i = 0; i < s->last_request_size; i++) {
        g_string_append_printf(text, "%02x", s->last_request[i]);
    }
    return g_string_free(text, false);
}

static char *get_mcu_configuration(Object *obj, Error **errp)
{
    MT8171BtifState *s = MT8171_BTIF(obj);
    GString *text = g_string_new(NULL);
    g_string_append_printf(text, "chip=%04x adie=%04x",
                           s->mcu_chip_id, s->mcu_adie_id);
    for (unsigned i = 0; i < ARRAY_SIZE(s->mcu_config); i++) {
        MT8171McuConfig *entry = &s->mcu_config[i];
        if (!entry->opcode) { continue; }
        g_string_append_printf(text, " %02x/%02x=", entry->opcode,
                               entry->selector);
        for (unsigned n = 0; n < entry->length; n++) {
            g_string_append_printf(text, "%02x", entry->data[n]);
        }
    }
    return g_string_free(text, false);
}

static char *get_calibration(Object *obj, Error **errp)
{
    MT8171BtifState *s = MT8171_BTIF(obj);
    GString *text = g_string_new(NULL);
    g_string_append_printf(text, "synthetic valid=%u thermal-enabled=%u emi-offset=%x bt=",
                           s->calibration_valid, s->thermal_enabled,
                           MT8171_SYNTH_CAL_OFFSET);
    for (unsigned i = 0; i < sizeof(s->calibration_bt); i++) {
        g_string_append_printf(text, "%02x", s->calibration_bt[i]);
    }
    g_string_append(text, " wifi=");
    for (unsigned i = 0; i < sizeof(s->calibration_wifi); i++) {
        g_string_append_printf(text, "%02x", s->calibration_wifi[i]);
    }
    return g_string_free(text, false);
}

static char *get_runtime_state(Object *obj, Error **errp)
{
    MT8171BtifState *s = MT8171_BTIF(obj);
    uint64_t utc_us = (uint64_t)s->utc_seconds * 1000000 + s->utc_microseconds;
    if (s->utc_valid) {
        utc_us += MAX(INT64_C(0), qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                                 s->utc_sync_ns) / 1000;
    }
    return g_strdup_printf("awake=%u blank=%u utc-valid=%u utc=%" PRIu64
                           ".%06" PRIu64,
                           s->firmware_awake, s->display_blank, s->utc_valid,
                           utc_us / 1000000, utc_us % 1000000);
}

static void init(Object *obj)
{
    MT8171BtifState *s = MT8171_BTIF(obj);
    for (unsigned i = 0; i < 3; i++) {
        s->bank[i].owner = s;
        s->bank[i].index = i;
        memory_region_init_io(&s->bank[i].iomem, obj, &ops, &s->bank[i],
                              i ? "mt8171-btif-dma" : "mt8171-btif-pio",
                              i ? 0x80 : 0x1000);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->bank[i].iomem);
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    object_property_add_uint64_ptr(obj, "tx-bytes", &s->tx_bytes, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "rx-bytes", &s->rx_bytes, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "dma-faults", &s->dma_faults, OBJ_PROP_FLAG_READ);
    object_property_add_str(obj, "mcu-ingress-prefix", get_ingress, NULL);
    object_property_add_str(obj, "last-wmt-request", get_last_request, NULL);
    object_property_add_uint64_ptr(obj, "protocol-commands", &s->protocol_commands, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "protocol-rejections", &s->protocol_rejections, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "protocol-resets", &s->protocol_resets, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "stp-replays", &s->stp_replays, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "stp-acks", &s->stp_acks, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "chip-id-sets", &s->chip_id_sets, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "adie-queries", &s->adie_queries, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "configuration-writes", &s->configuration_writes, OBJ_PROP_FLAG_READ);
    object_property_add_str(obj, "mcu-configuration", get_mcu_configuration, NULL);
    object_property_add_str(obj, "synthetic-calibration", get_calibration, NULL);
    object_property_add_uint64_ptr(obj, "synthetic-calibration-runs", &s->calibration_runs, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "calibration-restores", &s->calibration_restores, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "calibration-backups", &s->calibration_backups, OBJ_PROP_FLAG_READ);
    object_property_add_str(obj, "mcu-runtime-state", get_runtime_state, NULL);
    object_property_add_uint64_ptr(obj, "utc-syncs", &s->utc_syncs, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "blank-updates", &s->blank_updates, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "sleep-commands", &s->sleep_commands, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "wake-commands", &s->wake_commands, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "hardware-wake-events", &s->hardware_wake_events, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "function-control-requests", &s->function_control_requests, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "unavailable-function-requests", &s->unavailable_function_requests, OBJ_PROP_FLAG_READ);
}
static void class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), reset);
}
static const TypeInfo info = {
    .name = TYPE_MT8171_BTIF, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8171BtifState), .instance_init = init,
    .class_init = class_init,
};
static void register_types(void) { type_register_static(&info); }
type_init(register_types)
