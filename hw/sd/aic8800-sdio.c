/*
 * AIC8800 SDIO firmware interface and an open emulated access point.
 *
 * The two native SDIO functions serve the BSP downloader and FullMAC driver.
 * Implement their register/FIFO transport, debug-memory download protocol and
 * LMAC messages. Ethernet traffic crosses QEMU's NIC backend after the native
 * driver has scanned and associated with the emulated access point.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/sd/sd.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qemu/timer.h"
#include "sdmmc-internal.h"
#include "trace.h"

#define TYPE_AIC8800_SDIO "aic8800-sdio"
OBJECT_DECLARE_SIMPLE_TYPE(Aic8800State, AIC8800_SDIO)
#define AIC_MAX_IO 65536
#define AIC_MAX_MESSAGE 4096
#define AIC_MAX_QUEUE 128
#define AIC_SSID "PocketBook-QEMU"
#define AIC_MEMORY_PAGE 4096
#define AIC_TX_DESCRIPTOR_SIZE 28

static const uint8_t aic_bssid[6] = { 2, 0, 0, 0, 0, 4 };

typedef struct AicMessage {
    unsigned length;
    uint8_t data[AIC_MAX_MESSAGE];
} AicMessage;

struct Aic8800State {
    DeviceState parent_obj;
    NICConf conf;
    NICState *nic;
    qemu_irq irq;
    uint8_t cccr[0x300], cis[0x300], regs[3][0x100];
    uint8_t io[AIC_MAX_IO], station[6];
    unsigned address, position, length, function;
    unsigned vif, used_vifs, sequence;
    unsigned scan_function, scan_vif, connect_function, connect_vif;
    bool powered, writing, increment, firmware, associated;
    QEMUTimer *scan_timer, *connect_timer;
    GQueue rx[3];
    GHashTable *memory;
};

static void aic_update_irq(Aic8800State *s)
{
    unsigned pending = 0;
    for (unsigned fn = 1; fn <= 2; fn++) {
        if (!g_queue_is_empty(&s->rx[fn])) {
            pending |= BIT(fn);
        }
    }
    s->cccr[5] = pending;
    qemu_set_irq(s->irq, s->powered && (s->cccr[4] & 1) &&
                 (pending & s->cccr[4]));
}

static void aic_enqueue(Aic8800State *s, unsigned fn, const uint8_t *data,
                         unsigned length)
{
    AicMessage *message;
    if (!s->powered || fn < 1 || fn > 2 || length > AIC_MAX_MESSAGE ||
        g_queue_get_length(&s->rx[fn]) >= AIC_MAX_QUEUE) {
        return;
    }
    message = g_new0(AicMessage, 1);
    message->length = length;
    memcpy(message->data, data, length);
    g_queue_push_tail(&s->rx[fn], message);
    aic_update_irq(s);
}

static void aic_message(Aic8800State *s, unsigned fn, unsigned id,
                         const void *payload, unsigned length)
{
    uint8_t data[AIC_MAX_MESSAGE] = { 0 };
    if (length > sizeof(data) - 16) {
        return;
    }
    stw_le_p(data, 12 + length);
    data[2] = 0x11;
    stw_le_p(data + 4, id);
    stw_le_p(data + 6, 100); /* host task */
    stw_le_p(data + 8, id >> 10);
    stw_le_p(data + 10, length);
    stl_le_p(data + 12, 0xaddede2a);
    if (length) {
        memcpy(data + 16, payload, length);
    }
    trace_aic8800_message(false, fn, id, length);
    aic_enqueue(s, fn, data, ROUND_UP(16 + length, 4));
}

static void aic_memory_write(Aic8800State *s, uint32_t address,
                              const uint8_t *data, unsigned length)
{
    while (length) {
        uint32_t page = address & ~(AIC_MEMORY_PAGE - 1);
        unsigned offset = address & (AIC_MEMORY_PAGE - 1);
        unsigned count = MIN(length, AIC_MEMORY_PAGE - offset);
        uint8_t *bytes = g_hash_table_lookup(s->memory, GUINT_TO_POINTER(page));
        if (!bytes) {
            bytes = g_malloc0(AIC_MEMORY_PAGE);
            g_hash_table_insert(s->memory, GUINT_TO_POINTER(page), bytes);
        }
        memcpy(bytes + offset, data, count);
        data += count;
        address += count;
        length -= count;
    }
}

static uint32_t aic_memory_read(Aic8800State *s, uint32_t address)
{
    uint8_t data[4] = { 0 };
    if (address == 0x40500000) {
        return 7 << 16; /* AIC8801 U03 silicon revision */
    }
    for (unsigned i = 0; i < sizeof(data); i++) {
        uint8_t *page = g_hash_table_lookup(s->memory,
                         GUINT_TO_POINTER((address + i) & ~(AIC_MEMORY_PAGE - 1)));
        if (page) {
            data[i] = page[(address + i) & (AIC_MEMORY_PAGE - 1)];
        }
    }
    return ldl_le_p(data);
}

static unsigned aic_beacon(Aic8800State *s, uint8_t *frame)
{
    static const uint8_t broadcast[6] = { 255, 255, 255, 255, 255, 255 };
    static const uint8_t rates[] = {
        1, 8, 0x82, 0x84, 0x8b, 0x96, 12, 18, 24, 36,
        3, 1, 1, 50, 4, 48, 72, 96, 108, 5, 4, 0, 1, 0, 0,
    };
    unsigned pos = 36;
    memset(frame, 0, 128);
    stw_le_p(frame, 0x80);
    memcpy(frame + 4, broadcast, 6);
    memcpy(frame + 10, aic_bssid, 6);
    memcpy(frame + 16, aic_bssid, 6);
    stw_le_p(frame + 22, s->sequence++ << 4);
    stw_le_p(frame + 32, 100);
    stw_le_p(frame + 34, 0x421); /* open infrastructure BSS */
    frame[pos++] = 0;
    frame[pos++] = sizeof(AIC_SSID) - 1;
    memcpy(frame + pos, AIC_SSID, sizeof(AIC_SSID) - 1);
    pos += sizeof(AIC_SSID) - 1;
    memcpy(frame + pos, rates, sizeof(rates));
    return pos + sizeof(rates);
}

static void aic_scan_done(void *opaque)
{
    Aic8800State *s = opaque;
    unsigned fn = s->scan_function, vif = s->scan_vif;
    uint8_t result[160] = { 0 }, confirm[3] = { vif, 0, 0 };
    if (!qemu_get_queue(s->nic)->link_down) {
        unsigned length = aic_beacon(s, result + 12);
        stw_le_p(result, length);
        stw_le_p(result + 2, 0x80);
        stw_le_p(result + 4, 2412);
        result[7] = 0xff;
        result[8] = vif;
        result[9] = (uint8_t)-40;
        aic_message(s, fn, 0x1004, result, length + 12);
        confirm[2] = 1;
    }
    aic_message(s, fn, 0x1001, confirm, sizeof(confirm));
}

static void aic_scan(Aic8800State *s, unsigned fn, unsigned vif)
{
    s->scan_function = fn;
    s->scan_vif = vif;
    aic_message(s, fn, 0x1009, NULL, 0);
    /* Channel scanning completes after the command acknowledgment. */
    timer_mod(s->scan_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100 * SCALE_MS);
}

static void aic_connect_done(void *opaque)
{
    Aic8800State *s = opaque;
    unsigned fn = s->connect_function, vif = s->connect_vif;
    uint8_t indication[852] = { 0 };

    stw_le_p(indication, qemu_get_queue(s->nic)->link_down ? 1 : 0);
    memcpy(indication + 2, aic_bssid, 6);
    indication[9] = vif;
    indication[10] = 0; /* AP station */
    indication[11] = 0; /* channel context */
    stw_le_p(indication + 820, 1); /* AID */
    indication[822] = 0; /* 2.4 GHz */
    stw_le_p(indication + 824, 2412);
    stl_le_p(indication + 828, 2412);
    stl_le_p(indication + 836, 0x0000a403);
    stl_le_p(indication + 840, 0x0000a427);
    stl_le_p(indication + 844, 0x005e4322);
    stl_le_p(indication + 848, 0x002f3222);
    s->vif = vif;
    s->associated = !qemu_get_queue(s->nic)->link_down;
    aic_message(s, fn, 0x1802, indication, sizeof(indication));
    qemu_flush_queued_packets(qemu_get_queue(s->nic));
}

static void aic_connect(Aic8800State *s, unsigned fn, unsigned vif)
{
    uint8_t confirm = 0;

    s->connect_function = fn;
    s->connect_vif = vif;
    aic_message(s, fn, 0x1801, &confirm, sizeof(confirm));
    timer_mod(s->connect_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20 * SCALE_MS);
}

static void aic_disconnect(Aic8800State *s, unsigned reason)
{
    uint8_t data[6] = { 0 };
    timer_del(s->connect_timer);
    stw_le_p(data, reason);
    data[2] = s->vif;
    s->associated = false;
    aic_message(s, 1, 0x1805, data, sizeof(data));
}

static void aic_command_message(Aic8800State *s, unsigned fn,
                                 const uint8_t *message, unsigned length)
{
    uint8_t reply[1024] = { 0 };
    unsigned id, parameter_length, confirm, reply_length = 0;
    const uint8_t *data;
    if (length < 8) {
        return;
    }
    id = lduw_le_p(message);
    parameter_length = lduw_le_p(message + 6);
    if (parameter_length > length - 8) {
        return;
    }
    data = message + 8;
    confirm = id + 1;
    trace_aic8800_message(true, fn, id, parameter_length);

    switch (id) {
    case 0x400: /* DBG_MEM_READ_REQ */
        if (parameter_length < 4) {
            return;
        }
        memcpy(reply, data, 4);
        stl_le_p(reply + 4, aic_memory_read(s, ldl_le_p(data)));
        reply_length = 8;
        break;
    case 0x402: /* DBG_MEM_WRITE_REQ */
        if (parameter_length < 8) {
            return;
        }
        aic_memory_write(s, ldl_le_p(data), data + 4, 4);
        memcpy(reply, data, 8);
        reply_length = 8;
        break;
    case 0x40b: /* DBG_MEM_BLOCK_WRITE_REQ */
        if (parameter_length < 8 || ldl_le_p(data + 4) > parameter_length - 8) {
            return;
        }
        aic_memory_write(s, ldl_le_p(data), data + 8, ldl_le_p(data + 4));
        reply_length = 4; /* dbg_mem_block_write_cfm.wstatus. */
        break;
    case 0x411: /* DBG_MEM_MASK_WRITE_REQ */
        if (parameter_length < 12) {
            return;
        }
        memcpy(reply, data, 4);
        stl_le_p(reply + 4, (aic_memory_read(s, ldl_le_p(data)) &
                             ~ldl_le_p(data + 4)) |
                             (ldl_le_p(data + 8) & ldl_le_p(data + 4)));
        aic_memory_write(s, ldl_le_p(data), reply + 4, 4);
        reply_length = 8;
        break;
    case 0x40d: /* DBG_START_APP_REQ */
        s->firmware = true;
        reply_length = 4; /* dbg_start_app_cfm.bootstatus. */
        break;
    case 0x000: /* MM_RESET_REQ */
        timer_del(s->scan_timer);
        timer_del(s->connect_timer);
        s->associated = false;
        s->used_vifs = 0;
        break;
    case 0x004: /* MM_VERSION_REQ */
        stl_le_p(reply, 0x06040300);
        stl_le_p(reply + 4, 0x00000001);
        stl_le_p(reply + 12, 0x00000101); /* one stream, 20 MHz PHY */
        stl_le_p(reply + 16, 0x00000001);
        stl_le_p(reply + 20, BIT(8) | BIT(0)); /* FullMAC and beaconing */
        stw_le_p(reply + 24, 8);
        reply[26] = 4;
        reply_length = 28;
        break;
    case 0x006: /* MM_ADD_IF_REQ */
        reply_length = 2;
        if (parameter_length >= 6) {
            memcpy(s->station, data, 6);
        }
        while (reply[1] < 4 && (s->used_vifs & BIT(reply[1]))) {
            reply[1]++;
        }
        if (reply[1] < 4) {
            s->used_vifs |= BIT(reply[1]);
        } else {
            reply[0] = 1;
        }
        break;
    case 0x008: /* MM_REMOVE_IF_REQ */
        reply_length = 0;
        if (parameter_length && data[0] < 4) {
            s->used_vifs &= ~BIT(data[0]);
            if (data[0] == s->scan_vif) {
                timer_del(s->scan_timer);
            }
            if (data[0] == s->connect_vif) {
                timer_del(s->connect_timer);
            }
            if (data[0] == s->vif) {
                s->associated = false;
            }
        }
        break;
    case 0x010: /* MM_SET_CHANNEL_REQ: radio index and actual TX power. */
        reply_length = 2;
        break;
    case 0x024: /* MM_KEY_ADD_REQ: status, hardware key index, alignment. */
        reply_length = 4;
        break;
    case 0x02e: /* MM_SET_POWER_REQ: radio index and actual TX power. */
        if (parameter_length >= 2) {
            reply[1] = data[1];
        }
        reply_length = 2;
        break;
    case 0x051: /* MM_SET_P2P_NOA_REQ: CFM follows both P2P requests. */
        confirm = 0x053;
        reply_length = 1;
        break;
    case 0x052: /* MM_SET_P2P_OPPPS_REQ. */
        confirm = 0x054;
        reply_length = 1;
        break;
    case 0x069: /* MM_SET_RF_CALIB_REQ: four calibration-table addresses. */
        reply_length = 16;
        break;
    case 0x073: /* MM_GET_MAC_ADDR_REQ */
        memcpy(reply, s->conf.macaddr.a, 6);
        reply_length = 6;
        break;
    case 0x07b: /* MM_SET_STACK_START_REQ: 5 GHz support, vendor information. */
        reply_length = 2;
        break;
    case 0x080: /* MM_GET_FW_VERSION_REQ */
        reply[0] = 13;
        memcpy(reply + 1, "20231222-QEMU", 13);
        reply_length = 64;
        break;
    case 0x084: /* MM_GET_WIFI_DISABLE_REQ. */
        reply_length = 1;
        break;
    case 0x1000: /* SCANU_START_REQ */
        aic_scan(s, fn, parameter_length > 366 ? data[366] : s->vif);
        return;
    case 0x100a: /* SCANU_CANCEL_REQ */
        timer_del(s->scan_timer);
        reply[0] = s->scan_vif;
        aic_message(s, fn, 0x1001, reply, 3);
        reply_length = 0;
        break;
    case 0x1800: /* SM_CONNECT_REQ */
        aic_connect(s, fn, parameter_length > 61 ? data[61] : s->vif);
        return;
    case 0x1803: /* SM_DISCONNECT_REQ */
        aic_message(s, fn, 0x1804, NULL, 0);
        aic_disconnect(s, parameter_length >= 2 ? lduw_le_p(data) : 3);
        return;
    case 0x1407: /* ME_STA_ADD_REQ */
        reply_length = 4;
        reply[1] = 0;
        break;
    default:
        /* These native configuration confirmations have no payload. */
        if ((id >> 10) != 0 && (id >> 10) != 5 && id != 0x1007) {
            return;
        }
        break;
    }
    aic_message(s, fn, confirm, reply, reply_length);
}

static void aic_tx_data(Aic8800State *s, unsigned fn, const uint8_t *data,
                         unsigned length)
{
    uint8_t frame[2304], confirm[8] = { 0 };
    unsigned packet_length;
    uint32_t hostid;
    if (length < AIC_TX_DESCRIPTOR_SIZE) {
        return;
    }
    packet_length = lduw_le_p(data);
    hostid = ldl_le_p(data + 4);
    if (hostid & BIT(31)) {
        uint8_t message[12] = { 0 };
        stl_le_p(confirm, BIT(0) | BIT(3)); /* transmitted and acknowledged */
        stl_le_p(confirm + 4, hostid & ~BIT(31));
        stw_le_p(message, sizeof(confirm));
        message[2] = 0x12;
        memcpy(message + 4, confirm, sizeof(confirm));
        aic_enqueue(s, fn, message, sizeof(message));
    }
    if (!s->associated || qemu_get_queue(s->nic)->link_down ||
        packet_length > length - AIC_TX_DESCRIPTOR_SIZE ||
        packet_length + 14 > sizeof(frame)) {
        return;
    }
    memcpy(frame, data + 8, 14); /* Ethernet DA, SA, EtherType */
    memcpy(frame + 14, data + AIC_TX_DESCRIPTOR_SIZE, packet_length);
    qemu_send_packet(qemu_get_queue(s->nic), frame, packet_length + 14);
}

static void aic_fifo_write(Aic8800State *s)
{
    unsigned offset = 0;
    while (offset + 4 <= s->length) {
        const uint8_t *data = s->io + offset;
        unsigned length = lduw_le_p(data) & 0xfff;
        unsigned total = ROUND_UP(length + 4, 4);
        if (!length || total > s->length - offset) {
            break;
        }
        if ((data[2] & 0x7f) == 0x11 && length >= 12) {
            /* Commands include a dummy word before the eight-byte LMAC header. */
            aic_command_message(s, s->function, data + 8, length - 4);
        } else if (!(data[2] & 0x10)) {
            aic_tx_data(s, s->function, data + 4, length);
        }
        offset += total;
    }
}

static uint8_t aic_register_read(Aic8800State *s, unsigned fn, unsigned address)
{
    AicMessage *message = g_queue_peek_head(&s->rx[fn]);
    switch (address) {
    case 0x02: return message ? DIV_ROUND_UP(message->length, 4) : 0;
    case 0x05: return BIT(4); /* active clock domain */
    case 0x0a: return 32; /* available firmware TX buffers */
    case 0x12: return message ? DIV_ROUND_UP(message->length, 512) : 0;
    }
    return address < 0x100 ? s->regs[fn][address] : 0;
}

static void aic_register_write(Aic8800State *s, unsigned fn,
                                unsigned address, uint8_t value)
{
    if (address < 0x100) {
        s->regs[fn][address] = value;
    }
    aic_update_irq(s);
}

static uint8_t aic_cccr_read(Aic8800State *s, unsigned address)
{
    if (address >= 0x1000 && address < 0x1300) {
        return s->cis[address - 0x1000];
    }
    return address < sizeof(s->cccr) ? s->cccr[address] : 0;
}

static void aic_reset(DeviceState *dev)
{
    Aic8800State *s = AIC8800_SDIO(dev);
    timer_del(s->scan_timer);
    timer_del(s->connect_timer);
    for (unsigned fn = 1; fn <= 2; fn++) {
        g_queue_clear_full(&s->rx[fn], g_free);
    }
    g_hash_table_remove_all(s->memory);
    memset(s->cccr, 0, sizeof(s->cccr));
    memset(s->regs, 0, sizeof(s->regs));
    s->cccr[0] = 0x32;
    s->cccr[1] = 3;
    s->cccr[8] = 0x1e;
    stl_le_p(s->cccr + 9, 0x1000);
    s->cccr[0x13] = 1;
    for (unsigned fn = 1; fn <= 2; fn++) {
        s->cccr[fn * 0x100] = 7;
        stl_le_p(s->cccr + fn * 0x100 + 9, 0x1000 + fn * 0x100);
        stw_le_p(s->cccr + fn * 0x100 + 0x10, 512);
    }
    s->position = s->length = s->function = 0;
    s->vif = s->used_vifs = s->sequence = 0;
    s->scan_function = s->scan_vif = 0;
    s->connect_function = s->connect_vif = 0;
    s->firmware = s->associated = false;
    memcpy(s->station, s->conf.macaddr.a, 6);
    aic_update_irq(s);
}

static size_t aic_command(SDState *card, SDRequest *req,
                           uint8_t *response, size_t response_size)
{
    Aic8800State *s = AIC8800_SDIO(card);
    unsigned fn = (req->arg >> 28) & 7;
    unsigned address = (req->arg >> 9) & 0x1ffff;
    bool write = req->arg >> 31;
    if (!s->powered || response_size < 4) {
        return 0;
    }
    trace_aic8800_command(req->cmd, req->arg);
    memset(response, 0, 4);
    switch (req->cmd) {
    case 0: return 0;
    case 5: stl_be_p(response, 0xa0ff8000); return 4;
    case 3: stl_be_p(response, 0x00010000); return 4;
    case 7: return 4;
    case 52:
        if (fn > 2) {
            response[2] = 2;
        } else if (fn) {
            if (write) {
                aic_register_write(s, fn, address, req->arg);
            }
            response[3] = aic_register_read(s, fn, address);
        } else {
            if (write && address < sizeof(s->cccr)) {
                if (address == 6 && (req->arg & 8)) {
                    aic_reset(DEVICE(s));
                } else {
                    s->cccr[address] = req->arg;
                    if (address == 2) {
                        s->cccr[3] = req->arg & 6;
                    }
                }
            }
            response[3] = aic_cccr_read(s, address);
        }
        aic_update_irq(s);
        return 4;
    case 53:
        s->length = req->arg & 0x1ff;
        if (!s->length) {
            s->length = 512;
        }
        if (req->arg & BIT(27)) {
            unsigned block = fn <= 2 ? lduw_le_p(s->cccr + fn * 0x100 + 0x10) : 0;
            s->length *= block ? block : 512;
        }
        if (fn < 1 || fn > 2 || !(s->cccr[2] & BIT(fn)) ||
            s->length > sizeof(s->io)) {
            s->length = 0;
            response[2] = 2;
            return 4;
        }
        s->function = fn;
        s->address = address;
        s->increment = req->arg & BIT(26);
        s->writing = write;
        s->position = 0;
        memset(s->io, 0, sizeof(s->io));
        if (!write && address == 8) {
            AicMessage *message = g_queue_pop_head(&s->rx[fn]);
            if (message) {
                memcpy(s->io, message->data, MIN(message->length, s->length));
                g_free(message);
            }
            aic_update_irq(s);
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        return 4;
    default: return 0;
    }
}

static uint8_t aic_read_byte(SDState *card)
{
    Aic8800State *s = AIC8800_SDIO(card);
    unsigned position;
    if (s->writing || s->position >= s->length) {
        return 0;
    }
    if (s->address == 8) {
        return s->io[s->position++];
    }
    position = s->position++;
    return aic_register_read(s, s->function,
                              s->address + (s->increment ? position : 0));
}

static void aic_write_byte(SDState *card, uint8_t value)
{
    Aic8800State *s = AIC8800_SDIO(card);
    if (!s->writing || s->position >= s->length) {
        return;
    }
    s->io[s->position++] = value;
    if (s->address != 7) {
        aic_register_write(s, s->function,
                            s->address + (s->increment ? s->position - 1 : 0), value);
    } else if (s->position == s->length) {
        aic_fifo_write(s);
    }
}

static void aic_read_data(SDState *card, uint8_t *data, size_t length)
{
    while (length--) {
        *data++ = aic_read_byte(card);
    }
}

static void aic_write_data(SDState *card, const uint8_t *data, size_t length)
{
    while (length--) {
        aic_write_byte(card, *data++);
    }
}

static bool aic_receive_ready(SDState *card)
{
    Aic8800State *s = AIC8800_SDIO(card);
    return s->powered && s->writing && s->position < s->length;
}

static bool aic_data_ready(SDState *card)
{
    Aic8800State *s = AIC8800_SDIO(card);
    return s->powered && !s->writing && s->position < s->length;
}

static bool aic_inserted(SDState *card)
{
    return AIC8800_SDIO(card)->powered;
}

static bool aic_readonly(SDState *card)
{
    return false;
}

static void aic_voltage(SDState *card, uint16_t voltage)
{
}

static void aic_power(void *opaque, int line, int level)
{
    Aic8800State *s = opaque;
    if (s->powered != !!level) {
        s->powered = !!level;
        trace_aic8800_power(s->powered);
        aic_reset(DEVICE(s));
        sdbus_set_inserted(SD_BUS(qdev_get_parent_bus(DEVICE(s))), s->powered);
    }
}

static bool aic_can_receive(NetClientState *nc)
{
    Aic8800State *s = qemu_get_nic_opaque(nc);
    return s->powered && s->firmware && s->associated && !nc->link_down &&
           g_queue_get_length(&s->rx[1]) < AIC_MAX_QUEUE - 16;
}

static ssize_t aic_receive(NetClientState *nc, const uint8_t *data, size_t length)
{
    Aic8800State *s = qemu_get_nic_opaque(nc);
    uint8_t message[2400] = { 0 };
    if (length < 14 || length + 60 > sizeof(message)) {
        return length;
    }
    stw_le_p(message, length);
    stl_le_p(message + 48, BIT(6) | (s->vif << 8) | 0xff000000);
    stl_le_p(message + 52, 0xaaaaaa00);
    memcpy(message + 60, data, length);
    aic_enqueue(s, 1, message, ROUND_UP(length + 60, 4));
    return length;
}

static void aic_link_changed(NetClientState *nc)
{
    Aic8800State *s = qemu_get_nic_opaque(nc);
    if (nc->link_down && s->associated) {
        aic_disconnect(s, 3);
    }
}

static NetClientInfo aic_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = aic_can_receive,
    .receive = aic_receive,
    .link_status_changed = aic_link_changed,
};

static void aic_realize(DeviceState *dev, Error **errp)
{
    Aic8800State *s = AIC8800_SDIO(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&aic_net_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void aic_init(Object *obj)
{
    Aic8800State *s = AIC8800_SDIO(obj);
    static const uint8_t common[] = {
        0x20, 4, 0x49, 0x54, 0x45, 0x01,
        0x21, 2, 6, 0,
        0x22, 4, 0, 0, 2, 0x32, 0xff,
    };
    memset(s->cis, 0xff, sizeof(s->cis));
    memcpy(s->cis, common, sizeof(common));
    for (unsigned fn = 1; fn <= 2; fn++) {
        uint8_t *cis = s->cis + fn * 0x100;
        memcpy(cis, common, 10);
        if (fn == 2) {
            stw_le_p(cis + 2, 0x544a);
            stw_le_p(cis + 4, 0x0146);
        }
        cis[10] = 0x22;
        cis[11] = 42;
        memset(cis + 12, 0, 42);
        cis[12] = 1;
        stw_le_p(cis + 24, 2048);
        stw_le_p(cis + 40, 100);
        cis[54] = 0xff;
    }
    for (unsigned fn = 0; fn <= 2; fn++) {
        g_queue_init(&s->rx[fn]);
    }
    s->memory = g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL, g_free);
    s->scan_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, aic_scan_done, s);
    s->connect_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, aic_connect_done, s);
    qdev_init_gpio_in_named(DEVICE(s), aic_power, "power", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, "irq", 1);
}

static void aic_finalize(Object *obj)
{
    Aic8800State *s = AIC8800_SDIO(obj);
    for (unsigned fn = 0; fn <= 2; fn++) {
        g_queue_clear_full(&s->rx[fn], g_free);
    }
    g_hash_table_destroy(s->memory);
    timer_free(s->scan_timer);
    timer_free(s->connect_timer);
    if (s->nic) {
        qemu_del_nic(s->nic);
    }
}

static const Property aic_properties[] = {
    DEFINE_NIC_PROPERTIES(Aic8800State, conf),
};

static const VMStateDescription aic_vmstate = {
    .name = TYPE_AIC8800_SDIO,
    .unmigratable = 1,
};

static void aic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);
    dc->desc = "AIC8800 SDIO FullMAC Wi-Fi";
    dc->realize = aic_realize;
    dc->vmsd = &aic_vmstate;
    device_class_set_legacy_reset(dc, aic_reset);
    device_class_set_props(dc, aic_properties);
    sc->do_command = aic_command;
    sc->read_byte = aic_read_byte;
    sc->write_byte = aic_write_byte;
    sc->read_data = aic_read_data;
    sc->write_data = aic_write_data;
    sc->receive_ready = aic_receive_ready;
    sc->data_ready = aic_data_ready;
    sc->get_inserted = aic_inserted;
    sc->get_readonly = aic_readonly;
    sc->set_voltage = aic_voltage;
}

static const TypeInfo aic_type = {
    .name = TYPE_AIC8800_SDIO,
    .parent = TYPE_SDMMC_COMMON,
    .instance_size = sizeof(Aic8800State),
    .instance_init = aic_init,
    .instance_finalize = aic_finalize,
    .class_init = aic_class_init,
};

static void aic_register_types(void)
{
    type_register_static(&aic_type);
}
type_init(aic_register_types)
