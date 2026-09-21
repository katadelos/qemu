/*
 * Realtek RTL8192ES SDIO radio and an open emulated access point.
 *
 * The register/FIFO protocol follows Realtek's GPL RTL8192ES SDIO driver.
 * Guest firmware downloads initialize the modeled MAC; 802.11 management
 * frames and LLC data travel through the same descriptor queues as on silicon.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/sd/sd.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "sdmmc-internal.h"
#include "trace.h"

#define TYPE_RTL8192ES_SDIO "rtl8192es-sdio"
OBJECT_DECLARE_SIMPLE_TYPE(RTL8192ESState, RTL8192ES_SDIO)
#define RTL_MAX_FRAME 16384
#define RTL_SSID "Kindle-QEMU"
static const uint8_t rtl_bssid[6] = { 2, 0, 0, 0, 0, 3 };

typedef struct RTLPacket {
    unsigned length;
    uint8_t data[RTL_MAX_FRAME];
} RTLPacket;

struct RTL8192ESState {
    DeviceState parent_obj;
    NICConf conf;
    NICState *nic;
    qemu_irq irq, oob_irq;
    uint8_t cccr[0x200], cis[0x200], local[0x1000], regs[0x8000];
    uint8_t tx[RTL_MAX_FRAME];
    uint8_t station[6];
    uint32_t status, mask;
    unsigned address, position, length, fn;
    uint16_t sequence;
    bool powered, writing, increment, firmware, associated;
    GQueue rx;
    QEMUTimer *beacon_timer;
};

static void rtl_update_irq(RTL8192ESState *s)
{
    uint32_t pending = s->status | (g_queue_is_empty(&s->rx) ? 0 : 1);
    bool irq = s->powered && (s->cccr[4] & 3) == 3 && (pending & s->mask);

    s->cccr[5] = irq ? 2 : 0;
    qemu_set_irq(s->irq, irq);
    qemu_set_irq(s->oob_irq, irq);
}

static void rtl_enqueue(RTL8192ESState *s, const uint8_t *frame, size_t length)
{
    RTLPacket *packet;
    unsigned fcs = (ldl_le_p(s->regs + 0x608) & BIT(31)) ? 4 : 0;

    if (!s->powered || !s->firmware || length + 24 + fcs > RTL_MAX_FRAME ||
        g_queue_get_length(&s->rx) >= 128) {
        return;
    }
    packet = g_new0(RTLPacket, 1);
    packet->length = 24 + length + fcs;
    stl_le_p(packet->data, length + fcs);
    stl_le_p(packet->data + 4, BIT(14) | BIT(24)); /* RX ID match, PAM */
    stl_le_p(packet->data + 8, (lduw_le_p(frame + 22) >> 4) | BIT(31));
    stl_le_p(packet->data + 12, 0); /* 1 Mbps, no PHY trailer */
    memcpy(packet->data + 24, frame, length);
    g_queue_push_tail(&s->rx, packet);
    trace_rtl8192es_frame(false, lduw_le_p(frame), length);
    rtl_update_irq(s);
}

static unsigned rtl_header(RTL8192ESState *s, uint8_t *frame, uint16_t fc,
                           const uint8_t *destination)
{
    memset(frame, 0, 24);
    stw_le_p(frame, fc);
    memcpy(frame + 4, destination, 6);
    memcpy(frame + 10, rtl_bssid, 6);
    memcpy(frame + 16, rtl_bssid, 6);
    stw_le_p(frame + 22, s->sequence++ << 4);
    return 24;
}

static void rtl_beacon(RTL8192ESState *s, bool probe, const uint8_t *destination)
{
    static const uint8_t ies[] = {
        0, sizeof(RTL_SSID) - 1, 'K', 'i', 'n', 'd', 'l', 'e', '-', 'Q', 'E', 'M', 'U',
        1, 8, 0x82, 0x84, 0x8b, 0x96, 12, 18, 24, 36,
        3, 1, 1,
        50, 4, 48, 72, 96, 108,
        5, 4, 0, 1, 0, 0,
    };
    uint8_t frame[128] = { 0 };
    unsigned pos = rtl_header(s, frame, probe ? 0x50 : 0x80, destination);

    if (qemu_get_queue(s->nic)->link_down) {
        return;
    }
    stq_le_p(frame + pos, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    stw_le_p(frame + pos + 8, 100);
    stw_le_p(frame + pos + 10, 0x421);
    memcpy(frame + pos + 12, ies, sizeof(ies));
    rtl_enqueue(s, frame, pos + 12 + sizeof(ies));
}

static void rtl_beacon_tick(void *opaque)
{
    RTL8192ESState *s = opaque;
    static const uint8_t broadcast[6] = { 255, 255, 255, 255, 255, 255 };

    if (s->powered && s->firmware) {
        rtl_beacon(s, false, broadcast);
        timer_mod(s->beacon_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 102400000);
    }
}

static void rtl_tx_report(RTL8192ESState *s)
{
    RTLPacket *packet;

    if (g_queue_get_length(&s->rx) >= 128) {
        return;
    }
    packet = g_new0(RTLPacket, 1);
    packet->length = 24 + 8;
    stl_le_p(packet->data, 8);
    stl_le_p(packet->data + 8, BIT(28)); /* C2H, rather than an 802.11 frame */
    packet->data[24] = 3; /* C2H_CCX_TX_RPT, successful transmission */
    g_queue_push_tail(&s->rx, packet);
    rtl_update_irq(s);
}

static void rtl_tx_frame(RTL8192ESState *s, const uint8_t *data, unsigned length)
{
    unsigned packet_length, offset, subtype, pos;
    uint16_t fc;
    const uint8_t *frame;
    uint8_t reply[2048] = { 0 };

    while (length >= 40) {
        packet_length = lduw_le_p(data);
        offset = data[2];
        if (offset < 40 || offset + packet_length > length || packet_length < 24) {
            return;
        }
        frame = data + offset;
        fc = lduw_le_p(frame);
        subtype = fc & 0xfc;
        memcpy(s->station, frame + 10, 6);
        trace_rtl8192es_frame(true, fc, packet_length);
        if (subtype == 0x80) { /* reserved beacon page downloaded */
            s->regs[0x20a] |= 1;
        } else if (qemu_get_queue(s->nic)->link_down) {
            /* The access point cannot answer while the radio link is down. */
        } else if (subtype == 0x40) { /* probe request */
            rtl_beacon(s, true, frame + 10);
        } else if (subtype == 0xb0 && packet_length >= 30) { /* open authentication */
            pos = rtl_header(s, reply, 0xb0, frame + 10);
            stw_le_p(reply + pos, 0);
            stw_le_p(reply + pos + 2, 2);
            stw_le_p(reply + pos + 4, 0);
            rtl_enqueue(s, reply, pos + 6);
        } else if (subtype == 0 || subtype == 0x20) { /* (re)association */
            static const uint8_t rates[] = { 1, 8, 0x82, 0x84, 0x8b, 0x96, 12, 18, 24, 36 };
            pos = rtl_header(s, reply, subtype == 0 ? 0x10 : 0x30, frame + 10);
            stw_le_p(reply + pos, 0x421);
            stw_le_p(reply + pos + 2, 0);
            stw_le_p(reply + pos + 4, 0xc001);
            memcpy(reply + pos + 6, rates, sizeof(rates));
            s->associated = true;
            rtl_enqueue(s, reply, pos + 6 + sizeof(rates));
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        } else if ((fc & 0xc) == 8 && (fc & 0x100)) { /* ToDS data */
            unsigned header = 24 + ((fc & 0x80) ? 2 : 0);
            unsigned payload = packet_length - MIN(packet_length, header);

            if (payload >= 8 && payload + 6 <= sizeof(reply) &&
                !memcmp(frame + header, "\xaa\xaa\x03\0\0\0", 6)) {
                memcpy(reply, frame + 16, 6);
                memcpy(reply + 6, frame + 10, 6);
                memcpy(reply + 12, frame + header + 6, payload - 6);
                qemu_send_packet(qemu_get_queue(s->nic), reply, payload + 6);
            }
        } else if (subtype == 0xa0 || subtype == 0xc0) {
            s->associated = false;
        }
        if (ldl_le_p(data + 8) & BIT(19)) {
            /* The driver waits for a firmware acknowledgement of null data
             * frames when entering/leaving power save and background scans. */
            rtl_tx_report(s);
        }
        pos = ROUND_UP(offset + packet_length, 8);
        if (pos > length) {
            break;
        }
        data += pos;
        length -= pos;
    }
}

static uint8_t rtl_read(RTL8192ESState *s, unsigned fn, unsigned address)
{
    if (!fn) {
        if (address >= 0x1000 && address < 0x1200) {
            return s->cis[address - 0x1000];
        }
        return address < sizeof(s->cccr) ? s->cccr[address] : 0;
    }
    if (address < 0x1000) {
        RTLPacket *packet = g_queue_peek_head(&s->rx);
        if (address >= 0x18 && address < 0x1c) {
            return (s->status | (packet ? 1 : 0)) >> ((address - 0x18) * 8);
        }
        if (address == 0x1c || address == 0x1d) {
            return (packet ? packet->length : 0) >> ((address - 0x1c) * 8);
        }
        if (address >= 0x1e && address < 0x24) {
            return 0xff;
        }
        return s->local[address];
    }
    if (address & 0x10000) {
        unsigned reg = address & 0x7fff;
        uint8_t value = s->regs[reg];
        if (reg == 0x80) {
            value |= 4; /* firmware download checksum */
            if (value & 2) {
                value |= 0xc0; /* MCU running and initialization ready */
            }
        } else if (reg == 0x30) {
            value = 0xff; /* unprogrammed EFUSE data */
        } else if (reg == 0x33) {
            value |= 0x80; /* EFUSE read complete */
        }
        return value;
    }
    return 0;
}

static void rtl_write(RTL8192ESState *s, unsigned fn, unsigned address, uint8_t value)
{
    if (!fn) {
        if (address >= sizeof(s->cccr)) {
            return;
        }
        s->cccr[address] = value;
        if (address == 2) {
            s->cccr[3] = value & 2;
        }
    } else if (address < 0x1000) {
        s->local[address] = value;
        if (address >= 0x14 && address < 0x18) {
            s->mask = ldl_le_p(s->local + 0x14);
        } else if (address >= 0x18 && address < 0x1c) {
            s->status &= ~((uint32_t)value << ((address - 0x18) * 8));
        } else if (address == 0x86) {
            s->local[address] = (value & ~2) | ((value & 1) ? 0 : 2);
        } else if (address == 0x80) {
            s->local[0x24] = value;
            s->status |= BIT(18); /* CPWM1 */
        }
    } else if (address & 0x10000) {
        unsigned reg = address & 0x7fff;
        uint8_t previous = s->regs[reg];

        s->regs[reg] = value;
        if (reg == 5) {
            s->regs[reg] &= ~3; /* power-on/off state machines complete */
        } else if (reg == 6) {
            s->regs[reg] |= 2;
        } else if (reg == 0x1e3 || reg == 0x673) {
            s->regs[reg] &= 0x3f; /* LLT/CAM command complete */
        } else if (reg == 0x226) {
            s->regs[reg] &= ~1; /* automatic LLT initialization complete */
        } else if (reg == 0x20a || reg == 0x22a) {
            /* BCN_VALID is set on download and acknowledged with W1C. */
            s->regs[reg] = (value & ~1) | (previous & ~value & 1);
        } else if (reg == 0x80) {
            s->firmware = value & 2;
            if (s->firmware) {
                timer_mod(s->beacon_timer,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 102400000);
            } else {
                timer_del(s->beacon_timer);
                g_queue_clear_full(&s->rx, g_free);
                s->associated = false;
            }
        }
        trace_rtl8192es_register(true, reg, s->regs[reg]);
    }
    rtl_update_irq(s);
}

static size_t rtl_command(SDState *card, SDRequest *req, uint8_t *response,
                          size_t response_size)
{
    RTL8192ESState *s = RTL8192ES_SDIO(card);
    unsigned fn = (req->arg >> 28) & 7;
    unsigned address = (req->arg >> 9) & 0x1ffff;
    bool write = req->arg >> 31;

    if (!s->powered || response_size < 4) {
        return 0;
    }
    trace_rtl8192es_command(req->cmd, req->arg);
    memset(response, 0, 4);
    switch (req->cmd) {
    case 0: return 0;
    case 5: stl_be_p(response, 0x90ff8000); return 4;
    case 3: stl_be_p(response, 0x00010000); return 4;
    case 7: return 4;
    case 52:
        if (fn > 1) {
            response[2] = 2;
            return 4;
        }
        if (write) {
            rtl_write(s, fn, address, req->arg);
        }
        response[3] = (!write || (req->arg & BIT(27))) ?
                      rtl_read(s, fn, address) : (uint8_t)req->arg;
        return 4;
    case 53:
        if (fn != 1 || !(s->cccr[2] & 2)) {
            response[2] = 2;
            return 4;
        }
        s->fn = fn;
        s->address = address;
        s->writing = write;
        s->increment = req->arg & BIT(26);
        s->length = req->arg & 0x1ff;
        if (!s->length) {
            s->length = 512;
        }
        if (req->arg & BIT(27)) {
            unsigned block = lduw_le_p(s->cccr + 0x110);
            s->length *= block ? block : 512;
        }
        s->position = 0;
        return 4;
    default: return 0;
    }
}

static uint8_t rtl_read_byte(SDState *card)
{
    RTL8192ESState *s = RTL8192ES_SDIO(card);
    uint8_t value = 0;

    if (s->writing || s->position >= s->length) {
        return 0;
    }
    if ((s->address >> 13) == 7) {
        RTLPacket *packet = g_queue_peek_head(&s->rx);
        if (packet && s->position < packet->length) {
            value = packet->data[s->position];
        }
        if (++s->position == s->length && packet) {
            g_free(g_queue_pop_head(&s->rx));
            rtl_update_irq(s);
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
    } else {
        value = rtl_read(s, s->fn, s->address + (s->increment ? s->position : 0));
        s->position++;
    }
    return value;
}

static void rtl_write_byte(SDState *card, uint8_t value)
{
    RTL8192ESState *s = RTL8192ES_SDIO(card);
    unsigned domain = s->address >> 13;

    if (!s->writing || s->position >= s->length) {
        return;
    }
    if (domain >= 3 && domain <= 6) {
        if (s->position < sizeof(s->tx)) {
            s->tx[s->position] = value;
        }
        if (++s->position == s->length && s->length <= sizeof(s->tx)) {
            rtl_tx_frame(s, s->tx, s->length);
        }
    } else {
        rtl_write(s, s->fn, s->address + (s->increment ? s->position : 0), value);
        s->position++;
    }
}

static void rtl_read_data(SDState *card, uint8_t *data, size_t length)
{
    while (length--) {
        *data++ = rtl_read_byte(card);
    }
}

static void rtl_write_data(SDState *card, const uint8_t *data, size_t length)
{
    while (length--) {
        rtl_write_byte(card, *data++);
    }
}

static bool rtl_receive_ready(SDState *card)
{
    RTL8192ESState *s = RTL8192ES_SDIO(card);
    return s->powered && s->writing && s->position < s->length;
}

static bool rtl_data_ready(SDState *card)
{
    RTL8192ESState *s = RTL8192ES_SDIO(card);
    return s->powered && !s->writing && s->position < s->length;
}

static bool rtl_inserted(SDState *card)
{
    return RTL8192ES_SDIO(card)->powered;
}

static bool rtl_readonly(SDState *card)
{
    return false;
}

static void rtl_voltage(SDState *card, uint16_t voltage)
{
}

static void rtl_reset(DeviceState *dev)
{
    RTL8192ESState *s = RTL8192ES_SDIO(dev);
    timer_del(s->beacon_timer);
    g_queue_clear_full(&s->rx, g_free);
    memset(s->cccr, 0, sizeof(s->cccr));
    memset(s->local, 0, sizeof(s->local));
    memset(s->regs, 0, sizeof(s->regs));
    s->cccr[0] = 0x32;
    s->cccr[1] = 3;
    s->cccr[8] = 0x1e;
    stl_le_p(s->cccr + 9, 0x1000);
    s->cccr[0x13] = 1;
    s->cccr[0x100] = 7; /* WLAN function */
    stl_le_p(s->cccr + 0x109, 0x1100);
    stw_le_p(s->cccr + 0x110, 512);
    s->local[0x86] = 2;
    s->regs[6] = 2;
    s->regs[0xf0] = 0x80; /* normal production RTL8192E, 2T2R */
    s->regs[0xf2] = 0x10;
    s->status = s->mask = 0;
    s->length = s->position = 0;
    s->associated = s->firmware = false;
    s->sequence = 0;
    memcpy(s->station, s->conf.macaddr.a, 6);
    rtl_update_irq(s);
}

static void rtl_power(void *opaque, int line, int level)
{
    RTL8192ESState *s = opaque;
    if (s->powered != !!level) {
        s->powered = level;
        trace_rtl8192es_power(s->powered);
        rtl_reset(DEVICE(s));
        sdbus_set_inserted(SD_BUS(qdev_get_parent_bus(DEVICE(s))), s->powered);
    }
}

static bool rtl_can_receive(NetClientState *nc)
{
    RTL8192ESState *s = qemu_get_nic_opaque(nc);
    return s->powered && s->firmware && s->associated && !nc->link_down &&
           g_queue_get_length(&s->rx) < 128;
}

static ssize_t rtl_receive(NetClientState *nc, const uint8_t *data, size_t length)
{
    RTL8192ESState *s = qemu_get_nic_opaque(nc);
    uint8_t frame[2048];
    unsigned pos;

    if (length < 14 || length + 18 > sizeof(frame)) {
        return length;
    }
    pos = rtl_header(s, frame, 0x0208, data);
    memcpy(frame + 16, data + 6, 6);
    memcpy(frame + pos, "\xaa\xaa\x03\0\0\0", 6);
    memcpy(frame + pos + 6, data + 12, length - 12);
    rtl_enqueue(s, frame, pos + length - 6);
    return length;
}

static void rtl_link_status_changed(NetClientState *nc)
{
    RTL8192ESState *s = qemu_get_nic_opaque(nc);
    uint8_t frame[26];

    if (nc->link_down && s->associated) {
        rtl_header(s, frame, 0xc0, s->station); /* deauthentication */
        stw_le_p(frame + 24, 3); /* station leaving */
        rtl_enqueue(s, frame, sizeof(frame));
        s->associated = false;
    }
}

static NetClientInfo rtl_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = rtl_can_receive,
    .receive = rtl_receive,
    .link_status_changed = rtl_link_status_changed,
};

static void rtl_realize(DeviceState *dev, Error **errp)
{
    RTL8192ESState *s = RTL8192ES_SDIO(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&rtl_net_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void rtl_init(Object *obj)
{
    RTL8192ESState *s = RTL8192ES_SDIO(obj);
    static const uint8_t common[] = {
        0x20, 4, 0x4c, 0x02, 0x8b, 0x81,
        0x21, 2, 6, 0,
        0x22, 4, 0, 0, 2, 0x32, 0xff,
    };
    memset(s->cis, 0xff, sizeof(s->cis));
    memcpy(s->cis, common, sizeof(common));
    memcpy(s->cis + 0x100, common, 10);
    s->cis[0x10a] = 0x22;
    s->cis[0x10b] = 42;
    memset(s->cis + 0x10c, 0, 42);
    s->cis[0x10c] = 1;
    stw_le_p(s->cis + 0x118, 512);
    stw_le_p(s->cis + 0x128, 100);
    s->cis[0x136] = 0xff;
    g_queue_init(&s->rx);
    s->beacon_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rtl_beacon_tick, s);
    qdev_init_gpio_in_named(DEVICE(s), rtl_power, "power", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, "irq", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->oob_irq, "oob-irq", 1);
}

static void rtl_finalize(Object *obj)
{
    RTL8192ESState *s = RTL8192ES_SDIO(obj);
    timer_free(s->beacon_timer);
    g_queue_clear_full(&s->rx, g_free);
    if (s->nic) {
        qemu_del_nic(s->nic);
    }
}

static const Property rtl_properties[] = {
    DEFINE_NIC_PROPERTIES(RTL8192ESState, conf),
};

static const VMStateDescription rtl_vmstate = {
    .name = TYPE_RTL8192ES_SDIO,
    .unmigratable = 1,
};

static void rtl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);
    dc->desc = "Realtek RTL8192ES SDIO Wi-Fi";
    dc->realize = rtl_realize;
    dc->vmsd = &rtl_vmstate;
    device_class_set_legacy_reset(dc, rtl_reset);
    device_class_set_props(dc, rtl_properties);
    sc->do_command = rtl_command;
    sc->read_byte = rtl_read_byte;
    sc->write_byte = rtl_write_byte;
    sc->read_data = rtl_read_data;
    sc->write_data = rtl_write_data;
    sc->receive_ready = rtl_receive_ready;
    sc->data_ready = rtl_data_ready;
    sc->get_inserted = rtl_inserted;
    sc->get_readonly = rtl_readonly;
    sc->set_voltage = rtl_voltage;
}

static const TypeInfo rtl_type = {
    .name = TYPE_RTL8192ES_SDIO,
    .parent = TYPE_SDMMC_COMMON,
    .instance_size = sizeof(RTL8192ESState),
    .instance_init = rtl_init,
    .instance_finalize = rtl_finalize,
    .class_init = rtl_class_init,
};

static void rtl_register_types(void)
{
    type_register_static(&rtl_type);
}
type_init(rtl_register_types)
