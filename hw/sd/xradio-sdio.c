/*
 * Allwinner XR819 SDIO Wi-Fi and an open emulated access point.
 *
 * The HIF registers, download handshake and WSM messages follow PocketBook's
 * GPL kernel-b288 xradio driver. The original guest modules perform scanning,
 * authentication, association and data transport through the emulated radio.
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

#define TYPE_XRADIO_SDIO "xradio-sdio"
OBJECT_DECLARE_SIMPLE_TYPE(XRadioState, XRADIO_SDIO)
#define XR_MAX_MESSAGE 8192
#define XR_RX_FRAMES 96
#define XR_RX_MESSAGES 128
#define XR_SSID "PocketBook-QEMU"
#define XR_CONFIG_ACCESS BIT(10)
#define XR_CONFIG_IRQ BIT(16)
#define XR_DOWNLOAD_SIZE 0xff80
#define XR_DOWNLOAD_PUT 0xff88
#define XR_DOWNLOAD_GET 0xff90
#define XR_DOWNLOAD_STATUS 0xff94
static const uint8_t xr_bssid[6] = { 2, 0, 0, 0, 0, 4 };

typedef struct XRMessage {
    unsigned length;
    uint8_t data[XR_MAX_MESSAGE];
} XRMessage;

struct XRadioState {
    DeviceState parent_obj;
    NICConf conf;
    NICState *nic;
    qemu_irq irq;
    uint8_t cccr[0x200], cis[0x200], memory[0x10000];
    uint8_t io[XR_MAX_MESSAGE], station[6];
    uint32_t config, control, base, dpll;
    unsigned address, position, length, fn, interface;
    uint16_t sequence;
    uint8_t wsm_sequence;
    bool powered, writing, firmware, associated;
    GQueue rx;
    QEMUTimer *beacon_timer;
};

static uint16_t xr_control(XRadioState *s)
{
    XRMessage *message = g_queue_peek_head(&s->rx);
    return (s->control & BIT(12)) | BIT(13) |
           (message ? DIV_ROUND_UP(message->length, 2) : 0);
}

static void xr_update_irq(XRadioState *s)
{
    bool pending = s->powered && (s->config & XR_CONFIG_IRQ) &&
                   !g_queue_is_empty(&s->rx);
    s->cccr[5] = pending ? 2 : 0;
    qemu_set_irq(s->irq, pending);
}

static void xr_enqueue(XRadioState *s, uint16_t id,
                       const uint8_t *payload, unsigned length)
{
    XRMessage *message;

    if (!s->powered || length > XR_MAX_MESSAGE - 4 ||
        g_queue_get_length(&s->rx) >= XR_RX_MESSAGES) {
        return;
    }
    message = g_new0(XRMessage, 1);
    message->length = length + 4;
    stw_le_p(message->data, message->length);
    stw_le_p(message->data + 2, id | (s->wsm_sequence++ & 7) << 13);
    memcpy(message->data + 4, payload, length);
    g_queue_push_tail(&s->rx, message);
    trace_xradio_wsm(false, id, message->length);
    xr_update_irq(s);
}

static void xr_rx_frame(XRadioState *s, const uint8_t *frame,
                        unsigned length, unsigned interface)
{
    uint8_t payload[XR_MAX_MESSAGE - 4] = { 0 };

    /* Keep room for command/TX completions while the host delivers traffic. */
    if (!s->firmware || length > sizeof(payload) - 12 || length < 24 ||
        g_queue_get_length(&s->rx) >= XR_RX_FRAMES) {
        return;
    }
    stw_le_p(payload + 4, 1); /* channel 1 */
    payload[6] = 0; /* 1 Mbps */
    payload[7] = 140; /* RCPI: -40 dBm */
    memcpy(payload + 12, frame, length);
    trace_xradio_frame(false, lduw_le_p(frame), length);
    xr_enqueue(s, 0x804 | (interface << 6), payload, length + 12);
}

static unsigned xr_header(XRadioState *s, uint8_t *frame, uint16_t fc,
                           const uint8_t *destination)
{
    memset(frame, 0, 24);
    stw_le_p(frame, fc);
    memcpy(frame + 4, destination, 6);
    memcpy(frame + 10, xr_bssid, 6);
    memcpy(frame + 16, xr_bssid, 6);
    stw_le_p(frame + 22, s->sequence++ << 4);
    return 24;
}

static void xr_beacon(XRadioState *s, bool probe, unsigned interface)
{
    static const uint8_t broadcast[] = { 255, 255, 255, 255, 255, 255 };
    static const uint8_t rates[] = {
        1, 8, 0x82, 0x84, 0x8b, 0x96, 12, 18, 24, 36,
        3, 1, 1, 50, 4, 48, 72, 96, 108, 5, 4, 0, 1, 0, 0,
    };
    uint8_t frame[128] = { 0 };
    unsigned pos;

    if (qemu_get_queue(s->nic)->link_down) {
        return;
    }
    pos = xr_header(s, frame, probe ? 0x50 : 0x80,
                    probe ? s->station : broadcast);
    stq_le_p(frame + pos, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    stw_le_p(frame + pos + 8, 100);
    stw_le_p(frame + pos + 10, 0x421);
    pos += 12;
    frame[pos++] = 0;
    frame[pos++] = sizeof(XR_SSID) - 1;
    memcpy(frame + pos, XR_SSID, sizeof(XR_SSID) - 1);
    pos += sizeof(XR_SSID) - 1;
    memcpy(frame + pos, rates, sizeof(rates));
    xr_rx_frame(s, frame, pos + sizeof(rates), interface);
}

static void xr_beacon_tick(void *opaque)
{
    XRadioState *s = opaque;

    if (s->powered && s->firmware && s->associated) {
        xr_beacon(s, false, s->interface);
        timer_mod(s->beacon_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 102400000);
    }
}

static void xr_tx_frame(XRadioState *s, const uint8_t *data,
                        unsigned length, unsigned interface)
{
    uint8_t confirm[32] = { 0 }, reply[2048] = { 0 };
    unsigned offset = 24 + ((data[11] & BIT(7)) ? 2 : 0);
    unsigned pos, subtype;
    uint16_t fc;
    const uint8_t *frame;

    if (length < offset + 24) {
        return;
    }
    frame = data + offset;
    length -= offset;
    fc = lduw_le_p(frame);
    subtype = fc & 0xfc;
    memcpy(s->station, frame + 10, 6);
    trace_xradio_frame(true, fc, length);
    memcpy(confirm, data + 4, 4); /* packetID */
    confirm[8] = data[8]; /* transmitted rate */
    xr_enqueue(s, 0x404 | (interface << 6), confirm, sizeof(confirm));

    if (qemu_get_queue(s->nic)->link_down) {
        return;
    }
    if (subtype == 0x40) {
        xr_beacon(s, true, interface);
    } else if (subtype == 0xb0 && length >= 30) {
        pos = xr_header(s, reply, 0xb0, frame + 10);
        stw_le_p(reply + pos + 2, 2); /* open authentication, transaction 2 */
        xr_rx_frame(s, reply, pos + 6, interface);
    } else if (subtype == 0 || subtype == 0x20) {
        static const uint8_t rates[] = {
            1, 8, 0x82, 0x84, 0x8b, 0x96, 12, 18, 24, 36,
        };
        pos = xr_header(s, reply, subtype ? 0x30 : 0x10, frame + 10);
        stw_le_p(reply + pos, 0x421);
        stw_le_p(reply + pos + 4, 0xc001); /* association ID 1 */
        memcpy(reply + pos + 6, rates, sizeof(rates));
        s->associated = true;
        s->interface = interface;
        xr_rx_frame(s, reply, pos + 6 + sizeof(rates), interface);
        timer_mod(s->beacon_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 102400000);
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    } else if ((fc & 0xc) == 8 && (fc & 0x300) == 0x100) {
        unsigned header = 24 + ((fc & 0x80) ? 2 : 0);
        unsigned payload = length - MIN(length, header);

        if (payload >= 8 && payload + 6 <= sizeof(reply) &&
            !memcmp(frame + header, "\xaa\xaa\x03\0\0\0", 6)) {
            memcpy(reply, frame + 16, 6);
            memcpy(reply + 6, frame + 10, 6);
            memcpy(reply + 12, frame + header + 6, payload - 6);
            qemu_send_packet(qemu_get_queue(s->nic), reply, payload + 6);
        }
    } else if (subtype == 0xa0 || subtype == 0xc0) {
        s->associated = false;
        timer_del(s->beacon_timer);
    }
}

static void xr_wsm(XRadioState *s, const uint8_t *data, unsigned length)
{
    uint8_t reply[256] = { 0 };
    unsigned id, command, interface, size = 4;

    if (length < 4 || lduw_le_p(data) < 4 || lduw_le_p(data) > length) {
        return;
    }
    length = lduw_le_p(data);
    id = lduw_le_p(data + 2) & 0xfff;
    command = id & 0x3f;
    interface = (id >> 6) & 0xf;
    trace_xradio_wsm(true, id, length);
    if (command == 4) {
        if (length >= 24) {
            xr_tx_frame(s, data, length, interface);
        }
        return;
    }
    switch (command) {
    case 5: /* read MIB */
        if (length < 8) {
            return;
        }
        memcpy(reply + 4, data + 4, 2);
        size = MIN(lduw_le_p(data + 6), sizeof(reply) - 8);
        stw_le_p(reply + 6, size);
        size += 8;
        break;
    case 6: /* write MIB */
        if (length >= 14 && lduw_le_p(data + 4) == 0) {
            memcpy(s->station, data + 8, 6);
        }
        break;
    case 9: /* configuration */
        if (length >= 26) {
            memcpy(s->station, data + 20, 6);
        }
        memcpy(reply + 4, s->station, 6);
        reply[10] = 1; /* 2.4 GHz */
        stl_le_p(reply + 12, 0xfff);
        stl_le_p(reply + 20, 200); /* maximum power in tenths of dBm */
        stl_le_p(reply + 24, 10); /* power step */
        size = 40;
        break;
    case 0xa: /* reset MAC */
        s->associated = false;
        timer_del(s->beacon_timer);
        break;
    case 0xb: /* join BSS */
        stl_le_p(reply + 8, 200);
        size = 12;
        break;
    }
    xr_enqueue(s, id | 0x400, reply, size);
    if (command == 7) {
        /* Scan results use the generic interface, as on XR819 firmware. */
        xr_beacon(s, false, 2);
        memset(reply, 0, 8);
        reply[5] = length > 13 ? data[13] : 1;
        xr_enqueue(s, 0x806, reply, 8);
    } else if (command == 0x10) {
        xr_enqueue(s, 0x809 | (interface << 6), reply, 4);
    }
}

static void xr_startup(XRadioState *s)
{
    uint8_t payload[148] = { 0 };
    stw_le_p(payload, 8); /* input buffers */
    stw_le_p(payload + 2, 4096);
    stw_le_p(payload + 4, 819);
    stw_le_p(payload + 12, 2); /* WSM firmware */
    stw_le_p(payload + 14, 1); /* API version */
    stw_le_p(payload + 18, 1);
    memcpy(payload + 20, "QEMU XR819 WSM", 14);
    xr_enqueue(s, 0x801, payload, sizeof(payload));
}

static void xr_hif_write(XRadioState *s)
{
    unsigned reg = s->address & 0x1f;
    uint32_t value = s->length >= 4 ? ldl_le_p(s->io) : 0;

    trace_xradio_hif(true, reg, s->base, s->length, value);
    if (reg == 8) {
        xr_wsm(s, s->io, s->length);
    } else if (reg == 0 && s->length >= 4) {
        uint32_t old = s->config;
        s->config = (value & ~(BIT(11) | BIT(13))) | BIT(26);
        if (!(value & (BIT(12) | BIT(14))) &&
            ldl_le_p(s->memory + XR_DOWNLOAD_SIZE) == 0x87654321) {
            stl_le_p(s->memory + XR_DOWNLOAD_SIZE, 0x12345678);
        }
        if ((old & XR_CONFIG_ACCESS) && !(value & XR_CONFIG_ACCESS) &&
            s->firmware) {
            xr_startup(s);
        }
    } else if (reg == 4) {
        s->control = value;
    } else if (reg == 16) {
        s->base = value;
    } else if (reg == 24) {
        s->dpll = value;
    } else if ((reg == 12 || reg == 20) &&
               s->length <= sizeof(s->memory) - (s->base & 0xffff)) {
        memcpy(s->memory + (s->base & 0xffff), s->io, s->length);
        if ((s->base & 0xffff) == XR_DOWNLOAD_PUT) {
            uint32_t image = ldl_le_p(s->memory + XR_DOWNLOAD_SIZE);
            stl_le_p(s->memory + XR_DOWNLOAD_GET, value);
            if (image && image < sizeof(s->memory) * 16 && value >= image) {
                stl_le_p(s->memory + XR_DOWNLOAD_STATUS, 0);
                s->firmware = true;
            }
        }
    }
    xr_update_irq(s);
}

static void xr_hif_read(XRadioState *s)
{
    unsigned reg = s->address & 0x1f;
    uint32_t value = 0;
    memset(s->io, 0, sizeof(s->io));
    if (reg == 8) {
        XRMessage *message = g_queue_pop_head(&s->rx);
        if (message) {
            memcpy(s->io, message->data, MIN(message->length, s->length));
            g_free(message);
        }
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        if (s->length >= 2) {
            stw_le_p(s->io + s->length - 2, xr_control(s));
        }
        xr_update_irq(s);
    } else if (reg == 12 || reg == 20) {
        if (s->length <= sizeof(s->memory) - (s->base & 0xffff)) {
            memcpy(s->io, s->memory + (s->base & 0xffff), s->length);
        }
    } else {
        switch (reg) {
        case 0: value = s->config; break;
        case 4: value = xr_control(s); break;
        case 16: value = s->base; break;
        case 24: value = s->dpll; break;
        }
        stl_le_p(s->io, value);
    }
    trace_xradio_hif(false, reg, s->base, s->length, ldl_le_p(s->io));
}

static uint8_t xr_cccr_read(XRadioState *s, unsigned address)
{
    if (address >= 0x1000 && address < 0x1200) {
        return s->cis[address - 0x1000];
    }
    return address < sizeof(s->cccr) ? s->cccr[address] : 0;
}

static size_t xr_command(SDState *card, SDRequest *req,
                         uint8_t *response, size_t response_size)
{
    XRadioState *s = XRADIO_SDIO(card);
    unsigned fn = (req->arg >> 28) & 7;
    unsigned address = (req->arg >> 9) & 0x1ffff;
    bool write = req->arg >> 31;

    if (!s->powered || response_size < 4) {
        return 0;
    }
    trace_xradio_command(req->cmd, req->arg);
    memset(response, 0, 4);
    switch (req->cmd) {
    case 0: return 0;
    case 5: stl_be_p(response, 0x90ff8000); return 4;
    case 3: stl_be_p(response, 0x00010000); return 4;
    case 7: return 4;
    case 52:
        if (fn || address >= 0x1200) {
            response[2] = 2;
        } else {
            if (write && address < sizeof(s->cccr)) {
                s->cccr[address] = req->arg;
                if (address == 2) {
                    s->cccr[3] = req->arg & 2;
                }
            }
            response[3] = xr_cccr_read(s, address);
        }
        return 4;
    case 53:
        s->length = req->arg & 0x1ff;
        if (!s->length) {
            s->length = 512;
        }
        if (req->arg & BIT(27)) {
            unsigned block = lduw_le_p(s->cccr + 0x110);
            s->length *= block ? block : 512;
        }
        if (fn != 1 || !(s->cccr[2] & 2) || s->length > sizeof(s->io)) {
            s->length = 0;
            response[2] = 2;
            return 4;
        }
        s->fn = fn;
        s->address = address;
        s->writing = write;
        s->position = 0;
        if (!write) {
            xr_hif_read(s);
        }
        return 4;
    default: return 0;
    }
}

static uint8_t xr_read_byte(SDState *card)
{
    XRadioState *s = XRADIO_SDIO(card);
    if (s->writing || s->position >= s->length) {
        return 0;
    }
    return s->io[s->position++];
}

static void xr_write_byte(SDState *card, uint8_t value)
{
    XRadioState *s = XRADIO_SDIO(card);
    if (!s->writing || s->position >= s->length) {
        return;
    }
    s->io[s->position++] = value;
    if (s->position == s->length) {
        xr_hif_write(s);
    }
}

static void xr_read_data(SDState *card, uint8_t *data, size_t length)
{
    while (length--) {
        *data++ = xr_read_byte(card);
    }
}

static void xr_write_data(SDState *card, const uint8_t *data, size_t length)
{
    while (length--) {
        xr_write_byte(card, *data++);
    }
}

static bool xr_receive_ready(SDState *card)
{
    XRadioState *s = XRADIO_SDIO(card);
    return s->powered && s->writing && s->position < s->length;
}

static bool xr_data_ready(SDState *card)
{
    XRadioState *s = XRADIO_SDIO(card);
    return s->powered && !s->writing && s->position < s->length;
}

static bool xr_inserted(SDState *card)
{
    return XRADIO_SDIO(card)->powered;
}

static bool xr_readonly(SDState *card)
{
    return false;
}

static void xr_voltage(SDState *card, uint16_t voltage)
{
}

static void xr_reset(DeviceState *dev)
{
    XRadioState *s = XRADIO_SDIO(dev);
    timer_del(s->beacon_timer);
    g_queue_clear_full(&s->rx, g_free);
    memset(s->cccr, 0, sizeof(s->cccr));
    memset(s->memory, 0, sizeof(s->memory));
    s->cccr[0] = 0x32;
    s->cccr[1] = 3;
    s->cccr[8] = 0x1e;
    stl_le_p(s->cccr + 9, 0x1000);
    s->cccr[0x13] = 1;
    s->cccr[0x100] = 7;
    stl_le_p(s->cccr + 0x109, 0x1100);
    stw_le_p(s->cccr + 0x110, 512);
    s->config = BIT(26) | BIT(14) | BIT(12) | XR_CONFIG_ACCESS;
    s->control = s->base = s->dpll = 0;
    s->position = s->length = 0;
    s->sequence = s->wsm_sequence = s->interface = 0;
    s->firmware = s->associated = false;
    memcpy(s->station, s->conf.macaddr.a, 6);
    xr_update_irq(s);
}

static void xr_power(void *opaque, int line, int level)
{
    XRadioState *s = opaque;
    if (s->powered != !!level) {
        s->powered = !!level;
        trace_xradio_power(s->powered);
        xr_reset(DEVICE(s));
        sdbus_set_inserted(SD_BUS(qdev_get_parent_bus(DEVICE(s))), s->powered);
    }
}

static bool xr_can_receive(NetClientState *nc)
{
    XRadioState *s = qemu_get_nic_opaque(nc);
    return s->powered && s->firmware && s->associated && !nc->link_down &&
           g_queue_get_length(&s->rx) < XR_RX_FRAMES;
}

static ssize_t xr_receive(NetClientState *nc, const uint8_t *data, size_t length)
{
    XRadioState *s = qemu_get_nic_opaque(nc);
    uint8_t frame[2048];
    unsigned pos;

    if (length < 14 || length + 18 > sizeof(frame)) {
        return length;
    }
    pos = xr_header(s, frame, 0x0208, data);
    memcpy(frame + 16, data + 6, 6);
    memcpy(frame + pos, "\xaa\xaa\x03\0\0\0", 6);
    memcpy(frame + pos + 6, data + 12, length - 12);
    xr_rx_frame(s, frame, pos + length - 6, s->interface);
    return length;
}

static void xr_link_changed(NetClientState *nc)
{
    XRadioState *s = qemu_get_nic_opaque(nc);
    uint8_t frame[26];
    if (nc->link_down && s->associated) {
        xr_header(s, frame, 0xc0, s->station);
        stw_le_p(frame + 24, 3);
        xr_rx_frame(s, frame, sizeof(frame), s->interface);
        s->associated = false;
        timer_del(s->beacon_timer);
    }
}

static NetClientInfo xr_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = xr_can_receive,
    .receive = xr_receive,
    .link_status_changed = xr_link_changed,
};

static void xr_realize(DeviceState *dev, Error **errp)
{
    XRadioState *s = XRADIO_SDIO(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&xr_net_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void xr_init(Object *obj)
{
    XRadioState *s = XRADIO_SDIO(obj);
    static const uint8_t common[] = {
        0x20, 4, 0x20, 0, 0x81, 0x22,
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
    stw_le_p(s->cis + 0x118, 2048);
    stw_le_p(s->cis + 0x128, 100);
    s->cis[0x136] = 0xff;
    g_queue_init(&s->rx);
    s->beacon_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xr_beacon_tick, s);
    qdev_init_gpio_in_named(DEVICE(s), xr_power, "power", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, "irq", 1);
}

static void xr_finalize(Object *obj)
{
    XRadioState *s = XRADIO_SDIO(obj);
    timer_free(s->beacon_timer);
    g_queue_clear_full(&s->rx, g_free);
    if (s->nic) {
        qemu_del_nic(s->nic);
    }
}

static const Property xr_properties[] = {
    DEFINE_NIC_PROPERTIES(XRadioState, conf),
};

static const VMStateDescription xr_vmstate = {
    .name = TYPE_XRADIO_SDIO,
    .unmigratable = 1,
};

static void xr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);
    dc->desc = "Allwinner XR819 SDIO Wi-Fi";
    dc->realize = xr_realize;
    dc->vmsd = &xr_vmstate;
    device_class_set_legacy_reset(dc, xr_reset);
    device_class_set_props(dc, xr_properties);
    sc->do_command = xr_command;
    sc->read_byte = xr_read_byte;
    sc->write_byte = xr_write_byte;
    sc->read_data = xr_read_data;
    sc->write_data = xr_write_data;
    sc->receive_ready = xr_receive_ready;
    sc->data_ready = xr_data_ready;
    sc->get_inserted = xr_inserted;
    sc->get_readonly = xr_readonly;
    sc->set_voltage = xr_voltage;
}

static const TypeInfo xr_type = {
    .name = TYPE_XRADIO_SDIO,
    .parent = TYPE_SDMMC_COMMON,
    .instance_size = sizeof(XRadioState),
    .instance_init = xr_init,
    .instance_finalize = xr_finalize,
    .class_init = xr_class_init,
};

static void xr_register_types(void)
{
    type_register_static(&xr_type);
}
type_init(xr_register_types)
