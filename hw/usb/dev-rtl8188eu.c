/*
 * RTL8188EU USB WLAN, as used by PocketBook readers.
 *
 * The vendor driver runs its own MLME and uploads its embedded MCU firmware.
 * Model the USB register/descriptor interface and an open 802.11 access point;
 * Ethernet payloads pass through the configured QEMU network backend.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "desc.h"
#include "trace.h"

#define TYPE_RTL8188EU "rtl8188eu"
OBJECT_DECLARE_SIMPLE_TYPE(RTL8188EUState, RTL8188EU)

#define RTL_REG_SIZE 0x10000
#define RTL_RX_SLOTS 64
#define RTL_FRAME_SIZE 4096
#define RTL_TX_SIZE 32768
#define RTL_RX_DESC 24
#define RTL_TX_DESC 32
#define RTL_BEACON_MS 100

#define REG_APS_FSMCO 0x0004
#define REG_EFUSE_CTRL 0x0030
#define REG_9346CR 0x000a
#define REG_MCUFWDL 0x0080
#define REG_HMEBOX_E0 0x0088
#define REG_SYS_CFG 0x00f0
#define REG_PKTBUF_ACCESS 0x0106
#define REG_PKTBUF_DBG_ADDR 0x0140
#define REG_PKTBUF_DBG_CTRL 0x0143
#define REG_PKTBUF_DBG_DATA 0x0144
#define REG_CR 0x0100
#define REG_HMETFR 0x01cc
#define REG_LLT_INIT 0x01e0
#define REG_MACID 0x0610
#define REG_CAMCMD 0x0670
#define REG_RF_LSSI 0x0840

static const uint8_t rtl_bssid[6] = { 0x02, 0, 0, 0, 0, 0x02 };
static const char rtl_ssid[] = "Kindle-QEMU";
static const uint8_t rtl_rates[] = {
    0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,
    0x30, 0x48, 0x60, 0x6c,
};
static const uint8_t rtl_snap[6] = { 0xaa, 0xaa, 3, 0, 0, 0 };

struct RTL8188EUState {
    USBDevice parent_obj;
    NICConf conf;
    NICState *nic;
    QEMUTimer *beacon_timer;
    uint8_t regs[RTL_REG_SIZE];
    uint8_t efuse[512];
    uint32_t rf[256];
    uint8_t llt[256];
    uint8_t rx[RTL_RX_SLOTS][RTL_FRAME_SIZE];
    uint16_t rx_len[RTL_RX_SLOTS];
    unsigned rx_head, rx_count, rx_offset;
    uint8_t tx[2][RTL_TX_SIZE];
    uint8_t packet_ram[RTL_TX_SIZE];
    unsigned tx_len[2];
    uint16_t sequence;
    uint8_t channel;
    bool powered;
    bool associated;
};

static const USBDescStrings rtl_strings = {
    [1] = "Realtek",
    [2] = "RTL8188EU WLAN Adapter",
    [3] = "000000000001",
};

static const USBDescIface rtl_iface = {
    .bInterfaceNumber = 0,
    .bNumEndpoints = 4,
    .bInterfaceClass = 0xff,
    .bInterfaceSubClass = 0xff,
    .bInterfaceProtocol = 0xff,
    .eps = (USBDescEndpoint[]) {
        { .bEndpointAddress = 0x81, .bmAttributes = USB_ENDPOINT_XFER_BULK,
          .wMaxPacketSize = 512 },
        { .bEndpointAddress = 0x02, .bmAttributes = USB_ENDPOINT_XFER_BULK,
          .wMaxPacketSize = 512 },
        { .bEndpointAddress = 0x03, .bmAttributes = USB_ENDPOINT_XFER_BULK,
          .wMaxPacketSize = 512 },
        { .bEndpointAddress = 0x84, .bmAttributes = USB_ENDPOINT_XFER_INT,
          .wMaxPacketSize = 64, .bInterval = 1 },
    },
};

static const USBDescDevice rtl_device = {
    .bcdUSB = 0x0200,
    .bMaxPacketSize0 = 64,
    .bNumConfigurations = 1,
    .confs = (USBDescConfig[]) {
        { .bNumInterfaces = 1, .bConfigurationValue = 1,
          .bmAttributes = USB_CFG_ATT_ONE, .bMaxPower = 250,
          .nif = 1, .ifs = &rtl_iface },
    },
};

static const USBDescIface rtl_full_iface = {
    .bInterfaceNumber = 0,
    .bNumEndpoints = 4,
    .bInterfaceClass = 0xff,
    .bInterfaceSubClass = 0xff,
    .bInterfaceProtocol = 0xff,
    .eps = (USBDescEndpoint[]) {
        { .bEndpointAddress = 0x81, .bmAttributes = USB_ENDPOINT_XFER_BULK,
          .wMaxPacketSize = 64 },
        { .bEndpointAddress = 0x02, .bmAttributes = USB_ENDPOINT_XFER_BULK,
          .wMaxPacketSize = 64 },
        { .bEndpointAddress = 0x03, .bmAttributes = USB_ENDPOINT_XFER_BULK,
          .wMaxPacketSize = 64 },
        { .bEndpointAddress = 0x84, .bmAttributes = USB_ENDPOINT_XFER_INT,
          .wMaxPacketSize = 64, .bInterval = 1 },
    },
};

static const USBDescDevice rtl_full_device = {
    .bcdUSB = 0x0200,
    .bMaxPacketSize0 = 64,
    .bNumConfigurations = 1,
    .confs = (USBDescConfig[]) {
        { .bNumInterfaces = 1, .bConfigurationValue = 1,
          .bmAttributes = USB_CFG_ATT_ONE, .bMaxPower = 250,
          .nif = 1, .ifs = &rtl_full_iface },
    },
};

static const USBDesc rtl_desc = {
    .id = { .idVendor = 0x0bda, .idProduct = 0x8179, .bcdDevice = 0x0000,
            .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3 },
    .high = &rtl_device,
    .full = &rtl_full_device,
    .str = rtl_strings,
};

static bool rtl_receiving(RTL8188EUState *s)
{
    return s->powered && (s->regs[REG_MCUFWDL] & BIT(6)) &&
        (s->regs[REG_CR] & BIT(7));
}

static bool rtl_backend_up(RTL8188EUState *s)
{
    return !qemu_get_queue(s->nic)->link_down;
}

static void rtl_queue(RTL8188EUState *s, const uint8_t *frame, unsigned length)
{
    unsigned slot;
    uint8_t *rx;

    if (!rtl_receiving(s) || length + RTL_RX_DESC > RTL_FRAME_SIZE ||
        s->rx_count == RTL_RX_SLOTS) {
        return;
    }
    slot = (s->rx_head + s->rx_count++) % RTL_RX_SLOTS;
    rx = s->rx[slot];
    memset(rx, 0, RTL_RX_DESC);
    /* One normal MPDU, without appended FCS or PHY status. */
    stl_le_p(rx, length);
    stl_le_p(rx + 8, BIT(16) | ((s->sequence - 1) & 0xfff));
    stl_le_p(rx + 12, 11); /* OFDM 54 Mbit/s. */
    memcpy(rx + RTL_RX_DESC, frame, length);
    s->rx_len[slot] = RTL_RX_DESC + length;
    trace_rtl8188eu_frame(false, length, lduw_le_p(frame));
    usb_wakeup(usb_ep_get(USB_DEVICE(s), USB_TOKEN_IN, 1), 0);
}

static void rtl_header(RTL8188EUState *s, uint8_t *frame, uint16_t control,
                       const uint8_t *destination, const uint8_t *source)
{
    memset(frame, 0, 24);
    stw_le_p(frame, control);
    memcpy(frame + 4, destination, 6);
    memcpy(frame + 10, rtl_bssid, 6);
    memcpy(frame + 16, source, 6);
    stw_le_p(frame + 22, (s->sequence++ & 0xfff) << 4);
}

static unsigned rtl_ies(uint8_t *data)
{
    unsigned len = 0;

    data[len++] = 0; /* SSID */
    data[len++] = sizeof(rtl_ssid) - 1;
    memcpy(data + len, rtl_ssid, sizeof(rtl_ssid) - 1);
    len += sizeof(rtl_ssid) - 1;
    data[len++] = 1; /* Supported rates */
    data[len++] = 8;
    memcpy(data + len, rtl_rates, 8);
    len += 8;
    data[len++] = 3; /* DS parameter set */
    data[len++] = 1;
    data[len++] = 1;
    data[len++] = 50; /* Extended rates */
    data[len++] = 4;
    memcpy(data + len, rtl_rates + 8, 4);
    len += 4;
    return len;
}

static void rtl_beacon(RTL8188EUState *s, const uint8_t *destination,
                       bool probe)
{
    uint8_t frame[128];
    unsigned len = 36;

    if (s->channel != 1 || !rtl_backend_up(s) ||
        s->rx_count >= RTL_RX_SLOTS - 8) {
        return;
    }
    rtl_header(s, frame, probe ? 0x0050 : 0x0080, destination, rtl_bssid);
    stq_le_p(frame + 24, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    stw_le_p(frame + 32, 100);
    stw_le_p(frame + 34, 0x0001); /* ESS, open authentication. */
    len += rtl_ies(frame + len);
    /* TIM: no buffered unicast or multicast traffic. */
    memcpy(frame + len, (uint8_t[]) { 5, 4, 0, 1, 0, 0 }, 6);
    rtl_queue(s, frame, len + 6);
}

static void rtl_beacon_tick(void *opaque)
{
    RTL8188EUState *s = opaque;
    static const uint8_t broadcast[6] = { 255, 255, 255, 255, 255, 255 };

    rtl_beacon(s, broadcast, false);
    timer_mod(s->beacon_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              RTL_BEACON_MS * 1000000LL);
}

static void rtl_transmit(RTL8188EUState *s, const uint8_t *frame, unsigned len)
{
    uint16_t fc;
    uint8_t response[128];
    unsigned header;

    if (len < 24) {
        return;
    }
    fc = lduw_le_p(frame);
    trace_rtl8188eu_frame(true, len, fc);
    if (!rtl_backend_up(s)) {
        return;
    }
    switch (fc & 0x00fc) {
    case 0x0040: /* Probe request. */
        rtl_beacon(s, frame + 10, true);
        break;
    case 0x00b0: /* Open-system authentication. */
        if (len < 30 || memcmp(frame + 4, rtl_bssid, 6) ||
            lduw_le_p(frame + 24) != 0) {
            break;
        }
        rtl_header(s, response, 0x00b0, frame + 10, rtl_bssid);
        stw_le_p(response + 24, 0);
        stw_le_p(response + 26, 2);
        stw_le_p(response + 28, 0);
        rtl_queue(s, response, 30);
        break;
    case 0x0000: /* Association request. */
    case 0x0020: /* Reassociation request. */
        if (memcmp(frame + 4, rtl_bssid, 6)) {
            break;
        }
        rtl_header(s, response, (fc & 0x20) ? 0x0030 : 0x0010,
                   frame + 10, rtl_bssid);
        stw_le_p(response + 24, 1);
        stw_le_p(response + 26, 0);
        stw_le_p(response + 28, 0xc001);
        response[30] = 1;
        response[31] = 8;
        memcpy(response + 32, rtl_rates, 8);
        rtl_queue(s, response, 40);
        s->associated = true;
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        break;
    case 0x00a0: /* Disassociation. */
    case 0x00c0: /* Deauthentication. */
        s->associated = false;
        break;
    case 0x0008: /* Data. */
    case 0x0088: /* QoS data. */
        header = (fc & 0x0080) ? 26 : 24;
        if (!s->associated || (fc & 0x4300) != 0x0100 ||
            len < header + 8 || memcmp(frame + header, rtl_snap, 6)) {
            break;
        }
        /* Station -> distribution system: addr3 destination, addr2 source. */
        {
            g_autofree uint8_t *ethernet = g_malloc(len - header + 6);
            memcpy(ethernet, frame + 16, 6);
            memcpy(ethernet + 6, frame + 10, 6);
            memcpy(ethernet + 12, frame + header + 6, len - header - 6);
            qemu_send_packet(qemu_get_queue(s->nic), ethernet,
                             len - header + 6);
        }
        break;
    }
}

static ssize_t rtl_receive(NetClientState *nc, const uint8_t *buf, size_t len)
{
    RTL8188EUState *s = qemu_get_nic_opaque(nc);
    uint8_t frame[RTL_FRAME_SIZE];

    if (!s->associated || !rtl_receiving(s) || !rtl_backend_up(s)) {
        return len;
    }
    if (len < 14 || len + 18 > sizeof(frame)) {
        return len;
    }
    if (s->rx_count == RTL_RX_SLOTS) {
        return 0;
    }
    /* Distribution system -> station: addr1 destination, addr3 source. */
    rtl_header(s, frame, 0x0208, buf, buf + 6);
    memcpy(frame + 24, rtl_snap, 6);
    memcpy(frame + 30, buf + 12, len - 12);
    rtl_queue(s, frame, len + 18);
    return len;
}

static bool rtl_can_receive(NetClientState *nc)
{
    RTL8188EUState *s = qemu_get_nic_opaque(nc);

    return s->associated && rtl_receiving(s) && rtl_backend_up(s) &&
        s->rx_count < RTL_RX_SLOTS - 8;
}

static void rtl_link_changed(NetClientState *nc)
{
    RTL8188EUState *s = qemu_get_nic_opaque(nc);
    uint8_t frame[26];

    if (!rtl_backend_up(s) && s->associated) {
        s->rx_head = s->rx_count = s->rx_offset = 0;
        rtl_header(s, frame, 0x00c0, s->regs + REG_MACID, rtl_bssid);
        stw_le_p(frame + 24, 3); /* AP leaving the network. */
        rtl_queue(s, frame, sizeof(frame));
        s->associated = false;
    }
}

static uint8_t rtl_read_byte(RTL8188EUState *s, unsigned address)
{
    if (address == REG_APS_FSMCO + 2) {
        return s->regs[address] | BIT(1); /* Power ready. */
    }
    if (address == REG_HMETFR) {
        return 0; /* Firmware consumes all host mailbox commands. */
    }
    return s->regs[address];
}

static void rtl_write_byte(RTL8188EUState *s, unsigned address, uint8_t value);

static bool rtl_iol_configure(RTL8188EUState *s)
{
    unsigned offset = s->regs[0x209] * 128 + RTL_TX_DESC;
    while (offset + 4 <= sizeof(s->packet_ram)) {
        uint8_t *command = s->packet_ram + offset;
        unsigned length = command[0], opcode = command[1];
        unsigned address = lduw_le_p(command + 2), width;
        uint32_t value, mask, old;

        if (length < 4 || offset + length > sizeof(s->packet_ram)) {
            return false;
        }
        if (opcode == 0xff) {
            return length == 4;
        }
        if (opcode == 0x10 && length == 4) {
            /* Physical RF/BB settling delay; modeled registers settle at once. */
            offset += length;
            continue;
        }
        if (opcode < 4 || opcode > 7 || (length != 8 && length != 12)) {
            return false;
        }
        value = ldl_le_p(command + 4);
        width = opcode == 4 ? 1 : opcode == 5 ? 2 : 4;
        mask = length == 12 ? ldl_le_p(command + 8) :
               opcode == 4 ? 0xff : opcode == 5 ? 0xffff :
               opcode == 7 ? 0xfffff : 0xffffffff;
        if (opcode == 7) {
            if (address >> 8) {
                return false; /* RTL8188EU has one RF path. */
            }
            old = s->rf[address];
            s->rf[address] = (old & ~mask) | (value & mask);
            if (address == 0x18) {
                s->channel = s->rf[address] & 0x3ff;
            }
        } else {
            if (address + width > sizeof(s->regs)) {
                return false;
            }
            if (address <= REG_HMEBOX_E0 && address + width > REG_HMEBOX_E0) {
                return false; /* A script cannot recursively submit itself. */
            }
            for (unsigned i = 0; i < width; i++) {
                uint8_t byte_mask = mask >> (i * 8);
                rtl_write_byte(s, address + i,
                    (s->regs[address + i] & ~byte_mask) |
                    ((value >> (i * 8)) & byte_mask));
            }
        }
        offset += length;
    }
    return false;
}

static void rtl_iol_request(RTL8188EUState *s, uint8_t request)
{
    uint8_t commands = request & 0x0f, errors = 0;
    if (commands & BIT(0)) {
        errors |= BIT(0); /* Native drivers initialize LLT through its registers. */
    }
    if (commands & BIT(1)) {
        unsigned base = s->regs[0x209] * 128;
        if (base + 2 + sizeof(s->efuse) <= sizeof(s->packet_ram)) {
            stw_le_p(s->packet_ram + base, sizeof(s->efuse) + 2);
            memcpy(s->packet_ram + base + 2, s->efuse, sizeof(s->efuse));
        } else {
            errors |= BIT(1);
        }
    }
    /* EFUSE_PATCH applies the same virtual efuse already used by register IO. */
    if ((commands & BIT(3)) && !rtl_iol_configure(s)) {
        errors |= BIT(3);
    }
    s->regs[REG_HMEBOX_E0] = (request & ~(commands | (commands << 4))) |
                              (errors << 4);
}

static void rtl_write_byte(RTL8188EUState *s, unsigned address, uint8_t value)
{
    s->regs[address] = value;
    switch (address) {
    case REG_HMEBOX_E0:
        if ((s->regs[REG_SYS_CFG] & BIT(7)) && (s->regs[3] & BIT(2))) {
            rtl_iol_request(s, value);
        }
        break;
    case REG_PKTBUF_DBG_CTRL:
        if (s->regs[REG_PKTBUF_ACCESS] == 0x69 && !value) {
            unsigned offset = lduw_le_p(s->regs + REG_PKTBUF_DBG_ADDR) * 8;
            if (offset + 8 <= sizeof(s->packet_ram)) {
                memcpy(s->regs + REG_PKTBUF_DBG_DATA, s->packet_ram + offset, 8);
                s->regs[REG_PKTBUF_DBG_CTRL] = 1;
            }
        }
        break;
    case REG_APS_FSMCO + 1:
        s->regs[address] &= ~3; /* MAC power-on/off requests self-clear. */
        break;
    case REG_MCUFWDL:
        if (value & BIT(1)) {
            s->regs[address] |= BIT(6) | BIT(7); /* MCU initialized, RAM firmware. */
        }
        s->regs[address] |= BIT(2); /* Download checksum complete. */
        break;
    case REG_EFUSE_CTRL + 3:
        if (!(value & BIT(7))) {
            unsigned offset = lduw_le_p(s->regs + REG_EFUSE_CTRL + 1) & 0x3ff;
            s->regs[REG_EFUSE_CTRL] = offset < sizeof(s->efuse) ?
                s->efuse[offset] : 0xff;
            s->regs[address] |= BIT(7);
        }
        break;
    case REG_LLT_INIT + 3:
        {
            uint32_t command = ldl_le_p(s->regs + REG_LLT_INIT);
            unsigned index = (command >> 8) & 0xff;
            if ((command >> 30) == 1) {
                s->llt[index] = command;
            } else if ((command >> 30) == 2) {
                s->regs[REG_LLT_INIT] = s->llt[index];
            }
            s->regs[address] &= 0x3f;
        }
        break;
    case REG_CAMCMD + 3:
        s->regs[address] &= ~BIT(7); /* Security CAM operation complete. */
        break;
    case REG_RF_LSSI + 3:
        {
            uint32_t command = ldl_le_p(s->regs + REG_RF_LSSI);
            unsigned index = (command >> 20) & 0xff;
            s->rf[index] = command & 0xfffff;
            if (index == 0x18) {
                s->channel = command & 0x3ff;
            }
        }
        break;
    case 0x0827: /* RF serial read address, path A. */
        {
            unsigned index = (ldl_le_p(s->regs + 0x824) >> 23) & 0xff;
            stl_le_p(s->regs + 0x8a0, s->rf[index]);
            stl_le_p(s->regs + 0x8b8, s->rf[index]);
        }
        break;
    }
}

static void rtl_control(USBDevice *dev, USBPacket *packet, int request,
                        int value, int index, int length, uint8_t *data)
{
    RTL8188EUState *s = RTL8188EU(dev);
    int result = usb_desc_handle_control(dev, packet, request, value,
                                          index, length, data);
    bool read = request & (USB_DIR_IN << 8);
    unsigned address = value & 0xffff;
    uint32_t trace_value = 0;
    int i;

    if (result >= 0) {
        return;
    }
    if ((request & 0x7fff) != (VendorDeviceOutRequest | 5) ||
        length < 0 || length > RTL_REG_SIZE - address) {
        packet->status = USB_RET_STALL;
        return;
    }
    for (i = 0; i < length; i++) {
        if (read) {
            data[i] = rtl_read_byte(s, address + i);
        } else {
            rtl_write_byte(s, address + i, data[i]);
        }
        if (i < 4) {
            trace_value |= (uint32_t)data[i] << (8 * i);
        }
    }
    packet->actual_length = length;
    trace_rtl8188eu_register(read, address, length, trace_value);
}

static void rtl_data(USBDevice *dev, USBPacket *packet)
{
    RTL8188EUState *s = RTL8188EU(dev);
    unsigned len, slot;
    unsigned packet_size = packet->ep->max_packet_size;

    if (packet->pid == USB_TOKEN_IN && packet->ep->nr == 1) {
        if (!s->rx_count) {
            packet->status = USB_RET_NAK;
            return;
        }
        slot = s->rx_head;
        len = MIN(packet->iov.size, s->rx_len[slot] - s->rx_offset);
        usb_packet_copy(packet, s->rx[slot] + s->rx_offset, len);
        s->rx_offset += len;
        if (s->rx_offset == s->rx_len[slot] &&
            (len < packet->iov.size || s->rx_len[slot] % packet_size)) {
            s->rx_offset = 0;
            s->rx_head = (slot + 1) % RTL_RX_SLOTS;
            s->rx_count--;
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
    } else if (packet->pid == USB_TOKEN_OUT &&
               (packet->ep->nr == 2 || packet->ep->nr == 3)) {
        unsigned pipe = packet->ep->nr - 2;
        uint8_t *tx = s->tx[pipe];
        if (packet->iov.size > RTL_TX_SIZE - s->tx_len[pipe]) {
            s->tx_len[pipe] = 0;
            packet->status = USB_RET_STALL;
            return;
        }
        usb_packet_copy(packet, tx + s->tx_len[pipe], packet->iov.size);
        s->tx_len[pipe] += packet->iov.size;
        while (s->tx_len[pipe] >= RTL_TX_DESC) {
            unsigned payload = lduw_le_p(tx);
            unsigned offset = tx[2] + (((ldl_le_p(tx + 4) >> 26) & 31) * 8);
            unsigned total = offset + payload;
            if (offset < RTL_TX_DESC || !payload || total > RTL_TX_SIZE) {
                s->tx_len[pipe] = 0;
                break;
            }
            if (total > s->tx_len[pipe]) {
                break;
            }
            if (((ldl_le_p(tx + 4) >> 8) & 0x1f) == 0x10 &&
                payload <= sizeof(s->packet_ram) - RTL_TX_DESC) {
                /* The beacon queue is also the MCU's I/O-offload carrier. */
                memcpy(s->packet_ram, tx, RTL_TX_DESC);
                memcpy(s->packet_ram + RTL_TX_DESC, tx + offset, payload);
            } else {
                rtl_transmit(s, tx + offset, payload);
            }
            if (total < s->tx_len[pipe]) {
                total = MIN((total + 7) & ~7, s->tx_len[pipe]);
            }
            s->tx_len[pipe] -= total;
            memmove(tx, tx + total, s->tx_len[pipe]);
        }
        if (packet->iov.size % packet_size) {
            s->tx_len[pipe] = 0;
        }
    } else if (packet->pid == USB_TOKEN_IN && packet->ep->nr == 4) {
        packet->status = USB_RET_NAK;
    } else {
        packet->status = USB_RET_STALL;
    }
}

static void rtl_reset(USBDevice *dev)
{
    RTL8188EUState *s = RTL8188EU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->packet_ram, 0, sizeof(s->packet_ram));
    s->regs[REG_APS_FSMCO + 2] = BIT(1);
    s->regs[0x68] = BIT(2); /* WLAN function present. */
    s->regs[REG_9346CR] = BIT(5); /* Boot from programmed efuse. */
    memcpy(s->regs + REG_MACID, s->conf.macaddr.a, 6);
    s->rx_head = s->rx_count = s->rx_offset = 0;
    s->tx_len[0] = s->tx_len[1] = 0;
    s->sequence = 0;
    s->channel = 1;
    s->rf[0x18] = 1;
    s->associated = false;
}

static void rtl_power(void *opaque, int line, int level)
{
    RTL8188EUState *s = opaque;
    USBDevice *dev = USB_DEVICE(s);

    if (s->powered == !!level) {
        return;
    }
    s->powered = !!level;
    trace_rtl8188eu_power(s->powered);
    if (s->powered) {
        rtl_reset(dev);
        usb_device_attach(dev, &error_abort);
        timer_mod(s->beacon_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  RTL_BEACON_MS * 1000000LL);
    } else if (dev->attached) {
        timer_del(s->beacon_timer);
        usb_device_detach(dev);
        rtl_reset(dev);
    }
}

static NetClientInfo rtl_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = rtl_receive,
    .can_receive = rtl_can_receive,
    .link_status_changed = rtl_link_changed,
};

static void rtl_realize(USBDevice *dev, Error **errp)
{
    RTL8188EUState *s = RTL8188EU(dev);
    uint8_t map[512];
    unsigned section, index = 0;

    dev->auto_attach = false;
    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&rtl_net_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->qdev.id,
                          &dev->qdev.mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
    memset(map, 0xff, sizeof(map));
    stw_le_p(map, 0x8129);
    stw_le_p(map + 0xd0, 0x0bda);
    stw_le_p(map + 0xd2, 0x8179);
    memcpy(map + 0xd7, s->conf.macaddr.a, 6);
    map[0xb8] = 0x20; /* Worldwide channel plan. */
    map[0xc1] = 0; /* EEPROM revision. */
    memset(s->efuse, 0xff, sizeof(s->efuse));
    /* Physical efuse contains 8-byte sections with active-low word masks. */
    for (section = 0; section < sizeof(map) / 8; section++) {
        bool present = false;
        unsigned i;
        for (i = 0; i < 8; i++) {
            present |= map[section * 8 + i] != 0xff;
        }
        if (!present) {
            continue;
        }
        if (section < 16) {
            s->efuse[index++] = section << 4;
        } else {
            s->efuse[index++] = ((section & 7) << 5) | 0x0f;
            s->efuse[index++] = (section & 0x78) << 1;
        }
        memcpy(s->efuse + index, map + section * 8, 8);
        index += 8;
    }
    rtl_reset(dev);
    s->beacon_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rtl_beacon_tick, s);
}

static void rtl_unrealize(USBDevice *dev)
{
    RTL8188EUState *s = RTL8188EU(dev);
    timer_free(s->beacon_timer);
    qemu_del_nic(s->nic);
}

static void rtl_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), rtl_power, "power", 1);
}

static const VMStateDescription rtl_vmstate = {
    .name = TYPE_RTL8188EU,
    .unmigratable = 1,
};

static const Property rtl_properties[] = {
    DEFINE_NIC_PROPERTIES(RTL8188EUState, conf),
};

static void rtl_class_init(ObjectClass *klass, const void *data)
{
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    uc->realize = rtl_realize;
    uc->unrealize = rtl_unrealize;
    uc->product_desc = "Realtek RTL8188EU WLAN";
    uc->usb_desc = &rtl_desc;
    uc->handle_reset = rtl_reset;
    uc->handle_control = rtl_control;
    uc->handle_data = rtl_data;
    dc->vmsd = &rtl_vmstate;
    device_class_set_props(dc, rtl_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo rtl_type = {
    .name = TYPE_RTL8188EU,
    .parent = TYPE_USB_DEVICE,
    .instance_size = sizeof(RTL8188EUState),
    .instance_init = rtl_init,
    .class_init = rtl_class_init,
};

static void rtl_register_types(void)
{
    type_register_static(&rtl_type);
}

type_init(rtl_register_types)
