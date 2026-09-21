/*
 * USB CDC ECM host transport for the MediaTek device controller.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Enumerate the gadget over endpoint zero and exchange Ethernet frames using
 * USB packets. The controller owns all FIFO, DMA and interrupt behavior.
 */
#include "qemu/osdep.h"
#include "hw/usb/mtu3.h"
#include "hw/core/irq.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"

#define FRAME_SIZE 65536
#define CONTROL_SIZE 1024
#define WORK_LIMIT 256

struct MTU3EcmHost {
    MTU3State *controller;
    NICState *nic;
    QEMUTimer *timer;
    unsigned phase;
    bool attached, configured, failed, pending, status_out;
    uint8_t configuration, control_interface, data_interface, alternate;
    unsigned configuration_count, configuration_index;
    uint8_t in_ep, out_ep, notify_ep;
    unsigned in_max, out_max;
    uint8_t control[CONTROL_SIZE];
    unsigned control_length, requested;
    int64_t deadline;
    uint8_t tx[FRAME_SIZE], rx[FRAME_SIZE];
    size_t tx_length, rx_length, rx_offset;
    bool rx_pending;
};

static void mtu3_ecm_clear(MTU3EcmHost *h)
{
    h->phase = 0;
    h->attached = h->configured = h->failed = h->pending = false;
    h->status_out = h->rx_pending = false;
    h->tx_length = h->rx_length = h->rx_offset = 0;
    h->configuration_count = h->configuration_index = 0;
    h->in_ep = h->out_ep = h->notify_ep = 0;
    timer_del(h->timer);
    qemu_purge_queued_packets(qemu_get_queue(h->nic));
}

void mtu3_ecm_reset(MTU3State *s)
{
    if (s->host) {
        mtu3_ecm_clear(s->host);
        mtu3_ecm_kick(s);
    }
}

void mtu3_ecm_kick(MTU3State *s)
{
    if (s->host && !timer_pending(s->host->timer)) {
        timer_mod(s->host->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
    }
}

static bool mtu3_ecm_descriptors(MTU3EcmHost *h)
{
    unsigned interface = 0, alternate = 0, class = 0;
    bool ecm = false;

    if (h->control_length < 9 || h->control[1] != 2 ||
        lduw_le_p(h->control + 2) > h->control_length) {
        return false;
    }
    h->configuration = h->control[5];
    for (unsigned offset = 0; offset + 2 <= h->control_length;) {
        const uint8_t *d = h->control + offset;
        unsigned length = d[0];

        if (length < 2 || offset + length > h->control_length) {
            return false;
        }
        if (d[1] == 4 && length >= 9) {
            interface = d[2];
            alternate = d[3];
            class = d[5];
            if (class == 2 && d[6] == 6) {
                h->control_interface = interface;
                ecm = true;
            }
        } else if (d[1] == 5 && length >= 7) {
            unsigned ep = d[2] & 15;
            unsigned max = lduw_le_p(d + 4) & 0x7ff;

            if (!ep || ep > h->controller->endpoints || !max) {
                return false;
            }
            if (class == 0x0a && (d[3] & 3) == 2) {
                h->data_interface = interface;
                h->alternate = alternate;
                if (d[2] & 0x80) {
                    h->in_ep = ep;
                    h->in_max = max;
                } else {
                    h->out_ep = ep;
                    h->out_max = max;
                }
            } else if (class == 2 && (d[3] & 3) == 3 && (d[2] & 0x80)) {
                h->notify_ep = ep;
            }
        }
        offset += length;
    }
    return ecm && h->in_ep && h->out_ep;
}

static void mtu3_ecm_request(MTU3EcmHost *h, uint8_t type, uint8_t request,
                           uint16_t value, uint16_t index, uint16_t length)
{
    uint8_t setup[8] = {type, request};

    stw_le_p(setup + 2, value);
    stw_le_p(setup + 4, index);
    stw_le_p(setup + 6, length);
    if (mtu3_out_packet(h->controller, 0, setup, sizeof(setup), true) < 0) {
        return;
    }
    h->control_length = 0;
    h->requested = length;
    h->status_out = false;
    h->pending = true;
    h->deadline = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 5000;
}

static bool mtu3_ecm_control(MTU3EcmHost *h)
{
    uint8_t packet[MTU3_FIFO_SIZE];
    int count;

    if (h->status_out) {
        count = mtu3_out_packet(h->controller, 0, packet, 0, false);
    } else {
        count = mtu3_in_packet(h->controller, 0, packet, sizeof(packet));
    }
    if (count == -EAGAIN && qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) < h->deadline) {
        return false;
    }
    if (count < 0 || h->control_length + count > CONTROL_SIZE) {
        warn_report("mtu3: ECM control request failed in phase %u (%d)",
                    h->phase, count);
        h->failed = true;
        return false;
    }
    if (h->requested && !h->status_out) {
        memcpy(h->control + h->control_length, packet, count);
        h->control_length += count;
        if (count < 64 || h->control_length >= h->requested) {
            h->status_out = true;
        }
        return false;
    }
    h->pending = false;
    h->phase++;
    return true;
}

static bool mtu3_ecm_can_receive(NetClientState *nc)
{
    MTU3EcmHost *h = qemu_get_nic_opaque(nc);

    return h->configured && !h->rx_pending && !nc->link_down &&
           mtu3_connected(h->controller) &&
           h->controller->ep[false][h->out_ep].active;
}

static ssize_t mtu3_ecm_receive(NetClientState *nc, const uint8_t *buf,
                               size_t size)
{
    MTU3EcmHost *h = qemu_get_nic_opaque(nc);

    if (!mtu3_ecm_can_receive(nc)) {
        return 0;
    }
    if (size < 14 || size > sizeof(h->rx)) {
        return size;
    }
    memcpy(h->rx, buf, size);
    h->rx_length = size;
    h->rx_offset = 0;
    h->rx_pending = true;
    mtu3_ecm_kick(h->controller);
    return size;
}

static void mtu3_ecm_link_changed(NetClientState *nc)
{
    MTU3EcmHost *h = qemu_get_nic_opaque(nc);

    mtu3_ecm_clear(h);
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(h->controller), "vbus", 0),
                 !nc->link_down);
    mtu3_ecm_kick(h->controller);
}

static NetClientInfo mtu3_ecm_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = mtu3_ecm_can_receive,
    .receive = mtu3_ecm_receive,
    .link_status_changed = mtu3_ecm_link_changed,
};

static void mtu3_ecm_frames(MTU3EcmHost *h)
{
    MTU3State *s = h->controller;
    uint8_t packet[MTU3_FIFO_SIZE];
    unsigned work = 0;
    int count;

    /* Consume interrupt notifications separately; they are not Ethernet data. */
    if (h->notify_ep) {
        for (unsigned n = 0; n < 8; n++) {
            if (mtu3_in_packet(s, h->notify_ep, packet, sizeof(packet)) < 0) {
                break;
            }
        }
    }
    while (work++ < WORK_LIMIT) {
        count = mtu3_in_packet(s, h->in_ep, packet, sizeof(packet));
        if (count < 0) {
            break;
        }
        if (h->tx_length + count > sizeof(h->tx)) {
            h->tx_length = 0;
            break;
        }
        memcpy(h->tx + h->tx_length, packet, count);
        h->tx_length += count;
        if (count < h->in_max) {
            if (h->tx_length >= 14) {
                qemu_send_packet(qemu_get_queue(h->nic), h->tx, h->tx_length);
            }
            h->tx_length = 0;
        }
    }
    while (h->rx_pending && work++ < WORK_LIMIT) {
        size_t amount = MIN(h->out_max, h->rx_length - h->rx_offset);

        count = mtu3_out_packet(s, h->out_ep, h->rx + h->rx_offset,
                                amount, false);
        if (count == -EAGAIN) {
            break; /* Resume when the guest publishes another receive GPD. */
        }
        if (count < 0) {
            h->rx_pending = false;
            break;
        }
        h->rx_offset += count;
        if (amount < h->out_max) {
            h->rx_pending = false; /* Includes the ZLP after an exact multiple. */
        }
    }
    qemu_flush_queued_packets(qemu_get_queue(h->nic));
    /* A NAK needs a guest queue notification, not idle polling. */
    if (work >= WORK_LIMIT) {
        mtu3_ecm_kick(s);
    }
}

static void mtu3_ecm_run(void *opaque)
{
    MTU3EcmHost *h = opaque;
    MTU3State *s = h->controller;

    if (qemu_get_queue(h->nic)->link_down || !mtu3_connected(s)) {
        if (h->attached) {
            mtu3_ecm_clear(h);
        }
        return;
    }
    if (h->failed) {
        return;
    }
    if (!h->attached) {
        h->attached = true;
        mtu3_bus_reset(s);
    }
    if (h->configured) {
        mtu3_ecm_frames(h);
        return;
    }
    if (h->pending && !mtu3_ecm_control(h)) {
        goto wait;
    }
    /* Wait for reset, speed-change and EP0 interrupt setup. */
    if ((s->regs[0x241c / 4] & BIT(2)) || (s->regs[0x854 / 4] & BIT(0)) ||
        !(s->regs[0x084 / 4] & 1)) {
        goto wait;
    }
    switch (h->phase) {
    case 0: mtu3_ecm_request(h, 0x80, 6, 0x0100, 0, 18); break;
    case 1:
        h->configuration_count = h->control_length >= 18 ? h->control[17] : 0;
        mtu3_ecm_request(h, 0x00, 5, 1, 0, 0);
        break;
    case 2:
        mtu3_ecm_request(h, 0x80, 6, 0x0200 | h->configuration_index,
                         0, CONTROL_SIZE);
        break;
    case 3:
        if (!mtu3_ecm_descriptors(h)) {
            if (++h->configuration_index < h->configuration_count) {
                h->phase = 2;
                goto wait;
            }
            warn_report("mtu3: connected USB gadget has no CDC ECM interface");
            h->failed = true;
            return;
        }
        mtu3_ecm_request(h, 0x00, 9, h->configuration, 0, 0);
        break;
    case 4:
        mtu3_ecm_request(h, 0x01, 11, h->alternate, h->data_interface, 0);
        break;
    case 5:
        mtu3_ecm_request(h, 0x21, 0x43, 0x0f, h->control_interface, 0);
        break;
    default:
        h->configured = true;
        mtu3_ecm_frames(h);
        return;
    }
wait:
    if (!h->failed) {
        timer_mod(h->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
    }
}

void mtu3_ecm_realize(MTU3State *s)
{
    MTU3EcmHost *h;
    DeviceState *dev = DEVICE(s);

    if (!s->nic_conf.peers.ncs[0]) {
        return;
    }
    h = s->host = g_new0(MTU3EcmHost, 1);
    h->controller = s;
    h->nic = qemu_new_nic(&mtu3_ecm_net_info, &s->nic_conf,
                         object_get_typename(OBJECT(s)), dev->id,
                         &dev->mem_reentrancy_guard, h);
    qemu_format_nic_info_str(qemu_get_queue(h->nic), s->nic_conf.macaddr.a);
    h->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, mtu3_ecm_run, h);
    mtu3_ecm_link_changed(qemu_get_queue(h->nic));
}

void mtu3_ecm_unrealize(MTU3State *s)
{
    if (s->host) {
        timer_free(s->host->timer);
        qemu_del_nic(s->host->nic);
        g_free(s->host);
        s->host = NULL;
    }
}
