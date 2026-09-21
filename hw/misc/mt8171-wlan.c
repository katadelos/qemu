/*
 * MT8171 / CONNAC SoC2 1x1 WLAN host bootstrap.
 *
 * Contract: original wlan_drv_gen4m.ko's soc2_1x1_bus_info/map/eco_table,
 * asicPdmaConfig, halWpdmaInit{Tx,Rx}Ring, halWpdmaWriteCmd,
 * kalDevPortRead, asicFillInitCmdTxd and wlanSendInitSetQueryCmdImpl.
 * The released GPL tree omits gen4m_mt8115; see the local module audit.
 *
 * This is a bounded ROM command/download transport, not execution of WLAN
 * firmware. It performs real 36-bit physical descriptor/payload DMA and
 * retains downloaded bytes, including ciphertext, in download staging.
 * An explicitly virtual MCU protocol implements selected runtime services;
 * it does not execute or decrypt the original firmware. Its open virtual AP
 * implements scan, authentication, association and Ethernet transport through
 * the stock driver's native DMA queues. Unknown commands remain outstanding.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/misc/mt8171-wlan.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "system/address-spaces.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"

#define R(s, a) ((s)->regs[(a) / 4])
#define HIF_RESET 0x100
#define SLP_PROT 0x154
#define INT_STATUS 0x200
#define INT_MASK 0x204
#define DMA_CONFIG 0x208
#define RESET_INDEX 0x20c
#define TX_CMD 0x330  /* hardware ring3, logical command ring7 */
#define TX_CMD_EXT 0x51c
#define TX_RUNTIME 0x3f0 /* hardware15, logical runtime command ring6 */
#define TX_RUNTIME_EXT 0x518
#define RX_EVENT 0x410 /* event receive ring1 */
#define RX_EVENT_EXT 0x584
#define DESC_DONE BIT(31)
#define DESC_LAST BIT(30)
#define EVENT_SIZE 512
#define RX_DATA 0x400
#define RX_DATA_EXT 0x580

static const uint8_t ap_mac[6] = {0x02, 0x81, 0x71, 0, 0, 1};

static bool ready(MT8171WlanState *s)
{
    return !s->reset_held && mt8171_consys_boot_ready(s->consys);
}

static void update_irq(MT8171WlanState *s)
{
    qemu_set_irq(s->irq, ready(s) &&
                 (R(s, INT_STATUS) & R(s, INT_MASK)) != 0);
}

static bool dma(MT8171WlanState *s, hwaddr address, void *data,
                unsigned length, bool write)
{
    if (!ready(s) || address >= (1ULL << 36) ||
        length > (1ULL << 36) - address ||
        address_space_rw(&address_space_memory, address,
                         MEMTXATTRS_UNSPECIFIED, data, length, write) != MEMTX_OK) {
        s->dma_faults++;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt8171-wlan: %s DMA failed address=%" HWADDR_PRIx
                      " length=%u\n", write ? "write" : "read", address, length);
        return false;
    }
    return true;
}

/* The MCU delivers events and 802.11 frames through the host's real RX
 * descriptors. Keep ownership until a complete buffer can be written. */
static bool queue_rx(MT8171WlanState *s, const uint8_t *packet,
                     unsigned length, unsigned ring)
{
    unsigned slot = (s->rx_head + s->rx_count) % ARRAY_SIZE(s->rx_queue);
    if (s->rx_count == ARRAY_SIZE(s->rx_queue) ||
        length > sizeof(s->rx_queue[0])) {
        return false;
    }
    memcpy(s->rx_queue[slot], packet, length);
    s->rx_length[slot] = length;
    s->rx_ring[slot] = ring;
    s->rx_count++;
    timer_mod(s->dma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    return true;
}

static void deliver_rx(MT8171WlanState *s)
{
    while (s->rx_count && ready(s) && (R(s, DMA_CONFIG) & 4) &&
           !(R(s, SLP_PROT) & 1)) {
        unsigned slot = s->rx_head, ring = s->rx_ring[slot];
        unsigned reg = RX_DATA + ring * 16;
        unsigned count = R(s, reg + 4) & 0xfff;
        unsigned index = R(s, reg + 12) & 0xfff;
        unsigned length = s->rx_length[slot];
        uint8_t d[16];
        uint32_t control;
        hwaddr desc, buffer;
        if (!count || index >= count || index == (R(s, reg + 8) & 0xfff)) {
            break;
        }
        desc = (R(s, reg) | (uint64_t)(R(s, RX_DATA_EXT + ring * 4) & 15)
                << 32) + index * 16;
        if (!dma(s, desc, d, sizeof(d), false)) {
            break;
        }
        control = ldl_le_p(d + 4);
        if ((control & DESC_DONE) || extract32(control, 16, 14) < length) {
            break;
        }
        buffer = ldl_le_p(d) | (uint64_t)(ldl_le_p(d + 8) & 15) << 32;
        if (!dma(s, buffer, s->rx_queue[slot], length, true)) {
            break;
        }
        stl_le_p(d + 4, deposit32(control | DESC_DONE | DESC_LAST,
                                  16, 14, length));
        if (!dma(s, desc, d, sizeof(d), true)) {
            break;
        }
        R(s, reg + 12) = (index + 1) % count;
        R(s, INT_STATUS) |= BIT(ring);
        s->rx_head = (slot + 1) % ARRAY_SIZE(s->rx_queue);
        s->rx_count--;
        s->rx_packets += ring == 0;
    }
    update_irq(s);
}

static void runtime_event(MT8171WlanState *s, uint8_t id,
                          const void *payload, unsigned length)
{
    uint8_t event[EVENT_SIZE] = {0};
    assert(length + 32 <= sizeof(event));
    stl_le_p(event, (length + 32) | 0xe0000000U);
    stw_le_p(event + 20, length + 12);
    stw_le_p(event + 22, 0xe000);
    event[24] = id;
    memcpy(event + 32, payload, length);
    queue_rx(s, event, length + 32, 1);
}

static void management_rx(MT8171WlanState *s, uint16_t fc,
                          const uint8_t *destination,
                          const uint8_t *body, unsigned length)
{
    /* RXD20, group2 timestamp8, group3 RX vector24, MAC header24. */
    uint8_t packet[1024] = {0};
    uint8_t *header = packet + 52;
    stw_le_p(packet, 76 + length);
    stw_le_p(packet + 2, 0xe001 | (6 << 9));
    packet[4] = (destination[0] & 1) ? 8 : 2;
    packet[5] = 1;
    packet[6] = 24;
    packet[7] = s->context << 2;
    packet[8] = s->wlan_index;
    stw_le_p(packet + 10, 0x2000);
    stl_le_p(packet + 20, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    packet[40] = packet[41] = 150; /* virtual -35dBm station */
    stw_le_p(header, fc);
    memcpy(header + 4, destination, 6);
    memcpy(header + 10, ap_mac, 6);
    memcpy(header + 16, ap_mac, 6);
    stw_le_p(header + 22, s->frame_sequence++ << 4);
    memcpy(header + 24, body, length);
    queue_rx(s, packet, 76 + length, 1);
}

static void beacon(MT8171WlanState *s)
{
    static const uint8_t ies[] = {
        0, 11, 'K','i','n','d','l','e','-','Q','E','M','U',
        1, 8, 0x82, 0x84, 0x8b, 0x96, 12, 18, 24, 36,
        3, 1, 1, 5, 4, 0, 1, 0, 0, 50, 4, 48, 72, 96, 108,
    };
    uint8_t body[12 + sizeof(ies)] = {0};
    stq_le_p(body, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    stw_le_p(body + 8, 100);
    stw_le_p(body + 10, 0x421);
    memcpy(body + 12, ies, sizeof(ies));
    management_rx(s, 0x80, (const uint8_t *)"\xff\xff\xff\xff\xff\xff",
                  body, sizeof(body));
}

static void transmit(MT8171WlanState *s, const uint8_t *txd,
                      const uint8_t *frame, unsigned length)
{
    uint8_t done[88] = {0};
    bool delivered = false;
    if (s->nic && !qemu_get_queue(s->nic)->link_down &&
        (txd[5] & 0x60) == 0x40 && length >= 24) {
        uint16_t fc = lduw_le_p(frame);
        if ((fc & 0xfc) == 0xb0 && length >= 30 &&
            !lduw_le_p(frame + 24) && lduw_le_p(frame + 26) == 1) {
            uint8_t body[6] = {0, 0, 2, 0, 0, 0};
            management_rx(s, 0xb0, frame + 10, body, sizeof(body));
            delivered = true;
        } else if ((fc & 0xfc) == 0 || (fc & 0xfc) == 0x20) {
            static const uint8_t body[] = {
                0x21, 4, 0, 0, 1, 0xc0,
                1, 8, 0x82, 0x84, 0x8b, 0x96, 12, 18, 24, 36,
                50, 4, 48, 72, 96, 108,
            };
            management_rx(s, (fc & 0xfc) | 0x10, frame + 10,
                          body, sizeof(body));
            s->associated = true;
            delivered = true;
        } else if ((fc & 0xfc) == 0x40) {
            beacon(s);
            delivered = true;
        } else if ((fc & 0xfc) == 0xa0 || (fc & 0xfc) == 0xc0) {
            s->associated = false;
            delivered = true;
        }
    } else if (s->nic && !qemu_get_queue(s->nic)->link_down &&
               s->associated && length >= 14) {
        qemu_send_packet(qemu_get_queue(s->nic), frame, length);
        s->tx_packets++;
        delivered = true;
    }
    /* TX status contains the host's packet ID and WTBL index. The driver
     * uses it to advance authentication and association state machines. */
    if (txd[20] && (txd[21] & 6)) {
        done[0] = txd[20];
        done[1] = delivered ? 0 : 1;
        done[4] = txd[4];
        done[5] = 1;
        runtime_event(s, 0x0f, done, sizeof(done));
    }
}

static void transmit_data(MT8171WlanState *s)
{
    /* Logical data queues0/1 map to hardware rings0/1. CONNAC append V2
     * provides four packet tokens and 36-bit scatter addresses. */
    for (unsigned ring = 0; ring < 2; ring++) {
        unsigned reg = 0x300 + ring * 16;
        unsigned count = R(s, reg + 4) & 0xfff;
        unsigned index = R(s, reg + 12) & 0xfff;
        uint8_t descriptor[16], txd[128], frame[2048], report[12] = {0};
        if (!count || index >= count || index == (R(s, reg + 8) & 0xfff)) {
            continue;
        }
        hwaddr desc = (R(s, reg) | (uint64_t)(R(s, 0x500 + ring * 4) & 15)
                       << 32) + index * 16;
        if (!dma(s, desc, descriptor, sizeof(descriptor), false)) {
            continue;
        }
        uint32_t control = ldl_le_p(descriptor + 4);
        unsigned length = extract32(control, 16, 14);
        hwaddr buffer = ldl_le_p(descriptor) |
                       (uint64_t)(ldl_le_p(descriptor + 12) & 15) << 32;
        if (control & DESC_DONE || length < 64 || length > sizeof(txd) ||
            !dma(s, buffer, txd, length, false)) {
            continue;
        }
        for (unsigned i = 0; i < 4; i++) {
            unsigned token = lduw_le_p(txd + 32 + i * 2);
            unsigned offset = 40 + (i / 2) * 12;
            unsigned size = lduw_le_p(txd + offset + 4 + (i % 2) * 2);
            hwaddr address = ldl_le_p(txd + offset + (i % 2) * 8) |
                             (uint64_t)(size & 0x7000) << 20;
            size &= 0xfff;
            if (!(token & 0x8000)) {
                continue;
            }
            if (size >= 14 && size <= sizeof(frame) &&
                dma(s, address, frame, size, false)) {
                transmit(s, txd, frame, size);
            }
            /* A CONNAC MSDU report returns packet-buffer ownership. */
            stl_le_p(report, 12 | BIT(16) | (6U << 29));
            stw_le_p(report + 8, token & 0x7fff);
            queue_rx(s, report, sizeof(report), 1);
        }
        stl_le_p(descriptor + 4, control | DESC_DONE);
        dma(s, desc, descriptor, sizeof(descriptor), true);
        R(s, reg + 12) = (index + 1) % count;
        R(s, INT_STATUS) |= BIT(4 + ring);
        timer_mod(s->dma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    }
}

static bool can_receive(NetClientState *nc)
{
    MT8171WlanState *s = qemu_get_nic_opaque(nc);
    return ready(s) && s->associated && s->virtual_mcu_state == 2 &&
           !nc->link_down && s->rx_count < ARRAY_SIZE(s->rx_queue);
}

static ssize_t receive_packet(NetClientState *nc, const uint8_t *buf, size_t size)
{
    MT8171WlanState *s = qemu_get_nic_opaque(nc);
    uint8_t packet[2048] = {0};
    if (!can_receive(nc)) {
        return 0;
    }
    if (size < 14 || size + 60 > sizeof(packet)) {
        return size;
    }
    /* Hardware header translation: RXD + group4 retains MAC metadata,
     * followed by an ordinary Ethernet frame for the Linux data path. */
    stw_le_p(packet, size + 60);
    stw_le_p(packet + 2, 0x4000 | (12 << 9));
    packet[4] = (buf[0] & 1) ? 4 : 2;
    packet[5] = 1;
    packet[6] = 0x80 | 14;
    packet[7] = s->context << 2;
    packet[8] = s->wlan_index;
    stw_le_p(packet + 10, 0xc000);
    stw_le_p(packet + 20, 0x0208);
    memcpy(packet + 22, ap_mac, 6);
    stw_le_p(packet + 28, s->frame_sequence++ << 4);
    packet[48] = packet[49] = 150;
    memcpy(packet + 60, buf, size);
    return queue_rx(s, packet, size + 60, 0) ? size : 0;
}

static void beacon_lost(void *opaque)
{
    MT8171WlanState *s = opaque;
    if (ready(s) && s->virtual_mcu_state == 2 && s->associated &&
        qemu_get_queue(s->nic)->link_down) {
        uint8_t event[4] = { s->context, 0, 0, 0 };
        s->associated = false;
        runtime_event(s, 0x13, event, sizeof(event));
    }
}

static void link_changed(NetClientState *nc)
{
    MT8171WlanState *s = qemu_get_nic_opaque(nc);
    if (nc->link_down && s->associated) {
        /* The driver rejects beacon loss while data was received within two
         * seconds. Model missed beacons before reporting firmware timeout. */
        timer_mod(s->beacon_loss_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 3000000000LL);
    } else if (!nc->link_down) {
        timer_del(s->beacon_loss_timer);
    }
}

static bool identity(MT8171WlanState *s, uint32_t chip_addr, uint32_t *value)
{
    if (!ready(s)) {
        return false;
    }
    switch (chip_addr) {
    case 0x80021000: /* TOP_HVR: HW0, factory A, matching stock ECO table */
        *value = s->consys->regs[0x2000 / 4];
        break;
    case 0x80021004: /* TOP_FVR: software revision0 */
        *value = s->consys->regs[0x2004 / 4];
        break;
    case 0x80021008: /* TOP_HCR: SoC2 1x1 chip ID in original chip_info */
        *value = 1;
        break;
    default:
        return false;
    }
    s->identity_reads++;
    return true;
}

static void protect_done(void *opaque)
{
    MT8171WlanState *s = opaque;
    /* An idle HIF can acknowledge isolation; no unmodeled DMA job is
     * discarded to make this transition. Pending unsupported TX descriptors
     * have not been acquired by the device and remain owned by the host. */
    if (ready(s)) {
        R(s, SLP_PROT) = deposit32(R(s, SLP_PROT), 16, 1,
                                   R(s, SLP_PROT) & 1);
        s->protection_transitions++;
    }
}

/* wlanImageSectionConfig(1b862c) sends {address,length,mode}, then
 * wlanImageSectionDownload(1b8744) sends CID0 chunks <=2048 bytes. A config
 * response acknowledges the receive aperture only. Staging encrypted bytes
 * is not a decrypted RAM image, and never sets firmware-ready status. */
static bool configure_download(MT8171WlanState *s, const uint8_t *payload)
{
    uint32_t address = ldl_le_p(payload);
    uint32_t length = ldl_le_p(payload + 4);
    uint32_t mode = ldl_le_p(payload + 8);
    MT8171WlanSection *section;

    if (!length || length > 8 * MiB || address > UINT32_MAX - length ||
        (mode & ~0x8000005fU) || s->section_count == ARRAY_SIZE(s->sections) ||
        s->staging_size + length > 16 * MiB ||
        (s->section_count &&
         s->sections[s->section_count - 1].received !=
         s->sections[s->section_count - 1].length)) {
        return false;
    }
    section = &s->sections[s->section_count++];
    section->address = address;
    section->length = length;
    section->mode = mode;
    section->bytes = g_malloc0(length);
    s->staging_size += length;
    s->download_configs++;
    return true;
}

static void clear_download(MT8171WlanState *s)
{
    for (unsigned n = 0; n < s->section_count; n++) {
        g_free(s->sections[n].bytes);
    }
    memset(s->sections, 0, sizeof(s->sections));
    s->section_count = 0;
    s->staging_size = 0;
}

/* Whole CONSYS reset/power loss invalidates its WLAN child immediately.
 * HIF_RESET is intentionally separate: resetting PDMA does not reset an
 * independently running MCU. Counters survive the physical domain reset. */
static void clear_scan(MT8171WlanState *s);

static void subsystem_reset(void *opaque, int input, int level)
{
    MT8171WlanState *s = opaque;
    if (level && !s->reset_held) {
        timer_del(s->dma_timer);
        timer_del(s->protect_timer);
        timer_del(s->startup_timer);
        timer_del(s->beacon_loss_timer);
        clear_scan(s);
        clear_download(s);
        memset(s->regs, 0, sizeof(s->regs));
        memset(&s->runtime, 0, sizeof(s->runtime));
        memset(s->basic_config, 0, sizeof(s->basic_config));
        memset(s->log_config, 0, sizeof(s->log_config));
        s->start_flags = s->start_address = 0;
        s->virtual_mcu_state = 0;
        s->rx_head = s->rx_count = 0;
        s->associated = false;
        if (s->nic) {
            qemu_purge_queued_packets(qemu_get_queue(s->nic));
        }
        s->firmware_owns = false;
        s->unsupported_reported = false;
        s->subsystem_resets++;
        qemu_set_irq(s->irq, 0);
    }
    s->reset_held = level;
}

static void startup_done(void *opaque)
{
    MT8171WlanState *s = opaque;
    if (ready(s) && s->virtual_mcu_state == 1) {
        s->virtual_mcu_state = 2;
        s->virtual_starts++;
        timer_mod(s->dma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    } else if (ready(s) && s->virtual_mcu_state == 3) {
        /* wlanSendNicPowerCtrlCmd(1) shuts down the virtual service; the
         * driver polls its firmware-sync mask until this transition. */
        s->virtual_mcu_state = 0;
        s->virtual_stops++;
        memset(&s->runtime, 0, sizeof(s->runtime));
    }
}

static bool can_start(MT8171WlanState *s, const uint8_t *p)
{
    uint32_t flags = ldl_le_p(p), address = ldl_le_p(p + 4);
    bool found = !(flags & 1);
    if ((flags & ~15U) || !s->section_count) {
        return false;
    }
    for (unsigned n = 0; n < s->section_count; n++) {
        MT8171WlanSection *section = &s->sections[n];
        if (section->received != section->length) {
            return false;
        }
        found |= address >= section->address &&
                 address - section->address < section->length;
    }
    return found;
}

static unsigned add_capability(uint8_t *p, uint32_t tag,
                               const void *data, unsigned length)
{
    stl_le_p(p, tag);
    stl_le_p(p + 4, length);
    memcpy(p + 8, data, length);
    return 8 + length;
}

/* Exact TLV layouts come from the original gNicCapabilityV2InfoTable
 * consumers. Values describe this virtual station:
 * four BSS, one WMM/spatial stream, 32 retained table entries, no offloads.
 * No efuse, RF calibration, beamforming or measured radio limits invented. */
static unsigned capability_event(MT8171WlanState *s, uint8_t *event)
{
    uint8_t hw[20] = {0}, sw[24] = {0}, checksum[4] = {0};
    const uint8_t phy[12] = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3, 0};
    const uint8_t mac_limits[4] = {
        MT8171_WLAN_BSS, 1, MT8171_WLAN_WTBL, MT8171_WLAN_BSS
    };
    unsigned size = 36; /* RX20 + runtime event12 + TLV-list header4 */
    stw_le_p(hw, 1); /* SoC2 1x1 product identity, distinct from TOP_HVR */
    stw_le_p(sw, 1);
    memcpy(sw + 8, "QEMU-VMCU", 9);
    stw_le_p(event + 32, 6);
    size += add_capability(event + size, 4, checksum, sizeof(checksum));
    size += add_capability(event + size, 5, hw, sizeof(hw));
    size += add_capability(event + size, 6, sw, sizeof(sw));
    size += add_capability(event + size, 7, s->mac, sizeof(s->mac));
    size += add_capability(event + size, 8, phy, sizeof(phy));
    size += add_capability(event + size, 9, mac_limits, sizeof(mac_limits));
    event[24] = 0xec;
    return size;
}

/* wlanFeatureToFw / wlanCfgParse / wlanCfgSetGetFw: 12-byte header followed
 * by <=4 typed 68-byte {type,keylen,vallen,reserved,key[32],value[32]} items.
 * This virtual service stores configuration by key. No RF algorithms are
 * claimed to run as a result of retaining calibration or tuning inputs. */
static bool set_features(MT8171WlanState *s, const uint8_t *p, unsigned length)
{
    unsigned slots[4], count, used = s->runtime.setting_count;

    if (length != 284) {
        return false;
    }
    count = p[8];
    if (ldl_le_p(p) > 1 || ldl_le_p(p + 4) != 1 || !count || count > 4 ||
        lduw_le_p(p + 10) != count * 68) {
        return false;
    }
    /* Validate the complete batch before changing its retained values. */
    for (unsigned i = 0; i < count; i++) {
        const uint8_t *item = p + 12 + i * 68;
        unsigned slot = UINT_MAX;
        if (item[0] < 1 || item[0] > 3 || !item[1] || item[1] > 32 ||
            item[2] > 32) {
            return false;
        }
        for (unsigned j = 0; j < s->runtime.setting_count; j++) {
            const uint8_t *old = s->runtime.settings[j];
            if (old[1] == item[1] && !memcmp(old + 4, item + 4, item[1])) {
                slot = j;
                break;
            }
        }
        if (slot == UINT_MAX) {
            for (unsigned j = 0; j < i; j++) {
                const uint8_t *old = p + 12 + j * 68;
                if (old[1] == item[1] && !memcmp(old + 4, item + 4, item[1])) {
                    slot = slots[j];
                    break;
                }
            }
            if (slot == UINT_MAX) {
                slot = used++;
            }
        }
        if (slot >= ARRAY_SIZE(s->runtime.settings)) {
            return false;
        }
        slots[i] = slot;
    }
    for (unsigned i = 0; i < count; i++) {
        memcpy(s->runtime.settings[slots[i]], p + 12 + i * 68, 68);
    }
    s->runtime.setting_count = used;
    memcpy(s->log_config, p, sizeof(s->log_config));
    return true;
}

/* Runtime reply payloads are produced by specific virtual services. */
/* Virtual open AP on channel1: actual request bytes and channel intervals
 * are retained. Scan delivery uses real receive descriptors.
 * Layout: stock scnSendScanReqV2, corroborated by mt76_connac_hw_scan_req.
 * The released MCU's zero-dwell default is opaque. This virtual service uses
 * the local mt76 CONNAC nominal 60ms active / 120ms passive policy instead. */
static bool validate_scan(MT8171WlanState *s, const uint8_t *p, unsigned length)
{
    if (length != sizeof(s->scan_request) || p[1] >= MT8171_WLAN_CONTEXTS ||
        p[2] > 1 || p[7] != 1 || p[4] > 4 || p[0x33b] > 6 ||
        p[0x9e] > 4 || p[0x9f] > 32 || p[0x33a] > 32 ||
        (p[0x9e] == 4 && !(p[0x9f] + p[0x33a])) ||
        lduw_le_p(p + 0xe0) > 600 ||
        (p[6] & 4) || s->scan_active || s->scan_event_pending) {
        return false;
    }
    for (unsigned i = 0; i < p[4] + p[0x33b]; i++) {
        const uint8_t *ssid = i < p[4] ? p + 8 + i * 36 :
                              p + 0x380 + (i - p[4]) * 36;
        if (ldl_le_p(ssid) > 32) {
            return false;
        }
    }
    for (unsigned i = 0; i < p[0x9f] + p[0x33a]; i++) {
        const uint8_t *ch = i < p[0x9f] ? p + 0xa0 + i * 2 :
                            p + 0x33e + (i - p[0x9f]) * 2;
        if (ch[0] != 1 || ch[1] < 1 || ch[1] > 14) {
            return false; /* only the advertised 2.4GHz virtual PHY */
        }
    }
    return true;
}

static void clear_scan(MT8171WlanState *s)
{
    timer_del(s->scan_timer);
    s->scan_active = s->scan_event_pending = s->scan_timed_out = false;
    s->scheduled_scan_enabled = false;
    s->scan_count = s->scan_index = 0;
}

static void schedule_scan_channel(MT8171WlanState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->scan_channel_end_ns = now + s->scan_dwell_ms * 1000000LL;
    timer_mod(s->scan_timer, MIN(s->scan_channel_end_ns, s->scan_deadline_ns));
}

static void start_scan(MT8171WlanState *s, const uint8_t *p)
{
    unsigned timeout_ms = lduw_le_p(p + 0x9c);
    memcpy(s->scan_request, p, sizeof(s->scan_request));
    if (p[0x9e] == 4) {
        s->scan_count = p[0x9f] + p[0x33a];
        memcpy(s->scan_channels, p + 0xa0, p[0x9f] * 2);
        memcpy(s->scan_channels + p[0x9f], p + 0x33e, p[0x33a] * 2);
    } else {
        s->scan_count = 13;
        for (unsigned i = 0; i < s->scan_count; i++) {
            s->scan_channels[i][0] = 1;
            s->scan_channels[i][1] = i + 1;
        }
    }
    s->scan_index = 0;
    s->scan_dwell_ms = lduw_le_p(p + 0x9a);
    if (!s->scan_dwell_ms) {
        s->scan_dwell_ms = p[2] ? 60 : 120;
    }
    s->scan_dwell_ms = MAX(s->scan_dwell_ms, lduw_le_p(p + 0x33c));
    s->scan_dwell_ms = MAX(s->scan_dwell_ms, lduw_le_p(p + 0x98));
    s->scan_deadline_ns = timeout_ms ?
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + timeout_ms * 1000000LL : INT64_MAX;
    s->scan_active = true;
    s->scan_timed_out = false;
    s->scan_requests++;
    schedule_scan_channel(s);
}

static void scan_channel_done(void *opaque)
{
    MT8171WlanState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (!s->scan_active && s->scheduled_scan_enabled && ready(s) &&
        s->virtual_mcu_state == 2) {
        bool matches = !s->scheduled_scan[4];
        for (unsigned i = 0; i < MIN(s->scheduled_scan[4], 16); i++) {
            const uint8_t *match = s->scheduled_scan + 368 + i * 40;
            matches |= match[36] == 11 && !memcmp(match + 4, "Kindle-QEMU", 11);
        }
        if (matches && s->nic && !qemu_get_queue(s->nic)->link_down) {
            uint8_t done[4] = {s->scheduled_scan[1]};
            beacon(s);
            runtime_event(s, 0x23, done, sizeof(done));
            s->scheduled_scan_enabled = !s->scheduled_scan[2];
        }
        if (s->scheduled_scan_enabled) {
            unsigned seconds = MAX(lduw_le_p(s->scheduled_scan + 1140), 1);
            timer_mod(s->scan_timer, now + seconds * 1000000000LL);
        }
        return;
    }
    if (!s->scan_active || !ready(s) || s->virtual_mcu_state != 2) {
        return;
    }
    if (now >= s->scan_channel_end_ns) {
        if (s->scan_channels[s->scan_index][1] == 1 && s->nic &&
            !qemu_get_queue(s->nic)->link_down) {
            beacon(s);
        }
        s->scan_index++;
        s->scanned_channels++;
    }
    if (s->scan_index < s->scan_count && now < s->scan_deadline_ns) {
        schedule_scan_channel(s);
        return;
    }
    s->scan_timed_out = s->scan_index < s->scan_count;
    s->scan_active = false;
    s->scan_event_pending = true;
    timer_mod(s->dma_timer, now + 20000);
}

/* Asynchronous scan-done uses the same actual RX ring/buffer ownership as
 * command responses. A full ring retains the event until the host replenishes
 * it. MCU RX production does not require the AP to own the command port. */
static void deliver_scan_event(MT8171WlanState *s)
{
    uint8_t rxd[16], event[508] = {0}; /* RX20 + event12 + stock body476 */
    unsigned count = R(s, RX_EVENT + 4) & 0xfff;
    unsigned index = R(s, RX_EVENT + 12) & 0xfff;
    hwaddr desc, buffer;
    uint32_t control;
    if (!s->scan_event_pending || !ready(s) ||
        !(R(s, DMA_CONFIG) & 4) || (R(s, SLP_PROT) & 1) || !count ||
        index >= count || index == (R(s, RX_EVENT + 8) & 0xfff)) {
        return;
    }
    desc = (R(s, RX_EVENT) |
            (uint64_t)(R(s, RX_EVENT_EXT) & 15) << 32) + index * 16;
    if (!dma(s, desc, rxd, sizeof(rxd), false)) {
        return;
    }
    control = ldl_le_p(rxd + 4);
    if (control & DESC_DONE) {
        return;
    }
    if (extract32(control, 16, 14) < sizeof(event)) {
        s->dma_faults++;
        qemu_log_mask(LOG_GUEST_ERROR, "mt8171-wlan: short scan RX buffer\n");
        return;
    }
    buffer = ldl_le_p(rxd) | (uint64_t)(ldl_le_p(rxd + 8) & 15) << 32;
    stl_le_p(event, sizeof(event) | 0xe0000000U);
    stw_le_p(event + 20, sizeof(event) - 20);
    stw_le_p(event + 22, 0xe000);
    event[24] = 0x0d; /* original event table .data+4ae8 -> nicEventScanDone */
    event[32] = s->scan_request[0];
    event[36] = s->scan_index; /* complete_channel_num */
    event[37] = s->scan_timed_out ? 7 : 0;
    event[38] = 1; /* version1; no sparse-channel/RF statistics advertised */
    if (!dma(s, buffer, event, sizeof(event), true)) {
        return;
    }
    stl_le_p(rxd + 4, deposit32(control | DESC_DONE | DESC_LAST,
                               16, 14, sizeof(event)));
    if (!dma(s, desc, rxd, sizeof(rxd), true)) {
        return;
    }
    R(s, RX_EVENT + 12) = (index + 1) % count;
    R(s, INT_STATUS) |= BIT(1);
    s->scan_event_pending = false;
    s->scan_completions++;
    s->rx_events++;
    update_irq(s);
}

static bool runtime_command(MT8171WlanState *s, const uint8_t *command,
                            unsigned length, uint8_t *event,
                            unsigned *event_size)
{
    const uint8_t *p = command + 64;
    unsigned payload_len = length - 64;
    if (s->virtual_mcu_state != 2 || command[41] != 0 || command[42] != 3) {
        return false;
    }
    switch (command[36]) {
    case 0x79: /* channel/network manager snapshot, SoC2 single-radio layout */
        if (command[38] != 0 || payload_len != 165) {
            return false;
        }
        *event_size = 32 + 165;
        event[24] = 0x79;
        /* No DBDC or concurrent channels. The station occupies channel1. */
        if (s->associated && s->context < MT8171_WLAN_BSS) {
            event[33] = 1;
            event[35] = 1;
        }
        break;
    case 0x7e: /* host throughput/rate telemetry; no response requested */
        if (command[38] != 1 || payload_len != sizeof(s->runtime.performance)) {
            return false;
        }
        memcpy(s->runtime.performance, p, payload_len);
        break;
    case 0x10: /* station IP addresses for firmware ARP policy */
        if (command[38] != 1 || payload_len < 4 ||
            p[0] >= MT8171_WLAN_CONTEXTS || payload_len < 4 + p[1] * 4 ||
            payload_len > sizeof(s->runtime.network_addresses[0])) {
            return false;
        }
        memset(s->runtime.network_addresses[p[0]], 0,
               sizeof(s->runtime.network_addresses[0]));
        memcpy(s->runtime.network_addresses[p[0]], p, payload_len);
        break;
    case 0x90: /* force RTS policy; virtual medium has no RF contention */
        if (command[38] != 1 || payload_len != 4 || p[0] > 1) {
            return false;
        }
        memcpy(s->runtime.force_rts, p, payload_len);
        break;
    case 0xca: /* textual radio feature policy, e.g. SET_STBC 0 0 */
        if (command[38] != 1 || payload_len != sizeof(s->runtime.chip_config) ||
            lduw_le_p(p + 4) > 320) {
            return false;
        }
        memcpy(s->runtime.chip_config, p, payload_len);
        break;
    case 7: /* Removing an old security key is valid on the open AP. */
        if (command[38] != 1 || payload_len != 64 || p[0] != 0) {
            return false;
        }
        break;
    case 8: /* Default key selection; this open BSS uses no key material. */
        if (command[38] != 1 || payload_len != 4) {
            return false;
        }
        break;
    case 0x82:
        if (command[38] != 0) {
            return false;
        }
        *event_size = 128;
        event[24] = 3;
        stq_le_p(event + 32, s->tx_packets);
        stq_le_p(event + 104, s->rx_packets);
        break;
    case 0xcd: /* WTBL telemetry, queried by the stock Wi-Fi metrics job */
        if (command[38] != 0 || payload_len != 160 ||
            ldl_le_p(p) >= MT8171_WLAN_WTBL) {
            return false;
        }
        /* nicCmdEventQueryWlanInfo copies the 156-byte WTBL record after
         * the caller's index. The virtual open BSS has no key/BA/offload
         * state. Complete the query so telemetry cannot stall command DMA. */
        *event_size = 32 + 156;
        event[24] = 0xcd;
        break;
    case 0x85: /* station statistics: hardware RF counters are unavailable */
        if (command[38] != 0 || payload_len != 28 || p[0] >= 27) {
            return false;
        }
        /* Flags.bit0 clear tells nicUpdateStaStats there is no valid RF
         * sample. It still completes the OID and releases the command ring. */
        *event_size = 44;
        event[24] = 0x21;
        event[40] = p[0];
        break;
    case 9: /* infrastructure mode resets the previous connection */
        if (command[38] != 1 || payload_len) {
            return false;
        }
        s->associated = false;
        break;
    case 0x62: /* scheduled scan's fixed header and optional probe IEs */
        if (command[38] != 1 || payload_len < sizeof(s->scheduled_scan) ||
            payload_len != sizeof(s->scheduled_scan) + lduw_le_p(p + 6) ||
            p[3] > 10 || p[4] > 16) {
            return false;
        }
        memcpy(s->scheduled_scan, p, sizeof(s->scheduled_scan));
        break;
    case 0x61:
        if (command[38] != 1 || payload_len != 4 || p[0] > 1) {
            return false;
        }
        s->scheduled_scan_enabled = p[0] == 0;
        if (!s->scan_active && s->scheduled_scan_enabled) {
            timer_mod(s->scan_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                    100000000);
        }
        break;
    case 0x81: /* link quality for the virtual station */
        if (command[38] != 0) {
            return false;
        }
        *event_size = 32 + 8 * MT8171_WLAN_BSS;
        event[24] = 2;
        if (s->associated && s->context < MT8171_WLAN_BSS) {
            uint8_t *quality = event + 32 + s->context * 8;
            quality[0] = (uint8_t)-35;
            quality[1] = 100;
            stw_le_p(quality + 2, 108);
            quality[5] = 1;
        }
        break;
    case 0x1c: /* channel request/grant or release */
        if (command[38] != 1 || payload_len != 24 || p[2] > 1) {
            return false;
        }
        if (p[2] == 0) {
            *event_size = 56;
            event[24] = 0x10;
            memcpy(event + 32, p, 24);
            event[34] = 0; /* granted */
        }
        break;
    case 0x13: /* update station record; reply activates its host queue */
        if (command[38] != 1 || payload_len != 168) {
            return false;
        }
        s->station_index = p[0];
        s->context = p[12];
        /* Stock MT8171 cnmStaSendUpdateCmd uses offset0x33 for NeedResp
         * and0x36 for the WTBL index (the older gen4m layout differs). */
        s->wlan_index = p[0x36];
        if (p[0x33]) {
            *event_size = 40;
            event[24] = 0x0c;
            memcpy(event + 32, p + 2, 6);
            event[38] = p[0];
            event[39] = p[12];
        }
        break;
    case 0x12: /* BSS parameters */
    case 0x16: /* connected power management parameters */
    case 0x17: /* BSS disconnect */
    case 0x19: /* RLM parameters */
    case 0x1d: /* WMM parameters */
    case 0x30: /* roaming state */
        if (command[38] != 1 || !payload_len) {
            return false;
        }
        if (command[36] == 0x17) {
            s->associated = false;
        } else if (command[36] == 0x12 && payload_len >= 42) {
            s->associated = p[1] == 0 && !memcmp(p + 36, ap_mac, 6);
        }
        break;
    case 0x8a: /* wlanQueryNicCapabilityV2, query with no request payload */
        if (command[38] != 0 || payload_len != 0) {
            return false;
        }
        *event_size = capability_event(s, event);
        break;
    case 2: /* wlanUpdateBasicConfig: checksum/config fields, no reply requested */
        if (command[38] != 1 || payload_len != sizeof(s->basic_config) ||
            lduw_le_p(p + 4) || lduw_le_p(p + 6)) {
            return false; /* checksum offload is not advertised or implemented */
        }
        memcpy(s->basic_config, p, sizeof(s->basic_config));
        break;
    case 3: /* scnSendScanReqV2: descriptor accepted before async completion */
        if (command[38] != 1 || !validate_scan(s, p, payload_len)) {
            return false;
        }
        break;
    case 0x1b: /* scnFsmMsgAbort: host generates its own cancelled result */
        if (command[38] != 1 || payload_len != 4 || p[1] > 1 || p[2] || p[3]) {
            return false;
        }
        break;
    case 0x14: /* cnmStaSendRemoveCmd: one/all/all-except-index peer cleanup */
        if (command[38] != 1 || payload_len != 4 || p[0] > 2 ||
            (p[0] != 1 && p[1] >= 27) ||
            p[2] >= MT8171_WLAN_CONTEXTS || p[3]) {
            return false;
        }
        if (p[0] == 1 || (p[0] == 0 && p[1] == s->station_index) ||
            (p[0] == 2 && p[1] != s->station_index)) {
            s->associated = false;
        }
        break;
    case 0xa5: /* wlanoidNotifyFwCalibration after gRestoreSuccessFlag */
        if (command[38] != 1 || payload_len) {
            return false;
        }
        /* A notification, not an RF calibration result. No reply requested. */
        break;
    case 0x58: /* wlanoidNotifyFwSuspend: per-context suspend/resume policy */
        if (command[38] != 1 || payload_len != 68 ||
            p[0] >= MT8171_WLAN_CONTEXTS || p[1] > 1 || p[3] > 1) {
            return false;
        }
        /* wlanNotifyFwSuspend sets context, enable, MDTIM and WOW enable;
         * the remaining64 bytes are reserved. WOW is not advertised. */
        for (unsigned i = 4; i < 68; i++) {
            if (p[i]) {
                return false;
            }
        }
        break;
    case 4: /* wlanSendNicPowerCtrlCmd: mode1 stops, reserved bytes zero */
        if (command[38] != 1 || payload_len != 4 || ldl_le_p(p) != 1) {
            return false;
        }
        /* Apply after the actual command descriptor has completed DMA. */
        break;
    case 5: /* nicConfigPowerSaveProfile: per-context CAM/MAX/FAST policy */
        if (command[38] != 1 || payload_len != 4 ||
            p[0] >= MT8171_WLAN_CONTEXTS || p[1] > 2) {
            return false;
        }
        s->runtime.power_save[p[0]] = p[1];
        break;
    case 0x0a: /* wlanoidSetPacketFilter: filter bitmap + 64 reserved bytes */
        if (command[38] != 1 || payload_len != 68) {
            return false;
        }
        s->runtime.packet_filter = ldl_le_p(p);
        break;
    case 0x0f: /* rlmDomainSend{Domain,PassiveScan}InfoCmd, six subbands */
        if (command[38] != 1 || payload_len != 56 || lduw_le_p(p + 2) > 1) {
            return false;
        }
        memcpy(s->runtime.domain[lduw_le_p(p + 2)], p, 56);
        break;
    case 0x11: { /* nic{Activate,Deactivate}NetworkEx */
        unsigned context;
        if (command[38] != 1 || payload_len != 12) {
            return false;
        }
        context = p[0] & 0x3f;
        if (context >= MT8171_WLAN_CONTEXTS || p[1] > 1 ||
            (p[10] != 0xff && p[10] >= MT8171_WLAN_WTBL)) {
            return false;
        }
        for (unsigned i = 0; i < MT8171_WLAN_WTBL; i++) {
            if (s->runtime.wtbl[i][0] && s->runtime.wtbl[i][1] == context) {
                memset(s->runtime.wtbl[i], 0, sizeof(s->runtime.wtbl[i]));
            }
        }
        memcpy(s->runtime.network[context], p, 12);
        if (p[1] && p[10] != 0xff) {
            uint8_t *entry = s->runtime.wtbl[p[10]];
            entry[0] = 1;
            entry[1] = context;
            memcpy(entry + 2, p + 4, 6);
        }
        break;
    }
    case 0x28: /* cnmUpdateDbdcSetting: version1, length36, disable only */
        if (command[38] != 1 || payload_len != 36 || p[0] || p[4] != 1 ||
            lduw_le_p(p + 6) != 36) {
            return false; /* DBDC is not advertised by this virtual PHY */
        }
        memcpy(s->runtime.dbdc, p, 36);
        break;
    case 0x48: /* wlanLoadManufactureData: original NVRAM receive aperture */
        if (command[38] != 1 || payload_len != sizeof(s->runtime.nvram)) {
            return false;
        }
        memcpy(s->runtime.nvram, p, sizeof(s->runtime.nvram));
        break;
    case 0x49: /* rlmDomainSendPwrLimitCmd: fixed 1472-byte power-limit table */
        if (command[38] != 1 || payload_len != sizeof(s->runtime.power_limits)) {
            return false;
        }
        memcpy(s->runtime.power_limits, p, sizeof(s->runtime.power_limits));
        break;
    case 0x5a: /* aisSync11kCapabilities: set a context's RRM capability bytes */
        if (command[38] != 1 || payload_len != 44 || ldl_le_p(p) != 1 ||
            p[4] >= MT8171_WLAN_CONTEXTS || p[5] != 1) {
            return false;
        }
        memcpy(s->runtime.rrm[p[4]], p, 44);
        break;
    case 0x70:
        if (command[38] != 1 || !set_features(s, p, payload_len)) {
            return false;
        }
        break;
    case 0xc1: /* wlanoidSetMulticastList: count32, context8, MAC[32][6] */
        if (command[38] != 1 || payload_len != 200 || ldl_le_p(p) > 32 ||
            p[4] >= MT8171_WLAN_CONTEXTS) {
            return false;
        }
        memcpy(s->runtime.multicast[p[4]], p, 200);
        break;
    default:
        return false;
    }
    return true;
}

static void run_dma(void *opaque)
{
    MT8171WlanState *s = opaque;
    uint8_t txd[16], rxd[16], command[64 + 2048], event[EVENT_SIZE] = {0};
    unsigned txreg = TX_CMD, txext = TX_CMD_EXT, txirq = 7;
    deliver_rx(s);
    deliver_scan_event(s);
    if (ready(s) && s->virtual_mcu_state == 2 && !s->firmware_owns &&
        (R(s, DMA_CONFIG) & 1) && !(R(s, SLP_PROT) & 1)) {
        transmit_data(s);
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
    }
    /* INIT and runtime use different logical command queues in the stock
     * kalDevPortWrite / halWpdmaWriteCmd implementations. */
    if (R(s, TX_CMD + 8) == R(s, TX_CMD + 12)) {
        txreg = TX_RUNTIME; txext = TX_RUNTIME_EXT; txirq = 19;
    }
    uint32_t txcount = R(s, txreg + 4) & 0xfff;
    uint32_t rxcount = R(s, RX_EVENT + 4) & 0xfff;
    uint32_t txidx = R(s, txreg + 12) & 0xfff;
    uint32_t rxidx = R(s, RX_EVENT + 12) & 0xfff;
    uint32_t txctrl, rxctrl = 0, chip_addr, value, length;
    unsigned event_size = 0, cid = 0xff;
    hwaddr txdesc, rxdesc = 0, txbuf, rxbuf = 0;
    MT8171WlanSection *section = NULL;
    bool config = false, start = false, runtime = false;

    if (!ready(s) || s->firmware_owns || !(R(s, DMA_CONFIG) & 1) ||
        (R(s, SLP_PROT) & 1) || !txcount || txidx >= txcount ||
        txidx == (R(s, txreg + 8) & 0xfff)) {
        return;
    }
    txdesc = (R(s, txreg) |
              (uint64_t)(R(s, txext) & 0xf) << 32) + txidx * 16;
    if (!dma(s, txdesc, txd, sizeof(txd), false)) {
        return;
    }
    txctrl = ldl_le_p(txd + 4);
    if (txctrl & DESC_DONE) {
        return;
    }
    txbuf = ldl_le_p(txd) | (uint64_t)(ldl_le_p(txd + 12) & 0xf) << 32;
    length = extract32(txctrl, 16, 14);
    if (!(txctrl & DESC_LAST) || (txctrl & 0x3fff) ||
        length < 32 || length > sizeof(command)) {
        goto unsupported;
    }
    if (!dma(s, txbuf, command, length, false)) {
        return;
    }
    if ((command[5] & 0x60) == 0x40 && s->virtual_mcu_state == 2) {
        transmit(s, command, command + 32, length - 32);
        goto complete;
    }
    if (length < 64) {
        goto unsupported;
    }
    cid = command[36];
    /* nicTxInitCmd rounds the bus transfer to four bytes. Padding is not
     * part of the firmware stream recorded in the TXD byte count. */
    if (lduw_le_p(command) < 64 ||
        (lduw_le_p(command) != length &&
         ROUND_UP(lduw_le_p(command), 4) != length) ||
        command[37] != 0xa0) {
        goto unsupported;
    }
    length = lduw_le_p(command);
    runtime = command[42] == 3;
    if (runtime) {
        if (s->virtual_mcu_state == 1) {
            return; /* startup timer will resume the queued runtime request */
        }
        if (!runtime_command(s, command, length, event, &event_size)) {
            goto unsupported;
        }
    } else if (cid == 2) {
        if (length != 72 || !can_start(s, command + 64)) {
            goto unsupported;
        }
        start = true;
        event_size = 32;
        event[24] = 1;
    } else if (cid == 0) {
        if (lduw_le_p(command + 2) != 0xf800 || length == 64 ||
            !s->section_count) {
            goto unsupported;
        }
        section = &s->sections[s->section_count - 1];
        if (length - 64 > section->length - section->received) {
            goto unsupported;
        }
    } else if (cid == 1 || cid == 5) {
        if (length != 76) {
            goto unsupported;
        }
        config = true;
        event_size = 32; /* EID1: status byte followed by three reserved bytes */
        event[24] = 1;
    } else if (cid == 3) {
        if (length != 76 || command[64] != 0) {
            goto unsupported;
        }
        chip_addr = ldl_le_p(command + 68);
        if (!identity(s, chip_addr, &value)) {
            goto unsupported;
        }
        event_size = 36;
        event[24] = 2;
        stl_le_p(event + 28, chip_addr);
        stl_le_p(event + 32, value);
    } else {
        goto unsupported;
    }
    if (event_size) {
        if (!(R(s, DMA_CONFIG) & 4) || !rxcount || rxidx >= rxcount ||
            rxidx == (R(s, RX_EVENT + 8) & 0xfff)) {
            return;
        }
        rxdesc = (R(s, RX_EVENT) |
                  (uint64_t)(R(s, RX_EVENT_EXT) & 0xf) << 32) + rxidx * 16;
        if (!dma(s, rxdesc, rxd, sizeof(rxd), false)) {
            return;
        }
        rxctrl = ldl_le_p(rxd + 4);
        if (rxctrl & DESC_DONE) {
            return;
        }
        if (extract32(rxctrl, 16, 14) < event_size) {
            qemu_log_mask(LOG_GUEST_ERROR, "mt8171-wlan: short RX event buffer\n");
            s->dma_faults++;
            return;
        }
        rxbuf = ldl_le_p(rxd) | (uint64_t)(ldl_le_p(rxd + 8) & 0xf) << 32;
        if (config && !configure_download(s, command + 64)) {
            goto unsupported;
        }
        /* Stock RX prefix20 + INIT header8 + command-specific response. */
        stl_le_p(event, event_size | 0xe0000000U);
        stw_le_p(event + 20, event_size - 20);
        stw_le_p(event + 22, 0xe000);
        event[25] = command[39];
        if (!dma(s, rxbuf, event, event_size, true)) {
            return;
        }
        stl_le_p(rxd + 4, deposit32(rxctrl | DESC_DONE | DESC_LAST,
                                   16, 14, event_size));
        if (!dma(s, rxdesc, rxd, sizeof(rxd), true)) {
            return;
        }
    }
complete:
    stl_le_p(txd + 4, txctrl | DESC_DONE);
    if (!dma(s, txdesc, txd, sizeof(txd), true)) {
        return;
    }
    if (start) {
        s->start_flags = ldl_le_p(command + 64);
        s->start_address = ldl_le_p(command + 68);
        s->virtual_mcu_state = 1;
        s->firmware_owns = false;
        timer_mod(s->startup_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
    }
    if (runtime) {
        s->runtime_commands++;
        s->capability_queries += cid == 0x8a;
        s->configuration_commands += cid != 0x8a && cid != 4;
        if (cid == 3) {
            start_scan(s, command + 64);
        } else if (cid == 0x1b) {
            if ((s->scan_active || s->scan_event_pending) &&
                s->scan_request[0] == command[64]) {
                clear_scan(s);
            }
            s->scan_cancels++;
        } else if (cid == 0x14) {
            memcpy(s->runtime.station_cleanup, command + 64, 4);
            s->station_removals++;
        } else if (cid == 0xa5) {
            s->runtime.calibration_notified = true;
            s->calibration_notifications++;
        } else if (cid == 0x58) {
            memcpy(s->runtime.host_suspend[command[64]], command + 64, 68);
            s->host_suspend_notifications++;
        } else if (cid == 4) {
            clear_scan(s);
            s->virtual_mcu_state = 3;
            timer_mod(s->startup_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
        }
    }
    if (section) {
        memcpy(section->bytes + section->received, command + 64, length - 64);
        section->received += length - 64;
        s->download_bytes += length - 64;
        s->download_chunks++;
        if (section->received == section->length) {
            s->download_sections++;
        }
    }
    R(s, txreg + 12) = (txidx + 1) % txcount;
    R(s, INT_STATUS) |= BIT(txirq);
    if (event_size) {
        R(s, RX_EVENT + 12) = (rxidx + 1) % rxcount;
        R(s, INT_STATUS) |= BIT(1);
        s->rx_events++;
    }
    s->tx_commands++;
    s->unsupported_reported = false;
    update_irq(s);
    timer_mod(s->dma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    return;

unsupported:
    if (!s->unsupported_reported) {
        s->unsupported_commands++;
        qemu_log_mask(LOG_GUEST_ERROR, "mt8171-wlan: unsupported %s TX at %"
                      HWADDR_PRIx " descriptor=%08x cid=%u length=%u; "
                      "command left outstanding\n", runtime ? "runtime" : "init",
                      txbuf, txctrl, cid, length);
        s->unsupported_reported = true;
    }
}

static uint64_t read_reg(void *opaque, hwaddr address, unsigned size)
{
    MT8171WlanState *s = opaque;
    return ready(s) ? R(s, address) : 0;
}

static void write_reg(void *opaque, hwaddr address, uint64_t value, unsigned size)
{
    MT8171WlanState *s = opaque;
    if (!ready(s)) {
        return;
    }
    if (address == INT_STATUS) {
        R(s, address) &= ~value;
    } else if (address == SLP_PROT) {
        R(s, address) = (value & ~BIT(16)) | (R(s, address) & BIT(16));
        timer_mod(s->protect_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    } else if (address == RESET_INDEX) {
        for (unsigned n = 0; n < 16; n++) {
            if (value & BIT(n)) {
                R(s, 0x300 + n * 16 + 12) = 0;
            }
        }
        for (unsigned n = 0; n < 2; n++) {
            if (value & BIT(16 + n)) {
                R(s, 0x400 + n * 16 + 12) = 0;
            }
        }
    } else if (address >= 0x300 && address < 0x420 &&
               (address & 15) == 12) {
        return; /* DMA producer indices are hardware-owned. */
    } else {
        R(s, address) = value;
        if (address == DMA_CONFIG) {
            R(s, address) &= ~(BIT(1) | BIT(3)); /* no transfer active */
        }
        if (address == HIF_RESET && !(value & BIT(4))) {
            timer_del(s->dma_timer);
            R(s, INT_STATUS) = 0;
            R(s, TX_CMD + 12) = R(s, RX_EVENT + 12) = 0;
            R(s, TX_RUNTIME + 12) = 0;
            /* HIF reset ends the receive session. Staging is not physical
             * instruction RAM; retained-byte counters remain cumulative. */
            clear_download(s);
            s->firmware_owns = false;
            s->unsupported_reported = false;
        }
    }
    update_irq(s);
    if (!(R(s, SLP_PROT) & 1)) {
        timer_mod(s->dma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    }
}

static uint64_t read_identity(void *opaque, hwaddr address, unsigned size)
{
    uint32_t value = 0;
    identity(opaque, 0x80021000 + address, &value);
    return value;
}

/* chip_info firmware-sync CR and asicLowPowerOwn{Read,Set,Clear}.
 * Sync6 denotes this virtual implementation's running service processors.
 * It is not evidence that the staged encrypted instructions ran. */
static uint64_t read_sync(void *opaque, hwaddr address, unsigned size)
{
    MT8171WlanState *s = opaque;
    return ready(s) && (s->virtual_mcu_state == 2 ||
                        s->virtual_mcu_state == 3) ? 6 : 0;
}

static uint64_t read_ownership(void *opaque, hwaddr address, unsigned size)
{
    MT8171WlanState *s = opaque;
    return ready(s) && s->firmware_owns;
}

static void write_ownership(void *opaque, hwaddr address, uint64_t value,
                            unsigned size)
{
    MT8171WlanState *s = opaque;
    bool old = s->firmware_owns;
    if (!ready(s)) {
        return;
    }
    if (value & 2) {
        s->firmware_owns = false;
    } else if ((value & 1) &&
               R(s, TX_CMD + 8) == R(s, TX_CMD + 12) &&
               R(s, TX_RUNTIME + 8) == R(s, TX_RUNTIME + 12)) {
        /* halHifPowerOffWifi returns ownership after firmware-sync clears.
         * The host-CSR W1S/W1C register remains accessible while the WLAN
         * service is stopped; granting an idle bus does not restart it. */
        s->firmware_owns = true;
    }
    s->ownership_changes += old != s->firmware_owns;
    if (!s->firmware_owns) {
        timer_mod(s->dma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    }
}

static const MemoryRegionOps sync_ops = {
    .read = read_sync,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static const MemoryRegionOps ownership_ops = {
    .read = read_ownership, .write = write_ownership,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps ops = {
    .read = read_reg, .write = write_reg,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static const MemoryRegionOps identity_ops = {
    .read = read_identity,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void reset(DeviceState *dev)
{
    MT8171WlanState *s = MT8171_WLAN(dev);
    timer_del(s->dma_timer);
    timer_del(s->protect_timer);
    timer_del(s->startup_timer);
    timer_del(s->beacon_loss_timer);
    clear_scan(s);
    s->scan_requests = s->scan_completions = s->scan_cancels = 0;
    s->scanned_channels = s->station_removals = 0;
    s->calibration_notifications = s->host_suspend_notifications = 0;
    s->virtual_mcu_state = 0;
    s->rx_head = s->rx_count = 0;
    s->associated = false;
    s->firmware_owns = false;
    s->virtual_starts = s->runtime_commands = s->capability_queries = 0;
    s->configuration_commands = s->virtual_stops = 0;
    s->subsystem_resets = 0;
    memset(&s->runtime, 0, sizeof(s->runtime));
    s->ownership_changes = 0;
    memcpy(s->mac, s->conf.macaddr.a, sizeof(s->mac));
    memset(s->basic_config, 0, sizeof(s->basic_config));
    memset(s->log_config, 0, sizeof(s->log_config));
    memset(s->regs, 0, sizeof(s->regs));
    clear_download(s);
    s->download_configs = s->download_bytes = 0;
    s->download_chunks = s->download_sections = 0;
    s->unsupported_reported = false;
    s->tx_commands = s->rx_events = s->dma_faults = 0;
    s->unsupported_commands = s->protection_transitions = s->identity_reads = 0;
    qemu_set_irq(s->irq, 0);
}

static void init(Object *obj)
{
    MT8171WlanState *s = MT8171_WLAN(obj);
    memory_region_init_io(&s->iomem, obj, &ops, s, TYPE_MT8171_WLAN, 0x1000);
    memory_region_init_io(&s->identity_iomem, obj, &identity_ops, s,
                         "mt8171-wlan-top", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->identity_iomem);
    memory_region_init_io(&s->sync_iomem, obj, &sync_ops, s,
                         "mt8171-wlan-virtual-mcu-sync", 4);
    memory_region_init_io(&s->ownership_iomem, obj, &ownership_ops, s,
                         "mt8171-wlan-ownership", 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->sync_iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->ownership_iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->scan_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, scan_channel_done, s);
    s->beacon_loss_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, beacon_lost, s);
    s->startup_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, startup_done, s);
    s->dma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, run_dma, s);
    s->protect_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, protect_done, s);
    object_property_add_uint64_ptr(obj, "scan-requests", &s->scan_requests, OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "bss-context", &s->context, OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "wlan-index", &s->wlan_index, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "scan-completions", &s->scan_completions, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "scan-cancels", &s->scan_cancels, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "scanned-channels", &s->scanned_channels, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "station-removals", &s->station_removals, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "calibration-notifications", &s->calibration_notifications, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "host-suspend-notifications", &s->host_suspend_notifications, OBJ_PROP_FLAG_READ);
    qdev_init_gpio_in_named(DEVICE(obj), subsystem_reset, "reset", 1);
    object_property_add_uint64_ptr(obj, "subsystem-resets", &s->subsystem_resets, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "tx-commands", &s->tx_commands, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "rx-events", &s->rx_events, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "dma-faults", &s->dma_faults, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "unsupported-commands", &s->unsupported_commands, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "protection-transitions", &s->protection_transitions, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "identity-reads", &s->identity_reads, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "virtual-mcu-state", &s->virtual_mcu_state, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "virtual-starts", &s->virtual_starts, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "runtime-commands", &s->runtime_commands, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "configuration-commands", &s->configuration_commands, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "configuration-keys", &s->runtime.setting_count, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "virtual-stops", &s->virtual_stops, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "capability-queries", &s->capability_queries, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "ownership-changes", &s->ownership_changes, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "download-configs", &s->download_configs, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "download-bytes", &s->download_bytes, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "download-chunks", &s->download_chunks, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "download-sections", &s->download_sections, OBJ_PROP_FLAG_READ);
}

static void finalize(Object *obj)
{
    MT8171WlanState *s = MT8171_WLAN(obj);
    clear_download(s);
    timer_free(s->dma_timer);
    timer_free(s->protect_timer);
    timer_free(s->startup_timer);
    timer_free(s->scan_timer);
    timer_free(s->beacon_loss_timer);
}

static NetClientInfo net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = can_receive,
    .receive = receive_packet,
    .link_status_changed = link_changed,
};

static void realize(DeviceState *dev, Error **errp)
{
    MT8171WlanState *s = MT8171_WLAN(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    memcpy(s->mac, s->conf.macaddr.a, sizeof(s->mac));
    s->nic = qemu_new_nic(&net_info, &s->conf, TYPE_MT8171_WLAN, dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void unrealize(DeviceState *dev)
{
    MT8171WlanState *s = MT8171_WLAN(dev);
    qemu_del_nic(s->nic);
}

static const Property properties[] = {
    DEFINE_NIC_PROPERTIES(MT8171WlanState, conf),
};

static void class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_legacy_reset(dc, reset);
    device_class_set_props(dc, properties);
    dc->realize = realize;
    dc->unrealize = unrealize;
}

static const TypeInfo info = {
    .name = TYPE_MT8171_WLAN, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8171WlanState), .instance_init = init,
    .instance_finalize = finalize, .class_init = class_init,
};
static void register_types(void) { type_register_static(&info); }
type_init(register_types)
