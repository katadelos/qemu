/*
 * BCM4343W SDIO Wi-Fi (BCM43430 Wi-Fi core).
 *
 * Implements the backplane, firmware download and SDPCM/CDC/BDC interfaces used
 * by the Broadcom FullMAC host driver. The virtual radio exposes the open
 * Kindle-QEMU access point; Ethernet traffic is carried by a QEMU network backend.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/sd/sd.h"
#include "hw/sd/bcm4343w-sdio.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "sdmmc-internal.h"
#include "trace.h"

#define BCM_RAM_SIZE (512 * 1024)
#define BCM_BP_BASE 0x18000000u
#define BCM_BP_SIZE 0x00200000u
#define BCM_SDIO_BASE 0x18002000u
#define BCM_ARM_WRAP 0x18103000u
#define BCM_EROM 0x18010000u
#define BCM_SHARED 0x0007e000u
#define BCM_FRAME_MAX 16384
#define BCM_QUEUE_MAX 128
#define BCM_WLFC_FIFO_CREDITS 8
#define BCM_WLFC_FIFO_COUNT 5
#define BCM_FRAME_IND 0x40
#define BCM_HOST_INT 0x80
#define BCM_SSID "Kindle-QEMU"
#define BCM_SSID_LEN (sizeof(BCM_SSID) - 1)
#define BCM_CHANSPEC 0x1001 /* 2.4GHz, 20MHz, channel 1 (ioctl v2). */

static const uint8_t bcm_bssid[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x02 };
static const uint8_t bcm_rates[] = { 0x82, 0x84, 0x8b, 0x96, 12, 18, 24, 36,
                                     48, 72, 96, 108 };

typedef struct BCMFrame {
    unsigned len;
    uint8_t data[BCM_FRAME_MAX];
} BCMFrame;

struct BCM4343WState {
    DeviceState parent_obj;
    NICConf conf;
    NICState *nic;
    qemu_irq irq, oob_irq;
    bool powered, selected, firmware_running, associated, up, software_oob;
    bool wlfc;
    uint8_t cccr[0x300];
    uint8_t f1[0x100];
    uint8_t cis[0x300];
    uint8_t *ram, *bp;
    uint32_t window, intstatus, hostintmask, mailbox;
    uint32_t rx_packets, tx_packets, power_mode;
    uint8_t rx_seq, tx_seq;
    unsigned fn, address, length, position;
    bool writing, increment;
    uint8_t tx[BCM_FRAME_MAX];
    GQueue rx;
    unsigned rx_position;
    GHashTable *iovars;
    QEMUTimer *scan_timer, *join_timer;
    uint16_t scan_sync;
    bool escan;
};

static void bcm_update_irq(BCM4343WState *s)
{
    uint32_t status = s->intstatus;
    if (!g_queue_is_empty(&s->rx)) {
        status |= BCM_FRAME_IND;
    }
    bool pending = s->powered && (status & s->hostintmask);
    qemu_set_irq(s->irq, pending && (s->cccr[4] & 1) && (s->cccr[4] & 6));
    /* OOB must stay disabled during probe while the driver holds its bus lock. */
    if ((s->cccr[0xf2] & 3) == 3) {
        qemu_set_irq(s->oob_irq, (s->cccr[0xf2] & 4) ? pending : !pending);
    } else {
        qemu_set_irq(s->oob_irq, pending && s->software_oob);
    }
}

static void bcm_enqueue(BCM4343WState *s, unsigned channel,
                        const uint8_t *data, unsigned len)
{
    BCMFrame *f;
    if (len > BCM_FRAME_MAX - 12 || s->rx.length >= BCM_QUEUE_MAX) {
        return;
    }
    f = g_new0(BCMFrame, 1);
    f->len = len + 12;
    stw_le_p(f->data, f->len);
    stw_le_p(f->data + 2, ~f->len);
    f->data[4] = s->rx_seq++;
    f->data[5] = channel;
    f->data[7] = 12;
    f->data[9] = s->tx_seq + 32;
    if (len) {
        memcpy(f->data + 12, data, len);
    }
    g_queue_push_tail(&s->rx, f);
    bcm_update_irq(s);
}

static void bcm_event(BCM4343WState *s, uint32_t event, uint32_t status,
                      uint16_t flags, const uint8_t *data, unsigned len)
{
    uint8_t packet[2048] = { 0x20, 0, 0, 0 };
    uint8_t *eth = packet + 4, *hdr = eth + 14, *msg = hdr + 10;
    if (len > sizeof(packet) - 78) {
        return;
    }
    memcpy(eth, s->conf.macaddr.a, 6);
    memcpy(eth + 6, bcm_bssid, 6);
    stw_be_p(eth + 12, 0x886c);
    stw_be_p(hdr, 0x8001); /* BCMILCP_SUBTYPE_VENDOR_LONG */
    stw_be_p(hdr + 2, 58 + len);
    hdr[4] = 0;
    hdr[5] = 0x00;
    hdr[6] = 0x10;
    hdr[7] = 0x18;
    stw_be_p(hdr + 8, 1); /* BCMILCP_BCM_SUBTYPE_EVENT */
    stw_be_p(msg, 2);
    stw_be_p(msg + 2, flags);
    stl_be_p(msg + 4, event);
    stl_be_p(msg + 8, status);
    stl_be_p(msg + 20, len);
    memcpy(msg + 24, bcm_bssid, 6);
    memcpy(msg + 30, "wlan0", 6);
    if (len) {
        memcpy(msg + 48, data, len);
    }
    trace_bcm4343w_event(event, status, len);
    /*
     * Firmware event Ethernet frames carry a two-byte trailer.  DHD removes
     * ETHER_TYPE_LEN from the received length before validating bcm_event_t
     * and its data; omitting this padding makes newer drivers discard every
     * event, including completed scans and associations.
     */
    bcm_enqueue(s, 1, packet, 78 + len);
}

/* wl_bss_info version 109; byte offsets preserve the ARM ABI's alignment. */
static unsigned bcm_bss_info(uint8_t *bss)
{
    static const uint8_t ies[] = {
        0, BCM_SSID_LEN, 'K', 'i', 'n', 'd', 'l', 'e', '-', 'Q', 'E', 'M', 'U',
        1, 8, 0x82, 0x84, 0x8b, 0x96, 12, 18, 24, 36,
        3, 1, 1,
        50, 4, 48, 72, 96, 108,
        5, 4, 0, 1, 0, 0,
    };
    unsigned len = 128 + sizeof(ies);
    memset(bss, 0, len);
    stl_le_p(bss, 109);
    stl_le_p(bss + 4, len);
    memcpy(bss + 8, bcm_bssid, 6);
    stw_le_p(bss + 14, 100);
    stw_le_p(bss + 16, 0x421); /* ESS, short preamble, short slot. */
    bss[18] = BCM_SSID_LEN;
    memcpy(bss + 19, BCM_SSID, BCM_SSID_LEN);
    stl_le_p(bss + 52, sizeof(bcm_rates));
    memcpy(bss + 56, bcm_rates, sizeof(bcm_rates));
    stw_le_p(bss + 72, BCM_CHANSPEC);
    bss[76] = 1;
    stw_le_p(bss + 78, (uint16_t)-35);
    bss[80] = (uint8_t)-95;
    bss[88] = 1;
    stw_le_p(bss + 116, 128);
    stl_le_p(bss + 120, sizeof(ies));
    stw_le_p(bss + 124, 60);
    memcpy(bss + 128, ies, sizeof(ies));
    return len;
}

static void bcm_scan_done(void *opaque)
{
    BCM4343WState *s = opaque;
    uint8_t result[512] = { 0 };
    unsigned len;
    if (!s->powered || !s->firmware_running) {
        return;
    }
    if (s->escan) {
        len = 12 + bcm_bss_info(result + 12);
        stl_le_p(result, len);
        stl_le_p(result + 4, 1);
        stw_le_p(result + 8, s->scan_sync);
        stw_le_p(result + 10, 1);
        bcm_event(s, 69, 8, 0, result, len); /* ESCAN_RESULT, PARTIAL */
        stl_le_p(result, 12);
        stw_le_p(result + 10, 0);
        bcm_event(s, 69, 0, 0, result, 12);
    } else {
        bcm_event(s, 26, 0, 0, NULL, 0); /* SCAN_COMPLETE */
    }
}

static void bcm_join_done(void *opaque)
{
    BCM4343WState *s = opaque;
    if (!s->powered || !s->firmware_running) {
        return;
    }
    s->associated = true;
    if (s->wlfc) {
        uint8_t credits[BCM_WLFC_FIFO_COUNT + 1] = { 0 };

        /* Establish the firmware's five transmit FIFOs before link-up. */
        memset(credits, BCM_WLFC_FIFO_CREDITS, BCM_WLFC_FIFO_COUNT);
        bcm_event(s, 74, 0, 0, credits, sizeof(credits));
        bcm_event(s, 127, 0, 0, NULL, 0); /* BCMC_CREDIT_SUPPORT */
    }
    bcm_event(s, 3, 0, 0, NULL, 0); /* AUTH */
    bcm_event(s, 7, 0, 0, NULL, 0); /* ASSOC */
    bcm_event(s, 16, 0, 1, NULL, 0); /* LINK */
    bcm_event(s, 0, 0, 1, NULL, 0); /* SET_SSID */
    qemu_flush_queued_packets(qemu_get_queue(s->nic));
}

static int bcm_start_join(BCM4343WState *s, const uint8_t *in, size_t len)
{
    if (len < 4 + BCM_SSID_LEN || ldl_le_p(in) != BCM_SSID_LEN ||
        memcmp(in + 4, BCM_SSID, BCM_SSID_LEN)) {
        return -2; /* BCME_BADARG */
    }
    timer_mod(s->join_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 50000000);
    return 0;
}

static bool bcm_setting_known(const char *name)
{
    /* FullMAC configuration which has no RF side effects on the virtual AP. */
    static const char * const names[] = {
        "event_msgs", "event_msgs_ext", "country", "assoc_listen",
        "bcn_timeout", "roam_off", "fullroamperiod", "roam_env_detection",
        "mpc", "allmulti", "arp_ol", "arpoe", "arp_hostip", "arp_table_clear",
        "pkt_filter_add", "pkt_filter_enable", "pkt_filter_mode",
        "pkt_filter_delete", "ndoe", "nd_hostip", "nd_hostip_clear",
        "wsec", "wpa_auth", "wpaie", "auth", "infra", "closednet", "pm2_sleep_ret",
        "scan_assoc_time", "scan_unassoc_time", "scan_passive_time",
        "btc_mode", "btc_params", "btc_flags", "txbf", "buf_key_b4_m4",
        "assoc_retry_max", "roam_trigger", "roam_delta", "roam_scan_period",
        "keep_alive", "mkeep_alive", "bcn_li_dtim", "bcn_li_bcn",
        "dtim_assoc", "bus:idleclock", "bus:txglomalign", "bus:txglom",
        "ampdu_ba_wsize", "ampdu_mpdu", "ampdu_release", "ampdu_tid",
        "scb_timeout", "lpc", "okc_enable", "ccx_enable", "pmk",
        "pmkid_info", "vndr_ie", "apsta", "frameburst", "bss",
    };
    for (unsigned i = 0; i < ARRAY_SIZE(names); i++) {
        if (!strcmp(name, names[i])) {
            return true;
        }
    }
    return false;
}

static int bcm_iovar(BCM4343WState *s, const char *name, bool set,
                     const uint8_t *in, unsigned inlen, uint8_t *out,
                     unsigned *outlen)
{
    GBytes *value;
    if (g_str_has_prefix(name, "bsscfg:")) {
        name += 7;
        if (inlen < 4) {
            return -2;
        }
        in += 4;
        inlen -= 4;
    }
    if (set) {
        if (!strcmp(name, "escan")) {
            if (inlen < 8) {
                return -2;
            }
            s->escan = true;
            s->scan_sync = lduw_le_p(in + 6);
            timer_mod(s->scan_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000000);
        } else if (!strcmp(name, "join")) {
            return bcm_start_join(s, in, inlen);
        } else if (!strcmp(name, "cur_etheraddr")) {
            if (inlen != 6) {
                return -2;
            }
            memcpy(s->conf.macaddr.a, in, 6);
        } else if (!strcmp(name, "tlv")) {
            if (inlen < 4) {
                return -2;
            }
            s->wlfc = (ldl_le_p(in) & 8) != 0;
        } else if (!strcmp(name, "proptxstatus")) {
            if (inlen < 4) {
                return -2;
            }
            s->wlfc = ldl_le_p(in) != 0;
        } else if (!strcmp(name, "wlfc_mode")) {
            /* Firmware queues packet IDs, without sequence reuse. */
            if (inlen < 4 || ldl_le_p(in) != 4) {
                return -23;
            }
        } else if (!strcmp(name, "bus:rxglom") ||
                   !strcmp(name, "ampdu_hostreorder") ||
                   !strcmp(name, "hostreorder")) {
            /* These optional framing modes are not advertised or supported. */
            if (inlen >= 4 && ldl_le_p(in)) {
                return -23;
            }
        } else if (!bcm_setting_known(name)) {
            return -23;
        }
        g_hash_table_replace(s->iovars, g_strdup(name), g_bytes_new(in, inlen));
        *outlen = 0;
        return 0;
    }
    if (!strcmp(name, "cur_etheraddr")) {
        memcpy(out, s->conf.macaddr.a, 6);
        *outlen = 6;
    } else if (!strcmp(name, "ver")) {
        const char version[] = "BCM43430/1 QEMU SDPCM FullMAC 7.45.41.26";
        memcpy(out, version, sizeof(version));
        *outlen = sizeof(version);
    } else if (!strcmp(name, "cap")) {
        const char capabilities[] = "sta escan proptxstatus";
        memcpy(out, capabilities, sizeof(capabilities));
        *outlen = sizeof(capabilities);
    } else if (!strcmp(name, "wlfc_mode")) {
        stl_le_p(out, 4); /* WLFC AFQ capability bit */
        *outlen = 4;
    } else if (!strcmp(name, "proptxstatus")) {
        stl_le_p(out, s->wlfc);
        *outlen = 4;
    } else if (!strcmp(name, "country")) {
        memset(out, 0, 12);
        memcpy(out, "US", 2);
        memcpy(out + 8, "US", 2);
        *outlen = 12;
    } else if (!strcmp(name, "chanspecs")) {
        stl_le_p(out, 11);
        for (unsigned i = 0; i < 11; i++) {
            stl_le_p(out + 4 + i * 4, 0x1001 + i);
        }
        *outlen = 48;
    } else if (!strcmp(name, "chanspec")) {
        stl_le_p(out, BCM_CHANSPEC);
        *outlen = 4;
    } else if (!strcmp(name, "per_chan_info")) {
        stl_le_p(out, 3); /* valid hardware and software channel */
        *outlen = 4;
    } else if (!strcmp(name, "assoc_info")) {
        memset(out, 0, 12);
        stl_le_p(out, 4); /* fixed association request length */
        stl_le_p(out + 4, 6); /* fixed association response length */
        *outlen = 12;
    } else if (!strcmp(name, "assoc_req_ies") ||
               !strcmp(name, "assoc_resp_ies")) {
        *outlen = 0;
    } else if ((value = g_hash_table_lookup(s->iovars, name))) {
        gsize len;
        const void *data = g_bytes_get_data(value, &len);
        *outlen = MIN(len, BCM_FRAME_MAX - 32);
        if (*outlen) {
            memcpy(out, data, *outlen);
        }
    } else if (!strcmp(name, "event_msgs") ||
               !strcmp(name, "event_msgs_ext")) {
        memset(out, 0, 128);
        *outlen = 128;
    } else if (!strcmp(name, "nmode") || !strcmp(name, "vhtmode") ||
               !strcmp(name, "toe_ol") || !strcmp(name, "bcmerror") ||
               !strcmp(name, "roam_off") || !strcmp(name, "apsta") ||
               !strcmp(name, "mpc") || !strcmp(name, "wsec") ||
               !strcmp(name, "wpa_auth") || !strcmp(name, "auth") ||
               !strcmp(name, "p2p") || !strcmp(name, "arpoe") ||
               !strcmp(name, "bus:rxglom")) {
        stl_le_p(out, 0);
        *outlen = 4;
    } else {
        return -23; /* BCME_UNSUPPORTED */
    }
    return 0;
}

static void bcm_control(BCM4343WState *s, const uint8_t *in, unsigned len)
{
    uint8_t reply[BCM_FRAME_MAX - 12] = { 0 };
    uint8_t *out = reply + 16;
    const uint8_t *arg;
    uint32_t cmd, flags, requested;
    unsigned inlen, outlen = 0;
    int status = 0;
    bool set;
    const char *name = "";
    if (len < 16) {
        return;
    }
    cmd = ldl_le_p(in);
    flags = ldl_le_p(in + 8);
    requested = MIN(ldl_le_p(in + 4) & 0xffff, sizeof(reply) - 16);
    set = flags & 2;
    arg = in + 16;
    inlen = len - 16;
    if (cmd == 262 || cmd == 263) {
        unsigned namelen = strnlen((const char *)arg, inlen);
        if (namelen == inlen) {
            status = -2;
        } else {
            name = (const char *)arg;
            status = bcm_iovar(s, name, cmd == 263, arg + namelen + 1,
                               inlen - namelen - 1, out, &outlen);
        }
    } else {
        outlen = set ? 0 : 4;
        switch (cmd) {
        case 0: stl_le_p(out, 0x14e46c77); break;
        case 1: stl_le_p(out, 2); break;
        case 2: s->up = true; break;
        case 3: s->up = s->associated = false; break;
        case 12: stl_le_p(out, 108); break;
        case 19: stl_le_p(out, 1); break;
        case 23:
            if (s->associated) {
                memcpy(out, bcm_bssid, 6);
            }
            outlen = 6;
            break;
        case 25:
            stl_le_p(out, s->associated ? BCM_SSID_LEN : 0);
            memcpy(out + 4, BCM_SSID, BCM_SSID_LEN);
            outlen = 36;
            break;
        case 26: status = bcm_start_join(s, arg, inlen); break;
        case 29:
            stl_le_p(out, 1);
            stl_le_p(out + 4, 1);
            stl_le_p(out + 8, 1);
            outlen = 12;
            break;
        case 39: stl_le_p(out, 4); break; /* N PHY */
        case 50:
            s->escan = false;
            timer_mod(s->scan_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000000);
            break;
        case 51:
            outlen = 12 + bcm_bss_info(out + 12);
            stl_le_p(out, outlen);
            stl_le_p(out + 4, 109);
            stl_le_p(out + 8, 1);
            break;
        case 52:
            s->associated = false;
            timer_del(s->join_timer);
            bcm_event(s, 16, 0, 0, NULL, 0);
            break;
        case 71: case 114:
            stl_le_p(out, sizeof(bcm_rates));
            memcpy(out + 4, bcm_rates, sizeof(bcm_rates));
            outlen = 20;
            break;
        case 75: stl_le_p(out, 100); break;
        case 77: stl_le_p(out, 1); break;
        case 83: memcpy(out, "US", 2); break;
        case 85: stl_le_p(out, s->power_mode); break;
        case 86:
            if (inlen >= 4) {
                s->power_mode = ldl_le_p(arg);
            }
            break;
        case 98: /* wlc_rev_info_t */
            stl_le_p(out, 0x14e4);
            stl_le_p(out + 4, 0x43e2);
            stl_le_p(out + 8, 1);
            stl_le_p(out + 12, 1);
            stl_le_p(out + 16, 39);
            stl_le_p(out + 20, 0x1202);
            stl_le_p(out + 28, 43430);
            stl_le_p(out + 32, 1);
            stl_le_p(out + 36, 5); /* SDIO bus */
            stl_le_p(out + 44, 4);
            stl_le_p(out + 48, 1);
            outlen = 68;
            break;
        case 127: stl_le_p(out, -35); break;
        case 135: stl_le_p(out, -95); break;
        case 136:
            outlen = 4 + bcm_bss_info(out + 4);
            stl_le_p(out, outlen);
            break;
        case 137: /* get_pktcnt_t used by cfg80211's station query */
            stl_le_p(out, s->rx_packets);
            stl_le_p(out + 8, s->tx_packets);
            outlen = 20;
            break;
        case 140:
            stl_le_p(out, 1);
            stl_le_p(out + 4, 2);
            outlen = 8;
            break;
        case 141: stl_le_p(out, 2); break;
        case 159:
            stl_le_p(out, s->associated ? 1 : 0);
            memcpy(out + 4, bcm_bssid, 6);
            outlen = s->associated ? 10 : 4;
            break;
        case 162: stl_le_p(out, s->up); break;
        case 217: /* WLC_GET_VALID_CHANNELS */
            stl_le_p(out, 11);
            for (unsigned i = 0; i < 11; i++) {
                stl_le_p(out + 4 + 4 * i, i + 1);
            }
            outlen = 48;
            break;
        default:
            if (!set) {
                status = -23;
            }
            break;
        }
    }
    if (status) {
        flags |= 1;
        outlen = 0;
    }
    outlen = MIN(outlen, requested);
    stl_le_p(reply, cmd);
    stl_le_p(reply + 4, outlen);
    stl_le_p(reply + 8, flags);
    stl_le_p(reply + 12, status);
    trace_bcm4343w_ioctl(cmd, name, set, inlen, outlen, status);
    bcm_enqueue(s, 0, reply, 16 + outlen);
}

static void bcm_txstatus(BCM4343WState *s, const uint8_t *tlvs, unsigned len)
{
    while (len) {
        unsigned type = *tlvs++, size;
        uint32_t tag;
        unsigned fifo;
        uint8_t reply[20] = { 0x20, 0, 0, 4, 4, 4 };

        len--;
        if (type == 255) { /* WLFC_CTL_TYPE_FILLER */
            continue;
        }
        if (!len) {
            return;
        }
        size = *tlvs++;
        len--;
        if (size > len) {
            return;
        }
        if (type == 5 && size >= 4) { /* WLFC_CTL_TYPE_PKTTAG */
            tag = ldl_le_p(tlvs);
            fifo = (tag >> 24) & 7;
            if (!(tag & BIT(27)) || fifo >= BCM_WLFC_FIFO_COUNT) {
                return;
            }
            /* Echo the packet ID and generation with successful TX status. */
            stl_le_p(reply + 6, tag & ~0x78000000u);
            reply[10] = 11; /* WLFC_CTL_TYPE_FIFO_CREDITBACK */
            reply[11] = 6;
            reply[12 + fifo] = 1;
            reply[18] = reply[19] = 255;
            bcm_enqueue(s, 2, reply, sizeof(reply));
            return;
        }
        tlvs += size;
        len -= size;
    }
}

static void bcm_tx_frame(BCM4343WState *s, const uint8_t *frame, unsigned len)
{
    unsigned size, offset, channel;
    if (len < 12) {
        return;
    }
    size = lduw_le_p(frame);
    offset = frame[7];
    channel = frame[5] & 15;
    if (size < 12 || size > len || offset < 12 || offset > size ||
        (uint16_t)(size ^ lduw_le_p(frame + 2)) != 0xffff) {
        return;
    }
    s->tx_seq = frame[4] + 1;
    trace_bcm4343w_frame(true, channel, size, frame[4]);
    if (channel == 0) {
        bcm_control(s, frame + offset, size - offset);
    } else if (channel == 2 && size - offset >= 4) {
        unsigned eth_offset = offset + 4 + frame[offset + 3] * 4;
        if (eth_offset <= size && size - eth_offset >= 14 && s->associated) {
            s->tx_packets++;
            qemu_send_packet(qemu_get_queue(s->nic), frame + eth_offset,
                             size - eth_offset);
        }
        if (s->wlfc && eth_offset <= size) {
            bcm_txstatus(s, frame + offset + 4, eth_offset - offset - 4);
        }
        if (g_queue_is_empty(&s->rx)) {
            /* Header-only SDPCM frames update credits without an Ethernet RX. */
            bcm_enqueue(s, 2, NULL, 0);
        }
    }
}

static void bcm_firmware_start(BCM4343WState *s)
{
    if (!s->firmware_running) {
        s->firmware_running = true;
        /* Firmware publishes SDPCM shared state in the last word of RAM. */
        memset(s->ram + BCM_SHARED, 0, 64);
        stl_le_p(s->ram + BCM_SHARED, 1);
        stl_le_p(s->ram + BCM_RAM_SIZE - 4, BCM_SHARED);
        s->mailbox = 0x0004000a; /* SDPCM v4, DEVREADY and FWREADY */
        s->intstatus |= BCM_HOST_INT;
        trace_bcm4343w_firmware_start();
        bcm_update_irq(s);
    }
}

static uint32_t bcm_bp_read32(BCM4343WState *s, uint32_t addr)
{
    switch (addr) {
    case BCM_SDIO_BASE + 0x20:
        return s->intstatus | (g_queue_is_empty(&s->rx) ? 0 : BCM_FRAME_IND);
    case BCM_SDIO_BASE + 0x24: return s->hostintmask;
    case BCM_SDIO_BASE + 0x4c: return s->mailbox;
    default:
        if (addr < BCM_RAM_SIZE - 3) {
            return ldl_le_p(s->ram + addr);
        }
        if (addr >= BCM_BP_BASE && addr < BCM_BP_BASE + BCM_BP_SIZE - 3) {
            return ldl_le_p(s->bp + addr - BCM_BP_BASE);
        }
        return 0;
    }
}

static uint8_t bcm_bp_read(BCM4343WState *s, uint32_t addr)
{
    uint32_t value = bcm_bp_read32(s, addr & ~3u);
    if (!(addr & 3)) {
        trace_bcm4343w_backplane(false, addr, value);
    }
    return value >> ((addr & 3) * 8);
}

static void bcm_bp_write(BCM4343WState *s, uint32_t addr, uint8_t value)
{
    uint32_t aligned = addr & ~3u;
    unsigned shift = (addr & 3) * 8;
    if (addr < BCM_RAM_SIZE) {
        s->ram[addr] = value;
        return;
    }
    if (addr < BCM_BP_BASE || addr >= BCM_BP_BASE + BCM_BP_SIZE) {
        return;
    }
    switch (aligned) {
    case BCM_SDIO_BASE + 0x20:
        s->intstatus &= ~((uint32_t)value << shift);
        break;
    case BCM_SDIO_BASE + 0x24:
        s->hostintmask = (s->hostintmask & ~(0xffu << shift)) |
                         ((uint32_t)value << shift);
        break;
    case BCM_SDIO_BASE + 0x40:
        if (shift == 0 && (value & 4)) {
            s->software_oob = true;
        }
        if (shift == 0 && (value & 8)) {
            s->software_oob = false;
        }
        if (shift == 0 && (value & 2)) {
            s->mailbox = 0;
            s->intstatus &= ~BCM_HOST_INT;
        }
        if (shift == 0 && (value & 1)) {
            s->rx_position = 0;
            s->mailbox |= 1;
            s->intstatus |= BCM_HOST_INT;
        }
        break;
    default:
        s->bp[addr - BCM_BP_BASE] = value;
        if (addr == BCM_ARM_WRAP + 0x800 && !(value & 1) && ldl_le_p(s->ram + 4)) {
            bcm_firmware_start(s);
        }
        /* Clock requests complete synchronously; status bits are read-only. */
        if (aligned == BCM_BP_BASE + 0x1e0 ||
            aligned == BCM_SDIO_BASE + 0x1e0) {
            uint32_t val = ldl_le_p(s->bp + aligned - BCM_BP_BASE);
            stl_le_p(s->bp + aligned - BCM_BP_BASE, val | 0x30000);
        }
        break;
    }
    if ((addr & 3) == 3) {
        trace_bcm4343w_backplane(true, aligned, bcm_bp_read32(s, aligned));
    }
    bcm_update_irq(s);
}

static uint8_t bcm_read(BCM4343WState *s, unsigned fn, unsigned addr)
{
    if (fn == 0) {
        if (addr >= 0x1000 && addr < 0x1300) {
            return s->cis[addr - 0x1000];
        }
        switch (addr) {
        case 3: return s->cccr[2] & 6;
        case 5:
            return (s->intstatus || !g_queue_is_empty(&s->rx)) ? 2 : 0;
        default: return addr < sizeof(s->cccr) ? s->cccr[addr] : 0;
        }
    }
    if (fn == 1) {
        if (addr >= 0x10000) {
            switch (addr) {
            case 0x1000e: return s->f1[0x0e] | 0xc0;
            case 0x1001f: return s->f1[0x1f] | 2; /* KSO, device on */
            case 0x10019: case 0x1001a: return 0;
            case 0x1001b: case 0x1001c: {
                BCMFrame *f = g_queue_peek_head(&s->rx);
                unsigned remaining = f ? f->len - MIN(f->len, s->rx_position) : 0;
                return remaining >> (8 * (addr - 0x1001b));
            }
            default: return s->f1[addr & 0xff];
            }
        }
        return bcm_bp_read(s, s->window | (addr & 0x7fff));
    }
    return 0;
}

static void bcm_write(BCM4343WState *s, unsigned fn, unsigned addr, uint8_t value)
{
    if (fn == 0) {
        if (addr < sizeof(s->cccr)) {
            switch (addr) {
            case 2: case 4: case 7: case 0x13:
            case 0x110: case 0x111: case 0x210: case 0x211:
            case 0xf0: case 0xf1: case 0xf2:
                s->cccr[addr] = value;
                break;
            case 6:
                s->length = s->position = 0;
                break;
            default:
                break;
            }
        }
        bcm_update_irq(s);
    } else if (fn == 1) {
        if (addr >= 0x10000) {
            s->f1[addr & 0xff] = value;
            switch (addr) {
            case 0x1000a: case 0x1000b: case 0x1000c:
                s->window = ((uint32_t)s->f1[0x0c] << 24) |
                            ((uint32_t)s->f1[0x0b] << 16) |
                            ((uint32_t)(s->f1[0x0a] & 0x80) << 8);
                break;
            case 0x1000d:
                if ((value & 1) && !g_queue_is_empty(&s->rx)) {
                    g_free(g_queue_pop_head(&s->rx));
                    s->rx_position = 0;
                }
                bcm_update_irq(s);
                break;
            default:
                break;
            }
        } else {
            bcm_bp_write(s, s->window | (addr & 0x7fff), value);
        }
    }
}

static size_t bcm_command(SDState *card, SDRequest *req,
                          uint8_t *response, size_t response_size)
{
    BCM4343WState *s = BCM4343W_SDIO(card);
    unsigned fn = (req->arg >> 28) & 7;
    unsigned addr = (req->arg >> 9) & 0x1ffff;
    bool write = req->arg >> 31;
    uint8_t value;
    if (!s->powered || response_size < 4) {
        return 0;
    }
    memset(response, 0, 4);
    trace_bcm4343w_command(req->cmd, req->arg);
    switch (req->cmd) {
    case 0:
        s->selected = false;
        s->cccr[2] = s->cccr[4] = 0;
        bcm_update_irq(s);
        return 0;
    case 5:
        stl_be_p(response, 0xa0ff8000);
        return 4;
    case 3:
        stl_be_p(response, 0x00010000);
        return 4;
    case 7:
        s->selected = (req->arg >> 16) == 1;
        return 4;
    case 52:
        if (fn > 2) {
            response[2] = 2;
            return 4;
        }
        value = req->arg;
        if (write) {
            bcm_write(s, fn, addr, value);
        }
        response[3] = (!write || (req->arg & (1 << 27))) ?
                      bcm_read(s, fn, addr) : value;
        return 4;
    case 53:
        if (!fn || fn > 2 || !(s->cccr[2] & (1 << fn))) {
            response[2] = 2;
            return 4;
        }
        s->fn = fn;
        s->address = addr;
        s->writing = write;
        s->increment = req->arg & (1 << 26);
        s->length = req->arg & 0x1ff;
        if (!s->length) {
            s->length = 512;
        }
        if (req->arg & (1 << 27)) {
            unsigned blocksize = lduw_le_p(s->cccr + fn * 0x100 + 0x10);
            s->length *= blocksize ? blocksize : 512;
        }
        s->position = 0;
        if (write && fn == 2) {
            memset(s->tx, 0, sizeof(s->tx));
        }
        return 4;
    default:
        return 0;
    }
}

static uint8_t bcm_read_byte(SDState *card)
{
    BCM4343WState *s = BCM4343W_SDIO(card);
    uint8_t value = 0;
    BCMFrame *f;
    if (s->writing || s->position >= s->length) {
        return 0;
    }
    if (s->fn == 2) {
        f = g_queue_peek_head(&s->rx);
        if (f && s->rx_position < f->len) {
            /* Refresh credits immediately before the header is consumed. */
            f->data[9] = s->tx_seq + 32;
            value = f->data[s->rx_position++];
        }
        if (++s->position == s->length && f && s->rx_position >= f->len) {
            trace_bcm4343w_frame(false, f->data[5] & 15, f->len, f->data[4]);
            g_free(g_queue_pop_head(&s->rx));
            s->rx_position = 0;
            bcm_update_irq(s);
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
    } else {
        value = bcm_read(s, s->fn,
                         s->address + (s->increment ? s->position : 0));
        s->position++;
    }
    return value;
}

static void bcm_write_byte(SDState *card, uint8_t value)
{
    BCM4343WState *s = BCM4343W_SDIO(card);
    if (!s->writing || s->position >= s->length) {
        return;
    }
    if (s->fn == 2) {
        if (s->position < sizeof(s->tx)) {
            s->tx[s->position] = value;
        }
        if (++s->position == s->length && s->length <= sizeof(s->tx)) {
            bcm_tx_frame(s, s->tx, s->length);
        }
    } else {
        bcm_write(s, s->fn, s->address + (s->increment ? s->position : 0), value);
        s->position++;
    }
}

static void bcm_read_data(SDState *card, uint8_t *buf, size_t len)
{
    while (len--) {
        *buf++ = bcm_read_byte(card);
    }
}

static void bcm_write_data(SDState *card, const uint8_t *buf, size_t len)
{
    while (len--) {
        bcm_write_byte(card, *buf++);
    }
}

static bool bcm_receive_ready(SDState *card)
{
    BCM4343WState *s = BCM4343W_SDIO(card);
    return s->powered && s->writing && s->position < s->length;
}

static bool bcm_data_ready(SDState *card)
{
    BCM4343WState *s = BCM4343W_SDIO(card);
    return s->powered && !s->writing && s->position < s->length;
}

static bool bcm_inserted(SDState *card)
{
    return BCM4343W_SDIO(card)->powered;
}

static bool bcm_readonly(SDState *card)
{
    return false;
}

static void bcm_voltage(SDState *card, uint16_t voltage)
{
    /* Powered from WL_REG_ON; the SD bus signalling voltage is independent. */
}

static void bcm_core(BCM4343WState *s, unsigned *pos, unsigned core,
                     unsigned rev, unsigned index)
{
    uint8_t *erom = s->bp + BCM_EROM - BCM_BP_BASE;
    stl_le_p(erom + (*pos)++ * 4, 0x4bf00001 | (core << 8));
    stl_le_p(erom + (*pos)++ * 4, (rev << 24) | (1 << 19) | (1 << 9) | 1);
    stl_le_p(erom + (*pos)++ * 4, BCM_BP_BASE + index * 0x1000 + 5);
    stl_le_p(erom + (*pos)++ * 4, 0x18100000 + index * 0x1000 + 0x85);
    stl_le_p(s->bp + 0x100000 + index * 0x1000 + 0x408, 1);
}

static void bcm_reset(DeviceState *dev)
{
    BCM4343WState *s = BCM4343W_SDIO(dev);
    unsigned pos = 0;
    timer_del(s->scan_timer);
    timer_del(s->join_timer);
    g_queue_clear_full(&s->rx, g_free);
    g_hash_table_remove_all(s->iovars);
    memset(s->ram, 0, BCM_RAM_SIZE);
    memset(s->bp, 0, BCM_BP_SIZE);
    memset(s->cccr, 0, sizeof(s->cccr));
    memset(s->f1, 0, sizeof(s->f1));
    s->window = BCM_BP_BASE;
    s->f1[0x0c] = 0x18;
    s->f1[0x1f] = 3;
    s->intstatus = s->hostintmask = s->mailbox = 0;
    s->length = s->position = s->rx_position = 0;
    s->rx_seq = s->tx_seq = 0;
    s->rx_packets = s->tx_packets = s->power_mode = 0;
    s->selected = s->firmware_running = s->associated = s->up = false;
    s->software_oob = s->wlfc = false;
    s->cccr[0] = 0x32;
    s->cccr[1] = 3;
    s->cccr[8] = 0x1e;
    s->cccr[0x0a] = 0x10;
    s->cccr[0x13] = 1;
    for (unsigned fn = 1; fn <= 2; fn++) {
        unsigned base = fn * 0x100;
        s->cccr[base] = 0; /* vendor interface, as matched by DHD */
        stl_le_p(s->cccr + base + 9, 0x1000 + base);
        stw_le_p(s->cccr + base + 0x10, 512);
    }
    stl_le_p(s->bp, 0x1501a9a6); /* AI, five cores, BCM43430 revision 1 */
    stl_le_p(s->bp + 4, 0x10000000); /* PMU */
    stl_le_p(s->bp + 0x2c, 1);
    stl_le_p(s->bp + 0xfc, BCM_EROM);
    stl_le_p(s->bp + 0x604, 17); /* PMU revision */
    stl_le_p(s->bp + 0x608, 0x13c); /* LPO, ALP and HT available; HT selected */
    stl_le_p(s->bp + 0x1e0, 0x30000);
    bcm_core(s, &pos, 0x800, 48, 0); /* chipcommon */
    bcm_core(s, &pos, 0x812, 39, 1); /* 802.11 MAC */
    bcm_core(s, &pos, 0x829, 24, 2); /* SDIO */
    bcm_core(s, &pos, 0x82a, 3, 3); /* ARM CM3 */
    bcm_core(s, &pos, 0x80e, 17, 4); /* SOCRAM */
    stl_le_p(s->bp + BCM_EROM - BCM_BP_BASE + pos * 4, 0xf);
    stl_le_p(s->bp + 0x4000, 8 << 4); /* eight RAM banks */
    stl_le_p(s->bp + 0x4040, 7); /* 8 * 8192 bytes per bank */
    stl_le_p(s->bp + 0x2000, 5); /* CIS and F2 ready */
    stl_le_p(s->bp + 0x21e0, 0x30000);
    bcm_update_irq(s);
}

static void bcm_power(void *opaque, int line, int level)
{
    BCM4343WState *s = opaque;
    if (s->powered != !!level) {
        s->powered = !!level;
        trace_bcm4343w_power(s->powered);
        bcm_reset(DEVICE(s));
        sdbus_set_inserted(SD_BUS(qdev_get_parent_bus(DEVICE(s))), s->powered);
    }
}

static bool bcm_can_receive(NetClientState *nc)
{
    BCM4343WState *s = qemu_get_nic_opaque(nc);
    unsigned reserve = 8;

    /* Every outstanding transmit needs a completion, even under RX load. */
    if (s->wlfc) {
        reserve += BCM_WLFC_FIFO_COUNT * BCM_WLFC_FIFO_CREDITS;
    }
    return s->powered && s->associated && s->rx.length < BCM_QUEUE_MAX - reserve;
}

static ssize_t bcm_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    BCM4343WState *s = qemu_get_nic_opaque(nc);
    uint8_t packet[BCM_FRAME_MAX - 12] = { 0x20, 0, 0, 0 };
    if (size > sizeof(packet) - 4) {
        return size;
    }
    if (!bcm_can_receive(nc)) {
        return 0;
    }
    memcpy(packet + 4, buf, size);
    s->rx_packets++;
    bcm_enqueue(s, 2, packet, size + 4);
    return size;
}

static NetClientInfo bcm_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = bcm_can_receive,
    .receive = bcm_receive,
};

static void bcm_realize(DeviceState *dev, Error **errp)
{
    BCM4343WState *s = BCM4343W_SDIO(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&bcm_net_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void bcm_init(Object *obj)
{
    BCM4343WState *s = BCM4343W_SDIO(obj);
    static const uint8_t common_cis[] = {
        0x20, 4, 0xd0, 2, 0xa6, 0xa9,
        0x21, 2, 0x0c, 0,
        0x22, 4, 0, 0, 2, 0x32,
        0xff,
    };
    s->ram = g_malloc0(BCM_RAM_SIZE);
    s->bp = g_malloc0(BCM_BP_SIZE);
    g_queue_init(&s->rx);
    s->iovars = g_hash_table_new_full(g_str_hash, g_str_equal, g_free,
                                     (GDestroyNotify)g_bytes_unref);
    s->scan_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bcm_scan_done, s);
    s->join_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bcm_join_done, s);
    qdev_init_gpio_in_named(DEVICE(s), bcm_power, "power", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, "irq", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->oob_irq, "oob-irq", 1);
    memset(s->cis, 0xff, sizeof(s->cis));
    memcpy(s->cis, common_cis, sizeof(common_cis));
    for (unsigned fn = 1; fn <= 2; fn++) {
        uint8_t *cis = s->cis + fn * 0x100;
        memcpy(cis, common_cis, 10);
        cis[10] = 0x22;
        cis[11] = 42;
        memset(cis + 12, 0, 42);
        cis[12] = 1;
        stw_le_p(cis + 24, 512);
        stw_le_p(cis + 40, 100);
        cis[54] = 0xff;
    }
}

static void bcm_finalize(Object *obj)
{
    BCM4343WState *s = BCM4343W_SDIO(obj);
    timer_free(s->scan_timer);
    timer_free(s->join_timer);
    g_queue_clear_full(&s->rx, g_free);
    g_hash_table_unref(s->iovars);
    g_free(s->ram);
    g_free(s->bp);
}

static void bcm_unrealize(DeviceState *dev)
{
    BCM4343WState *s = BCM4343W_SDIO(dev);
    qemu_del_nic(s->nic);
}

static const Property bcm_properties[] = {
    DEFINE_NIC_PROPERTIES(BCM4343WState, conf),
};

static const VMStateDescription bcm_vmstate = {
    .name = TYPE_BCM4343W_SDIO,
    .unmigratable = true,
};

static void bcm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);
    dc->realize = bcm_realize;
    dc->unrealize = bcm_unrealize;
    dc->vmsd = &bcm_vmstate;
    dc->desc = "Broadcom BCM4343W SDIO Wi-Fi";
    device_class_set_legacy_reset(dc, bcm_reset);
    device_class_set_props(dc, bcm_properties);
    sc->do_command = bcm_command;
    sc->write_byte = bcm_write_byte;
    sc->write_data = bcm_write_data;
    sc->read_byte = bcm_read_byte;
    sc->read_data = bcm_read_data;
    sc->receive_ready = bcm_receive_ready;
    sc->data_ready = bcm_data_ready;
    sc->get_inserted = bcm_inserted;
    sc->get_readonly = bcm_readonly;
    sc->set_voltage = bcm_voltage;
}

static const TypeInfo bcm_type = {
    .name = TYPE_BCM4343W_SDIO,
    .parent = TYPE_SDMMC_COMMON,
    .instance_size = sizeof(BCM4343WState),
    .instance_init = bcm_init,
    .instance_finalize = bcm_finalize,
    .class_init = bcm_class_init,
};

static void bcm_register_types(void)
{
    type_register_static(&bcm_type);
}

type_init(bcm_register_types)
