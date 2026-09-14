/*
 * Bounded MT8171 MCU bootstrap command protocol, not firmware execution.
 * Packet contracts reconstructed from stock wmt_drv.ko .data templates,
 * wmt_core_reg_rw_raw, wmt_core_stp_init and mtk_wcn_stp_send_data.
 * Implements digital identity/register, wake/sleep and transport settings.
 * Calibration exchanges explicitly synthetic emulator records; RF physics,
 * function-on and radio traffic are unsupported.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/misc/mt8171-btif.h"
#include "qemu/bswap.h"
#include "qemu/log.h"

void mt8171_btif_protocol_reset(MT8171BtifState *s)
{
    s->ingress_used = s->loopback_head = s->loopback_used = 0;
    s->stp_config = 0x11; /* WMT_QUERY_STP_EVT_DEFAULT */
    s->baud_config = 115200; /* WMT_QUERY_BAUD_EVT_115200 */
    s->mcu_chip_id = s->mcu_adie_id = 0;
    memset(s->mcu_config, 0, sizeof(s->mcu_config));
    memset(s->calibration_bt, 0, sizeof(s->calibration_bt));
    memset(s->calibration_wifi, 0, sizeof(s->calibration_wifi));
    s->calibration_valid = s->thermal_enabled = false;
    s->utc_seconds = s->utc_microseconds = 0;
    s->utc_sync_ns = 0;
    s->utc_valid = s->display_blank = false;
    s->wake_low_ns = 0;
    s->wake_low_armed = s->hardware_wake_pending = false;
    s->firmware_awake = true;
    s->protocol_blocked = false;
    s->stp_active = false;
    s->stp_full = false;
    s->stp_rx_sequence = s->stp_tx_sequence = 0;
    s->stp_previous_request_size = s->stp_previous_reply_size = 0;
    s->last_request_size = 0;
    s->protocol_resets++;
}

static uint16_t stp_crc(const uint8_t *data, unsigned size)
{
    /* osal_crc16: initial zero, reflected polynomial0xa001, no final xor. */
    uint16_t crc = 0;
    for (unsigned i = 0; i < size; i++) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
        }
    }
    return crc;
}

static void enqueue_reply(MT8171BtifState *s, const uint8_t *packet,
                          unsigned size)
{
    for (unsigned i = 0; i < size; i++) {
        s->loopback[(s->loopback_head + s->loopback_used++) % sizeof(s->loopback)] = packet[i];
    }
}

static void consume_request(MT8171BtifState *s, unsigned size)
{
    s->ingress_used -= size;
    memmove(s->ingress, s->ingress + size, s->ingress_used);
}

/* These commands load board configuration into MCU state. They do not turn
 * on an RF function or perform calibration. Layouts are the stock SoC init
 * tables and dynamically constructed packets in mtk_wcn_soc_sw_init,
 * wmt_stp_init_coex. Keep opaque board bytes for the eventual radio model. */
static bool configure_board(MT8171BtifState *s, const uint8_t *p, unsigned size)
{
    bool valid = false;
    if (s->mcu_chip_id != 0x8171 || s->mcu_adie_id != 0x6631) {
        return false;
    }
    if (p[1] == 2) {
        switch (p[4]) {
        case 0x0d: /* BT TSSI from Wi-Fi: enable and LE16 target */
            valid = size == 8;
            break;
        case 0x0e: /* external PA configuration */
            valid = size == 6;
            break;
        case 0x14: /* Wi-Fi board configuration: byte count then data */
            valid = size >= 6 && size == 6U + p[5];
            break;
        }
    } else if (p[1] == 0x10) {
        switch (p[4]) {
        case 1: /* coexistence setting */
        case 6: /* SoC coexistence setting selected by wmt_stp_init_coex */
            valid = size == 6;
            break;
        case 0x0d: /* external component selection */
            valid = size == 7;
            break;
        case 0x1a: /* Wi-Fi path, or three-byte BT-control policy */
            valid = size == 7 || size == 8;
            break;
        case 0x1b: /* LNA gains, or three-byte opportunity-time policy */
            valid = size == 22 || size == 8;
            break;
        case 0x1c: /* BLE scan-time policy */
        case 0x1e: /* remapped BT-control policy */
        case 0x1f: /* remapped opportunity-time policy */
        case 0x20: /* remapped BLE scan-time policy */
            valid = size == 8;
            break;
        case 0x1d: /* EPA/ELNA configuration blob, or one-byte EPA mode */
            valid = size >= 6 && size <= 261;
            break;
        }
    } else if (p[1] == 0x14) {
        valid = p[4] == 7 && size == 8; /* antenna swap: three DT bytes */
    } else if (p[1] == 5) {
        valid = p[4] == 2 && size == 6; /* FM communication strap */
    } else if (p[1] == 0x0a) {
        valid = p[4] == 8 && size == 6; /* shared oscillator mode */
    } else if (p[1] == 0x0f) {
        valid = p[4] == 4 && size == 11; /* coredump level/configuration */
    }
    if (!valid || size - 5 > sizeof(s->mcu_config[0].data)) {
        return false;
    }
    MT8171McuConfig *entry = NULL;
    for (unsigned i = 0; i < ARRAY_SIZE(s->mcu_config); i++) {
        MT8171McuConfig *candidate = &s->mcu_config[i];
        if (candidate->opcode == p[1] && candidate->selector == p[4]) {
            entry = candidate;
            break;
        }
        if (!candidate->opcode && !entry) {
            entry = candidate;
        }
    }
    if (!entry) {
        return false;
    }
    entry->opcode = p[1];
    entry->selector = p[4];
    entry->length = size - 5;
    memcpy(entry->data, p + 5, entry->length);
    s->configuration_writes++;
    return true;
}

static bool bootstrap_selected(MT8171BtifState *s)
{
    uint32_t identity;
    return s->mcu_chip_id == 0x8171 && s->mcu_adie_id == 0x6631 &&
           mt8171_consys_mcu_adie(s->consys, &identity) && identity == 0x6631;
}

void mt8171_btif_protocol_hardware_wake(MT8171BtifState *s)
{
    uint8_t packet[12] = {0};
    static const uint8_t event[6] = {2, 3, 2, 0, 0, 3};
    if (!s->hardware_wake_pending || s->protocol_blocked ||
        !bootstrap_selected(s) || !s->stp_active ||
        sizeof(s->loopback) - s->loopback_used < sizeof(packet)) {
        return;
    }
    /* A CONSYS wake edge is not an AP STP packet: retain the last received
     * sequence in ACK, and advance only the MCU transmit sequence. Fullset
     * opfunc_pwr_sv receives the same six-byte WMT_WAKEUP_EVT as raw-ff mode. */
    packet[0] = s->stp_full ? 0x80 | (s->stp_tx_sequence << 3) |
                ((s->stp_rx_sequence - 1) & 7) : 0x80;
    packet[1] = 0x40;
    packet[2] = sizeof(event);
    memcpy(packet + 4, event, sizeof(event));
    if (s->stp_full) {
        packet[3] = packet[0] + packet[1] + packet[2];
        stw_le_p(packet + 10, stp_crc(event, sizeof(event)));
        s->stp_tx_sequence = (s->stp_tx_sequence + 1) & 7;
    }
    enqueue_reply(s, packet, sizeof(packet));
    s->hardware_wake_pending = false;
    s->firmware_awake = true;
    s->wake_commands++;
    s->hardware_wake_events++;
}

static bool calibration_record(MT8171BtifState *s, uint8_t *record)
{
    /* Calibration consumes the actual antenna configuration. Other board
     * configuration is optional in the shipped sw_init tables. Preserve its
     * contribution in this synthetic record, so stale backup data requests
     * recalibration after a board configuration change. */
    MT8171McuConfig *antenna = NULL;
    uint16_t configuration = 0;
    for (unsigned i = 0; i < ARRAY_SIZE(s->mcu_config); i++) {
        MT8171McuConfig *entry = &s->mcu_config[i];
        if (entry->opcode == 0x14 && entry->selector == 7) { antenna = entry; }
        if (entry->opcode == 2 || entry->opcode == 0x10) {
            configuration ^= stp_crc(entry->data, entry->length) ^
                             (entry->opcode << 8) ^ entry->selector;
        }
    }
    if (!bootstrap_selected(s) || !antenna || antenna->length != 3) {
        return false;
    }
    memset(record, 0, MT8171_SYNTH_CAL_SIZE);
    memcpy(record, "QEMU-SYNTH-CAL", 14);
    record[14] = 1; /* model record version */
    stw_le_p(record + 16, s->mcu_chip_id);
    stw_le_p(record + 18, s->mcu_adie_id);
    memcpy(record + 20, antenna->data, 3);
    record[23] = 25; /* deterministic emulator temperature, degrees C */
    stw_le_p(record + 24, configuration);
    stw_le_p(record + 30, stp_crc(record, 30));
    return true;
}

static unsigned calibration(MT8171BtifState *s, const uint8_t *p,
                            unsigned size, uint8_t *reply)
{
    uint8_t expected[MT8171_SYNTH_CAL_SIZE], memory[MT8171_SYNTH_CAL_SIZE];
    if (!calibration_record(s, expected)) { return 0; }
    stw_le_p(reply + 2, 2);
    reply[5] = p[4];
    switch (p[4]) {
    case 1: /* WMT_CORE_START_RF_CALIBRATION_CMD */
        if (size != 5) { return 0; }
        memcpy(memory, expected, sizeof(memory));
        memory[15] = 1; /* Wi-Fi record, distinct from BT record */
        stw_le_p(memory + 30, stp_crc(memory, 30));
        if (!mt8171_consys_calibration_memory(s->consys, memory,
                                              sizeof(memory), true)) {
            s->dma_faults++;
            return 0;
        }
        memcpy(s->calibration_bt, expected, sizeof(expected));
        memcpy(s->calibration_wifi, memory, sizeof(memory));
        s->calibration_valid = true;
        s->calibration_runs++;
        return 6;
    case 2: /* Restore actual BT bytes supplied by the host. */
        if (size < 7 || size != 7U + lduw_le_p(p + 5)) { return 0; }
        if (size != 7 + sizeof(expected) ||
            memcmp(p + 7, expected, sizeof(expected))) {
            /* The shipped host explicitly handles RECAL and runs START. */
            reply[4] = 1;
            s->calibration_valid = false;
            return 6;
        }
        memcpy(s->calibration_bt, p + 7, sizeof(expected));
        memcpy(s->calibration_wifi, expected, sizeof(expected));
        s->calibration_wifi[15] = 1;
        stw_le_p(s->calibration_wifi + 30, stp_crc(s->calibration_wifi, 30));
        /* Wi-Fi EMI is restored separately by the stock host's
         * mtk_wcn_soc_restore_wifi_cal_result, not by this BT command. */
        s->calibration_valid = true;
        s->calibration_restores++;
        return 6;
    case 3: /* GET: BT blob followed by Wi-Fi descriptor into real EMI. */
        if (size != 5 || !s->calibration_valid) { return 0; }
        if (!mt8171_consys_calibration_memory(s->consys, memory,
                                              sizeof(memory), false)) {
            s->dma_faults++;
            return 0;
        }
        if (memcmp(memory, s->calibration_wifi, sizeof(memory))) { return 0; }
        stw_le_p(reply + 2, sizeof(expected) + 14);
        stw_le_p(reply + 6, sizeof(expected));
        memcpy(reply + 8, s->calibration_bt, sizeof(expected));
        /* The two descriptor bytes before offset/length are opaque to the
         * shipped host. Encode the following eight-byte descriptor size. */
        stw_le_p(reply + 8 + sizeof(expected), 8);
        stl_le_p(reply + 10 + sizeof(expected), MT8171_SYNTH_CAL_OFFSET);
        stl_le_p(reply + 14 + sizeof(expected), sizeof(memory));
        s->calibration_backups++;
        return sizeof(expected) + 18;
    default:
        return 0;
    }
}

/* Return the actual length of a supported event; zero is unsupported.
 * No generic opcode->success rule exists: each accepted opcode performs
 * a concrete state transition or reads a modeled register. */
static unsigned command(MT8171BtifState *s, const uint8_t *p, unsigned size,
                        uint8_t *reply)
{
    if (size == 1 && p[0] == 0xff) {
        s->firmware_awake = true;
        s->wake_commands++;
        memcpy(reply, "\x02\x03\x02\x00\x00\x03", 6);
        return 6;
    }
    if (size < 5 || p[0] != 1 || lduw_le_p(p + 2) + 4 != size) { return 0; }
    if (!s->firmware_awake && !(p[1] == 3 && p[4] == 2)) { return 0; }
    reply[0] = 2;
    reply[1] = p[1];
    reply[4] = 0; /* status is success only after a supported operation */
    switch (p[1]) {
    case 2:
        if (p[4] == 0x11 && size == 9) { /* WMT_SET_CHIP_ID_CMD */
            uint32_t identity = 0;
            if (!mt8171_consys_mcu_reg(s->consys, 0x80000008,
                                      &identity, UINT32_MAX, false) ||
                ldl_le_p(p + 5) != identity) {
                return 0;
            }
            s->mcu_chip_id = identity;
            s->chip_id_sets++;
            stw_le_p(reply + 2, 1);
            return 5;
        }
        if (p[4] == 0x12 && size == 5) { /* WMT_QUERY_A_DIE_CMD */
            uint32_t identity;
            if (s->mcu_chip_id != 0x8171 ||
                !mt8171_consys_mcu_adie(s->consys, &identity)) {
                return 0;
            }
            s->mcu_adie_id = identity;
            s->adie_queries++;
            /* The shipped event template is the five-byte prefix
             * 02 02 05 00 00. Supply its four identity bytes too; WMT
             * compares the prefix and flushes unread bytes on next TX. */
            stw_le_p(reply + 2, 5);
            stl_le_p(reply + 5, identity);
            return 9;
        }
        if (!configure_board(s, p, size)) { return 0; }
        stw_le_p(reply + 2, 1);
        return 5;
    case 3: /* WMT_SLEEP_CMD / WMT_HOST_AWAKE_CMD */
        if (size != 5 || (p[4] != 1 && p[4] != 2)) { return 0; }
        s->firmware_awake = p[4] == 2;
        if (s->firmware_awake) { s->wake_commands++; }
        else { s->sleep_commands++; }
        stw_le_p(reply + 2, 2);
        reply[5] = p[4];
        return 6;
    case 4: /* Serial transport configuration; no RF state. */
        reply[5] = p[4];
        if ((p[4] == 1 || p[4] == 3) && size == 9) {
            if (p[4] == 1) { s->baud_config = ldl_le_p(p + 5); }
            else { s->stp_config = ldl_le_p(p + 5); }
            stw_le_p(reply + 2, 2);
            return 6;
        }
        if ((p[4] == 2 || p[4] == 4) && size == 5) {
            stw_le_p(reply + 2, 6);
            stl_le_p(reply + 6, p[4] == 2 ? s->baud_config : s->stp_config);
            return 10;
        }
        return 0;
    case 5: /* WMT_STRAP_CONF_CMD_FM_COMM, echoed communication strap. */
        if (!configure_board(s, p, size)) { return 0; }
        stw_le_p(reply + 2, 2);
        reply[5] = p[5];
        return 6;
    case 6: /* wmt_core_func_ctrl_cmd: explicit unavailable-radio result. */
        if (!bootstrap_selected(s) || size != 6 || p[5] > 1 ||
            (p[4] != 0 && p[4] != 3)) { return 0; }
        /* Only the BT(0) and WLAN(3) identities are established here. Neither
         * radio execution engine is supplied by this BTIF model. Disabling
         * an already-disabled function succeeds; enabling returns the stock
         * nonzero status instead of a timeout or invented ready state. The
         * SoC WLAN path uses its separate AXI HIF model, not this command. */
        s->function_control_requests++;
        reply[4] = p[5];
        if (p[5]) { s->unavailable_function_requests++; }
        stw_le_p(reply + 2, 1);
        return 5;
    case 8: { /* Single masked register access, WMT_SET_REG_CMD. */
        uint32_t address, value, mask;
        bool write = p[4] == 1;
        if (size != 20 || (p[4] != 1 && p[4] != 2) ||
            p[5] != 1 || p[6] != 0 || p[7] != 1) { return 0; }
        address = ldl_le_p(p + 8);
        value = ldl_le_p(p + 12);
        mask = ldl_le_p(p + 16);
        if (!mt8171_consys_mcu_reg(s->consys, address, &value, mask, write)) {
            return 0;
        }
        stw_le_p(reply + 2, write ? 4 : 12);
        reply[5] = write ? 0 : p[4];
        reply[6] = 0;
        reply[7] = 1;
        if (!write) {
            stl_le_p(reply + 8, address);
            stl_le_p(reply + 12, value);
        }
        return write ? 8 : 16;
    }
    case 0x0a:
    case 0x0f:
    case 0x10:
        if (!configure_board(s, p, size)) { return 0; }
        stw_le_p(reply + 2, 1);
        return 5;
    case 0x11: /* opfunc_therm_ctrl: enable, read, disable. */
        if (!bootstrap_selected(s) || size != 5) { return 0; }
        if (p[4] == 1 || p[4] == 3) {
            s->thermal_enabled = p[4] == 1;
            stw_le_p(reply + 2, 1);
            return 5;
        }
        if (p[4] != 2 || !s->thermal_enabled) { return 0; }
        stw_le_p(reply + 2, 2);
        reply[5] = 25;
        return 6;
    case 0x13: { /* WMT_GET_SOC_ADIE_CHIPID_CMD, TOP identity only. */
        uint32_t identity;
        if (!bootstrap_selected(s) || size != 8 ||
            memcmp(p + 4, "\x02\x04\x24\x00", 4) ||
            !mt8171_consys_mcu_adie(s->consys, &identity)) { return 0; }
        stw_le_p(reply + 2, 9);
        memcpy(reply + 5, p + 4, 4);
        /* TOPSPI address 0x24 reads 0x66310000; sw_init copies the
         * identity from event bytes11..12. Echo metadata is inferred. */
        stl_le_p(reply + 9, identity << 16);
        return 13;
    }
    case 0x14:
        if (p[4] == 7) {
            if (!configure_board(s, p, size)) { return 0; }
            stw_le_p(reply + 2, 2);
            reply[5] = 7;
            s->calibration_valid = false;
            return 6;
        }
        return calibration(s, p, size, reply);
    case 0xf0: /* Dedicated firmware-log time and AP display state. */
        if (!bootstrap_selected(s)) { return 0; }
        if (p[4] == 2 && size == 13) {
            /* connsys_dedicated_log_get_utc_time provides LE32 seconds and
             * microseconds; preserve their anchor to virtual time. */
            if (ldl_le_p(p + 9) >= 1000000) { return 0; }
            s->utc_seconds = ldl_le_p(p + 5);
            s->utc_microseconds = ldl_le_p(p + 9);
            s->utc_sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->utc_valid = true;
            s->utc_syncs++;
        } else if (p[4] == 3 && size == 6 && p[5] <= 1) {
            s->display_blank = p[5];
            s->blank_updates++;
        } else {
            return 0;
        }
        /* Unlike ordinary WMT events, f0 puts selector before status.
         * Exact UTC template: 02 f0 02 00 02 00. The blank diagnostic
         * expects 02 f0 02 00 03; status occupies its final byte. */
        stw_le_p(reply + 2, 2);
        reply[4] = p[4];
        reply[5] = 0;
        return 6;
    default:
        return 0;
    }
}

void mt8171_btif_protocol_process(MT8171BtifState *s)
{
    while (s->ingress_used && !s->protocol_blocked) {
        const uint8_t *p = s->ingress;
        uint8_t reply[96] = {0}, packet[104] = {0};
        unsigned payload, total, prefix = 0, reply_size;
        bool framed = (p[0] & 0x80) && p[0] != 0xff;
        bool full = false;
        unsigned sequence = 0;

        if (framed) {
            if (s->ingress_used < 4) { return; }
            payload = ((p[1] & 0xf) << 8) | p[2];
            total = payload + 6;
            prefix = 4;
            full = s->stp_full || p[3] != 0;
            if (full && (uint8_t)(p[0] + p[1] + p[2]) != p[3]) { goto reject; }
            /* stp_send_ack sends four bytes, no payload or CRC trailer. */
            if (full && payload == 0 && p[1] == 0) {
                s->stp_acks++;
                consume_request(s, 4);
                continue;
            }
            if ((p[1] >> 4) != 4 || payload > 1996) { goto reject; }
            if (s->ingress_used < total) { return; }
            if (full ? stp_crc(p + 4, payload) != lduw_le_p(p + total - 2) :
                       (p[total - 1] || p[total - 2])) { goto reject; }
            s->stp_active = true;
            if (full) {
                sequence = (p[0] >> 3) & 7;
                if (sizeof(s->loopback) - s->loopback_used < sizeof(packet)) { return; }
                if (sequence != s->stp_rx_sequence) {
                    /* A retry is acknowledged with the exact saved event,
                     * never by reapplying masked writes or state changes. */
                    if (sequence != ((s->stp_rx_sequence - 1) & 7) ||
                        payload != s->stp_previous_request_size ||
                        memcmp(p + 4, s->stp_previous_request, payload)) { goto reject; }
                    enqueue_reply(s, s->stp_previous_reply, s->stp_previous_reply_size);
                    s->stp_replays++;
                    consume_request(s, total);
                    continue;
                }
                s->stp_full = true;
            }
        } else if (p[0] == 0xff) {
            payload = total = 1;
        } else {
            if (s->ingress_used < 4) { return; }
            payload = total = lduw_le_p(p + 2) + 4;
            if (payload > 2000 || p[0] != 1) { goto reject; }
            if (s->ingress_used < total) { return; }
        }
        /* Reserve enough RX room before any command changes state. */
        if (sizeof(s->loopback) - s->loopback_used < sizeof(packet)) { return; }
        s->last_request_size = MIN(payload, sizeof(s->last_request));
        memcpy(s->last_request, p + prefix, s->last_request_size);
        reply_size = command(s, p + prefix, payload, reply);
        if (!reply_size) { goto reject; }
        /* Wake is sent as one raw0xff byte even with STP enabled. Its
         * event then returns over the already-selected STP transport. */
        if (framed || (payload == 1 && s->stp_active)) {
            bool full_reply = full || s->stp_full;
            packet[0] = full_reply ?
                0x80 | (s->stp_tx_sequence << 3) |
                (framed ? sequence : ((s->stp_rx_sequence - 1) & 7)) : 0x80;
            packet[1] = 0x40;
            packet[2] = reply_size;
            memcpy(packet + 4, reply, reply_size);
            if (full_reply) {
                packet[3] = packet[0] + packet[1] + packet[2];
                stw_le_p(packet + 4 + reply_size, stp_crc(reply, reply_size));
                s->stp_tx_sequence = (s->stp_tx_sequence + 1) & 7;
            }
            reply_size += 6;
        } else {
            memcpy(packet, reply, reply_size);
        }
        if (full) {
            memcpy(s->stp_previous_request, p + prefix, payload);
            s->stp_previous_request_size = payload;
            memcpy(s->stp_previous_reply, packet, reply_size);
            s->stp_previous_reply_size = reply_size;
            s->stp_rx_sequence = (sequence + 1) & 7;
        }
        enqueue_reply(s, packet, reply_size);
        s->protocol_commands++;
        consume_request(s, total);
        continue;
reject:
        s->protocol_blocked = true;
        s->protocol_rejections++;
        s->last_request_size = MIN(s->ingress_used, sizeof(s->last_request));
        memcpy(s->last_request, s->ingress, s->last_request_size);
        qemu_log_mask(LOG_UNIMP,
                      "mt8171-btif: unsupported MCU command/framing %02x %02x (%u queued bytes)\n",
                      p[0], s->ingress_used > 1 ? p[1] : 0, s->ingress_used);
    }
}
