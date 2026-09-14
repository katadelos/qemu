/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_MT8171_BTIF_H
#define HW_MISC_MT8171_BTIF_H
#include "hw/core/sysbus.h"
#include "hw/misc/mt8171-consys.h"
#define TYPE_MT8171_BTIF "mt8171-btif"
#define MT8171_SYNTH_CAL_SIZE 32
OBJECT_DECLARE_SIMPLE_TYPE(MT8171BtifState, MT8171_BTIF)
typedef struct MT8171BtifBank {
    MemoryRegion iomem;
    struct MT8171BtifState *owner;
    unsigned index;
    uint32_t regs[0x1000 / 4];
} MT8171BtifBank;
typedef struct MT8171McuConfig {
    uint8_t opcode, selector;
    uint16_t length;
    uint8_t data[256];
} MT8171McuConfig;
struct MT8171BtifState {
    SysBusDevice parent_obj;
    MT8171BtifBank bank[3];
    MT8171ConsysState *consys;
    qemu_irq irq[3];
    /* Bounded MCU ingress queue. Only a protocol consumer may remove bytes;
     * a full queue backpressures DMA instead of silently discarding data. */
    uint8_t ingress[65536], loopback[8192];
    unsigned ingress_used, loopback_head, loopback_used;
    uint64_t tx_bytes, rx_bytes, dma_faults;
    uint64_t protocol_commands, protocol_rejections, protocol_resets;
    uint64_t last_rom_boot;
    uint32_t stp_config, baud_config;
    uint32_t mcu_chip_id, mcu_adie_id;
    MT8171McuConfig mcu_config[24];
    uint64_t chip_id_sets, adie_queries, configuration_writes;
    /* Explicit emulator calibration records, not RF coefficients. */
    uint8_t calibration_bt[MT8171_SYNTH_CAL_SIZE];
    uint8_t calibration_wifi[MT8171_SYNTH_CAL_SIZE];
    bool calibration_valid, thermal_enabled;
    uint64_t calibration_runs, calibration_restores, calibration_backups;
    uint32_t utc_seconds, utc_microseconds;
    int64_t utc_sync_ns;
    bool utc_valid, display_blank;
    uint64_t utc_syncs, blank_updates, sleep_commands, wake_commands;
    uint64_t function_control_requests, unavailable_function_requests;
    int64_t wake_low_ns;
    bool wake_low_armed, hardware_wake_pending;
    uint64_t hardware_wake_events;
    bool firmware_awake, protocol_blocked, stp_active, stp_full;
    uint8_t stp_rx_sequence, stp_tx_sequence;
    uint8_t stp_previous_request[2000], stp_previous_reply[104];
    unsigned stp_previous_request_size, stp_previous_reply_size;
    uint64_t stp_replays, stp_acks;
    uint8_t last_request[512];
    unsigned last_request_size;
};
void mt8171_btif_protocol_reset(MT8171BtifState *s);
void mt8171_btif_protocol_process(MT8171BtifState *s);
void mt8171_btif_protocol_hardware_wake(MT8171BtifState *s);
#endif
