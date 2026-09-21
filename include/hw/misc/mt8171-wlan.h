/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_MT8171_WLAN_H
#define HW_MISC_MT8171_WLAN_H
#include "hw/core/sysbus.h"
#include "hw/misc/mt8171-consys.h"
#include "qemu/timer.h"
#include "net/net.h"
#define TYPE_MT8171_WLAN "mt8171-wlan"
OBJECT_DECLARE_SIMPLE_TYPE(MT8171WlanState, MT8171_WLAN)
typedef struct MT8171WlanSection {
    uint32_t address, length, mode, received;
    uint8_t *bytes; /* raw download stream, not decrypted executable RAM */
} MT8171WlanSection;
/* Virtual configuration capacity, not an assertion about physical RF limits.
 * The stock driver reserves WTBL0..3 and uses the last slot as its default. */
#define MT8171_WLAN_BSS 4
#define MT8171_WLAN_CONTEXTS (MT8171_WLAN_BSS + 1) /* extra P2P device context */
#define MT8171_WLAN_WTBL 32
typedef struct MT8171WlanRuntime {
    uint8_t domain[2][56], power_limits[1472], nvram[2048], dbdc[36];
    uint8_t rrm[MT8171_WLAN_CONTEXTS][44];
    uint8_t network[MT8171_WLAN_CONTEXTS][12];
    uint8_t network_addresses[MT8171_WLAN_CONTEXTS][68];
    uint8_t multicast[MT8171_WLAN_CONTEXTS][200];
    uint8_t power_save[MT8171_WLAN_CONTEXTS];
    uint8_t host_suspend[MT8171_WLAN_CONTEXTS][68];
    uint8_t station_cleanup[4];
    bool calibration_notified;
    uint8_t wtbl[MT8171_WLAN_WTBL][8]; /* active, context, local MAC */
    uint32_t packet_filter;
    uint8_t settings[256][68]; /* typed key/value records, replaced by key */
    uint8_t chip_config[328];
    uint8_t force_rts[4];
    uint8_t performance[80];
    uint64_t setting_count;
} MT8171WlanRuntime;
struct MT8171WlanState {
    SysBusDevice parent_obj;
    MemoryRegion iomem, identity_iomem, sync_iomem, ownership_iomem;
    MT8171ConsysState *consys;
    NICConf conf;
    NICState *nic;
    uint8_t rx_queue[64][2048];
    uint16_t rx_length[64];
    uint8_t rx_ring[64];
    unsigned rx_head, rx_count;
    uint8_t station_index, wlan_index, context;
    bool associated;
    uint16_t frame_sequence;
    uint64_t rx_packets, tx_packets;
    uint32_t regs[0x1000 / 4];
    QEMUTimer *dma_timer, *protect_timer, *startup_timer, *scan_timer;
    QEMUTimer *beacon_loss_timer;
    qemu_irq irq;
    bool unsupported_reported, firmware_owns, reset_held;
    uint64_t subsystem_resets;
    uint64_t virtual_mcu_state; /* 0 off/reset,1 starting,2 running,3 stopping */
    uint32_t start_flags, start_address;
    uint8_t mac[6], basic_config[12], log_config[284];
    MT8171WlanRuntime runtime;
    uint8_t scan_request[1192], scan_channels[64][2];
    unsigned scan_count, scan_index, scan_dwell_ms;
    int64_t scan_deadline_ns, scan_channel_end_ns;
    bool scan_active, scan_event_pending, scan_timed_out;
    uint8_t scheduled_scan[1224];
    bool scheduled_scan_enabled;
    uint64_t scan_requests, scan_completions, scan_cancels, scanned_channels;
    uint64_t station_removals;
    uint64_t calibration_notifications, host_suspend_notifications;
    uint64_t configuration_commands, virtual_stops;
    uint64_t virtual_starts, runtime_commands, capability_queries, ownership_changes;
    uint64_t tx_commands, rx_events, dma_faults, unsupported_commands;
    uint64_t protection_transitions, identity_reads;
    MT8171WlanSection sections[32];
    unsigned section_count, staging_size;
    uint64_t download_configs, download_bytes, download_chunks, download_sections;
};
#endif
