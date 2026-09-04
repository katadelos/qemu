/*
 * MediaTek MT8113 system-on-chip
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_MT8113_H
#define HW_ARM_MT8113_H

#include "hw/core/qdev.h"
#include "hw/display/mt8113_hwtcon.h"
#include "hw/gpio/mt8113_gpio.h"
#include "hw/i2c/mtk_i2c.h"
#include "hw/intc/arm_gicv3.h"
#include "hw/misc/mt8113_gce.h"
#include "hw/misc/mt8113_iommu.h"
#include "hw/sd/mtk-msdc.h"
#include "net/net.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qom/object.h"
#include "target/arm/cpu.h"

#define TYPE_MT8113 "mt8113"
#define TYPE_MT8110 "mt8110"
OBJECT_DECLARE_SIMPLE_TYPE(MT8113State, MT8113)

#define MT8113_NUM_CPUS          2

#define MT8113_RAM_BASE          0x40000000
#define MT8113_RAM_MAX           (2 * GiB)
#define MT8113_SRAM_BASE         0x00100000
#define MT8113_SRAM_SIZE         (1 * MiB)
#define MT8113_UART0_ADDR        0x11002000
#define MT8113_TOPCKGEN_ADDR     0x10000000
#define MT8113_INFRASYS_ADDR     0x10001000
#define MT8113_SCPSYS_ADDR       0x10006000
#define MT8113_TOPRGU_ADDR       0x10007000
#define MT8113_TIMER_ADDR        0x10008000
#define MT8113_APMIXEDSYS_ADDR   0x1000c000
#define MT8113_DVFSRC_ADDR       0x10012000
#define MT8113_MCUCFG_ADDR       0x10200000
#define MT8113_RNG_ADDR          0x1020f000
#define MT8113_GPIO_ADDR         0x10005000
#define MT8113_EINT_ADDR         0x1000b000
#define MT8113_I2C0_ADDR         0x11007000
#define MT8113_I2C0_DMA_ADDR     0x11000080
#define MT8113_I2C1_ADDR         0x10019000
#define MT8113_I2C1_DMA_ADDR     0x11000100
#define MT8113_I2C2_ADDR         0x1001e000
#define MT8113_I2C2_DMA_ADDR     0x11000180
#define MT8113_MSDC0_ADDR        0x11230000
#define MT8113_MSDC0_TOP_ADDR    0x11cd0000
#define MT8113_USBPHY0_ADDR      0x11cc0000
#define MT8113_USBPHY1_ADDR      0x11c40000
#define MT8113_USB_MAC_ADDR      0x11211000
#define MT8113_USB_IPPC_ADDR     0x11213e00
#define MT8113_SVS_ADDR          0x1100b000
#define MT8113_BTIF_ADDR         0x1100c000
#define MT8113_BTIF_TX_DMA_ADDR  0x11000480
#define MT8113_BTIF_RX_DMA_ADDR  0x11000500
#define MT8113_WIFI_ADDR         0x18000000
#define MT8113_WIFI_SIZE         (1 * MiB)
#define MT8113_EFUSE_ADDR        0x11c50000
#define MT8113_USB_MAC_SIZE      0x2e00
#define MT8113_USB_IPPC_SIZE     0x0100
#define MT8113_GIC_DIST_ADDR     0x0c000000
#define MT8113_GIC_REDIST_ADDR   0x0c080000
#define MT8113_HWTCON_ADDR       0x14000000
#define MT8113_HWTCON_IMG_ADDR   0x15000000
#define MT8113_GCE_ADDR          0x10238000
#define MT8113_IOMMU_ADDR        0x10209000

#define MT8113_NUM_SPIS          224
#define MT8113_UART0_IRQ         24
#define MT8113_I2C0_IRQ          23
#define MT8113_I2C1_IRQ          127
#define MT8113_I2C2_IRQ          117
#define MT8113_MSDC0_IRQ         20
#define MT8113_TIMER_IRQ         85
#define MT8113_GCE_IRQ           82
#define MT8113_IOMMU_IRQ         61
#define MT8113_EINT_IRQ          86
#define MT8113_SVS_IRQ           45
#define MT8113_BTIF_IRQ          39
#define MT8113_BTIF_TX_DMA_IRQ   35
#define MT8113_BTIF_RX_DMA_IRQ   36
#define MT8113_WIFI_IRQ          132
#define MT8113_USB_IRQ           16
#define MT8113_MDP_WROT_IRQ      150

#define MT8113_HWTCON_WB_FRAME_DONE_SPI         172
#define MT8113_HWTCON_WF_LUT_FRAME_DONE_SPI     184
#define MT8113_HWTCON_WF_LUT_RELEASE_SPI        183
#define MT8113_HWTCON_DPI_UPDATE_DONE_SPI       195
#define MT8113_HWTCON_TCON_END_SPI              197
#define MT8113_HWTCON_PIXEL_LUT_COLLISION_SPI   179
#define MT8113_HWTCON_DPI_SOF_SPI               180

typedef struct MT8113USBPHYState {
    MemoryRegion iomem;
    uint32_t regs[0x400 / sizeof(uint32_t)];
} MT8113USBPHYState;

typedef struct MT8113USBState {
    MemoryRegion mac_iomem;
    MemoryRegion ippc_iomem;
    uint32_t mac_regs[MT8113_USB_MAC_SIZE / sizeof(uint32_t)];
    uint32_t ippc_regs[MT8113_USB_IPPC_SIZE / sizeof(uint32_t)];
    NICConf nic_conf;
    NICState *nic;
    QEMUTimer *config_timer;
    QEMUTimer *tx_timer;
    qemu_irq irq;
    uint8_t ep0_fifo[64];
    unsigned ep0_fifo_length;
    unsigned ep0_fifo_offset;
    unsigned config_phase;
    bool setup_pending;
    bool configured;
    bool processing_tx;
} MT8113USBState;

struct MT8113State {
    DeviceState parent_obj;

    ARMCPU cpu[MT8113_NUM_CPUS];
    GICv3State gic;
    MT8113GPIOState gpio;
    MTKI2CState i2c[3];
    MTKMSDCState msdc0;
    MT8113GCEState gce;
    MT8113IOMMUState iommu;
    MT8113HWTCONState hwtcon;
    MemoryRegion sram;
    MemoryRegion topckgen_iomem;
    MemoryRegion infrasys_iomem;
    MemoryRegion scpsys_iomem;
    MemoryRegion toprgu_iomem;
    MemoryRegion timer_iomem;
    MemoryRegion apmixedsys_iomem;
    MemoryRegion dvfsrc_iomem;
    MemoryRegion mcucfg_iomem;
    MemoryRegion rng_iomem;
    MemoryRegion svs_iomem;
    MemoryRegion btif_iomem;
    MemoryRegion btif_tx_dma_iomem;
    MemoryRegion btif_rx_dma_iomem;
    MemoryRegion wifi_iomem;
    MemoryRegion efuse_iomem;
    QEMUTimer *gpt_timer;
    qemu_irq timer_irq;
    int64_t gpt_start_ns;
    uint32_t gpt_irq_status;
    MT8113USBPHYState usbphy[2];
    MT8113USBState usb;
    uint32_t topckgen_regs[0x1000 / sizeof(uint32_t)];
    uint32_t infrasys_regs[0x1000 / sizeof(uint32_t)];
    uint32_t scpsys_regs[0x1000 / sizeof(uint32_t)];
    uint32_t toprgu_regs[0x1000 / sizeof(uint32_t)];
    uint32_t timer_regs[0x1000 / sizeof(uint32_t)];
    uint32_t apmixedsys_regs[0x1000 / sizeof(uint32_t)];
    uint32_t dvfsrc_regs[0x1000 / sizeof(uint32_t)];
    uint32_t mcucfg_regs[0x1000 / sizeof(uint32_t)];
    uint32_t rng_ctrl;
    uint32_t svs_regs[0x1000 / sizeof(uint32_t)];
    uint32_t btif_regs[0x1000 / sizeof(uint32_t)];
    uint32_t btif_tx_dma_regs[0x80 / sizeof(uint32_t)];
    uint32_t btif_rx_dma_regs[0x80 / sizeof(uint32_t)];
    uint32_t wifi_regs[MT8113_WIFI_SIZE / sizeof(uint32_t)];
    uint32_t btif_wmt_stp_config;
    uint8_t btif_tx_stream[16 * KiB];
    uint8_t btif_rx_pending[16 * KiB];
    uint8_t btif_last_response[1 * KiB];
    uint32_t btif_tx_stream_len;
    uint32_t btif_rx_pending_len;
    uint32_t btif_last_response_len;
    uint32_t btif_wmt_pda_remaining;
    uint8_t btif_wmt_txseq;
    uint8_t btif_wmt_expected_rxseq;
    uint8_t btif_wmt_last_rxseq;
    bool btif_wmt_full_mode;
    bool btif_wmt_asleep;
    bool btif_wak;
    bool wifi_firmware_ready;
    uint32_t efuse_regs[0x1000 / sizeof(uint32_t)];
    qemu_irq svs_irq;
    qemu_irq btif_irq;
    qemu_irq btif_tx_dma_irq;
    qemu_irq btif_rx_dma_irq;
    qemu_irq wifi_irq;
    uint64_t reset_vector;
};

#endif /* HW_ARM_MT8113_H */
