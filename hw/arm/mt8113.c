/*
 * MediaTek MT8113 system-on-chip
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/arm/mt8113.h"
#include "hw/arm/bsa.h"
#include "hw/char/serial-mm.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "qapi/error.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qobject/qlist.h"
#include "system/dma.h"
#include "system/runstate.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "target/arm/cpu-qom.h"

#define MT8113_SCPSYS_PWR_STATUS         0x180
#define MT8113_SCPSYS_PWR_STATUS_2ND     0x184
#define MT8113_SCPSYS_CONSYS_PWR_CTRL    0x32c
#define MT8113_SCPSYS_PWR_ON             BIT(2)
#define MT8113_SCPSYS_PWR_ON_2ND         BIT(3)
#define MT8113_SCPSYS_PWR_RST_B          BIT(0)
#define MT8113_SCPSYS_PWR_ACK            BIT(30)
#define MT8113_SCPSYS_PWR_ACK_2ND        BIT(31)
#define MT8113_SCPSYS_CONSYS_TOP2_ACK    BIT(30)
#define MT8113_SCPSYS_PCM_REG15_DATA     0x13c
#define MT8113_SCPSYS_SW_RSV_9           0x658
#define MT8113_SCPSYS_DVFS_EVENT_STA     0x69c

#define MT8113_DVFSRC_LEVEL              0x0dc
#define MT8113_DVFSRC_FORCE              0x300
#define MT8113_DVFS_DEFAULT_LEVEL        BIT(7)

#define MT8113_WDT_MODE                  0x00
#define MT8113_WDT_MODE_KEY              0x22000000
#define MT8113_WDT_SWRST                 0x14
#define MT8113_WDT_SWRST_KEY             0x1209

#define MT8113_RNG_CTRL                  0x00
#define MT8113_RNG_DATA                  0x08
#define MT8113_RNG_ENABLE                BIT(0)
#define MT8113_RNG_READY                 BIT(31)

#define MT8113_BTIF_DMA_INT_FLAG         0x00
#define MT8113_BTIF_DMA_INT_EN           0x04
#define MT8113_BTIF_DMA_EN               0x08
#define MT8113_BTIF_DMA_RST              0x0c
#define MT8113_BTIF_DMA_STOP             0x10
#define MT8113_BTIF_DMA_FLUSH            0x14
#define MT8113_BTIF_DMA_VFF_ADDR         0x1c
#define MT8113_BTIF_DMA_VFF_LEN          0x24
#define MT8113_BTIF_DMA_VFF_WPT          0x2c
#define MT8113_BTIF_DMA_VFF_RPT          0x30
#define MT8113_BTIF_DMA_VALID_SIZE       0x3c
#define MT8113_BTIF_DMA_LEFT_SIZE        0x40
#define MT8113_BTIF_DMA_VFF_ADDR_H       0x54
#define MT8113_BTIF_DMA_PTR_MASK         0xffff
#define MT8113_BTIF_DMA_PTR_WRAP         BIT(16)
#define MT8113_BTIF_RX_IRQ_MASK          (BIT(0) | BIT(1))
#define MT8113_BTIF_IIR                  0x08
#define MT8113_BTIF_FIFOCTRL             0x08
#define MT8113_BTIF_LSR                  0x14
#define MT8113_BTIF_WAK                  0x64
#define MT8113_BTIF_IIR_NINT             BIT(0)
#define MT8113_BTIF_LSR_THRE             BIT(5)
#define MT8113_BTIF_LSR_TEMT             BIT(6)
#define MT8113_BTIF_STP_HEADER_SIZE      4
#define MT8113_BTIF_STP_CRC_SIZE         2
#define MT8113_BTIF_WMT_TASK             4

/* CONNAC WFDMA registers in the Wi-Fi bus window. */
#define MT8113_WIFI_INT_SOURCE            0x4200
#define MT8113_WIFI_INT_MASK              0x4204
#define MT8113_WIFI_WPDMA_GLO_CFG         0x4208
#define MT8113_WIFI_WPDMA_RST_IDX         0x420c
#define MT8113_WIFI_TX_RING_BASE          0x4300
#define MT8113_WIFI_RX_RING_BASE          0x4400
#define MT8113_WIFI_TX_RING_CPU_IDX_BASE  0x4500
#define MT8113_WIFI_RX_RING_CPU_IDX_BASE  0x4580
#define MT8113_WIFI_RING_STRIDE           0x10
#define MT8113_WIFI_RING_DESC_BASE        0x00
#define MT8113_WIFI_RING_COUNT            0x04
#define MT8113_WIFI_RING_CPU_IDX          0x08
#define MT8113_WIFI_RING_DMA_IDX          0x0c
#define MT8113_WIFI_TX_RING_COUNT         4
#define MT8113_WIFI_HW_TX_RING_COUNT      16
#define MT8113_WIFI_RX_RING_COUNT         2
#define MT8113_WIFI_TX_DONE(n)            BIT((n) + 4)
#define MT8113_WIFI_RX_DONE(n)            BIT(n)
#define MT8113_WIFI_DMA_DONE              BIT(31)
#define MT8113_WIFI_LAST_SEC0             BIT(30)
#define MT8113_WIFI_DMA_LEN_MASK          0x3fff
#define MT8113_WIFI_DMA_DESC_SIZE         16
#define MT8113_WIFI_RXD_SIZE              20
#define MT8113_WIFI_INIT_EVENT_HEADER_SIZE 28
#define MT8113_WIFI_INIT_EVENT_WORD_SIZE   4
#define MT8113_WIFI_ACCESS_EVENT_SIZE      36
#define MT8113_WIFI_INIT_CMD_SIZE         64
#define MT8113_WIFI_INIT_CMD_ID           36
#define MT8113_WIFI_INIT_CMD_SEQ          39
#define MT8113_WIFI_EVENT_ID              24
#define MT8113_WIFI_EVENT_SEQ             25
#define MT8113_WIFI_EVENT_DATA            28
#define MT8113_WIFI_EVENT_BYTE_COUNT      20
#define MT8113_WIFI_EVENT_PACKET_TYPE     22
#define MT8113_WIFI_EVENT_PACKET_TYPE_ID  0xe000
#define MT8113_WIFI_EVENT_CMD_RESULT      1
#define MT8113_WIFI_EVENT_ACCESS_REG      2
#define MT8113_WIFI_EVENT_PENDING_ERROR   3
#define MT8113_WIFI_EVENT_NIC_CAPABILITY  0x01
#define MT8113_WIFI_EVENT_RUNTIME_ACCESS  0x05
#define MT8113_WIFI_CMD_NIC_CAPABILITY    0x80
#define MT8113_WIFI_CMD_NIC_CAPABILITY_V2 0x8a
#define MT8113_WIFI_CMD_NOTIFY_CALIBRATION 0x91
#define MT8113_WIFI_CMD_RUNTIME_ACCESS    0xc0
#define MT8113_WIFI_CMD_DOWNLOAD_CONFIG   1
#define MT8113_WIFI_CMD_START             2
#define MT8113_WIFI_CMD_ACCESS_REG        3
#define MT8113_WIFI_CMD_PENDING_ERROR     4
#define MT8113_WIFI_CMD_PATCH_START       5
#define MT8113_WIFI_CMD_PATCH_FINISH      7
#define MT8113_WIFI_TOP_HVR               0x80021000
#define MT8113_WIFI_TOP_FVR               0x80021004
#define MT8113_WIFI_SW_SYNC0              0xc1140
#define MT8113_WIFI_FW_READY              (BIT(1) | BIT(2))
#define MT8113_WIFI_HIF_LPCTL             0x7000
#define MT8113_WIFI_HIF_DBGCR01           0x7104

#define MT8113_APMIXED_UNIVPLL2_CON0     0x208
#define MT8113_APMIXED_UNIVPLL2_CON1     0x20c
#define MT8113_APMIXED_MAINPLL_CON0      0x228
#define MT8113_APMIXED_MAINPLL_CON1      0x22c
#define MT8113_APMIXED_ARMPLL_CON0       0x30c
#define MT8113_APMIXED_ARMPLL_CON1       0x310
#define MT8113_APMIXED_ARMPLL_PWR_CON0   0x318
#define MT8113_APMIXED_TCONPLL_CON0      0x3a0
#define MT8113_APMIXED_TCONPLL_CON1      0x3a4
#define MT8113_MCU_BUS_PLL_DIVIDER_CFG   0x7c0
#define MT8113_MCU_BUS_SEL_ARMPLL        BIT(9)
#define MT8113_PLL_ENABLE                BIT(0)
#define MT8113_PLL_POWER_ON              BIT(0)
#define MT8113_PLL_RST_BAR               BIT(23)
#define MT8113_PLL_PCW_CHG               BIT(31)

#define MT8113_SVS_ENABLE                0xc38
#define MT8113_SVS_DCVALUES              0xc40
#define MT8113_SVS_AGEVALUES             0xc44
#define MT8113_SVS_VOP30                 0xc48
#define MT8113_SVS_VOP74                 0xc4c
#define MT8113_SVS_INTSTS                0xc54
#define MT8113_SVS_INTSTSRAW             0xc58
#define MT8113_SVS_INTST                 0xf08
#define MT8113_SVS_INIT01                BIT(0)
#define MT8113_SVS_INIT02                (BIT(0) | BIT(2))
#define MT8113_SVS_COMPLETE              BIT(0)

static uint64_t mt8113_topckgen_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    MT8113State *s = opaque;

    return s->topckgen_regs[offset / sizeof(uint32_t)];
}

static void mt8113_topckgen_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    MT8113State *s = opaque;
    hwaddr config;

    /* CLK_CFG_0..10 use value, set, clear registers at +0, +4, +8. */
    if (offset >= 0x44 && offset <= 0xe8 &&
        ((offset & 0xf) == 4 || (offset & 0xf) == 8)) {
        config = offset & ~0xf;
        if ((offset & 0xf) == 4) {
            s->topckgen_regs[config / 4] |= value;
        } else {
            s->topckgen_regs[config / 4] &= ~value;
        }
        return;
    }

    /* CLK_CFG_11 is irregular: value 0xec, set 0xf0, clear 0xf4. */
    if (offset == 0xf0) {
        s->topckgen_regs[0xec / 4] |= value;
        return;
    }
    if (offset == 0xf4) {
        s->topckgen_regs[0xec / 4] &= ~value;
        return;
    }

    s->topckgen_regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps mt8113_topckgen_ops = {
    .read = mt8113_topckgen_read,
    .write = mt8113_topckgen_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mt8113_apmixedsys_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    MT8113State *s = opaque;

    return s->apmixedsys_regs[offset / sizeof(uint32_t)];
}

static void mt8113_apmixedsys_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    MT8113State *s = opaque;

    s->apmixedsys_regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps mt8113_apmixedsys_ops = {
    .read = mt8113_apmixedsys_read,
    .write = mt8113_apmixedsys_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mt8113_mcucfg_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    MT8113State *s = opaque;

    return s->mcucfg_regs[offset / sizeof(uint32_t)];
}

static void mt8113_mcucfg_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    MT8113State *s = opaque;

    s->mcucfg_regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps mt8113_mcucfg_ops = {
    .read = mt8113_mcucfg_read,
    .write = mt8113_mcucfg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mt8113_efuse_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
    MT8113State *s = opaque;

    return s->efuse_regs[offset / sizeof(uint32_t)];
}

static void mt8113_efuse_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    /* One-time-programmable calibration storage is read-only. */
}

static const MemoryRegionOps mt8113_efuse_ops = {
    .read = mt8113_efuse_read,
    .write = mt8113_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mt8113_svs_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8113State *s = opaque;

    return s->svs_regs[offset / sizeof(uint32_t)];
}

static void mt8113_svs_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    MT8113State *s = opaque;

    if (offset == MT8113_SVS_INTSTS) {
        s->svs_regs[offset / 4] &= ~value;
        s->svs_regs[MT8113_SVS_INTSTSRAW / 4] &= ~value;
        if (!(s->svs_regs[offset / 4] & MT8113_SVS_COMPLETE)) {
            qemu_set_irq(s->svs_irq, 0);
        }
        return;
    }

    s->svs_regs[offset / sizeof(uint32_t)] = value;
    if (offset == MT8113_SVS_ENABLE &&
        (value == MT8113_SVS_INIT01 || value == MT8113_SVS_INIT02)) {
        s->svs_regs[MT8113_SVS_DCVALUES / 4] = 0;
        s->svs_regs[MT8113_SVS_AGEVALUES / 4] = 0;
        if (value == MT8113_SVS_INIT02) {
            /* Nominal MT8110 OPP voltages, fastest to slowest. */
            s->svs_regs[MT8113_SVS_VOP30 / 4] = 0x38424c54;
            s->svs_regs[MT8113_SVS_VOP74 / 4] = 0x1a1f2830;
        }
        s->svs_regs[MT8113_SVS_INTSTS / 4] = MT8113_SVS_COMPLETE;
        s->svs_regs[MT8113_SVS_INTSTSRAW / 4] = MT8113_SVS_COMPLETE;
        qemu_set_irq(s->svs_irq, 1);
    }
}

static const MemoryRegionOps mt8113_svs_ops = {
    .read = mt8113_svs_read,
    .write = mt8113_svs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

typedef struct MT8113BusProtection {
    hwaddr control;
    hwaddr set;
    hwaddr clear;
    hwaddr status;
} MT8113BusProtection;

static const MT8113BusProtection mt8113_bus_protection[] = {
    { 0x220, 0x2a0, 0x2a4, 0x228 },
    { 0x250, 0x2a8, 0x2ac, 0x258 },
    { 0x420, 0x42c, 0x430, 0x428 },
};

static uint64_t mt8113_infrasys_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    MT8113State *s = opaque;

    return s->infrasys_regs[offset / sizeof(uint32_t)];
}

static void mt8113_infrasys_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    MT8113State *s = opaque;

    for (size_t i = 0; i < ARRAY_SIZE(mt8113_bus_protection); i++) {
        const MT8113BusProtection *prot = &mt8113_bus_protection[i];
        uint32_t *control = &s->infrasys_regs[prot->control / 4];
        uint32_t *status = &s->infrasys_regs[prot->status / 4];

        if (offset == prot->control) {
            *control = value;
            *status = value;
            return;
        } else if (offset == prot->set) {
            *control |= value;
            *status |= value;
            return;
        } else if (offset == prot->clear) {
            *control &= ~value;
            *status &= ~value;
            return;
        }
    }

    s->infrasys_regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps mt8113_infrasys_ops = {
    .read = mt8113_infrasys_read,
    .write = mt8113_infrasys_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mt8113_btif_wmt_wakeup(MT8113State *s);

static uint64_t mt8113_btif_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8113State *s = opaque;

    if (offset == MT8113_BTIF_IIR) {
        return MT8113_BTIF_IIR_NINT;
    }
    if (offset == MT8113_BTIF_LSR) {
        return MT8113_BTIF_LSR_THRE | MT8113_BTIF_LSR_TEMT;
    }
    return s->btif_regs[offset / sizeof(uint32_t)];
}

static void mt8113_btif_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    MT8113State *s = opaque;
    bool old_wak;

    if (offset == MT8113_BTIF_FIFOCTRL) {
        return;
    }
    if (offset == MT8113_BTIF_WAK) {
        old_wak = s->btif_wak;
        s->btif_wak = value & BIT(0);
        s->btif_regs[offset / sizeof(uint32_t)] = s->btif_wak;
        if (!old_wak && s->btif_wak) {
            mt8113_btif_wmt_wakeup(s);
        }
        return;
    }
    s->btif_regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps mt8113_btif_ops = {
    .read = mt8113_btif_read,
    .write = mt8113_btif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static hwaddr mt8113_btif_dma_addr(uint32_t *regs)
{
    return regs[MT8113_BTIF_DMA_VFF_ADDR / 4] |
           ((hwaddr)regs[MT8113_BTIF_DMA_VFF_ADDR_H / 4] << 32);
}

static uint32_t mt8113_btif_dma_distance(uint32_t from, uint32_t to,
                                         uint32_t length)
{
    uint32_t from_pos = from & MT8113_BTIF_DMA_PTR_MASK;
    uint32_t to_pos = to & MT8113_BTIF_DMA_PTR_MASK;

    if ((from ^ to) & MT8113_BTIF_DMA_PTR_WRAP) {
        return to_pos + length - from_pos;
    }
    return to_pos - from_pos;
}

static bool mt8113_btif_dma_copy_to_guest(uint32_t *regs,
                                          const uint8_t *data, size_t length)
{
    uint32_t fifo_len = regs[MT8113_BTIF_DMA_VFF_LEN / 4];
    uint32_t valid = regs[MT8113_BTIF_DMA_VALID_SIZE / 4];
    uint32_t wpt = regs[MT8113_BTIF_DMA_VFF_WPT / 4];
    uint32_t pos = wpt & MT8113_BTIF_DMA_PTR_MASK;
    uint32_t wrap = wpt & MT8113_BTIF_DMA_PTR_WRAP;
    hwaddr addr = mt8113_btif_dma_addr(regs);
    size_t first;

    if (!fifo_len || length > fifo_len - MIN(valid, fifo_len)) {
        return false;
    }

    first = MIN(length, fifo_len - pos);
    dma_memory_write(&address_space_memory, addr + pos, data, first,
                     MEMTXATTRS_UNSPECIFIED);
    if (first < length) {
        dma_memory_write(&address_space_memory, addr, data + first,
                         length - first, MEMTXATTRS_UNSPECIFIED);
        pos = length - first;
        wrap ^= MT8113_BTIF_DMA_PTR_WRAP;
    } else {
        pos += length;
        if (pos == fifo_len) {
            pos = 0;
            wrap ^= MT8113_BTIF_DMA_PTR_WRAP;
        }
    }

    regs[MT8113_BTIF_DMA_VFF_WPT / 4] = pos | wrap;
    regs[MT8113_BTIF_DMA_VALID_SIZE / 4] += length;
    regs[MT8113_BTIF_DMA_LEFT_SIZE / 4] =
        fifo_len - regs[MT8113_BTIF_DMA_VALID_SIZE / 4];
    return true;
}

static void mt8113_btif_rx_irq_update(MT8113State *s)
{
    uint32_t *regs = s->btif_rx_dma_regs;
    bool pending = (regs[MT8113_BTIF_DMA_INT_FLAG / 4] &
                    regs[MT8113_BTIF_DMA_INT_EN / 4] &
                    MT8113_BTIF_RX_IRQ_MASK) != 0;

    qemu_set_irq(s->btif_rx_dma_irq, pending);
}

static void mt8113_btif_rx_pump(MT8113State *s)
{
    uint32_t *regs = s->btif_rx_dma_regs;
    uint32_t fifo_len = regs[MT8113_BTIF_DMA_VFF_LEN / 4];
    uint32_t valid = regs[MT8113_BTIF_DMA_VALID_SIZE / 4];
    size_t length;

    if (!(regs[MT8113_BTIF_DMA_EN / 4] & BIT(0)) || !fifo_len ||
        !s->btif_rx_pending_len || valid >= fifo_len) {
        return;
    }

    length = MIN(s->btif_rx_pending_len, fifo_len - valid);
    if (!mt8113_btif_dma_copy_to_guest(regs, s->btif_rx_pending, length)) {
        return;
    }
    s->btif_rx_pending_len -= length;
    memmove(s->btif_rx_pending, s->btif_rx_pending + length,
            s->btif_rx_pending_len);
    regs[MT8113_BTIF_DMA_INT_FLAG / 4] |= BIT(0);
    mt8113_btif_rx_irq_update(s);
}

static void mt8113_btif_rx_enqueue(MT8113State *s, const uint8_t *data,
                                    size_t length)
{
    if (length > sizeof(s->btif_rx_pending) - s->btif_rx_pending_len) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt8113-btif: firmware RX queue overflow (%zu bytes)\n",
                      length);
        return;
    }
    memcpy(s->btif_rx_pending + s->btif_rx_pending_len, data, length);
    s->btif_rx_pending_len += length;
    mt8113_btif_rx_pump(s);
}

static void mt8113_btif_wmt_session_reset(MT8113State *s)
{
    s->btif_wmt_stp_config = 0;
    s->btif_wmt_txseq = 0;
    s->btif_wmt_expected_rxseq = 0;
    s->btif_wmt_last_rxseq = 7;
    s->btif_wmt_full_mode = false;
    s->btif_wmt_asleep = false;
    s->btif_wak = false;
    s->btif_tx_stream_len = 0;
    s->btif_rx_pending_len = 0;
    s->btif_last_response_len = 0;
    s->btif_wmt_pda_remaining = 0;
    s->btif_tx_dma_regs[MT8113_BTIF_DMA_EN / 4] = 0;
    s->btif_tx_dma_regs[MT8113_BTIF_DMA_INT_FLAG / 4] = 0;
    s->btif_tx_dma_regs[MT8113_BTIF_DMA_VALID_SIZE / 4] = 0;
    s->btif_tx_dma_regs[MT8113_BTIF_DMA_LEFT_SIZE / 4] =
        s->btif_tx_dma_regs[MT8113_BTIF_DMA_VFF_LEN / 4];
    s->btif_rx_dma_regs[MT8113_BTIF_DMA_EN / 4] = 0;
    s->btif_rx_dma_regs[MT8113_BTIF_DMA_INT_FLAG / 4] = 0;
    s->btif_rx_dma_regs[MT8113_BTIF_DMA_VALID_SIZE / 4] = 0;
    s->btif_rx_dma_regs[MT8113_BTIF_DMA_LEFT_SIZE / 4] =
        s->btif_rx_dma_regs[MT8113_BTIF_DMA_VFF_LEN / 4];
    s->btif_rx_dma_regs[MT8113_BTIF_DMA_VFF_WPT / 4] =
        s->btif_rx_dma_regs[MT8113_BTIF_DMA_VFF_RPT / 4];
    qemu_set_irq(s->btif_tx_dma_irq, 0);
    qemu_set_irq(s->btif_rx_dma_irq, 0);
}

static uint16_t mt8113_btif_stp_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0;

    while (length--) {
        crc ^= *data++;
        for (unsigned int bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (crc & 1 ? 0xa001 : 0);
        }
    }
    return crc;
}

static void mt8113_btif_stp_send(MT8113State *s, const uint8_t *payload,
                                 size_t payload_len, bool full, bool cache)
{
    uint8_t frame[MT8113_BTIF_STP_HEADER_SIZE + 640 +
                  MT8113_BTIF_STP_CRC_SIZE];
    uint16_t crc = 0;
    size_t frame_len = sizeof(frame[0]) * MT8113_BTIF_STP_HEADER_SIZE +
                       payload_len + MT8113_BTIF_STP_CRC_SIZE;

    if (payload_len > 640) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt8113-btif: oversized STP payload (%zu bytes)\n",
                      payload_len);
        return;
    }

    frame[0] = 0x80;
    if (full) {
        frame[0] |= (s->btif_wmt_txseq << 3) | s->btif_wmt_last_rxseq;
    }
    frame[1] = (MT8113_BTIF_WMT_TASK << 4) |
               ((payload_len >> 8) & 0x0f);
    frame[2] = payload_len & 0xff;
    frame[3] = full ? (frame[0] + frame[1] + frame[2]) & 0xff : 0;
    memcpy(frame + MT8113_BTIF_STP_HEADER_SIZE, payload, payload_len);
    if (full) {
        crc = mt8113_btif_stp_crc16(payload, payload_len);
    }
    frame[MT8113_BTIF_STP_HEADER_SIZE + payload_len] = crc & 0xff;
    frame[MT8113_BTIF_STP_HEADER_SIZE + payload_len + 1] = crc >> 8;

    if (cache && frame_len <= sizeof(s->btif_last_response)) {
        memcpy(s->btif_last_response, frame, frame_len);
        s->btif_last_response_len = frame_len;
    }
    mt8113_btif_rx_enqueue(s, frame, frame_len);
    if (full) {
        s->btif_wmt_txseq = (s->btif_wmt_txseq + 1) & 0x07;
    }
}

static void mt8113_btif_wmt_reply(MT8113State *s, const uint8_t *command,
                                  size_t command_len, bool full)
{
    uint8_t event[640] = { 0x02 };
    uint8_t opcode;
    uint8_t subcommand;
    uint16_t declared_len;
    size_t event_len = 5;
    bool enter_full_mode = false;
    bool reset_protocol = false;
    uint32_t address;
    uint32_t value = 0;

    if (command_len < 4 || command[0] != 0x01) {
        return;
    }
    declared_len = command[2] | (command[3] << 8);
    if (declared_len + 4 != command_len) {
        return;
    }

    opcode = command[1];
    subcommand = command_len > 4 ? command[4] : 0;
    event[1] = opcode;
    event[2] = 1;
    event[4] = 0;

    if (opcode == 0x14 && command_len == 4) {
        /*
         * The calibration hand-off is the sole parameterless WMT command.
         * After the WLAN firmware finishes RF calibration, wmt_drv sends
         * { 01 14 00 00 } and requires { 02 14 02 00 00 02 } before its
         * worker can finish the WLAN_PROBE operation.
         */
        event_len = 6;
        event[2] = 0x02;
        event[5] = 0x02;
    } else if (opcode == 0x01 && subcommand == 0x00 && command_len == 21) {
        memcpy(&s->btif_wmt_pda_remaining, command + 9,
               sizeof(s->btif_wmt_pda_remaining));
        s->btif_wmt_pda_remaining += 12;
    } else if (opcode == 0x08 && subcommand == 0x02 && command_len >= 20) {
        memcpy(&address, command + 8, sizeof(address));
        if (address == 0x80000008) {
            value = 0x8512;
        }
        event_len = 16;
        event[2] = 0x0c;
        event[7] = 0x01;
        memcpy(event + 8, &address, sizeof(address));
        memcpy(event + 12, &value, sizeof(value));
    } else if (opcode == 0x08) {
        event_len = 8;
        event[2] = 0x04;
        event[7] = command_len > 7 ? command[7] : 1;
    } else if (opcode == 0x04 && subcommand == 0x04) {
        uint32_t config = s->btif_wmt_stp_config ?: 0x00000011;

        event_len = 10;
        event[2] = 0x06;
        event[5] = subcommand;
        memcpy(event + 6, &config, sizeof(config));
    } else if (opcode == 0x04 && subcommand == 0x02) {
        uint32_t baud = 115200;

        event_len = 10;
        event[2] = 0x06;
        event[5] = subcommand;
        memcpy(event + 6, &baud, sizeof(baud));
    } else if (opcode == 0x04) {
        event_len = 6;
        event[2] = 0x02;
        event[5] = subcommand;
        if (subcommand == 0x03 && command_len >= 9) {
            memcpy(&s->btif_wmt_stp_config, command + 5,
                   sizeof(s->btif_wmt_stp_config));
            enter_full_mode = true;
        }
    } else if (opcode == 0x03 || opcode == 0x05) {
        event_len = 6;
        event[2] = 0x02;
        event[5] = subcommand;
        if (opcode == 0x03) {
            s->btif_wmt_asleep = subcommand == 0x01;
        }
    } else if (opcode == 0x14 && subcommand == 0x03) {
        const uint16_t bt_size = 4;
        /*
         * The host caches this offset along with the calibration payload and
         * deliberately rejects zero when it attempts a later restore.  Keep
         * the synthetic Wi-Fi result in the first otherwise-unused page of
         * the 4 MiB CONSYS reserved-memory window.
         */
        const uint32_t wifi_offset = 0x1000;
        const uint32_t wifi_size = 4;

        event_len = 22;
        event[2] = event_len - 4;
        event[5] = subcommand;
        memcpy(event + 6, &bt_size, sizeof(bt_size));
        memset(event + 8, 0, bt_size);
        memcpy(event + 10 + bt_size, &wifi_offset, sizeof(wifi_offset));
        memcpy(event + 14 + bt_size, &wifi_size, sizeof(wifi_size));
    } else if (opcode == 0x14) {
        event_len = 6;
        event[2] = 0x02;
        event[5] = subcommand;
    } else if (opcode == 0x12) {
        event_len = 6;
        event[2] = 0x02;
        event[4] = subcommand;
        event[5] = command_len > 5 ? command[5] : 0;
    } else if (opcode == 0x11 && subcommand == 0x02) {
        event_len = 6;
        event[2] = 0x02;
        event[5] = 25;
    } else if (opcode == 0x0d && command_len >= 12) {
        event_len = 12;
        event[2] = 0x08;
        memcpy(event + 4, command + 4, 8);
        if (subcommand == 0x02) {
            memset(event + 8, 0, 4);
        }
    } else if (opcode == 0x13 && command_len >= 8) {
        event_len = 13;
        event[2] = 0x09;
        memcpy(event + 5, command + 4, 4);
    } else if (opcode == 0x01 && subcommand == 0x06 && command_len >= 9) {
        event_len = 13;
        event[2] = 0x09;
        memcpy(event + 4, command + 4, 5);
    } else if (opcode == 0x06 && subcommand == 0x05 && command_len >= 6) {
        event_len = 7;
        event[2] = 0x03;
        event[4] = command[4];
        event[5] = command[5];
        event[6] = 0;
    } else if (opcode == 0xf0) {
        event_len = 6;
        event[2] = 0x02;
        event[4] = subcommand;
        event[5] = 0;
    } else if (opcode == 0x07) {
        reset_protocol = true;
    } else if (opcode == 0x02 && subcommand == 0x12) {
        /* This intentionally partial event matches the shipped init table. */
        event[2] = 0x05;
    }

    mt8113_btif_stp_send(s, event, event_len, full, true);

    if (enter_full_mode) {
        s->btif_wmt_txseq = 0;
        s->btif_wmt_expected_rxseq = 0;
        s->btif_wmt_last_rxseq = 7;
        s->btif_last_response_len = 0;
        s->btif_wmt_full_mode = true;
    } else if (reset_protocol) {
        s->btif_wmt_stp_config = 0;
        s->btif_wmt_txseq = 0;
        s->btif_wmt_expected_rxseq = 0;
        s->btif_wmt_last_rxseq = 7;
        s->btif_wmt_full_mode = false;
        s->btif_wmt_asleep = false;
        s->btif_wmt_pda_remaining = 0;
        s->btif_last_response_len = 0;
    }
}

static void mt8113_btif_wmt_wakeup(MT8113State *s)
{
    static const uint8_t event[] = { 0x02, 0x03, 0x02, 0x00, 0x00, 0x03 };

    if (!s->btif_wmt_full_mode) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt8113-btif: ignored wake before full STP mode\n");
        return;
    }
    s->btif_wmt_asleep = false;
    mt8113_btif_stp_send(s, event, sizeof(event), true, false);
}

static void mt8113_btif_tx_stream_consume(MT8113State *s, size_t length)
{
    memmove(s->btif_tx_stream, s->btif_tx_stream + length,
            s->btif_tx_stream_len - length);
    s->btif_tx_stream_len -= length;
}

static void mt8113_btif_tx_parse(MT8113State *s)
{
    uint8_t *frame;
    uint16_t payload_len;
    uint16_t received_crc;
    uint16_t expected_crc;
    size_t frame_len;
    bool full;
    uint8_t seq;

    while (s->btif_tx_stream_len) {
        frame = s->btif_tx_stream;

        if (s->btif_wmt_pda_remaining) {
            size_t raw_len = MIN(s->btif_tx_stream_len,
                                 s->btif_wmt_pda_remaining);

            mt8113_btif_tx_stream_consume(s, raw_len);
            s->btif_wmt_pda_remaining -= raw_len;
            continue;
        }

        if (frame[0] == 0xff) {
            mt8113_btif_tx_stream_consume(s, 1);
            mt8113_btif_wmt_wakeup(s);
            continue;
        }
        if (s->btif_tx_stream_len >= 4 &&
            frame[0] == 0x7f && frame[1] == 0x7f &&
            frame[2] == 0x7f && frame[3] == 0x7f) {
            mt8113_btif_tx_stream_consume(s, 4);
            continue;
        }
        if ((frame[0] & 0xc0) != 0x80) {
            mt8113_btif_tx_stream_consume(s, 1);
            continue;
        }
        if (s->btif_tx_stream_len < MT8113_BTIF_STP_HEADER_SIZE) {
            return;
        }

        payload_len = ((frame[1] & 0x0f) << 8) | frame[2];
        if (!payload_len) {
            mt8113_btif_tx_stream_consume(s, MT8113_BTIF_STP_HEADER_SIZE);
            continue;
        }
        frame_len = MT8113_BTIF_STP_HEADER_SIZE + payload_len +
                    MT8113_BTIF_STP_CRC_SIZE;
        if (s->btif_tx_stream_len < frame_len) {
            return;
        }
        if ((frame[1] >> 4) != MT8113_BTIF_WMT_TASK) {
            mt8113_btif_tx_stream_consume(s, frame_len);
            continue;
        }

        full = s->btif_wmt_full_mode;
        if (full) {
            if (frame[3] != (uint8_t)(frame[0] + frame[1] + frame[2])) {
                mt8113_btif_tx_stream_consume(s, 1);
                continue;
            }
            memcpy(&received_crc,
                   frame + MT8113_BTIF_STP_HEADER_SIZE + payload_len,
                   sizeof(received_crc));
            expected_crc = mt8113_btif_stp_crc16(
                frame + MT8113_BTIF_STP_HEADER_SIZE, payload_len);
            if (received_crc != expected_crc) {
                mt8113_btif_tx_stream_consume(s, frame_len);
                continue;
            }

            seq = (frame[0] >> 3) & 0x07;
            if (seq != s->btif_wmt_expected_rxseq) {
                if (seq == s->btif_wmt_last_rxseq &&
                    s->btif_last_response_len) {
                    mt8113_btif_rx_enqueue(s, s->btif_last_response,
                                           s->btif_last_response_len);
                }
                mt8113_btif_tx_stream_consume(s, frame_len);
                continue;
            }
            s->btif_wmt_last_rxseq = seq;
            s->btif_wmt_expected_rxseq = (seq + 1) & 0x07;
        }

        mt8113_btif_wmt_reply(s,
                              frame + MT8113_BTIF_STP_HEADER_SIZE,
                              payload_len, full);
        mt8113_btif_tx_stream_consume(s, frame_len);
    }
}

static void mt8113_btif_tx_consume(MT8113State *s, uint32_t new_wpt)
{
    uint32_t *regs = s->btif_tx_dma_regs;
    uint32_t old_rpt = regs[MT8113_BTIF_DMA_VFF_RPT / 4];
    uint32_t fifo_len = regs[MT8113_BTIF_DMA_VFF_LEN / 4];
    uint32_t length = mt8113_btif_dma_distance(old_rpt, new_wpt, fifo_len);
    uint32_t pos = old_rpt & MT8113_BTIF_DMA_PTR_MASK;
    hwaddr addr = mt8113_btif_dma_addr(regs);
    g_autofree uint8_t *data = NULL;
    size_t first;

    regs[MT8113_BTIF_DMA_VFF_WPT / 4] = new_wpt;
    if (!fifo_len || !length || length > fifo_len) {
        return;
    }

    data = g_malloc(length);
    first = MIN(length, fifo_len - pos);
    dma_memory_read(&address_space_memory, addr + pos, data, first,
                    MEMTXATTRS_UNSPECIFIED);
    if (first < length) {
        dma_memory_read(&address_space_memory, addr, data + first,
                        length - first, MEMTXATTRS_UNSPECIFIED);
    }

    regs[MT8113_BTIF_DMA_VFF_RPT / 4] = new_wpt;
    regs[MT8113_BTIF_DMA_VALID_SIZE / 4] = 0;
    regs[MT8113_BTIF_DMA_LEFT_SIZE / 4] = fifo_len;
    regs[MT8113_BTIF_DMA_INT_FLAG / 4] |= BIT(0);
    if (regs[MT8113_BTIF_DMA_INT_EN / 4] & BIT(0)) {
        qemu_set_irq(s->btif_tx_dma_irq, 1);
    }
    if (length > sizeof(s->btif_tx_stream) - s->btif_tx_stream_len) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt8113-btif: host TX stream overflow (%u bytes)\n",
                      length);
        s->btif_tx_stream_len = 0;
        return;
    }
    memcpy(s->btif_tx_stream + s->btif_tx_stream_len, data, length);
    s->btif_tx_stream_len += length;
    mt8113_btif_tx_parse(s);
}

static uint64_t mt8113_btif_tx_dma_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    MT8113State *s = opaque;
    uint32_t *regs = s->btif_tx_dma_regs;

    if (offset == MT8113_BTIF_DMA_LEFT_SIZE) {
        return regs[MT8113_BTIF_DMA_VFF_LEN / 4] -
               regs[MT8113_BTIF_DMA_VALID_SIZE / 4];
    }
    return regs[offset / sizeof(uint32_t)];
}

static uint64_t mt8113_btif_rx_dma_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    MT8113State *s = opaque;

    return s->btif_rx_dma_regs[offset / sizeof(uint32_t)];
}

static void mt8113_btif_dma_reset_channel(uint32_t *regs, qemu_irq irq)
{
    regs[MT8113_BTIF_DMA_EN / 4] = 0;
    regs[MT8113_BTIF_DMA_STOP / 4] = 0;
    regs[MT8113_BTIF_DMA_FLUSH / 4] = 0;
    regs[MT8113_BTIF_DMA_INT_FLAG / 4] = 0;
    regs[MT8113_BTIF_DMA_VFF_WPT / 4] = 0;
    regs[MT8113_BTIF_DMA_VFF_RPT / 4] = 0;
    regs[MT8113_BTIF_DMA_VALID_SIZE / 4] = 0;
    regs[MT8113_BTIF_DMA_LEFT_SIZE / 4] =
        regs[MT8113_BTIF_DMA_VFF_LEN / 4];
    qemu_set_irq(irq, 0);
}

static void mt8113_btif_tx_dma_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    MT8113State *s = opaque;
    uint32_t *regs = s->btif_tx_dma_regs;

    if (offset == MT8113_BTIF_DMA_INT_FLAG) {
        regs[offset / 4] &= ~value;
        qemu_set_irq(s->btif_tx_dma_irq, 0);
        return;
    }
    if (offset == MT8113_BTIF_DMA_RST && (value & BIT(0))) {
        mt8113_btif_dma_reset_channel(regs, s->btif_tx_dma_irq);
    }
    if (offset == MT8113_BTIF_DMA_STOP && (value & BIT(0))) {
        regs[MT8113_BTIF_DMA_EN / 4] = 0;
        regs[MT8113_BTIF_DMA_STOP / 4] = 0;
        return;
    }
    if (offset == MT8113_BTIF_DMA_VFF_WPT) {
        mt8113_btif_tx_consume(s, value);
        return;
    }
    regs[offset / 4] = value;
}

static void mt8113_btif_rx_dma_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    MT8113State *s = opaque;
    uint32_t *regs = s->btif_rx_dma_regs;

    if (offset == MT8113_BTIF_DMA_INT_FLAG) {
        regs[offset / 4] &= ~value;
        if (regs[MT8113_BTIF_DMA_VALID_SIZE / 4]) {
            regs[offset / 4] |= BIT(0);
        }
        mt8113_btif_rx_irq_update(s);
        return;
    }
    if (offset == MT8113_BTIF_DMA_RST && (value & BIT(0))) {
        mt8113_btif_dma_reset_channel(regs, s->btif_rx_dma_irq);
    }
    if (offset == MT8113_BTIF_DMA_STOP && (value & BIT(0))) {
        regs[MT8113_BTIF_DMA_EN / 4] = 0;
        regs[MT8113_BTIF_DMA_STOP / 4] = 0;
        return;
    }
    regs[offset / 4] = value;
    if (offset == MT8113_BTIF_DMA_VFF_LEN) {
        regs[MT8113_BTIF_DMA_LEFT_SIZE / 4] = value;
    } else if (offset == MT8113_BTIF_DMA_VFF_RPT) {
        uint32_t valid = mt8113_btif_dma_distance(
            value, regs[MT8113_BTIF_DMA_VFF_WPT / 4],
            regs[MT8113_BTIF_DMA_VFF_LEN / 4]);
        regs[MT8113_BTIF_DMA_VALID_SIZE / 4] = valid;
        regs[MT8113_BTIF_DMA_LEFT_SIZE / 4] =
            regs[MT8113_BTIF_DMA_VFF_LEN / 4] - valid;
    }
    if (offset == MT8113_BTIF_DMA_EN ||
        offset == MT8113_BTIF_DMA_VFF_LEN ||
        offset == MT8113_BTIF_DMA_VFF_RPT ||
        offset == MT8113_BTIF_DMA_INT_EN) {
        mt8113_btif_rx_pump(s);
        mt8113_btif_rx_irq_update(s);
    }
}

static const MemoryRegionOps mt8113_btif_tx_dma_ops = {
    .read = mt8113_btif_tx_dma_read,
    .write = mt8113_btif_tx_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps mt8113_btif_rx_dma_ops = {
    .read = mt8113_btif_rx_dma_read,
    .write = mt8113_btif_rx_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

typedef struct MT8113WifiDmaDesc {
    uint32_t ptr0;
    uint32_t control;
    uint32_t ptr1;
    uint32_t info;
} MT8113WifiDmaDesc;

static void mt8113_wifi_irq_update(MT8113State *s)
{
    qemu_set_irq(s->wifi_irq,
                 (s->wifi_regs[MT8113_WIFI_INT_SOURCE / 4] &
                  s->wifi_regs[MT8113_WIFI_INT_MASK / 4]) != 0);
}

static bool mt8113_wifi_rx_event(MT8113State *s, unsigned int ring_index,
                                  const uint8_t *event, size_t length)
{
    hwaddr ring = MT8113_WIFI_RX_RING_BASE +
                  ring_index * MT8113_WIFI_RING_STRIDE;
    uint32_t count = s->wifi_regs[(ring + MT8113_WIFI_RING_COUNT) / 4];
    uint32_t index = s->wifi_regs[(ring + MT8113_WIFI_RING_DMA_IDX) / 4];
    uint32_t cpu = s->wifi_regs[(ring + MT8113_WIFI_RING_CPU_IDX) / 4];
    uint32_t base = s->wifi_regs[(ring + MT8113_WIFI_RING_DESC_BASE) / 4];
    MT8113WifiDmaDesc desc;

    if (!count || !base || length > MT8113_WIFI_DMA_LEN_MASK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt8113-wifi: no RX%u ring base=%08x count=%u\n",
                      ring_index, base, count);
        return false;
    }
    index %= count;
    if (((index + 1) % count) == ((cpu + 1) % count)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt8113-wifi: RX%u ring is full\n", ring_index);
        return false;
    }

    dma_memory_read(&address_space_memory,
                    base + index * MT8113_WIFI_DMA_DESC_SIZE,
                    &desc, sizeof(desc), MEMTXATTRS_UNSPECIFIED);
    if (!desc.ptr0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt8113-wifi: RX%u descriptor %u has no buffer\n",
                      ring_index, index);
        return false;
    }
    dma_memory_write(&address_space_memory, desc.ptr0, event, length,
                     MEMTXATTRS_UNSPECIFIED);
    desc.control = (desc.control & 0x0000ffff) |
                   (length << 16) | MT8113_WIFI_LAST_SEC0 |
                   MT8113_WIFI_DMA_DONE;
    dma_memory_write(&address_space_memory,
                     base + index * MT8113_WIFI_DMA_DESC_SIZE,
                     &desc, sizeof(desc), MEMTXATTRS_UNSPECIFIED);

    s->wifi_regs[(ring + MT8113_WIFI_RING_DMA_IDX) / 4] =
        (index + 1) % count;
    s->wifi_regs[MT8113_WIFI_INT_SOURCE / 4] |=
        MT8113_WIFI_RX_DONE(ring_index);
    mt8113_wifi_irq_update(s);
    return true;
}

static uint32_t mt8113_wifi_register_value(MT8113State *s, uint32_t address)
{
    switch (address) {
    case MT8113_WIFI_TOP_HVR:
        /* The module's ECO table accepts the zero-revision first silicon. */
        return 0;
    case MT8113_WIFI_TOP_FVR:
        return 0;
    default:
        return 0;
    }
}

static void mt8113_wifi_init_event(MT8113State *s, uint8_t event_id,
                                    uint8_t sequence, uint32_t word0,
                                    uint32_t word1)
{
    uint8_t event[MT8113_WIFI_ACCESS_EVENT_SIZE] = { 0 };
    size_t event_size = MT8113_WIFI_INIT_EVENT_HEADER_SIZE +
                        (event_id == MT8113_WIFI_EVENT_ACCESS_REG ?
                         2 : 1) * MT8113_WIFI_INIT_EVENT_WORD_SIZE;
    uint16_t event_byte_count = event_size - MT8113_WIFI_RXD_SIZE;
    uint16_t packet_type = MT8113_WIFI_EVENT_PACKET_TYPE_ID;

    /* The firmware event ABI starts after the 20-byte hardware RXD. */
    event[0] = event_size & 0xff;
    event[1] = event_size >> 8;
    memcpy(event + MT8113_WIFI_EVENT_BYTE_COUNT, &event_byte_count,
           sizeof(event_byte_count));
    memcpy(event + MT8113_WIFI_EVENT_PACKET_TYPE, &packet_type,
           sizeof(packet_type));
    event[MT8113_WIFI_EVENT_ID] = event_id;
    event[MT8113_WIFI_EVENT_SEQ] = sequence;
    memcpy(event + MT8113_WIFI_EVENT_DATA, &word0, sizeof(word0));
    memcpy(event + MT8113_WIFI_EVENT_DATA + 4, &word1, sizeof(word1));
    /* Init command responses are consumed from the driver's event RX ring. */
    mt8113_wifi_rx_event(s, 1, event, event_size);
}

static void mt8113_wifi_runtime_event(MT8113State *s, uint8_t event_id,
                                      uint8_t sequence,
                                      const void *payload,
                                      size_t payload_size)
{
    uint8_t event[144] = { 0 };
    size_t event_size = MT8113_WIFI_INIT_EVENT_HEADER_SIZE + payload_size;
    uint16_t packet_length;
    uint16_t packet_type = MT8113_WIFI_EVENT_PACKET_TYPE_ID;

    g_assert(event_size <= sizeof(event));
    packet_length = event_size - MT8113_WIFI_RXD_SIZE;
    event[0] = event_size & 0xff;
    event[1] = event_size >> 8;
    /* Connac's RX descriptor exposes the software-event type here. */
    memcpy(event + 2, &packet_type, sizeof(packet_type));
    memcpy(event + MT8113_WIFI_EVENT_BYTE_COUNT, &packet_length,
           sizeof(packet_length));
    memcpy(event + MT8113_WIFI_EVENT_PACKET_TYPE, &packet_type,
           sizeof(packet_type));
    event[MT8113_WIFI_EVENT_ID] = event_id;
    event[MT8113_WIFI_EVENT_SEQ] = sequence;
    if (payload_size) {
        memcpy(event + MT8113_WIFI_EVENT_DATA, payload, payload_size);
    }
    mt8113_wifi_rx_event(s, 1, event, event_size);
}

static void mt8113_wifi_runtime_command(MT8113State *s,
                                        const uint8_t *data,
                                        size_t length)
{
    uint8_t command = data[MT8113_WIFI_INIT_CMD_ID];
    uint8_t sequence = data[MT8113_WIFI_INIT_CMD_SEQ];

    switch (command) {
    case MT8113_WIFI_CMD_NIC_CAPABILITY: {
        uint8_t capability[116] = { 0 };
        uint16_t product_id = 0x6632;
        uint16_t version = 1;
        static const uint8_t mac_address[6] = {
            0x02, 0x00, 0x00, 0x66, 0x32, 0x01,
        };

        memcpy(capability, &product_id, sizeof(product_id));
        memcpy(capability + 2, &version, sizeof(version));
        memcpy(capability + 4, &version, sizeof(version));
        memcpy(capability + 8, mac_address, sizeof(mac_address));
        mt8113_wifi_runtime_event(s, MT8113_WIFI_EVENT_NIC_CAPABILITY,
                                  sequence, capability,
                                  sizeof(capability));
        break;
    }
    case MT8113_WIFI_CMD_NIC_CAPABILITY_V2: {
        uint32_t no_elements = 0;

        mt8113_wifi_runtime_event(s, 0xec, sequence, &no_elements,
                                  sizeof(no_elements));
        break;
    }
    case MT8113_WIFI_CMD_RUNTIME_ACCESS: {
        uint32_t registers[2] = { 0 };

        if (length >= MT8113_WIFI_INIT_CMD_SIZE + sizeof(registers)) {
            memcpy(registers, data + MT8113_WIFI_INIT_CMD_SIZE,
                   sizeof(registers));
        }
        mt8113_wifi_runtime_event(s, MT8113_WIFI_EVENT_RUNTIME_ACCESS,
                                  sequence, registers, sizeof(registers));
        break;
    }
    case MT8113_WIFI_CMD_NOTIFY_CALIBRATION: {
        uint32_t status = 0;

        mt8113_wifi_runtime_event(s, command, sequence, &status,
                                  sizeof(status));
        break;
    }
    default:
        break;
    }
}

static void mt8113_wifi_command(MT8113State *s, const uint8_t *data,
                                 size_t length)
{
    uint8_t command;
    uint8_t sequence;
    uint32_t address;
    uint32_t value;

    /* All init commands carry the 0xa0 command namespace in byte 37.
     * Access/download commands duplicate it in byte 5; START does not. */
    if (length < MT8113_WIFI_INIT_CMD_SIZE || data[37] != 0xa0) {
        if (length >= 40) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt8113-wifi: non-init TX len=%zu "
                          "h=%02x%02x%02x%02x/%02x%02x%02x%02x "
                          "cmd=%02x%02x%02x%02x\n",
                          length, data[0], data[1], data[2], data[3],
                          data[4], data[5], data[6], data[7], data[36],
                          data[37], data[38], data[39]);
        }
        return;
    }
    if (s->wifi_firmware_ready) {
        mt8113_wifi_runtime_command(s, data, length);
        return;
    }
    command = data[MT8113_WIFI_INIT_CMD_ID];
    sequence = data[MT8113_WIFI_INIT_CMD_SEQ];
    switch (command) {
    case 0:
        /* A firmware/patch data fragment; consuming the TX descriptor is ACK. */
        break;
    case MT8113_WIFI_CMD_ACCESS_REG:
        if (length < 76) {
            break;
        }
        memcpy(&address, data + 68, sizeof(address));
        memcpy(&value, data + 72, sizeof(value));
        if (data[64] == 0) {
            value = mt8113_wifi_register_value(s, address);
        }
        mt8113_wifi_init_event(s, MT8113_WIFI_EVENT_ACCESS_REG,
                               sequence, address, value);
        break;
    case MT8113_WIFI_CMD_PENDING_ERROR:
        mt8113_wifi_init_event(s, MT8113_WIFI_EVENT_PENDING_ERROR,
                               sequence, 0, 0);
        break;
    case MT8113_WIFI_CMD_START:
        s->wifi_firmware_ready = true;
        s->wifi_regs[MT8113_WIFI_SW_SYNC0 / 4] |= MT8113_WIFI_FW_READY;
        mt8113_wifi_init_event(s, MT8113_WIFI_EVENT_CMD_RESULT,
                               sequence, 0, 0);
        break;
    case MT8113_WIFI_CMD_DOWNLOAD_CONFIG:
    case MT8113_WIFI_CMD_PATCH_START:
    case MT8113_WIFI_CMD_PATCH_FINISH:
    default:
        mt8113_wifi_init_event(s, MT8113_WIFI_EVENT_CMD_RESULT,
                               sequence, 0, 0);
        break;
    }
}

static void mt8113_wifi_tx_ring(MT8113State *s, unsigned int ring_index,
                                 uint32_t new_index)
{
    hwaddr ring = MT8113_WIFI_TX_RING_BASE +
                  ring_index * MT8113_WIFI_RING_STRIDE;
    uint32_t count = s->wifi_regs[(ring + MT8113_WIFI_RING_COUNT) / 4];
    uint32_t index = s->wifi_regs[(ring + MT8113_WIFI_RING_DMA_IDX) / 4];
    uint32_t base = s->wifi_regs[(ring + MT8113_WIFI_RING_DESC_BASE) / 4];
    unsigned int processed = 0;
    unsigned int completed = 0;

    if (!count || !base) {
        if (new_index) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt8113-wifi: TX%u doorbell without ring "
                          "base=%08x count=%u\n",
                          ring_index, base, count);
        }
        return;
    }
    new_index %= count;
    index %= count;
    while (index != new_index && processed++ < count) {
        MT8113WifiDmaDesc desc;
        uint8_t packet[4096];
        size_t length;

        dma_memory_read(&address_space_memory,
                        base + index * MT8113_WIFI_DMA_DESC_SIZE,
                        &desc, sizeof(desc), MEMTXATTRS_UNSPECIFIED);
        length = (desc.control >> 16) & MT8113_WIFI_DMA_LEN_MASK;
        if (!desc.ptr0 || !length) {
            /* Ring setup writes the producer index back to zero before the
             * new descriptors are populated.  It is not a wrapped batch. */
            index = new_index;
            if (ring_index == 3 && new_index == 0) {
                s->wifi_firmware_ready = false;
            }
            break;
        }
        if (length > sizeof(packet)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt8113-wifi: TX%u descriptor length %zu is too "
                          "large\n", ring_index, length);
            index = (index + 1) % count;
            continue;
        }
        dma_memory_read(&address_space_memory, desc.ptr0, packet, length,
                        MEMTXATTRS_UNSPECIFIED);
        mt8113_wifi_command(s, packet, length);
        desc.control |= MT8113_WIFI_DMA_DONE;
        dma_memory_write(&address_space_memory,
                         base + index * MT8113_WIFI_DMA_DESC_SIZE,
                         &desc, sizeof(desc), MEMTXATTRS_UNSPECIFIED);
        completed++;
        index = (index + 1) % count;
    }
    s->wifi_regs[(ring + MT8113_WIFI_RING_DMA_IDX) / 4] = index;
    if (completed) {
        s->wifi_regs[MT8113_WIFI_INT_SOURCE / 4] |=
            MT8113_WIFI_TX_DONE(ring_index);
        mt8113_wifi_irq_update(s);
    }
}

static uint64_t mt8113_wifi_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8113State *s = opaque;

    if (offset == MT8113_WIFI_HIF_DBGCR01) {
        return 0;
    }
    return s->wifi_regs[offset / sizeof(uint32_t)];
}

static void mt8113_wifi_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    MT8113State *s = opaque;

    if (offset == MT8113_WIFI_INT_SOURCE) {
        s->wifi_regs[offset / 4] &= ~value;
        mt8113_wifi_irq_update(s);
        return;
    }
    if (offset == MT8113_WIFI_INT_MASK) {
        s->wifi_regs[offset / 4] = value;
        mt8113_wifi_irq_update(s);
        return;
    }
    if (offset == MT8113_WIFI_WPDMA_GLO_CFG) {
        /* DMA operations are synchronous, so hardware busy bits stay clear. */
        s->wifi_regs[offset / 4] = value & ~(BIT(1) | BIT(3));
        return;
    }
    if (offset == MT8113_WIFI_WPDMA_RST_IDX) {
        s->wifi_regs[offset / 4] = 0;
        return;
    }
    if (offset == MT8113_WIFI_HIF_LPCTL) {
        /* A driver-own request completes immediately. */
        s->wifi_regs[offset / 4] = (value & BIT(0)) ? BIT(0) : 0;
        return;
    }

    s->wifi_regs[offset / sizeof(uint32_t)] = value;
    if (offset >= MT8113_WIFI_TX_RING_BASE + MT8113_WIFI_RING_CPU_IDX &&
        offset < MT8113_WIFI_TX_RING_BASE +
                     MT8113_WIFI_HW_TX_RING_COUNT *
                         MT8113_WIFI_RING_STRIDE &&
        ((offset - MT8113_WIFI_TX_RING_BASE) % MT8113_WIFI_RING_STRIDE) ==
            MT8113_WIFI_RING_CPU_IDX) {
        mt8113_wifi_tx_ring(s,
            (offset - MT8113_WIFI_TX_RING_BASE) / MT8113_WIFI_RING_STRIDE,
            value);
    } else if (offset >= MT8113_WIFI_TX_RING_CPU_IDX_BASE &&
               offset < MT8113_WIFI_TX_RING_CPU_IDX_BASE +
                            MT8113_WIFI_TX_RING_COUNT * 4) {
        unsigned int ring_index =
            (offset - MT8113_WIFI_TX_RING_CPU_IDX_BASE) / 4;
        hwaddr ring = MT8113_WIFI_TX_RING_BASE +
                      ring_index * MT8113_WIFI_RING_STRIDE;

        s->wifi_regs[(ring + MT8113_WIFI_RING_CPU_IDX) / 4] = value;
        mt8113_wifi_tx_ring(s, ring_index, value);
    } else if (offset >= MT8113_WIFI_RX_RING_CPU_IDX_BASE &&
               offset < MT8113_WIFI_RX_RING_CPU_IDX_BASE +
                            MT8113_WIFI_RX_RING_COUNT * 4) {
        unsigned int ring_index =
            (offset - MT8113_WIFI_RX_RING_CPU_IDX_BASE) / 4;
        hwaddr ring = MT8113_WIFI_RX_RING_BASE +
                      ring_index * MT8113_WIFI_RING_STRIDE;

        s->wifi_regs[(ring + MT8113_WIFI_RING_CPU_IDX) / 4] = value;
    }
}

static const MemoryRegionOps mt8113_wifi_ops = {
    .read = mt8113_wifi_read,
    .write = mt8113_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mt8113_rng_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8113State *s = opaque;

    switch (offset) {
    case MT8113_RNG_CTRL:
        return s->rng_ctrl | (s->rng_ctrl & MT8113_RNG_ENABLE ?
                              MT8113_RNG_READY : 0);
    case MT8113_RNG_DATA: {
        uint32_t value;

        qemu_guest_getrandom_nofail(&value, sizeof(value));
        return value;
    }
    default:
        return 0;
    }
}

static void mt8113_rng_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    MT8113State *s = opaque;

    if (offset == MT8113_RNG_CTRL) {
        s->rng_ctrl = value & MT8113_RNG_ENABLE;
    }
}

static const MemoryRegionOps mt8113_rng_ops = {
    .read = mt8113_rng_read,
    .write = mt8113_rng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

typedef struct MT8113PowerDomain {
    hwaddr control;
    uint32_t status;
    uint32_t sram_power_down;
    uint32_t sram_ack;
} MT8113PowerDomain;

static const MT8113PowerDomain mt8113_power_domains[] = {
    { MT8113_SCPSYS_CONSYS_PWR_CTRL,
             BIT(1),  BIT(8),       BIT(12) }, /* CONSYS */
    { 0x374, BIT(15), BIT(8),       BIT(12) }, /* MM */
    { 0x38c, BIT(16), BIT(8),       BIT(12) }, /* IMG */
    { 0x39c, BIT(25), BIT(8),       BIT(12) }, /* IP0 */
    { 0x384, BIT(26), BIT(8),       BIT(12) }, /* IP1 */
    { 0x388, BIT(27), BIT(8),       BIT(12) }, /* IP2 */
    { 0x37c, BIT(17), 0x00000f00, 0x0000f000 }, /* DSP */
    { 0x314, BIT(24), 0x00000f00, 0x0001e000 }, /* AUDAFE */
    { 0x328, BIT(23), BIT(8),       BIT(12) }, /* AUDSRC */
    { 0x3a4, BIT(20), BIT(8),       BIT(12) }, /* USB */
};

static const MT8113PowerDomain *mt8113_power_domain(hwaddr offset)
{
    for (size_t i = 0; i < ARRAY_SIZE(mt8113_power_domains); i++) {
        if (mt8113_power_domains[i].control == offset) {
            return &mt8113_power_domains[i];
        }
    }
    return NULL;
}

static uint64_t mt8113_scpsys_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    MT8113State *s = opaque;

    return s->scpsys_regs[offset / sizeof(uint32_t)];
}

static void mt8113_scpsys_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    MT8113State *s = opaque;
    const MT8113PowerDomain *domain = mt8113_power_domain(offset);
    uint32_t control = value;

    if (domain) {
        uint32_t *status = &s->scpsys_regs[MT8113_SCPSYS_PWR_STATUS / 4];
        uint32_t *status_2nd =
            &s->scpsys_regs[MT8113_SCPSYS_PWR_STATUS_2ND / 4];

        if (control & MT8113_SCPSYS_PWR_ON) {
            *status |= domain->status;
            control |= MT8113_SCPSYS_PWR_ACK;
        } else {
            *status &= ~domain->status;
            control &= ~MT8113_SCPSYS_PWR_ACK;
        }
        if (control & MT8113_SCPSYS_PWR_ON_2ND) {
            *status_2nd |= domain->status;
            control |= MT8113_SCPSYS_PWR_ACK_2ND;
        } else {
            *status_2nd &= ~domain->status;
            control &= ~MT8113_SCPSYS_PWR_ACK_2ND;
        }

        if ((control & domain->sram_power_down) ==
            domain->sram_power_down) {
            control |= domain->sram_ack;
        } else if (!(control & domain->sram_power_down)) {
            control &= ~domain->sram_ack;
        }

        /* CONSYS has a second, top-off domain acknowledged after reset. */
        if (offset == MT8113_SCPSYS_CONSYS_PWR_CTRL) {
            if (!(control & MT8113_SCPSYS_PWR_RST_B) ||
                !(control & MT8113_SCPSYS_PWR_ON)) {
                mt8113_btif_wmt_session_reset(s);
            }
            if ((control & (MT8113_SCPSYS_PWR_RST_B |
                            MT8113_SCPSYS_PWR_ON)) ==
                (MT8113_SCPSYS_PWR_RST_B | MT8113_SCPSYS_PWR_ON)) {
                *status |= MT8113_SCPSYS_CONSYS_TOP2_ACK;
            } else {
                *status &= ~MT8113_SCPSYS_CONSYS_TOP2_ACK;
            }
            if ((control & (MT8113_SCPSYS_PWR_RST_B |
                            MT8113_SCPSYS_PWR_ON_2ND)) ==
                (MT8113_SCPSYS_PWR_RST_B | MT8113_SCPSYS_PWR_ON_2ND)) {
                *status_2nd |= MT8113_SCPSYS_CONSYS_TOP2_ACK;
            } else {
                *status_2nd &= ~MT8113_SCPSYS_CONSYS_TOP2_ACK;
            }
        }
    }
    s->scpsys_regs[offset / sizeof(uint32_t)] = control;
}

static const MemoryRegionOps mt8113_scpsys_ops = {
    .read = mt8113_scpsys_read,
    .write = mt8113_scpsys_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

#define MT8113_USBPHY_FM_VALUE   0x10c
#define MT8113_USBPHY_FM_MON     0x110
#define MT8113_USBPHY_FM_VALID   BIT(0)

static uint64_t mt8113_usbphy_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    MT8113USBPHYState *phy = opaque;

    if (offset == MT8113_USBPHY_FM_MON) {
        return phy->regs[offset / 4] | MT8113_USBPHY_FM_VALID;
    }
    if (offset == MT8113_USBPHY_FM_VALUE) {
        return phy->regs[offset / 4] ?: 1024;
    }
    return phy->regs[offset / 4];
}

static void mt8113_usbphy_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    MT8113USBPHYState *phy = opaque;

    phy->regs[offset / 4] = value;
}

static const MemoryRegionOps mt8113_usbphy_ops = {
    .read = mt8113_usbphy_read,
    .write = mt8113_usbphy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* MTU3 device controller and its IP-port power/clock controller. */
#define MT8113_USB_CAP_EPNTXFFSZ     0x0c08
#define MT8113_USB_CAP_EPNRXFFSZ     0x0c0c
#define MT8113_USB_CAP_EPINFO        0x0c10
#define MT8113_USB_FIFO_SIZE         0x8000
#define MT8113_USB_ENDPOINTS         8

#define MT8113_USB_IP_PW_CTRL2       0x08
#define MT8113_USB_IP_PW_STS1        0x10
#define MT8113_USB_IP_PW_STS2        0x14
#define MT8113_USB_IP_XHCI_CAP       0x24
#define MT8113_USB_IP_SLEEP_STS      BIT(30)
#define MT8113_USB_XHCI_RST_B_STS    BIT(11)
#define MT8113_USB_SYS125_RST_B_STS  BIT(10)
#define MT8113_USB_REF_RST_B_STS     BIT(8)
#define MT8113_USB_SYSPLL_STABLE     BIT(0)
#define MT8113_USB_U2_MAC_RST_B_STS  BIT(0)
#define MT8113_USB_IP_DEV_PDN        BIT(0)
#define MT8113_USB_XHCI_U2_PORTS     (1 << 8)

static uint64_t mt8113_usb_mac_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    MT8113USBState *usb = opaque;

    switch (offset) {
    case MT8113_USB_CAP_EPNTXFFSZ:
    case MT8113_USB_CAP_EPNRXFFSZ:
        return MT8113_USB_FIFO_SIZE;
    case MT8113_USB_CAP_EPINFO:
        return (MT8113_USB_ENDPOINTS << 8) | MT8113_USB_ENDPOINTS;
    default:
        return usb->mac_regs[offset / sizeof(uint32_t)];
    }
}

static void mt8113_usb_mac_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    MT8113USBState *usb = opaque;

    usb->mac_regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps mt8113_usb_mac_ops = {
    .read = mt8113_usb_mac_read,
    .write = mt8113_usb_mac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mt8113_usb_ippc_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    MT8113USBState *usb = opaque;

    switch (offset) {
    case MT8113_USB_IP_PW_STS1:
        return MT8113_USB_SYS125_RST_B_STS |
               MT8113_USB_XHCI_RST_B_STS |
               MT8113_USB_REF_RST_B_STS |
               MT8113_USB_SYSPLL_STABLE |
               ((usb->ippc_regs[MT8113_USB_IP_PW_CTRL2 / 4] &
                 MT8113_USB_IP_DEV_PDN) ? MT8113_USB_IP_SLEEP_STS : 0);
    case MT8113_USB_IP_PW_STS2:
        return MT8113_USB_U2_MAC_RST_B_STS;
    case MT8113_USB_IP_XHCI_CAP:
        return MT8113_USB_XHCI_U2_PORTS;
    default:
        return usb->ippc_regs[offset / sizeof(uint32_t)];
    }
}

static void mt8113_usb_ippc_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    MT8113USBState *usb = opaque;

    usb->ippc_regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps mt8113_usb_ippc_ops = {
    .read = mt8113_usb_ippc_read,
    .write = mt8113_usb_ippc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

#define MT8113_GPT_SYS_RATE_HZ   13000000
#define MT8113_GPT_RTC_RATE_HZ   32768
#define MT8113_GPT_IRQ_EN        0x00
#define MT8113_GPT_IRQ_STA       0x04
#define MT8113_GPT_IRQ_ACK       0x08
#define MT8113_GPT1_CTRL         0x10
#define MT8113_GPT1_CLOCK        0x14
#define MT8113_GPT1_COUNT        0x18
#define MT8113_GPT1_COMPARE      0x1c
#define MT8113_GPT2_COUNT        0x28
#define MT8113_GPT4_COUNT        0x48

#define MT8113_GPT1_IRQ          BIT(0)
#define MT8113_GPT_CTRL_ENABLE   BIT(0)
#define MT8113_GPT_CTRL_CLEAR    BIT(1)
#define MT8113_GPT_CTRL_REPEAT   (1 << 4)
#define MT8113_GPT_CTRL_OP_MASK  (3 << 4)
#define MT8113_GPT_CLOCK_RTC     BIT(4)
#define MT8113_GPT_CLOCK_DIV_MASK 0xf

static uint32_t mt8113_gpt1_rate(MT8113State *s)
{
    uint32_t clock =
        s->timer_regs[MT8113_GPT1_CLOCK / sizeof(uint32_t)];
    uint32_t rate = (clock & MT8113_GPT_CLOCK_RTC) ?
        MT8113_GPT_RTC_RATE_HZ : MT8113_GPT_SYS_RATE_HZ;

    return rate >> MIN(clock & MT8113_GPT_CLOCK_DIV_MASK, 15);
}

static uint64_t mt8113_gpt_ticks(int64_t delta_ns, uint32_t rate)
{
    return muldiv64(delta_ns, rate, NANOSECONDS_PER_SECOND);
}

static void mt8113_gpt_update_irq(MT8113State *s)
{
    qemu_set_irq(s->timer_irq,
                 s->gpt_irq_status &
                 s->timer_regs[MT8113_GPT_IRQ_EN / sizeof(uint32_t)]);
}

static void mt8113_gpt_rearm(MT8113State *s)
{
    uint32_t control =
        s->timer_regs[MT8113_GPT1_CTRL / sizeof(uint32_t)];
    uint32_t compare =
        s->timer_regs[MT8113_GPT1_COMPARE / sizeof(uint32_t)];
    int64_t period_ns;

    timer_del(s->gpt_timer);
    if (!(control & MT8113_GPT_CTRL_ENABLE) || !compare) {
        return;
    }

    period_ns = DIV_ROUND_UP((uint64_t)compare * NANOSECONDS_PER_SECOND,
                             mt8113_gpt1_rate(s));
    timer_mod_ns(s->gpt_timer, s->gpt_start_ns + period_ns);
}

static void mt8113_gpt_expire(void *opaque)
{
    MT8113State *s = opaque;
    uint32_t control =
        s->timer_regs[MT8113_GPT1_CTRL / sizeof(uint32_t)];

    s->gpt_irq_status |= MT8113_GPT1_IRQ;
    mt8113_gpt_update_irq(s);

    if ((control & MT8113_GPT_CTRL_OP_MASK) == MT8113_GPT_CTRL_REPEAT) {
        s->gpt_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        mt8113_gpt_rearm(s);
    }
}

static uint64_t mt8113_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8113State *s = opaque;
    int64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /*
     * U-Boot uses GPT4 (counter at 0x48), while Linux configures GPT2 as
     * its free-running clocksource (counter at 0x28).  Both counters run
     * from the same 13 MHz source on MT8113.
     */
    if (offset == MT8113_GPT2_COUNT || offset == MT8113_GPT4_COUNT) {
        return mt8113_gpt_ticks(now_ns, MT8113_GPT_SYS_RATE_HZ);
    }
    if (offset == MT8113_GPT_IRQ_STA) {
        return s->gpt_irq_status;
    }
    if (offset == MT8113_GPT1_COUNT) {
        uint32_t control =
            s->timer_regs[MT8113_GPT1_CTRL / sizeof(uint32_t)];
        uint32_t compare =
            s->timer_regs[MT8113_GPT1_COMPARE / sizeof(uint32_t)];
        uint64_t count;

        if (!(control & MT8113_GPT_CTRL_ENABLE)) {
            return 0;
        }
        count = mt8113_gpt_ticks(now_ns - s->gpt_start_ns,
                                 mt8113_gpt1_rate(s));
        if ((control & MT8113_GPT_CTRL_OP_MASK) ==
            MT8113_GPT_CTRL_REPEAT && compare) {
            count %= compare;
        } else if (compare) {
            count = MIN(count, compare);
        }
        return count;
    }
    return s->timer_regs[offset / sizeof(uint32_t)];
}

static void mt8113_timer_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    MT8113State *s = opaque;
    uint32_t old_control;

    switch (offset) {
    case MT8113_GPT_IRQ_EN:
        s->timer_regs[offset / sizeof(uint32_t)] = value;
        mt8113_gpt_update_irq(s);
        break;
    case MT8113_GPT_IRQ_ACK:
        s->gpt_irq_status &= ~value;
        mt8113_gpt_update_irq(s);
        break;
    case MT8113_GPT1_CTRL:
        old_control = s->timer_regs[offset / sizeof(uint32_t)];
        s->timer_regs[offset / sizeof(uint32_t)] =
            value & ~MT8113_GPT_CTRL_CLEAR;
        if ((value & MT8113_GPT_CTRL_CLEAR) ||
            (!(old_control & MT8113_GPT_CTRL_ENABLE) &&
             (value & MT8113_GPT_CTRL_ENABLE))) {
            s->gpt_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        mt8113_gpt_rearm(s);
        break;
    case MT8113_GPT1_CLOCK:
        s->timer_regs[offset / sizeof(uint32_t)] = value;
        if (s->timer_regs[MT8113_GPT1_CTRL / sizeof(uint32_t)] &
            MT8113_GPT_CTRL_ENABLE) {
            s->gpt_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            mt8113_gpt_rearm(s);
        }
        break;
    case MT8113_GPT1_COMPARE:
        s->timer_regs[offset / sizeof(uint32_t)] = value;
        if (s->timer_regs[MT8113_GPT1_CTRL / sizeof(uint32_t)] &
            MT8113_GPT_CTRL_ENABLE) {
            mt8113_gpt_rearm(s);
        }
        break;
    default:
        s->timer_regs[offset / sizeof(uint32_t)] = value;
        break;
    }
}

static const MemoryRegionOps mt8113_timer_ops = {
    .read = mt8113_timer_read,
    .write = mt8113_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mt8113_dvfsrc_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    MT8113State *s = opaque;

    return s->dvfsrc_regs[offset / sizeof(uint32_t)];
}

static void mt8113_dvfsrc_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    MT8113State *s = opaque;
    uint32_t level;

    s->dvfsrc_regs[offset / sizeof(uint32_t)] = value;
    if (offset != MT8113_DVFSRC_FORCE) {
        return;
    }

    /*
     * The SPM firmware acknowledges a forced DVFS level through SW_RSV_9.
     * Keep the DVFSRC current-level field and the SPM result coherent so the
     * Linux driver observes completion instead of spending 1 ms in every
     * request timeout and dumping the complete controller state.
     */
    level = value & 0xffff;
    if (!level) {
        level = MT8113_DVFS_DEFAULT_LEVEL;
    }
    s->scpsys_regs[MT8113_SCPSYS_SW_RSV_9 / sizeof(uint32_t)] = level;
    s->dvfsrc_regs[MT8113_DVFSRC_LEVEL / sizeof(uint32_t)] = level << 16;
}

static const MemoryRegionOps mt8113_dvfsrc_ops = {
    .read = mt8113_dvfsrc_read,
    .write = mt8113_dvfsrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t mt8113_toprgu_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    MT8113State *s = opaque;

    return s->toprgu_regs[offset / sizeof(uint32_t)];
}

static void mt8113_toprgu_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    MT8113State *s = opaque;

    if (offset == MT8113_WDT_MODE &&
        (value & 0xff000000) != MT8113_WDT_MODE_KEY) {
        return;
    }
    s->toprgu_regs[offset / sizeof(uint32_t)] = value;
    if (offset == MT8113_WDT_SWRST && value == MT8113_WDT_SWRST_KEY) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static const MemoryRegionOps mt8113_toprgu_ops = {
    .read = mt8113_toprgu_read,
    .write = mt8113_toprgu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mt8113_reset(DeviceState *dev)
{
    MT8113State *s = MT8113(dev);

    timer_del(s->gpt_timer);
    s->gpt_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->gpt_irq_status = 0;
    s->rng_ctrl = 0;
    s->wifi_firmware_ready = false;

    memset(s->topckgen_regs, 0, sizeof(s->topckgen_regs));
    memset(s->infrasys_regs, 0, sizeof(s->infrasys_regs));
    memset(s->scpsys_regs, 0, sizeof(s->scpsys_regs));
    memset(s->toprgu_regs, 0, sizeof(s->toprgu_regs));
    memset(s->timer_regs, 0, sizeof(s->timer_regs));
    memset(s->apmixedsys_regs, 0, sizeof(s->apmixedsys_regs));
    memset(s->dvfsrc_regs, 0, sizeof(s->dvfsrc_regs));
    memset(s->mcucfg_regs, 0, sizeof(s->mcucfg_regs));
    memset(s->svs_regs, 0, sizeof(s->svs_regs));
    memset(s->btif_regs, 0, sizeof(s->btif_regs));
    memset(s->btif_tx_dma_regs, 0, sizeof(s->btif_tx_dma_regs));
    memset(s->btif_rx_dma_regs, 0, sizeof(s->btif_rx_dma_regs));
    memset(s->wifi_regs, 0, sizeof(s->wifi_regs));
    for (int i = 0; i < ARRAY_SIZE(s->usbphy); i++) {
        memset(s->usbphy[i].regs, 0, sizeof(s->usbphy[i].regs));
    }
    memset(s->usb.mac_regs, 0, sizeof(s->usb.mac_regs));
    memset(s->usb.ippc_regs, 0, sizeof(s->usb.ippc_regs));

    s->scpsys_regs[MT8113_SCPSYS_PCM_REG15_DATA / sizeof(uint32_t)] = 1;
    s->scpsys_regs[MT8113_SCPSYS_SW_RSV_9 / sizeof(uint32_t)] =
        MT8113_DVFS_DEFAULT_LEVEL;
    s->apmixedsys_regs[MT8113_APMIXED_UNIVPLL2_CON0 / 4] =
        MT8113_PLL_ENABLE | MT8113_PLL_RST_BAR;
    s->apmixedsys_regs[MT8113_APMIXED_UNIVPLL2_CON1 / 4] =
        MT8113_PLL_PCW_CHG | 0x00180000;
    s->apmixedsys_regs[MT8113_APMIXED_MAINPLL_CON0 / 4] =
        MT8113_PLL_ENABLE | MT8113_PLL_RST_BAR;
    s->apmixedsys_regs[MT8113_APMIXED_MAINPLL_CON1 / 4] =
        MT8113_PLL_PCW_CHG | BIT(24) | 0x00150000;
    s->apmixedsys_regs[MT8113_APMIXED_ARMPLL_CON0 / 4] =
        MT8113_PLL_ENABLE;
    s->apmixedsys_regs[MT8113_APMIXED_ARMPLL_CON1 / 4] =
        MT8113_PLL_PCW_CHG | (2 << 24) | 0x001713b2;
    s->apmixedsys_regs[MT8113_APMIXED_ARMPLL_PWR_CON0 / 4] =
        MT8113_PLL_POWER_ON;
    s->apmixedsys_regs[MT8113_APMIXED_TCONPLL_CON0 / 4] =
        MT8113_PLL_ENABLE;
    s->apmixedsys_regs[MT8113_APMIXED_TCONPLL_CON1 / 4] = 0x82127627;
    s->topckgen_regs[0xd0 / 4] = 6 << 24;
    s->topckgen_regs[0xe0 / 4] = 2;
    s->mcucfg_regs[MT8113_MCU_BUS_PLL_DIVIDER_CFG / 4] =
        MT8113_MCU_BUS_SEL_ARMPLL;
    s->dvfsrc_regs[MT8113_DVFSRC_LEVEL / sizeof(uint32_t)] =
        MT8113_DVFS_DEFAULT_LEVEL << 16;

    mt8113_btif_wmt_session_reset(s);
    qemu_set_irq(s->timer_irq, 0);
    qemu_set_irq(s->svs_irq, 0);
    qemu_set_irq(s->btif_irq, 0);
    qemu_set_irq(s->wifi_irq, 0);
}

static bool mt8113_realize_gic(MT8113State *s, Error **errp)
{
    DeviceState *gicdev = DEVICE(&s->gic);
    SysBusDevice *gicsbd = SYS_BUS_DEVICE(&s->gic);
    QList *redist_region_count = qlist_new();
    static const int timer_irqs[] = {
        [GTIMER_PHYS] = ARCH_TIMER_NS_EL1_IRQ,
        [GTIMER_VIRT] = ARCH_TIMER_VIRT_IRQ,
        [GTIMER_HYP] = ARCH_TIMER_NS_EL2_IRQ,
        [GTIMER_SEC] = ARCH_TIMER_S_EL1_IRQ,
    };

    qdev_prop_set_uint32(gicdev, "num-cpu", MT8113_NUM_CPUS);
    qdev_prop_set_uint32(gicdev, "num-irq", MT8113_NUM_SPIS + GIC_INTERNAL);
    qlist_append_int(redist_region_count, MT8113_NUM_CPUS);
    qdev_prop_set_array(gicdev, "redist-region-count", redist_region_count);
    object_property_set_link(OBJECT(&s->gic), "sysmem",
                             OBJECT(get_system_memory()), &error_abort);
    if (!sysbus_realize(gicsbd, errp)) {
        return false;
    }
    sysbus_mmio_map(gicsbd, 0, MT8113_GIC_DIST_ADDR);
    sysbus_mmio_map(gicsbd, 1, MT8113_GIC_REDIST_ADDR);

    for (int cpu = 0; cpu < MT8113_NUM_CPUS; cpu++) {
        DeviceState *cpudev = DEVICE(&s->cpu[cpu]);
        int intidbase = MT8113_NUM_SPIS + cpu * GIC_INTERNAL;

        for (int irq = 0; irq < ARRAY_SIZE(timer_irqs); irq++) {
            qdev_connect_gpio_out(cpudev, irq,
                                  qdev_get_gpio_in(gicdev,
                                      intidbase + timer_irqs[irq]));
        }
        qdev_connect_gpio_out_named(cpudev,
                                    "gicv3-maintenance-interrupt", 0,
                                    qdev_get_gpio_in(gicdev,
                                        intidbase + ARCH_GIC_MAINT_IRQ));
        qdev_connect_gpio_out_named(cpudev, "pmu-interrupt", 0,
                                    qdev_get_gpio_in(gicdev,
                                        intidbase + VIRTUAL_PMU_IRQ));
        sysbus_connect_irq(gicsbd, cpu,
                           qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(gicsbd, cpu + MT8113_NUM_CPUS,
                           qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        sysbus_connect_irq(gicsbd, cpu + 2 * MT8113_NUM_CPUS,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
        sysbus_connect_irq(gicsbd, cpu + 3 * MT8113_NUM_CPUS,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    }
    return true;
}

static void mt8113_realize(DeviceState *dev, Error **errp)
{
    MT8113State *s = MT8113(dev);
    DeviceState *gicdev = DEVICE(&s->gic);
    static const hwaddr i2c_addr[] = {
        MT8113_I2C0_ADDR, MT8113_I2C1_ADDR, MT8113_I2C2_ADDR,
    };
    static const hwaddr i2c_dma_addr[] = {
        MT8113_I2C0_DMA_ADDR, MT8113_I2C1_DMA_ADDR,
        MT8113_I2C2_DMA_ADDR,
    };
    static const int i2c_irq[] = {
        MT8113_I2C0_IRQ, MT8113_I2C1_IRQ, MT8113_I2C2_IRQ,
    };

    for (int cpu = 0; cpu < MT8113_NUM_CPUS; cpu++) {
        Object *cpuobj = OBJECT(&s->cpu[cpu]);

        object_property_set_int(cpuobj, "cntfrq", 13000000, &error_abort);
        object_property_set_bool(cpuobj, "has_el2", false, &error_abort);
        object_property_set_bool(cpuobj, "has_el3", true, &error_abort);
        object_property_set_int(cpuobj, "rvbar", s->reset_vector,
                                &error_abort);
        object_property_set_int(cpuobj, "psci-conduit",
                                QEMU_PSCI_CONDUIT_SMC, &error_abort);
        object_property_set_int(cpuobj, "mtk-sip-vcorefs", 1,
                                &error_abort);
        object_property_set_int(cpuobj, "mtk-optee-fbe", 1,
                                &error_abort);
        object_property_set_int(cpuobj, "mp-affinity",
                                arm_build_mp_affinity(cpu,
                                                      MT8113_NUM_CPUS),
                                &error_abort);
        if (cpu) {
            object_property_set_bool(cpuobj, "start-powered-off", true,
                                     &error_abort);
        }
        if (!qdev_realize(DEVICE(cpuobj), NULL, errp)) {
            return;
        }
    }
    if (!mt8113_realize_gic(s, errp)) {
        return;
    }

    s->timer_irq = qdev_get_gpio_in(gicdev, MT8113_TIMER_IRQ);
    s->gpt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mt8113_gpt_expire, s);

    memory_region_init_ram(&s->sram, OBJECT(s), "mt8113.sram",
                           MT8113_SRAM_SIZE, errp);
    if (*errp) {
        return;
    }
    memory_region_add_subregion(get_system_memory(), MT8113_SRAM_BASE,
                                &s->sram);

    serial_mm_init(get_system_memory(), MT8113_UART0_ADDR, 2,
                   qdev_get_gpio_in(gicdev, MT8113_UART0_IRQ), 921600,
                   serial_hd(0), DEVICE_LITTLE_ENDIAN);

    memory_region_init_io(&s->topckgen_iomem, OBJECT(s),
                          &mt8113_topckgen_ops, s,
                          "mt8113.topckgen", 0x1000);
    memory_region_add_subregion(get_system_memory(), MT8113_TOPCKGEN_ADDR,
                                &s->topckgen_iomem);

    memory_region_init_io(&s->infrasys_iomem, OBJECT(s),
                          &mt8113_infrasys_ops, s,
                          "mt8113.infrasys", 0x1000);
    memory_region_add_subregion(get_system_memory(), MT8113_INFRASYS_ADDR,
                                &s->infrasys_iomem);

    memory_region_init_io(&s->scpsys_iomem, OBJECT(s), &mt8113_scpsys_ops,
                          s, "mt8113.scpsys", 0x1000);
    s->scpsys_regs[MT8113_SCPSYS_PCM_REG15_DATA / sizeof(uint32_t)] = 1;
    s->scpsys_regs[MT8113_SCPSYS_SW_RSV_9 / sizeof(uint32_t)] =
        MT8113_DVFS_DEFAULT_LEVEL;
    s->scpsys_regs[MT8113_SCPSYS_DVFS_EVENT_STA / sizeof(uint32_t)] = 0;
    memory_region_add_subregion(get_system_memory(), MT8113_SCPSYS_ADDR,
                                &s->scpsys_iomem);

    memory_region_init_io(&s->toprgu_iomem, OBJECT(s), &mt8113_toprgu_ops, s,
                          "mt8113.toprgu", sizeof(s->toprgu_regs));
    memory_region_add_subregion(get_system_memory(), MT8113_TOPRGU_ADDR,
                                &s->toprgu_iomem);

    memory_region_init_io(&s->timer_iomem, OBJECT(s), &mt8113_timer_ops, s,
                          "mt8113.timer", 0x1000);
    memory_region_add_subregion(get_system_memory(), MT8113_TIMER_ADDR,
                                &s->timer_iomem);

    memory_region_init_io(&s->apmixedsys_iomem, OBJECT(s),
                          &mt8113_apmixedsys_ops, s,
                          "mt8113.apmixedsys", 0x1000);
    s->apmixedsys_regs[MT8113_APMIXED_UNIVPLL2_CON0 / 4] =
        MT8113_PLL_ENABLE | MT8113_PLL_RST_BAR;
    s->apmixedsys_regs[MT8113_APMIXED_UNIVPLL2_CON1 / 4] =
        MT8113_PLL_PCW_CHG | 0x00180000; /* 2496 MHz, /6 = 416 MHz */
    s->apmixedsys_regs[MT8113_APMIXED_MAINPLL_CON0 / 4] =
        MT8113_PLL_ENABLE | MT8113_PLL_RST_BAR;
    s->apmixedsys_regs[MT8113_APMIXED_MAINPLL_CON1 / 4] =
        MT8113_PLL_PCW_CHG | BIT(24) | 0x00150000; /* 1092 MHz */
    s->apmixedsys_regs[MT8113_APMIXED_ARMPLL_CON0 / 4] =
        MT8113_PLL_ENABLE;
    s->apmixedsys_regs[MT8113_APMIXED_ARMPLL_CON1 / 4] =
        MT8113_PLL_PCW_CHG | (2 << 24) | 0x001713b2; /* 600 MHz */
    s->apmixedsys_regs[MT8113_APMIXED_ARMPLL_PWR_CON0 / 4] =
        MT8113_PLL_POWER_ON;
    s->apmixedsys_regs[MT8113_APMIXED_TCONPLL_CON0 / 4] =
        MT8113_PLL_ENABLE;
    s->apmixedsys_regs[MT8113_APMIXED_TCONPLL_CON1 / 4] =
        0x82127627; /* 480 MHz */
    /* mm_sel = univpll1_d2 (312 MHz), as handed off by platform firmware. */
    s->topckgen_regs[0xd0 / 4] = 6 << 24;
    s->topckgen_regs[0xe0 / 4] = 2; /* tconpll_d4 = 120 MHz */
    memory_region_add_subregion(get_system_memory(), MT8113_APMIXEDSYS_ADDR,
                                &s->apmixedsys_iomem);

    memory_region_init_io(&s->mcucfg_iomem, OBJECT(s), &mt8113_mcucfg_ops, s,
                          "mt8113.mcucfg", sizeof(s->mcucfg_regs));
    s->mcucfg_regs[MT8113_MCU_BUS_PLL_DIVIDER_CFG / 4] =
        MT8113_MCU_BUS_SEL_ARMPLL;
    memory_region_add_subregion(get_system_memory(), MT8113_MCUCFG_ADDR,
                                &s->mcucfg_iomem);

    memory_region_init_io(&s->dvfsrc_iomem, OBJECT(s), &mt8113_dvfsrc_ops, s,
                          "mt8113.dvfsrc", sizeof(s->dvfsrc_regs));
    s->dvfsrc_regs[MT8113_DVFSRC_LEVEL / sizeof(uint32_t)] =
        MT8113_DVFS_DEFAULT_LEVEL << 16;
    memory_region_add_subregion(get_system_memory(), MT8113_DVFSRC_ADDR,
                                &s->dvfsrc_iomem);

    memory_region_init_io(&s->rng_iomem, OBJECT(s), &mt8113_rng_ops, s,
                          "mt8113.rng", 0x100);
    memory_region_add_subregion(get_system_memory(), MT8113_RNG_ADDR,
                                &s->rng_iomem);

    s->svs_irq = qdev_get_gpio_in(gicdev, MT8113_SVS_IRQ);
    memory_region_init_io(&s->svs_iomem, OBJECT(s), &mt8113_svs_ops, s,
                          "mt8113.svs", sizeof(s->svs_regs));
    memory_region_add_subregion(get_system_memory(), MT8113_SVS_ADDR,
                                &s->svs_iomem);

    s->btif_irq = qdev_get_gpio_in(gicdev, MT8113_BTIF_IRQ);
    s->btif_tx_dma_irq =
        qdev_get_gpio_in(gicdev, MT8113_BTIF_TX_DMA_IRQ);
    s->btif_rx_dma_irq =
        qdev_get_gpio_in(gicdev, MT8113_BTIF_RX_DMA_IRQ);
    s->wifi_irq = qdev_get_gpio_in(gicdev, MT8113_WIFI_IRQ);
    memory_region_init_io(&s->btif_iomem, OBJECT(s), &mt8113_btif_ops, s,
                          "mt8113.btif", sizeof(s->btif_regs));
    memory_region_add_subregion(get_system_memory(), MT8113_BTIF_ADDR,
                                &s->btif_iomem);
    memory_region_init_io(&s->btif_tx_dma_iomem, OBJECT(s),
                          &mt8113_btif_tx_dma_ops, s,
                          "mt8113.btif-tx-dma",
                          sizeof(s->btif_tx_dma_regs));
    memory_region_add_subregion(get_system_memory(),
                                MT8113_BTIF_TX_DMA_ADDR,
                                &s->btif_tx_dma_iomem);
    memory_region_init_io(&s->btif_rx_dma_iomem, OBJECT(s),
                          &mt8113_btif_rx_dma_ops, s,
                          "mt8113.btif-rx-dma",
                          sizeof(s->btif_rx_dma_regs));
    memory_region_add_subregion(get_system_memory(),
                                MT8113_BTIF_RX_DMA_ADDR,
                                &s->btif_rx_dma_iomem);

    memory_region_init_io(&s->wifi_iomem, OBJECT(s), &mt8113_wifi_ops, s,
                          "mt8113.wifi", MT8113_WIFI_SIZE);
    memory_region_add_subregion(get_system_memory(), MT8113_WIFI_ADDR,
                                &s->wifi_iomem);

    memory_region_init_io(&s->efuse_iomem, OBJECT(s), &mt8113_efuse_ops, s,
                          "mt8113.efuse", sizeof(s->efuse_regs));
    /* Nominal MT8512 thermal ADC calibration: zero gain/offset trim, 40 C. */
    s->efuse_regs[(0x180 / 4) + 0] = 0x80200000;
    s->efuse_regs[(0x180 / 4) + 1] = 0x60904000;
    s->efuse_regs[(0x180 / 4) + 4] = 0x0a000000;
    s->efuse_regs[(0x580 / 4) + 0] = 0x00000010;
    s->efuse_regs[(0x580 / 4) + 1] = 0x20202020;
    s->efuse_regs[(0x580 / 4) + 2] = 0x20200000;
    memory_region_add_subregion(get_system_memory(), MT8113_EFUSE_ADDR,
                                &s->efuse_iomem);

    for (int i = 0; i < ARRAY_SIZE(s->usbphy); i++) {
        static const hwaddr usbphy_addr[] = {
            MT8113_USBPHY0_ADDR, MT8113_USBPHY1_ADDR,
        };
        g_autofree char *name = g_strdup_printf("mt8113.usbphy%d", i);

        memory_region_init_io(&s->usbphy[i].iomem, OBJECT(s),
                              &mt8113_usbphy_ops, &s->usbphy[i], name,
                              sizeof(s->usbphy[i].regs));
        memory_region_add_subregion(get_system_memory(), usbphy_addr[i],
                                    &s->usbphy[i].iomem);
    }

    memory_region_init_io(&s->usb.mac_iomem, OBJECT(s),
                          &mt8113_usb_mac_ops, &s->usb,
                          "mt8113.usb-mac", MT8113_USB_MAC_SIZE);
    memory_region_add_subregion(get_system_memory(), MT8113_USB_MAC_ADDR,
                                &s->usb.mac_iomem);
    memory_region_init_io(&s->usb.ippc_iomem, OBJECT(s),
                          &mt8113_usb_ippc_ops, &s->usb,
                          "mt8113.usb-ippc", MT8113_USB_IPPC_SIZE);
    memory_region_add_subregion(get_system_memory(), MT8113_USB_IPPC_ADDR,
                                &s->usb.ippc_iomem);

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0, MT8113_GPIO_ADDR);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 1, MT8113_EINT_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), 0,
                       qdev_get_gpio_in(gicdev, MT8113_EINT_IRQ));

    for (int i = 0; i < ARRAY_SIZE(s->i2c); i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->i2c[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 0, i2c_addr[i]);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 1, i2c_dma_addr[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(gicdev, i2c_irq[i]));
    }

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->msdc0), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->msdc0), 0, MT8113_MSDC0_ADDR);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->msdc0), 1, MT8113_MSDC0_TOP_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->msdc0), 0,
                       qdev_get_gpio_in(gicdev, MT8113_MSDC0_IRQ));

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gce), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gce), 0, MT8113_GCE_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gce), 0,
                       qdev_get_gpio_in(gicdev, MT8113_GCE_IRQ));

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->iommu), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->iommu), 0, MT8113_IOMMU_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->iommu), 0,
                       qdev_get_gpio_in(gicdev, MT8113_IOMMU_IRQ));

    object_property_set_link(OBJECT(&s->hwtcon), "iommu",
                             OBJECT(&s->iommu), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->hwtcon), errp)) {
        return;
    }
    qdev_connect_gpio_out_named(DEVICE(&s->hwtcon), "gce-frame-done", 0,
        qdev_get_gpio_in_named(DEVICE(&s->gce), "hwtcon-frame-done", 0));
    qdev_connect_gpio_out_named(DEVICE(&s->hwtcon), "mdp-wrot-irq", 0,
        qdev_get_gpio_in(gicdev, MT8113_MDP_WROT_IRQ));
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->hwtcon), 0, MT8113_HWTCON_ADDR);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->hwtcon), 1,
                    MT8113_HWTCON_IMG_ADDR);
    static const int hwtcon_irq[] = {
        MT8113_HWTCON_WB_FRAME_DONE_SPI,
        MT8113_HWTCON_WF_LUT_FRAME_DONE_SPI,
        MT8113_HWTCON_WF_LUT_RELEASE_SPI,
        MT8113_HWTCON_DPI_UPDATE_DONE_SPI,
        MT8113_HWTCON_TCON_END_SPI,
        MT8113_HWTCON_PIXEL_LUT_COLLISION_SPI,
        MT8113_HWTCON_DPI_SOF_SPI,
    };

    for (int i = 0; i < ARRAY_SIZE(hwtcon_irq); i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->hwtcon), i,
                           qdev_get_gpio_in(gicdev, hwtcon_irq[i]));
    }
}

static const Property mt8113_properties[] = {
    DEFINE_PROP_UINT64("reset-vector", MT8113State, reset_vector, 0),
};

static void mt8113_init(Object *obj)
{
    MT8113State *s = MT8113(obj);

    for (int cpu = 0; cpu < MT8113_NUM_CPUS; cpu++) {
        g_autofree char *name = g_strdup_printf("cpu%d", cpu);

        object_initialize_child(obj, name, &s->cpu[cpu],
                                ARM_CPU_TYPE_NAME("cortex-a53"));
    }
    object_initialize_child(obj, "gic", &s->gic, TYPE_ARM_GICV3);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_MT8113_GPIO);
    for (int i = 0; i < ARRAY_SIZE(s->i2c); i++) {
        g_autofree char *name = g_strdup_printf("i2c%d", i);

        object_initialize_child(obj, name, &s->i2c[i], TYPE_MTK_I2C);
    }
    object_initialize_child(obj, "msdc0", &s->msdc0, TYPE_MTK_MSDC);
    object_initialize_child(obj, "gce", &s->gce, TYPE_MT8113_GCE);
    object_initialize_child(obj, "iommu", &s->iommu, TYPE_MT8113_IOMMU);
    object_initialize_child(obj, "hwtcon", &s->hwtcon, TYPE_MT8113_HWTCON);
}

static void mt8113_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->desc = "MediaTek MT8113 SoC";
    dc->realize = mt8113_realize;
    device_class_set_legacy_reset(dc, mt8113_reset);
    device_class_set_props(dc, mt8113_properties);
    dc->user_creatable = false;
}

static const TypeInfo mt8113_type_info = {
    .name = TYPE_MT8113,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(MT8113State),
    .instance_init = mt8113_init,
    .class_init = mt8113_class_init,
};

/* MT8110 and MT8113 expose the same MT8512-class integration used here. */
static void mt8110_init(Object *obj)
{
    MT8113State *s = MT8113(obj);

    object_property_set_bool(OBJECT(&s->gce), "inclusive-end-address", false,
                             &error_abort);
    object_property_set_bool(OBJECT(&s->hwtcon), "scanout-image-buffer", true,
                             &error_abort);
    object_property_set_bool(OBJECT(&s->hwtcon), "retain-boot-splash", true,
                             &error_abort);
}

static const TypeInfo mt8110_type_info = {
    .name = TYPE_MT8110,
    .parent = TYPE_MT8113,
    .instance_init = mt8110_init,
};

static void mt8113_register_types(void)
{
    type_register_static(&mt8113_type_info);
    type_register_static(&mt8110_type_info);
}
type_init(mt8113_register_types)
