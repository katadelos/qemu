/*
 * MT8171 MCUPM AP mailbox and firmware services.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Implements the AP-visible protocol from mcupm/v2, mtk-mbox.c and
 * clk-fhctl-{mcupm,ap,pll}.c in the vendor kernel. The original BL2 SRAM
 * bootstrap remains board-owned. This is a service model, not RV33 execution.
 */
#include "qemu/osdep.h"
#include "hw/misc/mt8171-mcupm.h"
#include "hw/core/irq.h"
#include "system/address-spaces.h"
#include "qemu/module.h"
#include "qemu/log.h"

#define MBOX_BASE 0x0c55fb00
#define FH_BASE 0x1000ce00
#define PLL_BASE 0x1000c000
#define DDS_MASK 0x003fffff
#define PLT_INIT 0x504c5401
#define LOG_ENABLE 0x504c5402
#define SERV_READY 0x504c5403
/* Linux protocol errors, independent of the host's errno numbering. */
#define MC_EINVAL ((uint32_t)-22)
#define MC_EFAULT ((uint32_t)-14)
#define MC_ENODEV ((uint32_t)-19)
#define MC_ENOTSUP ((uint32_t)-95)

static uint32_t mc_read32(hwaddr addr)
{
    uint8_t b[4];
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       b, sizeof(b));
    return ldl_le_p(b);
}

static void mc_write32(hwaddr addr, uint32_t value)
{
    uint8_t b[4];
    stl_le_p(b, value);
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        b, sizeof(b));
}

static void mc_update32(hwaddr addr, uint32_t mask, uint32_t value)
{
    mc_write32(addr, (mc_read32(addr) & ~mask) | (value & mask));
}

static void mc_irqs(MT8171MCUPMState *s)
{
    for (unsigned i = 0; i < 8; i++) {
        qemu_set_irq(s->irq[i], !!(s->rx_pending & BIT(i)));
    }
}

static bool mc_dram_range(uint32_t base, uint32_t size)
{
    /* PA6-CS8 DRAM starts at 0x40000000. Check mapped RAM as well. */
    return base >= 0x40000000 && size >= 4 &&
           (uint64_t)base + size <= 0x100000000ULL &&
           address_space_access_valid(&address_space_memory, base, size,
                                      true, MEMTXATTRS_UNSPECIFIED);
}

static uint32_t mc_platform(MT8171MCUPMState *s, const uint32_t *tx)
{
    uint32_t base = tx[1], size = tx[2], hdr, logger;

    switch (tx[0]) {
    case 0xdead:
        return 1;
    case PLT_INIT:
        if (!mc_dram_range(base, size) || size < 16) {
            return MC_EFAULT;
        }
        hdr = mc_read32(base + 4);
        if (mc_read32(base) != PLT_INIT ||
            mc_read32(base + 8) != size ||
            mc_read32((uint64_t)base + size - 4) != PLT_INIT ||
            (hdr != 12 && hdr != 16)) {
            return MC_EINVAL;
        }
        logger = 0;
        if (hdr == 16) {
            uint32_t off = mc_read32(base + 12);
            uint32_t buff, len, info;
            if (off < hdr || (uint64_t)off + 24 > size - 4) {
                return MC_EINVAL;
            }
            logger = base + off;
            info = mc_read32(logger + 12);
            buff = mc_read32(logger + 16);
            len = mc_read32(logger + 20);
            if (mc_read32(logger) != LOG_ENABLE ||
                mc_read32(logger + 4) != 24 || info < 24 ||
                (uint64_t)info + 8 > buff ||
                (uint64_t)off + buff + len > size - 4) {
                return MC_EINVAL;
            }
        }
        s->shared_base = base;
        s->shared_size = size;
        s->logger_base = logger;
        s->service_ready = true;
        return 0;
    case LOG_ENABLE:
        if (!s->logger_base) {
            return MC_ENODEV;
        }
        if (tx[1] != 0x101 && tx[1] != 1) {
            return MC_EINVAL;
        }
        mc_write32(s->logger_base + 8, tx[1] == 0x101);
        return tx[1] == 0x101;
    case SERV_READY:
        s->service_ready = true;
        return 0;
    default:
        return MC_ENOTSUP;
    }
}

/* clk-mt8171.c PLL register definitions. The FHCTL driver's MMPLL entry
 * mistakenly repeats CCIPLL's 0x328; the physical MMPLL CON1 is 0x608. */
static const uint16_t pcw_offset[11] = {
    0x308, 0x318, 0x328, 0x428, 0x608, 0x618,
    0x628, 0x638, 0x508, 0x408, 0x418,
};

static void mc_ssc(MT8171MCUPMState *s, unsigned id, uint32_t rate)
{
    hwaddr cfg = FH_BASE + 0x3c + id * 0x14;
    uint32_t dds = mc_read32(PLL_BASE + pcw_offset[id]) & DDS_MASK;

    mc_update32(FH_BASE + 8, BIT(id), BIT(id));
    mc_update32(FH_BASE + 12, BIT(id), BIT(id));
    mc_update32(cfg, 7, 0);
    if (rate) {
        mc_update32(cfg, 0x00ff0000,
                    (s->ssc_df[id] << 20) | (s->ssc_dt[id] << 16));
        mc_write32(cfg + 8, dds | BIT(31));
        mc_write32(cfg + 4, ((((uint64_t)dds * rate) >> 5) / 100) << 16);
        mc_update32(FH_BASE, BIT(id), BIT(id));
        mc_update32(cfg, 7, 3);
    } else {
        mc_update32(FH_BASE, BIT(id), 0);
    }
    s->ssc_rate[id] = rate;
}

static uint32_t mc_fhctl(MT8171MCUPMState *s, const uint32_t *tx)
{
    unsigned id = tx[1];
    hwaddr cfg, pcw;
    uint32_t dds, div, old_rate;

    switch (tx[0]) {
    case 0x2001: return s->tr_begin;
    case 0x2002: return s->tr_begin >> 32;
    case 0x2003: return s->tr_end;
    case 0x2004: return s->tr_end >> 32;
    case 0x2005: return s->tr_id;
    case 0x2006: return s->tr_value;
    case 0x2007: return s->tr_begin;
    case 0x2008: return s->tr_end;
    }
    if (id >= ARRAY_SIZE(pcw_offset)) {
        return MC_EINVAL;
    }
    cfg = FH_BASE + 0x3c + id * 0x14;
    pcw = PLL_BASE + pcw_offset[id];
    switch (tx[0]) {
    case 0x1004:
        if (tx[3] > 15 || tx[4] > 15 || tx[5] || tx[6] > 1000) {
            return MC_EINVAL;
        }
        s->ssc_dt[id] = tx[3];
        s->ssc_df[id] = tx[4];
        mc_ssc(s, id, tx[6]);
        return 0;
    case 0x1005:
        mc_ssc(s, id, 0);
        return 0;
    case 0x1006:
        dds = tx[2];
        div = tx[3];
        if ((dds & ~DDS_MASK) || (div != UINT32_MAX &&
            (div == 0 || div > 16 || (div & (div - 1))))) {
            return MC_EINVAL;
        }
        s->tr_begin = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->tr_id = tx[8];
        s->tr_value = dds;
        old_rate = s->ssc_rate[id];
        mc_ssc(s, id, 0);
        mc_write32(cfg + 8, (mc_read32(pcw) & DDS_MASK) | BIT(31));
        mc_update32(cfg, 7, 5);
        mc_write32(FH_BASE + 0x10, 0x06003c97);
        mc_write32(FH_BASE + 0x14, 0x06003c97);
        mc_update32(FH_BASE, BIT(id), BIT(id));
        mc_write32(cfg + 12, dds | BIT(31));
        mc_write32(cfg + 16, dds);
        /* MT8171 PLL_SETCLR defines post-divider exponent in bits 26:24. */
        if (div != UINT32_MAX) {
            mc_update32(pcw, 7 << 24, ctz32(div) << 24);
        }
        mc_update32(pcw, DDS_MASK | BIT(31), dds | BIT(31));
        mc_update32(FH_BASE, BIT(id), 0);
        if (old_rate) {
            mc_ssc(s, id, old_rate);
        }
        s->tr_end = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        return s->tr_id;
    default:
        return MC_ENOTSUP;
    }
}

static void mc_service(void *opaque)
{
    MT8171MCUPMState *s = opaque;
    uint32_t pending = s->tx_pending & ~s->rx_pending;

    if (!s->firmware_loaded || (s->regs[0] & 3) != 3) {
        return;
    }
    for (unsigned i = 0; i < 8; i++) {
        uint32_t result;
        if (!(pending & BIT(i))) {
            continue;
        }
        result = i == 0 ? mc_platform(s, s->tx[i]) :
                 i == 2 ? mc_fhctl(s, s->tx[i]) : MC_ENOTSUP;
        if (result == MC_ENOTSUP) {
            qemu_log_mask(LOG_UNIMP, "MCUPM unsupported channel %u command %#x\n",
                          i, s->tx[i][0]);
        }
        /* RX buffers can be four words (CPU DVFS and MET), never echo TX. */
        for (unsigned w = 0; w < 20; w++) {
            mc_write32(MBOX_BASE + i * 0xa0 + 0x50 + w * 4,
                       w ? 0 : result);
        }
        s->tx_pending &= ~BIT(i);
        s->rx_pending |= BIT(i);
    }
    mc_irqs(s);
}

static void mc_schedule(MT8171MCUPMState *s)
{
    if (s->tx_pending & ~s->rx_pending) {
        /* Ordered firmware dispatch; AP can observe TX busy before completion. */
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000);
    }
}

static uint64_t mc_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8171MCUPMState *s = opaque;
    switch (offset) {
    case 0x2000: case 0x2004: return s->tx_pending;
    case 0x74: case 0x78: return s->rx_pending;
    default: return s->regs[offset / 4];
    }
}

static void mc_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    MT8171MCUPMState *s = opaque;
    switch (offset) {
    case 0x2004:
        for (unsigned i = 0; i < 8; i++) {
            if ((value & BIT(i)) && !(s->tx_pending & BIT(i))) {
                for (unsigned w = 0; w < 20; w++) {
                    s->tx[i][w] = mc_read32(MBOX_BASE + i * 0xa0 + w * 4);
                }
                s->tx_pending |= BIT(i);
            }
        }
        mc_schedule(s);
        break;
    case 0x74:
        s->rx_pending &= ~value;
        mc_irqs(s);
        mc_schedule(s);
        break;
    case 0x2000: case 0x78:
        break;
    default:
        s->regs[offset / 4] = value;
        if (!offset) {
            mc_schedule(s);
        }
    }
}

static const MemoryRegionOps mc_ops = {
    .read = mc_read,
    .write = mc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

void mt8171_mcupm_set_loaded(MT8171MCUPMState *s, bool loaded)
{
    s->firmware_loaded = loaded;
    s->regs[0] = loaded ? 3 : 0;
}

static void mc_reset(DeviceState *dev)
{
    MT8171MCUPMState *s = MT8171_MCUPM(dev);
    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->tx, 0, sizeof(s->tx));
    memset(s->ssc_rate, 0, sizeof(s->ssc_rate));
    memset(s->ssc_dt, 0, sizeof(s->ssc_dt));
    memset(s->ssc_df, 0, sizeof(s->ssc_df));
    s->tx_pending = s->rx_pending = 0;
    s->shared_base = s->shared_size = s->logger_base = 0;
    s->service_ready = false;
    s->tr_begin = s->tr_end = s->tr_id = s->tr_value = 0;
    s->regs[0] = s->firmware_loaded ? 3 : 0;
    mc_irqs(s);
}

static void mc_init(Object *obj)
{
    MT8171MCUPMState *s = MT8171_MCUPM(obj);
    memory_region_init_io(&s->iomem, obj, &mc_ops, s, TYPE_MT8171_MCUPM, 0x3000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (unsigned i = 0; i < 8; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mc_service, s);
}

static void mc_finalize(Object *obj)
{
    timer_free(MT8171_MCUPM(obj)->timer);
}

static void mc_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), mc_reset);
}

static const TypeInfo mc_type = {
    .name = TYPE_MT8171_MCUPM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8171MCUPMState),
    .instance_init = mc_init,
    .instance_finalize = mc_finalize,
    .class_init = mc_class_init,
};
static void mc_register(void)
{
    type_register_static(&mc_type);
}
type_init(mc_register);
