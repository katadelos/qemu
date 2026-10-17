/*
 * Allwinner B300 power subsystem: RSB, AXP858, AXP2585 and SCP mailbox.
 *
 * The ARM firmware keeps its native OP-TEE monitor. The SCP side of its
 * hardware mailbox implements the vendor's 64-byte ARISC message protocol;
 * direct RSB transfers and SCP transfers share the same physical PMIC state.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/misc/allwinner-b300-power.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "system/runstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "trace.h"

#define RSB_CTRL          (0x00 / 4)
#define RSB_INTE          (0x08 / 4)
#define RSB_STATUS        (0x0c / 4)
#define RSB_ADDRESS       (0x10 / 4)
#define RSB_DATA          (0x1c / 4)
#define RSB_LCR           (0x24 / 4)
#define RSB_MODE          (0x28 / 4)
#define RSB_COMMAND       (0x2c / 4)
#define RSB_SLAVE         (0x30 / 4)
#define RSB_START         BIT(7)
#define RSB_ABORT         BIT(6)
#define RSB_IRQ_ENABLE    BIT(1)
#define RSB_RESET         BIT(0)
#define RSB_DONE          BIT(0)
#define RSB_ERROR         BIT(1)
#define RSB_NACK          BIT(16)

#define SCP_POOL          0x48105000U
#define SCP_POOL_SIZE     0x1000
#define SCP_MESSAGE_SIZE  64
#define SCP_STARTUP_FRAME (SCP_POOL_SIZE - SCP_MESSAGE_SIZE)

static const uint16_t pmic_address[] = { 0x745, 0x3a3 };

static int aw_b300_scp_dvfs(const uint8_t *params)
{
    uint32_t freq = ldl_le_p(params);
    uint32_t pll = ldl_le_p(params + 4);
    MemTxResult result;
    unsigned p, n = 0;
    uint32_t factors, cfg;

    if (pll != 1 || !freq) {
        return -EINVAL;
    }
    /* SUN8IW15: CPU PLL = 24 MHz * (N + 1) / 2^P. Requests use kHz. */
    for (p = 0; p < 4; p++) {
        uint64_t numerator = (uint64_t)freq << p;

        if (!(numerator % 24000) && numerator / 24000 <= 256) {
            n = numerator / 24000;
            break;
        }
    }
    if (!n) {
        return -EINVAL;
    }
    /* The native SCP programs the same CCU registers the kernel reads. */
    factors = address_space_ldl_le(&address_space_memory, 0x03001000,
                                    MEMTXATTRS_UNSPECIFIED, &result);
    if (result != MEMTX_OK) {
        return -EIO;
    }
    factors &= ~(0xffU << 8 | 3U << 16 | BIT(30));
    factors |= (n - 1) << 8 | p << 16 | BIT(31);
    address_space_stl_le(&address_space_memory, 0x03001000, factors,
                         MEMTXATTRS_UNSPECIFIED, &result);
    if (result != MEMTX_OK) {
        return -EIO;
    }
    cfg = address_space_ldl_le(&address_space_memory, 0x03001500,
                                MEMTXATTRS_UNSPECIFIED, &result);
    if (result != MEMTX_OK) {
        return -EIO;
    }
    cfg = deposit32(cfg, 24, 3, 3); /* CPU parent: pll_cpu. */
    address_space_stl_le(&address_space_memory, 0x03001500, cfg,
                         MEMTXATTRS_UNSPECIFIED, &result);
    return result == MEMTX_OK ? 0 : -EIO;
}

static void aw_b300_power_update(AWB300PowerState *s)
{
    bool pending = false;

    qemu_set_irq(s->rsb_irq, (s->rsb[RSB_CTRL] & RSB_IRQ_ENABLE) &&
                 (s->rsb[RSB_INTE] & s->rsb[RSB_STATUS] & 7));
    /* CPUX is user 1; each channel's receive event occupies its even bit. */
    qemu_set_irq(s->msgbox_irq,
                 !!(s->msgbox[0x60 / 4] & s->msgbox[0x70 / 4]));
    for (unsigned i = 0; i < 6; i++) {
        pending |= s->bmu[0x40 + i] & s->bmu[0x48 + i];
        if (i < 3) {
            pending |= s->pmu[0x40 + i] & s->pmu[0x48 + i];
        }
    }
    qemu_set_irq(s->pmic_irq, pending);
    qemu_set_irq(s->wifi_power, !!(s->pmu[0x11] & BIT(0)));
    qemu_set_irq(s->lcd_power, !!(s->pmu[0x12] & BIT(0)));
}

static int aw_b300_pmic_index(AWB300PowerState *s, uint8_t address)
{
    for (unsigned i = 0; i < ARRAY_SIZE(s->runtime_address); i++) {
        if (s->runtime_address[i] && address == s->runtime_address[i]) {
            return i;
        }
    }
    return -1;
}

static uint8_t aw_b300_pmic_read(AWB300PowerState *s, unsigned chip,
                                 uint8_t reg)
{
    if (!chip) {
        return s->pmu[reg];
    }
    if (reg != 0xff && (s->bmu[0xff] & 1)) {
        return s->bmu_extended[reg];
    }
    if (reg == 0xe2 || reg == 0xe3) {
        /* The driver programs the model's calibrated full-charge capacity. */
        unsigned full = ((s->bmu[0xe0] & 0x7f) << 8) | s->bmu[0xe1];
        unsigned charge = full * (s->bmu[0xb9] & 0x7f) / 100;

        return reg == 0xe2 ? charge >> 8 : charge;
    }
    return s->bmu[reg];
}

static void aw_b300_pmic_write(AWB300PowerState *s, unsigned chip,
                                uint8_t reg, uint8_t value)
{
    uint8_t *regs = chip ? s->bmu : s->pmu;

    if (chip && reg != 0xff && (s->bmu[0xff] & 1)) {
        s->bmu_extended[reg] = value;
        return;
    }
    if (reg >= 0x48 && reg < (chip ? 0x4e : 0x4b)) {
        regs[reg] &= ~value;
    } else if (reg != 3 && (chip || reg > 1)) {
        regs[reg] = value;
    }
    if ((!chip && reg == 0x32 && (value & BIT(7))) ||
        (chip && reg == 0x17 && (value & BIT(0)) &&
         (s->bmu[0x10] & BIT(7)))) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
    aw_b300_power_update(s);
}

static bool aw_b300_rsb_assign(AWB300PowerState *s, uint16_t address,
                                uint8_t runtime)
{
    for (unsigned i = 0; i < ARRAY_SIZE(pmic_address); i++) {
        if (pmic_address[i] == address) {
            s->runtime_address[i] = runtime;
            return true;
        }
    }
    return false;
}

static void aw_b300_rsb_transfer(AWB300PowerState *s)
{
    uint32_t command = s->rsb[RSB_COMMAND] & 0xff;
    unsigned count;
    int chip;
    bool read;

    s->rsb[RSB_CTRL] &= ~RSB_START;
    if (command == 0xe8) {
        bool ok = aw_b300_rsb_assign(s, s->rsb[RSB_SLAVE] & 0xffff,
                                     s->rsb[RSB_SLAVE] >> 16);
        s->rsb[RSB_STATUS] |= ok ? RSB_DONE : RSB_ERROR | RSB_NACK;
        return;
    }
    switch (command) {
    case 0x8b: count = 1; read = true; break;
    case 0x9c: count = 2; read = true; break;
    case 0xa6: count = 4; read = true; break;
    case 0x4e: count = 1; read = false; break;
    case 0x59: count = 2; read = false; break;
    case 0x63: count = 4; read = false; break;
    default:
        s->rsb[RSB_STATUS] |= RSB_ERROR;
        return;
    }
    chip = aw_b300_pmic_index(s, s->rsb[RSB_SLAVE] >> 16);
    if (chip < 0) {
        s->rsb[RSB_STATUS] |= RSB_ERROR | RSB_NACK;
        return;
    }
    if (read) {
        s->rsb[RSB_DATA] = 0;
    }
    for (unsigned i = 0; i < count; i++) {
        uint8_t reg = s->rsb[RSB_ADDRESS] + i;

        if (read) {
            s->rsb[RSB_DATA] |= (uint32_t)aw_b300_pmic_read(s, chip, reg)
                                  << (8 * i);
        } else {
            aw_b300_pmic_write(s, chip, reg, s->rsb[RSB_DATA] >> (8 * i));
        }
    }
    s->rsb[RSB_STATUS] |= RSB_DONE;
}

static uint64_t aw_b300_rsb_read(void *opaque, hwaddr offset, unsigned size)
{
    AWB300PowerState *s = opaque;
    return s->rsb[offset / 4];
}

static void aw_b300_rsb_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    AWB300PowerState *s = opaque;
    unsigned reg = offset / 4;

    switch (reg) {
    case RSB_STATUS:
        s->rsb[reg] &= ~value;
        break;
    case RSB_CTRL:
        if (value & RSB_RESET) {
            /* Controller reset does not reset its powered PMIC slaves. */
            memset(s->rsb, 0, sizeof(s->rsb));
            s->rsb[RSB_LCR] = 0x30;
            s->rsb[0x300 / 4] = 0x10000;
        } else {
            s->rsb[reg] = value & ~RSB_ABORT;
            if (value & RSB_ABORT) {
                s->rsb[reg] &= ~RSB_START;
            } else if (value & RSB_START) {
                aw_b300_rsb_transfer(s);
            }
        }
        break;
    case RSB_MODE:
        s->rsb[reg] = value & ~BIT(31);
        if (value & BIT(31)) {
            /* The broadcast 0x3e/0x7c command selects RSB wire mode. */
            s->rsb[RSB_STATUS] |= RSB_DONE;
        }
        break;
    case RSB_LCR:
        s->rsb[reg] = (value & 0xf) | 0x30;
        break;
    default:
        s->rsb[reg] = value;
    }
    aw_b300_power_update(s);
}

static const MemoryRegionOps aw_b300_rsb_ops = {
    .read = aw_b300_rsb_read,
    .write = aw_b300_rsb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static bool aw_b300_msg_push(AWB300PowerState *s, unsigned channel,
                             uint32_t value)
{
    unsigned tail = (s->head[channel] + s->count[channel]) % 8;

    if (s->count[channel] == 8) {
        return false;
    }
    s->fifo[channel][tail] = value;
    s->count[channel]++;
    s->msgbox[0x50 / 4] |= BIT(channel * 2);
    s->msgbox[0x70 / 4] |= BIT(channel * 2);
    aw_b300_power_update(s);
    return true;
}

static void aw_b300_scp_startup(AWB300PowerState *s)
{
    uint8_t frame[SCP_MESSAGE_SIZE] = { 2, 2, 0x90, 0 };
    static const char version[] = "B300 QEMU SCP";

    if (!s->scp_started || s->scp_notified) {
        return;
    }
    stl_le_p(frame + 0x14, 0x6b); /* ARISC protocol revision. */
    memcpy(frame + 0x18, version, sizeof(version));
    if (dma_memory_write(&address_space_memory, SCP_POOL + SCP_STARTUP_FRAME,
                         frame, sizeof(frame), MEMTXATTRS_UNSPECIFIED)) {
        return;
    }
    s->scp_notified = aw_b300_msg_push(s, 0, SCP_STARTUP_FRAME);
}

void aw_b300_power_scp_start(AWB300PowerState *s)
{
    s->scp_started = true;
}

static int aw_b300_scp_rsb(AWB300PowerState *s, uint8_t type, uint8_t *frame)
{
    uint8_t *params = frame + 0x14;
    uint32_t format = ldl_le_p(params);
    unsigned count = format & 0xffff;
    unsigned width = format >> 16;
    int chip = aw_b300_pmic_index(s, ldl_le_p(params + 4));
    uint32_t addresses = ldl_le_p(params + 8);

    if (chip < 0 || !count || count > 4 ||
        (width != 1 && width != 2 && width != 4)) {
        return -EINVAL;
    }
    for (unsigned i = 0; i < count; i++) {
        uint8_t reg = addresses >> (8 * i);
        uint32_t value = ldl_le_p(params + 12 + i * 4);
        uint32_t old = 0;

        for (unsigned byte = 0; byte < width; byte++) {
            old |= (uint32_t)aw_b300_pmic_read(s, chip, reg + byte)
                    << (byte * 8);
        }
        switch (type) {
        case 0x80:
            stl_le_p(params + 12 + i * 4, old);
            continue;
        case 0x81:
            break;
        case 0x82:
            value = ldl_le_p(params + 32) ? old | value : old & ~value;
            break;
        }
        for (unsigned byte = 0; byte < width; byte++) {
            aw_b300_pmic_write(s, chip, reg + byte, value >> (byte * 8));
        }
    }
    return 0;
}

static void aw_b300_scp_request(AWB300PowerState *s, unsigned channel,
                                uint32_t offset)
{
    uint8_t frame[SCP_MESSAGE_SIZE];
    uint8_t *params = frame + 0x14;
    uint8_t type;
    int result = 0;

    if (!s->scp_started || offset > SCP_POOL_SIZE - SCP_MESSAGE_SIZE ||
        (offset & (SCP_MESSAGE_SIZE - 1)) ||
        dma_memory_read(&address_space_memory, SCP_POOL + offset, frame,
                        sizeof(frame), MEMTXATTRS_UNSPECIFIED)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid SCP message 0x%x\n",
                      TYPE_AW_B300_POWER, offset);
        return;
    }
    if (channel == 3) {
        /* CPUX has returned a message originally allocated by the SCP. */
        frame[0] = 0;
    } else {
        type = frame[2];
        trace_allwinner_b300_scp_request(type, channel, ldl_le_p(params),
                                         ldl_le_p(params + 4),
                                         ldl_le_p(params + 8));
        switch (type) {
        case 0x30: /* Native CPU voltage/frequency request. */
            result = aw_b300_scp_dvfs(params);
            break;
        case 0x80: /* RSB read. */
        case 0x81: /* RSB write. */
        case 0x82: /* RSB read/modify/write. */
            result = aw_b300_scp_rsb(s, type, frame);
            break;
        case 0x83: /* RSB interface mode. */
            break;
        case 0x84: /* RSB runtime address assignment. */
            result = aw_b300_rsb_assign(s, ldl_le_p(params),
                                        ldl_le_p(params + 4)) ? 0 : -ENXIO;
            break;
        case 0x50: /* PMIC IRQ notification acknowledgement. */
        case 0x51: /* PMIC IRQ forwarding disable. */
        case 0x52: /* PMIC IRQ forwarding enable. */
        case 0x54: /* PMIC board parameters. */
        case 0x59: /* Regulator dependency tree. */
        case 0x5a: /* Clear the SCP's NMI latch. */
        case 0x5b: /* NMI trigger selection. */
        case 0x60: /* SCP log level. */
        case 0x61: /* Message loopback. */
        case 0x62: /* Debug UART baud rate. */
        case 0x63: /* DRAM board parameters. */
        case 0x64: /* DRAM checksum settings. */
        case 0x65: /* IR wakeup settings. */
        case 0x31: /* CPU voltage/frequency table. */
        case 0x32: /* CPU voltage/frequency policy. */
            break;
        case 0x53: /* Silicon identifier. */
            /* Four SID words, matching the emulated SoC's fixed identifier. */
            stl_le_p(params, 0x03000001);
            stl_le_p(params + 4, 0x00003000);
            stl_le_p(params + 8, 0x20240001);
            stl_le_p(params + 12, 0x00000001);
            break;
        case 0x23: /* Query wakeup source. */
            stl_le_p(params, BIT(0));
            break;
        case 0x58: /* PMIC system reboot. */
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "%s: SCP request type 0x%02x\n",
                          TYPE_AW_B300_POWER, type);
            result = -ENOSYS;
            break;
        }
        frame[0] = 5; /* ARISC_MESSAGE_PROCESSED. */
        frame[3] = result;
    }
    if (dma_memory_write(&address_space_memory, SCP_POOL + offset, frame,
                         sizeof(frame), MEMTXATTRS_UNSPECIFIED)) {
        return;
    }
    if (channel == 4) {
        aw_b300_msg_push(s, 5, offset);
    } else if (channel == 1 && (frame[1] & 1)) {
        aw_b300_msg_push(s, 0, offset);
    } else if (channel == 1) {
        frame[0] = 0;
        dma_memory_write(&address_space_memory, SCP_POOL + offset, frame, 1,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

static uint64_t aw_b300_msgbox_read(void *opaque, hwaddr offset, unsigned size)
{
    AWB300PowerState *s = opaque;

    if (offset >= 0x100 && offset < 0x120) {
        return s->count[(offset - 0x100) / 4] == 8;
    }
    if (offset >= 0x140 && offset < 0x160) {
        aw_b300_scp_startup(s);
        return s->count[(offset - 0x140) / 4];
    }
    if (offset >= 0x180 && offset < 0x1a0) {
        unsigned channel = (offset - 0x180) / 4;
        uint32_t value = 0;

        if (s->count[channel]) {
            value = s->fifo[channel][s->head[channel]];
            s->head[channel] = (s->head[channel] + 1) % 8;
            s->count[channel]--;
        }
        return value;
    }
    return s->msgbox[offset / 4];
}

static void aw_b300_msgbox_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    AWB300PowerState *s = opaque;

    if (offset >= 0x180 && offset < 0x1a0) {
        unsigned channel = (offset - 0x180) / 4;

        if (channel == 1 || channel == 3 || channel == 4) {
            aw_b300_scp_request(s, channel, value);
        } else {
            aw_b300_msg_push(s, channel, value);
        }
    } else if (offset == 0x50 || offset == 0x70) {
        s->msgbox[offset / 4] &= ~value;
    } else if (offset < 0x100) {
        s->msgbox[offset / 4] = value;
    }
    aw_b300_power_update(s);
}

static const MemoryRegionOps aw_b300_msgbox_ops = {
    .read = aw_b300_msgbox_read,
    .write = aw_b300_msgbox_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t aw_b300_spinlock_read(void *opaque, hwaddr offset, unsigned size)
{
    AWB300PowerState *s = opaque;

    if (offset >= 0x100 && offset < 0x180) {
        uint32_t mask = BIT((offset - 0x100) / 4);
        bool taken = s->locks & mask;

        s->locks |= mask;
        return taken;
    }
    switch (offset) {
    case 0: return 1; /* 32 hardware locks. */
    case 0x10: return s->locks;
    case 0x20: return s->lock_irq_enable;
    case 0x40: return s->lock_irq_pending;
    default: return 0;
    }
}

static void aw_b300_spinlock_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size)
{
    AWB300PowerState *s = opaque;

    if (offset >= 0x100 && offset < 0x180 && !value) {
        s->locks &= ~BIT((offset - 0x100) / 4);
    } else if (offset == 0x20) {
        s->lock_irq_enable = value;
    } else if (offset == 0x40) {
        s->lock_irq_pending &= ~value;
    }
}

static const MemoryRegionOps aw_b300_spinlock_ops = {
    .read = aw_b300_spinlock_read,
    .write = aw_b300_spinlock_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void aw_b300_power_reset(Object *obj, ResetType type)
{
    AWB300PowerState *s = AW_B300_POWER(obj);

    memset(s->rsb, 0, sizeof(s->rsb));
    memset(s->msgbox, 0, sizeof(s->msgbox));
    memset(s->fifo, 0, sizeof(s->fifo));
    memset(s->head, 0, sizeof(s->head));
    memset(s->count, 0, sizeof(s->count));
    memset(s->pmu, 0, sizeof(s->pmu));
    memset(s->bmu, 0, sizeof(s->bmu));
    memset(s->bmu_extended, 0, sizeof(s->bmu_extended));
    s->runtime_address[0] = 0x2d;
    s->runtime_address[1] = 0x3a;
    s->rsb[RSB_LCR] = 0x30;
    s->rsb[0x300 / 4] = 0x10000;
    s->locks = 0;
    s->lock_irq_enable = 0;
    s->lock_irq_pending = 0;
    s->scp_started = false;
    s->scp_notified = false;
    s->pmu[0] = BIT(2); /* Power-key startup. */
    s->pmu[3] = 0x44;   /* AXP858 / PMU1736. */
    s->pmu[0x10] = 0x3f;
    s->pmu[0x11] = 0xff;
    s->pmu[0x12] = 0xff;
    s->pmu[0x13] = 18;  /* DCDC1 3.3 V. */
    s->pmu[0x14] = 60;  /* DCDC2 1.1 V. */
    s->pmu[0x15] = 60;
    s->pmu[0x16] = 60;
    s->pmu[0x17] = 70;
    s->pmu[0x19] = 26;  /* ALDO1 Wi-Fi 3.3 V. */
    s->pmu[0x23] = 11;  /* ALDO5 Wi-Fi I/O 1.8 V. */
    s->bmu[2] = BIT(4) | BIT(3); /* Battery present and detected. */
    s->bmu[3] = 0x56;   /* AXP2585 / BMU1760. */
    s->bmu[0x22] = BIT(1);
    s->bmu[0x24] = 0xff;
    s->bmu[0x56] = 0x64;
    s->bmu[0x58] = 0x2e; /* Thermistor ADC: 25 degrees C. */
    s->bmu[0x59] = 0x0e;
    s->bmu[0x78] = 0xd0; /* Battery ADC: 4.0 V, 1.2 mV per unit. */
    s->bmu[0x79] = 5;
    s->bmu[0xb8] = 0xc0;
    s->bmu[0xb9] = 0x80 | 85;
    s->bmu[0xbc] = 0xd0;
    s->bmu[0xbd] = 5;
    s->bmu[0xe4] = 0x80 | 85;
    s->bmu[0xe5] = 0x80 | 85;
}

static void aw_b300_power_reset_hold(Object *obj, ResetType type)
{
    aw_b300_power_update(AW_B300_POWER(obj));
}

static int aw_b300_power_post_load(void *opaque, int version)
{
    AWB300PowerState *s = opaque;

    for (unsigned i = 0; i < ARRAY_SIZE(s->count); i++) {
        if (s->count[i] > 8 || s->head[i] >= 8) {
            return -EINVAL;
        }
    }
    aw_b300_power_update(s);
    return 0;
}

static const VMStateDescription aw_b300_power_vmstate = {
    .name = TYPE_AW_B300_POWER,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = aw_b300_power_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(rsb, AWB300PowerState, 0x304 / 4),
        VMSTATE_UINT32_ARRAY(msgbox, AWB300PowerState, 0x1000 / 4),
        VMSTATE_UINT32_2DARRAY(fifo, AWB300PowerState, 8, 8),
        VMSTATE_UINT8_ARRAY(head, AWB300PowerState, 8),
        VMSTATE_UINT8_ARRAY(count, AWB300PowerState, 8),
        VMSTATE_UINT32(locks, AWB300PowerState),
        VMSTATE_UINT32(lock_irq_enable, AWB300PowerState),
        VMSTATE_UINT32(lock_irq_pending, AWB300PowerState),
        VMSTATE_UINT8_ARRAY(pmu, AWB300PowerState, 256),
        VMSTATE_UINT8_ARRAY(bmu, AWB300PowerState, 256),
        VMSTATE_UINT8_ARRAY(bmu_extended, AWB300PowerState, 256),
        VMSTATE_UINT8_ARRAY(runtime_address, AWB300PowerState, 2),
        VMSTATE_BOOL(scp_started, AWB300PowerState),
        VMSTATE_BOOL(scp_notified, AWB300PowerState),
        VMSTATE_END_OF_LIST()
    },
};

static void aw_b300_power_init(Object *obj)
{
    AWB300PowerState *s = AW_B300_POWER(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->rsb_mmio, obj, &aw_b300_rsb_ops, s,
                          "b300-rsb", 0x304);
    memory_region_init_io(&s->msgbox_mmio, obj, &aw_b300_msgbox_ops, s,
                          "b300-msgbox", 0x1000);
    memory_region_init_io(&s->spinlock_mmio, obj, &aw_b300_spinlock_ops, s,
                          "b300-hwspinlock", 0x1000);
    sysbus_init_mmio(dev, &s->rsb_mmio);
    sysbus_init_mmio(dev, &s->msgbox_mmio);
    sysbus_init_mmio(dev, &s->spinlock_mmio);
    sysbus_init_irq(dev, &s->rsb_irq);
    sysbus_init_irq(dev, &s->msgbox_irq);
    sysbus_init_irq(dev, &s->pmic_irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->wifi_power, "wifi-power", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->lcd_power, "lcd-power", 1);
}

static void aw_b300_power_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Allwinner B300 RSB and SCP power subsystem";
    dc->vmsd = &aw_b300_power_vmstate;
    rc->phases.enter = aw_b300_power_reset;
    rc->phases.hold = aw_b300_power_reset_hold;
}

static const TypeInfo aw_b300_power_info = {
    .name = TYPE_AW_B300_POWER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AWB300PowerState),
    .instance_init = aw_b300_power_init,
    .class_init = aw_b300_power_class_init,
};

static void aw_b300_power_register_types(void)
{
    type_register_static(&aw_b300_power_info);
}
type_init(aw_b300_power_register_types)
