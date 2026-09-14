/*
 * MT8171 GCE v5.1. Register and instruction definitions follow Lab126's
 * mtk-cmdq-mailbox-ext.c, mtk-cmdq-helper-ext.c and mt8171-gce.h.
 * Packets use physical DMA addresses, encoded in units of eight bytes.
 * Unsupported instructions fault; unmet POLL/WFE conditions retain the PC.
 * No event is manufactured from packet submission or elapsed frame time.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/misc/mt8171-gce.h"
#include "hw/core/irq.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TR(s, t, r) ((s)->regs[(0x100 + (t) * 0x80 + (r)) / 4])
#define ENABLE 4
#define SUSPEND 8
#define STATUS 12
#define IRQ 16
#define IRQ_EN 20
#define PC 32
#define END 36
#define COUNT 40
#define WAIT 48
#define SLICE_NS 10000

static void update_irq(MT8171GCEState *s)
{
    bool active = false;
    for (unsigned t = 0; t < 32; t++) {
        active |= !!(TR(s, t, IRQ) & TR(s, t, IRQ_EN));
    }
    qemu_set_irq(s->irq, active);
}

static void reschedule(MT8171GCEState *s)
{
    int64_t next = s->pending ?
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SLICE_NS : INT64_MAX;
    for (unsigned i = 0; i < 16; i++) {
        if (s->compare_armed & BIT(i)) {
            next = MIN(next, s->compare_deadline[i]);
        }
    }
    if (next == INT64_MAX) { timer_del(s->timer); }
    else { timer_mod(s->timer, next); }
}

static void schedule(MT8171GCEState *s, unsigned t)
{
    s->pending |= BIT(t);
    reschedule(s);
}

static void arm_compare(MT8171GCEState *s, unsigned i)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t ticks = now * 26 / 1000;
    uint32_t delta = s->gpr[32 + i] - ticks;
    if (s->regs[0xdc / 4] & BIT(i)) {
        s->compare_armed |= BIT(i);
        s->compare_deadline[i] = now + ((uint64_t)delta * 1000 + 25) / 26;
    } else {
        s->compare_armed &= ~BIT(i);
    }
    reschedule(s);
}

static void token_update(MT8171GCEState *s, unsigned token, bool value)
{
    s->tokens[token] = value;
    for (unsigned t = 0; t < 32; t++) {
        if ((s->waiting & BIT(t)) && s->token[t] == token) {
            s->waiting &= ~BIT(t);
            schedule(s, t);
        }
    }
}

static uint32_t reg_read(MT8171GCEState *s, unsigned t, uint16_t r)
{
    if (r < 4) {
        return s->spr[t][r];
    }
    if (r == 56) {
        /* 26 MHz free-running timestamp used by cmdq_pkt_poll_timeout. */
        return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) * 26 / 1000;
    }
    return s->gpr[r];
}

static void reg_write(MT8171GCEState *s, unsigned t, uint16_t r, uint32_t v)
{
    if (r < 4) {
        s->spr[t][r] = v;
    } else {
        s->gpr[r] = v;
        if (r >= 32 && r < 48) { arm_compare(s, r - 32); }
    }
}

static hwaddr subsys(unsigned id)
{
    static const uint32_t base[] = {
        0x14000000, 0x14010000, 0x14020000, 0x1f000000, 0x1f010000,
        0x16000000, 0x16010000, 0x16020000, 0x1c000000, 0x1c010000,
        0x10220000, 0x1b000000, 0x15010000, 0x15020000, 0x15810000,
        0x15820000, 0x17000000, 0x17010000, 0x17020000, 0x17030000,
        0x17040000, 0x17060000, 0x1a000000, 0x1a010000, 0x1a030000,
        0x1a040000, 0x1a050000, 0x1a060000, 0x1a090000, 0x1a0a0000,
    };
    return id < ARRAY_SIZE(base) ? base[id] : 0;
}

static hwaddr address(MT8171GCEState *s, unsigned t, uint16_t low,
                      unsigned sub, bool indirect)
{
    if (indirect) {
        return reg_read(s, t, low);
    }
    if (low & 2) {
        return ((hwaddr)reg_read(s, t, sub) << 16) | (low & ~2);
    }
    return subsys(sub) | low;
}

static bool condition(unsigned op, uint32_t a, uint32_t b)
{
    switch (op) {
    case 0: return a == b;
    case 1: return a != b;
    case 2: return a >= b;
    case 3: return a <= b;
    case 4: return a > b;
    case 5: return a < b;
    default: return false;
    }
}

static bool logic(unsigned op, uint32_t a, uint32_t b, uint32_t *v)
{
    switch (op) {
    case 1: *v = a + b; break;
    case 2: *v = a - b; break;
    case 3: *v = a * b; break;
    case 8: *v = a ^ b; break;
    case 9: *v = ~a; break;
    case 10: *v = a | b; break;
    case 11: *v = a & b; break;
    case 12: *v = a << (b & 31); break;
    case 13: *v = a >> (b & 31); break;
    default: return false;
    }
    return true;
}

static void execute(MT8171GCEState *s, unsigned t)
{
    hwaddr pc = (hwaddr)TR(s, t, PC) << 3;
    hwaddr end = (hwaddr)TR(s, t, END) << 3;
    MemTxResult result;
    uint64_t raw = 0;
    unsigned step;

    s->executing = t;
    for (step = 0; step < 2048; step++) {
        uint16_t a, b, c;
        unsigned op, sub, flags;
        uint32_t left, right, value;
        hwaddr addr, next = pc + 8;
        bool ar, br, cr;

        if (pc == end) {
            /* END points at the final jump in non-looping vendor packets. */
            break;
        }
        if (dma_memory_read(&address_space_memory, pc, &raw, 8,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            goto fault;
        }
        raw = le64_to_cpu(raw);
        c = raw; b = raw >> 16; a = raw >> 32;
        flags = (raw >> 48) & 255; op = raw >> 56; sub = flags & 31;
        ar = flags & 128; br = flags & 64; cr = flags & 32;
        value = br ? reg_read(s, t, b) : (uint32_t)b << 16 | c;
        left = br ? reg_read(s, t, b) : b;
        right = cr ? reg_read(s, t, c) : c;
        switch (op) {
        case 0x02: /* MOVE / inverted write mask */
            if (ar) {
                s->gpr[sub] = (uint32_t)b << 16 | c;
            } else {
                s->mask[t] = (uint32_t)b << 16 | c;
            }
            break;
        case 0x04: /* legacy WRITE */
        case 0x90:
        case 0x91:
            addr = address(s, t, a, sub, ar);
            if (op == 0x91 || (op == 0x04 && (a & 1))) {
                uint32_t old;
                addr &= ~1ULL;
                old = address_space_ldl_le(&address_space_memory, addr,
                                          MEMTXATTRS_UNSPECIFIED, &result);
                if (result != MEMTX_OK) { goto fault; }
                value = (old & s->mask[t]) | (value & ~s->mask[t]);
            }
            address_space_stl_le(&address_space_memory, addr, value,
                                 MEMTXATTRS_UNSPECIFIED, &result);
            if (result != MEMTX_OK) { goto fault; }
            break;
        case 0x80: /* READ_S */
            addr = address(s, t, b, sub, br);
            value = address_space_ldl_le(&address_space_memory, addr,
                                        MEMTXATTRS_UNSPECIFIED, &result);
            if (result != MEMTX_OK) { goto fault; }
            reg_write(s, t, a, value);
            break;
        case 0x08: /* POLL */
            addr = ar ? s->gpr[sub] : subsys(sub) | (a & ~1);
            left = address_space_ldl_le(&address_space_memory, addr,
                                       MEMTXATTRS_UNSPECIFIED, &result);
            if (result != MEMTX_OK) { goto fault; }
            right = a & 1 ? ~s->mask[t] : UINT32_MAX;
            if ((left & right) != (value & right)) {
                schedule(s, t);
                goto stopped;
            }
            break;
        case 0xa0:
            if (sub && !logic(sub, left, right, &value)) { goto fault; }
            reg_write(s, t, a, value);
            break;
        case 0xb0:
        case 0xb1:
            if (sub > 5) { goto fault; }
            if (condition(sub, left, right)) {
                uint32_t target = ar ? reg_read(s, t, a) : a;
                next = op == 0xb0 ? (hwaddr)target << 3 :
                       pc + (int64_t)(int32_t)target * 8;
            }
            break;
        case 0x10:
            next = a ? (hwaddr)value << 3 : pc + (int64_t)(int32_t)value * 8;
            break;
        case 0x20: {
            unsigned token = a & 1023;
            if ((value & BIT(15)) &&
                s->tokens[token] != !!(value & 1)) {
                s->waiting |= BIT(t);
                s->token[t] = token;
                TR(s, t, WAIT) = BIT(31) | token;
                goto stopped;
            }
            TR(s, t, WAIT) = 0;
            if (value & BIT(31)) {
                token_update(s, token, value & BIT(16));
            }
            break;
        }
        case 0x40:
            TR(s, t, COUNT)++;
            if (c & 1) {
                TR(s, t, IRQ) |= 1;
                update_irq(s);
            }
            break;
        default:
            goto fault;
        }
        pc = next;
    }
    if (step == 2048) {
        schedule(s, t);
    }
stopped:
    TR(s, t, PC) = pc >> 3;
    return;
fault:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "mt8171-gce: thread %u fault at 0x%" HWADDR_PRIx
                  " instruction 0x%016" PRIx64 "\n", t, pc, raw);
    TR(s, t, IRQ) |= 0x10;
    TR(s, t, STATUS) |= BIT(3);
    update_irq(s);
    goto stopped;
}

static void run(void *opaque)
{
    MT8171GCEState *s = opaque;
    uint32_t pending;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (unsigned i = 0; i < 16; i++) {
        if ((s->compare_armed & BIT(i)) && now >= s->compare_deadline[i]) {
            s->compare_armed &= ~BIT(i);
            token_update(s, 994 + i, true);
        }
    }
    pending = s->pending;
    s->pending = 0;
    while (pending) {
        unsigned t = ctz32(pending);
        pending &= ~BIT(t);
        if ((TR(s, t, ENABLE) & 1) && !(TR(s, t, SUSPEND) & 1) &&
            !(s->waiting & BIT(t))) {
            execute(s, t);
        }
    }
    reschedule(s);
}

static void event(void *opaque, int line, int level)
{
    if (level) {
        token_update(opaque, line, true);
    }
}

static uint64_t read_reg(void *opaque, hwaddr offset, unsigned size)
{
    MT8171GCEState *s = opaque;
    if (offset == 0x10) {
        uint32_t status = UINT32_MAX;
        for (unsigned t = 0; t < 32; t++) {
            if (TR(s, t, IRQ) & TR(s, t, IRQ_EN)) { status &= ~BIT(t); }
        }
        return status;
    }
    if (offset == 0x18) { return s->executing; }
    if (offset == 0x64) { return s->tokens[s->selected]; }
    if (offset >= 0x80 && offset < 0xc0) {
        return reg_read(s, 0, 32 + (offset - 0x80) / 4);
    }
    if (offset >= 0x100 && offset < 0x1100 && (offset & 0x7f) >= 0x60 &&
        (offset & 0x7f) < 0x70) {
        return s->spr[(offset - 0x100) / 0x80][(offset & 15) / 4];
    }
    return s->regs[offset / 4];
}

static void write_reg(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    MT8171GCEState *s = opaque;
    if (offset == 0xdc) {
        uint32_t changed = s->regs[offset / 4] ^ value;
        s->regs[offset / 4] = value;
        for (unsigned i = 0; i < 16; i++) {
            if (changed & BIT(i)) { arm_compare(s, i); }
        }
        return;
    }
    if (offset == 0x60) { s->selected = value & 1023; }
    if (offset == 0x68) { token_update(s, value & 1023, value & BIT(16)); }
    if (offset >= 0x80 && offset < 0xc0) {
        reg_write(s, 0, 32 + (offset - 0x80) / 4, value);
    }
    if (offset >= 0x100 && offset < 0x1100) {
        unsigned t = (offset - 0x100) / 0x80;
        unsigned r = offset & 0x7f;
        if (r >= 0x60 && r < 0x70) { s->spr[t][(r - 0x60) / 4] = value; }
        switch (r) {
        case 0:
            if (value & (BIT(0) | BIT(16))) {
                memset(&TR(s, t, 0), 0, 0x80);
                memset(s->spr[t], 0, sizeof(s->spr[t]));
                s->mask[t] = 0;
                s->waiting &= ~BIT(t); s->pending &= ~BIT(t);
                update_irq(s);
            }
            return;
        case IRQ:
            TR(s, t, IRQ) &= value;
            update_irq(s);
            return;
        case SUSPEND:
            TR(s, t, STATUS) = (TR(s, t, STATUS) & ~2) | (value & 1) * 2;
            break;
        case PC:
            s->waiting &= ~BIT(t);
            break;
        }
        s->regs[offset / 4] = value;
        if ((r == ENABLE || r == SUSPEND || r == END || r == PC) &&
            (TR(s, t, ENABLE) & 1) && !(TR(s, t, SUSPEND) & 1)) {
            schedule(s, t);
        }
        if (r == IRQ_EN) { update_irq(s); }
        return;
    }
    s->regs[offset / 4] = value;
}

static const MemoryRegionOps ops = {
    .read = read_reg, .write = write_reg,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static void reset(DeviceState *dev)
{
    MT8171GCEState *s = MT8171_GCE(dev);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->gpr, 0, sizeof(s->gpr));
    memset(s->spr, 0, sizeof(s->spr));
    memset(s->mask, 0, sizeof(s->mask));
    memset(s->tokens, 0, sizeof(s->tokens));
    s->pending = s->waiting = s->selected = s->executing = 0;
    s->compare_armed = 0;
    timer_del(s->timer);
    update_irq(s);
}
static void init(Object *obj)
{
    MT8171GCEState *s = MT8171_GCE(obj);
    memory_region_init_io(&s->iomem, obj, &ops, s, TYPE_MT8171_GCE, 0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), event, "event", 1024);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, run, s);
}
static void finalize(Object *obj)
{
    timer_free(MT8171_GCE(obj)->timer);
}
static void class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), reset);
}
static const TypeInfo info = {
    .name = TYPE_MT8171_GCE, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8171GCEState), .instance_init = init,
    .instance_finalize = finalize, .class_init = class_init,
};
static void register_types(void) { type_register_static(&info); }
type_init(register_types)
