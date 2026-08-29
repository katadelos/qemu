/*
 * MediaTek MT8113 Global Command Engine
 *
 * The Bellatrix4 HWTCON driver uses GCE command packets for display register
 * programming.  This model executes the packet subset used by that driver and
 * implements the mailbox thread completion protocol.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/mt8113_gce.h"
#include "hw/core/irq.h"
#include "qemu/bitops.h"
#include "qemu/module.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "trace.h"

#define GCE_CURR_IRQ_STATUS       0x010
#define GCE_SYNC_TOKEN_ID         0x060
#define GCE_SYNC_TOKEN_VAL        0x064
#define GCE_SYNC_TOKEN_UPD        0x068

#define GCE_THR_BASE              0x100
#define GCE_THR_SIZE              0x080
#define GCE_THR_WARM_RESET        0x000
#define GCE_THR_ENABLE            0x004
#define GCE_THR_SUSPEND           0x008
#define GCE_THR_STATUS            0x00c
#define GCE_THR_IRQ_STATUS        0x010
#define GCE_THR_IRQ_ENABLE        0x014
#define GCE_THR_CURR_ADDR         0x020
#define GCE_THR_END_ADDR          0x024

#define GCE_THR_IRQ_DONE          BIT(0)
#define GCE_THR_STATUS_SUSPENDED  BIT(1)
#define GCE_MAX_PACKET_STEPS      4096
#define GCE_EXEC_DELAY_NS         (1 * SCALE_MS)

#define MT8113_IOMMU_BASE         0x10209000
#define MT8113_IOMMU_TTBR         0x000

#define GCE_CODE_MASK             0x02
#define GCE_CODE_WRITE            0x04
#define GCE_CODE_POLL             0x08
#define GCE_CODE_JUMP             0x10
#define GCE_CODE_WFE              0x20
#define GCE_CODE_EOC              0x40
#define GCE_CODE_READ_S           0x80
#define GCE_CODE_WRITE_S          0x90
#define GCE_CODE_WRITE_S_MASK     0x91
#define GCE_CODE_LOGIC            0xa0
#define GCE_CODE_JUMP_C_ABSOLUTE  0xb0
#define GCE_CODE_JUMP_C_RELATIVE  0xb1

#define GCE_LOGIC_ASSIGN          0
#define GCE_LOGIC_ADD             1
#define GCE_LOGIC_SUBTRACT        2
#define GCE_LOGIC_MULTIPLY        3
#define GCE_LOGIC_XOR             8
#define GCE_LOGIC_NOT             9
#define GCE_LOGIC_OR              10
#define GCE_LOGIC_AND             11
#define GCE_LOGIC_LEFT_SHIFT      12
#define GCE_LOGIC_RIGHT_SHIFT     13

#define GCE_COND_EQUAL            0
#define GCE_COND_NOT_EQUAL        1
#define GCE_COND_GREATER_EQUAL    2
#define GCE_COND_LESS_EQUAL       3
#define GCE_COND_GREATER          4
#define GCE_COND_LESS             5

#define GCE_WFE_UPDATE            BIT(31)
#define GCE_WFE_UPDATE_VALUE      BIT(16)
#define GCE_WFE_WAIT              BIT(15)
#define GCE_WFE_WAIT_VALUE        BIT(0)

#define GCE_DPI0_FRAME_DONE_EVENT 9
/* MT8512/MT8113 GCE stores byte addresses; newer GCE revisions use >> 3. */
#define GCE_ADDR_SHIFT            0

static uint32_t *mt8113_gce_thread_reg(MT8113GCEState *s, unsigned thread,
                                       hwaddr reg)
{
    hwaddr offset = GCE_THR_BASE + thread * GCE_THR_SIZE + reg;

    return &s->regs[offset / sizeof(uint32_t)];
}

static uint32_t mt8113_gce_irq_status(MT8113GCEState *s)
{
    uint32_t status = UINT32_MAX;

    for (unsigned i = 0; i < MT8113_GCE_THREADS; i++) {
        if (*mt8113_gce_thread_reg(s, i, GCE_THR_IRQ_STATUS)) {
            status &= ~BIT(i);
        }
    }
    return status;
}

static void mt8113_gce_update_irq(MT8113GCEState *s)
{
    qemu_set_irq(s->irq, mt8113_gce_irq_status(s) != UINT32_MAX);
}

static hwaddr mt8113_gce_subsys_base(unsigned id)
{
    switch (id) {
    case 1:
        return 0x14000000;
    case 22:
        return 0x15000000;
    default:
        return 0;
    }
}

static hwaddr mt8113_gce_translate_iova(hwaddr iova);

static uint32_t mt8113_gce_load32(hwaddr addr)
{
    MemTxResult result;

    addr = mt8113_gce_translate_iova(addr);
    return address_space_ldl_le(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, &result);
}

static hwaddr mt8113_gce_translate_iova(hwaddr iova)
{
    uint32_t ttbr, l1, l2;
    hwaddr table;

    if (iova >= 0x10000000) {
        return iova;
    }

    ttbr = mt8113_gce_load32(MT8113_IOMMU_BASE + MT8113_IOMMU_TTBR);
    table = ttbr & 0xffffc000;
    if (!table) {
        return iova;
    }

    l1 = mt8113_gce_load32(table + ((iova >> 20) & 0xfff) * 4);
    if ((l1 & 3) == 1) {
        table = l1 & 0xfffffc00;
        l2 = mt8113_gce_load32(table + ((iova >> 12) & 0xff) * 4);
        if (l2 & 3) {
            return (l2 & 0xfffff000) | (iova & 0xfff);
        }
    } else if ((l1 & 3) == 2) {
        return (l1 & 0xfff00000) | (iova & 0xfffff);
    }

    return iova;
}

static void mt8113_gce_store32(hwaddr addr, uint32_t value)
{
    addr = mt8113_gce_translate_iova(addr);
    address_space_stl_le(&address_space_memory, addr, value,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

static hwaddr mt8113_gce_write_addr(MT8113GCEState *s, uint16_t arg_a,
                                    uint8_t s_op, bool arg_a_reg)
{
    hwaddr base;

    if (arg_a_reg) {
        return s->gpr[arg_a];
    }

    /* Bit 1 tags the low half of an address held in a GCE register. */
    if (arg_a & BIT(1)) {
        return ((hwaddr)s->gpr[s_op] << 16) | (arg_a & ~BIT(1));
    }

    base = mt8113_gce_subsys_base(s_op);
    return base ? base + arg_a : arg_a;
}

static hwaddr mt8113_gce_read_addr(MT8113GCEState *s, uint16_t arg_b,
                                   uint8_t s_op, bool arg_b_reg)
{
    hwaddr base;

    if (arg_b_reg) {
        return s->gpr[arg_b];
    }
    if (arg_b & BIT(1)) {
        return ((hwaddr)s->gpr[s_op] << 16) | (arg_b & ~BIT(1));
    }

    base = mt8113_gce_subsys_base(s_op);
    return base ? base + arg_b : arg_b;
}

static uint32_t mt8113_gce_operand(MT8113GCEState *s, uint16_t high,
                                   uint16_t low, bool high_reg)
{
    return high_reg ? s->gpr[high] : ((uint32_t)high << 16) | low;
}

static uint32_t mt8113_gce_operand16(MT8113GCEState *s, uint16_t value,
                                     bool is_reg)
{
    return is_reg ? s->gpr[value] : value;
}

static uint32_t mt8113_gce_logic(unsigned op, uint32_t left,
                                 uint32_t right)
{
    switch (op) {
    case GCE_LOGIC_ADD:
        return left + right;
    case GCE_LOGIC_SUBTRACT:
        return left - right;
    case GCE_LOGIC_MULTIPLY:
        return left * right;
    case GCE_LOGIC_XOR:
        return left ^ right;
    case GCE_LOGIC_NOT:
        return ~left;
    case GCE_LOGIC_OR:
        return left | right;
    case GCE_LOGIC_AND:
        return left & right;
    case GCE_LOGIC_LEFT_SHIFT:
        return left << (right & 31);
    case GCE_LOGIC_RIGHT_SHIFT:
        return left >> (right & 31);
    default:
        return left;
    }
}

static bool mt8113_gce_condition(unsigned op, uint32_t left,
                                 uint32_t right)
{
    switch (op) {
    case GCE_COND_EQUAL:
        return left == right;
    case GCE_COND_NOT_EQUAL:
        return left != right;
    case GCE_COND_GREATER_EQUAL:
        return left >= right;
    case GCE_COND_LESS_EQUAL:
        return left <= right;
    case GCE_COND_GREATER:
        return left > right;
    case GCE_COND_LESS:
        return left < right;
    default:
        return false;
    }
}

static void mt8113_gce_schedule_thread(MT8113GCEState *s, unsigned thread)
{
    s->pending_threads |= BIT(thread);
    timer_mod(s->exec_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GCE_EXEC_DELAY_NS);
}

static void mt8113_gce_execute(MT8113GCEState *s, unsigned thread)
{
    uint32_t *pc_reg = mt8113_gce_thread_reg(s, thread,
                                              GCE_THR_CURR_ADDR);
    uint32_t end = *mt8113_gce_thread_reg(s, thread, GCE_THR_END_ADDR);
    uint32_t mask = 0;
    hwaddr pc = *pc_reg;
    unsigned steps = 0;
    unsigned writes = 0;

    trace_mt8113_gce_execute(thread, pc, end);

    for (steps = 0; steps < GCE_MAX_PACKET_STEPS; steps++) {
        uint64_t raw;
        uint16_t arg_c, arg_b, arg_a;
        uint8_t flags, s_op, op;
        bool arg_c_reg, arg_b_reg, arg_a_reg;
        hwaddr next = pc + 8;

        hwaddr packet_addr = mt8113_gce_translate_iova(pc);

        if (dma_memory_read(&address_space_memory, packet_addr, &raw,
                            sizeof(raw),
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            trace_mt8113_gce_dma_error(thread, pc, packet_addr);
            break;
        }
        raw = le64_to_cpu(raw);
        arg_c = extract64(raw, 0, 16);
        arg_b = extract64(raw, 16, 16);
        arg_a = extract64(raw, 32, 16);
        flags = extract64(raw, 48, 8);
        s_op = flags & 0x1f;
        arg_c_reg = flags & BIT(5);
        arg_b_reg = flags & BIT(6);
        arg_a_reg = flags & BIT(7);
        op = extract64(raw, 56, 8);

        switch (op) {
        case GCE_CODE_MASK:
            if (s_op) {
                s->gpr[s_op] = ((uint32_t)arg_b << 16) | arg_c;
            } else {
                mask = ((uint32_t)arg_b << 16) | arg_c;
            }
            break;
        case GCE_CODE_WRITE:
        case GCE_CODE_WRITE_S:
        case GCE_CODE_WRITE_S_MASK: {
            hwaddr addr = mt8113_gce_write_addr(s, arg_a, s_op,
                                                 arg_a_reg);
            uint32_t value = mt8113_gce_operand(s, arg_b, arg_c,
                                                 arg_b_reg);

            if (op == GCE_CODE_WRITE_S_MASK) {
                uint32_t old = mt8113_gce_load32(addr);

                value = (old & mask) | (value & ~mask);
            }
            mt8113_gce_store32(addr, value);
            writes++;
            break;
        }
        case GCE_CODE_READ_S: {
            hwaddr addr = mt8113_gce_read_addr(s, arg_b, s_op,
                                                arg_b_reg);

            s->gpr[arg_a] = mt8113_gce_load32(addr);
            break;
        }
        case GCE_CODE_LOGIC:
            if (s_op == GCE_LOGIC_ASSIGN) {
                s->gpr[arg_a] = mt8113_gce_operand(s, arg_b, arg_c,
                                                    arg_b_reg);
            } else {
                uint32_t left = mt8113_gce_operand16(s, arg_b,
                                                     arg_b_reg);
                uint32_t right = mt8113_gce_operand16(s, arg_c,
                                                      arg_c_reg);

                s->gpr[arg_a] = mt8113_gce_logic(s_op, left, right);
            }
            break;
        case GCE_CODE_JUMP_C_ABSOLUTE:
        case GCE_CODE_JUMP_C_RELATIVE: {
            uint32_t left = mt8113_gce_operand16(s, arg_b, arg_b_reg);
            uint32_t right = mt8113_gce_operand16(s, arg_c, arg_c_reg);

            if (mt8113_gce_condition(s_op, left, right)) {
                uint32_t target = arg_a_reg ? s->gpr[arg_a] : arg_a;

                if (op == GCE_CODE_JUMP_C_ABSOLUTE) {
                    next = (hwaddr)target << GCE_ADDR_SHIFT;
                } else {
                    next = pc + ((int32_t)target << GCE_ADDR_SHIFT);
                }
            }
            break;
        }
        case GCE_CODE_JUMP:
        {
            uint32_t target = ((uint32_t)arg_b << 16) | arg_c;

            next = arg_a ? (hwaddr)target << GCE_ADDR_SHIFT :
                           pc + ((int32_t)target << GCE_ADDR_SHIFT);
            break;
        }
        case GCE_CODE_WFE: {
            uint32_t option = ((uint32_t)arg_b << 16) | arg_c;
            unsigned token = arg_a & 0x3ff;

            if (token == GCE_DPI0_FRAME_DONE_EVENT &&
                (option & GCE_WFE_WAIT) &&
                s->tokens[token] != !!(option & GCE_WFE_WAIT_VALUE)) {
                s->waiting[thread] = true;
                s->wait_token[thread] = token;
                *pc_reg = pc;
                trace_mt8113_gce_complete(thread, pc, end, steps, writes);
                return;
            }
            if (option & GCE_WFE_UPDATE) {
                s->tokens[token] = !!(option & GCE_WFE_UPDATE_VALUE);
            }
            break;
        }
        case GCE_CODE_EOC:
            if (arg_c & BIT(0)) {
                *mt8113_gce_thread_reg(s, thread, GCE_THR_IRQ_STATUS) |=
                    GCE_THR_IRQ_DONE;
                mt8113_gce_update_irq(s);
            }
            break;
        case GCE_CODE_POLL:
        default:
            break;
        }

        if (pc == end && !(op == GCE_CODE_JUMP && arg_a)) {
            break;
        }
        pc = next;
    }

    trace_mt8113_gce_complete(thread, pc, end, steps, writes);
    *pc_reg = end;
    if (*mt8113_gce_thread_reg(s, thread, GCE_THR_IRQ_ENABLE) &
        GCE_THR_IRQ_DONE) {
        *mt8113_gce_thread_reg(s, thread, GCE_THR_IRQ_STATUS) |=
            GCE_THR_IRQ_DONE;
        mt8113_gce_update_irq(s);
    }
}

static void mt8113_gce_run_pending(void *opaque)
{
    MT8113GCEState *s = opaque;
    uint32_t pending = s->pending_threads;

    s->pending_threads = 0;
    while (pending) {
        unsigned thread = ctz32(pending);

        pending &= ~BIT(thread);
        if ((*mt8113_gce_thread_reg(s, thread, GCE_THR_ENABLE) & 1) &&
            !(*mt8113_gce_thread_reg(s, thread, GCE_THR_SUSPEND) & 1) &&
            !s->waiting[thread]) {
            mt8113_gce_execute(s, thread);
        }
    }
}

static void mt8113_gce_event(void *opaque, int line, int level)
{
    MT8113GCEState *s = opaque;
    unsigned token = GCE_DPI0_FRAME_DONE_EVENT;

    if (!level) {
        return;
    }

    s->tokens[token] = true;
    for (unsigned thread = 0; thread < MT8113_GCE_THREADS; thread++) {
        if (s->waiting[thread] && s->wait_token[thread] == token) {
            s->waiting[thread] = false;
            mt8113_gce_schedule_thread(s, thread);
        }
    }
}

static uint64_t mt8113_gce_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8113GCEState *s = opaque;

    switch (offset) {
    case GCE_CURR_IRQ_STATUS:
        return mt8113_gce_irq_status(s);
    case GCE_SYNC_TOKEN_VAL:
        return s->tokens[s->selected_token];
    default:
        return s->regs[offset / sizeof(uint32_t)];
    }
}

static void mt8113_gce_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    MT8113GCEState *s = opaque;

    if (offset == GCE_SYNC_TOKEN_ID) {
        s->selected_token = value & 0x3ff;
        s->regs[offset / 4] = value;
        return;
    }
    if (offset == GCE_SYNC_TOKEN_UPD) {
        unsigned token = value & 0x3ff;

        s->tokens[token] = value & BIT(16);
        s->regs[offset / 4] = value;
        return;
    }

    if (offset >= GCE_THR_BASE &&
        offset < GCE_THR_BASE + MT8113_GCE_THREADS * GCE_THR_SIZE) {
        unsigned thread = (offset - GCE_THR_BASE) / GCE_THR_SIZE;
        hwaddr reg = (offset - GCE_THR_BASE) % GCE_THR_SIZE;
        uint32_t *thread_reg = mt8113_gce_thread_reg(s, thread, reg);

        switch (reg) {
        case GCE_THR_WARM_RESET:
            *thread_reg = 0;
            *mt8113_gce_thread_reg(s, thread, GCE_THR_ENABLE) = 0;
            *mt8113_gce_thread_reg(s, thread, GCE_THR_SUSPEND) = 0;
            *mt8113_gce_thread_reg(s, thread, GCE_THR_STATUS) = 0;
            *mt8113_gce_thread_reg(s, thread, GCE_THR_IRQ_STATUS) = 0;
            *mt8113_gce_thread_reg(s, thread, GCE_THR_IRQ_ENABLE) = 0;
            s->waiting[thread] = false;
            s->pending_threads &= ~BIT(thread);
            mt8113_gce_update_irq(s);
            return;
        case GCE_THR_SUSPEND:
            *thread_reg = value;
            if (value) {
                *mt8113_gce_thread_reg(s, thread, GCE_THR_STATUS) |=
                    GCE_THR_STATUS_SUSPENDED;
            } else {
                *mt8113_gce_thread_reg(s, thread, GCE_THR_STATUS) &=
                    ~GCE_THR_STATUS_SUSPENDED;
                if (*mt8113_gce_thread_reg(s, thread,
                                           GCE_THR_ENABLE) & 1) {
                    mt8113_gce_schedule_thread(s, thread);
                }
            }
            return;
        case GCE_THR_IRQ_STATUS:
            *thread_reg &= value;
            mt8113_gce_update_irq(s);
            return;
        case GCE_THR_ENABLE:
            *thread_reg = value;
            if (value & 1) {
                if (*mt8113_gce_thread_reg(s, thread,
                                            GCE_THR_IRQ_ENABLE)) {
                    /*
                     * Mailbox tasks must not interrupt the guest before the
                     * driver has linked them into task_busy_list.  The
                     * interrupt-free boot-time token-clear program, however,
                     * is polled synchronously by the stock driver.
                     */
                    mt8113_gce_schedule_thread(s, thread);
                } else {
                    mt8113_gce_execute(s, thread);
                }
            } else {
                s->pending_threads &= ~BIT(thread);
            }
            return;
        default:
            *thread_reg = value;
            return;
        }
    }

    s->regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps mt8113_gce_ops = {
    .read = mt8113_gce_read,
    .write = mt8113_gce_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mt8113_gce_reset(DeviceState *dev)
{
    MT8113GCEState *s = MT8113_GCE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->gpr, 0, sizeof(s->gpr));
    memset(s->tokens, 0, sizeof(s->tokens));
    memset(s->waiting, 0, sizeof(s->waiting));
    memset(s->wait_token, 0, sizeof(s->wait_token));
    s->selected_token = 0;
    s->pending_threads = 0;
    timer_del(s->exec_timer);
    qemu_set_irq(s->irq, 0);
}

static void mt8113_gce_init(Object *obj)
{
    MT8113GCEState *s = MT8113_GCE(obj);

    memory_region_init_io(&s->iomem, obj, &mt8113_gce_ops, s,
                          TYPE_MT8113_GCE, MT8113_GCE_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), mt8113_gce_event,
                            "hwtcon-frame-done", 1);
    s->exec_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 mt8113_gce_run_pending, s);
}

static void mt8113_gce_finalize(Object *obj)
{
    MT8113GCEState *s = MT8113_GCE(obj);

    timer_free(s->exec_timer);
}

static void mt8113_gce_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, mt8113_gce_reset);
}

static const TypeInfo mt8113_gce_type = {
    .name = TYPE_MT8113_GCE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8113GCEState),
    .instance_init = mt8113_gce_init,
    .instance_finalize = mt8113_gce_finalize,
    .class_init = mt8113_gce_class_init,
};

static void mt8113_gce_register_types(void)
{
    type_register_static(&mt8113_gce_type);
}
type_init(mt8113_gce_register_types)
