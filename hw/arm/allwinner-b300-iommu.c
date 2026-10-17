/*
 * Allwinner sun8iw15p1 seven-master multimedia IOMMU.
 * Native 4 KiB pages use a 4096-entry directory and 256-entry tables.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/arm/allwinner-b300-iommu.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "system/address-spaces.h"

#define TYPE_B300_IOMMU_MEMORY "allwinner-b300-iommu-memory"
#define B300_IOMMU_MASTERS 7
#define B300_IOMMU_SIZE 0x1000
#define R_RESET 0x10
#define R_ENABLE 0x20
#define R_BYPASS 0x30
#define R_TTB 0x50
#define R_FLUSH 0x80
#define R_TLB_INVALIDATE 0x98
#define R_PC_INVALIDATE 0xa8
#define R_INT_ENABLE 0x100
#define R_INT_CLEAR 0x104
#define R_INT_STATUS 0x108
#define R_L1_STATUS 0x180
#define R_L2_STATUS 0x184
#define R_VA 0x190
#define R_VA_DATA 0x194
#define R_VA_CONTROL 0x198

typedef struct B300IOMMUMemory {
    IOMMUMemoryRegion parent_obj;
    AWB300IOMMUState *state;
    unsigned master;
} B300IOMMUMemory;

struct AWB300IOMMUState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    B300IOMMUMemory dma[B300_IOMMU_MASTERS];
    AddressSpace as[B300_IOMMU_MASTERS];
    uint32_t regs[B300_IOMMU_SIZE / 4];
    qemu_irq irq;
};

static void b300_iommu_irq(AWB300IOMMUState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R_INT_STATUS / 4] &
                           s->regs[R_INT_ENABLE / 4]));
}

static void b300_iommu_invalidate(AWB300IOMMUState *s)
{
    IOMMUTLBEvent event = {
        .type = IOMMU_NOTIFIER_UNMAP,
        .entry = {
            .target_as = &address_space_memory,
            .iova = 0,
            .addr_mask = UINT32_MAX,
            .perm = IOMMU_NONE,
        },
    };

    /* There is no host translation cache. Invalidate any DMA clients' views. */
    for (unsigned i = 0; i < B300_IOMMU_MASTERS; i++) {
        memory_region_notify_iommu(&s->dma[i].parent_obj, 0, event);
    }
}

static void b300_iommu_fault(AWB300IOMMUState *s, unsigned master,
                             hwaddr addr, unsigned level, uint32_t data)
{
    uint32_t status = level ? BIT(15 + level) : BIT(master);
    unsigned slot = level ? 0x130 + (level - 1) * 4 : 0x110 + master * 4;

    s->regs[R_INT_STATUS / 4] |= status;
    s->regs[slot / 4] = addr;
    s->regs[(slot + 0x40) / 4] = data;
    if (level) {
        s->regs[(level == 1 ? R_L1_STATUS : R_L2_STATUS) / 4] |= BIT(master);
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "B300 IOMMU master%u fault level%u IOVA0x%08" HWADDR_PRIx
                  " entry0x%08x\n", master, level, addr, data);
    b300_iommu_irq(s);
}

static IOMMUTLBEntry b300_iommu_walk(AWB300IOMMUState *s, unsigned master,
                                    hwaddr addr, IOMMUAccessFlags flags,
                                    bool debug)
{
    MemTxResult result;
    uint32_t l1, l2;
    hwaddr table;
    IOMMUTLBEntry entry = {
        .target_as = &address_space_memory,
        .iova = addr & ~0xfffULL,
        .addr_mask = 0xfff,
        .perm = IOMMU_NONE,
    };

    if (!debug && (!(s->regs[R_ENABLE / 4] & BIT(0)) ||
                   (s->regs[R_BYPASS / 4] & BIT(master)))) {
        entry.translated_addr = entry.iova;
        entry.perm = IOMMU_RW;
        return entry;
    }
    table = s->regs[R_TTB / 4] & ~0x3fffU;
    l1 = address_space_ldl_le(&address_space_memory,
                              table + ((addr >> 20) & 0xfff) * 4,
                              MEMTXATTRS_UNSPECIFIED, &result);
    if (result != MEMTX_OK || (l1 & 3) != 1) {
        b300_iommu_fault(s, master, addr, 1, l1);
        return entry;
    }
    table = l1 & ~0x3ffU;
    l2 = address_space_ldl_le(&address_space_memory,
                              table + ((addr >> 12) & 0xff) * 4,
                              MEMTXATTRS_UNSPECIFIED, &result);
    if (result != MEMTX_OK || !(l2 & BIT(1))) {
        b300_iommu_fault(s, master, addr, 2, l2);
        return entry;
    }
    entry.perm = ((l2 & BIT(2)) ? IOMMU_RO : 0) |
                 ((l2 & BIT(3)) ? IOMMU_WO : 0);
    if ((flags & entry.perm) != flags) {
        b300_iommu_fault(s, master, addr, 0, l2);
        entry.perm = IOMMU_NONE;
        return entry;
    }
    entry.translated_addr = l2 & ~0xfffU;
    return entry;
}

static IOMMUTLBEntry b300_iommu_translate(IOMMUMemoryRegion *iommu,
                                         hwaddr addr, IOMMUAccessFlags flags,
                                         int iommu_idx)
{
    B300IOMMUMemory *dma = container_of(iommu, B300IOMMUMemory, parent_obj);

    return b300_iommu_walk(dma->state, dma->master, addr, flags, false);
}

AddressSpace *aw_b300_iommu_as(AWB300IOMMUState *s, unsigned master)
{
    assert(master < B300_IOMMU_MASTERS);
    return &s->as[master];
}

static uint64_t b300_iommu_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_B300_IOMMU(opaque)->regs[offset / 4];
}

static void b300_iommu_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    AWB300IOMMUState *s = AW_B300_IOMMU(opaque);

    if (offset == 0 || offset == R_INT_STATUS ||
        offset == R_L1_STATUS || offset == R_L2_STATUS) {
        return;
    }
    if (offset == R_INT_CLEAR) {
        s->regs[R_INT_STATUS / 4] &= ~value;
        if (value & BIT(16)) {
            s->regs[R_L1_STATUS / 4] = 0;
        }
        if (value & BIT(17)) {
            s->regs[R_L2_STATUS / 4] = 0;
        }
        b300_iommu_irq(s);
        return;
    }
    s->regs[offset / 4] = value;
    switch (offset) {
    case R_ENABLE:
    case R_BYPASS:
    case R_TTB:
    case R_RESET:
        b300_iommu_invalidate(s);
        break;
    case R_FLUSH:
    case R_TLB_INVALIDATE:
    case R_PC_INVALIDATE:
        b300_iommu_invalidate(s);
        s->regs[offset / 4] = 0;
        break;
    case R_VA_CONTROL:
        if (value & BIT(0)) {
            bool write = value & BIT(8);
            IOMMUTLBEntry entry = b300_iommu_walk(s, 0, s->regs[R_VA / 4],
                                        write ? IOMMU_WO : IOMMU_RO, true);
            if (entry.perm != IOMMU_NONE) {
                hwaddr physical = entry.translated_addr |
                                   (s->regs[R_VA / 4] & 0xfff);
                if (write) {
                    address_space_stl_le(&address_space_memory, physical,
                        s->regs[R_VA_DATA / 4], MEMTXATTRS_UNSPECIFIED, NULL);
                } else {
                    s->regs[R_VA_DATA / 4] = address_space_ldl_le(
                        &address_space_memory, physical,
                        MEMTXATTRS_UNSPECIFIED, NULL);
                }
            }
            s->regs[offset / 4] &= ~BIT(0);
        }
        break;
    case 0x210: /* PMU counters are zero when no transactions are counted. */
        if (value & BIT(0)) {
            memset(&s->regs[0x230 / 4], 0, 0x140);
        }
        s->regs[offset / 4] &= ~BIT(0);
        break;
    }
    b300_iommu_irq(s);
}

static const MemoryRegionOps b300_iommu_ops = {
    .read = b300_iommu_read,
    .write = b300_iommu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void b300_iommu_reset(DeviceState *dev)
{
    AWB300IOMMUState *s = AW_B300_IOMMU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_RESET / 4] = UINT32_MAX;
    s->regs[R_BYPASS / 4] = 0x7f;
    b300_iommu_invalidate(s);
    b300_iommu_irq(s);
}

static int b300_iommu_post_load(void *opaque, int version)
{
    AWB300IOMMUState *s = opaque;

    b300_iommu_invalidate(s);
    b300_iommu_irq(s);
    return 0;
}

static const VMStateDescription b300_iommu_vmstate = {
    .name = TYPE_AW_B300_IOMMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = b300_iommu_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AWB300IOMMUState, B300_IOMMU_SIZE / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void b300_iommu_init(Object *obj)
{
    AWB300IOMMUState *s = AW_B300_IOMMU(obj);

    memory_region_init_io(&s->mmio, obj, &b300_iommu_ops, s,
                            TYPE_AW_B300_IOMMU, B300_IOMMU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    for (unsigned i = 0; i < B300_IOMMU_MASTERS; i++) {
        g_autofree char *name = g_strdup_printf("b300.iommu-master%u", i);

        memory_region_init_iommu(&s->dma[i].parent_obj, sizeof(s->dma[i]),
                                  TYPE_B300_IOMMU_MEMORY, obj, name, 1ULL << 32);
        s->dma[i].state = s;
        s->dma[i].master = i;
        address_space_init(&s->as[i], MEMORY_REGION(&s->dma[i].parent_obj), name);
    }
}

static void b300_iommu_finalize(Object *obj)
{
    AWB300IOMMUState *s = AW_B300_IOMMU(obj);

    for (unsigned i = 0; i < B300_IOMMU_MASTERS; i++) {
        address_space_destroy(&s->as[i]);
    }
}

static void b300_iommu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->vmsd = &b300_iommu_vmstate;
    device_class_set_legacy_reset(dc, b300_iommu_reset);
}

static void b300_iommu_memory_class_init(ObjectClass *oc, const void *data)
{
    IOMMU_MEMORY_REGION_CLASS(oc)->translate = b300_iommu_translate;
}

static const TypeInfo b300_iommu_types[] = {
    {
        .name = TYPE_AW_B300_IOMMU,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AWB300IOMMUState),
        .instance_init = b300_iommu_init,
        .instance_finalize = b300_iommu_finalize,
        .class_init = b300_iommu_class_init,
    }, {
        .name = TYPE_B300_IOMMU_MEMORY,
        .parent = TYPE_IOMMU_MEMORY_REGION,
        .instance_size = sizeof(B300IOMMUMemory),
        .class_init = b300_iommu_memory_class_init,
    },
};
DEFINE_TYPES(b300_iommu_types)
