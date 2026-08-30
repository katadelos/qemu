/*
 * MediaTek MT8113 multimedia IOMMU register interface
 *
 * Linux owns the ARMv7s translation tables.  DMA-capable MT8113 models can
 * walk those tables directly; this block supplies the hardware-visible TTBR
 * and completes TLB invalidation requests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/mt8113_iommu.h"
#include "hw/core/irq.h"
#include "qemu/bitops.h"
#include "qemu/module.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define MT8113_IOMMU_INVALIDATE  0x020
#define MT8113_IOMMU_INV_START   0x024
#define MT8113_IOMMU_INV_END     0x028
#define MT8113_IOMMU_CPE_DONE    0x12c
#define MT8113_IOMMU_INV_RANGE   BIT(0)
#define MT8113_IOMMU_INV_ALL     BIT(1)
#define MT8113_IOMMU_PAGE_SHIFT  12
#define MT8113_IOMMU_PAGE_SIZE   (1 << MT8113_IOMMU_PAGE_SHIFT)

static void mt8113_iommu_flush_tlb(MT8113IOMMUState *s)
{
    memset(s->tlb_iova_page, 0xff, sizeof(s->tlb_iova_page));
}

static void mt8113_iommu_flush_tlb_range(MT8113IOMMUState *s,
                                          uint32_t start, uint32_t end)
{
    uint32_t first_page = start >> MT8113_IOMMU_PAGE_SHIFT;
    uint32_t last_page = end >> MT8113_IOMMU_PAGE_SHIFT;

    if (end < start || last_page - first_page >= MT8113_IOMMU_TLB_ENTRIES) {
        mt8113_iommu_flush_tlb(s);
        return;
    }

    for (uint32_t page = first_page; page <= last_page; page++) {
        unsigned index = page & (MT8113_IOMMU_TLB_ENTRIES - 1);

        if (s->tlb_iova_page[index] == page) {
            s->tlb_iova_page[index] = UINT32_MAX;
        }
    }
}

static uint64_t mt8113_iommu_mmio_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    MT8113IOMMUState *s = opaque;

    return s->regs[offset / sizeof(uint32_t)];
}

static void mt8113_iommu_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    MT8113IOMMUState *s = opaque;
    uint32_t old = s->regs[offset / sizeof(uint32_t)];

    s->regs[offset / sizeof(uint32_t)] = value;
    if (offset == 0 && old != value) {
        mt8113_iommu_flush_tlb(s);
    }
    if (offset == MT8113_IOMMU_INVALIDATE) {
        if (value & MT8113_IOMMU_INV_ALL) {
            mt8113_iommu_flush_tlb(s);
        } else if (value & MT8113_IOMMU_INV_RANGE) {
            mt8113_iommu_flush_tlb_range(
                s, s->regs[MT8113_IOMMU_INV_START / sizeof(uint32_t)],
                s->regs[MT8113_IOMMU_INV_END / sizeof(uint32_t)]);
        }
        if (value & MT8113_IOMMU_INV_RANGE) {
            s->regs[MT8113_IOMMU_CPE_DONE / sizeof(uint32_t)] = 1;
        }
    }
}

static const MemoryRegionOps mt8113_iommu_ops = {
    .read = mt8113_iommu_mmio_read,
    .write = mt8113_iommu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static bool mt8113_iommu_translate_span(MT8113IOMMUState *s, hwaddr iova,
                                        hwaddr *physical, size_t *span)
{
    MemTxResult result;
    uint32_t l1, l2;
    uint32_t iova_page;
    unsigned index;
    hwaddr table = s->regs[0] & 0xffffc000;

    if (iova >= 0x10000000) {
        *physical = iova;
        *span = SIZE_MAX;
        return true;
    }
    if (!table) {
        return false;
    }

    iova_page = iova >> MT8113_IOMMU_PAGE_SHIFT;
    index = iova_page & (MT8113_IOMMU_TLB_ENTRIES - 1);
    if (s->tlb_iova_page[index] == iova_page) {
        *physical = s->tlb_physical_page[index] |
                    (iova & (MT8113_IOMMU_PAGE_SIZE - 1));
        *span = MT8113_IOMMU_PAGE_SIZE -
                (iova & (MT8113_IOMMU_PAGE_SIZE - 1));
        return true;
    }

    l1 = address_space_ldl_le(&address_space_memory,
                              table + ((iova >> 20) & 0xfff) * 4,
                              MEMTXATTRS_UNSPECIFIED, &result);
    if (result != MEMTX_OK) {
        return false;
    }
    if ((l1 & 3) == 1) {
        table = l1 & 0xfffffc00;
        l2 = address_space_ldl_le(&address_space_memory,
                                  table + ((iova >> 12) & 0xff) * 4,
                                  MEMTXATTRS_UNSPECIFIED, &result);
        if (result != MEMTX_OK || !(l2 & 3)) {
            return false;
        }
        if ((l2 & 3) == 1) {
            /* ARMv7 short-descriptor large page: the same 64 KiB
             * descriptor occupies sixteen consecutive L2 entries. */
            *physical = (l2 & 0xffff0000) | (iova & 0xffff);
        } else {
            *physical = (l2 & 0xfffff000) | (iova & 0xfff);
        }
    } else if ((l1 & 3) == 2) {
        *physical = (l1 & 0xfff00000) | (iova & 0xfffff);
    } else {
        return false;
    }

    s->tlb_iova_page[index] = iova_page;
    s->tlb_physical_page[index] = *physical &
                                  ~(hwaddr)(MT8113_IOMMU_PAGE_SIZE - 1);
    *span = MT8113_IOMMU_PAGE_SIZE -
            (iova & (MT8113_IOMMU_PAGE_SIZE - 1));
    return true;
}

bool mt8113_iommu_translate(MT8113IOMMUState *s, hwaddr iova,
                            hwaddr *physical)
{
    size_t span;

    return mt8113_iommu_translate_span(s, iova, physical, &span);
}

bool mt8113_iommu_dma_read(MT8113IOMMUState *s, hwaddr iova,
                           void *buffer, size_t length)
{
    uint8_t *destination = buffer;

    while (length) {
        hwaddr physical;
        size_t span;
        size_t chunk;

        if (!mt8113_iommu_translate_span(s, iova, &physical, &span)) {
            return false;
        }
        chunk = MIN(length, span);
        if (dma_memory_read(&address_space_memory, physical, destination,
                            chunk, MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return false;
        }
        iova += chunk;
        destination += chunk;
        length -= chunk;
    }
    return true;
}

bool mt8113_iommu_dma_write(MT8113IOMMUState *s, hwaddr iova,
                            const void *buffer, size_t length)
{
    const uint8_t *source = buffer;

    while (length) {
        hwaddr physical;
        size_t span;
        size_t chunk;

        if (!mt8113_iommu_translate_span(s, iova, &physical, &span)) {
            return false;
        }
        chunk = MIN(length, span);
        if (dma_memory_write(&address_space_memory, physical, source,
                             chunk, MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return false;
        }
        iova += chunk;
        source += chunk;
        length -= chunk;
    }
    return true;
}

static void mt8113_iommu_reset(DeviceState *dev)
{
    MT8113IOMMUState *s = MT8113_IOMMU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    mt8113_iommu_flush_tlb(s);
    qemu_set_irq(s->irq, 0);
}

static void mt8113_iommu_init(Object *obj)
{
    MT8113IOMMUState *s = MT8113_IOMMU(obj);

    memory_region_init_io(&s->iomem, obj, &mt8113_iommu_ops, s,
                          TYPE_MT8113_IOMMU, MT8113_IOMMU_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void mt8113_iommu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, mt8113_iommu_reset);
}

static const TypeInfo mt8113_iommu_type = {
    .name = TYPE_MT8113_IOMMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8113IOMMUState),
    .instance_init = mt8113_iommu_init,
    .class_init = mt8113_iommu_class_init,
};

static void mt8113_iommu_register_types(void)
{
    type_register_static(&mt8113_iommu_type);
}
type_init(mt8113_iommu_register_types)
