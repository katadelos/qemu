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
#define MT8113_IOMMU_CPE_DONE    0x12c
#define MT8113_IOMMU_INV_RANGE   BIT(0)

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

    s->regs[offset / sizeof(uint32_t)] = value;
    if (offset == MT8113_IOMMU_INVALIDATE &&
        (value & MT8113_IOMMU_INV_RANGE)) {
        s->regs[MT8113_IOMMU_CPE_DONE / sizeof(uint32_t)] = 1;
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
    hwaddr table = s->regs[0] & 0xffffc000;

    if (iova >= 0x10000000) {
        *physical = iova;
        *span = SIZE_MAX;
        return true;
    }
    if (!table) {
        return false;
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
        *physical = (l2 & 0xfffff000) | (iova & 0xfff);
        *span = 0x1000 - (iova & 0xfff);
        return true;
    }
    if ((l1 & 3) == 2) {
        *physical = (l1 & 0xfff00000) | (iova & 0xfffff);
        *span = 0x100000 - (iova & 0xfffff);
        return true;
    }
    return false;
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

static void mt8113_iommu_reset(DeviceState *dev)
{
    MT8113IOMMUState *s = MT8113_IOMMU(dev);

    memset(s->regs, 0, sizeof(s->regs));
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
