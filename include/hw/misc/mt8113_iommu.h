/*
 * MediaTek MT8113 multimedia IOMMU
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MT8113_IOMMU_H
#define HW_MISC_MT8113_IOMMU_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_MT8113_IOMMU "mt8113.iommu"
OBJECT_DECLARE_SIMPLE_TYPE(MT8113IOMMUState, MT8113_IOMMU)

#define MT8113_IOMMU_MMIO_SIZE 0x1000

struct MT8113IOMMUState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[MT8113_IOMMU_MMIO_SIZE / sizeof(uint32_t)];
};

bool mt8113_iommu_translate(MT8113IOMMUState *s, hwaddr iova,
                            hwaddr *physical);
bool mt8113_iommu_dma_read(MT8113IOMMUState *s, hwaddr iova,
                           void *buffer, size_t length);

#endif
