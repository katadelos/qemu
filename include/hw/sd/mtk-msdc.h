/*
 * MediaTek MultiMediaCard/Secure Digital Card controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SD_MTK_MSDC_H
#define HW_SD_MTK_MSDC_H

#include "hw/core/sysbus.h"
#include "hw/sd/sd.h"
#include "qom/object.h"

#define TYPE_MTK_MSDC "mtk-msdc"
OBJECT_DECLARE_SIMPLE_TYPE(MTKMSDCState, MTK_MSDC)

#define MTK_MSDC_MMIO_SIZE 0x1000

struct MTKMSDCState {
    SysBusDevice parent_obj;

    SDBus sdbus;
    MemoryRegion iomem;
    MemoryRegion top_iomem;
    qemu_irq irq;

    uint32_t regs[MTK_MSDC_MMIO_SIZE / sizeof(uint32_t)];
    uint32_t top_regs[MTK_MSDC_MMIO_SIZE / sizeof(uint32_t)];
    uint32_t transfer_remaining;
    bool transfer_write;
    bool dma_active;
    QEMUTimer *dma_timer;
};

#endif /* HW_SD_MTK_MSDC_H */
