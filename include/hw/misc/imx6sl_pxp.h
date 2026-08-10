/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_IMX6SL_PXP_H
#define HW_MISC_IMX6SL_PXP_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_IMX6SL_PXP "imx6sl-pxp"
OBJECT_DECLARE_SIMPLE_TYPE(IMX6SLPXPState, IMX6SL_PXP)

#define IMX6SL_PXP_SIZE 0x4000

struct IMX6SLPXPState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer completion_timer;
    uint32_t regs[IMX6SL_PXP_SIZE / sizeof(uint32_t)];
    uint8_t lut[256];
    uint16_t lut_addr;
    bool running;
};

typedef struct IMX6SLPXPFetch {
    uint64_t addr;
    uint32_t pitch;
    uint32_t left;
    uint32_t top;
    uint32_t width;
    uint32_t height;
} IMX6SLPXPFetch;

bool imx6sl_pxp_get_wfe_a_fetch(IMX6SLPXPState *s,
                                IMX6SLPXPFetch *fetch);
bool imx6sl_pxp_get_wfe_b_store(IMX6SLPXPState *s,
                                IMX6SLPXPFetch *store);

#endif
