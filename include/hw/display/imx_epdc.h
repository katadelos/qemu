/*
 * i.MX electrophoretic display controller (EPDC)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_IMX_EPDC_H
#define HW_DISPLAY_IMX_EPDC_H

#include "hw/core/sysbus.h"
#include "hw/misc/imx6sl_pxp.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "ui/console.h"

#define TYPE_IMX_EPDC "imx.epdc"
OBJECT_DECLARE_SIMPLE_TYPE(IMXEPDCState, IMX_EPDC)

#define IMX_EPDC_MMIO_SIZE 0x1000

struct IMXEPDCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUBH *complete_bh;
    QEMUTimer *refresh_timer;
    QemuConsole *console;
    IMX6SLPXPState *pxp;
    uint8_t *fb_buffer;
    uint64_t fb_addr;
    uint32_t fb_width;
    uint32_t fb_height;
    uint32_t fb_stride;
    uint8_t fb_bpp;
    uint32_t regs[IMX_EPDC_MMIO_SIZE / sizeof(uint32_t)];
    bool update_pending;
};

#endif
