/*
 * i.MX50 electrophoretic display controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_IMX50_EPDC_H
#define HW_DISPLAY_IMX50_EPDC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "ui/console.h"

#define TYPE_IMX50_EPDC "imx50.epdc"
OBJECT_DECLARE_SIMPLE_TYPE(IMX50EPDCState, IMX50_EPDC)

#define IMX50_EPDC_MMIO_SIZE 0x1000
#define IMX50_EPDC_RGB_WIDTH 1800
#define IMX50_EPDC_RGB_HEIGHT 800

struct IMX50EPDCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *wb_timer;
    QEMUTimer *lut_timer;
    QEMUTimer *scanout_timer;
    QemuConsole *console;
    uint8_t *migration_panel;
    uint32_t migration_panel_width;
    uint32_t migration_panel_height;
    uint32_t migration_panel_size;
    uint32_t regs[IMX50_EPDC_MMIO_SIZE / sizeof(uint32_t)];
    uint32_t pending_luts;
    int64_t lut_deadline[16];
    uint64_t direct_fb_addr;
    uint32_t direct_fb_stride;
    uint32_t direct_fb_width;
    uint32_t direct_fb_height;
    uint8_t *direct_fb_buffer;
    uint64_t scanout_refreshes;
    bool direct_fb;
    bool rotate_ccw;
    bool rgb_stripe;
    uint8_t rgb_buffer[IMX50_EPDC_RGB_WIDTH * IMX50_EPDC_RGB_HEIGHT];
};

#endif
