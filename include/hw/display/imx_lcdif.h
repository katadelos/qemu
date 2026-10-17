/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_DISPLAY_IMX_LCDIF_H
#define HW_DISPLAY_IMX_LCDIF_H
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#define LCDIF_WORDS (0x8000 / 4)
#define TYPE_IMX_LCDIF "imx-lcdif"
OBJECT_DECLARE_SIMPLE_TYPE(IMXLCDIFState, IMX_LCDIF)
typedef void (*IMXLCDIFOutput)(void *opaque, hwaddr buffer,
                               unsigned width, unsigned height,
                               unsigned word_length);
struct IMXLCDIFState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[LCDIF_WORDS];
    uint32_t pixel_clock;
    bool latch_masked_frame_events;
    IMXLCDIFOutput output;
    void *output_opaque;
};
void imx_lcdif_set_output(IMXLCDIFState *s, IMXLCDIFOutput output, void *opaque);
#endif
