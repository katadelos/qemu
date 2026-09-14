/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_DISPLAY_MT8171_PANEL_H
#define HW_DISPLAY_MT8171_PANEL_H
#include "hw/core/sysbus.h"
#include "ui/console.h"
#include "qemu/queue.h"
#include "hw/misc/mt8171-m4u.h"

/* Diagnostic guest-framebuffer console, explicitly before CFA/waveform drive. */
#define TYPE_MT8171_PANEL "mt8171-framebuffer-preview"
OBJECT_DECLARE_SIMPLE_TYPE(MT8171PanelState, MT8171_PANEL)
typedef struct MT8171PanelTile MT8171PanelTile;
struct MT8171PanelState {
    SysBusDevice parent_obj;
    QemuConsole *console;
    /* Read-only link to the controller's EPD register bank. */
    uint32_t *hwtcon_regs;
    /* Optional observed CFA intermediate allocation. Zero follows HWTCON. */
    uint64_t image_buffer;
    uint64_t observed_image_buffer;
    uint32_t width, height;
    /* Physical controller coordinates; frontend presentation rotates CW90. */
    uint32_t *pixels;
    unsigned plane_width, plane_height;
    uint64_t captured_tiles, presented_tiles, rejected_tiles;
    size_t queued_bytes;
    bool dirty;
    QTAILQ_HEAD(, MT8171PanelTile) pending;
};

/* Observe bytes already fetched by a genuine MDP DMA transaction. The source
 * format/swap is the programmed RDMA encoding. Destination metadata is the
 * WROT output address after normalizing its rotation offset to the tile origin.
 * Rendering never reads guest memory, performs DMA, changes registers or raises
 * hardware interrupts. Pixel processing may still fail after this observation.
 */
void mt8171_panel_capture_mdp(MT8171PanelState *s, const uint8_t *source,
                              unsigned source_pitch, unsigned width,
                              unsigned height, unsigned format, bool swap,
                              unsigned rotation, hwaddr destination,
                              unsigned destination_pitch,
                              unsigned destination_bpp);
/* Explicit diagnostic IOVA observation through Linux's bank0 page table,
 * independent of hardware stream enables. No physical fallback or device
 * completion; only mapped RAM is read. Source starts at the cropped tile. */
void mt8171_panel_capture_iova(MT8171PanelState *s, MT8171M4UState *m4u,
                               hwaddr source, unsigned source_pitch,
                               unsigned width, unsigned height,
                               unsigned format, bool swap, unsigned rotation,
                               hwaddr destination, unsigned destination_pitch,
                               unsigned destination_bpp);
#endif
