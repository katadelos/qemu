/*
 * B300 EINK index engine, waveform decoder, EDMA and timing controller.
 *
 * The native driver supplies 8-bit images to the index engine, which writes
 * 32-by-32 transition indices. Hardware pipelines and the driver's software
 * pipelines compose two-bit electrical drive frames in shared DMA buffers.
 * EDMA scans those buffers through TCON. Only that physical scan changes the
 * retained optical panel; preparing an index or decoding a frame does not.
 *
 * Register layout and packing follow PocketBook's kernel-b300 de_eink.c,
 * de_lcd_type.h and eink_pipeline_manager.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/arm/allwinner-b300-iommu.h"
#include "hw/display/allwinner-b300-eink.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "trace.h"

#define DE_SIZE 0x400000
#define TCON_SIZE 0x1000
#define EINK_SIZE 0x20000
#define TOP_SIZE 0x2000
#define PIPE_COUNT 16

#define EE_CTL 0x000
#define EE_IRQ 0x004
#define EE_IDX_GEN 0x008
#define EE_IMG_SIZE 0x00c
#define EE_LAST_IMG_ADDR 0x010
#define EE_LAST_IMG_PITCH 0x014
#define EE_CURR_IMG_ADDR 0x018
#define EE_CURR_IMG_PITCH 0x01c
#define EE_LAST_IDX_ADDR 0x020
#define EE_IDX_WIN0 0x024
#define EE_IDX_WIN1 0x028
#define EE_NEW_IDX_ADDR 0x02c
#define EE_PIPE 0x040
#define EE_DEC_CTL 0x150
#define EE_DEC_IDX 0x154
#define EE_DEC_WAV 0x158
#define EE_DEC_PITCH 0x15c
#define EDMA_GLB_CTL 0x400
#define EDMA_CTL 0x404
#define EDMA_WAV_ADDR 0x408
#define EDMA_WAV_PITCH 0x40c
#define EDMA_WAV_SIZE 0x410
#define EDMA_IMG_COOR 0x414
#define EDMA_IMG_SIZE 0x418
#define INDEX_REGS (EE_NEW_IDX_ADDR / 4 + 1)
#define DECODE_REGS (EE_DEC_PITCH / 4 + 1)

OBJECT_DECLARE_SIMPLE_TYPE(AwB300EinkState, AW_B300_EINK)

struct AwB300EinkState {
    SysBusDevice parent_obj;
    MemoryRegion de_mr, tcon_mr, eink_mr, top_mr, tcon1_mr;
    uint32_t de[DE_SIZE / 4];
    uint32_t tcon[TCON_SIZE / 4], tcon1[TCON_SIZE / 4];
    uint32_t eink[EINK_SIZE / 4], top[TOP_SIZE / 4];
    uint32_t index_command[INDEX_REGS], decode_command[DECODE_REGS];
    uint32_t width, height, rotation, color_filter, refresh_rate, panel_size;
    uint8_t *panel;
    uint32_t edma_active[5], edma_pending[5];
    bool dirty, edma_ready, edma_active_valid, frame_latched, vblank;
    int64_t frame_start, last_presented;
    QemuConsole *console;
    QEMUTimer *frame_timer, *index_timer, *decode_timer;
    qemu_irq de_irq, tcon_irq, eink_irq, tcon1_irq;
    AWB300IOMMUState *iommu;
};

static bool b300_dma_valid(hwaddr address, uint64_t length)
{
    return length && address <= UINT32_MAX &&
           length <= 0x100000000ULL - address;
}

static unsigned b300_eink_channel(unsigned x, unsigned y)
{
    /* Native colormask 3: GBR / BRG / RGB in panel coordinates. */
    return (x + y + 1) % 3;
}

static uint32_t b300_eink_color(AwB300EinkState *s, unsigned x, unsigned y)
{
    unsigned sum[3] = { 0 }, weight[3] = { 0 };

    if (x && y && x + 1 < s->width && y + 1 < s->height) {
        const uint8_t *pixel = s->panel + y * s->width + x;
        const uint8_t *above = pixel - s->width;
        const uint8_t *below = pixel + s->width;
        unsigned center = (4 * pixel[0] + above[1] + below[-1]) / 6;
        unsigned next = (2 * pixel[1] + 2 * below[0] + above[-1]) / 5;
        unsigned previous = (2 * pixel[-1] + 2 * above[0] + below[1]) / 5;

        /*
         * The diagonal filter gives the center channel weight 6, the other
         * two weight 5.  Only border pixels need clamped sampling below.
         */
        switch (b300_eink_channel(x, y)) {
        case 0:
            return rgb_to_pixel32(center, next, previous);
        case 1:
            return rgb_to_pixel32(previous, center, next);
        default:
            return rgb_to_pixel32(next, previous, center);
        }
    }

    for (int oy = -1; oy <= 1; oy++) {
        unsigned sy = CLAMP((int)y + oy, 0, (int)s->height - 1);
        for (int ox = -1; ox <= 1; ox++) {
            unsigned sx = CLAMP((int)x + ox, 0, (int)s->width - 1);
            unsigned c = b300_eink_channel(sx, sy);
            unsigned w = (ox ? 1 : 2) * (oy ? 1 : 2);
            sum[c] += s->panel[sy * s->width + sx] * w;
            weight[c] += w;
        }
    }
    return rgb_to_pixel32(sum[0] / weight[0], sum[1] / weight[1],
                          sum[2] / weight[2]);
}

static bool b300_eink_enabled(AwB300EinkState *s);

static void b300_eink_draw(void *opaque)
{
    AwB300EinkState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->console);
    uint8_t *pixels = surface_data(surface);
    unsigned stride = surface_stride(surface);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

    /* Bound host presentation work without hiding continuous native updates. */
    if (!s->dirty || (b300_eink_enabled(s) && s->last_presented &&
                      now - s->last_presented < NANOSECONDS_PER_SECOND / 5)) {
        return;
    }
    for (unsigned y = 0; y < s->height; y++) {
        for (unsigned x = 0; x < s->width; x++) {
            unsigned dx = x, dy = y;
            uint8_t grey = s->panel[y * s->width + x];
            uint32_t pixel = s->color_filter ? b300_eink_color(s, x, y) :
                                              rgb_to_pixel32(grey, grey, grey);

            if (s->rotation == 90) {
                dx = s->height - y - 1;
                dy = x;
            } else if (s->rotation == 270) {
                dx = y;
                dy = s->width - x - 1;
            } else if (s->rotation == 180) {
                dx = s->width - x - 1;
                dy = s->height - y - 1;
            }
            stl_he_p(pixels + dy * stride + dx * 4, pixel);
        }
    }
    s->dirty = false;
    s->last_presented = now;
    dpy_gfx_update(s->console, 0, 0, surface_width(surface),
                   surface_height(surface));
}

static void b300_eink_invalidate(void *opaque)
{
    AW_B300_EINK(opaque)->dirty = true;
}

static const GraphicHwOps b300_eink_graphics = {
    .invalidate = b300_eink_invalidate,
    .gfx_update = b300_eink_draw,
};

static void b300_eink_irq(AwB300EinkState *s)
{
    uint32_t status = s->eink[EE_IRQ / 4];
    qemu_set_irq(s->eink_irq, !!((status & BIT(4)) && (status & 3)));
    qemu_set_irq(s->tcon_irq, !!((s->tcon[1] >> 16) & s->tcon[1] & 0xffff));
    qemu_set_irq(s->tcon1_irq,
                 !!((s->tcon1[1] >> 16) & s->tcon1[1] & 0xffff));
}

static unsigned b300_eink_bits(uint32_t control)
{
    return (control & 3) == 2 ? 5 : 4;
}

static void b300_eink_index_done(void *opaque)
{
    AwB300EinkState *s = opaque;
    const uint32_t *r = s->index_command;
    unsigned w = (r[EE_IMG_SIZE / 4] & 0xfff) + 1;
    unsigned h = ((r[EE_IMG_SIZE / 4] >> 16) & 0xfff) + 1;
    unsigned bits = b300_eink_bits(r[EE_CTL / 4]), unit = bits == 5 ? 2 : 1;
    unsigned old_pitch = r[EE_LAST_IMG_PITCH / 4];
    unsigned new_pitch = r[EE_CURR_IMG_PITCH / 4];
    hwaddr old_img = r[EE_LAST_IMG_ADDR / 4];
    hwaddr new_img = r[EE_CURR_IMG_ADDR / 4];
    hwaddr old_idx = r[EE_LAST_IDX_ADDR / 4];
    hwaddr new_idx = r[EE_NEW_IDX_ADDR / 4];
    bool automatic = r[EE_IDX_GEN / 4] & BIT(4);
    unsigned x0 = automatic ? 0 : r[EE_IDX_WIN0 / 4] & 0xfff;
    unsigned y0 = automatic ? 0 : (r[EE_IDX_WIN0 / 4] >> 16) & 0xfff;
    unsigned x1 = automatic ? w - 1 : r[EE_IDX_WIN1 / 4] & 0xfff;
    unsigned y1 = automatic ? h - 1 : (r[EE_IDX_WIN1 / 4] >> 16) & 0xfff;
    unsigned changed_x0 = w - 1, changed_y0 = h - 1, changed_x1 = 0, changed_y1 = 0;
    uint64_t index_size = (uint64_t)w * h * unit;
    AddressSpace *as = aw_b300_iommu_as(s->iommu, 2);
    g_autofree uint8_t *old_row = NULL;
    g_autofree uint8_t *new_row = NULL;
    g_autofree uint8_t *index_row = NULL;

    if (w != s->width || h != s->height || old_pitch < w || new_pitch < w ||
        !b300_dma_valid(old_img, (uint64_t)old_pitch * h) ||
        !b300_dma_valid(new_img, (uint64_t)new_pitch * h) ||
        !b300_dma_valid(old_idx, index_size) ||
        !b300_dma_valid(new_idx, index_size)) {
        goto complete;
    }

    old_row = g_malloc(w);
    new_row = g_malloc(w);
    index_row = g_malloc(w * unit);

    for (unsigned y = 0; y < h; y++) {
        if (address_space_read(as, old_idx + (uint64_t)y * w * unit,
                               MEMTXATTRS_UNSPECIFIED, index_row,
                               w * unit) != MEMTX_OK) {
            goto complete;
        }
        if (y >= y0 && y <= y1) {
            if (address_space_read(as, old_img + (uint64_t)y * old_pitch,
                                   MEMTXATTRS_UNSPECIFIED, old_row,
                                   w) != MEMTX_OK ||
                address_space_read(as, new_img + (uint64_t)y * new_pitch,
                                   MEMTXATTRS_UNSPECIFIED, new_row,
                                   w) != MEMTX_OK) {
                goto complete;
            }
            for (unsigned x = x0; x <= MIN(x1, w - 1); x++) {
                unsigned old = old_row[x] >> 4, new = new_row[x] >> 4;
                unsigned index = bits == 5 ? (old << 6) | (new << 1) :
                                             (old << 4) | new;
                if (unit == 2) {
                    stw_le_p(index_row + x * 2, index);
                } else {
                    index_row[x] = index;
                }
                if (old != new) {
                    changed_x0 = MIN(changed_x0, x);
                    changed_y0 = MIN(changed_y0, y);
                    changed_x1 = MAX(changed_x1, x);
                    changed_y1 = MAX(changed_y1, y);
                }
            }
        }
        if (address_space_write(as, new_idx + (uint64_t)y * w * unit,
                                MEMTXATTRS_UNSPECIFIED, index_row,
                                w * unit) != MEMTX_OK) {
            goto complete;
        }
    }
    if (automatic) {
        s->eink[EE_IDX_WIN0 / 4] = (changed_y0 << 16) | changed_x0;
        s->eink[EE_IDX_WIN1 / 4] = (changed_y1 << 16) | changed_x1;
    }
complete:
    trace_allwinner_b300_eink_index(r[EE_IDX_GEN / 4], old_idx, new_idx,
        r[EE_IDX_WIN0 / 4], r[EE_IDX_WIN1 / 4]);
    s->eink[EE_IDX_GEN / 4] &= ~1;
    s->eink[EE_IRQ / 4] |= BIT(1);
    b300_eink_irq(s);
}

static void b300_eink_decode_done(void *opaque)
{
    AwB300EinkState *s = opaque;
    const uint32_t *r = s->decode_command;
    int64_t started = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    unsigned w = (r[EE_IMG_SIZE / 4] & 0xfff) + 1;
    unsigned h = ((r[EE_IMG_SIZE / 4] >> 16) & 0xfff) + 1;
    unsigned bits = b300_eink_bits(r[EE_CTL / 4]), unit = bits == 5 ? 2 : 1;
    bool wide = r[EE_CTL / 4] & BIT(4);
    unsigned group_shift = wide ? 3 : 2, group = 1u << group_shift;
    unsigned word_size = wide ? 4 : 2;
    unsigned groups = (w + group - 1) >> group_shift;
    unsigned bytes = groups * word_size;
    unsigned index_mask = (1u << (bits * 2)) - 1;
    unsigned pitch = r[EE_DEC_PITCH / 4];
    hwaddr indexes = r[EE_DEC_IDX / 4];
    hwaddr output = r[EE_DEC_WAV / 4];
    AddressSpace *as = aw_b300_iommu_as(s->iommu, 6);
    uint8_t waves[PIPE_COUNT][1024];
    bool active[PIPE_COUNT];
    unsigned window[PIPE_COUNT][4];
    unsigned pipe_mask = 0;
    uint64_t index_size = (uint64_t)w * h * unit;
    uint64_t drive_size = (uint64_t)(h - 1) * pitch + bytes;
    g_autofree uint8_t *index_data = NULL;
    g_autofree uint8_t *drive_data = NULL;

    if (w != s->width || h != s->height || pitch < bytes || pitch > 16384 ||
        !b300_dma_valid(indexes, index_size) ||
        !b300_dma_valid(output, drive_size)) {
        goto complete;
    }

    for (unsigned p = 0; p < PIPE_COUNT; p++) {
        const uint32_t *pipe = &r[(EE_PIPE + p * 16) / 4];
        active[p] = (pipe[0] & 1) && b300_dma_valid(pipe[3], 1 << (bits * 2));
        if (active[p]) {
            trace_allwinner_b300_eink_pipe(p, pipe[0], pipe[1], pipe[2],
                                          pipe[3]);
            window[p][0] = pipe[1] & 0xfff;
            window[p][1] = (pipe[1] >> 16) & 0xfff;
            window[p][2] = MIN(pipe[2] & 0xfff, w - 1);
            window[p][3] = (pipe[2] >> 16) & 0xfff;
            pipe_mask |= BIT(p);
            if (address_space_read(as, pipe[3], MEMTXATTRS_UNSPECIFIED,
                                   waves[p], 1 << (bits * 2)) != MEMTX_OK) {
                goto complete;
            }
        }
    }

    /*
     * Transfer complete spans through the decoder's IOMMU context.  Row
     * padding is retained along with each word's timing/control pins.
     */
    index_data = g_malloc(index_size);
    drive_data = g_malloc(drive_size);
    if (address_space_read(as, indexes, MEMTXATTRS_UNSPECIFIED,
                           index_data, index_size) != MEMTX_OK ||
        address_space_read(as, output, MEMTXATTRS_UNSPECIFIED,
                           drive_data, drive_size) != MEMTX_OK) {
        goto complete;
    }
    for (unsigned y = 0; y < h; y++) {
        const uint8_t *index_row = index_data + (uint64_t)y * w * unit;
        uint8_t *drive_row = drive_data + (uint64_t)y * pitch;

        /* Leave timing/control pins intact, and default to no pixel drive. */
        for (unsigned g = 0; g < groups; g++) {
            if (wide) {
                stl_le_p(drive_row + g * 4,
                          ldl_le_p(drive_row + g * 4) & 0xff0000ff);
            } else {
                stw_le_p(drive_row + g * 2,
                          lduw_le_p(drive_row + g * 2) & 0x00ff);
            }
        }
        for (unsigned p = 0; p < PIPE_COUNT; p++) {
            unsigned x;

            if (!active[p] || y < window[p][1] || y > window[p][3]) {
                continue;
            }
            x = window[p][0];
            while (x <= window[p][2]) {
                unsigned g = x >> group_shift;
                unsigned last = MIN(window[p][2], x | (group - 1));
                unsigned shift = 8 + (x & (group - 1)) * 2;
                uint8_t *word = drive_row + g * word_size;
                uint32_t data = wide ? ldl_le_p(word) : lduw_le_p(word);

                for (; x <= last; x++, shift += 2) {
                    unsigned index = unit == 2 ?
                        lduw_le_p(index_row + x * 2) : index_row[x];
                    unsigned drive = waves[p][index & index_mask] & 3;

                    data = (data & ~(3u << shift)) | (drive << shift);
                }
                if (wide) {
                    stl_le_p(word, data);
                } else {
                    stw_le_p(word, data);
                }
            }
        }
    }
    if (address_space_write(as, output, MEMTXATTRS_UNSPECIFIED,
                            drive_data, drive_size) != MEMTX_OK) {
        goto complete;
    }
complete:
    trace_allwinner_b300_eink_decode(indexes, output, pipe_mask,
        (qemu_clock_get_ns(QEMU_CLOCK_REALTIME) - started) / 1000);
    s->eink[EE_DEC_CTL / 4] &= ~1;
    s->eink[EE_IRQ / 4] |= BIT(0);
    b300_eink_irq(s);
}

static bool b300_eink_enabled(AwB300EinkState *s)
{
    return (s->tcon[0] & BIT(31)) && (s->tcon[0x40 / 4] & BIT(31));
}

static void b300_eink_drive(AwB300EinkState *s)
{
    int64_t started = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    bool wide = s->eink[EDMA_CTL / 4] & BIT(4);
    unsigned group = wide ? 8 : 4, bytes = s->width / group * (wide ? 4 : 2);
    unsigned pitch = s->edma_active[1];
    unsigned x0 = s->edma_active[3] & 0xffff, y0 = s->edma_active[3] >> 16;
    hwaddr source = s->edma_active[0];
    hwaddr first = source + (uint64_t)y0 * pitch + x0 / 2;
    unsigned w = (s->edma_active[4] & 0xffff) + 1;
    unsigned h = (s->edma_active[4] >> 16) + 1;
    AddressSpace *as = aw_b300_iommu_as(s->iommu, 1);
    unsigned dark = 0, light = 0;
    uint8_t initial = s->panel[0];
    bool dirty = s->dirty;

    if (!s->edma_active_valid || !(s->eink[EDMA_CTL / 4] & 1) ||
        !(s->eink[EDMA_GLB_CTL / 4] & 1) ||
        w != s->width || h != s->height || pitch < x0 / 2 + bytes ||
        pitch > 16384 || !b300_dma_valid(first, (uint64_t)pitch * h)) {
        return;
    }

    uint64_t span = (uint64_t)(s->height - 1) * pitch + bytes;
    g_autofree uint8_t *scan = g_malloc(span);
    if (address_space_read(as, first, MEMTXATTRS_UNSPECIFIED, scan,
                           span) != MEMTX_OK) {
        return;
    }
    for (unsigned y = 0; y < s->height; y++) {
        const uint8_t *row = scan + (uint64_t)y * pitch;
        uint8_t *panel = s->panel + y * s->width;
        for (unsigned x = 0; x < s->width; x += group) {
            unsigned data = wide ? ldl_le_p(row + x / group * 4) >> 8 & 0xffff :
                                   lduw_le_p(row + x / group * 2) >> 8;
            if (!data) {
                continue;
            }
            dark += ctpop32(data & ~(data >> 1) & 0x5555);
            light += ctpop32((data >> 1) & ~data & 0x5555);
            for (unsigned i = 0; i < group; i++, data >>= 2) {
                unsigned drive = data & 3;
                uint8_t grey = panel[x + i];

                if (drive == 1) {
                    grey = MAX(0, (int)grey - 17);
                } else if (drive == 2) {
                    grey = MIN(255, (int)grey + 17);
                }
                dirty |= grey != panel[x + i];
                panel[x + i] = grey;
            }
        }
    }
    s->dirty = dirty;
    trace_allwinner_b300_eink_scan(source, dark, light, initial, s->panel[0],
        (qemu_clock_get_ns(QEMU_CLOCK_REALTIME) - started) / 1000);
}

static void b300_eink_latch(AwB300EinkState *s, unsigned phase)
{
    memcpy(s->edma_active, s->edma_pending, sizeof(s->edma_active));
    s->edma_ready = false;
    s->edma_active_valid = true;
    s->frame_latched = true;
    s->eink[EDMA_GLB_CTL / 4] &= ~BIT(16);
    trace_allwinner_b300_eink_dbuf(s->edma_active[0], phase);
}

static int64_t b300_eink_line_delay(AwB300EinkState *s)
{
    int64_t period = NANOSECONDS_PER_SECOND / s->refresh_rate;
    unsigned lines = MAX(2, ((s->tcon[0x50 / 4] >> 16) & 0x1fff) / 2);
    unsigned line = CLAMP((s->tcon[0x08 / 4] >> 16) & 0xfff, 1, lines - 1);

    return period * line / lines;
}

static void b300_eink_frame(void *opaque)
{
    AwB300EinkState *s = opaque;
    int64_t period = NANOSECONDS_PER_SECOND / s->refresh_rate;
    if (!b300_eink_enabled(s)) {
        return;
    }
    if (s->vblank) {
        /* Complete the physical scan before the IRQ releases its DMA buffer. */
        b300_eink_drive(s);
        s->frame_latched = false;
        if (s->edma_ready) {
            b300_eink_latch(s, 2);
        }
        s->frame_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->tcon[1] |= BIT(15);
    } else {
        /*
         * An empty shadow slot can be filled during blanking.  Once the frame
         * has latched, a later DBUF belongs to the following frame, including
         * when the driver prequeues two buffers before enabling TCON.
         */
        if (s->edma_ready && !s->frame_latched) {
            b300_eink_latch(s, 2);
        }
        s->tcon[1] |= BIT(13);
    }
    b300_eink_irq(s);
    s->vblank = !s->vblank;
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (s->vblank ? period - b300_eink_line_delay(s) :
                           b300_eink_line_delay(s)));
}

static uint64_t b300_eink_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_B300_EINK(opaque)->eink[offset / 4];
}

static void b300_eink_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    AwB300EinkState *s = opaque;
    if (offset == EE_IRQ) {
        s->eink[EE_IRQ / 4] = ((s->eink[EE_IRQ / 4] & 3) & ~value) |
                                 (value & BIT(4));
    } else {
        s->eink[offset / 4] = value;
    }
    /* A running command retains its register set while later frames configure. */
    if (offset == EE_IDX_GEN && (value & 1)) {
        memcpy(s->index_command, s->eink, sizeof(s->index_command));
        timer_mod(s->index_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
    } else if (offset == EE_DEC_CTL && (value & 1)) {
        unsigned pipes = 0;

        memcpy(s->decode_command, s->eink, sizeof(s->decode_command));
        for (unsigned p = 0; p < PIPE_COUNT; p++) {
            pipes |= (s->decode_command[(EE_PIPE + p * 16) / 4] & 1) << p;
        }
        trace_allwinner_b300_eink_kick(s->decode_command[EE_DEC_IDX / 4],
                                      s->decode_command[EE_DEC_WAV / 4], pipes);
        timer_mod(s->decode_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
    } else if (offset == EDMA_GLB_CTL && (value & BIT(16))) {
        memcpy(s->edma_pending, &s->eink[EDMA_WAV_ADDR / 4], sizeof(s->edma_pending));
        s->edma_ready = true;
        trace_allwinner_b300_eink_dbuf(s->edma_pending[0], 0);
        if (!b300_eink_enabled(s) && !s->edma_active_valid) {
            b300_eink_latch(s, 1);
        }
    }
    b300_eink_irq(s);
}

static uint64_t b300_tcon_read(void *opaque, hwaddr offset, unsigned size)
{
    AwB300EinkState *s = opaque;
    if (offset == 0xfc) {
        unsigned lines = ((s->tcon[0x50 / 4] >> 16) & 0x1fff) / 2;
        unsigned line = lines ? lines - 1 : s->height;
        if (b300_eink_enabled(s)) {
            int64_t period = NANOSECONDS_PER_SECOND / s->refresh_rate;
            int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->frame_start;
            line = (elapsed % period) * (uint64_t)MAX(lines, s->height) / period;
        }
        return (line & 0xfff) << 16;
    }
    return s->tcon[offset / 4];
}

static void b300_tcon_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    AwB300EinkState *s = opaque;
    bool was_enabled = b300_eink_enabled(s);
    if (offset == 4) {
        s->tcon[1] = (value & 0xffff0000) | (s->tcon[1] & value & 0xffff);
    } else {
        s->tcon[offset / 4] = value;
    }
    if (b300_eink_enabled(s)) {
        if (!timer_pending(s->frame_timer)) {
            s->vblank = false;
            s->frame_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            timer_mod(s->frame_timer, s->frame_start + b300_eink_line_delay(s));
        }
    } else {
        timer_del(s->frame_timer);
        if (was_enabled) {
            s->edma_active_valid = false;
            s->frame_latched = false;
            s->dirty = true;
        }
    }
    b300_eink_irq(s);
}

static uint64_t b300_de_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_B300_EINK(opaque)->de[offset / 4];
}

static void b300_de_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    AW_B300_EINK(opaque)->de[offset / 4] = value;
}

static uint64_t b300_top_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_B300_EINK(opaque)->top[offset / 4];
}

static void b300_top_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    AW_B300_EINK(opaque)->top[offset / 4] = value;
}

static uint64_t b300_tcon1_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_B300_EINK(opaque)->tcon1[offset / 4];
}

static void b300_tcon1_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    AwB300EinkState *s = opaque;
    if (offset == 4) {
        s->tcon1[1] = (value & 0xffff0000) | (s->tcon1[1] & value & 0xffff);
    } else {
        s->tcon1[offset / 4] = value;
    }
    b300_eink_irq(s);
}

#define B300_DISPLAY_OPS(name) \
static const MemoryRegionOps b300_##name##_ops = { \
    .read = b300_##name##_read, .write = b300_##name##_write, \
    .endianness = DEVICE_LITTLE_ENDIAN, \
    .valid = { .min_access_size = 4, .max_access_size = 4 }, \
}
B300_DISPLAY_OPS(de);
B300_DISPLAY_OPS(tcon);
B300_DISPLAY_OPS(eink);
B300_DISPLAY_OPS(top);
B300_DISPLAY_OPS(tcon1);

static void b300_eink_reset(DeviceState *dev)
{
    AwB300EinkState *s = AW_B300_EINK(dev);
    memset(s->de, 0, sizeof(s->de));
    memset(s->tcon, 0, sizeof(s->tcon));
    memset(s->tcon1, 0, sizeof(s->tcon1));
    memset(s->eink, 0, sizeof(s->eink));
    memset(s->top, 0, sizeof(s->top));
    memset(s->index_command, 0, sizeof(s->index_command));
    memset(s->decode_command, 0, sizeof(s->decode_command));
    /* The native driver seeds both its working image and index map with zero. */
    memset(s->panel, 0, s->panel_size);
    memset(s->edma_active, 0, sizeof(s->edma_active));
    memset(s->edma_pending, 0, sizeof(s->edma_pending));
    s->edma_ready = false;
    s->edma_active_valid = false;
    s->frame_latched = false;
    s->vblank = false;
    s->frame_start = 0;
    s->last_presented = 0;
    s->dirty = true;
    timer_del(s->frame_timer);
    timer_del(s->index_timer);
    timer_del(s->decode_timer);
    b300_eink_irq(s);
}

static int b300_eink_post_load(void *opaque, int version_id)
{
    AwB300EinkState *s = opaque;
    s->dirty = true;
    s->last_presented = 0;
    b300_eink_irq(s);
    return 0;
}

static const VMStateDescription b300_eink_vmstate = {
    .name = TYPE_AW_B300_EINK,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = b300_eink_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(de, AwB300EinkState, DE_SIZE / 4),
        VMSTATE_UINT32_ARRAY(tcon, AwB300EinkState, TCON_SIZE / 4),
        VMSTATE_UINT32_ARRAY(tcon1, AwB300EinkState, TCON_SIZE / 4),
        VMSTATE_UINT32_ARRAY(eink, AwB300EinkState, EINK_SIZE / 4),
        VMSTATE_UINT32_ARRAY(top, AwB300EinkState, TOP_SIZE / 4),
        VMSTATE_UINT32_ARRAY(index_command, AwB300EinkState, INDEX_REGS),
        VMSTATE_UINT32_ARRAY(decode_command, AwB300EinkState, DECODE_REGS),
        VMSTATE_UINT32_ARRAY(edma_active, AwB300EinkState, 5),
        VMSTATE_UINT32_ARRAY(edma_pending, AwB300EinkState, 5),
        VMSTATE_BOOL(edma_ready, AwB300EinkState),
        VMSTATE_BOOL(edma_active_valid, AwB300EinkState),
        VMSTATE_BOOL(frame_latched, AwB300EinkState),
        VMSTATE_BOOL(vblank, AwB300EinkState),
        VMSTATE_INT64(frame_start, AwB300EinkState),
        VMSTATE_VBUFFER_UINT32(panel, AwB300EinkState, 1, NULL, panel_size),
        VMSTATE_TIMER_PTR(frame_timer, AwB300EinkState),
        VMSTATE_TIMER_PTR(index_timer, AwB300EinkState),
        VMSTATE_TIMER_PTR(decode_timer, AwB300EinkState),
        VMSTATE_END_OF_LIST()
    },
};

static void b300_eink_realize(DeviceState *dev, Error **errp)
{
    AwB300EinkState *s = AW_B300_EINK(dev);
    if (!s->iommu) {
        error_setg(errp, "B300 EINK requires an IOMMU connection");
        return;
    }
    if (!s->width || !s->height || s->width > 4096 || s->height > 4096 ||
        s->width % 8 || s->height < 3 || s->width < 3 ||
        (s->rotation != 0 && s->rotation != 90 &&
         s->rotation != 180 && s->rotation != 270) ||
        (s->color_filter != 0 && s->color_filter != 3) ||
        !s->refresh_rate || s->refresh_rate > 240) {
        error_setg(errp, "B300 EINK requires valid panel geometry, rotation, color filter and refresh rate");
        return;
    }
    s->panel_size = s->width * s->height;
    s->panel = g_malloc(s->panel_size);
    s->console = graphic_console_init(dev, 0, &b300_eink_graphics, s);
    bool portrait = s->rotation == 90 || s->rotation == 270;
    qemu_console_resize(s->console, portrait ? s->height : s->width,
                         portrait ? s->width : s->height);
}

static void b300_eink_init(Object *obj)
{
    AwB300EinkState *s = AW_B300_EINK(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(s);
    memory_region_init_io(&s->de_mr, obj, &b300_de_ops, s, "b300.de", DE_SIZE);
    memory_region_init_io(&s->tcon_mr, obj, &b300_tcon_ops, s, "b300.tcon", TCON_SIZE);
    memory_region_init_io(&s->eink_mr, obj, &b300_eink_ops, s, "b300.eink", EINK_SIZE);
    memory_region_init_io(&s->top_mr, obj, &b300_top_ops, s, "b300.tcon-top", TOP_SIZE);
    memory_region_init_io(&s->tcon1_mr, obj, &b300_tcon1_ops, s, "b300.tcon1", TCON_SIZE);
    sysbus_init_mmio(dev, &s->de_mr);
    sysbus_init_mmio(dev, &s->tcon_mr);
    sysbus_init_mmio(dev, &s->eink_mr);
    sysbus_init_mmio(dev, &s->top_mr);
    sysbus_init_mmio(dev, &s->tcon1_mr);
    sysbus_init_irq(dev, &s->de_irq);
    sysbus_init_irq(dev, &s->tcon_irq);
    sysbus_init_irq(dev, &s->eink_irq);
    sysbus_init_irq(dev, &s->tcon1_irq);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, b300_eink_frame, s);
    s->index_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, b300_eink_index_done, s);
    s->decode_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, b300_eink_decode_done, s);
}

static void b300_eink_finalize(Object *obj)
{
    AwB300EinkState *s = AW_B300_EINK(obj);
    timer_free(s->frame_timer);
    timer_free(s->index_timer);
    timer_free(s->decode_timer);
    g_free(s->panel);
}

static const Property b300_eink_properties[] = {
    DEFINE_PROP_LINK("iommu", AwB300EinkState, iommu,
                     TYPE_AW_B300_IOMMU, AWB300IOMMUState *),
    DEFINE_PROP_UINT32("width", AwB300EinkState, width, 0),
    DEFINE_PROP_UINT32("height", AwB300EinkState, height, 0),
    DEFINE_PROP_UINT32("rotation", AwB300EinkState, rotation, 0),
    DEFINE_PROP_UINT32("color-filter", AwB300EinkState, color_filter, 3),
    DEFINE_PROP_UINT32("refresh-rate", AwB300EinkState, refresh_rate, 85),
};

static void b300_eink_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = b300_eink_realize;
    dc->vmsd = &b300_eink_vmstate;
    device_class_set_legacy_reset(dc, b300_eink_reset);
    device_class_set_props(dc, b300_eink_properties);
}

static const TypeInfo b300_eink_type = {
    .name = TYPE_AW_B300_EINK,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AwB300EinkState),
    .instance_init = b300_eink_init,
    .instance_finalize = b300_eink_finalize,
    .class_init = b300_eink_class_init,
};

static void b300_eink_register_types(void)
{
    type_register_static(&b300_eink_type);
}
type_init(b300_eink_register_types)
