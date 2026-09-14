/*
 * MT8171 EPD and DISP control register banks.
 *
 * Source: hwtcon/platform/mt8171/hwtcon_reg.h and the hwtcon HAL. IRQ
 * status is read-only except for the documented zero-clear/pulse-clear
 * interfaces. Full-buffer LUT clearing performs actual M4U-routed DMA.
 * Normal uncompressed 24-bit updates perform image and working-buffer DMA.
 * Shadow-latched 5+5-bit waveform jobs decode real WB/table DMA into a
 * numerical drive raster and release only after their configured frame count.
 * Monochrome and CSR2 REGAL use a numerical, uncalibrated image/WB transform.
 * Other waveform formats remain incomplete, without invented DMA.
 *
 * The first 4 KiB of each syscon belongs to the board's clock controller.
 * The two MMIO windows begin at EPD +0x1000 and DISP +0x1000 respectively.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/display/mt8171-hwtcon.h"
#include "hw/core/irq.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "system/address-spaces.h"

#define R(s, a) ((s)->bank[0].regs[(a) / 4])
static const uint16_t release_status[8] = {
    0x4dd8, 0x4ddc, 0x40b8, 0x40bc, 0x43d8, 0x43dc, 0x43e8, 0x43ec,
};
static const uint16_t end_status[8] = {
    0x4d58, 0x4d5c, 0x4df8, 0x4dfc, 0x4ed8, 0x4edc, 0x4ee8, 0x4eec,
};

static void update_irq(MT8171HWTCONState *s)
{
    uint32_t release = 0, end = 0;
    uint32_t paper = R(s, 0xd02c), en = R(s, 0xd044);
    for (unsigned i = 0; i < 8; i++) {
        release |= R(s, release_status[i]);
        end |= R(s, end_status[i]);
    }
    qemu_set_irq(s->irq[0], !!(R(s, 0xa000) & R(s, 0xa004)));
    qemu_set_irq(s->irq[1], !!(R(s, 0x4004) & R(s, 0x4008)));
    qemu_set_irq(s->irq[2], release != 0 && (R(s, 0x4024) & BIT(25)));
    qemu_set_irq(s->irq[3], !!((paper & BIT(9)) && (en & BIT(2))));
    qemu_set_irq(s->irq[4], !!((paper & BIT(10)) && (en & BIT(5))));
    qemu_set_irq(s->irq[5], !!((paper & BIT(5)) && (en & BIT(6))));
    qemu_set_irq(s->irq[6], !!(R(s, 0x6008) & R(s, 0x600c)));
    qemu_set_irq(s->irq[7], end != 0 && !(R(s, 0x4d0c) & BIT(15)));
    qemu_set_irq(s->irq[8], !!(s->bank[1].regs[0x6000 / 4] &
                            s->bank[1].regs[0x6004 / 4]));
}

/* Larb8 ports 10 and 11 carry working-buffer reads/writes. NONSEC_CON
 * selects the M4U context; secure SEC_CON controls MMU enable. Never infer bypass from
 * address value. The walker preserves MT8171's 35-bit physical addresses. */
static bool fabric_dma(MT8171HWTCONState *s, uint32_t *larb, unsigned master,
                       unsigned port, hwaddr addr, void *buffer,
                       size_t bytes, bool write)
{
    uint8_t *p = buffer;
    if (!s->m4u || !larb) {
        qemu_log_mask(LOG_GUEST_ERROR, "mt8171-hwtcon: WB DMA fabric not wired\n");
        return false;
    }
    uint32_t con = larb[0x380 / 4 + port];
    bool mmu_enabled = larb[0xf80 / 4 + port] & 1;
    while (bytes) {
        hwaddr pa = addr;
        size_t chunk = MIN(bytes, 4096 - (addr & 4095));
        if (mmu_enabled) {
            unsigned bank = (con >> (8 + 2 * ((addr >> 32) & 3))) & 3;
            size_t span;
            if (!mt8171_m4u_translate(s->m4u, bank,
                                     (master << 10) | (port << 2),
                                     addr, write, &pa, &span)) {
                s->dma_faults++;
                qemu_log_mask(LOG_GUEST_ERROR,
                              "mt8171-hwtcon: M4U fault master%u port%u bank%u "
                              "IOVA=%" HWADDR_PRIx " write=%u\n",
                              master, port, bank, addr, write);
                return false;
            }
            chunk = MIN(chunk, span);
        }
        if (address_space_rw(&address_space_memory, pa,
                             MEMTXATTRS_UNSPECIFIED, p, chunk, write) != MEMTX_OK) {
            s->dma_faults++;
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt8171-hwtcon: WB DMA fault at %" HWADDR_PRIx "\n", pa);
            return false;
        }
        addr += chunk; p += chunk; bytes -= chunk;
    }
    return true;
}

static bool wb_dma(MT8171HWTCONState *s, hwaddr addr, void *buffer,
                   size_t bytes, bool write)
{
    return fabric_dma(s, s->larb8_regs, 5, write ? 11 : 10,
                      addr, buffer, bytes, write);
}

static const uint16_t active_offset[8] = {
    0xd0d0, 0xd0d4, 0xd1a0, 0xd1a4, 0xd1a8, 0xd1ac, 0xd1b0, 0xd1b4,
};
static const uint16_t collision_offset[8] = {
    0xd0d8, 0xd0dc, 0xd1c0, 0xd1c4, 0xd1c8, 0xd1cc, 0xd1d0, 0xd1d4,
};

/* A histogram is a presence bitmap, not a pixel count; the vendor's
 * pipeline_get_histogram_24 returns eight words for 256 possible values. */
static unsigned hist_offset(bool next, unsigned value)
{
    unsigned word = value / 32;
    return word ? (next ? 0xd170 : 0xd150) + (word - 1) * 4 :
                  (next ? 0xd088 : 0xd084);
}

/* pipeline_config_24_bits_all_format encodes 8/4/5 valid image bits as
 * 0/1/2 in ACEP_REGAL_FLAG[19:16]. PA6 normal input is Y4 in a byte-wide
 * image plane; REGAL output is Y5, and CFA uses the complete byte. The
 * source Y4/Y5 levels are left aligned (normal endpoints 0x00/0xf0 and
 * REGAL steps of eight in hwtcon_table.c). These are precision controls,
 * not packed-pixel strides. WB previous/current remain full-byte fields.
 * Bit11 selects REGAL processing and does not alter the storage layout. */
bool mt8171_hwtcon_image_mask(uint32_t format, uint8_t *mask)
{
    if ((format & ~(BIT(11) | 0x000f0000U)) != 1) {
        return false;
    }
    switch ((format >> 16) & 15) {
    case 0: *mask = 0xff; return true;
    case 1: *mask = 0xf0; return true;
    case 2: *mask = 0xf8; return true;
    default: return false;
    }
}

/* Normal {current,previous,LUT} byte transition. The CSR preparation path
 * hwtcon_mdp_csr_handle runs the REGAL-to-normal LUT on RGB24's B channel
 * (byte0) only, providing byte-order evidence for current at byte0. The pipeline
 * header defines KEEP_PRE/LUT_OVERRIDE/CUR2PRE and the programmed mapping
 * converts former REGAL working-buffer values back into ordinary values.
 * Compressed fields, REGAL processing, and interruptible stylus are left
 * unsupported rather than interpreting them as this ordinary update. */
static bool update_working_buffer(MT8171HWTCONState *s, uint32_t trigger)
{
    uint32_t flags = R(s, 0xdf0c);
    uint32_t allowed = BIT(0) | BIT(1) | BIT(2) | BIT(4) | BIT(5) | BIT(15) |
                       BIT(20) | BIT(21) | BIT(22);
    bool clear = flags & BIT(15);
    unsigned width = R(s, 0xd00c) & 0x3fff;
    unsigned height = (R(s, 0xd00c) >> 16) & 0x3fff;
    unsigned x0 = R(s, 0xd020) & 0x3fff;
    unsigned y0 = (R(s, 0xd020) >> 16) & 0x3fff;
    unsigned w = R(s, 0xd024) & 0x3fff;
    unsigned h = (R(s, 0xd024) >> 16) & 0x3fff;
    unsigned image_pitch = R(s, 0xd060) & BIT(6) ?
                           R(s, 0xd064) & 0xffff : width;
    unsigned wb_pitch = R(s, 0xd060) & BIT(7) ?
                        R(s, 0xd064) >> 16 : width * 3;
    unsigned id = (trigger >> 4) & 255;
    unsigned xmin = width, ymin = height, xmax = 0, ymax = 0;
    unsigned cxmin = width, cymin = height, cxmax = 0, cymax = 0;
    uint64_t assigned = 0, collisions = 0;
    uint32_t status = 0;
    uint8_t image_mask;
    g_autofree uint8_t *wb = NULL;
    g_autofree uint8_t *image = NULL;

    /* REGAL's bit11 survives subsequent format writes; PIPELINE_FLAG bit3
     * selects that processing path. It does not change normal WB byte size. */
    if (!mt8171_hwtcon_image_mask(R(s, 0xdf10), &image_mask) ||
        (flags & ~allowed) || ((flags & BIT(1)) && image_mask != 0xf8)) {
        return false;
    }
    if (!width || !height || width > 8192 || height > 8192 || !w || !h ||
        x0 + w > width || y0 + h > height || image_pitch < width ||
        wb_pitch < width * 3 || (uint64_t)wb_pitch * height > 64 * 1024 * 1024 ||
        !(R(s, 0x8010) & 1) || !(R(s, 0x9010) & 1) || !(R(s, 0xa008) & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "mt8171-hwtcon: invalid update geometry/enables\n");
        return false;
    }
    if (id == 255 || (R(s, active_offset[id / 32]) & BIT(id % 32))) {
        R(s, 0xd02c) |= id == 255 ? BIT(0) : BIT(3);
        update_irq(s);
        return false;
    }
    for (unsigned i = 0; i < 8; i++) {
        R(s, collision_offset[i]) = 0;
        R(s, hist_offset(false, i * 32)) = 0;
        R(s, hist_offset(true, i * 32)) = 0;
    }
    /* CLEAR composes with normal full/partial updates when the driver's LUT
     * allocator reuses released IDs. Scan the entire WB to free inactive
     * IDs, then apply the requested image only inside its update rectangle.
     * A standalone clear has a zero-sized rectangle and is handled below. */
    unsigned scan_x = clear ? 0 : x0;
    unsigned scan_width = clear ? width : w;
    unsigned scan_y = clear ? 0 : y0;
    unsigned scan_end = clear ? height : y0 + h;
    wb = g_malloc(scan_width * 3);
    image = g_malloc(w);
    for (unsigned y = scan_y; y < scan_end; y++) {
        bool image_row = y >= y0 && y < y0 + h;
        hwaddr wb_offset = (uint64_t)y * wb_pitch + scan_x * 3;
        if (!wb_dma(s, R(s, 0xdf04) + wb_offset,
                    wb, scan_width * 3, false) ||
            (image_row && !fabric_dma(s, s->larb3_regs, 4, 10,
                        R(s, 0xdf08) + (uint64_t)y * image_pitch + x0,
                        image, w, false))) {
            return false;
        }
        for (unsigned x = 0; x < scan_width; x++) {
            uint8_t *p = wb + x * 3;
            unsigned old_id = p[2], previous = p[1], current = p[0];
            unsigned panel_x = scan_x + x;
            if (clear && !(R(s, active_offset[old_id / 32]) &
                           BIT(old_id % 32))) {
                p[2] = old_id = 255;
                if (flags & BIT(21)) {
                    p[1] = previous = current;
                }
            }
            if (!image_row || panel_x < x0 || panel_x >= x0 + w) {
                continue;
            }
            unsigned next = image[panel_x - x0];
            if ((R(s, 0xdf14) & BIT(16)) && next == (R(s, 0xdf14) & 255)) {
                continue; /* Source-programmed REGAL no-update code. */
            }
            next &= image_mask;
            if (flags & BIT(20)) {
                current = (R(s, 0xd300 + (current / 4) * 4) >>
                           ((current % 4) * 8)) & 255;
                previous = (R(s, 0xd300 + (previous / 4) * 4) >>
                            ((previous % 4) * 8)) & 255;
            }
            if (R(s, 0xd070) & 1) {
                R(s, hist_offset(false, current)) |= BIT(current % 32);
                R(s, hist_offset(true, next)) |= BIT(next % 32);
            }
            if (old_id == id) { status |= BIT(5); }
            if (!(flags & BIT(0)) && next == current) { continue; }
            if (R(s, active_offset[old_id / 32]) & BIT(old_id % 32)) {
                R(s, collision_offset[old_id / 32]) |= BIT(old_id % 32);
                cxmin = MIN(cxmin, panel_x); cxmax = MAX(cxmax, panel_x);
                cymin = MIN(cymin, y); cymax = MAX(cymax, y);
                collisions++;
                continue;
            }
            p[2] = id;
            if (!(flags & BIT(2))) {
                p[1] = flags & BIT(4) ? previous : current;
                p[0] = next;
            } else if (flags & BIT(22)) {
                p[1] = p[0];
            }
            xmin = MIN(xmin, panel_x); xmax = MAX(xmax, panel_x);
            ymin = MIN(ymin, y); ymax = MAX(ymax, y);
            assigned++;
        }
        if (!(flags & BIT(5)) &&
            !wb_dma(s, R(s, 0xdf00) + wb_offset,
                    wb, scan_width * 3, true)) {
            return false;
        }
    }
    R(s, 0xd0e0) = assigned ? xmin | ymin << 16 : 0;
    R(s, 0xd0e4) = assigned ? (xmax - xmin + 1) | (ymax - ymin + 1) << 16 : 0;
    R(s, 0xd0e8) = collisions ? cxmin | cymin << 16 : 0;
    R(s, 0xd0ec) = collisions ? (cxmax - cxmin + 1) | (cymax - cymin + 1) << 16 : 0;
    R(s, 0xd034) = assigned ? 0 : BIT(0);
    R(s, 0xd02c) |= status | BIT(2) | BIT(6) | BIT(7) | BIT(8);
    R(s, 0xa004) |= 1;
    if (!(flags & BIT(5))) {
        s->normal_writebacks++;
        s->clear_writebacks += clear;
    }
    qemu_log_mask(LOG_UNIMP,
                  "mt8171-hwtcon: WB update id=%u pixels=%" PRIu64
                  " collisions=%" PRIu64 " image=%08x wb=%08x clear=%u mask=%02x\n",
                  id, assigned, collisions, R(s, 0xdf08), R(s, 0xdf00), clear,
                  image_mask);
    update_irq(s);
    qemu_irq_pulse(s->event[0]);
    return true;
}

/* pipeline_config.h specifies CLEAR as replacing the LUT of non-active
 * pixels, and CLR_PRE_EN as {cur,cur,free_ID}. Uncompressed 24-bit bytes
 * follow the CSR gamma evidence above; the older 16-bit 5/5/6 bit packing
 * remains an inference. Other compressed
 * 24-bit formats and REGAL/stylus operations are not treated as copies. */
static bool clear_working_buffer(MT8171HWTCONState *s)
{
    uint32_t flags = R(s, 0xdf0c), acep = R(s, 0xdf10);
    bool mode24 = acep & 1, changed = false;
    unsigned bpp = mode24 ? 3 : 2, lut_bits = mode24 ? 8 : 6;
    unsigned value_bits = mode24 ? 8 : 5;
    uint32_t lut_mask = (1U << lut_bits) - 1;
    uint32_t prev_mask = ((1U << value_bits) - 1) << lut_bits;
    unsigned width = R(s, 0xd00c) & 0x3fff;
    unsigned height = (R(s, 0xd00c) >> 16) & 0x3fff;
    unsigned pitch = R(s, 0xd060) & BIT(7) ? R(s, 0xd064) >> 16 : width * bpp;
    hwaddr src = R(s, 0xdf04), dst = R(s, 0xdf00);
    uint32_t allowed = BIT(0) | BIT(1) | BIT(5) | BIT(15) | BIT(21);
    g_autofree uint8_t *row = NULL;

    if (R(s, 0xd024) ||
        (flags & (BIT(0) | BIT(15))) != (BIT(0) | BIT(15)) ||
        (flags & ~allowed) || (mode24 && (acep & 0x0fff0000))) {
        return false;
    }
    if (!width || !height || width > 8192 || height > 8192 ||
        pitch < width * bpp || (uint64_t)pitch * height > 64 * 1024 * 1024 ||
        !(R(s, 0xa008) & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "mt8171-hwtcon: invalid WB clear geometry\n");
        return false;
    }
    row = g_malloc(width * bpp);
    for (unsigned y = 0; y < height; y++) {
        if (!wb_dma(s, src + (uint64_t)y * pitch, row, width * bpp, false)) {
            return false;
        }
        for (unsigned x = 0; x < width; x++) {
            uint8_t *p = row + x * bpp;
            if (mode24) {
                unsigned lut = p[2];
                if (!(R(s, active_offset[lut / 32]) & BIT(lut % 32))) {
                    changed |= p[2] != 255;
                    p[2] = 255;
                    if (flags & BIT(21)) {
                        changed |= p[1] != p[0];
                        p[1] = p[0];
                    }
                }
                continue;
            }
            uint32_t pixel = p[0] | (uint32_t)p[1] << 8;
            if (mode24) { pixel |= (uint32_t)p[2] << 16; }
            unsigned lut = pixel & lut_mask;
            if (!(R(s, active_offset[lut / 32]) & BIT(lut % 32))) {
                uint32_t out = (pixel & ~lut_mask) | lut_mask;
                if (flags & BIT(21)) {
                    out = (out & ~prev_mask) | ((pixel >> value_bits) & prev_mask);
                }
                changed |= out != pixel;
                p[0] = out; p[1] = out >> 8;
                if (mode24) { p[2] = out >> 16; }
            }
        }
        if (!(flags & BIT(5)) &&
            !wb_dma(s, dst + (uint64_t)y * pitch, row, width * bpp, true)) {
            return false;
        }
    }
    if (!(flags & BIT(5))) { s->clear_writebacks++; }
    R(s, 0xd034) = (changed ? BIT(1) : 0) | BIT(0);
    R(s, 0xd02c) |= BIT(2) | BIT(6) | BIT(8);
    R(s, 0xa004) |= 1;
    update_irq(s);
    qemu_irq_pulse(s->event[0]);
    return true;
}

/* Exact semantic decoding from hwtcon_table.c's regal_table. The low three
 * bits, including CFA's color-content flag, survive endpoint aliases. */
static uint8_t regal_normal_value(uint8_t value)
{
    unsigned base = value & 0xf8;
    if (base == 0x08 || base == 0x18 || base == 0x28) { return value & 7; }
    if (base == 0xe8 || base == 0xf8) { return 0xf0 | (value & 7); }
    return value;
}

/* Numerical REGAL contract, not a reconstruction of the proprietary spatial
 * ghost-compensation algorithm. Stock hwtcon_regal_config.c programs 32
 * output levels, opaque masks/coefficient words and the processing mode. We use
 * nearest-level quantization (ties choose the lower level), retain CFA bit0
 * only for the full-byte color path, and mask PA6's input to its Y4 precision,
 * and compare decoded current WB against the new level. Unchanged pixels
 * emit the driver's exact 0xd9 sentinel; changed pixels emit the selected
 * programmed level. WB history and the real DMA buffers determine results.
 * Opaque SFT/SIT thresholds, spatial masks and physical ghosting are not
 * calibrated here. Unsupported formats/modes never complete through a copy.
 *
 * pipeline_trigger_regal allocates a dense width*height 8-bit temporary image.
 * The WB input retains its 24-bit/padded pitch; the output is not a WB copy.
 */
static bool numerical_regal(MT8171HWTCONState *s)
{
    unsigned width = R(s, 0xd00c) & 0x3fff;
    unsigned height = (R(s, 0xd00c) >> 16) & 0x3fff;
    unsigned image_pitch = R(s, 0xd060) & BIT(6) ?
                           R(s, 0xd064) & 0xffff : width;
    unsigned wb_pitch = R(s, 0xd060) & BIT(7) ?
                        R(s, 0xd064) >> 16 : width * 3;
    uint32_t csr = R(s, 0xc0d4);
    uint8_t levels[32], quantized[256], image_mask;
    uint64_t changed = 0, unchanged = 0;
    g_autofree uint8_t *wb = NULL, *image = NULL, *output = NULL;
    if (R(s, 0xdf0c) != (BIT(0) | BIT(3)) ||
        !(R(s, 0xdf10) & BIT(11)) ||
        !mt8171_hwtcon_image_mask(R(s, 0xdf10), &image_mask) ||
        (image_mask != 0xff && image_mask != 0xf0) ||
        (image_mask == 0xf0 ? csr != 0xd800 :
         (csr != 0xd808 && csr != 0xd818 && csr != 0xd928 && csr != 0xd938))) {
        return false;
    }
    if (!width || !height || width > 8192 || height > 8192 ||
        image_pitch < width || wb_pitch < width * 3 ||
        (uint64_t)wb_pitch * height > 64 * 1024 * 1024 ||
        R(s, 0xd020) || R(s, 0xd024) != R(s, 0xd00c) ||
        R(s, 0xc00c) != width - 1 || R(s, 0xc010) != height - 1 ||
        !(R(s, 0xc000) & 1) || !(R(s, 0x8010) & 1) ||
        !(R(s, 0x9010) & 1) || !(R(s, 0xa008) & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "mt8171-hwtcon: invalid REGAL geometry/enables\n");
        return false;
    }
    for (unsigned i = 0; i < 32; i++) {
        levels[i] = R(s, 0xc0a4 + (i / 4) * 4) >> ((i % 4) * 8);
        if ((levels[i] & 7) || (i && levels[i] <= levels[i - 1])) {
            qemu_log_mask(LOG_UNIMP, "mt8171-hwtcon: unsupported REGAL quantization table\n");
            return false;
        }
    }
    for (unsigned v = 0; v < 256; v++) {
        unsigned target = regal_normal_value(v & image_mask) & 0xfe;
        unsigned color_flag = image_mask == 0xff ? v & 1 : 0;
        unsigned best = 0, distance = 256;
        for (unsigned i = 0; i < 32; i++) {
            /* 0xd9 is a control token, never a changed-pixel output. */
            if ((levels[i] | color_flag) == 0xd9) { continue; }
            unsigned d = abs((int)levels[i] - (int)target);
            if (d < distance) { distance = d; best = i; }
        }
        quantized[v] = levels[best] | color_flag;
    }
    wb = g_malloc(width * 3);
    image = g_malloc(width);
    output = g_malloc(width);
    for (unsigned y = 0; y < height; y++) {
        if (!wb_dma(s, (hwaddr)R(s, 0xdf04) + (uint64_t)y * wb_pitch,
                    wb, width * 3, false) ||
            !fabric_dma(s, s->larb3_regs, 4, 10,
                        (hwaddr)R(s, 0xdf08) + (uint64_t)y * image_pitch,
                        image, width, false)) {
            return false;
        }
        for (unsigned x = 0; x < width; x++) {
            uint8_t value = quantized[image[x]];
            if (regal_normal_value(wb[x * 3]) == regal_normal_value(value)) {
                output[x] = 0xd9;
                unchanged++;
            } else {
                output[x] = value;
                changed++;
            }
        }
        if (!wb_dma(s, (hwaddr)R(s, 0xdf00) + (uint64_t)y * width,
                    output, width, true)) {
            return false;
        }
    }
    s->numerical_regal_jobs++;
    R(s, 0xd034) = changed ? 0 : BIT(0);
    R(s, 0xd02c) |= BIT(2) | BIT(6) | BIT(7) | BIT(8);
    R(s, 0xa004) |= 1;
    qemu_log_mask(LOG_UNIMP,
                  "mt8171-hwtcon: numerical REGAL csr=%x changed=%" PRIu64
                  " unchanged=%" PRIu64 " image=%08x wb=%08x output=%08x\n",
                  csr, changed, unchanged, R(s, 0xdf08), R(s, 0xdf04), R(s, 0xdf00));
    update_irq(s);
    qemu_irq_pulse(s->event[0]);
    return true;
}

/* The stock job submission is SHADOW_UP, not WF_LUT_TRIG. An enabled
 * shadow job scans its WB pixels for each waveform frame. This first decoder
 * supports the observed single-layer uncompressed 24-bit, 5+5-bit waveform
 * format. Nibble ordering/8-to-5 quantization follow the synthetic waveform
 * convention; the resulting drive raster is not a calibrated optical model.
 * Unknown formats and failed DMA retain their active job, with no release. */
static bool waveform_frame(MT8171HWTCONState *s, unsigned id)
{
    MT8171WaveJob *j = &s->job[id];
    unsigned panel_width = R(s, 0xd00c) & 0x3fff;
    unsigned panel_height = (R(s, 0xd00c) >> 16) & 0x3fff;
    unsigned group = id / 16;
    /* ADDR12 resumes at+f50, leaving+f40..4c reserved. LEN is contiguous. */
    unsigned address_reg = 0x10f10 + j->slot * 4 + (j->slot >= 12 ? 0x10 : 0);
    unsigned format = (R(s, 0x10df4) >> (group * 2)) & 3;
    unsigned format_bits = (R(s, 0x10df8) >> (format * 8)) & 255;
    unsigned pitch = R(s, 0x4044) & 0xffff;
    uint32_t con = R(s, 0x4030);
    uint8_t table[512];
    uint32_t *table_larb = group & 1 ? s->larb8_regs : s->larb3_regs;
    unsigned table_master = group & 1 ? 5 : 4;
    g_autofree uint8_t *row = NULL;

    if (!panel_width || !panel_height || panel_width > 8192 ||
        panel_height > 8192 || (uint64_t)panel_width * panel_height > 32 * 1024 * 1024 ||
        !j->width || !j->height || j->x + j->width > panel_width ||
        j->y + j->height > panel_height || R(s, 0x402c) != 1 ||
        (con & 0x1f000) != 0x1000 || (con & (BIT(9) | BIT(10))) ||
        R(s, 0x403c) || pitch < panel_width * 3 || format_bits != 0x55 ||
        (R(s, 0x10df0) & 0x1f0) || !(R(s, 0x10d0c) & BIT(13))) {
        qemu_log_mask(LOG_UNIMP,
                      "mt8171-hwtcon: waveform id%u unsupported layer/format "
                      "src=%08x con=%08x format=%02x map=%08x\n",
                      id, R(s, 0x402c), con, format_bits, R(s, 0x10df0));
        return false;
    }
    if (!fabric_dma(s, table_larb, table_master, group / 2,
                    (hwaddr)R(s, address_reg) + j->frame * sizeof(table),
                    table, sizeof(table), false)) {
        return false;
    }
    size_t raster_size = (size_t)panel_width * panel_height;
    if (s->drive_frame_size != raster_size) {
        s->drive_frame = g_realloc(s->drive_frame, raster_size);
        s->drive_accumulator = g_realloc(s->drive_accumulator, raster_size * sizeof(int16_t));
        memset(s->drive_accumulator, 0, raster_size * sizeof(int16_t));
        memset(s->drive_frame, 0, raster_size);
        s->drive_frame_size = raster_size;
    }
    row = g_malloc(j->width * 3);
    for (unsigned y = j->y; y < j->y + j->height; y++) {
        if (!fabric_dma(s, s->larb3_regs, 4, 8,
                        (hwaddr)R(s, 0x4f40) + (uint64_t)y * pitch + j->x * 3,
                        row, j->width * 3, false)) {
            return false;
        }
        for (unsigned x = 0; x < j->width; x++) {
            uint8_t *p = row + x * 3;
            if (p[2] != id) { continue; }
            unsigned previous = p[1] >> 3, current = p[0] >> 3;
            unsigned index = previous * 32 + current;
            uint8_t drive = (table[index / 2] >> ((index & 1) * 4)) & 15;
            size_t pixel = (size_t)y * panel_width + j->x + x;
            s->drive_frame[pixel] = drive;
            /* Synthetic numerical convention: 0 neutral, 1 positive,
             * 2 negative. Preserve other source symbols in drive_frame. */
            int delta = drive == 1 ? 1 : drive == 2 ? -1 : 0;
            s->drive_accumulator[pixel] = CLAMP(
                s->drive_accumulator[pixel] + delta, INT16_MIN, INT16_MAX);
            s->drive_pixels++;
        }
    }
    s->completed_frames++;
    return true;
}

static void waveform_schedule(MT8171HWTCONState *s)
{
    if (!(R(s, 0x400c) & 1) || !(R(s, 0x6000) & 1) ||
        timer_pending(s->frame_timer)) { return; }
    for (unsigned id = 0; id < 255; id++) {
        if (s->job[id].active) {
            /* Nominal 60 Hz frame cadence, not a reconstructed pixel PLL. */
            timer_mod(s->frame_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 16666667);
            return;
        }
    }
}

static void waveform_tick(void *opaque)
{
    MT8171HWTCONState *s = opaque;
    bool frame_done = false, pending = false;
    if (!(R(s, 0x400c) & 1) || !(R(s, 0x6000) & 1)) { return; }
    for (unsigned id = 0; id < 255; id++) {
        MT8171WaveJob *j = &s->job[id];
        if (!j->active) { continue; }
        if (!waveform_frame(s, id)) {
            /* Fault leaves state inspectable; a later submission/control
             * write can restart the scan after the DMA configuration changes. */
            continue;
        }
        frame_done = true;
        if (++j->frame == j->frames) {
            j->active = false;
            s->completed_waveforms++;
            R(s, 0x4270 + (id / 16) * 4) &= ~BIT(id % 16);
            R(s, release_status[id / 32]) |= BIT(id % 32);
            R(s, end_status[id / 32]) |= BIT(id % 32);
            qemu_log_mask(LOG_UNIMP,
                          "mt8171-hwtcon: waveform id%u completed %u DMA frames\n",
                          id, j->frames);
        } else {
            pending = true;
        }
    }
    if (frame_done) {
        R(s, 0x4008) |= BIT(1); /* wf_lut_inten: frame complete */
        R(s, 0x600c) |= BIT(0);
        R(s, 0xd02c) |= BIT(9) | BIT(10);
        qemu_irq_pulse(s->event[1]);
        qemu_irq_pulse(s->event[2]);
    }
    update_irq(s);
    if (pending) { waveform_schedule(s); }
}

static void waveform_shadow(MT8171HWTCONState *s)
{
    unsigned id = (R(s, 0x4dc8) >> 8) & 255;
    unsigned slot = R(s, 0x4dc8) & 255;
    if (id == 255 || slot >= 32 || s->job[id].active) { return; }
    unsigned frames = R(s, 0x10d10 + slot * 4);
    if (!frames || frames > 4096) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mt8171-hwtcon: invalid waveform slot%u length%u\n", slot, frames);
        return;
    }
    s->job[id] = (MT8171WaveJob) {
        .x = (R(s, 0x4dc0) >> 16) & 0x3fff,
        .y = R(s, 0x4dc0) & 0x3fff,
        .width = (R(s, 0x4dc4) >> 16) & 0x3fff,
        .height = R(s, 0x4dc4) & 0x3fff,
        .slot = slot, .frames = frames, .active = true,
    };
    s->waveform_jobs++;
    R(s, 0x4270 + (id / 16) * 4) |= BIT(id % 16);
    waveform_schedule(s);
}

static bool status_register(unsigned bank, hwaddr a)
{
    if (bank == 0) {
        if (a == 0x4000 || a == 0xd02c || a == 0xd034 || a == 0xd090 ||
            (a >= 0x4270 && a <= 0x42ac)) {
            return true;
        }
        for (unsigned i = 0; i < 8; i++) {
            if (a == release_status[i] || a == end_status[i]) { return true; }
        }
    }
    return false;
}

static uint64_t read_reg(void *opaque, hwaddr offset, unsigned size)
{
    MT8171HWTCONBank *b = opaque;
    return b->regs[(offset + 0x1000) / 4];
}

static void write_reg(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    MT8171HWTCONBank *b = opaque;
    MT8171HWTCONState *s = b->owner;
    hwaddr a = offset + 0x1000;
    uint32_t old = b->regs[a / 4];

    if (status_register(b->index, a)) { return; }
    if (b->index == 0) {
        for (unsigned i = 0; i < 8; i++) {
            if (a == release_status[i] - 8 || a == end_status[i] - 8) {
                b->regs[(a + 8) / 4] &= ~value;
                b->regs[a / 4] = value;
                update_irq(s);
                return;
            }
        }
        switch (a) {
        case 0x4008: /* WF LUT INTSTA */
        case 0x600c: /* DPI INTSTA */
        case 0xa004: /* working-buffer WDMA INTSTA */
        case 0x13008: /* blend INTSTA */
            value &= old;
            break;
        case 0xd044: {
            /* IRQ_CTL[23:16] are pulse clears. Status bit locations are
             * not contiguous: see pipeline_print_irq_status(). */
            static const uint8_t bit[] = { 3, 2, 9, 0, 1, 10, 5, 13 };
            for (unsigned i = 0; i < 8; i++) {
                if (value & BIT(16 + i)) { R(s, 0xd02c) &= ~BIT(bit[i]); }
            }
            break;
        }
        case 0x4014: /* waveform reset, held while asserted */
            if (value & 1) {
                timer_del(s->frame_timer);
                memset(s->job, 0, sizeof(s->job));
                R(s, 0x4000) = R(s, 0x4008) = 0;
                for (unsigned i = 0; i < 8; i++) {
                    R(s, release_status[i]) = R(s, end_status[i]) = 0;
                }
                memset(&R(s, 0x4270), 0, 16 * 4);
            }
            break;
        case 0x4204: /* Per-LUT job shadow latch, self-clearing. */
            if (value & 1) { waveform_shadow(s); }
            value &= ~1;
            break;
        case 0xa00c: /* WDMA software reset */
            if (value & 1) { R(s, 0xa004) = 0; }
            break;
        case 0xd028:
            if ((value & 1) && !(old & 1)) {
                s->pipeline_triggers++;
                if (clear_working_buffer(s) || numerical_regal(s) ||
                    update_working_buffer(s, value)) { break; }
                qemu_log_mask(LOG_UNIMP,
                    "mt8171-hwtcon: pipeline trigger %" PRIu64
                    " flags=%08x image=%08x wb=%08x: pixel processing unimplemented\n",
                    s->pipeline_triggers, R(s, 0xdf0c), R(s, 0xdf08), R(s, 0xdf00));
            }
            break;
        case 0x4010:
            if ((value & 1) && !(old & 1)) {
                s->waveform_triggers++;
                qemu_log_mask(LOG_UNIMP,
                    "mt8171-hwtcon: waveform trigger %" PRIu64
                    ": waveform processing unimplemented\n", s->waveform_triggers);
            }
            break;
        case 0x4d0c:
            if (value & BIT(15)) {
                for (unsigned i = 0; i < 8; i++) { R(s, end_status[i]) = 0; }
            }
            break;
        }
    } else {
        switch (a) {
        case 0x6004: /* display RDMA */
        case 0x1a004: /* display WDMA */
        case 0x1600c: /* DSI0 */
        case 0x1b00c: /* DSI1 */
            value &= old;
            break;
        }
    }
    b->regs[a / 4] = value;
    if (b->index == 0 && (a == 0x400c || a == 0x6000)) {
        waveform_schedule(s);
    }
    update_irq(s);
}
static const MemoryRegionOps ops = {
    .read = read_reg, .write = write_reg, .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static void reset(DeviceState *dev)
{
    MT8171HWTCONState *s = MT8171_HWTCON(dev);
    for (unsigned i = 0; i < 2; i++) {
        memset(s->bank[i].regs, 0, sizeof(s->bank[i].regs));
    }
    timer_del(s->frame_timer);
    memset(s->job, 0, sizeof(s->job));
    s->waveform_jobs = s->completed_frames = s->completed_waveforms = 0;
    s->drive_pixels = 0;
    s->pipeline_triggers = s->waveform_triggers = 0;
    s->clear_writebacks = s->normal_writebacks = s->dma_faults = 0;
    s->numerical_regal_jobs = 0;
    update_irq(s);
    for (unsigned i = 0; i < 3; i++) { qemu_set_irq(s->event[i], 0); }
}
static void init(Object *obj)
{
    MT8171HWTCONState *s = MT8171_HWTCON(obj);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, waveform_tick, s);
    object_property_add_uint64_ptr(obj, "normal-writebacks", &s->normal_writebacks, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "numerical-regal-jobs", &s->numerical_regal_jobs, OBJ_PROP_FLAG_READ);
    object_property_set_description(obj, "numerical-regal-jobs",
        "Completed real-DMA CSR2 numerical quantization jobs; spatial ghost compensation is uncalibrated");
    object_property_add_uint64_ptr(obj, "waveform-jobs", &s->waveform_jobs, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "completed-frames", &s->completed_frames, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "completed-waveforms", &s->completed_waveforms, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "drive-pixels", &s->drive_pixels, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "dma-faults", &s->dma_faults, OBJ_PROP_FLAG_READ);
    for (unsigned i = 0; i < 2; i++) {
        s->bank[i].owner = s; s->bank[i].index = i;
        memory_region_init_io(&s->bank[i].iomem, obj, &ops, &s->bank[i],
                             i ? "mt8171-disp" : "mt8171-epd",
                             i ? 0x1d000 : 0x13000);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->bank[i].iomem);
    }
    for (unsigned i = 0; i < 9; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), s->event, "event", 3);
}
static void finalize(Object *obj)
{
    MT8171HWTCONState *s = MT8171_HWTCON(obj);
    timer_free(s->frame_timer);
    g_free(s->drive_frame);
    g_free(s->drive_accumulator);
}
static void class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), reset);
}
static const TypeInfo info = {
    .name = TYPE_MT8171_HWTCON, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8171HWTCONState), .instance_init = init,
    .class_init = class_init, .instance_finalize = finalize,
};
static void register_types(void) { type_register_static(&info); }
type_init(register_types)
