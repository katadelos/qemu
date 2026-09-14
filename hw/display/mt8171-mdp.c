/*
 * MT8171 MDP register fabric, e-paper color LUT DMA and RDMA/WROT.
 * Register contract: vendor mtk_mdp_{color,rdma,wrot,mutex}.{c,h}.
 * LUT bank extents follow the offsets in color.c and the shipped
 * cfa_memory_mode5_cfa.bin (74752 B) / mode8 (104000 B) layouts.
 * Unsupported pixel processing stalls instead of reporting an unperformed job.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/display/mt8171-mdp.h"
#include "hw/core/irq.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/module.h"
#include "system/address-spaces.h"

#define R(s, a) ((s)->regs[(a) / 4])
#define COLOR 0x14000
static const size_t lut_size[8] = {
    0, 0x8000, 0x2000, 0x8000, 0x400, 0, 0x17640, 0x2000,
};

static void mdp_event(MT8171MDPState *s, unsigned event)
{
    qemu_irq_pulse(s->events[event]);
}

static void mdp_irq_update(MT8171MDPState *s)
{
    qemu_set_irq(s->irq[0], !!(R(s, 0x1000) & R(s, 0x1004)));
    for (unsigned i = 0; i < 3; i++) {
        unsigned rdma = 0x3000 + i * 0x1000;
        unsigned wrot = 0xf000 + i * 0x1000;
        qemu_set_irq(s->irq[1 + i],
                     !!(R(s, rdma + 0x10) & R(s, rdma + 0x18)));
        qemu_set_irq(s->irq[13 + i],
                     !!(R(s, wrot + 0x18) & R(s, wrot + 0x1c)));
    }
    qemu_set_irq(s->irq[17], !!(R(s, 0x13008) & R(s, 0x1300c)));
    qemu_set_irq(s->irq[18], s->color_pending);
}

static bool mdp_dma(MT8171MDPState *s, unsigned port, hwaddr address,
                    void *buffer, size_t bytes, bool write)
{
    uint8_t *p = buffer;
    if (!s->larb_regs || !s->m4u) {
        qemu_log_mask(LOG_GUEST_ERROR, "mt8171-mdp: DMA fabric not wired\n");
        return false;
    }
    uint32_t con = s->larb_regs[0x380 / 4 + port];
    /* MT8171 configures port MMU enable via the secure-monitor SMI command.
     * On security-controlled SMI, NONSEC_CON.MMU_EN is ineffective; the bank
     * selector remains NONSEC_CON (upstream drivers/memory/mtk-smi.c).
     * The board enforces nonsecure access protection on SEC_CON writes. */
    bool mmu_enabled = s->larb_regs[0xf80 / 4 + port] & 1;
    while (bytes) {
        hwaddr physical = address;
        size_t span = MIN(bytes, 4096 - (address & 4095));
        if (mmu_enabled) {
            size_t translated_span;
            unsigned bank = (con >> (8 + 2 * ((address >> 32) & 3))) & 3;
            /* MT8171 larbid_remap: larb2 is common master 2, subcommon0. */
            if (!mt8171_m4u_translate(s->m4u, bank, (2 << 10) | (port << 2),
                                      address, write, &physical,
                                      &translated_span)) {
                return false;
            }
            span = MIN(span, translated_span);
        }
        /* Physical DMA applies only while secure MMU enable is clear. */
        if (address_space_rw(&address_space_memory, physical,
                             MEMTXATTRS_UNSPECIFIED, p, span, write) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt8171-mdp: DMA transaction failed port%u at %"HWADDR_PRIx"\n",
                          port, physical);
            return false;
        }
        address += span;
        p += span;
        bytes -= span;
    }
    return true;
}

static void mdp_lut_done(void *opaque)
{
    MT8171MDPState *s = opaque;
    unsigned mode = s->lut_mode;
    if (!mdp_dma(s, 0, s->lut_address, s->lut[mode], lut_size[mode], false)) {
        return; /* Leave busy asserted; fault/timeout is observable. */
    }
    s->lut_valid |= 1 << mode;
    if (qemu_loglevel_mask(LOG_UNIMP)) {
        unsigned lo = 255, hi = 0;
        for (size_t i = 0; i < lut_size[mode]; i++) {
            lo = MIN(lo, s->lut[mode][i]);
            hi = MAX(hi, s->lut[mode][i]);
        }
        qemu_log_mask(LOG_UNIMP,
                      "mt8171-mdp: LUT bank=%u src=%" HWADDR_PRIx
                      " sec=%#x nonsec=%#x bytes=%zu range=%u..%u first=%08x\n",
                      mode, s->lut_address, s->larb_regs[0xf80 / 4],
                      s->larb_regs[0x380 / 4], lut_size[mode], lo, hi,
                      ldl_le_p(s->lut[mode]));
    }
    s->color_busy = false;
    s->color_pending = true;
    R(s, COLOR + 0x10) = 0;
    mdp_event(s, 317);
    mdp_irq_update(s);
}

static void mdp_color_start(MT8171MDPState *s)
{
    unsigned mode = R(s, COLOR + 4);
    if (s->color_busy) {
        return;
    }
    s->color_busy = true;
    if (mode < ARRAY_SIZE(lut_size) && lut_size[mode]) {
        s->lut_mode = mode;
        s->lut_address = R(s, COLOR + 8);
        /* Nominal transfer latency, not a captured hardware timing. */
        timer_mod(s->lut_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  lut_size[mode] * 10);
    } else if (!(mode & (1 << 8))) {
        qemu_log_mask(LOG_UNIMP, "mt8171-mdp: color operation %#x unsupported\n", mode);
    }
    /* Real-time mode waits for an upstream streaming image. */
}

static unsigned mdp_bpp(unsigned format)
{
    static const unsigned bpp[8] = { 2, 3, 4, 4, 0, 0, 0, 1 };
    return format < 8 ? bpp[format] : 0;
}

static void mdp_unpack_pixel(const uint8_t *p, unsigned format, bool swap,
                              uint8_t rgb[3])
{
    switch (format) {
    case 0: {
        unsigned v = lduw_le_p(p);
        rgb[0] = ((v >> 11) * 255 + 15) / 31;
        rgb[1] = (((v >> 5) & 63) * 255 + 31) / 63;
        rgb[2] = ((v & 31) * 255 + 15) / 31;
        break;
    }
    case 1:
    case 2:
        rgb[0] = p[2]; rgb[1] = p[1]; rgb[2] = p[0];
        break;
    case 3:
        rgb[0] = p[1]; rgb[1] = p[2]; rgb[2] = p[3];
        break;
    default:
        rgb[0] = rgb[1] = rgb[2] = p[0];
        break;
    }
    if (swap) {
        uint8_t t = rgb[0]; rgb[0] = rgb[2]; rgb[2] = t;
    }
}

static void mdp_pack_pixel(uint8_t *p, unsigned format, bool swap,
                            const uint8_t in[3])
{
    uint8_t r = in[swap ? 2 : 0], g = in[1], b = in[swap ? 0 : 2];
    switch (format) {
    case 0:
        stw_le_p(p, ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
        break;
    case 1:
    case 2:
        p[0] = b; p[1] = g; p[2] = r;
        if (format == 2) { p[3] = 255; }
        break;
    case 3:
        p[0] = 255; p[1] = r; p[2] = g; p[3] = b;
        break;
    default:
        p[0] = b;
        break;
    }
}

static bool mdp_gamma_pixel(MT8171MDPState *s, uint8_t rgb[3])
{
    uint32_t cfg = R(s, 0x13020);
    if (!(R(s, 0x13000) & 1)) {
        return false;
    }
    if (cfg & 1) {
        return true;
    }
    if (!(cfg & 2) || (cfg & 0x18)) {
        return false;
    }
    /* Driver writes 256 RGB10 entries at eight-byte intervals; low two
     * bits of each component are zero for its eight-bit tables. */
    for (unsigned i = 0; i < 3; i++) {
        uint32_t word = R(s, 0x13700 + rgb[i] * 8);
        rgb[i] = (word >> (2 + 10 * i)) & 255;
    }
    return true;
}

/* Mode5 numerical model: the shipped payload is an18^3 RGBX cube with
 * red varying fastest. The active driver selects CFA and software dithering;
 * the original CPU libeink pass performs the later spatial quantization.
 *
 * Inferred interpolation contract (not a captured hardware calibration):
 * distribute18 nodes uniformly over the8-bit input range, hence15 input
 * codes per cell, apply trilinear weights and round once to nearest integer.
 * This preserves every original node exactly and bounds output by its eight
 * source corners. It never substitutes the framebuffer for an absent LUT.
 * Fixed-point hardware rounding between nodes remains unverified. */
static bool mdp_color_pixel(MT8171MDPState *s, const uint8_t rgb[3],
                            bool color_flag, unsigned x, unsigned y,
                            uint8_t *out)
{
    uint32_t mode = R(s, COLOR + 4);
    uint32_t cfa = R(s, COLOR + 0x20), xy = R(s, COLOR + 0x64);
    unsigned cfa_width = (cfa & 255) + 1;
    unsigned cfa_height = ((cfa >> 8) & 255) + 1;
    if ((mode & 0xff) != 5 || !(mode & 0x100) ||
        !(mode & (1 << 24)) || R(s, COLOR + 0x8c) != 2 ||
        !(s->lut_valid & 2) || cfa_width > 16 || cfa_height > 16) {
        return false;
    }
    x = (x + ((xy >> 16) & 15)) % cfa_width;
    y = (y + ((xy >> 24) & 15)) % cfa_height;
    unsigned selector = (R(s, COLOR + 0x24 + y * 4) >> (x * 2)) & 3;
    if (!selector) { return false; }
    /* CFA pattern comparison with libeink's RGB/BGR adjustment identifies
     * selector1=B,2=G,3=R. Registers already account for image rotation. */
    unsigned channel = 3 - selector;
    unsigned index[3], fraction[3];
    uint32_t sum = 0;
    for (unsigned i = 0; i < 3; i++) {
        index[i] = rgb[i] / 15;
        fraction[i] = rgb[i] % 15;
    }
    for (unsigned corner = 0; corner < 8; corner++) {
        unsigned coordinate[3], weight = 1;
        for (unsigned i = 0; i < 3; i++) {
            bool upper = corner & (1 << i);
            coordinate[i] = MIN(17, index[i] + upper);
            weight *= upper ? fraction[i] : 15 - fraction[i];
        }
        unsigned node = (coordinate[2] * 18 * 18 +
                         coordinate[1] * 18 + coordinate[0]) * 4;
        sum += weight * s->lut[1][node + channel];
    }
    *out = (((sum + 1687) / 3375) & 0xfe) |
           ((mode & (1U << 29)) && color_flag);
    return true;
}

/* The mode5 companion preserves original RGB for the kernel's subsequent
 * CPU dither pass. rsz_config() explicitly disables RSZ scaling and programs
 * equal input/output dimensions; no resampling approximation is involved. */
static bool mdp_companion_wrot0(MT8171MDPState *s, const uint8_t *input,
                                unsigned width, unsigned height,
                                unsigned format, bool swap)
{
    const unsigned wb = 0xf000;
    uint32_t wc = R(s, wb);
    unsigned rotation = (wc >> 20) & 3;
    unsigned bpp = mdp_bpp(format);
    unsigned outw = (rotation & 1) ? height : width;
    unsigned outh = (rotation & 1) ? width : height;
    unsigned pitch = R(s, wb + 0x30) & 0x3fff;
    hwaddr dst = ((hwaddr)R(s, wb + 0xf34) << 32) | R(s, wb + 0xf00);
    dst += ((hwaddr)R(s, wb + 0xf40) << 32) | R(s, wb + 0x2c);
    if (!(R(s, 0xf30) & 8)) {
        return true; /* RDMA1 has no companion branch configured. */
    }
    if (R(s, 0xf84) != 1 || R(s, 0xf54) != 2 ||
        R(s, 0xf94) != 4 || R(s, 0xf64) != 1 ||
        (R(s, 0xa000) & 1) || (R(s, 0xa004) & 3) ||
        R(s, 0xa010) != (height << 16 | width) ||
        R(s, 0xa014) != R(s, 0xa010) ||
        !(R(s, wb + 0x7c) & 1) || (wc & 15) != format ||
        (R(s, wb + 0x20) & 0x3fff3fff) ||
        (R(s, wb + 0x24) & 0x3fff3fff) != (height << 16 | width) ||
        (R(s, wb + 0x88) & 15) || !bpp || pitch < outw * bpp) {
        qemu_log_mask(LOG_UNIMP,
            "mt8171-mdp: unsupported companion RDMA1/RSZ1/WROT0 route\n");
        return false;
    }
    if (rotation == 1) {
        dst -= height * bpp - 1;
    } else if (rotation == 2) {
        dst -= (height - 1) * pitch + width * bpp - 1;
    } else if (rotation == 3) {
        dst -= (width - 1) * pitch;
    }
    g_autofree uint8_t *row = g_malloc(outw * bpp);
    mdp_event(s, 268);
    for (unsigned y = 0; y < outh; y++) {
        for (unsigned x = 0; x < outw; x++) {
            unsigned sx, sy;
            switch (rotation) {
            case 0: sx = x; sy = y; break;
            case 1: sx = y; sy = height - x - 1; break;
            case 2: sx = width - x - 1; sy = height - y - 1; break;
            default: sx = width - y - 1; sy = x; break;
            }
            const uint8_t *p = input + ((size_t)sy * width + sx) * bpp;
            if (swap == !!(wc & (1 << 8))) {
                memcpy(row + x * bpp, p, bpp);
            } else {
                uint8_t rgb[3];
                mdp_unpack_pixel(p, format, swap, rgb);
                mdp_pack_pixel(row + x * bpp, format, !!(wc & (1 << 8)), rgb);
                if (format == 2) { row[x * bpp + 3] = p[3]; }
                if (format == 3) { row[x * bpp] = p[0]; }
            }
        }
        if (!mdp_dma(s, 3, dst + (uint64_t)y * pitch, row, outw * bpp, true)) {
            return false;
        }
    }
    R(s, wb + 0x1c) |= 1;
    s->companion_tiles++;
    mdp_event(s, 276);
    mdp_irq_update(s);
    return true;
}

/* RDMA1 -> BYP1/WROT1, gamma, and mode5 CFA with original CLUT data.
 * DMA is page-batched; rotations operate on the fetched tile in host memory.
 * An unsupported transform never writes output or emits EOF. */
static bool mdp_copy_path(MT8171MDPState *s)
{
    unsigned rb = 0x4000, wb = 0x10000;
    uint32_t rc = R(s, rb + 0x30), wc = R(s, wb);
    unsigned format = rc & 15, output_format = wc & 15;
    unsigned bpp = mdp_bpp(format), obpp = mdp_bpp(output_format);
    unsigned width = R(s, wb + 0x24) & 0x3fff;
    unsigned height = (R(s, wb + 0x24) >> 16) & 0x3fff;
    unsigned srcw = R(s, rb + 0x78) & 0x3fff;
    unsigned srch = (R(s, rb + 0x78) >> 16) & 0x3fff;
    unsigned cx = R(s, wb + 0x20) & 0x3fff;
    unsigned cy = (R(s, wb + 0x20) >> 16) & 0x3fff;
    unsigned rotation = (wc >> 20) & 3;
    unsigned spitch = R(s, rb + 0x60) & 0x1fffff;
    unsigned dpitch = R(s, wb + 0x30) & 0x3fff;
    hwaddr src = ((hwaddr)R(s, rb + 0xf30) << 32) | R(s, rb + 0xf00);
    src += ((hwaddr)R(s, rb + 0xf44) << 32) | R(s, rb + 0x118);
    hwaddr dst = ((hwaddr)R(s, wb + 0xf34) << 32) | R(s, wb + 0xf00);
    dst += ((hwaddr)R(s, wb + 0xf40) << 32) | R(s, wb + 0x2c);
    bool plain = (R(s, 0xf28) & 1) && R(s, 0xf68) == 0;
    bool gamma = (R(s, 0xf28) & 2) && (R(s, 0xf30) & 4) &&
                  R(s, 0xf78) == 2;
    bool color = (R(s, 0xf28) & 2) && R(s, 0xf98) == 0 &&
                  R(s, 0xf34) == 1 &&
                  ((gamma && R(s, 0xf40) == 0 && R(s, 0xf5c) == 1) ||
                   ((R(s, 0xf30) & 2) && R(s, 0xf5c) == 0));
    bool gamma_only = gamma && R(s, 0xf40) == 2 && R(s, 0xf98) == 4;
    unsigned outw = (rotation & 1) ? height : width;
    unsigned outh = (rotation & 1) ? width : height;
    bool trace = qemu_loglevel_mask(LOG_UNIMP);
    unsigned in_min[3] = {255, 255, 255}, in_max[3] = {0};
    unsigned gamma_min[3] = {255, 255, 255}, gamma_max[3] = {0};

    if (!(R(s, rb) & 1) || !(R(s, wb + 0x7c) & 1)) {
        return false;
    }
    if ((!plain && !color && !gamma_only) || !bpp || !obpp ||
        (plain && output_format != format) ||
        (color && output_format != 7) || (R(s, wb + 0x88) & 15) ||
        !width || !height || cx + width > srcw || cy + height > srch ||
        spitch < (cx + width) * bpp || dpitch < outw * obpp ||
        (uint64_t)width * height * bpp > 64 * MiB) {
        qemu_log_mask(LOG_UNIMP,
                      "mt8171-mdp: unimplemented stream route/format: bypass=%#x rdma=%#x wrot=%#x size=%ux%u\n",
                      R(s, 0xf28), rc, wc, width, height);
        return false;
    }
    /* WROT addresses are biased to the first source pixel's rotated
     * destination by mdp_tile_config()/wrot_config(). Normalize to tile origin. */
    if (rotation == 1) {
        dst -= height * obpp - 1;
    } else if (rotation == 2) {
        dst -= (height - 1) * dpitch + width * obpp - 1;
    } else if (rotation == 3) {
        dst -= (width - 1) * dpitch;
    }
    /* This observer walks the known Linux vmalloc DMA mapping explicitly.
     * Its result does not feed the hardware DMA or completion path below. */
    /* The preview observes the image plane. RGB24 CSR jobs operate on
     * packed working-buffer history and are not framebuffer submissions. */
    if (obpp == 1) {
        mt8171_panel_capture_iova(s->panel, s->m4u,
                              src + (uint64_t)cy * spitch + cx * bpp,
                              spitch, width, height, format,
                              !!(rc & (1 << 14)), rotation, dst, dpitch, obpp);
    }
    g_autofree uint8_t *input = g_malloc((size_t)width * height * bpp);
    g_autofree uint8_t *output = g_malloc((size_t)width * height * obpp);
    g_autofree uint8_t *row = g_malloc(outw * obpp);
    mdp_event(s, 257);
    mdp_event(s, 269);
    for (unsigned y = 0; y < height; y++) {
        if (!mdp_dma(s, 2, src + (y + cy) * spitch + cx * bpp,
                     input + (size_t)y * width * bpp, width * bpp, false)) {
            return false;
        }
    }
    if (!mdp_companion_wrot0(s, input, width, height, format,
                             !!(rc & (1 << 14)))) {
        return false;
    }
    for (size_t i = 0; i < (size_t)width * height; i++) {
        uint8_t rgb[3];
        mdp_unpack_pixel(input + i * bpp, format, !!(rc & (1 << 14)), rgb);
        if (trace) {
            for (unsigned c = 0; c < 3; c++) {
                in_min[c] = MIN(in_min[c], rgb[c]);
                in_max[c] = MAX(in_max[c], rgb[c]);
            }
        }
        bool flag = !((rgb[0] == 0 && rgb[1] == 0 && rgb[2] == 0) ||
                      (rgb[0] == 255 && rgb[1] == 255 && rgb[2] == 255));
        if (gamma && !mdp_gamma_pixel(s, rgb)) {
            return false;
        }
        if (trace) {
            for (unsigned c = 0; c < 3; c++) {
                gamma_min[c] = MIN(gamma_min[c], rgb[c]);
                gamma_max[c] = MAX(gamma_max[c], rgb[c]);
            }
        }
        if (color) {
            if (!mdp_color_pixel(s, rgb, flag, i % width, i / width, output + i)) {
                qemu_log_mask(LOG_UNIMP,
                              "mt8171-mdp: color pixel %u,%u,%u mode=%#x needs unimplemented processing\n",
                              rgb[0], rgb[1], rgb[2], R(s, COLOR + 4));
                return false;
            }
        } else if (plain && !!(rc & (1 << 14)) == !!(wc & (1 << 8))) {
            /* Preserve the alpha and RGB565 bit patterns of a direct copy. */
            memcpy(output + i * obpp, input + i * bpp, obpp);
        } else {
            mdp_pack_pixel(output + i * obpp, output_format,
                           !!(wc & (1 << 8)), rgb);
        }
    }
    for (unsigned y = 0; y < outh; y++) {
        for (unsigned x = 0; x < outw; x++) {
            unsigned ix, iy;
            switch (rotation) {
            case 0: ix = x; iy = y; break;
            case 1: ix = y; iy = height - x - 1; break;
            case 2: ix = width - x - 1; iy = height - y - 1; break;
            default: ix = width - y - 1; iy = x; break;
            }
            memcpy(row + x * obpp, output + ((size_t)iy * width + ix) * obpp, obpp);
        }
        if (!mdp_dma(s, 4, dst + y * dpitch, row, outw * obpp, true)) {
            return false;
        }
    }
    R(s, rb + 0x18) |= 1;
    R(s, wb + 0x1c) |= 1;
    if (color) {
        s->color_busy = false;
        s->color_pending = true;
        R(s, COLOR + 0x10) = 0;
        mdp_event(s, 317);
    }
    if (gamma) {
        R(s, 0x1300c) |= 1;
    }
    mdp_event(s, 284);
    mdp_event(s, 275);
    s->completed_tiles++;
    if (trace) {
        unsigned lo = 255, hi = 0;
        for (size_t i = 0; i < (size_t)width * height * obpp; i++) {
            lo = MIN(lo, output[i]);
            hi = MAX(hi, output[i]);
        }
        qemu_log_mask(LOG_UNIMP,
                      "mt8171-mdp: tile=%" PRIu64 " src=%" HWADDR_PRIx
                      " dst=%" HWADDR_PRIx " %ux%u fmt=%u/%u"
                      " sec[2,3,4]=%x,%x,%x nonsec[2,3,4]=%x,%x,%x"
                      " gamma=%u cfg=%x color=%u mode=%x"
                      " inputRGB=%u..%u,%u..%u,%u..%u"
                      " gammaRGB=%u..%u,%u..%u,%u..%u out=%u..%u"
                      " gammaLUT[0,128,255]=%08x,%08x,%08x\n",
                      s->completed_tiles, src, dst, width, height,
                      format, output_format,
                      s->larb_regs[0xf80 / 4 + 2],
                      s->larb_regs[0xf80 / 4 + 3],
                      s->larb_regs[0xf80 / 4 + 4],
                      s->larb_regs[0x380 / 4 + 2],
                      s->larb_regs[0x380 / 4 + 3],
                      s->larb_regs[0x380 / 4 + 4],
                      gamma, R(s, 0x13020), color, R(s, COLOR + 4),
                      in_min[0], in_max[0], in_min[1], in_max[1],
                      in_min[2], in_max[2], gamma_min[0], gamma_max[0],
                      gamma_min[1], gamma_max[1], gamma_min[2], gamma_max[2],
                      lo, hi, R(s, 0x13700), R(s, 0x13b00), R(s, 0x13ef8));
    }
    mdp_irq_update(s);
    return true;
}

static uint64_t mdp_read_offset(MT8171MDPState *s, hwaddr off)
{
    if (off == 0xf4 && (R(s, 0x300) & 31) == 15) {
        return s->color_busy ? 0 : 8;
    }
    if (off == COLOR + 0x104) {
        unsigned control = R(s, COLOR + 0x100);
        unsigned bank = (control >> 24) & 7;
        unsigned index = (control & 0xffff) * 4;
        if (index + 4 <= lut_size[bank]) {
            return ldl_le_p(s->lut[bank] + index);
        }
        return 0;
    }
    return R(s, off);
}

static void mdp_write_offset(MT8171MDPState *s, hwaddr off, uint32_t value)
{
    unsigned block = off & ~0xfff, reg = off & 0xfff;
    if (off == 0x700) {
        /* imgsys_reset(): active-low resets for RDMA1, WROT1 and color. */
        if (!(value & (1 << 5))) {
            memset(&R(s, 0x4000), 0, 0x1000);
        }
        if (!(value & (1 << 17))) {
            memset(&R(s, 0x10000), 0, 0x1000);
        }
        if (!(value & (1 << 22))) {
            timer_del(s->lut_timer);
            s->color_busy = s->color_pending = false;
            s->lut_valid = 0;
            memset(&R(s, COLOR), 0, 0x1000);
        }
        R(s, off) = value;
        mdp_irq_update(s);
        return;
    }
    if (off == 0x104) {
        R(s, 0x100) |= value;
        return;
    }
    if (off == 0x108) {
        R(s, 0x100) &= ~value;
        return;
    }
    if (off == 0xf4 || off == COLOR + 0x14 || off == COLOR + 0x104) {
        return;
    }
    if (block >= 0x3000 && block <= 0x5000) {
        if (reg == 0x18) {
            R(s, off) &= value; /* RDMA software clears status by zero. */
            mdp_irq_update(s);
            return;
        }
        if (reg == 8 && (value & 1)) {
            memset(&R(s, block), 0, 0x1000);
            if (block < 0x5000) {
                mdp_event(s, 314 - (block - 0x3000) / 0x1000);
            }
        }
    }
    if (block >= 0xf000 && block <= 0x11000) {
        if (reg == 0x1c) {
            R(s, off) &= ~value;
            mdp_irq_update(s);
            return;
        }
        if (reg == 0x14) {
            return;
        }
        if (reg == 0x10) {
            if (value & 1) {
                memset(&R(s, block), 0, 0x1000);
                if (block < 0x11000) {
                    mdp_event(s, 310 - (block - 0xf000) / 0x1000);
                }
            }
            R(s, block + 0x14) = value & 1;
        }
    }
    if (off == 0x1300c) {
        R(s, off) &= value;
        mdp_irq_update(s);
        return;
    }
    if (off == 0x13004 && (value & 1)) {
        memset(&R(s, 0x13000), 0, 0x1000);
    }
    if (off == COLOR + 0x18) {
        if (value & 1) {
            s->color_pending = false;
            mdp_irq_update(s);
        }
        return;
    }
    R(s, off) = value;
    if (off == COLOR + 0x10 && (value & 1)) {
        mdp_color_start(s);
    } else if (off == COLOR + 0x100 && (value & (1 << 16))) {
        unsigned bank = (value >> 24) & 7;
        unsigned index = (value & 0xffff) * 4;
        if (index + 4 <= lut_size[bank]) {
            stl_le_p(s->lut[bank] + index, R(s, COLOR + 0x108));
        }
    } else if (off >= 0x1020 && off < 0x1140 &&
               !(off & 31) && (value & 1)) {
        if (mdp_copy_path(s)) {
            R(s, off) &= ~1;
        }
    }
    mdp_irq_update(s);
}

static uint64_t mdp_control_read(void *p, hwaddr a, unsigned n)
{ return mdp_read_offset(p, a); }
static void mdp_control_write(void *p, hwaddr a, uint64_t v, unsigned n)
{ mdp_write_offset(p, a, v); }
static uint64_t mdp_engines_read(void *p, hwaddr a, unsigned n)
{ return mdp_read_offset(p, a + 0x3000); }
static void mdp_engines_write(void *p, hwaddr a, uint64_t v, unsigned n)
{ mdp_write_offset(p, a + 0x3000, v); }
static const MemoryRegionOps control_ops = {
    .read = mdp_control_read, .write = mdp_control_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static const MemoryRegionOps engines_ops = {
    .read = mdp_engines_read, .write = mdp_engines_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static void mdp_reset(DeviceState *dev)
{
    MT8171MDPState *s = MT8171_MDP(dev);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->lut, 0, sizeof(s->lut));
    R(s, 0x700) = R(s, 0x704) = R(s, 0x708) = UINT32_MAX;
    s->lut_valid = 0;
    s->completed_tiles = s->companion_tiles = 0;
    s->color_busy = s->color_pending = false;
    timer_del(s->lut_timer);
    for (unsigned i = 0; i < ARRAY_SIZE(s->irq); i++) {
        qemu_irq_lower(s->irq[i]);
    }
}
static void mdp_init(Object *obj)
{
    MT8171MDPState *s = MT8171_MDP(obj);
    object_property_add_uint64_ptr(obj, "completed-tiles", &s->completed_tiles,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "companion-tiles", &s->companion_tiles,
                                   OBJ_PROP_FLAG_READ);
    object_property_set_description(obj, "completed-tiles",
        "Tiles processed and written by the hardware WROT1 DMA path");
    object_property_set_description(obj, "companion-tiles",
        "Original RGB tiles written by the hardware companion WROT0 DMA path");
    memory_region_init_io(&s->control, obj, &control_ops, s,
                          TYPE_MT8171_MDP ".control", 0x2000);
    memory_region_init_io(&s->engines, obj, &engines_ops, s,
                          TYPE_MT8171_MDP ".engines", 0x13000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->control);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->engines);
    for (unsigned i = 0; i < ARRAY_SIZE(s->irq); i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), s->events, "event", 1024);
    s->lut_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mdp_lut_done, s);
}
static void mdp_finalize(Object *obj)
{ timer_free(MT8171_MDP(obj)->lut_timer); }
static void mdp_class_init(ObjectClass *klass, const void *data)
{ device_class_set_legacy_reset(DEVICE_CLASS(klass), mdp_reset); }
static const TypeInfo mdp_type = {
    .name = TYPE_MT8171_MDP, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8171MDPState), .instance_init = mdp_init,
    .instance_finalize = mdp_finalize, .class_init = mdp_class_init,
};
static void mdp_register(void) { type_register_static(&mdp_type); }
type_init(mdp_register);
