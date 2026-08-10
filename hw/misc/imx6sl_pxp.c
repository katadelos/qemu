/*
 * i.MX6 SoloLite enhanced Pixel Pipeline (ePxP v2)
 *
 * Implements the register interface used by the Freescale 3.0.35 PXP DMA
 * driver and the RGB/greyscale data path used by the Wario EPDC driver.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/misc/imx6sl_pxp.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "system/dma.h"

#define PXP_CTRL          0x000
#define PXP_STAT          0x010
#define PXP_OUT_CTRL      0x020
#define PXP_OUT_BUF       0x030
#define PXP_OUT_PITCH     0x050
#define PXP_OUT_LRC       0x060
#define PXP_OUT_PS_ULC    0x070
#define PXP_OUT_PS_LRC    0x080
#define PXP_PS_CTRL       0x0b0
#define PXP_PS_BUF        0x0c0
#define PXP_PS_PITCH      0x0f0
#define PXP_PS_SCALE      0x110
#define PXP_LUT_CTRL      0x240
#define PXP_LUT_ADDR      0x250
#define PXP_LUT_DATA      0x260
#define PXP_HIST_CTRL     0x290
#define PXP_CTRL2         0x310
#define PXP_IRQ           0x3a0
#define PXP_IRQ_SET       0x3a4
#define PXP_IRQ_CLR       0x3a8
#define PXP_IRQ_TOG       0x3ac
#define PXP_VERSION       0x430
#define PXP_WFA_FETCH1_ADDR  0x0c50
#define PXP_WFA_FETCH1_PITCH 0x0c60
#define PXP_WFA_FETCH1_SIZE  0x0c70
#define PXP_WFA_FETCH1_CORD  0x0db0
#define PXP_WFB_STORE_CTRL0  0x1340
#define PXP_WFB_STORE_SIZE0  0x1380
#define PXP_WFB_STORE_PITCH  0x13a0
#define PXP_WFB_STORE_SHIFT0 0x13b0
#define PXP_WFB_STORE_ADDR0  0x1410
#define PXP_WFB_STORE_FILL0  0x1430

#define CTRL_SFTRST       BIT(31)
#define CTRL_CLKGATE      BIT(30)
#define CTRL_VFLIP        BIT(11)
#define CTRL_HFLIP        BIT(10)
#define CTRL_ROTATE_MASK  (3U << 8)
#define CTRL_IRQ_ENABLE   BIT(1)
#define CTRL_ENABLE       BIT(0)
#define STAT_IRQ          BIT(0)
#define STAT_AXI_READ     BIT(2)
#define STAT_AXI_WRITE    BIT(1)
#define LUT_BYPASS        BIT(31)
#define CTRL2_ENABLE      BIT(0)
#define CTRL2_WFE_B       BIT(19)
#define WFB_FILL_ENABLE   BIT(11)
#define WFB_STORE_ENABLE  BIT(9)
#define IRQ_WFB_CH0       BIT(10)

#define PXP_MAX_DMA_BYTES (32 * MiB)

static inline uint32_t *pxp_reg(IMX6SLPXPState *s, hwaddr offset)
{
    return &s->regs[offset >> 2];
}

bool imx6sl_pxp_get_wfe_a_fetch(IMX6SLPXPState *s,
                                IMX6SLPXPFetch *fetch)
{
    uint32_t size = *pxp_reg(s, PXP_WFA_FETCH1_SIZE);
    uint32_t cord = *pxp_reg(s, PXP_WFA_FETCH1_CORD);

    fetch->addr = *pxp_reg(s, PXP_WFA_FETCH1_ADDR);
    fetch->pitch = *pxp_reg(s, PXP_WFA_FETCH1_PITCH) & 0xffff;
    fetch->left = cord & 0x3fff;
    fetch->top = (cord >> 16) & 0x3fff;
    fetch->width = (size & 0x3fff) + 1;
    fetch->height = ((size >> 16) & 0x3fff) + 1;

    return fetch->addr && fetch->pitch && fetch->width && fetch->height &&
           fetch->left + fetch->width <= fetch->pitch;
}

bool imx6sl_pxp_get_wfe_b_store(IMX6SLPXPState *s,
                                IMX6SLPXPFetch *store)
{
    uint32_t size = *pxp_reg(s, PXP_WFB_STORE_SIZE0);
    unsigned bpp = 1U << ((*pxp_reg(s, PXP_WFB_STORE_SHIFT0) >> 2) & 3);

    store->addr = *pxp_reg(s, PXP_WFB_STORE_ADDR0);
    store->pitch = *pxp_reg(s, PXP_WFB_STORE_PITCH) & 0xffff;
    store->left = 0;
    store->top = 0;
    store->width = (size & 0xffff) + 1;
    store->height = (size >> 16) + 1;

    return bpp == 1 && store->addr && store->pitch && store->width &&
           store->height && store->width <= store->pitch;
}

static bool pxp_wfe_b_fill(IMX6SLPXPState *s)
{
    uint32_t ctrl = *pxp_reg(s, PXP_WFB_STORE_CTRL0);
    uint32_t size = *pxp_reg(s, PXP_WFB_STORE_SIZE0);
    uint32_t pitch = *pxp_reg(s, PXP_WFB_STORE_PITCH) & 0xffff;
    uint32_t width = (size & 0xffff) + 1;
    uint32_t height = (size >> 16) + 1;
    uint32_t value = *pxp_reg(s, PXP_WFB_STORE_FILL0);
    unsigned bpp = 1U << ((*pxp_reg(s, PXP_WFB_STORE_SHIFT0) >> 2) & 3);
    size_t bytes;
    g_autofree uint8_t *buffer = NULL;
    unsigned x, y;

    if (bpp > sizeof(value) ||
        !(ctrl & WFB_FILL_ENABLE) || !(ctrl & WFB_STORE_ENABLE) ||
        !pitch || !width || !height || width > pitch / bpp) {
        return false;
    }
    bytes = (size_t)pitch * height;
    if (bytes > PXP_MAX_DMA_BYTES) {
        return false;
    }
    buffer = g_malloc(bytes);
    for (y = 0; y < height; y++) {
        uint8_t *row = buffer + (size_t)y * pitch;

        memset(row, 0, pitch);
        for (x = 0; x < width; x++) {
            memcpy(row + (size_t)x * bpp, &value, bpp);
        }
    }
    return dma_memory_write(&address_space_memory,
                            *pxp_reg(s, PXP_WFB_STORE_ADDR0),
                            buffer, bytes, MEMTXATTRS_UNSPECIFIED) == MEMTX_OK;
}

static void imx6sl_pxp_update_irq(IMX6SLPXPState *s)
{
    /*
     * The i.MX6ULL/v3P driver rewrites CTRL as it selects its data path and
     * can clear the legacy IRQ_ENABLE bit after arming the transaction.  Its
     * normal completion output remains level-sensitive to STAT.IRQ.
     */
    qemu_set_irq(s->irq, !!(*pxp_reg(s, PXP_STAT) & STAT_IRQ));
}

static unsigned pxp_bytes_per_pixel(unsigned format)
{
    switch (format) {
    case 0x00: /* ARGB8888 */
    case 0x04: /* RGB888 in a 32-bit word on ePxP v2 */
        return 4;
    case 0x08: /* ARGB1555 */
    case 0x09: /* ARGB4444 */
    case 0x0c: /* RGB555 */
    case 0x0d: /* RGB444 */
    case 0x0e: /* RGB565 */
        return 2;
    case 0x14: /* Y8 */
        return 1;
    case 0x15: /* Y4 */
        return 0;
    default:
        return UINT_MAX;
    }
}

static uint8_t pxp_luma(const uint8_t *p, unsigned format, unsigned x)
{
    unsigned r, g, b;
    uint16_t v;

    switch (format) {
    case 0x00:
    case 0x04:
        b = p[0]; g = p[1]; r = p[2];
        break;
    case 0x0e:
        v = lduw_le_p(p);
        r = ((v >> 11) & 0x1f) * 255 / 31;
        g = ((v >> 5) & 0x3f) * 255 / 63;
        b = (v & 0x1f) * 255 / 31;
        break;
    case 0x0c:
    case 0x08:
        v = lduw_le_p(p);
        r = ((v >> 10) & 0x1f) * 255 / 31;
        g = ((v >> 5) & 0x1f) * 255 / 31;
        b = (v & 0x1f) * 255 / 31;
        break;
    case 0x0d:
    case 0x09:
        v = lduw_le_p(p);
        r = ((v >> 8) & 0xf) * 17;
        g = ((v >> 4) & 0xf) * 17;
        b = (v & 0xf) * 17;
        break;
    case 0x14:
        return p[0];
    case 0x15:
        return ((x & 1) ? (p[0] >> 4) : (p[0] & 0xf)) * 17;
    default:
        return 0;
    }

    /* ePxP's standard RGB-to-Y coefficients, rounded to 8 bits. */
    return (77 * r + 150 * g + 29 * b + 128) >> 8;
}

static void pxp_store_gray(uint8_t *p, unsigned format, unsigned x, uint8_t y)
{
    uint16_t v;

    switch (format) {
    case 0x14:
        p[0] = y;
        break;
    case 0x15:
        if (x & 1) {
            p[0] = (p[0] & 0x0f) | (y & 0xf0);
        } else {
            p[0] = (p[0] & 0xf0) | (y >> 4);
        }
        break;
    case 0x0e:
        v = ((y >> 3) << 11) | ((y >> 2) << 5) | (y >> 3);
        stw_le_p(p, v);
        break;
    case 0x00:
    case 0x04:
        p[0] = y; p[1] = y; p[2] = y; p[3] = 0xff;
        break;
    default:
        break;
    }
}

static bool pxp_do_dma(IMX6SLPXPState *s)
{
    uint32_t out_lrc = *pxp_reg(s, PXP_OUT_LRC);
    uint32_t ps_ulc = *pxp_reg(s, PXP_OUT_PS_ULC);
    uint32_t ps_lrc = *pxp_reg(s, PXP_OUT_PS_LRC);
    uint32_t scale = *pxp_reg(s, PXP_PS_SCALE);
    uint32_t ctrl = *pxp_reg(s, PXP_CTRL);
    unsigned out_w = ((out_lrc >> 16) & 0x3fff) + 1;
    unsigned out_h = (out_lrc & 0x3fff) + 1;
    unsigned left = (ps_ulc >> 16) & 0x3fff;
    unsigned top = ps_ulc & 0x3fff;
    unsigned right = (ps_lrc >> 16) & 0x3fff;
    unsigned bottom = ps_lrc & 0x3fff;
    unsigned src_fmt = *pxp_reg(s, PXP_PS_CTRL) & 0x1f;
    unsigned dst_fmt = *pxp_reg(s, PXP_OUT_CTRL) & 0x1f;
    unsigned src_bpp = pxp_bytes_per_pixel(src_fmt);
    unsigned dst_bpp = pxp_bytes_per_pixel(dst_fmt);
    unsigned src_pitch = *pxp_reg(s, PXP_PS_PITCH) & 0xffff;
    unsigned dst_pitch = *pxp_reg(s, PXP_OUT_PITCH) & 0xffff;
    unsigned xscale = scale & 0x7fff;
    unsigned yscale = (scale >> 16) & 0x7fff;
    size_t src_size, dst_size;
    g_autofree uint8_t *src = NULL;
    g_autofree uint8_t *dst = NULL;
    MemTxResult mr;
    unsigned x, y;

    if (!xscale) { xscale = 0x1000; }
    if (!yscale) { yscale = 0x1000; }
    if (src_bpp == UINT_MAX || dst_bpp == UINT_MAX ||
        right < left || bottom < top || !src_pitch || !dst_pitch ||
        !out_w || !out_h) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "imx6sl-pxp: unsupported/invalid operation "
                      "src=%#x dst=%#x %ux%u\n",
                      src_fmt, dst_fmt, out_w, out_h);
        return false;
    }

    src_size = (size_t)src_pitch *
               (((uint64_t)(bottom - top + 1) * yscale >> 12) + 2);
    dst_size = (size_t)dst_pitch * out_h;
    if (src_size > PXP_MAX_DMA_BYTES || dst_size > PXP_MAX_DMA_BYTES) {
        return false;
    }
    src = g_malloc0(src_size);
    dst = g_malloc0(dst_size);
    mr = dma_memory_read(&address_space_memory, *pxp_reg(s, PXP_PS_BUF),
                         src, src_size, MEMTXATTRS_UNSPECIFIED);
    if (mr != MEMTX_OK) {
        *pxp_reg(s, PXP_STAT) |= STAT_AXI_READ;
        return false;
    }
    /* Preserve bytes outside the programmed process-surface rectangle. */
    mr = dma_memory_read(&address_space_memory, *pxp_reg(s, PXP_OUT_BUF),
                         dst, dst_size, MEMTXATTRS_UNSPECIFIED);
    if (mr != MEMTX_OK) {
        memset(dst, 0, dst_size);
    }

    for (y = top; y <= bottom && y < out_h; y++) {
        for (x = left; x <= right && x < out_w; x++) {
            unsigned dx = x - left, dy = y - top;
            unsigned sx = ((uint64_t)dx * xscale) >> 12;
            unsigned sy = ((uint64_t)dy * yscale) >> 12;
            unsigned tx = x, ty = y;
            const uint8_t *sp;
            uint8_t *dp;
            uint8_t lum;

            if (ctrl & CTRL_HFLIP) { sx = right - left - sx; }
            if (ctrl & CTRL_VFLIP) { sy = bottom - top - sy; }
            switch ((ctrl & CTRL_ROTATE_MASK) >> 8) {
            case 1: tx = out_w - 1 - y; ty = x; break;
            case 2: tx = out_w - 1 - x; ty = out_h - 1 - y; break;
            case 3: tx = y; ty = out_h - 1 - x; break;
            default: break;
            }
            if (tx >= out_w || ty >= out_h ||
                (size_t)sy * src_pitch >= src_size) {
                continue;
            }
            sp = src + (size_t)sy * src_pitch +
                 (src_bpp ? sx * src_bpp : sx / 2);
            dp = dst + (size_t)ty * dst_pitch +
                 (dst_bpp ? tx * dst_bpp : tx / 2);
            if (sp >= src + src_size || dp >= dst + dst_size) {
                continue;
            }
            lum = pxp_luma(sp, src_fmt, sx);
            if (!(*pxp_reg(s, PXP_LUT_CTRL) & LUT_BYPASS)) {
                lum = s->lut[lum];
            }
            pxp_store_gray(dp, dst_fmt, tx, lum);
        }
    }

    mr = dma_memory_write(&address_space_memory, *pxp_reg(s, PXP_OUT_BUF),
                          dst, dst_size, MEMTXATTRS_UNSPECIFIED);
    if (mr != MEMTX_OK) {
        *pxp_reg(s, PXP_STAT) |= STAT_AXI_WRITE;
        return false;
    }
    *pxp_reg(s, PXP_HIST_CTRL) =
        (*pxp_reg(s, PXP_HIST_CTRL) & ~0xfU) | 0xf;
    return true;
}

static void imx6sl_pxp_complete(void *opaque)
{
    IMX6SLPXPState *s = opaque;

    if (!s->running) {
        return;
    }
    pxp_do_dma(s);
    s->running = false;
    *pxp_reg(s, PXP_CTRL) &= ~CTRL_ENABLE;
    *pxp_reg(s, PXP_STAT) |= STAT_IRQ;
    imx6sl_pxp_update_irq(s);
}

static void imx6sl_pxp_start(IMX6SLPXPState *s)
{
    s->running = true;
    /*
     * Finish after the enabling MMIO transaction returns.  A zero-delay
     * virtual timer preserves the hardware's asynchronous IRQ edge without
     * introducing enough emulated latency for the dispatch kthread to park
     * before completion becomes runnable.
     */
    timer_mod(&s->completion_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static bool pxp_alias_base(hwaddr offset, hwaddr *base, unsigned *op)
{
    hwaddr candidate = offset & ~0xfULL;

    if ((candidate == PXP_CTRL || candidate == PXP_STAT ||
         candidate == PXP_OUT_CTRL || candidate == PXP_PS_CTRL) &&
        (offset & 0xf) <= 0xc && !(offset & 3)) {
        *base = candidate;
        *op = (offset & 0xf) >> 2;
        return true;
    }
    return false;
}

static uint64_t imx6sl_pxp_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX6SLPXPState *s = opaque;
    hwaddr base;
    unsigned op;

    if (pxp_alias_base(offset, &base, &op) && op) {
        return 0;
    }
    return *pxp_reg(s, offset);
}

static void imx6sl_pxp_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    IMX6SLPXPState *s = opaque;
    hwaddr base;
    unsigned op;
    uint32_t old, val = value;

    if (offset == PXP_IRQ_SET || offset == PXP_IRQ_CLR ||
        offset == PXP_IRQ_TOG) {
        uint32_t *irq = pxp_reg(s, PXP_IRQ);

        if (offset == PXP_IRQ_SET) {
            *irq |= val;
        } else if (offset == PXP_IRQ_CLR) {
            *irq &= ~val;
        } else {
            *irq ^= val;
        }
        return;
    }

    if (offset == PXP_CTRL2) {
        *pxp_reg(s, offset) = val;
        if ((val & (CTRL2_ENABLE | CTRL2_WFE_B)) ==
            (CTRL2_ENABLE | CTRL2_WFE_B)) {
            pxp_wfe_b_fill(s);
            *pxp_reg(s, PXP_IRQ) |= IRQ_WFB_CH0;
        }
        return;
    }

    if (offset == PXP_VERSION) {
        return;
    }
    if (offset == PXP_LUT_DATA) {
        unsigned i;
        for (i = 0; i < 4; i++) {
            s->lut[(s->lut_addr + i) & 0xff] = val >> (i * 8);
        }
        s->lut_addr = (s->lut_addr + 4) & 0x3fff;
        *pxp_reg(s, offset) = val;
        return;
    }
    if (offset == PXP_LUT_ADDR) {
        s->lut_addr = val & 0x3fff;
    }

    if (!pxp_alias_base(offset, &base, &op)) {
        *pxp_reg(s, offset) = val;
        return;
    }
    old = *pxp_reg(s, base);
    switch (op) {
    case 0: old = val; break;
    case 1: old |= val; break;
    case 2: old &= ~val; break;
    case 3: old ^= val; break;
    }
    if (base == PXP_CTRL && (old & CTRL_SFTRST)) {
        timer_del(&s->completion_timer);
        memset(s->regs, 0, sizeof(s->regs));
        s->regs[PXP_VERSION >> 2] = 0x02000000;
        s->regs[PXP_CTRL >> 2] = CTRL_SFTRST | CTRL_CLKGATE;
        s->running = false;
        qemu_set_irq(s->irq, 0);
        return;
    }
    *pxp_reg(s, base) = old;
    if (base == PXP_CTRL) {
        if ((old & CTRL_ENABLE) && !s->running && !(old & CTRL_CLKGATE)) {
            imx6sl_pxp_start(s);
        } else if (!(old & CTRL_ENABLE) && s->running) {
            timer_del(&s->completion_timer);
            s->running = false;
        }
    }
    if (base == PXP_CTRL || base == PXP_STAT) {
        imx6sl_pxp_update_irq(s);
    }
}

static const MemoryRegionOps imx6sl_pxp_ops = {
    .read = imx6sl_pxp_read,
    .write = imx6sl_pxp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4,
               .unaligned = false },
};

static void imx6sl_pxp_reset(DeviceState *dev)
{
    IMX6SLPXPState *s = IMX6SL_PXP(dev);

    timer_del(&s->completion_timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->lut, 0, sizeof(s->lut));
    s->regs[PXP_VERSION >> 2] = 0x02000000;
    s->lut_addr = 0;
    s->running = false;
    qemu_set_irq(s->irq, 0);
}

static const VMStateDescription vmstate_imx6sl_pxp = {
    .name = TYPE_IMX6SL_PXP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMX6SLPXPState,
                             IMX6SL_PXP_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT8_ARRAY(lut, IMX6SLPXPState, 256),
        VMSTATE_UINT16(lut_addr, IMX6SLPXPState),
        VMSTATE_BOOL(running, IMX6SLPXPState),
        VMSTATE_TIMER(completion_timer, IMX6SLPXPState),
        VMSTATE_END_OF_LIST()
    },
};

static void imx6sl_pxp_init(Object *obj)
{
    IMX6SLPXPState *s = IMX6SL_PXP(obj);

    memory_region_init_io(&s->iomem, obj, &imx6sl_pxp_ops, s,
                          TYPE_IMX6SL_PXP, IMX6SL_PXP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    timer_init_ns(&s->completion_timer, QEMU_CLOCK_VIRTUAL,
                  imx6sl_pxp_complete, s);
}

static void imx6sl_pxp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, imx6sl_pxp_reset);
    dc->vmsd = &vmstate_imx6sl_pxp;
    dc->desc = "i.MX6 SoloLite enhanced Pixel Pipeline v2";
}

static const TypeInfo imx6sl_pxp_info = {
    .name = TYPE_IMX6SL_PXP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX6SLPXPState),
    .instance_init = imx6sl_pxp_init,
    .class_init = imx6sl_pxp_class_init,
};

static void imx6sl_pxp_register_types(void)
{
    type_register_static(&imx6sl_pxp_info);
}
type_init(imx6sl_pxp_register_types)
