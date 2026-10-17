/*
 * B288 EINK engine, timing controller and DE2 register file.
 *
 * The EINK decoder consumes one transition index per pixel. Its low nibble
 * is the destination grey level, so electrical waveform decoding can be
 * represented directly while preserving DMA, interrupts and frame timing.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/display/allwinner-b288-eink.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"

#define DE_SIZE 0x300000
#define TCON_SIZE 0x2000
#define EINK_SIZE 0x1000
OBJECT_DECLARE_SIMPLE_TYPE(AwB288EinkState, AW_B288_EINK)

struct AwB288EinkState {
    SysBusDevice parent_obj;
    MemoryRegion de_mr, tcon_mr, eink_mr;
    uint32_t de[DE_SIZE / 4];
    uint32_t tcon[TCON_SIZE / 4];
    uint32_t eink[EINK_SIZE / 4];
    uint32_t width, height, rotation, color_filter, panel_size;
    uint8_t *panel;
    uint32_t pending;
    bool dirty;
    QemuConsole *console;
    QEMUTimer *frame_timer, *decode_timer;
    qemu_irq tcon_irq, de_irq, eink_irq;
};

static unsigned b288_eink_color_channel(AwB288EinkState *s, unsigned x,
                                        unsigned y)
{
    if (s->color_filter == 1) {
        /* Original Kaleido, from the vendor's colormask 1 table. */
        return (x / 3 + y) % 3;
    }
    /* Kaleido Plus: the channel bits of the vendor's colormask 2 table.
     * The other bits supply software dithering before EINK indexing; the
     * optical filter has the following six-column, two-row period.
     */
    static const uint8_t channels[2][6] = {
        { 1, 2, 2, 0, 0, 1 },
        { 0, 0, 1, 1, 2, 2 },
    };
    return channels[y & 1][x % 6];
}

static uint32_t b288_eink_color_pixel(AwB288EinkState *s, unsigned x,
                                     unsigned y)
{
    unsigned sum[3] = { 0 }, weight[3] = { 0 };
    unsigned cx = x, cy = y;

    if (s->color_filter == 1) {
        /* Kaleido needs three distinct rows to sample every channel. */
        cx = CLAMP(x, 1, s->width - 2);
        cy = CLAMP(y, 1, s->height - 2);
    }

    /* Reconstruct RGB from the actual, individually driven filter pixels.
     * A three-by-three neighbourhood contains all three channels. Keep
     * filtering in panel coordinates so display rotation does not alter it.
     */
    for (int oy = -1; oy <= 1; oy++) {
        unsigned sy = CLAMP((int)cy + oy, 0, (int)s->height - 1);
        for (int ox = -1; ox <= 1; ox++) {
            unsigned sx = CLAMP((int)cx + ox, 0, (int)s->width - 1);
            unsigned channel = b288_eink_color_channel(s, sx, sy);
            unsigned w = (ox ? 1 : 2) * (oy ? 1 : 2);
            sum[channel] += s->panel[sy * s->width + sx] * w;
            weight[channel] += w;
        }
    }
    return rgb_to_pixel32(sum[0] / weight[0], sum[1] / weight[1],
                          sum[2] / weight[2]);
}

static void b288_eink_update_irq(AwB288EinkState *s)
{
    qemu_set_irq(s->eink_irq, !!((s->eink[1] & 0x10) && (s->eink[1] & 3)));
    qemu_set_irq(s->tcon_irq, !!((s->tcon[1] >> 16) & s->tcon[1] & 0xffff));
}

static void b288_eink_draw(void *opaque)
{
    AwB288EinkState *s = opaque;
    if (!s->dirty) {
        return;
    }
    DisplaySurface *surface = qemu_console_surface(s->console);
    uint8_t *pixels = surface_data(surface);
    unsigned stride = surface_stride(surface);

    for (unsigned y = 0; y < s->height; y++) {
        for (unsigned x = 0; x < s->width; x++) {
            unsigned dx = x, dy = y;
            uint8_t grey = s->panel[y * s->width + x];
            uint32_t pixel = s->color_filter ? b288_eink_color_pixel(s, x, y) :
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
    dpy_gfx_update(s->console, 0, 0, surface_width(surface), surface_height(surface));
    s->dirty = false;
}

static void b288_eink_invalidate(void *opaque)
{
    AW_B288_EINK(opaque)->dirty = true;
    b288_eink_draw(opaque);
}

static const GraphicHwOps b288_eink_graphics = {
    .invalidate = b288_eink_invalidate,
    .gfx_update = b288_eink_draw,
};

static void b288_eink_decode_done(void *opaque)
{
    AwB288EinkState *s = opaque;
    unsigned w = (s->eink[0x0c / 4] & 0xfff) + 1;
    unsigned h = ((s->eink[0x0c / 4] >> 16) & 0xfff) + 1;
    hwaddr source = s->pending == 1 ? s->eink[0x154 / 4] : s->eink[0x18 / 4];
    unsigned pitch = s->pending == 1 ? w : s->eink[0x1c / 4];

    if (w == s->width && h == s->height && pitch >= w &&
        source >= 0x40000000 && source + (uint64_t)pitch * h <= 0x80000000) {
        g_autofree uint8_t *row = g_malloc(w);
        bool changed = false;
        for (unsigned y = 0; y < h; y++) {
            address_space_read(&address_space_memory, source + (uint64_t)pitch * y,
                                MEMTXATTRS_UNSPECIFIED, row, w);
            for (unsigned x = 0; x < w; x++) {
                uint8_t grey = s->pending == 1 ? (row[x] & 15) * 17 : row[x];
                changed |= s->panel[y * w + x] != grey;
                s->panel[y * w + x] = grey;
            }
        }
        s->dirty |= changed;
        b288_eink_draw(s);
    }
    s->eink[1] |= s->pending;
    s->eink[0x150 / 4] &= ~1;
    s->eink[0x08 / 4] &= ~1;
    s->pending = 0;
    b288_eink_update_irq(s);
}

static void b288_eink_frame(void *opaque)
{
    AwB288EinkState *s = opaque;
    if ((s->tcon[0] & BIT(31)) && (s->tcon[0x40 / 4] & BIT(31))) {
        s->tcon[1] |= (s->tcon[1] >> 16) & (BIT(15) | BIT(13));
        b288_eink_update_irq(s);
        timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000000);
    }
}

static uint64_t b288_eink_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_B288_EINK(opaque)->eink[offset / 4];
}

static void b288_eink_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    AwB288EinkState *s = opaque;
    if (offset == 4) {
        s->eink[1] = ((s->eink[1] & 3) & ~value) | (value & 0x10);
    } else {
        s->eink[offset / 4] = value;
    }
    if ((offset == 0x150 || offset == 8) && (value & 1)) {
        s->pending = offset == 0x150 ? 1 : 2;
        timer_mod(s->decode_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
    }
    b288_eink_update_irq(s);
}

static uint64_t b288_tcon_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_B288_EINK(opaque)->tcon[offset / 4];
}

static void b288_tcon_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    AwB288EinkState *s = opaque;
    if (offset == 4) {
        /* TCON interrupt status is cleared by writing zero. */
        s->tcon[1] = (value & 0xffff0000) | (s->tcon[1] & value & 0xffff);
    } else {
        s->tcon[offset / 4] = value;
    }
    if (offset == 0 || offset == 0x40 || offset == 4) {
        if (!timer_pending(s->frame_timer)) {
            timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000000);
        }
    }
    b288_eink_update_irq(s);
}

static uint64_t b288_de_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_B288_EINK(opaque)->de[offset / 4];
}

static void b288_de_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    AW_B288_EINK(opaque)->de[offset / 4] = value;
}

#define B288_DISPLAY_OPS(name) \
static const MemoryRegionOps b288_##name##_ops = { \
    .read = b288_##name##_read, .write = b288_##name##_write, \
    .endianness = DEVICE_LITTLE_ENDIAN, \
    .valid = { .min_access_size = 4, .max_access_size = 4 }, \
}
B288_DISPLAY_OPS(de);
B288_DISPLAY_OPS(tcon);
B288_DISPLAY_OPS(eink);

static void b288_eink_reset(DeviceState *dev)
{
    AwB288EinkState *s = AW_B288_EINK(dev);
    memset(s->de, 0, sizeof(s->de));
    memset(s->tcon, 0, sizeof(s->tcon));
    memset(s->eink, 0, sizeof(s->eink));
    memset(s->panel, 0xff, s->panel_size);
    s->pending = 0;
    s->dirty = true;
    timer_del(s->frame_timer);
    timer_del(s->decode_timer);
    b288_eink_update_irq(s);
}

static int b288_eink_post_load(void *opaque, int version_id)
{
    AwB288EinkState *s = opaque;
    s->dirty = true;
    b288_eink_update_irq(s);
    return 0;
}

static const VMStateDescription b288_eink_vmstate = {
    .name = TYPE_AW_B288_EINK,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = b288_eink_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(de, AwB288EinkState, DE_SIZE / 4),
        VMSTATE_UINT32_ARRAY(tcon, AwB288EinkState, TCON_SIZE / 4),
        VMSTATE_UINT32_ARRAY(eink, AwB288EinkState, EINK_SIZE / 4),
        VMSTATE_UINT32(pending, AwB288EinkState),
        VMSTATE_VBUFFER_UINT32(panel, AwB288EinkState, 1, NULL, panel_size),
        VMSTATE_TIMER_PTR(frame_timer, AwB288EinkState),
        VMSTATE_TIMER_PTR(decode_timer, AwB288EinkState),
        VMSTATE_END_OF_LIST()
    },
};

static void b288_eink_realize(DeviceState *dev, Error **errp)
{
    AwB288EinkState *s = AW_B288_EINK(dev);
    if (!s->width || !s->height || s->width > 4096 || s->height > 4096 ||
        (s->rotation != 0 && s->rotation != 90 && s->rotation != 180 && s->rotation != 270) ||
        s->color_filter > 2 ||
        (s->color_filter == 1 && (s->width < 3 || s->height < 3)) ||
        (s->color_filter == 2 && (s->width < 2 || s->height < 2))) {
        error_setg(errp, "B288 EINK requires valid panel dimensions, rotation and color filter");
        return;
    }
    s->panel_size = s->width * s->height;
    s->panel = g_malloc(s->panel_size);
    s->console = graphic_console_init(dev, 0, &b288_eink_graphics, s);
    bool portrait = s->rotation == 90 || s->rotation == 270;
    qemu_console_resize(s->console, portrait ? s->height : s->width,
                         portrait ? s->width : s->height);
}

static void b288_eink_init(Object *obj)
{
    AwB288EinkState *s = AW_B288_EINK(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(s);
    memory_region_init_io(&s->de_mr, obj, &b288_de_ops, s, "b288.de", DE_SIZE);
    memory_region_init_io(&s->tcon_mr, obj, &b288_tcon_ops, s, "b288.tcon", TCON_SIZE);
    memory_region_init_io(&s->eink_mr, obj, &b288_eink_ops, s, "b288.eink", EINK_SIZE);
    sysbus_init_mmio(dev, &s->de_mr);
    sysbus_init_mmio(dev, &s->tcon_mr);
    sysbus_init_mmio(dev, &s->eink_mr);
    sysbus_init_irq(dev, &s->tcon_irq);
    sysbus_init_irq(dev, &s->de_irq);
    sysbus_init_irq(dev, &s->eink_irq);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, b288_eink_frame, s);
    s->decode_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, b288_eink_decode_done, s);
}

static void b288_eink_finalize(Object *obj)
{
    AwB288EinkState *s = AW_B288_EINK(obj);
    timer_free(s->frame_timer);
    timer_free(s->decode_timer);
    g_free(s->panel);
}

static const Property b288_eink_properties[] = {
    DEFINE_PROP_UINT32("width", AwB288EinkState, width, 0),
    DEFINE_PROP_UINT32("height", AwB288EinkState, height, 0),
    DEFINE_PROP_UINT32("rotation", AwB288EinkState, rotation, 0),
    DEFINE_PROP_UINT32("color-filter", AwB288EinkState, color_filter, 0),
};

static void b288_eink_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = b288_eink_realize;
    dc->vmsd = &b288_eink_vmstate;
    device_class_set_legacy_reset(dc, b288_eink_reset);
    device_class_set_props(dc, b288_eink_properties);
}

static const TypeInfo b288_eink_type = {
    .name = TYPE_AW_B288_EINK,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AwB288EinkState),
    .instance_init = b288_eink_init,
    .instance_finalize = b288_eink_finalize,
    .class_init = b288_eink_class_init,
};

static void b288_eink_register_types(void)
{
    type_register_static(&b288_eink_type);
}
type_init(b288_eink_register_types)
