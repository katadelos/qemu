/*
 * Allwinner A13 display backend and TCON driving a parallel E Ink panel.
 *
 * The vendor EPDC software composes electrical drive frames in RAM. DEBE
 * fetches palette indices, each expanding to four two-bit panel drives on
 * LCD data pins. Accumulate those drives only while the pins use LCD muxing;
 * this retains the optical image when the driver stops clocks or sends a
 * no-drive frame. No application framebuffer or guest virtual state is read.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/display/allwinner-a13-eink.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"

#define DE_SIZE 0x100000
#define DEBE_BASE 0x60000
#define TCON_SIZE 0x1000
#define FRAME_NS 20000000

OBJECT_DECLARE_SIMPLE_TYPE(AwA13EinkState, AW_A13_EINK)

struct AwA13EinkState {
    SysBusDevice parent_obj;
    MemoryRegion de_mr, tcon_mr;
    uint32_t de[DE_SIZE / 4], tcon[TCON_SIZE / 4];
    uint32_t width, height, rotation, row_start, column_start, panel_size;
    uint8_t initial_grey;
    uint8_t *panel;
    bool dirty, drive_enabled, vblank;
    QemuConsole *console;
    QEMUTimer *frame_timer;
    qemu_irq irq;
};

static void a13_eink_irq(AwA13EinkState *s)
{
    qemu_set_irq(s->irq, !!((s->tcon[1] >> 16) & s->tcon[1] & 0xffff));
}

static bool a13_eink_enabled(AwA13EinkState *s)
{
    return (s->tcon[0] & BIT(31)) && (s->tcon[0x40 / 4] & BIT(31));
}

static void a13_eink_draw(void *opaque)
{
    AwA13EinkState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->console);
    uint8_t *pixels = surface_data(surface);
    unsigned stride = surface_stride(surface);

    if (!s->dirty) {
        return;
    }
    for (unsigned y = 0; y < s->height; y++) {
        for (unsigned x = 0; x < s->width; x++) {
            unsigned dx = x, dy = y;
            uint8_t grey = s->panel[y * s->width + x];

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
            stl_he_p(pixels + dy * stride + dx * 4,
                      rgb_to_pixel32(grey, grey, grey));
        }
    }
    s->dirty = false;
    dpy_gfx_update(s->console, 0, 0, surface_width(surface),
                   surface_height(surface));
}

static void a13_eink_invalidate(void *opaque)
{
    AW_A13_EINK(opaque)->dirty = true;
}

static const GraphicHwOps a13_eink_graphics = {
    .invalidate = a13_eink_invalidate,
    .gfx_update = a13_eink_draw,
};

static void a13_eink_drive(AwA13EinkState *s)
{
    uint32_t *be = &s->de[DEBE_BASE / 4];
    uint32_t pitch = be[0x840 / 4] / 8;
    uint32_t source = (be[0x850 / 4] >> 3) | 0x40000000;
    unsigned groups = s->width / 4;
    uint64_t first = source + (uint64_t)s->row_start * pitch +
                     s->column_start;
    uint64_t last = first + (uint64_t)(s->height - 1) * pitch + groups;
    static const unsigned low_bit[4] = { 11, 7, 5, 3 };
    static const unsigned high_bit[4] = { 12, 10, 6, 4 };

    if (!s->drive_enabled || !(be[0x800 / 4] & 1) ||
        !(be[0x800 / 4] & BIT(8)) || pitch < s->column_start + groups ||
        pitch > 16384 || first < 0x40000000 || last > 0x60000000) {
        return;
    }

    g_autofree uint8_t *row = g_malloc(groups);
    for (unsigned y = 0; y < s->height; y++) {
        address_space_read(&address_space_memory, first + (uint64_t)y * pitch,
                            MEMTXATTRS_UNSPECIFIED, row, groups);
        for (unsigned g = 0; g < groups; g++) {
            uint32_t pins = be[0x5000 / 4 + row[g]];
            for (unsigned p = 0; p < 4; p++) {
                unsigned drive = ((pins >> low_bit[p]) & 1) |
                                 (((pins >> high_bit[p]) & 1) << 1);
                unsigned index = y * s->width + g * 4 + p;
                uint8_t grey = s->panel[index];

                /* The neutral waveform uses one step per grey-level delta. */
                if (drive == 1) {
                    grey = MAX(0, (int)grey - 17);
                } else if (drive == 2) {
                    grey = MIN(255, (int)grey + 17);
                }
                s->dirty |= grey != s->panel[index];
                s->panel[index] = grey;
            }
        }
    }
    a13_eink_draw(s);
}

static void a13_eink_frame(void *opaque)
{
    AwA13EinkState *s = opaque;
    if (!a13_eink_enabled(s)) {
        return;
    }
    if (s->vblank) {
        s->tcon[1] |= BIT(15);
    } else {
        a13_eink_drive(s);
        s->de[(DEBE_BASE + 0x870) / 4] &= ~1;
        s->tcon[1] |= BIT(13);
    }
    a13_eink_irq(s);
    s->vblank = !s->vblank;
    /* Separate line and VBI IRQs: the stock handler services one per entry. */
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (s->vblank ? 1000000 : FRAME_NS - 1000000));
}

static void a13_eink_drive_enable(void *opaque, int line, int level)
{
    AW_A13_EINK(opaque)->drive_enabled = !!level;
}

static uint64_t a13_de_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_A13_EINK(opaque)->de[offset / 4];
}

static void a13_de_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
    AW_A13_EINK(opaque)->de[offset / 4] = value;
}

static uint64_t a13_tcon_read(void *opaque, hwaddr offset, unsigned size)
{
    return AW_A13_EINK(opaque)->tcon[offset / 4];
}

static void a13_tcon_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    AwA13EinkState *s = opaque;
    if (offset == 4) {
        s->tcon[1] = (value & 0xffff0000) | (s->tcon[1] & value & 0xffff);
    } else {
        s->tcon[offset / 4] = value;
    }
    if (a13_eink_enabled(s)) {
        if (!timer_pending(s->frame_timer)) {
            s->vblank = true;
            timer_mod(s->frame_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
        }
    } else {
        timer_del(s->frame_timer);
    }
    a13_eink_irq(s);
}

#define A13_DISPLAY_OPS(name) \
static const MemoryRegionOps a13_##name##_ops = { \
    .read = a13_##name##_read, .write = a13_##name##_write, \
    .endianness = DEVICE_LITTLE_ENDIAN, \
    .valid = { .min_access_size = 4, .max_access_size = 4 }, \
}
A13_DISPLAY_OPS(de);
A13_DISPLAY_OPS(tcon);

static void a13_eink_reset(DeviceState *dev)
{
    AwA13EinkState *s = AW_A13_EINK(dev);
    memset(s->de, 0, sizeof(s->de));
    memset(s->tcon, 0, sizeof(s->tcon));
    memset(s->panel, s->initial_grey, s->panel_size);
    s->drive_enabled = s->vblank = false;
    s->dirty = true;
    timer_del(s->frame_timer);
    a13_eink_irq(s);
}

static int a13_eink_post_load(void *opaque, int version_id)
{
    AwA13EinkState *s = opaque;
    s->dirty = true;
    a13_eink_irq(s);
    return 0;
}

static const VMStateDescription a13_eink_vmstate = {
    .name = TYPE_AW_A13_EINK,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = a13_eink_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(de, AwA13EinkState, DE_SIZE / 4),
        VMSTATE_UINT32_ARRAY(tcon, AwA13EinkState, TCON_SIZE / 4),
        VMSTATE_BOOL(drive_enabled, AwA13EinkState),
        VMSTATE_BOOL(vblank, AwA13EinkState),
        VMSTATE_VBUFFER_UINT32(panel, AwA13EinkState, 1, NULL, panel_size),
        VMSTATE_TIMER_PTR(frame_timer, AwA13EinkState),
        VMSTATE_END_OF_LIST()
    },
};

static void a13_eink_realize(DeviceState *dev, Error **errp)
{
    AwA13EinkState *s = AW_A13_EINK(dev);
    if (!s->width || s->width % 4 || !s->height || s->width > 4096 ||
        s->height > 4096 || s->row_start > 4096 || s->column_start > 4096 ||
        (s->rotation != 0 && s->rotation != 90 && s->rotation != 180 &&
         s->rotation != 270)) {
        error_setg(errp, "A13 E Ink requires valid panel dimensions and rotation");
        return;
    }
    s->panel_size = s->width * s->height;
    s->panel = g_malloc(s->panel_size);
    s->console = graphic_console_init(dev, 0, &a13_eink_graphics, s);
    bool portrait = s->rotation == 90 || s->rotation == 270;
    qemu_console_resize(s->console, portrait ? s->height : s->width,
                         portrait ? s->width : s->height);
}

static void a13_eink_init(Object *obj)
{
    AwA13EinkState *s = AW_A13_EINK(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(s);
    memory_region_init_io(&s->de_mr, obj, &a13_de_ops, s, "a13.de", DE_SIZE);
    memory_region_init_io(&s->tcon_mr, obj, &a13_tcon_ops, s, "a13.tcon", TCON_SIZE);
    sysbus_init_mmio(dev, &s->de_mr);
    sysbus_init_mmio(dev, &s->tcon_mr);
    sysbus_init_irq(dev, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), a13_eink_drive_enable, "drive-enable", 1);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, a13_eink_frame, s);
}

static void a13_eink_finalize(Object *obj)
{
    AwA13EinkState *s = AW_A13_EINK(obj);
    timer_free(s->frame_timer);
    g_free(s->panel);
}

static const Property a13_eink_properties[] = {
    DEFINE_PROP_UINT32("width", AwA13EinkState, width, 1024),
    DEFINE_PROP_UINT32("height", AwA13EinkState, height, 758),
    DEFINE_PROP_UINT32("rotation", AwA13EinkState, rotation, 90),
    /* Optical state handed over by the bootloader to the software EPDC. */
    DEFINE_PROP_UINT8("initial-grey", AwA13EinkState, initial_grey, 0xff),
    /* Native 1024-panel timings: FSL2+FBL4-1, HSL6+HBL6. */
    DEFINE_PROP_UINT32("row-start", AwA13EinkState, row_start, 5),
    DEFINE_PROP_UINT32("column-start", AwA13EinkState, column_start, 12),
};

static void a13_eink_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = a13_eink_realize;
    dc->vmsd = &a13_eink_vmstate;
    device_class_set_legacy_reset(dc, a13_eink_reset);
    device_class_set_props(dc, a13_eink_properties);
}

static const TypeInfo a13_eink_type = {
    .name = TYPE_AW_A13_EINK,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AwA13EinkState),
    .instance_init = a13_eink_init,
    .instance_finalize = a13_eink_finalize,
    .class_init = a13_eink_class_init,
};

static void a13_eink_register_types(void)
{
    type_register_static(&a13_eink_type);
}
type_init(a13_eink_register_types)
