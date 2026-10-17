/*
 * Paper Pro Cumulus FPGA and AC118TC1F2 source-driver voltage stream.
 * Eight three-bit lanes drive four columns, alternating upper/lower rows.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/imx_lcdif.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "system/dma.h"
#include "ui/console.h"

#define TYPE_CUMULUS "remarkable3-panel"
#define WIDTH 1620
#define HEIGHT 2160
#define PIXELS (WIDTH * HEIGHT)
OBJECT_DECLARE_SIMPLE_TYPE(CumulusState, CUMULUS)
struct CumulusState {
    SSIPeripheral parent_obj;
    IMXLCDIFState *lcdif;
    QemuConsole *console;
    QEMUTimer erase_timer;
    qemu_irq init_n, done, pmu_feedback;
    int16_t pigment[PIXELS];
    uint8_t command;
    uint32_t position;
    uint64_t frames, active_frames;
    bool configured, pmu_awake, dirty;
};

static void cumulus_frame(void *opaque, hwaddr buffer, unsigned width,
                           unsigned height, unsigned word_length)
{
    CumulusState *s = opaque;
    /* The source-driver codes select +24/+12/+6 and -6/-12/-24 V.
     * Keep pigment travel outside the optically visible interval: clipping
     * reflectance after every pulse discards overdrive and biases the
     * response to the following drive phases. */
    static const int delta[] = {0, 24, 12, 6, -6, -12, -24, 0};
    bool changed = false;
    s->frames++;
    if (!s->configured || !s->pmu_awake || width != 405 ||
        height != 1084 || word_length != 3) {
        return;
    }
    g_autofree uint8_t *wire = g_malloc(width * height * 4);
    if (dma_memory_read(&address_space_memory, buffer, wire, width * height * 4,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return;
    }
    for (unsigned y = 0; y < HEIGHT / 2; y++) {
        const uint8_t *line = wire + (y + 4) * width * 4;
        for (unsigned cell = 0; cell < width; cell++) {
            uint32_t data = ldl_le_p(line + cell * 4);
            if (!(data & 0xffffff)) { continue; }
            for (unsigned lane = 0; lane < 8; lane++) {
                unsigned pixel = (y * 2 + lane % 2) * WIDTH + cell * 4 + lane / 2;
                int old = s->pigment[pixel];
                int value = MIN(240, MAX(-240,
                    old + delta[(data >> (lane * 3)) & 7]));
                if (value != old) {
                    s->pigment[pixel] = value;
                    changed = true;
                }
            }
        }
    }
    if (changed) {
        s->active_frames++;
        s->dirty = true;
    }
}
static void cumulus_update(void *opaque)
{
    CumulusState *s = opaque;
    if (!s->dirty) { return; }
    DisplaySurface *surface = qemu_console_surface(s->console);
    for (unsigned y = 0; y < HEIGHT; y++) {
        uint32_t *line = (uint32_t *)(surface_data(surface) + y * surface_stride(surface));
        for (unsigned x = 0; x < WIDTH; x++) {
            unsigned value = MIN(384, MAX(0,
                s->pigment[y * WIDTH + x] + 192));
            line[x] = (value * 255 / 384) * 0x010101;
        }
    }
    dpy_gfx_update_full(s->console);
    s->dirty = false;
}
static void cumulus_invalidate(void *opaque) { CUMULUS(opaque)->dirty = true; }
static const GraphicHwOps cumulus_graphics = {
    .gfx_update = cumulus_update, .invalidate = cumulus_invalidate,
};
static uint32_t cumulus_transfer(SSIPeripheral *dev, uint32_t value)
{
    CumulusState *s = CUMULUS(dev);
    unsigned position = s->position++;
    if (!position) { s->command = value; }
    if (position >= 4 && position < 8) {
        uint32_t result = s->command == 0xe0 ? 0x010f0043 :
                          s->command == 0x3c && s->configured ? BIT(8) : 0;
        return (result >> (24 - (position - 4) * 8)) & 0xff;
    }
    return 0;
}
static int cumulus_cs(SSIPeripheral *dev, bool level)
{
    CumulusState *s = CUMULUS(dev);
    if (!level) {
        s->position = 0;
    } else if (s->command == 0x7a && s->position > 32) {
        s->configured = true;
        qemu_set_irq(s->done, 1);
    } else if (s->command == 0x26 && s->configured) {
        qemu_set_irq(s->done, 1);
    }
    return 0;
}
static void cumulus_erased(void *opaque)
{
    CumulusState *s = opaque;
    qemu_set_irq(s->init_n, 1);
}
static void cumulus_program(void *opaque, int n, int level)
{
    CumulusState *s = opaque;
    if (!level) {
        s->configured = false;
        qemu_set_irq(s->init_n, 0);
        qemu_set_irq(s->done, 0);
        timer_mod_ns(&s->erase_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
    }
}
static void cumulus_pmu(void *opaque, int n, int level)
{
    CumulusState *s = opaque;
    s->pmu_awake = level;
    qemu_set_irq(s->pmu_feedback, level);
}
static void cumulus_realize(SSIPeripheral *dev, Error **errp)
{
    CumulusState *s = CUMULUS(dev);
    if (!s->lcdif) {
        error_setg(errp, "Cumulus requires an LCDIF source");
        return;
    }
    s->console = graphic_console_init(DEVICE(dev), 0, &cumulus_graphics, s);
    qemu_console_resize(s->console, WIDTH, HEIGHT);
    imx_lcdif_set_output(s->lcdif, cumulus_frame, s);
    for (unsigned i = 0; i < PIXELS; i++) {
        s->pigment[i] = 240;
    }
    s->dirty = true;
    qemu_set_irq(s->init_n, 1);
}
static int cumulus_post_load(void *opaque, int version)
{
    CumulusState *s = CUMULUS(opaque);
    s->dirty = true;
    qemu_set_irq(s->init_n, !timer_pending(&s->erase_timer));
    qemu_set_irq(s->done, s->configured);
    qemu_set_irq(s->pmu_feedback, s->pmu_awake);
    return 0;
}
static void cumulus_reset(DeviceState *dev)
{
    CumulusState *s = CUMULUS(dev);
    s->parent_obj.cs = true;
    s->position = 0;
    s->configured = s->pmu_awake = false;
    timer_del(&s->erase_timer);
    qemu_set_irq(s->init_n, 1);
    qemu_set_irq(s->done, 0);
    qemu_set_irq(s->pmu_feedback, 0);
}
static const VMStateDescription cumulus_vmstate = {
    .name = TYPE_CUMULUS, .version_id = 1, .minimum_version_id = 1,
    .post_load = cumulus_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, CumulusState),
        VMSTATE_INT16_ARRAY(pigment, CumulusState, PIXELS),
        VMSTATE_UINT8(command, CumulusState),
        VMSTATE_UINT32(position, CumulusState),
        VMSTATE_UINT64(frames, CumulusState),
        VMSTATE_UINT64(active_frames, CumulusState),
        VMSTATE_BOOL(configured, CumulusState),
        VMSTATE_BOOL(pmu_awake, CumulusState),
        VMSTATE_TIMER(erase_timer, CumulusState),
        VMSTATE_END_OF_LIST()
    },
};
static const Property cumulus_properties[] = {
    DEFINE_PROP_LINK("lcdif", CumulusState, lcdif, TYPE_IMX_LCDIF, IMXLCDIFState *),
};
static void cumulus_init(Object *obj)
{
    CumulusState *s = CUMULUS(obj);
    DeviceState *dev = DEVICE(obj);
    qdev_init_gpio_in_named(dev, cumulus_program, "program-n", 1);
    qdev_init_gpio_in_named(dev, cumulus_pmu, "pmu", 1);
    qdev_init_gpio_out_named(dev, &s->init_n, "init-n", 1);
    qdev_init_gpio_out_named(dev, &s->done, "done", 1);
    qdev_init_gpio_out_named(dev, &s->pmu_feedback, "pmu-feedback", 1);
    timer_init_ns(&s->erase_timer, QEMU_CLOCK_VIRTUAL, cumulus_erased, s);
    object_property_add_uint64_ptr(obj, "frames", &s->frames, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "active-frames", &s->active_frames, OBJ_PROP_FLAG_READ);
}
static void cumulus_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SSIPeripheralClass *sc = SSI_PERIPHERAL_CLASS(oc);
    dc->vmsd = &cumulus_vmstate;
    device_class_set_legacy_reset(dc, cumulus_reset);
    sc->realize = cumulus_realize;
    sc->transfer = cumulus_transfer;
    sc->set_cs = cumulus_cs;
    sc->cs_polarity = SSI_CS_LOW;
    device_class_set_props(dc, cumulus_properties);
}
static const TypeInfo cumulus_info = {
    .name = TYPE_CUMULUS, .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(CumulusState), .instance_init = cumulus_init,
    .class_init = cumulus_class_init,
};
static void cumulus_register(void) { type_register_static(&cumulus_info); }
type_init(cumulus_register)
