/* reMarkable 2 ED103TC2 panel connected to LCDIF.
 * The LCD bus carries source-driver voltages, not RGB pixels. Decode the
 * eight two-bit source lanes and accumulate their effect on reflectance.
 * Signal layout: reMarkable zero-sugar U-Boot and yobert/swtcon's fb format.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/imx_lcdif.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "ui/console.h"
#define TYPE_RM2_PANEL "remarkable2-panel"
#define PANEL_WIDTH 1404
#define PANEL_HEIGHT 1872
#define PANEL_PIXELS (PANEL_WIDTH * PANEL_HEIGHT)
OBJECT_DECLARE_SIMPLE_TYPE(RM2PanelState, RM2_PANEL)
struct RM2PanelState {
    DeviceState parent_obj;
    IMXLCDIFState *lcdif;
    QemuConsole *console;
    uint8_t reflectance[PANEL_PIXELS];
    bool dirty;
    uint64_t frames, active_frames;
};
static void rm2_panel_frame(void *opaque, hwaddr buffer, unsigned width,
                             unsigned height, unsigned word_length)
{
    RM2PanelState *s = opaque;
    g_autofree uint8_t *wire = NULL;
    unsigned prefix;
    bool changed = false;
    s->frames++;
    if ((height != 1408 && height != 1405) || width < 234 || width > 512 ||
        word_length != 3) {
        return;
    }
    prefix = height - PANEL_WIDTH;
    wire = g_malloc(width * height * 4);
    if (dma_memory_read(&address_space_memory, buffer, wire,
                        width * height * 4, MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return;
    }
    for (unsigned col = 0; col < PANEL_WIDTH; col++) {
        unsigned row = 0;
        const uint8_t *line = wire + (col + prefix) * width * 4;
        for (unsigned cell = 0; cell < width && row < PANEL_HEIGHT; cell++) {
            uint32_t data = ldl_le_p(line + cell * 4);
            if (!(data & BIT(20))) {
                continue;
            }
            if (!(data & 0xffff)) {
                row += 8;
                continue;
            }
            for (unsigned lane = 0; lane < 8 && row < PANEL_HEIGHT; lane++, row++) {
                unsigned cmd = (data >> (14 - 2 * lane)) & 3;
                unsigned pixel = (PANEL_HEIGHT - 1 - row) * PANEL_WIDTH +
                                   PANEL_WIDTH - 1 - col;
                int old = s->reflectance[pixel];
                int value = old;
                if (cmd == 1) {
                    value = MAX(0, old - 32);
                } else if (cmd == 2) {
                    value = MIN(255, old + 32);
                }
                if (value != old) {
                    s->reflectance[pixel] = value;
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
static void rm2_panel_update(void *opaque)
{
    RM2PanelState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->console);
    if (!s->dirty) {
        return;
    }
    for (unsigned y = 0; y < PANEL_HEIGHT; y++) {
        uint32_t *line = (uint32_t *)(surface_data(surface) + y * surface_stride(surface));
        for (unsigned x = 0; x < PANEL_WIDTH; x++) {
            unsigned gray = s->reflectance[y * PANEL_WIDTH + x];
            line[x] = gray * 0x010101;
        }
    }
    dpy_gfx_update_full(s->console);
    s->dirty = false;
}
static void rm2_panel_invalidate(void *opaque)
{
    RM2_PANEL(opaque)->dirty = true;
}
static const GraphicHwOps rm2_panel_graphics = {
    .gfx_update = rm2_panel_update, .invalidate = rm2_panel_invalidate,
};
static void rm2_panel_realize(DeviceState *dev, Error **errp)
{
    RM2PanelState *s = RM2_PANEL(dev);
    if (!s->lcdif) {
        error_setg(errp, "reMarkable 2 panel requires an LCDIF source");
        return;
    }
    s->console = graphic_console_init(dev, 0, &rm2_panel_graphics, s);
    qemu_console_resize(s->console, PANEL_WIDTH, PANEL_HEIGHT);
    imx_lcdif_set_output(s->lcdif, rm2_panel_frame, s);
    memset(s->reflectance, 255, sizeof(s->reflectance));
    s->dirty = true;
}
static int rm2_panel_post_load(void *opaque, int version)
{
    RM2_PANEL(opaque)->dirty = true;
    return 0;
}
static const VMStateDescription rm2_panel_vmstate = {
    .name = TYPE_RM2_PANEL, .version_id = 1, .minimum_version_id = 1,
    .post_load = rm2_panel_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(reflectance, RM2PanelState, PANEL_PIXELS),
        VMSTATE_UINT64(frames, RM2PanelState),
        VMSTATE_UINT64(active_frames, RM2PanelState),
        VMSTATE_END_OF_LIST()
    },
};
static const Property rm2_panel_properties[] = {
    DEFINE_PROP_LINK("lcdif", RM2PanelState, lcdif, TYPE_IMX_LCDIF, IMXLCDIFState *),
};
static void rm2_panel_init(Object *obj)
{
    RM2PanelState *s = RM2_PANEL(obj);
    object_property_add_uint64_ptr(obj, "frames", &s->frames, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "active-frames", &s->active_frames, OBJ_PROP_FLAG_READ);
}
static void rm2_panel_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = rm2_panel_realize;
    dc->vmsd = &rm2_panel_vmstate;
    dc->user_creatable = false;
    device_class_set_props(dc, rm2_panel_properties);
}
static const TypeInfo rm2_panel_info = {
    .name = TYPE_RM2_PANEL, .parent = TYPE_DEVICE,
    .instance_size = sizeof(RM2PanelState), .instance_init = rm2_panel_init,
    .class_init = rm2_panel_class_init,
};
static void rm2_panel_register(void) { type_register_static(&rm2_panel_info); }
type_init(rm2_panel_register)
