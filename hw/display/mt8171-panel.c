/*
 * MT8171 guest framebuffer preview for graphical QEMU frontends.
 *
 * Displays actual MDP input DMA pixels, placed using the configured WROT
 * destination and HWTCON image-buffer geometry. This is a diagnostic preview
 * before CFA, REGAL, waveform selection and physical panel driving. It does
 * not acknowledge those operations or turn unsupported display jobs into
 * completed jobs. No invented splash screen or boot-blank filtering is used.
 *
 * Source geometry/packing: vendor mtk_mdp_{rdma,wrot,path}.c and
 * hwtcon/hal/hwtcon_pipeline_config.c. Panel size: signed PA6-CS8 DT.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/display/mt8171-panel.h"
#include "hw/display/mt8171-hwtcon.h"
#include "hw/core/qdev-properties.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "system/address-spaces.h"

struct MT8171PanelTile {
    QTAILQ_ENTRY(MT8171PanelTile) next;
    hwaddr destination;
    unsigned width, height, pitch, bpp;
    size_t bytes;
    uint32_t pixels[];
};

static void panel_clear_surface(MT8171PanelState *s)
{
    DisplaySurface *surface = qemu_console_surface(s->console);
    memset(s->pixels, 0, (size_t)s->plane_width * s->plane_height * 4);
    memset(surface_data(surface), 0, (size_t)surface_stride(surface) * surface_height(surface));
    s->dirty = true;
}

static void panel_resize(MT8171PanelState *s, unsigned width, unsigned height)
{
    if (s->plane_width != width || s->plane_height != height) {
        g_free(s->pixels);
        s->pixels = g_new0(uint32_t, (size_t)width * height);
        s->plane_width = width;
        s->plane_height = height;
        s->dirty = true;
    }
    DisplaySurface *surface = qemu_console_surface(s->console);
    if (surface_width(surface) != height || surface_height(surface) != width ||
        surface_is_placeholder(surface)) {
        qemu_console_resize(s->console, height, width);
        s->dirty = true;
    }
}

static void panel_clear_queue(MT8171PanelState *s)
{
    MT8171PanelTile *tile, *next;
    QTAILQ_FOREACH_SAFE(tile, &s->pending, next, next) {
        QTAILQ_REMOVE(&s->pending, tile, next);
        g_free(tile);
    }
    s->queued_bytes = 0;
}

static bool panel_geometry(MT8171PanelState *s, hwaddr *base,
                            unsigned *width, unsigned *height, unsigned *pitch)
{
    uint32_t *r = s->hwtcon_regs;
    uint8_t image_mask;
    if (!r) { return false; }
    *base = s->image_buffer ? s->image_buffer :
            s->observed_image_buffer ? s->observed_image_buffer : r[0xdf08 / 4];
    *width = r[0xd00c / 4] & 0x3fff;
    *height = (r[0xd00c / 4] >> 16) & 0x3fff;
    /* PA6's Y4/Y5 precision still uses a byte-addressed image plane. The
     * preview retains original input pixels; precision is applied by the
     * real HWTCON DMA path, independently of this observer. */
    if (!mt8171_hwtcon_image_mask(r[0xdf10 / 4], &image_mask)) {
        return false;
    }
    *pitch = (r[0xd060 / 4] & (1 << 6)) ?
             r[0xd064 / 4] & 0xffff : *width;
    return *base && *width && *height && *width <= 8192 && *height <= 8192 &&
           *pitch >= *width && (uint64_t)*width * *height * 4 <= 64 * MiB;
}

/* CFA mode5 targets color_buffer_pa, a separate full-panel allocation from
 * img_buffer_pa (hwtcon_mdp.c). Identify that allocation only when actual WROT
 * destinations cover an entire panel in contiguous nonoverlapping full-width
 * strips. A partial update is insufficient evidence to infer its origin. */
static bool panel_observe_complete_plane(MT8171PanelState *s, unsigned width,
                                         unsigned height, unsigned pitch,
                                         hwaddr *base)
{
    MT8171PanelTile *tile;
    hwaddr first = UINT64_MAX;
    uint64_t rows = 0;
    QTAILQ_FOREACH(tile, &s->pending, next) {
        if (tile->bpp != 1 || tile->pitch != pitch || tile->width != width) {
            return false;
        }
        first = MIN(first, tile->destination);
        rows += tile->height;
    }
    if (rows != height) { return false; }
    g_autofree bool *covered = g_new0(bool, height);
    QTAILQ_FOREACH(tile, &s->pending, next) {
        uint64_t offset = tile->destination - first;
        uint64_t y = offset / pitch;
        if (offset % pitch || y + tile->height > height) { return false; }
        for (unsigned i = 0; i < tile->height; i++) {
            if (covered[y + i]) { return false; }
            covered[y + i] = true;
        }
    }
    s->observed_image_buffer = *base = first;
    qemu_log_mask(LOG_GUEST_ERROR,
        "mt8171-framebuffer-preview: observed complete pre-CFA plane "
        "base=%" HWADDR_PRIx " size=%ux%u stride=%u\n",
        first, width, height, pitch);
    return true;
}

static void panel_present_pending(MT8171PanelState *s)
{
    hwaddr base;
    unsigned width, height, pitch;
    MT8171PanelTile *tile, *next;
    if (!s->console || !panel_geometry(s, &base, &width, &height, &pitch)) {
        return;
    }
    /* Once complete-plane observation has established the image allocation,
     * a foreign head must be rejected below, not restart origin discovery and
     * hold every later valid partial update behind it. */
    if (!s->image_buffer && !s->observed_image_buffer &&
        !QTAILQ_EMPTY(&s->pending)) {
        tile = QTAILQ_FIRST(&s->pending);
        if ((tile->destination < base ||
             tile->destination - base >= (uint64_t)pitch * height) &&
            !panel_observe_complete_plane(s, width, height, pitch, &base)) {
            return;
        }
    }
    panel_resize(s, width, height);
    QTAILQ_FOREACH_SAFE(tile, &s->pending, next, next) {
        uint64_t offset = tile->destination - base;
        uint64_t y = offset / pitch;
        uint64_t x = offset % pitch;
        bool valid = tile->bpp == 1 && tile->pitch == pitch &&
                     tile->destination >= base &&
                     x + tile->width <= width && y + tile->height <= height;
        if (valid) {
            for (unsigned row = 0; row < tile->height; row++) {
                uint32_t *out = s->pixels + (y + row) * width + x;
                memcpy(out, tile->pixels + (size_t)row * tile->width,
                       tile->width * sizeof(*out));
            }
            s->presented_tiles++;
            s->dirty = true;
        } else {
            s->rejected_tiles++;
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt8171-framebuffer-preview: tile outside configured image "
                          "dst=%" HWADDR_PRIx " base=%" HWADDR_PRIx
                          " tile=%ux%u stride=%u image=%ux%u stride=%u\n",
                          tile->destination, base, tile->width, tile->height,
                          tile->pitch, width, height, pitch);
        }
        s->queued_bytes -= tile->bytes;
        QTAILQ_REMOVE(&s->pending, tile, next);
        g_free(tile);
    }
}

static uint32_t panel_unpack(const uint8_t *p, unsigned format, bool swap)
{
    uint8_t red, green, blue;
    switch (format) {
    case 0: {
        unsigned v = lduw_le_p(p);
        red = ((v >> 11) * 255 + 15) / 31;
        green = (((v >> 5) & 63) * 255 + 31) / 63;
        blue = ((v & 31) * 255 + 15) / 31;
        break;
    }
    case 1:
    case 2:
        red = p[2]; green = p[1]; blue = p[0];
        break;
    case 3:
        red = p[1]; green = p[2]; blue = p[3];
        break;
    default: /* Y8 */
        red = green = blue = p[0];
        break;
    }
    if (swap) {
        uint8_t t = red; red = blue; blue = t;
    }
    return 0xff000000 | (uint32_t)red << 16 | (uint32_t)green << 8 | blue;
}

void mt8171_panel_capture_mdp(MT8171PanelState *s, const uint8_t *source,
                              unsigned source_pitch, unsigned width,
                              unsigned height, unsigned format, bool swap,
                              unsigned rotation, hwaddr destination,
                              unsigned destination_pitch,
                              unsigned destination_bpp)
{
    static const unsigned source_bpp[8] = { 2, 3, 4, 4, 0, 0, 0, 1 };
    MT8171PanelTile *tile;
    unsigned outw = (rotation & 1) ? height : width;
    unsigned outh = (rotation & 1) ? width : height;
    size_t bytes;
    if (!s || !s->console) { return; }
    if (destination_bpp != 1 || destination_pitch < outw ||
        !source || !width || !height || width > 8192 || height > 8192 ||
        format >= ARRAY_SIZE(source_bpp) || !source_bpp[format] || rotation > 3 ||
        source_pitch < width * source_bpp[format] ||
        (uint64_t)outw * outh * 4 > 64 * MiB) {
        s->rejected_tiles++;
        return;
    }
    bytes = (size_t)outw * outh * sizeof(uint32_t);
    /* Bounded pending storage when the controller has not yet supplied its
     * image-buffer geometry. Oldest observations can be discarded; neither
     * guest memory nor the last displayed frame is changed by this limit. */
    while (!QTAILQ_EMPTY(&s->pending) && s->queued_bytes + bytes > 64 * MiB) {
        tile = QTAILQ_FIRST(&s->pending);
        s->queued_bytes -= tile->bytes;
        QTAILQ_REMOVE(&s->pending, tile, next);
        g_free(tile);
        s->rejected_tiles++;
    }
    tile = g_malloc(sizeof(*tile) + bytes);
    tile->destination = destination;
    tile->width = outw; tile->height = outh;
    tile->pitch = destination_pitch; tile->bpp = destination_bpp;
    tile->bytes = bytes;
    for (unsigned y = 0; y < outh; y++) {
        for (unsigned x = 0; x < outw; x++) {
            unsigned sx, sy;
            switch (rotation) {
            case 0: sx = x; sy = y; break;
            case 1: sx = y; sy = height - x - 1; break;
            case 2: sx = width - x - 1; sy = height - y - 1; break;
            default: sx = width - y - 1; sy = x; break;
            }
            tile->pixels[(size_t)y * outw + x] =
                panel_unpack(source + (size_t)sy * source_pitch +
                             sx * source_bpp[format], format, swap);
        }
    }
    QTAILQ_INSERT_TAIL(&s->pending, tile, next);
    s->queued_bytes += bytes;
    s->captured_tiles++;
    panel_present_pending(s);
}

void mt8171_panel_capture_iova(MT8171PanelState *s, MT8171M4UState *m4u,
                               hwaddr source, unsigned source_pitch,
                               unsigned width, unsigned height,
                               unsigned format, bool swap, unsigned rotation,
                               hwaddr destination, unsigned destination_pitch,
                               unsigned destination_bpp)
{
    static const unsigned source_bpp[8] = { 2, 3, 4, 4, 0, 0, 0, 1 };
    unsigned row_bytes;
    if (!s || !s->console || !m4u) { return; }
    /* Only the uncompressed Y8 image-plane destination belongs in this
     * observer. Reject WB/CSR destinations before any read or queue insertion. */
    if (destination_bpp != 1 ||
        destination_pitch < ((rotation & 1) ? height : width) || rotation > 3 ||
        format >= ARRAY_SIZE(source_bpp) || !source_bpp[format] ||
        !width || !height || width > 8192 || height > 8192 ||
        (uint64_t)width * height * source_bpp[format] > 64 * MiB) {
        s->rejected_tiles++;
        return;
    }
    row_bytes = width * source_bpp[format];
    if (source_pitch < row_bytes) { s->rejected_tiles++; return; }
    g_autofree uint8_t *pixels = g_malloc((size_t)row_bytes * height);
    for (unsigned y = 0; y < height; y++) {
        hwaddr iova = source + (uint64_t)y * source_pitch;
        size_t left = row_bytes;
        uint8_t *out = pixels + (size_t)y * row_bytes;
        while (left) {
            hwaddr pa, translated, length;
            size_t span;
            MemoryRegion *region;
            if (!mt8171_m4u_debug_translate(m4u, 0, iova, &pa, &span)) {
                goto unmapped;
            }
            length = MIN(left, span);
            region = address_space_translate(&address_space_memory, pa,
                       &translated, &length, false, MEMTXATTRS_UNSPECIFIED);
            if (!memory_region_is_ram(region) || !length) {
                goto unmapped;
            }
            memcpy(out, (uint8_t *)memory_region_get_ram_ptr(region) + translated,
                   length);
            iova += length; out += length; left -= length;
        }
    }
    mt8171_panel_capture_mdp(s, pixels, row_bytes, width, height, format, swap,
                             rotation, destination, destination_pitch,
                             destination_bpp);
    return;
unmapped:
    s->rejected_tiles++;
    qemu_log_mask(LOG_GUEST_ERROR,
                  "mt8171-framebuffer-preview: unmapped bank0 source IOVA "
                  "%" HWADDR_PRIx " size=%ux%u stride=%u\n",
                  source, width, height, source_pitch);
}

static void panel_gfx_update(void *opaque)
{
    MT8171PanelState *s = opaque;
    panel_present_pending(s);
    if (s->dirty) {
        DisplaySurface *surface = qemu_console_surface(s->console);
        /* The CS8 controller scans2648x1986. Its portrait mounting is a
         * clockwise quarter turn. Keep tile placement physical and rotate
         * only the frontend surface, after observing the guest pixel plane. */
        for (unsigned y = 0; y < s->plane_width; y++) {
            uint32_t *out = (uint32_t *)((uint8_t *)surface_data(surface) +
                                        (size_t)y * surface_stride(surface));
            for (unsigned x = 0; x < s->plane_height; x++) {
                out[x] = s->pixels[(size_t)(s->plane_height - x - 1) *
                                   s->plane_width + y];
            }
        }
        s->dirty = false;
        dpy_gfx_update_full(s->console);
    }
}
static void panel_invalidate(void *opaque)
{ MT8171_PANEL(opaque)->dirty = true; }
static const GraphicHwOps panel_gfx_ops = {
    .invalidate = panel_invalidate,
    .gfx_update = panel_gfx_update,
};
static void panel_reset(DeviceState *dev)
{
    MT8171PanelState *s = MT8171_PANEL(dev);
    panel_clear_queue(s);
    s->captured_tiles = s->presented_tiles = s->rejected_tiles = 0;
    s->observed_image_buffer = 0;
    if (s->console) { panel_clear_surface(s); }
}
static void panel_realize(DeviceState *dev, Error **errp)
{
    MT8171PanelState *s = MT8171_PANEL(dev);
    if (!s->width || !s->height || s->width > 8192 || s->height > 8192 ||
        (uint64_t)s->width * s->height * 4 > 64 * MiB) {
        error_setg(errp, "Invalid MT8171 preview dimensions");
        return;
    }
    s->console = graphic_console_init(dev, 0, &panel_gfx_ops, s);
    panel_resize(s, s->width, s->height);
    panel_clear_surface(s);
}
static void panel_unrealize(DeviceState *dev)
{
    MT8171PanelState *s = MT8171_PANEL(dev);
    panel_clear_queue(s);
    g_clear_pointer(&s->pixels, g_free);
    graphic_console_close(s->console);
    s->console = NULL;
}
static void panel_init(Object *obj)
{
    MT8171PanelState *s = MT8171_PANEL(obj);
    QTAILQ_INIT(&s->pending);
    object_property_add_uint64_ptr(obj, "captured-tiles", &s->captured_tiles,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "presented-tiles", &s->presented_tiles,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "rejected-tiles", &s->rejected_tiles,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "image-buffer", &s->image_buffer,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint64_ptr(obj, "observed-image-buffer",
                                   &s->observed_image_buffer, OBJ_PROP_FLAG_READ);
    object_property_set_description(obj, "image-buffer",
        "Observed CFA color_buffer_pa for diagnostic tile placement; "
        "zero selects the HWTCON image buffer");
}
static const Property panel_properties[] = {
    DEFINE_PROP_UINT32("width", MT8171PanelState, width, 2648),
    DEFINE_PROP_UINT32("height", MT8171PanelState, height, 1986),
};
static void panel_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = panel_realize;
    dc->unrealize = panel_unrealize;
    dc->desc = "MT8171 guest framebuffer preview before CFA and waveform drive";
    device_class_set_legacy_reset(dc, panel_reset);
    device_class_set_props(dc, panel_properties);
}
static const TypeInfo panel_type = {
    .name = TYPE_MT8171_PANEL, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8171PanelState), .instance_init = panel_init,
    .class_init = panel_class_init,
};
static void panel_register(void) { type_register_static(&panel_type); }
type_init(panel_register);
