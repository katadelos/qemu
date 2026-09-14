/*
 * Focaltech FT3A81 touch application and read-only bootloader transport.
 * Sources: stock device_module/drivers/input/touchscreen/ft3a81/
 * focaltech_{core,common,config,flash}.{c,h}, upgrade_ft3a81.c.
 *
 * The virtual variant is explicitly configurable, not identified from a
 * physical CS8 controller. Application/boot IDs56d2/56c2 are driver constants;
 * application version is loaded from the original firmware asset at0x10e.
 * Factory module config is a separate modeled record containing the chosen
 * vendor ID and complement. The update asset does not expose a trustworthy
 * factory record at the bootloader's0x1f80 address.
 *
 * Host input supplies up to two contacts, matching the signed CS8 DT.
 * Programming/erase/ECC commands are
 * unsupported and NACKed; firmware data and reported version never change in
 * response to an unperformed upgrade. Scan timing is a nominal20ms model,
 * with80ms reset recovery matching the driver's bootloader reset delay.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/focaltech-ft3a81.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "ui/input.h"

#define FTS_SCAN_NS 20000000
#define FTS_QUEUE_SIZE 64
typedef struct FTSContact {
    int x, y;
    bool down;
} FTSContact;
typedef struct FTSReport {
    uint8_t data[14], length, mask;
} FTSReport;

OBJECT_DECLARE_SIMPLE_TYPE(FocaltechFT3A81State, FOCALTECH_FT3A81)
struct FocaltechFT3A81State {
    I2CSlave parent_obj;
    qemu_irq irq;
    char *firmware_image;
    uint8_t *firmware;
    size_t firmware_size;
    uint8_t vendor_id, regs[256], factory_config[128];
    uint16_t resolution_x, resolution_y;
    uint8_t tx[16], reply[4], pointer;
    unsigned tx_len, reply_len, reply_pos;
    uint32_t flash_pointer;
    bool bootloader, flash_read, reset_level, upgrade_armed, boot_started;
    int64_t ready_ns, scan_epoch_ns;
    uint64_t commands, flash_reads, rejected_commands;
    QemuInputHandlerState *input_handler;
    QEMUTimer *scan_timer;
    FTSContact input[2], sampled[2];
    FTSReport reports[FTS_QUEUE_SIZE];
    unsigned report_head, report_count, report_length, report_read;
    bool report_active, invert_x, invert_y;
    uint64_t touch_reports, touch_reads, touch_presses, touch_releases;
    uint64_t touch_overruns;
};

static bool fts_ready(FocaltechFT3A81State *s)
{
    return s->reset_level &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= s->ready_ns;
}

static bool fts_scanning(FocaltechFT3A81State *s)
{
    return fts_ready(s) && !s->bootloader && s->regs[0xa5] != 3;
}

static void fts_clear_contacts(FocaltechFT3A81State *s)
{
    timer_del(s->scan_timer);
    memset(s->input, 0, sizeof(s->input));
    memset(s->sampled, 0, sizeof(s->sampled));
    s->report_head = s->report_count = s->report_read = 0;
    s->report_active = false;
    s->regs[1] = s->regs[2] = 0;
    memset(s->regs + 3, 0xff, 60);
    qemu_set_irq(s->irq, 1);
}

static void fts_scan(void *opaque)
{
    FocaltechFT3A81State *s = opaque;
    if (!fts_scanning(s) || s->report_active || !s->report_count) {
        return;
    }
    FTSReport *r = &s->reports[s->report_head];
    memset(s->regs + 1, 0xff, 62);
    memcpy(s->regs + 1, r->data, r->length);
    s->report_length = r->length;
    s->report_read = 0;
    s->report_active = true;
    s->touch_reports++;
    qemu_set_irq(s->irq, 0);
}

static void fts_schedule_scan(FocaltechFT3A81State *s)
{
    if (!s->report_active && s->report_count && !timer_pending(s->scan_timer)) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t elapsed = MAX(now - s->scan_epoch_ns, 0);
        timer_mod(s->scan_timer, s->scan_epoch_ns +
                  (elapsed / FTS_SCAN_NS + 1) * FTS_SCAN_NS);
    }
}

static void fts_input_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    FocaltechFT3A81State *s = FOCALTECH_FT3A81(dev);
    unsigned slot = 0;
    InputAxis axis;
    int value;
    if (!fts_scanning(s)) { return; }
    switch (evt->type) {
    case INPUT_EVENT_KIND_BTN:
        if (evt->u.btn.data->button == INPUT_BUTTON_LEFT) {
            s->input[0].down = evt->u.btn.data->down;
        }
        return;
    case INPUT_EVENT_KIND_ABS:
        axis = evt->u.abs.data->axis;
        value = evt->u.abs.data->value;
        break;
    case INPUT_EVENT_KIND_MTT: {
        InputMultiTouchEvent *m = evt->u.mtt.data;
        if (m->slot < 0 || m->slot >= ARRAY_SIZE(s->input)) { return; }
        slot = m->slot;
        if (m->type == INPUT_MULTI_TOUCH_TYPE_BEGIN) {
            s->input[slot].down = true;
        } else if (m->type == INPUT_MULTI_TOUCH_TYPE_END ||
                   m->type == INPUT_MULTI_TOUCH_TYPE_CANCEL) {
            s->input[slot].down = false;
        }
        if (m->type != INPUT_MULTI_TOUCH_TYPE_DATA) { return; }
        axis = m->axis;
        value = m->value;
        break;
    }
    default:
        return;
    }
    value = CLAMP(value, INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX);
    if (axis == INPUT_AXIS_X) { s->input[slot].x = value; }
    if (axis == INPUT_AXIS_Y) { s->input[slot].y = value; }
}

static void fts_input_sync(DeviceState *dev)
{
    FocaltechFT3A81State *s = FOCALTECH_FT3A81(dev);
    FTSReport r = { .length = 2 };
    bool changed = false, transition = false;
    if (!fts_scanning(s)) { return; }
    for (unsigned i = 0; i < ARRAY_SIZE(s->input); i++) {
        FTSContact *c = &s->input[i], *p = &s->sampled[i];
        changed |= c->down != p->down ||
                   (c->down && (c->x != p->x || c->y != p->y));
        transition |= c->down != p->down;
        if (c->down) { r.mask |= 1 << i; r.data[1]++; }
        if (!c->down && !p->down) { continue; }
        /* DT x/y maxima are inclusive. Invert the physical sensor mounting;
         * the original driver applies its DT flips back into portrait space. */
        unsigned x = qemu_input_scale_axis(c->x, 0, INPUT_EVENT_ABS_MAX,
                                           0, s->resolution_x - 1);
        unsigned y = qemu_input_scale_axis(c->y, 0, INPUT_EVENT_ABS_MAX,
                                           0, s->resolution_y - 1);
        if (s->invert_x) { x = s->resolution_x - x; }
        if (s->invert_y) { y = s->resolution_y - y; }
        unsigned event = c->down ? (p->down ? 2 : 0) : 1;
        uint8_t *b = r.data + r.length;
        b[0] = event << 6 | x >> 8;
        b[1] = x;
        b[2] = i << 4 | y >> 8;
        b[3] = y;
        b[4] = 63; /* Nominal finger pressure/area; no palm or pen inference. */
        b[5] = 32;
        r.length += 6;
    }
    if (!changed) { return; }
    unsigned tail = (s->report_head + s->report_count) % FTS_QUEUE_SIZE;
    if (!transition && s->report_count > (unsigned)s->report_active) {
        unsigned last = (tail + FTS_QUEUE_SIZE - 1) % FTS_QUEUE_SIZE;
        /* Coalesce only consecutive motion snapshots, never down/up edges. */
        bool last_motion = true;
        for (unsigned i = 2; i < s->reports[last].length; i += 6) {
            last_motion &= (s->reports[last].data[i] >> 6) == 2;
        }
        if (last_motion && s->reports[last].mask == r.mask) {
            s->reports[last] = r;
            memcpy(s->sampled, s->input, sizeof(s->input));
            return;
        }
    }
    if (s->report_count == FTS_QUEUE_SIZE) { s->touch_overruns++; return; }
    for (unsigned i = 0; i < ARRAY_SIZE(s->input); i++) {
        s->touch_presses += s->input[i].down && !s->sampled[i].down;
        s->touch_releases += !s->input[i].down && s->sampled[i].down;
    }
    s->reports[tail] = r;
    s->report_count++;
    memcpy(s->sampled, s->input, sizeof(s->input));
    fts_schedule_scan(s);
}

static const QemuInputHandler fts_input_handler = {
    .name = "FT3A81 touchscreen",
    .mask = INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_MTT,
    .event = fts_input_event, .sync = fts_input_sync,
};

static void fts_application(FocaltechFT3A81State *s)
{
    fts_clear_contacts(s);
    memset(s->regs, 0, sizeof(s->regs));
    /* No contacts: event type/default count0, unused contact IDs0xf. */
    memset(s->regs + 3, 0xff, 60);
    s->regs[0xa3] = 0x56;
    s->regs[0x9f] = 0xd2;
    s->regs[0xa6] = s->firmware ? s->firmware[0x10e] : 0;
    s->regs[0xa8] = s->vendor_id;
    s->regs[0xb4] = 0; /* normal application, no IDE parameter update */
    s->bootloader = s->flash_read = s->upgrade_armed = s->boot_started = false;
    s->reply_len = s->reply_pos = s->tx_len = 0;
    s->pointer = 0;
    s->scan_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->ready_ns = s->scan_epoch_ns + 80000000;
    qemu_set_irq(s->irq, 1);
}

static void fts_command(FocaltechFT3A81State *s)
{
    uint8_t *t = s->tx;
    unsigned n = s->tx_len;
    if (!n) { return; }
    s->commands++;
    s->tx_len = s->reply_len = s->reply_pos = 0;
    s->flash_read = false;
    if (n == 3 && t[0] == 0xeb && t[1] == 0xaa && t[2] == 9) {
        memcpy(s->reply, "\xeb\xaa\x08", 3);
        s->reply_len = 3;
        return;
    }
    if (s->bootloader) {
        if (t[0] == 0x55 && (n == 1 || (n == 2 && t[1] == 0xaa))) {
            s->boot_started = true;
        } else if (t[0] == 0x90 && s->boot_started) {
            s->reply[0] = 0x56; s->reply[1] = 0xc2;
            s->reply_len = 2;
        } else if (t[0] == 3 && n == 4) {
            s->flash_pointer = (uint32_t)t[1] << 16 | t[2] << 8 | t[3];
            s->flash_read = true;
            s->flash_reads++;
        } else if (t[0] == 7 && n == 1) {
            fts_application(s);
        } else if (t[0] == 0x6a && n == 1) {
            /* NOP; no erase/write/ECC was performed. */
            s->reply[0] = s->reply[1] = 0;
            s->reply_len = 2;
        }
        return;
    }
    s->pointer = t[0];
    for (unsigned i = 1; i < n; i++) {
        uint8_t reg = t[0] + i - 1, value = t[i];
        if (reg == 0xfc) {
            if (value == 0x55 && s->upgrade_armed) {
                s->bootloader = true;
                fts_clear_contacts(s);
                s->upgrade_armed = false;
                s->ready_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 80000000;
            } else {
                s->upgrade_armed = value == 0xaa;
            }
        } else if (reg != 0xa3 && reg != 0x9f && reg != 0xa6 &&
                   reg != 0xa8 && reg != 0x8f && reg != 0x91 &&
                   reg != 0xe3 && reg != 0xe4 && reg > 0x3f) {
            s->regs[reg] = value;
            if (reg == 0xa5 && value == 3) { fts_clear_contacts(s); }
        }
    }
}

static int fts_send(I2CSlave *dev, uint8_t value)
{
    FocaltechFT3A81State *s = FOCALTECH_FT3A81(dev);
    if (s->tx_len >= sizeof(s->tx)) { return -1; }
    if (!s->tx_len && s->bootloader && value != 0x55 && value != 0x90 &&
        value != 3 && value != 7 && value != 0x6a && value != 0xeb) {
        s->rejected_commands++;
        qemu_log_mask(LOG_UNIMP, "FT3A81 unsupported bootloader command %#x\n", value);
        return -1;
    }
    s->tx[s->tx_len++] = value;
    return 0;
}

static uint8_t fts_recv(I2CSlave *dev)
{
    FocaltechFT3A81State *s = FOCALTECH_FT3A81(dev);
    if (s->reply_pos < s->reply_len) { return s->reply[s->reply_pos++]; }
    if (s->flash_read) {
        uint32_t address = s->flash_pointer++;
        if (address >= 0x1f80 && address < 0x2000) {
            return s->factory_config[address - 0x1f80];
        }
        return address < s->firmware_size ? s->firmware[address] : 0xff;
    }
    if (s->bootloader) { return 0; }
    uint8_t reg = s->pointer++;
    if (s->report_active && reg >= 1 && reg <= s->report_length) {
        if (reg == s->report_read + 1) { s->report_read = reg; }
    }
    if (reg == 0x91 && s->regs[0xa5] != 3) {
        return (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->scan_epoch_ns) / 20000000;
    }
    return s->regs[reg];
}

static int fts_event(I2CSlave *dev, enum i2c_event event)
{
    FocaltechFT3A81State *s = FOCALTECH_FT3A81(dev);
    switch (event) {
    case I2C_START_SEND:
        s->tx_len = 0;
        return fts_ready(s) ? 0 : -1;
    case I2C_START_RECV:
        fts_command(s);
        return fts_ready(s) ? 0 : -1;
    case I2C_FINISH:
        fts_command(s);
        if (s->report_active && s->report_read == s->report_length) {
            s->report_active = false;
            s->touch_reads++;
            s->report_head = (s->report_head + 1) % FTS_QUEUE_SIZE;
            s->report_count--;
            qemu_set_irq(s->irq, 1);
            fts_schedule_scan(s);
        }
        break;
    default:
        break;
    }
    return 0;
}

static void fts_reset_input(void *opaque, int pin, int level)
{
    FocaltechFT3A81State *s = opaque;
    if (!s->reset_level && level) { fts_application(s); }
    s->reset_level = !!level;
    if (!level) { fts_clear_contacts(s); }
}

static void fts_reset(DeviceState *dev)
{
    FocaltechFT3A81State *s = FOCALTECH_FT3A81(dev);
    s->reset_level = true;
    s->commands = s->flash_reads = s->rejected_commands = 0;
    s->touch_reports = s->touch_reads = s->touch_presses = s->touch_releases = 0;
    s->touch_overruns = 0;
    fts_application(s);
}

static void fts_realize(DeviceState *dev, Error **errp)
{
    FocaltechFT3A81State *s = FOCALTECH_FT3A81(dev);
    g_autofree char *data = NULL;
    gsize length;
    if (!s->resolution_x || !s->resolution_y ||
        s->resolution_x > 4095 || s->resolution_y > 4095) {
        error_setg(errp, "FT3A81 resolution must fit its 12-bit coordinate fields");
        return;
    }
    if (!s->firmware_image || !g_file_get_contents(s->firmware_image, &data,
                                                  &length, NULL) ||
        length <= 0x10f || length > 256 * 1024 ||
        (uint8_t)data[0x10e] + (uint8_t)data[0x10f] != 255) {
        error_setg(errp, "FT3A81 requires original firmware with valid version complement");
        return;
    }
    s->firmware = (uint8_t *)g_steal_pointer(&data);
    s->firmware_size = length;
    memset(s->factory_config, 0xff, sizeof(s->factory_config));
    s->factory_config[4] = s->vendor_id;
    s->factory_config[5] = s->vendor_id ^ 0xff;
    fts_reset(dev);
    s->input_handler = qemu_input_handler_register(dev, &fts_input_handler);
    qemu_input_handler_activate(s->input_handler);
}

static const Property fts_properties[] = {
    DEFINE_PROP_STRING("firmware-image", FocaltechFT3A81State, firmware_image),
    DEFINE_PROP_UINT8("vendor-id", FocaltechFT3A81State, vendor_id, 0x5d),
    DEFINE_PROP_UINT16("resolution-x", FocaltechFT3A81State, resolution_x, 1986),
    DEFINE_PROP_UINT16("resolution-y", FocaltechFT3A81State, resolution_y, 2648),
    DEFINE_PROP_BOOL("input-invert-x", FocaltechFT3A81State, invert_x, true),
    DEFINE_PROP_BOOL("input-invert-y", FocaltechFT3A81State, invert_y, true),
};
static void fts_init(Object *obj)
{
    FocaltechFT3A81State *s = FOCALTECH_FT3A81(obj);
    s->scan_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fts_scan, s);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in_named(DEVICE(obj), fts_reset_input, "reset", 1);
    object_property_add_uint64_ptr(obj, "commands", &s->commands, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "flash-reads", &s->flash_reads, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "rejected-commands", &s->rejected_commands, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "touch-reports", &s->touch_reports, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "touch-reads", &s->touch_reads, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "touch-presses", &s->touch_presses, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "touch-releases", &s->touch_releases, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "touch-overruns", &s->touch_overruns, OBJ_PROP_FLAG_READ);
}
static void fts_finalize(Object *obj)
{
    FocaltechFT3A81State *s = FOCALTECH_FT3A81(obj);
    if (s->input_handler) { qemu_input_handler_unregister(s->input_handler); }
    timer_free(s->scan_timer);
    g_free(s->firmware);
}
static void fts_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(klass);
    dc->realize = fts_realize;
    device_class_set_legacy_reset(dc, fts_reset);
    device_class_set_props(dc, fts_properties);
    ic->send = fts_send; ic->recv = fts_recv; ic->event = fts_event;
}
static const TypeInfo fts_type = {
    .name = TYPE_FOCALTECH_FT3A81, .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FocaltechFT3A81State),
    .instance_init = fts_init, .instance_finalize = fts_finalize,
    .class_init = fts_class_init,
};
static void fts_register(void) { type_register_static(&fts_type); }
type_init(fts_register);
