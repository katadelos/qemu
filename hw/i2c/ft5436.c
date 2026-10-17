/*
 * FocalTech classic I2C touchscreens.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define TYPE_FT5436 "ft5436"
OBJECT_DECLARE_TYPE(FT5436State, FT5436Class, FT5436)

#define FT5436_QUEUE_SIZE 16

struct FT5436Class {
    I2CSlaveClass parent_class;
    uint8_t chip_id, chip_id_low;
};

struct FT5436State {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer;
    bool address_byte, report_read, reset_level;
    uint16_t width, height;
    bool invert_x, invert_y, swap_axes;
    uint16_t input_x, input_y, reported_x, reported_y;
    bool pressed, reported_pressed;
    uint16_t queue_x[FT5436_QUEUE_SIZE], queue_y[FT5436_QUEUE_SIZE];
    uint8_t queue_event[FT5436_QUEUE_SIZE], queue_head, queue_count;
    bool report_active;
    qemu_irq irq;
    QEMUTimer *report_timer;
    QemuInputHandlerState *input;
};

static void ft5436_deliver(void *opaque)
{
    FT5436State *s = opaque;
    unsigned head = s->queue_head;
    uint16_t x = s->queue_x[head], y = s->queue_y[head];
    uint8_t event = s->queue_event[head];

    if (s->report_active || !s->queue_count || !s->reset_level) {
        return;
    }
    /* Standard six-byte contacts, starting at register 3, tracking ID 0. */
    memset(s->regs + 1, 0xff, 62);
    s->regs[1] = 0;
    s->regs[2] = event == 1 ? 0 : 1;
    s->regs[3] = (event << 6) | (x >> 8);
    s->regs[4] = x;
    s->regs[5] = y >> 8;
    s->regs[6] = y;
    s->regs[7] = 0x3f;
    s->regs[8] = 0x30;
    s->report_active = true;
    qemu_set_irq(s->irq, 0);
}

static void ft5436_input_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    FT5436State *s = FT5436(dev);

    if (evt->type == INPUT_EVENT_KIND_ABS) {
        InputMoveEvent *move = evt->u.abs.data;
        if (move->axis == INPUT_AXIS_X) {
            s->input_x = move->value;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->input_y = move->value;
        }
    } else if (evt->type == INPUT_EVENT_KIND_BTN &&
               evt->u.btn.data->button == INPUT_BUTTON_LEFT) {
        s->pressed = evt->u.btn.data->down;
    }
}

static void ft5436_input_sync(DeviceState *dev)
{
    FT5436State *s = FT5436(dev);
    uint16_t ix = s->swap_axes ? s->input_y : s->input_x;
    uint16_t iy = s->swap_axes ? s->input_x : s->input_y;
    uint16_t x = qemu_input_scale_axis(ix, 0, INPUT_EVENT_ABS_MAX,
                                      0, s->width - 1);
    uint16_t y = qemu_input_scale_axis(iy, 0, INPUT_EVENT_ABS_MAX,
                                      0, s->height - 1);
    unsigned tail;

    if (s->invert_x) {
        x = s->width - 1 - x;
    }
    if (s->invert_y) {
        y = s->height - 1 - y;
    }
    if (!s->reset_level ||
        (s->pressed == s->reported_pressed &&
         (!s->pressed || (x == s->reported_x && y == s->reported_y)))) {
        return;
    }
    if (s->queue_count == FT5436_QUEUE_SIZE) {
        /* Keep the release even if a burst of pointer moves fills the queue. */
        tail = (s->queue_head + s->queue_count - 1) % FT5436_QUEUE_SIZE;
        if (s->pressed) {
            return;
        }
    } else {
        tail = (s->queue_head + s->queue_count++) % FT5436_QUEUE_SIZE;
    }
    s->queue_x[tail] = x;
    s->queue_y[tail] = y;
    s->queue_event[tail] = s->pressed ? (s->reported_pressed ? 2 : 0) : 1;
    s->reported_x = x;
    s->reported_y = y;
    s->reported_pressed = s->pressed;
    if (!timer_pending(s->report_timer)) {
        ft5436_deliver(s);
    }
}

static const QemuInputHandler ft5436_input_handler = {
    .name = "FocalTech classic I2C touchscreen",
    .mask = INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN,
    .event = ft5436_input_event,
    .sync = ft5436_input_sync,
};

static void ft5436_reset(DeviceState *dev)
{
    FT5436State *s = FT5436(dev);
    FT5436Class *klass = FT5436_GET_CLASS(s);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0xa3] = klass->chip_id;
    s->regs[0x9f] = klass->chip_id_low;
    /* The stock driver treats 0xff as a failed firmware-version read. */
    s->regs[0xa6] = 0xfe;
    s->regs[0xa8] = 0x79;
    s->pointer = 0;
    s->address_byte = s->report_read = s->report_active = false;
    s->pressed = s->reported_pressed = false;
    s->input_x = s->input_y = s->reported_x = s->reported_y = 0;
    s->queue_head = s->queue_count = 0;
    timer_del(s->report_timer);
    qemu_set_irq(s->irq, 1);
}

static void ft5436_reset_pin(void *opaque, int line, int level)
{
    FT5436State *s = opaque;
    if (!level) {
        ft5436_reset(DEVICE(s));
    }
    s->reset_level = level;
}

static int ft5436_send(I2CSlave *i2c, uint8_t data)
{
    FT5436State *s = FT5436(i2c);
    if (s->address_byte) {
        s->pointer = data;
        s->address_byte = false;
    } else {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t ft5436_recv(I2CSlave *i2c)
{
    FT5436State *s = FT5436(i2c);
    return s->regs[s->pointer++];
}

static int ft5436_event(I2CSlave *i2c, enum i2c_event event)
{
    FT5436State *s = FT5436(i2c);
    if (event == I2C_START_SEND) {
        s->address_byte = true;
    } else if (event == I2C_START_RECV) {
        s->report_read = s->report_active && s->pointer <= 3;
    } else if (event == I2C_FINISH && s->report_read) {
        s->report_read = s->report_active = false;
        s->queue_head = (s->queue_head + 1) % FT5436_QUEUE_SIZE;
        s->queue_count--;
        qemu_set_irq(s->irq, 1);
        if (s->queue_count) {
            timer_mod(s->report_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
        }
    }
    return 0;
}

static const VMStateDescription ft5436_vmstate = {
    .name = TYPE_FT5436,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FT5436State),
        VMSTATE_UINT8_ARRAY(regs, FT5436State, 256),
        VMSTATE_UINT8(pointer, FT5436State),
        VMSTATE_BOOL(address_byte, FT5436State),
        VMSTATE_BOOL(report_read, FT5436State),
        VMSTATE_BOOL(reset_level, FT5436State),
        VMSTATE_BOOL(report_active, FT5436State),
        VMSTATE_UINT16(input_x, FT5436State),
        VMSTATE_UINT16(input_y, FT5436State),
        VMSTATE_UINT16(reported_x, FT5436State),
        VMSTATE_UINT16(reported_y, FT5436State),
        VMSTATE_BOOL(pressed, FT5436State),
        VMSTATE_BOOL(reported_pressed, FT5436State),
        VMSTATE_UINT16_ARRAY(queue_x, FT5436State, FT5436_QUEUE_SIZE),
        VMSTATE_UINT16_ARRAY(queue_y, FT5436State, FT5436_QUEUE_SIZE),
        VMSTATE_UINT8_ARRAY(queue_event, FT5436State, FT5436_QUEUE_SIZE),
        VMSTATE_UINT8(queue_head, FT5436State),
        VMSTATE_UINT8(queue_count, FT5436State),
        VMSTATE_TIMER_PTR(report_timer, FT5436State),
        VMSTATE_END_OF_LIST()
    },
};

static const Property ft5436_properties[] = {
    DEFINE_PROP_UINT16("width", FT5436State, width, 1264),
    DEFINE_PROP_UINT16("height", FT5436State, height, 1680),
    DEFINE_PROP_BOOL("invert-x", FT5436State, invert_x, false),
    DEFINE_PROP_BOOL("invert-y", FT5436State, invert_y, false),
    DEFINE_PROP_BOOL("swap-axes", FT5436State, swap_axes, false),
};

static void ft5436_realize(DeviceState *dev, Error **errp)
{
    FT5436State *s = FT5436(dev);
    s->report_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, ft5436_deliver, s);
    s->input = qemu_input_handler_register(dev, &ft5436_input_handler);
}

static void ft5436_unrealize(DeviceState *dev)
{
    FT5436State *s = FT5436(dev);
    qemu_input_handler_unregister(s->input);
    timer_free(s->report_timer);
}

static void ft5436_init(Object *obj)
{
    qdev_init_gpio_out_named(DEVICE(obj), &FT5436(obj)->irq, "irq", 1);
    qdev_init_gpio_in_named(DEVICE(obj), ft5436_reset_pin, "reset", 1);
}

static void ft5436_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    FT5436Class *klass = FT5436_CLASS(oc);
    klass->chip_id = 0x54;
    klass->chip_id_low = 0x22;
    dc->realize = ft5436_realize;
    dc->unrealize = ft5436_unrealize;
    dc->vmsd = &ft5436_vmstate;
    device_class_set_legacy_reset(dc, ft5436_reset);
    device_class_set_props(dc, ft5436_properties);
    sc->send = ft5436_send;
    sc->recv = ft5436_recv;
    sc->event = ft5436_event;
}

static void ft5726_class_init(ObjectClass *oc, const void *data)
{
    FT5436Class *klass = FT5436_CLASS(oc);
    klass->chip_id = 0x58;
    klass->chip_id_low = 0x22;
}

static void ft6336_class_init(ObjectClass *oc, const void *data)
{
    FT5436Class *klass = FT5436_CLASS(oc);
    klass->chip_id = 0x64;
    klass->chip_id_low = 0x26;
}

static const TypeInfo ft5436_types[] = {
    {
        .name = TYPE_FT5436,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(FT5436State),
        .class_size = sizeof(FT5436Class),
        .instance_init = ft5436_init,
        .class_init = ft5436_class_init,
    }, {
        .name = "ft5726",
        .parent = TYPE_FT5436,
        .class_init = ft5726_class_init,
    }, {
        .name = "ft6336",
        .parent = TYPE_FT5436,
        .class_init = ft6336_class_init,
    },
};

DEFINE_TYPES(ft5436_types)
