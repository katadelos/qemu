/*
 * MStar MSG2133 / CNT touchscreen, as wired in the PocketBook Ultra 650.
 * Protocol: Platform_MX6/Kernel/drivers/input/touchscreen/cntouch_ts.c.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/msg2133.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define MSG2133_RESOLUTION 2036
#define MSG2133_READY_NS 100000

OBJECT_DECLARE_SIMPLE_TYPE(MSG2133State, MSG2133)

struct MSG2133State {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *ready;
    QemuInputHandlerState *input;
    uint8_t tx[3];
    uint8_t rx[13];
    uint8_t tx_len, rx_len, rx_pos;
    int32_t x, y;
    bool powered, reset_high;
    bool pending, report, reading;
    bool down, reported, dirty;
};

static void msg2133_input_sync(DeviceState *dev);

static bool msg2133_enabled(MSG2133State *s)
{
    return s->powered && s->reset_high;
}

static void msg2133_ready(void *opaque)
{
    MSG2133State *s = opaque;

    qemu_set_irq(s->irq, msg2133_enabled(s) && s->pending && s->report);
}

static void msg2133_clear(MSG2133State *s)
{
    timer_del(s->ready);
    s->tx_len = s->rx_len = s->rx_pos = 0;
    s->pending = s->report = s->reading = false;
    s->down = s->reported = s->dirty = false;
    memset(s->rx, 0, sizeof(s->rx));
    qemu_set_irq(s->irq, 0);
}

static void msg2133_command(MSG2133State *s)
{
    s->tx_len = 0;
    if (s->tx[0] == 0x53 && s->tx[1] == 0 && s->tx[2] == 0x74) {
        timer_del(s->ready);
        memset(s->rx, 0, sizeof(s->rx));
        s->rx[0] = 1; /* Firmware 1.0; version words are little endian. */
        s->rx_len = sizeof(s->rx);
        s->rx_pos = 0;
        s->pending = true;
        s->report = false;
        qemu_set_irq(s->irq, 0);
    }
}

static int msg2133_send(I2CSlave *i2c, uint8_t value)
{
    MSG2133State *s = MSG2133(i2c);

    if (s->tx_len < sizeof(s->tx)) {
        s->tx[s->tx_len++] = value;
    }
    return 0;
}

static uint8_t msg2133_recv(I2CSlave *i2c)
{
    MSG2133State *s = MSG2133(i2c);

    return s->rx_pos < s->rx_len ? s->rx[s->rx_pos++] : 0;
}

static int msg2133_event(I2CSlave *i2c, enum i2c_event event)
{
    MSG2133State *s = MSG2133(i2c);

    if ((event == I2C_START_SEND || event == I2C_START_RECV) &&
        !msg2133_enabled(s)) {
        return -1;
    }
    switch (event) {
    case I2C_START_SEND:
        s->tx_len = 0;
        s->reading = false;
        break;
    case I2C_START_RECV:
        if (s->tx_len == sizeof(s->tx)) {
            msg2133_command(s);
        }
        s->reading = true;
        s->rx_pos = 0;
        break;
    case I2C_FINISH:
        if (!s->reading && s->tx_len == sizeof(s->tx)) {
            msg2133_command(s);
        } else if (s->reading && s->rx_pos >= s->rx_len) {
            timer_del(s->ready);
            s->pending = s->report = false;
            qemu_set_irq(s->irq, 0);
            msg2133_input_sync(DEVICE(s));
        }
        break;
    default:
        break;
    }
    return 0;
}

static void msg2133_input_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    MSG2133State *s = MSG2133(dev);

    if (evt->type == INPUT_EVENT_KIND_ABS) {
        InputMoveEvent *move = evt->u.abs.data;

        if (move->axis == INPUT_AXIS_X) {
            s->x = move->value;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->y = move->value;
        }
        s->dirty = true;
    } else if (evt->type == INPUT_EVENT_KIND_BTN &&
               evt->u.btn.data->button == INPUT_BUTTON_LEFT) {
        s->down = evt->u.btn.data->down;
        s->dirty = true;
    }
}

static void msg2133_input_sync(DeviceState *dev)
{
    MSG2133State *s = MSG2133(dev);
    unsigned x, y, checksum = 0;

    if (!s->dirty || s->pending || !msg2133_enabled(s)) {
        return;
    }
    s->dirty = false;
    if (!s->down && !s->reported) {
        return;
    }
    memset(s->rx, 0, sizeof(s->rx));
    s->rx[0] = 0x52;
    if (s->down) {
        x = qemu_input_scale_axis(s->x, INPUT_EVENT_ABS_MIN,
                                 INPUT_EVENT_ABS_MAX, 0, MSG2133_RESOLUTION);
        y = qemu_input_scale_axis(s->y, INPUT_EVENT_ABS_MIN,
                                 INPUT_EVENT_ABS_MAX, 0, MSG2133_RESOLUTION);
        s->rx[1] = ((x >> 4) & 0xf0) | ((y >> 8) & 0xf);
        s->rx[2] = x;
        s->rx[3] = y;
    } else {
        s->rx[1] = s->rx[2] = s->rx[3] = s->rx[4] = s->rx[6] = 0xff;
    }
    for (unsigned i = 0; i < 7; i++) {
        checksum += s->rx[i];
    }
    s->rx[7] = -checksum;
    s->rx_len = 8;
    s->rx_pos = 0;
    s->pending = s->report = true;
    s->reported = s->down;
    qemu_set_irq(s->irq, 0);
    timer_mod(s->ready, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              MSG2133_READY_NS);
}

static void msg2133_power(void *opaque, int line, int level)
{
    MSG2133State *s = opaque;

    if (s->powered != !!level) {
        s->powered = !!level;
        msg2133_clear(s);
    }
}

static void msg2133_reset_pin(void *opaque, int line, int level)
{
    MSG2133State *s = opaque;

    if (s->reset_high != !!level) {
        s->reset_high = !!level;
        msg2133_clear(s);
    }
}

static void msg2133_reset(DeviceState *dev)
{
    msg2133_clear(MSG2133(dev));
}

static int msg2133_post_load(void *opaque, int version_id)
{
    MSG2133State *s = opaque;

    if (s->tx_len > sizeof(s->tx) || s->rx_len > sizeof(s->rx) ||
        s->rx_pos > s->rx_len) {
        return -EINVAL;
    }
    if (!timer_pending(s->ready)) {
        msg2133_ready(s);
    }
    return 0;
}

static const VMStateDescription msg2133_vmstate = {
    .name = TYPE_MSG2133,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = msg2133_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MSG2133State),
        VMSTATE_BUFFER(tx, MSG2133State),
        VMSTATE_BUFFER(rx, MSG2133State),
        VMSTATE_UINT8(tx_len, MSG2133State),
        VMSTATE_UINT8(rx_len, MSG2133State),
        VMSTATE_UINT8(rx_pos, MSG2133State),
        VMSTATE_INT32(x, MSG2133State),
        VMSTATE_INT32(y, MSG2133State),
        VMSTATE_BOOL(powered, MSG2133State),
        VMSTATE_BOOL(reset_high, MSG2133State),
        VMSTATE_BOOL(pending, MSG2133State),
        VMSTATE_BOOL(report, MSG2133State),
        VMSTATE_BOOL(reading, MSG2133State),
        VMSTATE_BOOL(down, MSG2133State),
        VMSTATE_BOOL(reported, MSG2133State),
        VMSTATE_BOOL(dirty, MSG2133State),
        VMSTATE_TIMER_PTR(ready, MSG2133State),
        VMSTATE_END_OF_LIST()
    },
};

static const QemuInputHandler msg2133_input_handler = {
    .name = "MStar MSG2133 touchscreen",
    .mask = INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN,
    .event = msg2133_input_event,
    .sync = msg2133_input_sync,
};

static void msg2133_realize(DeviceState *dev, Error **errp)
{
    MSG2133(dev)->input = qemu_input_handler_register(dev,
                                                    &msg2133_input_handler);
}

static void msg2133_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(MSG2133(dev)->input);
}

static void msg2133_init(Object *obj)
{
    MSG2133State *s = MSG2133(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
    qdev_init_gpio_in_named(DEVICE(obj), msg2133_power, "power", 1);
    qdev_init_gpio_in_named(DEVICE(obj), msg2133_reset_pin, "reset", 1);
    s->ready = timer_new_ns(QEMU_CLOCK_VIRTUAL, msg2133_ready, s);
}

static void msg2133_finalize(Object *obj)
{
    timer_free(MSG2133(obj)->ready);
}

static void msg2133_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);

    dc->realize = msg2133_realize;
    dc->unrealize = msg2133_unrealize;
    dc->vmsd = &msg2133_vmstate;
    device_class_set_legacy_reset(dc, msg2133_reset);
    ic->send = msg2133_send;
    ic->recv = msg2133_recv;
    ic->event = msg2133_event;
}

static const TypeInfo msg2133_type = {
    .name = TYPE_MSG2133,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MSG2133State),
    .instance_init = msg2133_init,
    .instance_finalize = msg2133_finalize,
    .class_init = msg2133_class_init,
};

static void msg2133_register_types(void)
{
    type_register_static(&msg2133_type);
}

type_init(msg2133_register_types)
