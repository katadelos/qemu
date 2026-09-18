/* Elan EKTH3500 HID-over-I2C touchscreen, as used by Kobo Elipsa 2E.
 * Protocol: Kobo Linux 4.9 drivers/input/touchscreen/elants_i2c.c.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define TYPE_ELAN_EKTH3500 "elan-ekth3500"
OBJECT_DECLARE_SIMPLE_TYPE(ElanState, ELAN_EKTH3500)

struct ElanState {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *ready;
    QemuInputHandlerState *input;
    uint8_t tx[37], rx[67];
    unsigned tx_len, rx_pos;
    int x, y;
    uint8_t pen_state;
    bool reading, pending, down, reported, dirty;
};

static void elan_input_sync(DeviceState *dev);

static void elan_ready(void *opaque)
{
    ElanState *s = opaque;
    qemu_set_irq(s->irq, s->pending ? 0 : 1);
}

static void elan_queue(ElanState *s)
{
    s->rx_pos = 0;
    s->pending = true;
    qemu_set_irq(s->irq, 1);
    timer_mod(s->ready, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
}

static void elan_command(ElanState *s)
{
    if (s->tx_len < 11) {
        s->tx_len = 0;
        return;
    }
    uint8_t cmd = s->tx[7], sub = s->tx[8];
    s->tx_len = 0;
    s->rx_pos = 0;
    memset(s->rx, 0, sizeof(s->rx));
    s->rx[0] = 0x43;
    if (cmd == 0x53) {
        s->rx[4] = 0x52;
        s->rx[5] = sub | 1;
        s->rx[6] = 0;
        s->rx[7] = 0x10;
        switch (sub) {
        case 0xc1: /* runtime pen-status reads complete through the IRQ */
            s->rx[6] = s->pen_state;
            elan_queue(s);
            break;
        case 0xd6: /* oversampling ratio */
            s->rx[7] = 32;
            break;
        case 0xd7: /* physical sensor dimensions, millimetres */
            stw_be_p(s->rx + 6, 210);
            break;
        case 0xd8:
            stw_be_p(s->rx + 6, 158);
            break;
        }
    } else if (cmd == 0x5b) {
        s->rx[4] = 0x9b;
        s->rx[6] = 60; /* (60 - 1) * 32 = 1888 sensor units */
        s->rx[7] = 45; /* (45 - 1) * 32 = 1408 sensor units */
    } else if (cmd == 0x96) {
        s->rx[4] = 0x95;
        s->rx[5] = s->rx[6] = 0x80;
        s->rx[7] = 0x10;
        s->rx[9] = 0x21;
    } else if (cmd == 0x54 && sub == 0xc1) {
        s->pen_state = s->tx[9];
    } else if (cmd == 0x54 && sub == 0x29) {
        s->rx[2] = 2;
        s->rx[3] = 4;
        memset(s->rx + 4, 0x66, 4);
        elan_queue(s);
    }
}

static int elan_send(I2CSlave *i2c, uint8_t value)
{
    ElanState *s = ELAN_EKTH3500(i2c);
    if (s->tx_len < sizeof(s->tx)) {
        s->tx[s->tx_len++] = value;
    }
    return 0;
}

static uint8_t elan_recv(I2CSlave *i2c)
{
    ElanState *s = ELAN_EKTH3500(i2c);
    return s->rx_pos < sizeof(s->rx) ? s->rx[s->rx_pos++] : 0;
}

static int elan_event(I2CSlave *i2c, enum i2c_event event)
{
    ElanState *s = ELAN_EKTH3500(i2c);
    if (event == I2C_START_SEND) {
        s->tx_len = 0;
        s->reading = false;
    } else if (event == I2C_START_RECV) {
        if (s->tx_len) {
            elan_command(s);
        }
        s->rx_pos = 0;
        s->reading = true;
    } else if (event == I2C_FINISH) {
        if (!s->reading && s->tx_len) {
            elan_command(s);
        } else if (s->reading && s->rx_pos == sizeof(s->rx)) {
            s->pending = false;
            qemu_set_irq(s->irq, 1);
            elan_input_sync(DEVICE(s));
        }
    }
    return 0;
}

static void elan_input_event(DeviceState *dev, QemuConsole *con, InputEvent *evt)
{
    ElanState *s = ELAN_EKTH3500(dev);
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

static void elan_input_sync(DeviceState *dev)
{
    ElanState *s = ELAN_EKTH3500(dev);
    if (!s->dirty || s->pending) {
        return;
    }
    s->dirty = false;
    if (!s->down && !s->reported) {
        return;
    }
    memset(s->rx, 0, sizeof(s->rx));
    s->rx[0] = 0x3f;
    s->rx[62] = 1; /* contact count in the 0x3f HID report */
    s->rx[3] = 4 | (s->down ? 3 : 0); /* contact ID 1, down/up */
    s->rx[4] = s->down ? 8 : 0;
    /* Undo Nickel's landscape-sensor to portrait-screen transform. */
    stw_le_p(s->rx + 8, qemu_input_scale_axis(INPUT_EVENT_ABS_MAX - s->y,
        INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX, 0, 1888));
    stw_le_p(s->rx + 12, qemu_input_scale_axis(s->x,
        INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX, 0, 1408));
    s->reported = s->down;
    elan_queue(s);
}

static const QemuInputHandler elan_input = {
    .name = "Kobo Elipsa 2E touchscreen",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = elan_input_event,
    .sync = elan_input_sync,
};

static void elan_reset(DeviceState *dev)
{
    ElanState *s = ELAN_EKTH3500(dev);
    timer_del(s->ready);
    s->tx_len = s->rx_pos = 0;
    s->pen_state = 3;
    s->pending = s->down = s->reported = s->dirty = false;
    memset(s->rx, 0, sizeof(s->rx));
    qemu_set_irq(s->irq, 1);
}

static void elan_reset_gpio(void *opaque, int line, int level)
{
    if (!level) {
        elan_reset(DEVICE(opaque));
    }
}

static void elan_init(Object *obj)
{
    ElanState *s = ELAN_EKTH3500(obj);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
    qdev_init_gpio_in_named(DEVICE(obj), elan_reset_gpio, "reset", 1);
    s->ready = timer_new_ns(QEMU_CLOCK_VIRTUAL, elan_ready, s);
}

static void elan_realize(DeviceState *dev, Error **errp)
{
    ELAN_EKTH3500(dev)->input = qemu_input_handler_register(dev, &elan_input);
}

static void elan_finalize(Object *obj)
{
    ElanState *s = ELAN_EKTH3500(obj);
    if (s->input) {
        qemu_input_handler_unregister(s->input);
    }
    timer_free(s->ready);
}

static void elan_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);
    dc->realize = elan_realize;
    device_class_set_legacy_reset(dc, elan_reset);
    ic->send = elan_send;
    ic->recv = elan_recv;
    ic->event = elan_event;
}

static const TypeInfo elan_type = {
    .name = TYPE_ELAN_EKTH3500,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ElanState),
    .instance_init = elan_init,
    .instance_finalize = elan_finalize,
    .class_init = elan_class_init,
};

static void elan_register_types(void)
{
    type_register_static(&elan_type);
}
type_init(elan_register_types)
