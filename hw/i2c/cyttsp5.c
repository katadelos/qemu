/* Cypress TrueTouch Gen5 HID/PIP controller used by Kobo Forma and Kindle Oasis.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "qemu/bswap.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"
#include "migration/vmstate.h"

#define TYPE_CYTTSP5 "cyttsp5"
#define CYTTSP5_TOUCH_QUEUE_SIZE 64
OBJECT_DECLARE_SIMPLE_TYPE(CYTTSP5State, CYTTSP5)
typedef struct CYTTSP5Touch {
    uint16_t x, y;
    uint8_t event;
} CYTTSP5Touch;

struct CYTTSP5State {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *ready_timer;
    QemuInputHandlerState *input_handler;
    uint8_t tx[512], rx[512], params[256];
    unsigned tx_len, rx_len, rx_pos;
    int input_x, input_y;
    uint16_t width, height;
    bool invert_x, invert_y, swap_axes;
    bool reading, bootloader, reset_level, pressed, reported;
    bool input_dirty;
    bool irq_level;
    bool rx_pending, rx_is_touch;
    uint8_t sequence;
    CYTTSP5Touch touches[CYTTSP5_TOUCH_QUEUE_SIZE];
    uint8_t touch_head, touch_count;
};
static void cyttsp5_deliver_touch(CYTTSP5State *s);

static const uint8_t cyttsp5_report_descriptor[] = {
    0x05, 0x0d, 0x09, 0x04, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x22, 0xa1, 0x02, 0x06, 0x0d, 0x00, 0x09,
    0x56, 0x15, 0x00, 0x26, 0xff, 0xff, 0x75, 0x10, 0x95, 0x01, 0x81, 0x02, 0x06, 0x0d, 0x00, 0x09,
    0x54, 0x15, 0x00, 0x26, 0x0a, 0x00, 0x75, 0x08, 0x95, 0x01, 0x81, 0x02, 0x06, 0x01, 0xff, 0x09,
    0x42, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x01, 0x81, 0x02, 0xc0, 0x05, 0x0d, 0x09,
    0x22, 0xa1, 0x02, 0x06, 0x01, 0xff, 0x09, 0x60, 0x15, 0x00, 0x26, 0x07, 0x00, 0x75, 0x08, 0x95,
    0x01, 0x81, 0x02, 0x06, 0x0d, 0x00, 0x09, 0x51, 0x15, 0x00, 0x26, 0x09, 0x00, 0x75, 0x08, 0x95,
    0x01, 0x81, 0x02, 0x06, 0x01, 0xff, 0x09, 0x61, 0x15, 0x00, 0x26, 0x03, 0x00, 0x75, 0x08, 0x95,
    0x01, 0x81, 0x02, 0x06, 0x0d, 0x00, 0x09, 0x42, 0x15, 0x00, 0x26, 0x01, 0x00, 0x75, 0x08, 0x95,
    0x01, 0x81, 0x02, 0x06, 0x01, 0x00, 0x09, 0x30, 0x15, 0x00, 0x26, 0x9f, 0x05, 0x75, 0x10, 0x95,
    0x01, 0x81, 0x02, 0x06, 0x01, 0x00, 0x09, 0x31, 0x15, 0x00, 0x26, 0x7f, 0x07, 0x75, 0x10, 0x95,
    0x01, 0x81, 0x02, 0x06, 0x0d, 0x00, 0x09, 0x30, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95,
    0x01, 0x81, 0x02, 0x06, 0x01, 0xff, 0x09, 0x62, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95,
    0x01, 0x81, 0x02, 0x06, 0x01, 0xff, 0x09, 0x63, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95,
    0x01, 0x81, 0x02, 0x06, 0x01, 0xff, 0x09, 0x64, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95,
    0x01, 0x81, 0x02, 0xc0, 0xc0
};

static void cyttsp5_ready(void *opaque)
{
    CYTTSP5State *s = opaque;
    s->irq_level = false;
    qemu_set_irq(s->irq, 0);
}

static void cyttsp5_queue(CYTTSP5State *s, unsigned len)
{
    stw_le_p(s->rx, len);
    s->rx_len = MAX(len, 2);
    s->rx_pos = 0;
    s->rx_pending = true;
    s->irq_level = true;
    qemu_set_irq(s->irq, 1);
    timer_mod(s->ready_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
}

static uint16_t cyttsp5_crc(const uint8_t *data, unsigned size)
{
    uint16_t crc = 0xffff;
    for (unsigned i = 0; i < size; i++) {
        crc ^= data[i] << 8;
        for (unsigned j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        }
    }
    return crc;
}

static void cyttsp5_command(CYTTSP5State *s)
{
    uint16_t reg;
    uint8_t cmd;
    if (s->tx_len < 2) {
        return;
    }
    reg = lduw_le_p(s->tx);
    if (reg != 1 && reg != 2 && reg != 4 && reg != 5) {
        return;
    }
    /* An interrupted input report stays at the head of the touch queue. */
    s->rx_is_touch = false;
    memset(s->rx, 0, sizeof(s->rx));
    if (reg == 1 && s->tx_len == 2) {
        s->rx[2] = s->bootloader ? 0xff : 0xf7;
        stw_le_p(s->rx + 4, 0x100);
        stw_le_p(s->rx + 6, sizeof(cyttsp5_report_descriptor) + 3);
        stw_le_p(s->rx + 8, 2); /* report descriptor */
        stw_le_p(s->rx + 10, 3); /* input */
        stw_le_p(s->rx + 12, 512);
        stw_le_p(s->rx + 14, 4); /* output */
        stw_le_p(s->rx + 16, 512);
        stw_le_p(s->rx + 18, 5); /* command */
        stw_le_p(s->rx + 20, 6); /* data */
        stw_le_p(s->rx + 22, 0x04b4);
        stw_le_p(s->rx + 24, s->bootloader ? 0xc100 : 0xc101);
        stw_le_p(s->rx + 26, 0x100);
        cyttsp5_queue(s, 32);
    } else if (reg == 2 && s->tx_len == 2) {
        s->rx[2] = 0xf6;
        memcpy(s->rx + 3, cyttsp5_report_descriptor, sizeof(cyttsp5_report_descriptor));
        /* Logical maxima in the X and Y HID items. */
        stw_le_p(s->rx + 3 + 139, s->width - 1);
        stw_le_p(s->rx + 3 + 155, s->height - 1);
        cyttsp5_queue(s, sizeof(cyttsp5_report_descriptor) + 3);
    } else if (reg == 5 && s->tx_len >= 4) {
        cmd = s->tx[3] & 0xf;
        if (cmd == 1) {
            s->bootloader = true;
            cyttsp5_queue(s, 0);
        } else {
            s->rx[2] = 0xf0;
            s->rx[3] = s->tx[2] & 3;
            s->rx[4] = cmd;
            cyttsp5_queue(s, 5);
        }
    } else if (reg == 4 && s->tx_len >= 7) {
        if (s->tx[4] == 0x40 && s->tx_len >= 8) {
            cmd = s->tx[7];
            if (cmd == 0x3b) {
                s->bootloader = false;
                cyttsp5_queue(s, 0);
            } else {
                s->rx[2] = 0x30;
                s->rx[4] = 1;
                stw_le_p(s->rx + 6, 1);
                s->rx[8] = cmd == 0x3e ? 0xff : 1;
                stw_le_p(s->rx + 9, cyttsp5_crc(s->rx + 4, 5));
                s->rx[11] = 0x17;
                cyttsp5_queue(s, 12);
            }
            return;
        }
        cmd = s->tx[6] & 0x7f;
        s->rx[2] = 0x1f;
        s->rx[4] = cmd;
        switch (cmd) {
        case 1: /* start bootloader */
            s->bootloader = true;
            cyttsp5_queue(s, 0);
            break;
        case 2: /* PIP 1.6 system and sensing configuration. */
            s->rx[5] = 1; s->rx[6] = 6;
            s->rx[9] = 0xff;
            stl_le_p(s->rx + 11, 0xffffffff);
            stw_le_p(s->rx + 15, 0xffff);
            s->rx[17] = 1;
            s->rx[33] = 24; s->rx[34] = 32;
            /* Physical dimensions in hundredths of a millimetre. */
            stw_le_p(s->rx + 35, 12200);
            stw_le_p(s->rx + 37, 16300);
            stw_le_p(s->rx + 39, s->width);
            stw_le_p(s->rx + 41, s->height);
            stw_le_p(s->rx + 43, 255);
            s->rx[47] = 0xff;
            s->rx[50] = 10;
            cyttsp5_queue(s, 51);
            break;
        case 5: /* get RAM parameter */
        case 6: /* set RAM parameter */
            if (s->tx_len < 8) {
                break;
            }
            if (s->tx_len > 9 && cmd == 6) {
                s->params[s->tx[7]] = s->tx[9];
            }
            s->rx[5] = s->tx[7];
            s->rx[6] = 1;
            s->rx[7] = s->params[s->tx[7]];
            cyttsp5_queue(s, 8);
            break;
        case 0x20: /* verify config CRC */
            cyttsp5_queue(s, 10);
            break;
        case 0x21: /* configuration row size */
            stw_le_p(s->rx + 5, 128);
            cyttsp5_queue(s, 7);
            break;
        case 0x22: /* read configuration block, including version at byte 8 */
            if (s->tx_len >= 12) {
                unsigned offset = lduw_le_p(s->tx + 7) * 128;
                unsigned len = MIN(lduw_le_p(s->tx + 9),
                                   sizeof(s->rx) - 12);

                s->rx[6] = s->tx[11];
                stw_le_p(s->rx + 7, len);
                for (unsigned i = 0; i < len; i++) {
                    if (offset + i == 8 || offset + i == 9) {
                        s->rx[10 + i] = 0xff;
                    }
                }
                stw_le_p(s->rx + 10 + len, cyttsp5_crc(s->rx + 10, len));
                cyttsp5_queue(s, 12 + len);
            } else {
                s->rx[5] = 1;
                cyttsp5_queue(s, 6);
            }
            break;
        default: /* null, scanning and calibration completion */
            cyttsp5_queue(s, 6);
            break;
        }
    }
}

static int cyttsp5_send(I2CSlave *i2c, uint8_t value)
{
    CYTTSP5State *s = CYTTSP5(i2c);
    if (s->tx_len < sizeof(s->tx)) {
        s->tx[s->tx_len++] = value;
    }
    return 0;
}
static uint8_t cyttsp5_recv(I2CSlave *i2c)
{
    CYTTSP5State *s = CYTTSP5(i2c);
    return s->rx_pos < sizeof(s->rx) ? s->rx[s->rx_pos++] : 0;
}
static int cyttsp5_event(I2CSlave *i2c, enum i2c_event event)
{
    CYTTSP5State *s = CYTTSP5(i2c);
    switch (event) {
    case I2C_START_SEND:
        s->reading = false;
        s->tx_len = 0;
        break;
    case I2C_START_RECV:
        s->reading = true;
        s->rx_pos = 0;
        break;
    case I2C_FINISH:
        if (!s->reading) {
            cyttsp5_command(s);
        } else if (s->rx_pending && s->rx_pos >= s->rx_len) {
            timer_del(s->ready_timer);
            if (s->rx_is_touch) {
                s->reported = s->touches[s->touch_head].event != 3;
                s->touch_head = (s->touch_head + 1) % CYTTSP5_TOUCH_QUEUE_SIZE;
                s->touch_count--;
            }
            s->rx_pending = s->rx_is_touch = false;
            memset(s->rx, 0, sizeof(s->rx));
            stw_le_p(s->rx, 2);
            s->rx_len = 2;
            s->irq_level = true;
            qemu_set_irq(s->irq, 1);
            cyttsp5_deliver_touch(s);
        }
        break;
    default:
        break;
    }
    return 0;
}
static void cyttsp5_gpio_reset(void *opaque, int line, int level)
{
    CYTTSP5State *s = opaque;
    if (level && !s->reset_level) {
        s->bootloader = true;
        s->touch_head = s->touch_count = 0;
        s->pressed = s->reported = s->input_dirty = false;
        s->rx_is_touch = false;
        memset(s->rx, 0, sizeof(s->rx));
        cyttsp5_queue(s, 0);
    }
    s->reset_level = level;
}
static void cyttsp5_input_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    CYTTSP5State *s = CYTTSP5(dev);
    if (evt->type == INPUT_EVENT_KIND_ABS) {
        InputMoveEvent *move = evt->u.abs.data;
        if (move->axis == INPUT_AXIS_X) {
            s->input_dirty |= s->input_x != move->value;
            s->input_x = move->value;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->input_dirty |= s->input_y != move->value;
            s->input_y = move->value;
        }
    } else if (evt->type == INPUT_EVENT_KIND_BTN &&
               evt->u.btn.data->button == INPUT_BUTTON_LEFT) {
        s->input_dirty |= s->pressed != evt->u.btn.data->down;
        s->pressed = evt->u.btn.data->down;
    }
}
static void cyttsp5_input_sync(DeviceState *dev)
{
    CYTTSP5State *s = CYTTSP5(dev);
    CYTTSP5Touch touch;
    bool previous_pressed = s->reported;
    unsigned tail = 0;

    if (s->bootloader || !s->input_dirty) {
        return;
    }
    s->input_dirty = false;
    if (s->touch_count) {
        tail = (s->touch_head + s->touch_count - 1) % CYTTSP5_TOUCH_QUEUE_SIZE;
        previous_pressed = s->touches[tail].event != 3;
    }
    if (!s->pressed && !previous_pressed) {
        return;
    }
    touch.x = qemu_input_scale_axis(s->swap_axes ? s->input_y : s->input_x,
                                   INPUT_EVENT_ABS_MIN,
                                   INPUT_EVENT_ABS_MAX, 0, s->width - 1);
    touch.y = qemu_input_scale_axis(s->swap_axes ? s->input_x : s->input_y,
                                   INPUT_EVENT_ABS_MIN,
                                   INPUT_EVENT_ABS_MAX, 0, s->height - 1);
    touch.event = s->pressed ? (previous_pressed ? 2 : 1) : 3;

    /* Coalesce motion only; a down/up pair must survive unread replies. */
    if (touch.event == 2 && s->touch_count > 1 &&
        s->touches[tail].event == 2) {
        s->touches[tail] = touch;
    } else {
        if (s->touch_count == CYTTSP5_TOUCH_QUEUE_SIZE) {
            /* Keep the active report and reconcile the latest host state. */
            s->touch_count = 1;
            previous_pressed = s->touches[s->touch_head].event != 3;
            if (!s->pressed && !previous_pressed) {
                return;
            }
            touch.event = s->pressed ? (previous_pressed ? 2 : 1) : 3;
        }
        tail = (s->touch_head + s->touch_count) % CYTTSP5_TOUCH_QUEUE_SIZE;
        s->touches[tail] = touch;
        s->touch_count++;
    }
    cyttsp5_deliver_touch(s);
}

static void cyttsp5_deliver_touch(CYTTSP5State *s)
{
    CYTTSP5Touch *touch;
    bool pressed;

    if (s->bootloader || s->rx_pending || !s->touch_count) {
        return;
    }
    touch = &s->touches[s->touch_head];
    pressed = touch->event != 3;
    memset(s->rx, 0, sizeof(s->rx));
    s->rx[2] = 1;
    stw_le_p(s->rx + 3, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
    s->rx[5] = pressed;
    s->rx[6] = s->sequence++;
    s->rx[9] = touch->event;
    s->rx[10] = pressed;
    /* Compensate for the sensor mounting described by the board's DT. */
    stw_le_p(s->rx + 11, s->invert_x ? s->width - touch->x : touch->x);
    stw_le_p(s->rx + 13, s->invert_y ? s->height - touch->y : touch->y);
    s->rx[15] = pressed ? 64 : 0;
    s->rx[16] = s->rx[17] = 2;
    /* An empty contact frame releases the Linux protocol-A touch slot. */
    cyttsp5_queue(s, pressed ? 19 : 7);
    s->rx_is_touch = true;
}
static const QemuInputHandler cyttsp5_input = {
    .name = "Cypress TrueTouch Gen5 touchscreen",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = cyttsp5_input_event,
    .sync = cyttsp5_input_sync,
};
static void cyttsp5_reset(DeviceState *dev)
{
    CYTTSP5State *s = CYTTSP5(dev);
    timer_del(s->ready_timer);
    s->tx_len = s->rx_pos = 0;
    s->rx_len = 2;
    memset(s->rx, 0, sizeof(s->rx));
    memset(s->params, 0, sizeof(s->params));
    s->rx[0] = 2;
    s->bootloader = s->reset_level = true;
    s->pressed = s->reported = false;
    s->rx_pending = s->rx_is_touch = false;
    s->touch_head = s->touch_count = 0;
    s->input_dirty = false;
    s->irq_level = true;
    s->reading = false;
    s->sequence = 0;
    s->input_x = s->input_y = 0;
    qemu_set_irq(s->irq, 1);
}
static int cyttsp5_post_load(void *opaque, int version_id)
{
    CYTTSP5State *s = opaque;

    if (s->tx_len > sizeof(s->tx) || s->rx_len > sizeof(s->rx) ||
        s->rx_pos > sizeof(s->rx) ||
        s->touch_head >= CYTTSP5_TOUCH_QUEUE_SIZE ||
        s->touch_count > CYTTSP5_TOUCH_QUEUE_SIZE ||
        (s->rx_is_touch && (!s->rx_pending || !s->touch_count))) {
        return -EINVAL;
    }
    qemu_set_irq(s->irq, s->irq_level);
    return 0;
}
static const VMStateDescription cyttsp5_touch_vmstate = {
    .name = "cyttsp5/touch",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(x, CYTTSP5Touch),
        VMSTATE_UINT16(y, CYTTSP5Touch),
        VMSTATE_UINT8(event, CYTTSP5Touch),
        VMSTATE_END_OF_LIST()
    },
};
static const VMStateDescription cyttsp5_vmstate = {
    .name = TYPE_CYTTSP5,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = cyttsp5_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, CYTTSP5State),
        VMSTATE_UINT8_ARRAY(tx, CYTTSP5State, 512),
        VMSTATE_UINT8_ARRAY(rx, CYTTSP5State, 512),
        VMSTATE_UINT8_ARRAY(params, CYTTSP5State, 256),
        VMSTATE_UINT32(tx_len, CYTTSP5State),
        VMSTATE_UINT32(rx_len, CYTTSP5State),
        VMSTATE_UINT32(rx_pos, CYTTSP5State),
        VMSTATE_INT32(input_x, CYTTSP5State),
        VMSTATE_INT32(input_y, CYTTSP5State),
        VMSTATE_BOOL(reading, CYTTSP5State),
        VMSTATE_BOOL(bootloader, CYTTSP5State),
        VMSTATE_BOOL(reset_level, CYTTSP5State),
        VMSTATE_BOOL(pressed, CYTTSP5State),
        VMSTATE_BOOL(reported, CYTTSP5State),
        VMSTATE_BOOL(input_dirty, CYTTSP5State),
        VMSTATE_BOOL(irq_level, CYTTSP5State),
        VMSTATE_BOOL(rx_pending, CYTTSP5State),
        VMSTATE_BOOL(rx_is_touch, CYTTSP5State),
        VMSTATE_UINT8(touch_head, CYTTSP5State),
        VMSTATE_UINT8(touch_count, CYTTSP5State),
        VMSTATE_STRUCT_ARRAY(touches, CYTTSP5State, CYTTSP5_TOUCH_QUEUE_SIZE,
                             0, cyttsp5_touch_vmstate, CYTTSP5Touch),
        VMSTATE_UINT8(sequence, CYTTSP5State),
        VMSTATE_TIMER_PTR(ready_timer, CYTTSP5State),
        VMSTATE_END_OF_LIST()
    },
};
static void cyttsp5_realize(DeviceState *dev, Error **errp)
{
    CYTTSP5State *s = CYTTSP5(dev);
    s->input_handler = qemu_input_handler_register(dev, &cyttsp5_input);
}
static void cyttsp5_init(Object *obj)
{
    CYTTSP5State *s = CYTTSP5(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in(DEVICE(obj), cyttsp5_gpio_reset, 1);
    s->ready_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cyttsp5_ready, s);
}
static void cyttsp5_finalize(Object *obj)
{
    CYTTSP5State *s = CYTTSP5(obj);
    if (s->input_handler) {
        qemu_input_handler_unregister(s->input_handler);
    }
    timer_free(s->ready_timer);
}
static const Property cyttsp5_properties[] = {
    DEFINE_PROP_UINT16("width", CYTTSP5State, width, 1440),
    DEFINE_PROP_UINT16("height", CYTTSP5State, height, 1920),
    DEFINE_PROP_BOOL("invert-x", CYTTSP5State, invert_x, false),
    DEFINE_PROP_BOOL("invert-y", CYTTSP5State, invert_y, true),
    DEFINE_PROP_BOOL("swap-axes", CYTTSP5State, swap_axes, false),
};
static void cyttsp5_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    device_class_set_props(dc, cyttsp5_properties);
    dc->realize = cyttsp5_realize;
    dc->vmsd = &cyttsp5_vmstate;
    device_class_set_legacy_reset(dc, cyttsp5_reset);
    sc->send = cyttsp5_send;
    sc->recv = cyttsp5_recv;
    sc->event = cyttsp5_event;
}
static const TypeInfo cyttsp5_type = {
    .name = TYPE_CYTTSP5,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(CYTTSP5State),
    .instance_init = cyttsp5_init,
    .instance_finalize = cyttsp5_finalize,
    .class_init = cyttsp5_class_init,
};
static void cyttsp5_register_types(void)
{
    type_register_static(&cyttsp5_type);
}
type_init(cyttsp5_register_types)
