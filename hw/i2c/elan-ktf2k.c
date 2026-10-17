/*
 * ELAN KTF2K touchscreens used by PocketBook readers.
 * Protocols: kernel_622/elan_touch_i2c.c and kernel-b288/ektf2k_5B.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/elan-ktf2k.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define ELAN_READY_NS 100000

OBJECT_DECLARE_SIMPLE_TYPE(ElanKTF2KState, ELAN_KTF2K)

struct ElanKTF2KState {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *ready;
    QemuInputHandlerState *input;
    uint8_t tx[4];
    uint8_t rx[8];
    uint8_t tx_len;
    uint8_t rx_len;
    uint8_t rx_pos;
    uint8_t power_mode;
    uint8_t sensitivity;
    uint16_t x_resolution, y_resolution;
    uint16_t firmware_version, firmware_id;
    uint8_t contact_mask;
    int32_t x, y;
    bool swap_axes, invert_x, invert_y;
    bool powered;
    bool reset_high;
    bool reading;
    bool pending;
    bool down;
    bool reported;
    bool dirty;
};

static void elan_ktf2k_input_sync(DeviceState *dev);

static bool elan_ktf2k_enabled(ElanKTF2KState *s)
{
    return s->powered && s->reset_high;
}

static void elan_ktf2k_ready(void *opaque)
{
    ElanKTF2KState *s = opaque;

    qemu_set_irq(s->irq, !(elan_ktf2k_enabled(s) && s->pending));
}

static void elan_ktf2k_queue(ElanKTF2KState *s, unsigned length)
{
    s->rx_len = length;
    s->rx_pos = 0;
    s->pending = true;
    qemu_set_irq(s->irq, 1);
    timer_mod(s->ready, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ELAN_READY_NS);
}

static void elan_ktf2k_clear(ElanKTF2KState *s)
{
    timer_del(s->ready);
    s->tx_len = s->rx_len = s->rx_pos = 0;
    s->reading = s->pending = s->down = s->reported = s->dirty = false;
    s->power_mode = 8;
    s->sensitivity = 0x7a;
    memset(s->rx, 0, sizeof(s->rx));
    qemu_set_irq(s->irq, 1);
}

static void elan_ktf2k_hello(ElanKTF2KState *s)
{
    elan_ktf2k_clear(s);
    memset(s->rx, 0x55, 4);
    elan_ktf2k_queue(s, 4);
}

static void elan_ktf2k_version(ElanKTF2KState *s, uint16_t version)
{
    s->rx[1] = (s->rx[1] & 0xf0) | (version >> 12);
    s->rx[2] = version >> 4;
    s->rx[3] = (version & 0xf) << 4;
}

static void elan_ktf2k_command(ElanKTF2KState *s)
{
    unsigned command = s->tx[0];
    unsigned reg = s->tx[1];

    s->tx_len = 0;
    if (command == 0x54) {
        switch (reg) {
        case 0x50:
        case 0x54:
        case 0x58:
            s->power_mode = reg & 0xc;
            break;
        case 0x40:
            s->sensitivity = s->tx[2];
            break;
        }
        return;
    }
    if (command != 0x53) {
        return;
    }

    memset(s->rx, 0, sizeof(s->rx));
    s->rx[0] = 0x52;
    s->rx[1] = reg;
    switch (reg) {
    case 0x00: /* Firmware version, encoded as four nibbles. */
        elan_ktf2k_version(s, s->firmware_version);
        break;
    case 0x01: /* Bootcode version 1.0. */
        elan_ktf2k_version(s, 0x0100);
        break;
    case 0x50:
        s->rx[1] |= s->power_mode;
        break;
    case 0x40:
        s->rx[2] = s->sensitivity;
        s->rx[3] = 0x11;
        break;
    case 0x60:
        s->rx[2] = s->x_resolution & 0xff;
        s->rx[3] = (s->x_resolution >> 4) & 0xf0;
        break;
    case 0x63:
        s->rx[2] = s->y_resolution & 0xff;
        s->rx[3] = (s->y_resolution >> 4) & 0xf0;
        break;
    case 0xf0:
        elan_ktf2k_version(s, s->firmware_id);
        break;
    }
    elan_ktf2k_queue(s, 4);
}

static int elan_ktf2k_send(I2CSlave *i2c, uint8_t value)
{
    ElanKTF2KState *s = ELAN_KTF2K(i2c);

    if (s->tx_len < sizeof(s->tx)) {
        s->tx[s->tx_len++] = value;
    }
    return 0;
}

static uint8_t elan_ktf2k_recv(I2CSlave *i2c)
{
    ElanKTF2KState *s = ELAN_KTF2K(i2c);

    return s->rx_pos < s->rx_len ? s->rx[s->rx_pos++] : 0;
}

static int elan_ktf2k_event(I2CSlave *i2c, enum i2c_event event)
{
    ElanKTF2KState *s = ELAN_KTF2K(i2c);

    if ((event == I2C_START_SEND || event == I2C_START_RECV) &&
        !elan_ktf2k_enabled(s)) {
        return -1;
    }
    switch (event) {
    case I2C_START_SEND:
        s->tx_len = 0;
        s->reading = false;
        break;
    case I2C_START_RECV:
        if (s->tx_len == sizeof(s->tx)) {
            elan_ktf2k_command(s);
        }
        s->reading = true;
        s->rx_pos = 0;
        break;
    case I2C_FINISH:
        if (!s->reading && s->tx_len == sizeof(s->tx)) {
            elan_ktf2k_command(s);
        } else if (s->reading && s->rx_pos >= s->rx_len) {
            timer_del(s->ready);
            s->pending = false;
            qemu_set_irq(s->irq, 1);
            elan_ktf2k_input_sync(DEVICE(s));
        }
        break;
    default:
        break;
    }
    return 0;
}

static void elan_ktf2k_input_event(DeviceState *dev, QemuConsole *con,
                                    InputEvent *evt)
{
    ElanKTF2KState *s = ELAN_KTF2K(dev);

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

static void elan_ktf2k_input_sync(DeviceState *dev)
{
    ElanKTF2KState *s = ELAN_KTF2K(dev);
    unsigned x, y;

    if (!s->dirty || s->pending || !elan_ktf2k_enabled(s) ||
        !s->power_mode) {
        return;
    }
    s->dirty = false;
    if (!s->down && !s->reported) {
        return;
    }

    /* Board mounting and the guest's coordinate transform are independent
     * of the controller's packet format and native coordinate range. */
    x = s->swap_axes ? s->y : s->x;
    y = s->swap_axes ? s->x : s->y;
    if (s->invert_x) {
        x = INPUT_EVENT_ABS_MAX - x;
    }
    if (s->invert_y) {
        y = INPUT_EVENT_ABS_MAX - y;
    }
    x = qemu_input_scale_axis(x,
                             INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
                             0, s->x_resolution);
    y = qemu_input_scale_axis(y, INPUT_EVENT_ABS_MIN,
                             INPUT_EVENT_ABS_MAX, 0, s->y_resolution);
    memset(s->rx, 0, sizeof(s->rx));
    s->rx[0] = 0x5a;
    s->rx[1] = ((x >> 4) & 0xf0) | ((y >> 8) & 0x0f);
    s->rx[2] = x;
    s->rx[3] = y;
    s->rx[7] = s->down ? s->contact_mask : 0;
    s->reported = s->down;
    elan_ktf2k_queue(s, sizeof(s->rx));
}

static const QemuInputHandler elan_ktf2k_input = {
    .name = "ELAN KTF2K touchscreen",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = elan_ktf2k_input_event,
    .sync = elan_ktf2k_input_sync,
};

static void elan_ktf2k_reset_gpio(void *opaque, int n, int level)
{
    ElanKTF2KState *s = opaque;

    if (s->reset_high == !!level) {
        return;
    }
    s->reset_high = !!level;
    if (elan_ktf2k_enabled(s)) {
        elan_ktf2k_hello(s);
    } else {
        elan_ktf2k_clear(s);
    }
}

static void elan_ktf2k_power_gpio(void *opaque, int n, int level)
{
    ElanKTF2KState *s = opaque;

    if (s->powered == !!level) {
        return;
    }
    s->powered = !!level;
    if (elan_ktf2k_enabled(s)) {
        elan_ktf2k_hello(s);
    } else {
        elan_ktf2k_clear(s);
    }
}

static void elan_ktf2k_reset(DeviceState *dev)
{
    ElanKTF2KState *s = ELAN_KTF2K(dev);

    if (elan_ktf2k_enabled(s)) {
        elan_ktf2k_hello(s);
    } else {
        elan_ktf2k_clear(s);
    }
}

static int elan_ktf2k_post_load(void *opaque, int version_id)
{
    ElanKTF2KState *s = opaque;

    if (s->tx_len > sizeof(s->tx) || s->rx_len > sizeof(s->rx) ||
        s->rx_pos > s->rx_len) {
        return -EINVAL;
    }
    if (!timer_pending(s->ready)) {
        elan_ktf2k_ready(s);
    }
    return 0;
}

static const VMStateDescription elan_ktf2k_vmstate = {
    .name = TYPE_ELAN_KTF2K,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = elan_ktf2k_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, ElanKTF2KState),
        VMSTATE_BUFFER(tx, ElanKTF2KState),
        VMSTATE_BUFFER(rx, ElanKTF2KState),
        VMSTATE_UINT8(tx_len, ElanKTF2KState),
        VMSTATE_UINT8(rx_len, ElanKTF2KState),
        VMSTATE_UINT8(rx_pos, ElanKTF2KState),
        VMSTATE_UINT8(power_mode, ElanKTF2KState),
        VMSTATE_UINT8(sensitivity, ElanKTF2KState),
        VMSTATE_INT32(x, ElanKTF2KState),
        VMSTATE_INT32(y, ElanKTF2KState),
        VMSTATE_BOOL(powered, ElanKTF2KState),
        VMSTATE_BOOL(reset_high, ElanKTF2KState),
        VMSTATE_BOOL(reading, ElanKTF2KState),
        VMSTATE_BOOL(pending, ElanKTF2KState),
        VMSTATE_BOOL(down, ElanKTF2KState),
        VMSTATE_BOOL(reported, ElanKTF2KState),
        VMSTATE_BOOL(dirty, ElanKTF2KState),
        VMSTATE_TIMER_PTR(ready, ElanKTF2KState),
        VMSTATE_END_OF_LIST()
    },
};

static void elan_ktf2k_realize(DeviceState *dev, Error **errp)
{
    ElanKTF2KState *s = ELAN_KTF2K(dev);

    if (!s->x_resolution || s->x_resolution > 0xfff ||
        !s->y_resolution || s->y_resolution > 0xfff ||
        (s->contact_mask != 1 && s->contact_mask != 2)) {
        error_setg(errp, "ELAN requires 12-bit coordinate ranges and contact mask 1 or 2");
        return;
    }
    s->input = qemu_input_handler_register(dev, &elan_ktf2k_input);
}

static void elan_ktf2k_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(ELAN_KTF2K(dev)->input);
}

static void elan_ktf2k_init(Object *obj)
{
    ElanKTF2KState *s = ELAN_KTF2K(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
    qdev_init_gpio_in_named(DEVICE(obj), elan_ktf2k_reset_gpio, "reset", 1);
    qdev_init_gpio_in_named(DEVICE(obj), elan_ktf2k_power_gpio, "power", 1);
    s->ready = timer_new_ns(QEMU_CLOCK_VIRTUAL, elan_ktf2k_ready, s);
}

static void elan_ktf2k_finalize(Object *obj)
{
    timer_free(ELAN_KTF2K(obj)->ready);
}

static void elan_ktf2k_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);
    static const Property properties[] = {
        DEFINE_PROP_UINT16("x-resolution", ElanKTF2KState, x_resolution, 1088),
        DEFINE_PROP_UINT16("y-resolution", ElanKTF2KState, y_resolution, 768),
        DEFINE_PROP_UINT16("firmware-version", ElanKTF2KState,
                           firmware_version, 0x0100),
        DEFINE_PROP_UINT16("firmware-id", ElanKTF2KState, firmware_id, 0x0001),
        DEFINE_PROP_UINT8("contact-mask", ElanKTF2KState, contact_mask, 2),
        DEFINE_PROP_BOOL("swap-axes", ElanKTF2KState, swap_axes, true),
        DEFINE_PROP_BOOL("invert-x", ElanKTF2KState, invert_x, true),
        DEFINE_PROP_BOOL("invert-y", ElanKTF2KState, invert_y, false),
    };

    device_class_set_props(dc, properties);
    dc->realize = elan_ktf2k_realize;
    dc->unrealize = elan_ktf2k_unrealize;
    dc->vmsd = &elan_ktf2k_vmstate;
    device_class_set_legacy_reset(dc, elan_ktf2k_reset);
    ic->send = elan_ktf2k_send;
    ic->recv = elan_ktf2k_recv;
    ic->event = elan_ktf2k_event;
}

static const TypeInfo elan_ktf2k_type = {
    .name = TYPE_ELAN_KTF2K,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ElanKTF2KState),
    .instance_init = elan_ktf2k_init,
    .instance_finalize = elan_ktf2k_finalize,
    .class_init = elan_ktf2k_class_init,
};

static void elan_ktf2k_register_types(void)
{
    type_register_static(&elan_ktf2k_type);
}

type_init(elan_ktf2k_register_types)
