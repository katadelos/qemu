/*
 * Goodix GT813 touchscreen, as wired on the PocketBook Color Lux 801.
 * Protocol: pocketbook-free/801, kernel/drivers/hwmon/gt813.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/goodix_gt813.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define GT813_COORD       0x0f40
#define GT813_VERSION     0x0f7d
#define GT813_CONFIG      0x0f80
#define GT813_SLEEP       0x0ff2
#define GT813_END         0x8000
#define GT813_READY_NS    100000

OBJECT_DECLARE_SIMPLE_TYPE(GoodixGT813State, GOODIX_GT813)

struct GoodixGT813State {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *ready;
    QemuInputHandlerState *input;
    uint8_t regs[0x1000];
    uint16_t pointer;
    uint8_t address_bytes;
    int32_t x, y;
    bool powered;
    bool reset_high;
    bool sleeping;
    bool pending;
    bool report_read;
    bool down;
    bool reported;
    bool dirty;
};

static void goodix_gt813_input_sync(DeviceState *dev);

static bool goodix_gt813_enabled(GoodixGT813State *s)
{
    return s->powered && s->reset_high && !s->sleeping;
}

static void goodix_gt813_ready(void *opaque)
{
    GoodixGT813State *s = opaque;

    /* The vendor config selects a rising-edge interrupt. */
    qemu_set_irq(s->irq, goodix_gt813_enabled(s) && s->pending);
}

static void goodix_gt813_clear_report(GoodixGT813State *s)
{
    timer_del(s->ready);
    s->pending = s->report_read = false;
    s->regs[GT813_COORD] = 0;
    qemu_set_irq(s->irq, 0);
}

static void goodix_gt813_reset_state(GoodixGT813State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[GT813_VERSION] = 0x81;
    s->regs[GT813_VERSION + 1] = 0x01;
    s->regs[GT813_VERSION + 2] = 0x00;
    /* The 112-byte configuration contains big-endian sensor dimensions. */
    stw_be_p(s->regs + GT813_CONFIG + 69, 600);
    stw_be_p(s->regs + GT813_CONFIG + 71, 800);
    s->regs[GT813_CONFIG + 64] = 0x08;
    s->pointer = s->address_bytes = 0;
    s->sleeping = s->down = s->reported = s->dirty = false;
    goodix_gt813_clear_report(s);
}

static int goodix_gt813_send(I2CSlave *i2c, uint8_t data)
{
    GoodixGT813State *s = GOODIX_GT813(i2c);

    if (s->address_bytes < 2) {
        s->pointer = (s->pointer << 8) | data;
        s->address_bytes++;
    } else if (s->pointer < sizeof(s->regs)) {
        s->regs[s->pointer] = data;
        if (s->pointer == GT813_SLEEP && data == 0xc0) {
            s->sleeping = true;
            goodix_gt813_clear_report(s);
        }
        s->pointer++;
    }
    return 0;
}

static uint8_t goodix_gt813_recv(I2CSlave *i2c)
{
    GoodixGT813State *s = GOODIX_GT813(i2c);

    if (s->pointer == GT813_COORD) {
        s->report_read = true;
    }
    return s->pointer < sizeof(s->regs) ? s->regs[s->pointer++] : 0;
}

static int goodix_gt813_event(I2CSlave *i2c, enum i2c_event event)
{
    GoodixGT813State *s = GOODIX_GT813(i2c);

    if ((event == I2C_START_SEND || event == I2C_START_RECV) &&
        (!s->powered || !s->reset_high)) {
        return -1;
    }
    if (event == I2C_START_SEND) {
        s->pointer = s->address_bytes = 0;
    } else if (event == I2C_FINISH && s->address_bytes == 2 &&
               s->pointer == GT813_END && s->report_read) {
        /* 80 00 releases the read report and rearms the next rising edge. */
        goodix_gt813_clear_report(s);
        goodix_gt813_input_sync(DEVICE(s));
    }
    return 0;
}

static void goodix_gt813_input_event(DeviceState *dev, QemuConsole *con,
                                      InputEvent *evt)
{
    GoodixGT813State *s = GOODIX_GT813(dev);

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

static void goodix_gt813_input_sync(DeviceState *dev)
{
    GoodixGT813State *s = GOODIX_GT813(dev);
    uint8_t *report = s->regs + GT813_COORD;
    unsigned width = lduw_be_p(s->regs + GT813_CONFIG + 69);
    unsigned height = lduw_be_p(s->regs + GT813_CONFIG + 71);
    unsigned x, y, i;

    if (!s->dirty || s->pending || !goodix_gt813_enabled(s) ||
        !width || !height) {
        return;
    }
    s->dirty = false;
    if (!s->down && !s->reported) {
        return;
    }
    x = qemu_input_scale_axis(s->x, INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
                              0, width);
    /* The stock RGB driver reverses the controller's Y coordinate. */
    y = qemu_input_scale_axis(INPUT_EVENT_ABS_MAX - s->y,
                              INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
                              0, height);
    memset(report, 0, 8);
    report[0] = s->down ? 0x81 : 0x80;
    if (s->down) {
        stw_be_p(report + 2, x);
        stw_be_p(report + 4, y);
        report[6] = 32;
        for (i = 2; i < 7; i++) {
            report[7] += report[i];
        }
    }
    s->reported = s->down;
    s->pending = true;
    qemu_set_irq(s->irq, 0);
    timer_mod(s->ready, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GT813_READY_NS);
}

static const QemuInputHandler goodix_gt813_input = {
    .name = "Goodix GT813 touchscreen",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = goodix_gt813_input_event,
    .sync = goodix_gt813_input_sync,
};

static void goodix_gt813_power(void *opaque, int n, int level)
{
    GoodixGT813State *s = opaque;

    if (s->powered != !!level) {
        s->powered = !!level;
        goodix_gt813_reset_state(s);
    }
}

static void goodix_gt813_reset_pin(void *opaque, int n, int level)
{
    GoodixGT813State *s = opaque;

    if (s->reset_high != !!level) {
        s->reset_high = !!level;
        goodix_gt813_reset_state(s);
    }
}

static void goodix_gt813_wake(void *opaque, int n, int level)
{
    GoodixGT813State *s = opaque;

    /* Resume drives the interrupt pin low twice before returning to input. */
    if (!level && s->sleeping) {
        s->sleeping = false;
        goodix_gt813_input_sync(DEVICE(s));
    }
}

static void goodix_gt813_reset(DeviceState *dev)
{
    goodix_gt813_reset_state(GOODIX_GT813(dev));
}

static int goodix_gt813_post_load(void *opaque, int version_id)
{
    GoodixGT813State *s = opaque;

    if (s->address_bytes > 2) {
        return -EINVAL;
    }
    if (!timer_pending(s->ready)) {
        goodix_gt813_ready(s);
    }
    return 0;
}

static const VMStateDescription goodix_gt813_vmstate = {
    .name = TYPE_GOODIX_GT813,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = goodix_gt813_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, GoodixGT813State),
        VMSTATE_BUFFER(regs, GoodixGT813State),
        VMSTATE_UINT16(pointer, GoodixGT813State),
        VMSTATE_UINT8(address_bytes, GoodixGT813State),
        VMSTATE_INT32(x, GoodixGT813State),
        VMSTATE_INT32(y, GoodixGT813State),
        VMSTATE_BOOL(powered, GoodixGT813State),
        VMSTATE_BOOL(reset_high, GoodixGT813State),
        VMSTATE_BOOL(sleeping, GoodixGT813State),
        VMSTATE_BOOL(pending, GoodixGT813State),
        VMSTATE_BOOL(report_read, GoodixGT813State),
        VMSTATE_BOOL(down, GoodixGT813State),
        VMSTATE_BOOL(reported, GoodixGT813State),
        VMSTATE_BOOL(dirty, GoodixGT813State),
        VMSTATE_TIMER_PTR(ready, GoodixGT813State),
        VMSTATE_END_OF_LIST()
    },
};

static void goodix_gt813_realize(DeviceState *dev, Error **errp)
{
    GOODIX_GT813(dev)->input =
        qemu_input_handler_register(dev, &goodix_gt813_input);
}

static void goodix_gt813_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(GOODIX_GT813(dev)->input);
}

static void goodix_gt813_init(Object *obj)
{
    GoodixGT813State *s = GOODIX_GT813(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
    qdev_init_gpio_in_named(DEVICE(obj), goodix_gt813_power, "power", 1);
    qdev_init_gpio_in_named(DEVICE(obj), goodix_gt813_reset_pin, "reset", 1);
    qdev_init_gpio_in_named(DEVICE(obj), goodix_gt813_wake, "wake", 1);
    s->ready = timer_new_ns(QEMU_CLOCK_VIRTUAL, goodix_gt813_ready, s);
}

static void goodix_gt813_finalize(Object *obj)
{
    timer_free(GOODIX_GT813(obj)->ready);
}

static void goodix_gt813_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);

    dc->realize = goodix_gt813_realize;
    dc->unrealize = goodix_gt813_unrealize;
    dc->vmsd = &goodix_gt813_vmstate;
    device_class_set_legacy_reset(dc, goodix_gt813_reset);
    ic->send = goodix_gt813_send;
    ic->recv = goodix_gt813_recv;
    ic->event = goodix_gt813_event;
}

static const TypeInfo goodix_gt813_type = {
    .name = TYPE_GOODIX_GT813,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(GoodixGT813State),
    .instance_init = goodix_gt813_init,
    .instance_finalize = goodix_gt813_finalize,
    .class_init = goodix_gt813_class_init,
};

static void goodix_gt813_register_types(void)
{
    type_register_static(&goodix_gt813_type);
}

type_init(goodix_gt813_register_types)
