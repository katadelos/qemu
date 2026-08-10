/* Minimal Goodix GT9xx touchscreen used by Lab126 Rex. */

#include "qemu/osdep.h"
#include "hw/i2c/goodix_gt9xx.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define GT9XX_REG_CONFIG       0x8047
#define GT9XX_REG_FW_STATUS    0x41e4
#define GT9XX_REG_HW_INFO      0x4220
#define GT9XX_REG_VERSION      0x8140
#define GT9XX_REG_SENSOR_ID    0x814a
#define GT9XX_REG_COORD        0x814e
#define GT9XX_MAX_X            1072
#define GT9XX_MAX_Y            1448

struct GoodixGT9xxState {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *irq_timer;
    QemuInputHandlerState *input_handler;
    uint8_t regs[UINT16_MAX + 1];
    uint16_t pointer;
    uint8_t pointer_bytes;
    uint16_t input_x;
    uint16_t input_y;
    bool input_pressed;
    bool report_pressed;
    bool reset_level;
};

static void goodix_irq_release(void *opaque)
{
    GoodixGT9xxState *s = opaque;

    qemu_set_irq(s->irq, 1);
}

static void goodix_pulse_irq(GoodixGT9xxState *s)
{
    qemu_set_irq(s->irq, 1);
    qemu_set_irq(s->irq, 0);
    timer_mod(s->irq_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
}

static void goodix_input_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    GoodixGT9xxState *s = GOODIX_GT9XX(dev);

    if (evt->type == INPUT_EVENT_KIND_ABS) {
        InputMoveEvent *move = evt->u.abs.data;

        if (move->axis == INPUT_AXIS_X) {
            s->input_x = move->value;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->input_y = move->value;
        }
    } else if (evt->type == INPUT_EVENT_KIND_BTN) {
        InputBtnEvent *btn = evt->u.btn.data;

        if (btn->button == INPUT_BUTTON_LEFT) {
            s->input_pressed = btn->down;
        }
    }
}

static void goodix_input_sync(DeviceState *dev)
{
    GoodixGT9xxState *s = GOODIX_GT9XX(dev);
    uint16_t x, y;

    if (s->input_pressed == s->report_pressed && !s->input_pressed) {
        return;
    }

    x = qemu_input_scale_axis(s->input_x, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, GT9XX_MAX_X);
    y = qemu_input_scale_axis(s->input_y, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, GT9XX_MAX_Y);
    memset(&s->regs[GT9XX_REG_COORD], 0, 10);
    s->regs[GT9XX_REG_COORD] = 0x80 | (s->input_pressed ? 1 : 0);
    if (s->input_pressed) {
        s->regs[GT9XX_REG_COORD + 1] = 0; /* tracking ID */
        s->regs[GT9XX_REG_COORD + 2] = x;
        s->regs[GT9XX_REG_COORD + 3] = x >> 8;
        s->regs[GT9XX_REG_COORD + 4] = y;
        s->regs[GT9XX_REG_COORD + 5] = y >> 8;
        s->regs[GT9XX_REG_COORD + 6] = 32;
    }
    s->report_pressed = s->input_pressed;
    goodix_pulse_irq(s);
}

static const QemuInputHandler goodix_input_handler = {
    .name = "Goodix GT9xx touchscreen",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = goodix_input_event,
    .sync = goodix_input_sync,
};

static int goodix_send(I2CSlave *i2c, uint8_t data)
{
    GoodixGT9xxState *s = GOODIX_GT9XX(i2c);

    if (s->pointer_bytes == 0) {
        s->pointer = (uint16_t)data << 8;
        s->pointer_bytes = 1;
    } else if (s->pointer_bytes == 1) {
        s->pointer |= data;
        s->pointer_bytes = 2;
    } else {
        uint16_t address = s->pointer++;

        /* The watchdog feed register clears after accepting 0xaa. */
        s->regs[address] =
            (address == 0x8040 && data == 0xaa) ? 0 : data;
    }
    return 0;
}

static uint8_t goodix_recv(I2CSlave *i2c)
{
    GoodixGT9xxState *s = GOODIX_GT9XX(i2c);

    return s->regs[s->pointer++];
}

static int goodix_event(I2CSlave *i2c, enum i2c_event event)
{
    GoodixGT9xxState *s = GOODIX_GT9XX(i2c);

    if (event == I2C_START_SEND) {
        s->pointer_bytes = 0;
    }
    return 0;
}

static void goodix_reset_input(void *opaque, int line, int level)
{
    GoodixGT9xxState *s = opaque;

    s->reset_level = level;
    if (!level) {
        s->pointer = 0;
        s->pointer_bytes = 0;
        s->report_pressed = false;
        qemu_set_irq(s->irq, 1);
    }
}

static void goodix_reset(DeviceState *dev)
{
    GoodixGT9xxState *s = GOODIX_GT9XX(dev);

    memset(s->regs, 0, sizeof(s->regs));
    /* Match the Rex firmware bundled with the stock rootfs.  The Linux
     * driver starts its in-kernel updater when these values differ, which
     * intentionally takes the controller out of normal reporting mode. */
    s->regs[GT9XX_REG_FW_STATUS] = 0xbe; /* valid firmware checksum */
    s->regs[GT9XX_REG_HW_INFO + 0] = 0x00;
    s->regs[GT9XX_REG_HW_INFO + 1] = 0x60;
    s->regs[GT9XX_REG_HW_INFO + 2] = 0x01;
    s->regs[GT9XX_REG_HW_INFO + 3] = 0x00;
    s->regs[GT9XX_REG_CONFIG] = 0x63;
    s->regs[GT9XX_REG_CONFIG + 1] = GT9XX_MAX_X & 0xff;
    s->regs[GT9XX_REG_CONFIG + 2] = GT9XX_MAX_X >> 8;
    s->regs[GT9XX_REG_CONFIG + 3] = GT9XX_MAX_Y & 0xff;
    s->regs[GT9XX_REG_CONFIG + 4] = GT9XX_MAX_Y >> 8;
    s->regs[GT9XX_REG_CONFIG + 6] = 1; /* falling-edge interrupt */
    memcpy(&s->regs[GT9XX_REG_VERSION], "967\0", 4);
    s->regs[GT9XX_REG_VERSION + 4] = 0x77;
    s->regs[GT9XX_REG_VERSION + 5] = 0x10;
    s->regs[GT9XX_REG_SENSOR_ID] = 0;
    s->pointer = 0;
    s->pointer_bytes = 0;
    s->input_pressed = false;
    s->report_pressed = false;
    qemu_set_irq(s->irq, 1);
}

static void goodix_realize(DeviceState *dev, Error **errp)
{
    GoodixGT9xxState *s = GOODIX_GT9XX(dev);

    s->irq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, goodix_irq_release, s);
    s->input_handler = qemu_input_handler_register(dev, &goodix_input_handler);
}

static void goodix_unrealize(DeviceState *dev)
{
    GoodixGT9xxState *s = GOODIX_GT9XX(dev);

    qemu_input_handler_unregister(s->input_handler);
    timer_free(s->irq_timer);
}

static const VMStateDescription goodix_vmstate = {
    .name = TYPE_GOODIX_GT9XX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(pointer, GoodixGT9xxState),
        VMSTATE_UINT8(pointer_bytes, GoodixGT9xxState),
        VMSTATE_UINT16(input_x, GoodixGT9xxState),
        VMSTATE_UINT16(input_y, GoodixGT9xxState),
        VMSTATE_BOOL(input_pressed, GoodixGT9xxState),
        VMSTATE_BOOL(report_pressed, GoodixGT9xxState),
        VMSTATE_BOOL(reset_level, GoodixGT9xxState),
        VMSTATE_UINT8_ARRAY(regs, GoodixGT9xxState, UINT16_MAX + 1),
        VMSTATE_END_OF_LIST()
    },
};

static void goodix_init(Object *obj)
{
    GoodixGT9xxState *s = GOODIX_GT9XX(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in(DEVICE(obj), goodix_reset_input, 1);
}

static void goodix_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = goodix_realize;
    dc->unrealize = goodix_unrealize;
    device_class_set_legacy_reset(dc, goodix_reset);
    dc->vmsd = &goodix_vmstate;
    sc->send = goodix_send;
    sc->recv = goodix_recv;
    sc->event = goodix_event;
}

static const TypeInfo goodix_info = {
    .name = TYPE_GOODIX_GT9XX,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(GoodixGT9xxState),
    .instance_init = goodix_init,
    .class_init = goodix_class_init,
};

static void goodix_register_types(void)
{
    type_register_static(&goodix_info);
}

type_init(goodix_register_types)
