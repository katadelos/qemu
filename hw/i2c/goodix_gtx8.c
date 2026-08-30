/* Goodix GT6861/GTX8 capacitive touchscreen controller model. */

#include "qemu/osdep.h"
#include "hw/i2c/goodix_gtx8.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define GTX8_REG_DOZE_CTRL       0x30f0
#define GTX8_REG_DOZE_STAT       0x3100
#define GTX8_REG_ESD_TICK        0x3103
#define GTX8_REG_COORD           0x4100
#define GTX8_REG_VERSION_BASE    0x452c
#define GTX8_REG_PID             0x4535
#define GTX8_REG_VID             0x453d
#define GTX8_REG_SENSOR_ID       0x4541
#define GTX8_REG_COMMAND         0x6f68
#define GTX8_REG_CONFIG          0x6f78

#define GTX8_VERSION_LENGTH      0x48
#define GTX8_TOUCH_DATA_SIZE     12
#define GTX8_TOUCH_QUEUE_SIZE    16
#define GTX8_MAX_X               1072
#define GTX8_MAX_Y               1448

#define GTX8_DOZE_DISABLE        0xaa
#define GTX8_DOZE_DISABLED       0xbb
#define GTX8_CMD_START_CONFIG    0x80
#define GTX8_CMD_CONFIG_READY    0x82
#define GTX8_CMD_END_CONFIG      0x83
#define GTX8_CMD_START_READ_CFG  0x86
#define GTX8_CMD_READ_CFG_READY  0x85
#define GTX8_CMD_READY           0xff

struct GoodixGTX8State {
    I2CSlave parent_obj;
    uint8_t regs[UINT16_MAX + 1];
    uint16_t pointer;
    uint8_t pointer_bytes;
    bool reset_level;
    qemu_irq irq;
    QEMUTimer *irq_timer;
    QemuInputHandlerState *input_handler;
    uint16_t input_x;
    uint16_t input_y;
    uint16_t report_x;
    uint16_t report_y;
    bool input_pressed;
    bool report_pressed;
    uint16_t touch_x[GTX8_TOUCH_QUEUE_SIZE];
    uint16_t touch_y[GTX8_TOUCH_QUEUE_SIZE];
    bool touch_pressed[GTX8_TOUCH_QUEUE_SIZE];
    uint8_t touch_head;
    uint8_t touch_count;
    bool touch_report_active;
};

static void gtx8_deliver_touch_report(void *opaque)
{
    GoodixGTX8State *s = opaque;
    uint8_t *report = &s->regs[GTX8_REG_COORD];
    unsigned index;
    uint8_t checksum = 0;

    if (!s->touch_count || s->touch_report_active) {
        return;
    }

    index = s->touch_head;
    memset(report, 0, GTX8_TOUCH_DATA_SIZE);
    report[0] = 0x80;
    if (s->touch_pressed[index]) {
        uint16_t x = s->touch_x[index];
        uint16_t y = s->touch_y[index];

        report[1] = 1;
        report[2] = 0; /* tracking ID */
        report[3] = x;
        report[4] = x >> 8;
        report[5] = y;
        report[6] = y >> 8;
        report[7] = 32; /* touch major */
        for (unsigned i = 0; i < GTX8_TOUCH_DATA_SIZE - 1; i++) {
            checksum += report[i];
        }
        report[GTX8_TOUCH_DATA_SIZE - 1] = 0 - checksum;
    } else {
        /* A zero-contact report is checksummed over its four-byte header. */
        report[3] = 0x80;
    }

    /* The controller holds its active-low interrupt until status is cleared. */
    qemu_set_irq(s->irq, 1);
    qemu_set_irq(s->irq, 0);
    s->touch_report_active = true;
}

static void gtx8_queue_touch_report(GoodixGTX8State *s, uint16_t x,
                                    uint16_t y, bool pressed)
{
    unsigned tail;

    if (pressed && s->touch_count) {
        tail = (s->touch_head + s->touch_count - 1) %
               GTX8_TOUCH_QUEUE_SIZE;
        if (s->touch_pressed[tail] &&
            !(s->touch_report_active && tail == s->touch_head)) {
            s->touch_x[tail] = x;
            s->touch_y[tail] = y;
            return;
        }
    }

    if (s->touch_count == GTX8_TOUCH_QUEUE_SIZE) {
        /* Preserve release by replacing the last queued pointer move. */
        tail = (s->touch_head + s->touch_count - 1) %
               GTX8_TOUCH_QUEUE_SIZE;
        if (!pressed && s->touch_pressed[tail]) {
            s->touch_x[tail] = x;
            s->touch_y[tail] = y;
            s->touch_pressed[tail] = false;
        }
        return;
    }

    tail = (s->touch_head + s->touch_count) % GTX8_TOUCH_QUEUE_SIZE;
    s->touch_x[tail] = x;
    s->touch_y[tail] = y;
    s->touch_pressed[tail] = pressed;
    s->touch_count++;
    if (!timer_pending(s->irq_timer)) {
        gtx8_deliver_touch_report(s);
    }
}

static void gtx8_acknowledge_touch_report(GoodixGTX8State *s)
{
    if (!s->touch_report_active || !s->touch_count) {
        return;
    }

    qemu_set_irq(s->irq, 1);
    s->touch_report_active = false;
    s->touch_head = (s->touch_head + 1) % GTX8_TOUCH_QUEUE_SIZE;
    s->touch_count--;
    if (s->touch_count) {
        /* Present the next edge after the guest's threaded handler returns. */
        timer_mod(s->irq_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
    }
}

static void gtx8_input_event(DeviceState *dev, QemuConsole *src,
                             InputEvent *evt)
{
    GoodixGTX8State *s = GOODIX_GTX8(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS: {
        InputMoveEvent *move = evt->u.abs.data;

        if (move->axis == INPUT_AXIS_X) {
            s->input_x = move->value;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->input_y = move->value;
        }
        break;
    }
    case INPUT_EVENT_KIND_BTN: {
        InputBtnEvent *btn = evt->u.btn.data;

        if (btn->button == INPUT_BUTTON_LEFT) {
            s->input_pressed = btn->down;
        }
        break;
    }
    default:
        break;
    }
}

static void gtx8_input_sync(DeviceState *dev)
{
    GoodixGTX8State *s = GOODIX_GTX8(dev);
    uint16_t x, y;

    x = qemu_input_scale_axis(s->input_x, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, GTX8_MAX_X);
    y = qemu_input_scale_axis(s->input_y, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, GTX8_MAX_Y);
    if (s->input_pressed == s->report_pressed &&
        (!s->input_pressed || (x == s->report_x && y == s->report_y))) {
        return;
    }

    gtx8_queue_touch_report(s, x, y, s->input_pressed);
    s->report_x = x;
    s->report_y = y;
    s->report_pressed = s->input_pressed;
}

static const QemuInputHandler gtx8_input_handler = {
    .name = "Goodix GT6861 touchscreen",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = gtx8_input_event,
    .sync = gtx8_input_sync,
};

static void gtx8_write_register(GoodixGTX8State *s, uint16_t address,
                                uint8_t data)
{
    s->regs[address] = data;

    if (address == GTX8_REG_COORD && data == 0) {
        gtx8_acknowledge_touch_report(s);
    } else if (address == GTX8_REG_DOZE_CTRL &&
               data == GTX8_DOZE_DISABLE) {
        s->regs[GTX8_REG_DOZE_STAT] = GTX8_DOZE_DISABLED;
    } else if (address == GTX8_REG_COMMAND) {
        switch (data) {
        case GTX8_CMD_START_CONFIG:
            s->regs[address] = GTX8_CMD_CONFIG_READY;
            break;
        case GTX8_CMD_END_CONFIG:
        case GTX8_CMD_READY:
            s->regs[address] = GTX8_CMD_READY;
            break;
        case GTX8_CMD_START_READ_CFG:
            s->regs[address] = GTX8_CMD_READ_CFG_READY;
            break;
        default:
            break;
        }
    }
}

static int gtx8_send(I2CSlave *i2c, uint8_t data)
{
    GoodixGTX8State *s = GOODIX_GTX8(i2c);

    if (s->pointer_bytes == 0) {
        s->pointer = (uint16_t)data << 8;
        s->pointer_bytes = 1;
    } else if (s->pointer_bytes == 1) {
        s->pointer |= data;
        s->pointer_bytes = 2;
    } else {
        gtx8_write_register(s, s->pointer++, data);
    }
    return 0;
}

static uint8_t gtx8_recv(I2CSlave *i2c)
{
    GoodixGTX8State *s = GOODIX_GTX8(i2c);

    return s->regs[s->pointer++];
}

static int gtx8_event(I2CSlave *i2c, enum i2c_event event)
{
    GoodixGTX8State *s = GOODIX_GTX8(i2c);

    if (event == I2C_START_SEND) {
        s->pointer_bytes = 0;
    }
    return 0;
}

static void gtx8_register_reset(GoodixGTX8State *s)
{
    uint8_t checksum = 0;

    memset(s->regs, 0, sizeof(s->regs));
    memcpy(&s->regs[GTX8_REG_PID], "6861", 4);
    s->regs[GTX8_REG_VID + 0] = 0x01;
    s->regs[GTX8_REG_VID + 1] = 0x00;
    s->regs[GTX8_REG_VID + 2] = 0x01;
    s->regs[GTX8_REG_VID + 3] = 0x51;
    s->regs[GTX8_REG_SENSOR_ID] = 0;
    for (unsigned i = 0; i < GTX8_VERSION_LENGTH - 1; i++) {
        checksum += s->regs[GTX8_REG_VERSION_BASE + i];
    }
    s->regs[GTX8_REG_VERSION_BASE + GTX8_VERSION_LENGTH - 1] =
        0 - checksum;

    s->regs[GTX8_REG_COMMAND] = GTX8_CMD_READY;
    /* A valid newer empty config lets the stock driver skip reflashing it. */
    s->regs[GTX8_REG_CONFIG + 0] = 0xff;
    s->regs[GTX8_REG_CONFIG + 1] = 0;
    s->regs[GTX8_REG_CONFIG + 2] = 0;
    s->regs[GTX8_REG_CONFIG + 3] = 1;
    s->regs[GTX8_REG_ESD_TICK] = 0;

    s->pointer = 0;
    s->pointer_bytes = 0;
    s->input_x = 0;
    s->input_y = 0;
    s->report_x = 0;
    s->report_y = 0;
    s->input_pressed = false;
    s->report_pressed = false;
    s->touch_head = 0;
    s->touch_count = 0;
    s->touch_report_active = false;
    if (s->irq_timer) {
        timer_del(s->irq_timer);
    }
    qemu_set_irq(s->irq, 1);
}

static void gtx8_reset_input(void *opaque, int line, int level)
{
    GoodixGTX8State *s = opaque;

    if (level && !s->reset_level) {
        gtx8_register_reset(s);
    }
    s->reset_level = level;
}

static void gtx8_reset(DeviceState *dev)
{
    GoodixGTX8State *s = GOODIX_GTX8(dev);

    gtx8_register_reset(s);
    s->reset_level = false;
}

static const VMStateDescription gtx8_vmstate = {
    .name = TYPE_GOODIX_GTX8,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, GoodixGTX8State),
        VMSTATE_UINT8_ARRAY(regs, GoodixGTX8State, UINT16_MAX + 1),
        VMSTATE_UINT16(pointer, GoodixGTX8State),
        VMSTATE_UINT8(pointer_bytes, GoodixGTX8State),
        VMSTATE_BOOL(reset_level, GoodixGTX8State),
        VMSTATE_UINT16(input_x, GoodixGTX8State),
        VMSTATE_UINT16(input_y, GoodixGTX8State),
        VMSTATE_UINT16(report_x, GoodixGTX8State),
        VMSTATE_UINT16(report_y, GoodixGTX8State),
        VMSTATE_BOOL(input_pressed, GoodixGTX8State),
        VMSTATE_BOOL(report_pressed, GoodixGTX8State),
        VMSTATE_UINT16_ARRAY(touch_x, GoodixGTX8State,
                             GTX8_TOUCH_QUEUE_SIZE),
        VMSTATE_UINT16_ARRAY(touch_y, GoodixGTX8State,
                             GTX8_TOUCH_QUEUE_SIZE),
        VMSTATE_BOOL_ARRAY(touch_pressed, GoodixGTX8State,
                           GTX8_TOUCH_QUEUE_SIZE),
        VMSTATE_UINT8(touch_head, GoodixGTX8State),
        VMSTATE_UINT8(touch_count, GoodixGTX8State),
        VMSTATE_BOOL(touch_report_active, GoodixGTX8State),
        VMSTATE_END_OF_LIST()
    },
};

static void gtx8_realize(DeviceState *dev, Error **errp)
{
    GoodixGTX8State *s = GOODIX_GTX8(dev);

    s->irq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                gtx8_deliver_touch_report, s);
    s->input_handler = qemu_input_handler_register(dev,
                                                   &gtx8_input_handler);
    qemu_input_handler_activate(s->input_handler);
}

static void gtx8_unrealize(DeviceState *dev)
{
    GoodixGTX8State *s = GOODIX_GTX8(dev);

    qemu_input_handler_unregister(s->input_handler);
    timer_free(s->irq_timer);
}

static void gtx8_init(Object *obj)
{
    GoodixGTX8State *s = GOODIX_GTX8(obj);

    qdev_init_gpio_in_named(DEVICE(obj), gtx8_reset_input, "reset", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
}

static void gtx8_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, gtx8_reset);
    dc->realize = gtx8_realize;
    dc->unrealize = gtx8_unrealize;
    dc->vmsd = &gtx8_vmstate;
    sc->send = gtx8_send;
    sc->recv = gtx8_recv;
    sc->event = gtx8_event;
}

static const TypeInfo gtx8_info = {
    .name = TYPE_GOODIX_GTX8,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(GoodixGTX8State),
    .instance_init = gtx8_init,
    .class_init = gtx8_class_init,
};

static void gtx8_register_types(void)
{
    type_register_static(&gtx8_info);
}
type_init(gtx8_register_types)
