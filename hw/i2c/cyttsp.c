/*
 * Cypress TrueTouch Gen3 controller used by the Kindle Paperwhite/Celeste.
 *
 * The Lab126 2.6.31 driver uses the compact Gen3 register protocol rather
 * than the descriptor-based Gen4 protocol used by Wario.  This model covers
 * the bootloader-to-sysinfo-to-operational sequence and one host pointer
 * contact, which is sufficient for the stock driver and framework.
 */

#include "qemu/osdep.h"
#include "hw/i2c/cyttsp.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "trace.h"
#include "ui/input.h"

#define CY_REG_COUNT          256
#define CY_HST_HANDSHAKE      BIT(7)
#define CY_HST_MODE_MASK      (7 << 4)
#define CY_HST_SYSINFO        (1 << 4)
#define CY_HST_SOFT_RESET     BIT(0)
#define CY_BL_STATUS_APP      0x11
#define CY_BL_EXIT            0xa5
#define CY_TOUCH_MAX_X        757
#define CY_TOUCH_MAX_Y        1023

struct CYTTSPState {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *irq_timer;
    QEMUTimer *heartbeat_timer;
    QemuInputHandlerState *input_handler;
    uint8_t regs[CY_REG_COUNT];
    uint8_t pointer;
    uint8_t write_buf[32];
    uint8_t write_len;
    bool have_pointer;
    bool bootloader;
    bool reset_level;
    uint16_t input_x;
    uint16_t input_y;
    uint16_t report_x;
    uint16_t report_y;
    bool input_pressed;
    bool report_pressed;
};

static void cyttsp_irq_release(void *opaque)
{
    CYTTSPState *s = opaque;

    qemu_set_irq(s->irq, 1);
}

static void cyttsp_pulse_irq(CYTTSPState *s)
{
    qemu_set_irq(s->irq, 1);
    qemu_set_irq(s->irq, 0);
    timer_mod(s->irq_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
}

static void cyttsp_set_operational(CYTTSPState *s)
{
    uint8_t handshake = s->regs[0] & CY_HST_HANDSHAKE;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = handshake;
    s->bootloader = false;
    timer_del(s->heartbeat_timer);
}

static void cyttsp_set_sysinfo(CYTTSPState *s)
{
    uint8_t handshake = s->regs[0] & CY_HST_HANDSHAKE;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = handshake | CY_HST_SYSINFO;
    s->regs[1] = 0x86; /* manufacturing command complete */
    s->regs[3] = 0x00;
    s->regs[4] = 0x01;
    s->regs[5] = 0x01;
    s->regs[15] = 0x01; /* bootloader 1.1 */
    s->regs[16] = 0x01;
    s->regs[17] = 0x03; /* TrueTouch Gen3 */
    s->regs[18] = 0x00;
    s->regs[19] = 0x00; /* application ID */
    s->regs[20] = 0x01;
    s->regs[21] = 0x01; /* application version */
    s->regs[22] = 0x00;
    s->regs[28] = 0x02; /* self-capacitance scan selected by Lab126 */
    s->regs[29] = 0x10;
    s->regs[30] = 0x64;
    s->regs[31] = 0x03;
    s->bootloader = false;
    timer_del(s->heartbeat_timer);
}

static void cyttsp_set_bootloader(CYTTSPState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = 0x00;
    s->regs[1] = CY_BL_STATUS_APP;
    s->regs[2] = 0x00;
    s->regs[3] = 0x01;
    s->regs[4] = 0x01;
    s->regs[7] = 0x03;
    s->regs[8] = 0x00;
    s->regs[9] = 0x00;
    s->regs[10] = 0x01;
    s->regs[11] = 0x01;
    s->regs[12] = 0x00;
    s->bootloader = true;
    timer_mod(s->heartbeat_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 50);
}

static void cyttsp_heartbeat(void *opaque)
{
    CYTTSPState *s = opaque;

    if (!s->bootloader) {
        return;
    }
    cyttsp_pulse_irq(s);
    timer_mod(s->heartbeat_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 50);
}

static void cyttsp_input_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    CYTTSPState *s = CYTTSP(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        if (evt->u.abs.data->axis == INPUT_AXIS_X) {
            s->input_x = evt->u.abs.data->value;
        } else if (evt->u.abs.data->axis == INPUT_AXIS_Y) {
            s->input_y = evt->u.abs.data->value;
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (evt->u.btn.data->button == INPUT_BUTTON_LEFT) {
            s->input_pressed = evt->u.btn.data->down;
        }
        break;
    default:
        break;
    }
}

static void cyttsp_input_sync(DeviceState *dev)
{
    CYTTSPState *s = CYTTSP(dev);
    uint16_t x, y;

    if (s->bootloader || (s->regs[0] & CY_HST_MODE_MASK)) {
        return;
    }

    x = qemu_input_scale_axis(s->input_x, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, CY_TOUCH_MAX_X);
    y = qemu_input_scale_axis(s->input_y, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, CY_TOUCH_MAX_Y);
    trace_cyttsp_input(s->input_pressed, x, y, s->bootloader, s->regs[0]);
    if (s->input_pressed == s->report_pressed &&
        (!s->input_pressed || (x == s->report_x && y == s->report_y))) {
        return;
    }

    memset(&s->regs[1], 0, 13);
    if (s->input_pressed) {
        s->regs[2] = 1;
        /* Celeste's stock driver decodes the Gen3 wire fields as BE16. */
        s->regs[3] = x >> 8;
        s->regs[4] = x;
        s->regs[5] = y >> 8;
        s->regs[6] = y;
        s->regs[7] = 0xff;
        s->regs[8] = 0x00; /* contact 0 in the high nibble */
    }
    s->report_x = x;
    s->report_y = y;
    s->report_pressed = s->input_pressed;
    trace_cyttsp_report(s->report_pressed, s->report_x, s->report_y,
                        s->regs[2], s->regs[8]);
    cyttsp_pulse_irq(s);
}

static const QemuInputHandler cyttsp_input_handler = {
    .name = "Cypress TrueTouch Gen3",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = cyttsp_input_event,
    .sync = cyttsp_input_sync,
};

static void cyttsp_finish_write(CYTTSPState *s)
{
    uint8_t command;

    if (!s->write_len) {
        return;
    }

    command = s->write_buf[0];
    trace_cyttsp_command(command, s->write_len);
    if (s->pointer == 0 && s->write_len == 1) {
        bool mode_changed = false;

        if (command & CY_HST_SOFT_RESET) {
            cyttsp_set_bootloader(s);
            mode_changed = true;
        } else if ((command & CY_HST_MODE_MASK) == CY_HST_SYSINFO) {
            if (s->bootloader ||
                (s->regs[0] & CY_HST_MODE_MASK) != CY_HST_SYSINFO) {
                cyttsp_set_sysinfo(s);
                mode_changed = true;
            }
        } else if ((command & CY_HST_MODE_MASK) == 0) {
            if (s->bootloader || (s->regs[0] & CY_HST_MODE_MASK) != 0) {
                cyttsp_set_operational(s);
                mode_changed = true;
            }
        }
        s->regs[0] = (s->regs[0] & ~CY_HST_HANDSHAKE) |
                     (command & CY_HST_HANDSHAKE);
        if (mode_changed) {
            cyttsp_pulse_irq(s);
        } else {
            /* A handshake acknowledges the current packet and IRQ. */
            qemu_set_irq(s->irq, 1);
        }
        return;
    }

    if (s->pointer == 0 && s->write_len >= 3 &&
        s->write_buf[1] == 0xff && s->write_buf[2] == CY_BL_EXIT) {
        cyttsp_set_operational(s);
        cyttsp_pulse_irq(s);
        return;
    }

    if (s->pointer == 2) {
        s->regs[1] = 0x86;
    }
    memcpy(&s->regs[s->pointer], s->write_buf,
           MIN((size_t)s->write_len,
               sizeof(s->regs) - (size_t)s->pointer));
}

static int cyttsp_send(I2CSlave *i2c, uint8_t data)
{
    CYTTSPState *s = CYTTSP(i2c);

    if (!s->have_pointer) {
        s->pointer = data;
        s->have_pointer = true;
    } else if (s->write_len < sizeof(s->write_buf)) {
        s->write_buf[s->write_len++] = data;
    }
    return 0;
}

static uint8_t cyttsp_recv(I2CSlave *i2c)
{
    CYTTSPState *s = CYTTSP(i2c);

    if (s->pointer == 0) {
        trace_cyttsp_read(s->regs[0], s->regs[1], s->regs[2],
                          ((unsigned)s->regs[3] << 8) | s->regs[4],
                          ((unsigned)s->regs[5] << 8) | s->regs[6],
                          s->regs[7], s->regs[8]);
    }
    return s->regs[s->pointer++];
}

static int cyttsp_event(I2CSlave *i2c, enum i2c_event event)
{
    CYTTSPState *s = CYTTSP(i2c);

    if (event == I2C_START_SEND) {
        s->have_pointer = false;
        s->write_len = 0;
    } else if (event == I2C_START_RECV || event == I2C_FINISH) {
        cyttsp_finish_write(s);
        s->write_len = 0;
    }
    return 0;
}

static void cyttsp_reset_input(void *opaque, int line, int level)
{
    CYTTSPState *s = opaque;

    if (level && !s->reset_level) {
        cyttsp_set_bootloader(s);
    }
    s->reset_level = level;
}

static void cyttsp_reset(DeviceState *dev)
{
    CYTTSPState *s = CYTTSP(dev);

    s->pointer = 0;
    s->write_len = 0;
    s->have_pointer = false;
    s->reset_level = false;
    s->input_x = 0;
    s->input_y = 0;
    s->report_x = 0;
    s->report_y = 0;
    s->input_pressed = false;
    s->report_pressed = false;
    timer_del(s->irq_timer);
    timer_del(s->heartbeat_timer);
    qemu_set_irq(s->irq, 1);
    cyttsp_set_operational(s);
}

static void cyttsp_realize(DeviceState *dev, Error **errp)
{
    CYTTSPState *s = CYTTSP(dev);

    s->irq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, cyttsp_irq_release, s);
    s->heartbeat_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                      cyttsp_heartbeat, s);
    s->input_handler = qemu_input_handler_register(dev,
                                                   &cyttsp_input_handler);
    qemu_input_handler_activate(s->input_handler);
}

static void cyttsp_unrealize(DeviceState *dev)
{
    CYTTSPState *s = CYTTSP(dev);

    qemu_input_handler_unregister(s->input_handler);
    timer_free(s->irq_timer);
    timer_free(s->heartbeat_timer);
}

static const VMStateDescription cyttsp_vmstate = {
    .name = TYPE_CYTTSP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, CYTTSPState),
        VMSTATE_UINT8_ARRAY(regs, CYTTSPState, CY_REG_COUNT),
        VMSTATE_UINT8(pointer, CYTTSPState),
        VMSTATE_UINT8_ARRAY(write_buf, CYTTSPState, 32),
        VMSTATE_UINT8(write_len, CYTTSPState),
        VMSTATE_BOOL(have_pointer, CYTTSPState),
        VMSTATE_BOOL(bootloader, CYTTSPState),
        VMSTATE_BOOL(reset_level, CYTTSPState),
        VMSTATE_UINT16(input_x, CYTTSPState),
        VMSTATE_UINT16(input_y, CYTTSPState),
        VMSTATE_UINT16(report_x, CYTTSPState),
        VMSTATE_UINT16(report_y, CYTTSPState),
        VMSTATE_BOOL(input_pressed, CYTTSPState),
        VMSTATE_BOOL(report_pressed, CYTTSPState),
        VMSTATE_TIMER_PTR(irq_timer, CYTTSPState),
        VMSTATE_TIMER_PTR(heartbeat_timer, CYTTSPState),
        VMSTATE_END_OF_LIST()
    },
};

static void cyttsp_init(Object *obj)
{
    CYTTSPState *s = CYTTSP(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in(DEVICE(obj), cyttsp_reset_input, 1);
}

static void cyttsp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = cyttsp_realize;
    dc->unrealize = cyttsp_unrealize;
    device_class_set_legacy_reset(dc, cyttsp_reset);
    dc->vmsd = &cyttsp_vmstate;
    sc->send = cyttsp_send;
    sc->recv = cyttsp_recv;
    sc->event = cyttsp_event;
}

static const TypeInfo cyttsp_info = {
    .name = TYPE_CYTTSP,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(CYTTSPState),
    .instance_init = cyttsp_init,
    .class_init = cyttsp_class_init,
};

static void cyttsp_register_types(void)
{
    type_register_static(&cyttsp_info);
}

type_init(cyttsp_register_types)
