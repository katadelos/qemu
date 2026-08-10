/* Minimal Cypress TrueTouch Gen4 controller used by Lab126 Wario. */

#include "qemu/osdep.h"
#include "hw/i2c/cyttsp4.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define CY_HST_MODE_CHANGE  BIT(3)
#define CY_HST_MODE         (7 << 4)
#define CY_HST_SYSINFO      (1 << 4)
#define CY_HST_RESET        BIT(0)
#define CY_CMD_COMPLETE     BIT(6)
#define CY_LDR_HOST_SYNC    0xff
#define CY_LDR_SOP          0x01
#define CY_LDR_EOP          0x17
#define CY_LDR_VERIFY_CSUM  0x31
#define CY_LDR_ERASE_ROW    0x34
#define CY_LDR_SEND_DATA    0x37
#define CY_LDR_ENTER        0x38
#define CY_LDR_PROG_ROW     0x39
#define CY_LDR_VERIFY_ROW   0x3a
#define CY_LDR_EXIT         0x3b
#define CY_LDR_INIT         0x48

#define CY_TOUCH_MAX_X      757
#define CY_TOUCH_MAX_Y      1023
#define CY_TOUCH_REPORT_OFS 3
#define CY_TOUCH_STATUS_OFS 5
#define CY_TOUCH_RECORD_OFS 6
#define CY_TOUCH_RECORD_LEN 7
#define CY_EV_TOUCHDOWN     1
#define CY_EV_MOVE          2

struct CYTTSP4State {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *irq_timer;
    QEMUTimer *heartbeat_timer;
    QEMUTimer *exit_loader_timer;
    QemuInputHandlerState *input_handler;
    uint8_t regs[768];
    uint16_t pointer;
    uint8_t write_buf[256];
    uint16_t write_len;
    bool have_pointer;
    bool bootloader;
    bool loader_session;
    bool reset_level;
    uint16_t input_x;
    uint16_t input_y;
    uint16_t report_x;
    uint16_t report_y;
    bool input_pressed;
    bool report_pressed;
    bool invert_x;
};

static void cyttsp4_set_sysinfo(CYTTSP4State *s);

static void cyttsp4_irq_release(void *opaque)
{
    CYTTSP4State *s = opaque;

    qemu_set_irq(s->irq, 1);
}

static void cyttsp4_pulse_irq(CYTTSP4State *s)
{
    /* Ensure every controller event produces a fresh falling edge. */
    qemu_set_irq(s->irq, 1);
    qemu_set_irq(s->irq, 0);
    timer_mod(s->irq_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
}

static void cyttsp4_input_event(DeviceState *dev, QemuConsole *src,
                                InputEvent *evt)
{
    CYTTSP4State *s = CYTTSP4(dev);
    InputMoveEvent *move;
    InputBtnEvent *btn;

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        move = evt->u.abs.data;
        if (move->axis == INPUT_AXIS_X) {
            s->input_x = move->value;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->input_y = move->value;
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        btn = evt->u.btn.data;
        if (btn->button == INPUT_BUTTON_LEFT) {
            s->input_pressed = btn->down;
        }
        break;
    default:
        break;
    }
}

static void cyttsp4_input_sync(DeviceState *dev)
{
    CYTTSP4State *s = CYTTSP4(dev);
    uint16_t x, y;
    uint8_t event;

    if (s->bootloader || (s->regs[0] & CY_HST_MODE) != 0) {
        return;
    }

    x = qemu_input_scale_axis(s->input_x, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, CY_TOUCH_MAX_X);
    y = qemu_input_scale_axis(s->input_y, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, CY_TOUCH_MAX_Y);
    if (s->invert_x) {
        x = CY_TOUCH_MAX_X - x;
    }

    if (s->input_pressed == s->report_pressed &&
        (!s->input_pressed || (x == s->report_x && y == s->report_y))) {
        return;
    }

    memset(&s->regs[CY_TOUCH_RECORD_OFS], 0, CY_TOUCH_RECORD_LEN);
    s->regs[CY_TOUCH_REPORT_OFS + 1] = 0; /* valid report */

    if (s->input_pressed) {
        event = s->report_pressed ? CY_EV_MOVE : CY_EV_TOUCHDOWN;
        s->regs[CY_TOUCH_REPORT_OFS] =
            CY_TOUCH_RECORD_OFS + CY_TOUCH_RECORD_LEN -
            CY_TOUCH_REPORT_OFS;
        s->regs[CY_TOUCH_STATUS_OFS] = 1;
        s->regs[CY_TOUCH_RECORD_OFS] = x >> 8;
        s->regs[CY_TOUCH_RECORD_OFS + 1] = x;
        s->regs[CY_TOUCH_RECORD_OFS + 2] = y >> 8;
        s->regs[CY_TOUCH_RECORD_OFS + 3] = y;
        s->regs[CY_TOUCH_RECORD_OFS + 4] = 0xff; /* pressure */
        /* Tracking ID 0, event in bits 3:2, standard finger object. */
        s->regs[CY_TOUCH_RECORD_OFS + 5] = event << 2;
        s->regs[CY_TOUCH_RECORD_OFS + 6] = 1; /* contact width */
    } else {
        s->regs[CY_TOUCH_REPORT_OFS] =
            CY_TOUCH_RECORD_OFS - CY_TOUCH_REPORT_OFS;
        s->regs[CY_TOUCH_STATUS_OFS] = 0;
    }

    s->report_x = x;
    s->report_y = y;
    s->report_pressed = s->input_pressed;
    cyttsp4_pulse_irq(s);
}

static const QemuInputHandler cyttsp4_input_handler = {
    .name = "Cypress TrueTouch Gen4",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = cyttsp4_input_event,
    .sync = cyttsp4_input_sync,
};

static void cyttsp4_heartbeat(void *opaque)
{
    CYTTSP4State *s = opaque;

    if (!s->bootloader) {
        return;
    }
    cyttsp4_pulse_irq(s);
    timer_mod(s->heartbeat_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 50);
}

static void cyttsp4_exit_loader(void *opaque)
{
    CYTTSP4State *s = opaque;

    cyttsp4_set_sysinfo(s);
    cyttsp4_pulse_irq(s);
}

static void cyttsp4_set_bootloader(CYTTSP4State *s)
{
    timer_del(s->exit_loader_timer);
    memset(s->regs, 0, sizeof(s->regs));
    /* Either bit 0/6 in HST_MODE or RESET_DETECT marks bootloader mode. */
    s->regs[0] = 0x01;
    s->regs[1] = 0x10;
    s->bootloader = true;
    s->loader_session = false;
    timer_mod(s->heartbeat_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 50);
}

static void cyttsp4_set_sysinfo(CYTTSP4State *s)
{
    static const uint8_t sysinfo_header[] = {
        0x10, 0x00, 0x00, 0x5c, 0x00, 0x10, 0x00, 0x2b,
        0x00, 0x2d, 0x00, 0x39, 0x00, 0x59, 0x00, 0x5b,
    };
    uint8_t *opcfg;

    memset(s->regs, 0, sizeof(s->regs));
    memcpy(s->regs, sysinfo_header, sizeof(sysinfo_header));

    /* CYDATA: product, firmware, bootloader and one-byte manufacturer ID. */
    /* Exact identity reported by the production controller in the live unit. */
    s->regs[0x10] = 0x00;
    s->regs[0x11] = 0x02;
    s->regs[0x12] = 0x01;
    s->regs[0x13] = 0x01;
    s->regs[0x19] = 0x09;
    s->regs[0x1a] = 0x0d;
    s->regs[0x1b] = 0xbd;
    s->regs[0x1c] = 0x02;
    s->regs[0x1d] = 0x00;
    s->regs[0x22] = 1;
    s->regs[0x23] = 0x01;
    s->regs[0x29] = 0x04;
    s->regs[0x2a] = 0x00;

    s->regs[0x2c] = 0x0a; /* Live panel-test/scanning status. */

    /* Panel configuration: 758 x 1024, 8-bit pressure. */
    s->regs[0x2d] = 24;
    s->regs[0x2e] = 32;
    s->regs[0x33] = 0x02;
    s->regs[0x34] = 0xf6;
    s->regs[0x35] = 0x04;
    s->regs[0x36] = 0x00;
    s->regs[0x37] = 0x00;
    s->regs[0x38] = 0xff;

    opcfg = &s->regs[0x39];
    opcfg[0] = 2;       /* command offset */
    opcfg[1] = 3;       /* report offset */
    opcfg[3] = 0x50;    /* report size */
    opcfg[5] = 5;       /* touch-status offset */
    opcfg[7] = 10;      /* maximum touches */
    opcfg[8] = 7;       /* legacy touch record size */
    opcfg[9] = 0;  opcfg[10] = 12; /* X */
    opcfg[11] = 2; opcfg[12] = 12; /* Y */
    opcfg[13] = 4; opcfg[14] = 8;  /* pressure */
    opcfg[15] = 5; opcfg[16] = 4;  /* tracking ID */
    opcfg[17] = 5; opcfg[18] = 2;  /* event ID */
    opcfg[19] = 5; opcfg[20] = 2;  /* object ID */
    opcfg[21] = 6; opcfg[22] = 8;  /* width */

    /*
     * Lab126's userspace compares these two DDATA bytes with the firmware
     * selected for this platform before it loads cyttsp4_loader.  The stock
     * 2023 Wario rootfs selects V3_000E.bin, so presenting the older 0017
     * value from the reference device unnecessarily starts a firmware update
     * on every fresh emulated boot.
     */
    s->regs[0x59] = 0x00;
    s->regs[0x5a] = 0x0e;
    s->bootloader = false;
    timer_del(s->heartbeat_timer);
}

static bool cyttsp4_loader_command(CYTTSP4State *s)
{
    uint8_t command;

    if (!s->bootloader || s->pointer != 0 || s->write_len < 3 ||
        s->write_buf[0] != CY_LDR_HOST_SYNC ||
        s->write_buf[1] != CY_LDR_SOP) {
        return false;
    }

    command = s->write_buf[2];
    if (command == CY_LDR_EXIT) {
        /* Real silicon changes mode after the I2C transaction completes. */
        timer_mod(s->exit_loader_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                  (s->loader_session ? 20 : 1));
        s->loader_session = false;
        return true;
    }

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = CY_LDR_SOP;
    s->regs[1] = 0; /* ERROR_SUCCESS */

    switch (command) {
    case CY_LDR_ENTER:
        s->loader_session = true;
        /* Eight response bytes: silicon ID, revision and BL version. */
        s->regs[2] = 8;
        s->regs[4] = 0x04;
        s->regs[5] = 0x24;
        s->regs[6] = 0x01;
        s->regs[7] = 0x00;
        s->regs[8] = 1;
        s->regs[9] = 1;
        s->regs[10] = 0;
        s->regs[11] = 1;
        s->regs[14] = CY_LDR_EOP;
        break;
    case CY_LDR_VERIFY_ROW:
    case CY_LDR_VERIFY_CSUM:
        s->regs[2] = 1;
        s->regs[4] = 1;
        s->regs[7] = CY_LDR_EOP;
        break;
    case CY_LDR_INIT:
    case CY_LDR_ERASE_ROW:
    case CY_LDR_SEND_DATA:
    case CY_LDR_PROG_ROW:
        s->regs[6] = CY_LDR_EOP;
        break;
    default:
        s->regs[1] = 15; /* ERROR_INVALID_COMMAND */
        s->regs[6] = CY_LDR_EOP;
        break;
    }

    cyttsp4_pulse_irq(s);
    return true;
}

static void cyttsp4_finish_write(CYTTSP4State *s)
{
    uint8_t command;

    if (!s->write_len) {
        return;
    }

    if (cyttsp4_loader_command(s)) {
        return;
    }

    if (s->pointer == 0 && s->write_len == 1) {
        command = s->write_buf[0];
        if (command & CY_HST_RESET) {
            cyttsp4_set_bootloader(s);
            return;
        }
        if (command & CY_HST_MODE_CHANGE) {
            s->regs[0] = command & (CY_HST_MODE | BIT(7));
            /* Operational/CaT maps expose an idle command register. */
            if ((command & CY_HST_MODE) != CY_HST_SYSINFO) {
                s->regs[1] = 0;
                s->regs[2] = CY_CMD_COMPLETE;
            }
            cyttsp4_pulse_irq(s);
            return;
        }
        /* Host handshake/low-power toggles only update HST_MODE. */
        s->regs[0] = command;
        /*
         * The controller holds INT low until the host handshakes the
         * report.  Releasing it here is important: startup changes mode
         * immediately after acknowledging the SYSINFO interrupt, often
         * before the fallback pulse timer has expired.
         */
        timer_del(s->irq_timer);
        qemu_set_irq(s->irq, 1);
        return;
    }

    if (s->pointer == 2) {
        command = s->write_buf[0];
        memcpy(&s->regs[2], s->write_buf, s->write_len);
        s->regs[2] = command | CY_CMD_COMPLETE;
        memset(&s->regs[3], 0, 8);
        cyttsp4_pulse_irq(s);
        return;
    }

    memcpy(&s->regs[s->pointer], s->write_buf, s->write_len);
}

static int cyttsp4_send(I2CSlave *i2c, uint8_t data)
{
    CYTTSP4State *s = CYTTSP4(i2c);

    if (!s->have_pointer) {
        s->pointer = data;
        s->have_pointer = true;
        return 0;
    }
    if (s->write_len < sizeof(s->write_buf)) {
        s->write_buf[s->write_len++] = data;
    }
    return 0;
}

static uint8_t cyttsp4_recv(I2CSlave *i2c)
{
    CYTTSP4State *s = CYTTSP4(i2c);

    return s->regs[s->pointer++ % sizeof(s->regs)];
}

static int cyttsp4_event(I2CSlave *i2c, enum i2c_event event)
{
    CYTTSP4State *s = CYTTSP4(i2c);

    if (event == I2C_START_SEND) {
        s->have_pointer = false;
        s->write_len = 0;
    } else if (event == I2C_START_RECV || event == I2C_FINISH) {
        cyttsp4_finish_write(s);
        s->write_len = 0;
    }
    return 0;
}

static void cyttsp4_reset_input(void *opaque, int line, int level)
{
    CYTTSP4State *s = opaque;

    if (level && !s->reset_level) {
        cyttsp4_set_bootloader(s);
    }
    s->reset_level = level;
}

static void cyttsp4_reset(DeviceState *dev)
{
    CYTTSP4State *s = CYTTSP4(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->write_len = 0;
    s->have_pointer = false;
    s->bootloader = false;
    s->loader_session = false;
    s->reset_level = false;
    s->input_x = 0;
    s->input_y = 0;
    s->report_x = 0;
    s->report_y = 0;
    s->input_pressed = false;
    s->report_pressed = false;
    timer_del(s->irq_timer);
    timer_del(s->heartbeat_timer);
    timer_del(s->exit_loader_timer);
    qemu_set_irq(s->irq, 1);
}

static void cyttsp4_realize(DeviceState *dev, Error **errp)
{
    CYTTSP4State *s = CYTTSP4(dev);

    s->irq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                cyttsp4_irq_release, s);
    s->heartbeat_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                      cyttsp4_heartbeat, s);
    s->exit_loader_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                        cyttsp4_exit_loader, s);
    s->input_handler = qemu_input_handler_register(dev,
                                                   &cyttsp4_input_handler);
    qemu_input_handler_activate(s->input_handler);
}

static void cyttsp4_unrealize(DeviceState *dev)
{
    CYTTSP4State *s = CYTTSP4(dev);

    qemu_input_handler_unregister(s->input_handler);
    timer_free(s->irq_timer);
    timer_free(s->heartbeat_timer);
    timer_free(s->exit_loader_timer);
}

static const VMStateDescription cyttsp4_vmstate = {
    .name = TYPE_CYTTSP4,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, CYTTSP4State),
        VMSTATE_UINT8_ARRAY(regs, CYTTSP4State, 768),
        VMSTATE_UINT16(pointer, CYTTSP4State),
        VMSTATE_UINT8_ARRAY(write_buf, CYTTSP4State, 256),
        VMSTATE_UINT16(write_len, CYTTSP4State),
        VMSTATE_BOOL(have_pointer, CYTTSP4State),
        VMSTATE_BOOL(bootloader, CYTTSP4State),
        VMSTATE_BOOL(loader_session, CYTTSP4State),
        VMSTATE_BOOL(reset_level, CYTTSP4State),
        VMSTATE_UINT16(input_x, CYTTSP4State),
        VMSTATE_UINT16(input_y, CYTTSP4State),
        VMSTATE_UINT16(report_x, CYTTSP4State),
        VMSTATE_UINT16(report_y, CYTTSP4State),
        VMSTATE_BOOL(input_pressed, CYTTSP4State),
        VMSTATE_BOOL(report_pressed, CYTTSP4State),
        VMSTATE_TIMER_PTR(irq_timer, CYTTSP4State),
        VMSTATE_TIMER_PTR(heartbeat_timer, CYTTSP4State),
        VMSTATE_TIMER_PTR(exit_loader_timer, CYTTSP4State),
        VMSTATE_END_OF_LIST()
    },
};

static void cyttsp4_init(Object *obj)
{
    CYTTSP4State *s = CYTTSP4(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in(DEVICE(obj), cyttsp4_reset_input, 1);
}

static const Property cyttsp4_properties[] = {
    DEFINE_PROP_BOOL("invert-x", CYTTSP4State, invert_x, false),
};

static void cyttsp4_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = cyttsp4_realize;
    dc->unrealize = cyttsp4_unrealize;
    device_class_set_legacy_reset(dc, cyttsp4_reset);
    dc->vmsd = &cyttsp4_vmstate;
    device_class_set_props(dc, cyttsp4_properties);
    sc->send = cyttsp4_send;
    sc->recv = cyttsp4_recv;
    sc->event = cyttsp4_event;
}

static const TypeInfo cyttsp4_info = {
    .name = TYPE_CYTTSP4,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(CYTTSP4State),
    .instance_init = cyttsp4_init,
    .class_init = cyttsp4_class_init,
};

static void cyttsp4_register_types(void)
{
    type_register_static(&cyttsp4_info);
}
type_init(cyttsp4_register_types)
