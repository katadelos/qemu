/* FocalTech FT5536G capacitive touchscreen controller model. */

#include "qemu/osdep.h"
#include "hw/i2c/ft5536g.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define FT5536G_CHIP_ID_HIGH_REG  0xa3
#define FT5536G_CHIP_ID_LOW_REG   0x9f
#define FT5536G_FW_VERSION_REG    0xa6
#define FT5536G_VENDOR_ID_REG     0xa8
#define FT5536G_MODULE_ID_REG     0xe3

#define FT5536G_CHIP_ID_HIGH      0x54
#define FT5536G_CHIP_ID_LOW       0x52
#define FT5536G_BOOTLOADER_ID_LOW 0x5c
#define FT5536G_COLOR_VENDOR_ID   0x60
#define FT5536G_COMMAND_MAX       32
#define FT5536G_CONFIG_SIZE       32
#define FT5536G_TOUCH_DATA_REG    0x01
#define FT5536G_TOUCH_DATA_SIZE   21
#define FT5536G_TOUCH_RECORD      4
#define FT5536G_TOUCH_QUEUE_SIZE  16
#define FT5536G_TOUCH_AREA_MIN    0x20
#define FT5536G_TOUCH_AREA_MAX    0x21
#define FT5536G_X_MAX             1272
#define FT5536G_Y_MAX             1696

#define FT5536G_TOUCH_DOWN        0
#define FT5536G_TOUCH_UP          1
#define FT5536G_TOUCH_CONTACT     2

typedef enum FT5536GResponse {
    FT5536G_RESPONSE_REGISTERS,
    FT5536G_RESPONSE_HID,
    FT5536G_RESPONSE_BOOT_ID,
    FT5536G_RESPONSE_CONFIG,
} FT5536GResponse;

struct FT5536GState {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t command[FT5536G_COMMAND_MAX];
    uint8_t command_len;
    uint8_t pointer;
    uint8_t response_pos;
    uint8_t response;
    bool sending;
    bool reset_level;
    bool bootloader;
    bool upgrade_aa;
    qemu_irq irq;
    QEMUTimer *irq_timer;
    QemuInputHandlerState *input_handler;
    uint16_t input_x;
    uint16_t input_y;
    uint16_t report_x;
    uint16_t report_y;
    bool input_pressed;
    bool report_pressed;
    uint8_t contact_area;
    uint16_t touch_x[FT5536G_TOUCH_QUEUE_SIZE];
    uint16_t touch_y[FT5536G_TOUCH_QUEUE_SIZE];
    uint8_t touch_event[FT5536G_TOUCH_QUEUE_SIZE];
    uint8_t touch_area[FT5536G_TOUCH_QUEUE_SIZE];
    uint8_t touch_head;
    uint8_t touch_count;
    bool touch_report_active;
    bool touch_read_active;
};

static void ft5536g_deliver_touch_report(void *opaque)
{
    FT5536GState *s = opaque;
    uint8_t *report = &s->regs[FT5536G_TOUCH_DATA_REG];
    unsigned index;
    uint32_t x16, y16;
    uint8_t area, event;

    if (!s->touch_count || s->touch_report_active || s->bootloader) {
        return;
    }

    index = s->touch_head;
    event = s->touch_event[index];
    area = s->touch_area[index];
    x16 = (uint32_t)s->touch_x[index] << 4;
    y16 = (uint32_t)s->touch_y[index] << 4;
    memset(report, 0, FT5536G_TOUCH_DATA_SIZE);

    /*
     * Protocol-v2 reports are what the Colorsoft's focaltech driver reads:
     * byte 1 is 0x2n and each contact starts at byte 4. Coordinates carry
     * four fractional bits which the driver divides back down by sixteen.
     */
    report[1] = 0x20 | 1;
    report[FT5536G_TOUCH_RECORD + 0] = (event << 6) | (x16 >> 12);
    report[FT5536G_TOUCH_RECORD + 1] = x16 >> 4;
    /* Tracking ID zero occupies the high nibble. */
    report[FT5536G_TOUCH_RECORD + 2] = y16 >> 12;
    report[FT5536G_TOUCH_RECORD + 3] = y16 >> 4;
    report[FT5536G_TOUCH_RECORD + 4] = (x16 << 4) | (y16 & 0x0f);
    report[FT5536G_TOUCH_RECORD + 5] = area;
    report[FT5536G_TOUCH_RECORD + 6] = area;

    /* Hold the active-low IRQ until the guest consumes this report. */
    qemu_set_irq(s->irq, 1);
    qemu_set_irq(s->irq, 0);
    s->touch_report_active = true;
}

static void ft5536g_queue_touch_report(FT5536GState *s, uint16_t x,
                                       uint16_t y, uint8_t event,
                                       uint8_t area)
{
    unsigned tail;

    if (event == FT5536G_TOUCH_CONTACT && s->touch_count) {
        tail = (s->touch_head + s->touch_count - 1) %
               FT5536G_TOUCH_QUEUE_SIZE;
        if (s->touch_event[tail] == FT5536G_TOUCH_CONTACT &&
            !(s->touch_report_active && tail == s->touch_head)) {
            s->touch_x[tail] = x;
            s->touch_y[tail] = y;
            s->touch_area[tail] = area;
            return;
        }
    }

    if (s->touch_count == FT5536G_TOUCH_QUEUE_SIZE) {
        /* Preserve a release transition by replacing the last queued move. */
        tail = (s->touch_head + s->touch_count - 1) %
               FT5536G_TOUCH_QUEUE_SIZE;
        if (event == FT5536G_TOUCH_UP &&
            s->touch_event[tail] == FT5536G_TOUCH_CONTACT) {
            s->touch_x[tail] = x;
            s->touch_y[tail] = y;
            s->touch_event[tail] = event;
            s->touch_area[tail] = area;
        }
        return;
    }

    tail = (s->touch_head + s->touch_count) % FT5536G_TOUCH_QUEUE_SIZE;
    s->touch_x[tail] = x;
    s->touch_y[tail] = y;
    s->touch_event[tail] = event;
    s->touch_area[tail] = area;
    s->touch_count++;
    if (!timer_pending(s->irq_timer)) {
        ft5536g_deliver_touch_report(s);
    }
}

static void ft5536g_acknowledge_touch_report(FT5536GState *s)
{
    if (!s->touch_report_active || !s->touch_count) {
        return;
    }

    qemu_set_irq(s->irq, 1);
    s->touch_report_active = false;
    s->touch_head = (s->touch_head + 1) % FT5536G_TOUCH_QUEUE_SIZE;
    s->touch_count--;
    if (s->touch_count) {
        /* Let the one-shot handler return before presenting another edge. */
        timer_mod(s->irq_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
    }
}

static void ft5536g_input_event(DeviceState *dev, QemuConsole *src,
                                InputEvent *evt)
{
    FT5536GState *s = FT5536G(dev);

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

static void ft5536g_input_sync(DeviceState *dev)
{
    FT5536GState *s = FT5536G(dev);
    uint16_t x, y;
    uint8_t event;

    if (s->bootloader) {
        return;
    }

    x = qemu_input_scale_axis(s->input_x, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, FT5536G_X_MAX);
    y = qemu_input_scale_axis(s->input_y, INPUT_EVENT_ABS_MIN,
                              INPUT_EVENT_ABS_MAX, 0, FT5536G_Y_MAX);
    if (s->input_pressed == s->report_pressed &&
        (!s->input_pressed || (x == s->report_x && y == s->report_y))) {
        return;
    }

    event = s->input_pressed ?
            (s->report_pressed ? FT5536G_TOUCH_CONTACT :
                                 FT5536G_TOUCH_DOWN) :
            FT5536G_TOUCH_UP;
    if (event == FT5536G_TOUCH_DOWN) {
        /*
         * Real contacts do not report an invariant minimum-sized area.  The
         * stock X multitouch driver rejects touch-major values at or below
         * five percent of the advertised 0xff range, and Linux suppresses
         * repeated ABS values.  Alternate two realistic areas so a new X
         * server also observes the first contact after reopening event1.
         */
        s->contact_area = s->contact_area == FT5536G_TOUCH_AREA_MAX ?
                          FT5536G_TOUCH_AREA_MIN : s->contact_area + 1;
    }
    ft5536g_queue_touch_report(s, x, y, event, s->contact_area);
    s->report_x = x;
    s->report_y = y;
    s->report_pressed = s->input_pressed;
}

static const QemuInputHandler ft5536g_input_handler = {
    .name = "FocalTech FT5536G touchscreen",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = ft5536g_input_event,
    .sync = ft5536g_input_sync,
};

static void ft5536g_register_reset(FT5536GState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[FT5536G_CHIP_ID_HIGH_REG] = FT5536G_CHIP_ID_HIGH;
    s->regs[FT5536G_CHIP_ID_LOW_REG] = FT5536G_CHIP_ID_LOW;
    /* A maximum version prevents the stock auto-updater rewriting the model. */
    s->regs[FT5536G_FW_VERSION_REG] = 0xff;
    s->regs[FT5536G_VENDOR_ID_REG] = FT5536G_COLOR_VENDOR_ID;
    s->regs[FT5536G_MODULE_ID_REG] = 0;
    s->pointer = 0;
    s->command_len = 0;
    s->response_pos = 0;
    s->response = FT5536G_RESPONSE_REGISTERS;
    s->sending = false;
    s->bootloader = false;
    s->upgrade_aa = false;
    s->input_x = 0;
    s->input_y = 0;
    s->report_x = 0;
    s->report_y = 0;
    s->input_pressed = false;
    s->report_pressed = false;
    s->contact_area = FT5536G_TOUCH_AREA_MIN - 1;
    s->touch_head = 0;
    s->touch_count = 0;
    s->touch_report_active = false;
    s->touch_read_active = false;
    if (s->irq_timer) {
        timer_del(s->irq_timer);
    }
    qemu_set_irq(s->irq, 1);
}

static void ft5536g_finish_command(FT5536GState *s)
{
    static const uint8_t hid_command[] = { 0xeb, 0xaa, 0x09 };
    static const uint8_t boot_id_command[] = { 0x90, 0x00, 0x00, 0x00 };

    if (!s->command_len) {
        return;
    }

    s->pointer = s->command[0];
    s->response = FT5536G_RESPONSE_REGISTERS;
    if (s->command_len == sizeof(hid_command) &&
        !memcmp(s->command, hid_command, sizeof(hid_command))) {
        s->response = FT5536G_RESPONSE_HID;
    } else if (s->command[0] == boot_id_command[0] &&
               (s->command_len == 1 ||
                (s->command_len == sizeof(boot_id_command) &&
                 !memcmp(s->command, boot_id_command,
                         sizeof(boot_id_command))))) {
        s->response = FT5536G_RESPONSE_BOOT_ID;
    } else if ((s->command_len == 1 && s->command[0] == 0xa8) ||
               (s->command_len == 4 && s->command[0] == 0x03)) {
        s->response = FT5536G_RESPONSE_CONFIG;
    } else if (s->command_len == 1 && s->command[0] == 0x07) {
        s->bootloader = false;
        s->upgrade_aa = false;
    } else {
        for (unsigned i = 1; i < s->command_len; i++) {
            s->regs[s->pointer++] = s->command[i];
        }
        if (s->command_len == 2 && s->command[0] == 0xfc) {
            if (s->command[1] == 0xaa) {
                s->upgrade_aa = true;
            } else if (s->command[1] == 0x55 && s->upgrade_aa) {
                s->bootloader = true;
                s->upgrade_aa = false;
            }
        }
        s->pointer = s->command[0];
    }
    s->response_pos = 0;
    s->command_len = 0;
}

static int ft5536g_send(I2CSlave *i2c, uint8_t data)
{
    FT5536GState *s = FT5536G(i2c);

    if (s->command_len < sizeof(s->command)) {
        s->command[s->command_len++] = data;
    }
    return 0;
}

static uint8_t ft5536g_recv(I2CSlave *i2c)
{
    FT5536GState *s = FT5536G(i2c);
    static const uint8_t hid_response[] = { 0xeb, 0xaa, 0x08 };
    uint8_t config[FT5536G_CONFIG_SIZE] = { 0 };

    switch (s->response) {
    case FT5536G_RESPONSE_HID:
        return s->response_pos < sizeof(hid_response) ?
               hid_response[s->response_pos++] : 0;
    case FT5536G_RESPONSE_BOOT_ID:
        if (s->response_pos++ == 0) {
            return FT5536G_CHIP_ID_HIGH;
        }
        return s->response_pos == 2 ?
               (s->bootloader ? FT5536G_BOOTLOADER_ID_LOW :
                                FT5536G_CHIP_ID_LOW) : 0;
    case FT5536G_RESPONSE_CONFIG:
        /* Color-panel module 0x0060, with each ID followed by its inverse. */
        config[4] = FT5536G_COLOR_VENDOR_ID;
        config[5] = ~FT5536G_COLOR_VENDOR_ID;
        config[30] = 0;
        config[31] = 0xff;
        return s->response_pos < sizeof(config) ?
               config[s->response_pos++] : 0;
    case FT5536G_RESPONSE_REGISTERS:
    default:
        return s->regs[s->pointer++];
    }
}

static int ft5536g_event(I2CSlave *i2c, enum i2c_event event)
{
    FT5536GState *s = FT5536G(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->command_len = 0;
        s->sending = true;
        break;
    case I2C_START_RECV:
        if (s->sending) {
            ft5536g_finish_command(s);
        }
        s->sending = false;
        s->touch_read_active =
            s->touch_report_active &&
            s->response == FT5536G_RESPONSE_REGISTERS &&
            s->pointer == FT5536G_TOUCH_DATA_REG;
        break;
    case I2C_FINISH:
        if (s->sending) {
            ft5536g_finish_command(s);
        } else if (s->touch_read_active) {
            ft5536g_acknowledge_touch_report(s);
        }
        s->sending = false;
        s->touch_read_active = false;
        break;
    default:
        break;
    }
    return 0;
}

static void ft5536g_reset_input(void *opaque, int line, int level)
{
    FT5536GState *s = opaque;

    if (!level && s->reset_level) {
        ft5536g_register_reset(s);
    }
    s->reset_level = level;
}

static void ft5536g_reset(DeviceState *dev)
{
    FT5536GState *s = FT5536G(dev);

    ft5536g_register_reset(s);
    s->reset_level = false;
    qemu_set_irq(s->irq, 1);
}

static const VMStateDescription ft5536g_vmstate = {
    .name = TYPE_FT5536G,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FT5536GState),
        VMSTATE_UINT8_ARRAY(regs, FT5536GState, 256),
        VMSTATE_UINT8_ARRAY(command, FT5536GState, FT5536G_COMMAND_MAX),
        VMSTATE_UINT8(command_len, FT5536GState),
        VMSTATE_UINT8(pointer, FT5536GState),
        VMSTATE_UINT8(response_pos, FT5536GState),
        VMSTATE_UINT8(response, FT5536GState),
        VMSTATE_BOOL(sending, FT5536GState),
        VMSTATE_BOOL(reset_level, FT5536GState),
        VMSTATE_BOOL(bootloader, FT5536GState),
        VMSTATE_BOOL(upgrade_aa, FT5536GState),
        VMSTATE_UINT16(input_x, FT5536GState),
        VMSTATE_UINT16(input_y, FT5536GState),
        VMSTATE_UINT16(report_x, FT5536GState),
        VMSTATE_UINT16(report_y, FT5536GState),
        VMSTATE_BOOL(input_pressed, FT5536GState),
        VMSTATE_BOOL(report_pressed, FT5536GState),
        VMSTATE_UINT8(contact_area, FT5536GState),
        VMSTATE_UINT16_ARRAY(touch_x, FT5536GState,
                             FT5536G_TOUCH_QUEUE_SIZE),
        VMSTATE_UINT16_ARRAY(touch_y, FT5536GState,
                             FT5536G_TOUCH_QUEUE_SIZE),
        VMSTATE_UINT8_ARRAY(touch_event, FT5536GState,
                            FT5536G_TOUCH_QUEUE_SIZE),
        VMSTATE_UINT8_ARRAY(touch_area, FT5536GState,
                            FT5536G_TOUCH_QUEUE_SIZE),
        VMSTATE_UINT8(touch_head, FT5536GState),
        VMSTATE_UINT8(touch_count, FT5536GState),
        VMSTATE_BOOL(touch_report_active, FT5536GState),
        VMSTATE_BOOL(touch_read_active, FT5536GState),
        VMSTATE_END_OF_LIST()
    },
};

static void ft5536g_realize(DeviceState *dev, Error **errp)
{
    FT5536GState *s = FT5536G(dev);

    s->irq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                ft5536g_deliver_touch_report, s);
    s->input_handler = qemu_input_handler_register(dev,
                                                   &ft5536g_input_handler);
    qemu_input_handler_activate(s->input_handler);
}

static void ft5536g_unrealize(DeviceState *dev)
{
    FT5536GState *s = FT5536G(dev);

    qemu_input_handler_unregister(s->input_handler);
    timer_free(s->irq_timer);
}

static void ft5536g_init(Object *obj)
{
    FT5536GState *s = FT5536G(obj);

    qdev_init_gpio_in_named(DEVICE(obj), ft5536g_reset_input, "reset", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
}

static void ft5536g_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, ft5536g_reset);
    dc->realize = ft5536g_realize;
    dc->unrealize = ft5536g_unrealize;
    dc->vmsd = &ft5536g_vmstate;
    sc->send = ft5536g_send;
    sc->recv = ft5536g_recv;
    sc->event = ft5536g_event;
}

static const TypeInfo ft5536g_info = {
    .name = TYPE_FT5536G,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FT5536GState),
    .instance_init = ft5536g_init,
    .class_init = ft5536g_class_init,
};

static void ft5536g_register_types(void)
{
    type_register_static(&ft5536g_info);
}
type_init(ft5536g_register_types)
