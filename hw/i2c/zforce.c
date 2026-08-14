/*
 * Neonode zForce touchscreen used by Kobo Touch and Kindle Touch, plus the
 * TPS65185 display PMIC used by Kobo Touch.
 *
 * The touchscreen protocol follows drivers/input/touchscreen/neonode.c in
 * Kobo's linux-2.6.35.3 tree and Lab126's Yoshi linux-2.6.31 tree. A response
 * is read as two I2C transactions: first the 0xee delimiter and payload
 * length, then the payload itself.
 */

#include "qemu/osdep.h"
#include "hw/i2c/zforce.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "trace.h"
#include "ui/input.h"

#define ZFORCE_FRAME_START 0xee
#define ZFORCE_ACTIVATE    0x01
#define ZFORCE_RESOLUTION  0x02
#define ZFORCE_TOUCH_DATA  0x04
#define ZFORCE_BOOT_DONE   0x07
#define ZFORCE_SCAN_FREQ   0x08
#define ZFORCE_VERSION     0x0a
#define ZFORCE_MCU_STATUS  0x1e
#define ZFORCE_LCD_LEVEL   0x1c
#define ZFORCE_DEACTIVATE  0x00
#define ZFORCE_INVALID     0xfe
#define ZFORCE_WIDTH       600
#define ZFORCE_HEIGHT      800
#define ZFORCE_KINDLE_MAX  4095
#define ZFORCE_QUEUE_LEN   8
#define ZFORCE_PAYLOAD_MAX 129

typedef struct ZForcePacket {
    uint8_t len;
    uint8_t data[ZFORCE_PAYLOAD_MAX];
} ZForcePacket;

struct ZForceState {
    I2CSlave parent_obj;
    qemu_irq irq;
    QemuInputHandlerState *input_handler;
    ZForcePacket queue[ZFORCE_QUEUE_LEN];
    uint8_t queue_head;
    uint8_t queue_count;
    uint8_t write_buf[16];
    uint8_t write_len;
    uint8_t read_pos;
    bool reading_payload;
    bool active;
    bool streaming;
    bool packed_protocol;
    bool kindle_protocol;
    bool kindle_v2_protocol;
    bool kindle_ti_protocol;
    bool input_events;
    bool reset_level;
    uint16_t input_x;
    uint16_t input_y;
    uint16_t report_x;
    uint16_t report_y;
    bool input_pressed;
    bool report_pressed;
};

static ZForcePacket *zforce_packet(ZForceState *s)
{
    return s->queue_count ? &s->queue[s->queue_head] : NULL;
}

static void zforce_update_irq(ZForceState *s)
{
    /* Both vendor drivers request a falling-edge interrupt. */
    qemu_set_irq(s->irq, s->queue_count ? 0 : 1);
}

static bool zforce_data_is_touch_move(const uint8_t *data, size_t len)
{
    if (len >= 11 && data[0] == ZFORCE_TOUCH_DATA && data[1] == 1) {
        return (data[6] & 0x0f) == 1;
    }
    return len >= 9 && data[0] == ZFORCE_TOUCH_DATA &&
           data[1] == 1 && (data[6] >> 6) == 1;
}

static bool zforce_is_touch_move(const ZForcePacket *packet)
{
    return zforce_data_is_touch_move(packet->data, packet->len);
}

static bool zforce_read_in_progress(const ZForceState *s)
{
    return s->reading_payload || s->read_pos;
}

static void zforce_remove_queued_packet(ZForceState *s, unsigned logical)
{
    unsigned i;

    for (i = logical; i + 1 < s->queue_count; i++) {
        unsigned dst = (s->queue_head + i) % ZFORCE_QUEUE_LEN;
        unsigned src = (s->queue_head + i + 1) % ZFORCE_QUEUE_LEN;

        s->queue[dst] = s->queue[src];
    }
    s->queue_count--;
}

static void zforce_queue_packet(ZForceState *s, const uint8_t *data,
                                size_t len, bool notify)
{
    ZForcePacket *packet;
    unsigned i, tail;
    bool is_move;

    if (!len || len > ZFORCE_PAYLOAD_MAX) {
        return;
    }

    /*
     * The controller scans at most 60 Hz, but a host pointer can deliver
     * motion much faster than that.  Preserve down/up transitions while
     * replacing an unread move with the latest coordinates.  In particular,
     * never replace queue_head while a read is in progress: the Lab126
     * driver fetches the header and payload in separate I2C transactions.
     */
    is_move = zforce_data_is_touch_move(data, len);
    if (is_move && s->queue_count) {
        tail = (s->queue_head + s->queue_count - 1) % ZFORCE_QUEUE_LEN;
        packet = &s->queue[tail];
        if (!(zforce_read_in_progress(s) && tail == s->queue_head) &&
            zforce_is_touch_move(packet)) {
            packet->len = len;
            memcpy(packet->data, data, len);
            trace_zforce_queue_coalesce(data[6] >> 6, s->queue_count,
                                        zforce_read_in_progress(s));
            return;
        }
    }

    if (s->queue_count == ZFORCE_QUEUE_LEN) {
        /*
         * Coalescing keeps normal touch traffic below this limit.  If a
         * pathological stream still fills it, discard an incoming move.
         * For a transition or command response, evict an unread move first
         * so a release cannot be stranded behind stale coordinates.
         */
        unsigned first_unread = zforce_read_in_progress(s) ? 1 : 0;

        if (is_move) {
            trace_zforce_queue_drop(data[0], s->queue_count,
                                    zforce_read_in_progress(s));
            return;
        }
        for (i = first_unread; i < s->queue_count; i++) {
            unsigned index = (s->queue_head + i) % ZFORCE_QUEUE_LEN;

            if (zforce_is_touch_move(&s->queue[index])) {
                trace_zforce_queue_drop(s->queue[index].data[0],
                                        s->queue_count,
                                        zforce_read_in_progress(s));
                zforce_remove_queued_packet(s, i);
                break;
            }
        }
        if (s->queue_count == ZFORCE_QUEUE_LEN) {
            trace_zforce_queue_drop(data[0], s->queue_count,
                                    zforce_read_in_progress(s));
            return;
        }
    }
    tail = (s->queue_head + s->queue_count) % ZFORCE_QUEUE_LEN;
    packet = &s->queue[tail];
    packet->len = len;
    memcpy(packet->data, data, len);
    s->queue_count++;
    trace_zforce_queue(data[0], len, s->queue_count,
                       zforce_read_in_progress(s));
    if (notify) {
        zforce_update_irq(s);
    }
}

static void zforce_queue(ZForceState *s, const uint8_t *data, size_t len)
{
    zforce_queue_packet(s, data, len, true);
}

static void zforce_pop(ZForceState *s)
{
    uint8_t type = s->queue_count ? s->queue[s->queue_head].data[0] : 0xff;

    /*
     * Whitney configures data-ready as a falling-edge interrupt.  If another
     * frame is queued, cycle the line inactive before asserting it again so
     * the GPIO controller latches a fresh edge while the driver's work item
     * still has the IRQ masked.
     */
    qemu_set_irq(s->irq, 1);
    if (s->queue_count) {
        s->queue_head = (s->queue_head + 1) % ZFORCE_QUEUE_LEN;
        s->queue_count--;
    }
    trace_zforce_pop(type, s->queue_count);
    s->reading_payload = false;
    s->read_pos = 0;
    zforce_update_irq(s);
}

static void zforce_ack(ZForceState *s, uint8_t command)
{
    uint8_t response[] = { command, 0 };

    zforce_queue(s, response, sizeof(response));
}

static unsigned zforce_direct_command_size(uint8_t command)
{
    switch (command) {
    case ZFORCE_DEACTIVATE:
    case ZFORCE_ACTIVATE:
    case ZFORCE_TOUCH_DATA:
    case ZFORCE_VERSION:
    case ZFORCE_LCD_LEVEL:
    case 0x1a: /* force calibration */
        return 1;
    case ZFORCE_SCAN_FREQ:
        return 3;
    case ZFORCE_RESOLUTION:
    case 0x03: /* configuration */
        return 5;
    case 0x0f: /* fixed pulse strength */
        return 2;
    case 0x69: /* reliability response period */
        return 3;
    default:
        return 1;
    }
}

static void zforce_finish_write(ZForceState *s)
{
    uint8_t response[ZFORCE_PAYLOAD_MAX] = { 0 };
    const uint8_t *command_data = s->write_buf;
    unsigned expected;
    uint8_t command;

    if (!s->write_len) {
        return;
    }

    if (s->write_buf[0] == ZFORCE_FRAME_START) {
        if (s->write_len < 2) {
            return;
        }
        expected = s->write_buf[1] + 2;
        if (s->write_len < expected) {
            return;
        }
        if (s->write_buf[1] == 0) {
            s->write_len = 0;
            return;
        }
        command_data = &s->write_buf[2];
        s->packed_protocol = true;
    } else {
        expected = zforce_direct_command_size(s->write_buf[0]);
        if (s->write_len < expected) {
            return;
        }
    }
    command = command_data[0];

    switch (command) {
    case ZFORCE_DEACTIVATE:
        s->active = false;
        s->streaming = false;
        zforce_ack(s, ZFORCE_DEACTIVATE);
        break;
    case ZFORCE_ACTIVATE:
        s->active = true;
        zforce_ack(s, ZFORCE_ACTIVATE);
        break;
    case ZFORCE_RESOLUTION:
    case ZFORCE_SCAN_FREQ:
    case 0x03: /* configuration */
        zforce_ack(s, command);
        break;
    case ZFORCE_TOUCH_DATA:
        s->streaming = true;
        break;
    case ZFORCE_VERSION:
        response[0] = ZFORCE_VERSION;
        /*
         * Whitney's userspace requires the production 2.0b0r12 firmware.
         * Advertising the older Kobo-era version makes ihbslupdater enter
         * the Neonode serial bootloader and removes the touch device.
         */
        response[1] = s->kindle_protocol ? 2 : 1; /* major */
        response[3] = 0; /* minor */
        response[5] = 0; /* build */
        response[7] = s->kindle_protocol ? 12 : 1; /* revision */
        zforce_queue(s, response, 9);
        break;
    case ZFORCE_MCU_STATUS:
        response[0] = ZFORCE_MCU_STATUS;
        if (s->kindle_ti_protocol) {
            /*
             * The Eanab TI driver consumes 128 bytes following the command
             * byte.  Keep this layout separate from the older ST/Kobo
             * status response: their firmware and drivers use different
             * field offsets and a shorter reserved tail.
             */
            response[1] = 1;      /* firmware major, little endian */
            response[9] = 1;      /* one supported contact */
            response[18] = 1;     /* dual-touch configuration enabled */
            response[22] = 45;    /* idle scan frequency */
            response[24] = 60;    /* finger scan frequency */
            response[26] = 60;    /* stylus scan frequency */
            response[28] = ZFORCE_WIDTH & 0xff;
            response[29] = ZFORCE_WIDTH >> 8;
            response[30] = ZFORCE_HEIGHT & 0xff;
            response[31] = ZFORCE_HEIGHT >> 8;
            response[32] = 90;    /* physical width, millimetres */
            response[34] = 120;   /* physical height, millimetres */
            response[36] = 1;     /* first active X LED */
            response[37] = 16;    /* last active X LED */
            response[38] = 1;     /* first active Y LED */
            response[39] = 22;    /* last active Y LED */
            response[46] = 2;     /* interface protocol major */
            response[47] = 0;
            zforce_queue(s, response, 129);
        } else {
            /* Preserve the existing ST/Kobo status response verbatim. */
            response[1] = 1;      /* firmware major, little endian */
            response[7] = s->kindle_v2_protocol ? 8 : 0;
            response[21] = ZFORCE_WIDTH & 0xff;
            response[22] = ZFORCE_WIDTH >> 8;
            response[23] = ZFORCE_HEIGHT & 0xff;
            response[24] = ZFORCE_HEIGHT >> 8;
            response[41] = 2;     /* interface protocol major */
            response[42] = 0;
            zforce_queue(s, response, 124);
        }
        break;
    case ZFORCE_LCD_LEVEL:
        response[0] = ZFORCE_LCD_LEVEL;
        response[1] = 0;
        response[2] = 0;
        zforce_queue(s, response, 3);
        break;
    default:
        response[0] = ZFORCE_INVALID;
        response[1] = s->write_buf[0];
        zforce_queue(s, response, 2);
        break;
    }
    s->write_len = 0;
}

static int zforce_send(I2CSlave *i2c, uint8_t data)
{
    ZForceState *s = ZFORCE(i2c);

    if (s->write_len < sizeof(s->write_buf)) {
        s->write_buf[s->write_len++] = data;
    }
    return 0;
}

static uint8_t zforce_recv(I2CSlave *i2c)
{
    ZForceState *s = ZFORCE(i2c);
    ZForcePacket *packet = zforce_packet(s);

    if (!packet) {
        return 0xff;
    }
    if (!s->reading_payload) {
        if (s->read_pos++ == 0) {
            return ZFORCE_FRAME_START;
        }
        /*
         * Hardware releases data-ready once the host accepts the final
         * queued frame's header.  The payload remains readable, but the
         * guest's IRQ worker will not schedule an unnecessary drain pass.
         */
        if (s->queue_count == 1) {
            qemu_set_irq(s->irq, 1);
        }
        return packet->len;
    }
    if (s->read_pos < packet->len) {
        return packet->data[s->read_pos++];
    }
    return 0;
}

static size_t zforce_build_touch_report(ZForceState *s, uint8_t *response,
                                        bool include_contact, uint8_t state)
{
    uint16_t screen_x, screen_y, native_x, native_y;

    response[0] = ZFORCE_TOUCH_DATA;
    response[1] = include_contact ? 1 : 0;
    if (!include_contact) {
        return 2;
    }

    screen_x = qemu_input_scale_axis(s->input_x, INPUT_EVENT_ABS_MIN,
                                     INPUT_EVENT_ABS_MAX, 0,
                                     ZFORCE_WIDTH - 1);
    screen_y = qemu_input_scale_axis(s->input_y, INPUT_EVENT_ABS_MIN,
                                     INPUT_EVENT_ABS_MAX, 0,
                                     ZFORCE_HEIGHT - 1);
    if (s->kindle_protocol) {
        /*
         * Lab126's Yoshi/Whitney driver uses a seven-byte contact record:
         * little-endian X/Y, state in bits 7:6, ID in bits 5:2, one
         * reserved byte, and probability.  Whitney consumes direct portrait
         * coordinates (only the original Yoshi board flips them).
         */
        /*
         * Whitney's Lab126 driver advertises both axes as 0..4096.  The
         * controller therefore reports native 12-bit coordinates, not EPDC
         * framebuffer pixels.  Sending 600x800 values confines Xorg input
         * to the upper-left corner and makes Cocoa taps appear badly mapped.
         */
        native_x = qemu_input_scale_axis(
            s->input_x, INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
            0, ZFORCE_KINDLE_MAX);
        native_y = qemu_input_scale_axis(
            s->input_y, INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
            0, ZFORCE_KINDLE_MAX);
        response[6] = (state << 6) | (1 << 2);
        response[7] = 0;
        response[8] = state == 2 ? 0 : 100;
    } else if (s->packed_protocol || s->kindle_ti_protocol) {
        /*
         * The captured Kobo kernel's framed protocol reports direct
         * portrait coordinates in a packed state/ID record.
         */
        native_x = screen_x;
        native_y = screen_y;
        response[6] = (1 << 4) | state;
        response[7] = 1;
        response[8] = 1;
        response[9] = state == 2 ? 0 : 100;
        response[10] = 100;
    } else {
        /*
         * Kobolabs neonode.c exposes ABS_X=native Y and
         * ABS_Y=600-native X and consumes the original seven-byte record.
         */
        native_x = ZFORCE_WIDTH -
                   screen_y * ZFORCE_WIDTH / ZFORCE_HEIGHT;
        native_y = screen_x * ZFORCE_HEIGHT / ZFORCE_WIDTH;
        response[6] = state == 2 ? 0x80 : state;
        response[7] = 0;
        response[8] = 100;
    }
    response[2] = native_x;
    response[3] = native_x >> 8;
    response[4] = native_y;
    response[5] = native_y >> 8;

    return (s->packed_protocol || s->kindle_ti_protocol) ? 11 : 9;
}

static int zforce_event(I2CSlave *i2c, enum i2c_event event)
{
    ZForceState *s = ZFORCE(i2c);
    ZForcePacket *packet;
    uint8_t idle_touch[11] = { 0 };
    size_t idle_len;

    switch (event) {
    case I2C_START_SEND:
        break;
    case I2C_START_RECV:
        if (!s->queue_count) {
            /*
             * Kobo's vendor driver keeps draining the controller while its
             * data-ready work item is active.  A held contact must remain
             * present in these scans; returning zero contacts here cancels
             * a normal host tap before its actual button-up arrives.
             */
            idle_len = zforce_build_touch_report(s, idle_touch,
                                                  s->input_pressed,
                                                  s->input_pressed ? 1 : 0);
            zforce_queue_packet(s, idle_touch, idle_len, false);
        }
        s->read_pos = 0;
        break;
    case I2C_FINISH:
        if (s->write_len) {
            zforce_finish_write(s);
        } else {
            packet = zforce_packet(s);
            if (!s->reading_payload && s->read_pos) {
                s->reading_payload = true;
            } else if (s->reading_payload && packet && s->read_pos) {
                /*
                 * The i.MX controller latches the final byte before issuing
                 * STOP, so the slave can observe FINISH before the guest's
                 * last I2DR read.  The payload transaction length came from
                 * our own header and therefore always consumes this packet.
                 */
                zforce_pop(s);
            }
        }
        break;
    default:
        break;
    }
    return 0;
}

static void zforce_queue_boot_complete(ZForceState *s)
{
    static const uint8_t response[] = {
        ZFORCE_BOOT_DONE, 0, 0, 0, 0
    };

    zforce_queue(s, response, sizeof(response));
}

static void zforce_input_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    ZForceState *s = ZFORCE(dev);

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

static void zforce_input_sync(DeviceState *dev)
{
    ZForceState *s = ZFORCE(dev);
    uint8_t response[11] = { 0 };
    uint16_t screen_x, screen_y;
    size_t response_len;

    if (!s->active || !s->streaming) {
        return;
    }
    screen_x = qemu_input_scale_axis(s->input_x, INPUT_EVENT_ABS_MIN,
                                     INPUT_EVENT_ABS_MAX, 0,
                                     ZFORCE_WIDTH);
    screen_y = qemu_input_scale_axis(s->input_y, INPUT_EVENT_ABS_MIN,
                                     INPUT_EVENT_ABS_MAX, 0,
                                     ZFORCE_HEIGHT);
    if (s->input_pressed == s->report_pressed &&
        (!s->input_pressed ||
         (screen_x == s->report_x && screen_y == s->report_y))) {
        return;
    }

    response_len = zforce_build_touch_report(
        s, response, true,
        s->input_pressed ? (s->report_pressed ? 1 : 0) : 2);
    zforce_queue(s, response, response_len);
    s->report_x = screen_x;
    s->report_y = screen_y;
    s->report_pressed = s->input_pressed;
}

static const QemuInputHandler zforce_input_handler = {
    .name = "Neonode zForce",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = zforce_input_event,
    .sync = zforce_input_sync,
};

static void zforce_reset_input(void *opaque, int line, int level)
{
    ZForceState *s = opaque;

    if (level != s->reset_level) {
        s->active = false;
        s->streaming = false;
        s->packed_protocol = false;
        s->queue_head = 0;
        s->queue_count = 0;
        s->reading_payload = false;
        s->read_pos = 0;
        if (s->kindle_ti_protocol) {
            /* Eanab queues boot-complete only when reset is deasserted. */
            if (level) {
                zforce_queue_boot_complete(s);
            } else {
                zforce_update_irq(s);
            }
        } else if (!s->kindle_protocol) {
            zforce_queue_boot_complete(s);
        } else {
            zforce_update_irq(s);
        }
    }
    s->reset_level = level;
}

static void zforce_reset(DeviceState *dev)
{
    ZForceState *s = ZFORCE(dev);

    s->queue_head = 0;
    s->queue_count = 0;
    s->write_len = 0;
    s->read_pos = 0;
    s->reading_payload = false;
    s->active = false;
    s->streaming = false;
    s->packed_protocol = false;
    s->reset_level = false;
    s->input_pressed = false;
    s->report_pressed = false;
    if (!s->kindle_protocol) {
        zforce_queue_boot_complete(s);
    } else {
        zforce_update_irq(s);
    }
}

static void zforce_realize(DeviceState *dev, Error **errp)
{
    ZForceState *s = ZFORCE(dev);

    if (s->input_events) {
        s->input_handler = qemu_input_handler_register(
            dev, &zforce_input_handler);
        qemu_input_handler_activate(s->input_handler);
    }
}

static void zforce_unrealize(DeviceState *dev)
{
    ZForceState *s = ZFORCE(dev);

    if (s->input_handler) {
        qemu_input_handler_unregister(s->input_handler);
    }
}

static void zforce_init(Object *obj)
{
    ZForceState *s = ZFORCE(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in(DEVICE(obj), zforce_reset_input, 1);
}

static void kindle_zforce_init(Object *obj)
{
    ZForceState *s = ZFORCE(obj);

    s->kindle_protocol = true;
}

static void kindle_zforce2_init(Object *obj)
{
    ZForceState *s = ZFORCE(obj);

    /* Bourbon uses the second-generation framed controller protocol. */
    s->kindle_v2_protocol = true;
}

static void kindle_zforce2_ti_init(Object *obj)
{
    ZForceState *s = ZFORCE(obj);

    /* Heisenberg's loader selects the TI protocol and I2C address 0x51. */
    s->kindle_ti_protocol = true;
}

static void zforce_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    static const Property properties[] = {
        DEFINE_PROP_BOOL("input-events", ZForceState, input_events, true),
    };

    dc->realize = zforce_realize;
    dc->unrealize = zforce_unrealize;
    device_class_set_legacy_reset(dc, zforce_reset);
    device_class_set_props(dc, properties);
    sc->send = zforce_send;
    sc->recv = zforce_recv;
    sc->event = zforce_event;
}

/*
 * TPS65185 is a conventional byte-addressed SMBus device.  Kobo's driver
 * requires a recognised revision, writable sequencing/VCOM registers and a
 * completed temperature conversion.  Power-good itself is a board GPIO.
 */
struct TPS65185State {
    I2CSlave parent_obj;
    qemu_irq pwrgood;
    qemu_irq irq;
    uint8_t regs[17];
    uint8_t pointer;
    bool have_pointer;
    bool powerup;
    bool power_enable;
};

static void tps65185_update_pwrgood(TPS65185State *s)
{
    /*
     * Trilogy enables the six rails with ENABLE=0x3f.  Its PWR0 and PWR2
     * board controls gate the PMIC's rail sequencer and therefore PWR_GOOD.
     */
    qemu_set_irq(s->pwrgood,
                 s->powerup && s->power_enable &&
                 (s->regs[1] & 0x3f) == 0x3f);
}

static int tps65185_send(I2CSlave *i2c, uint8_t data)
{
    TPS65185State *s = TPS65185(i2c);
    uint8_t reg;

    if (!s->have_pointer) {
        s->pointer = data;
        s->have_pointer = true;
    } else if (s->pointer < sizeof(s->regs)) {
        reg = s->pointer++;
        s->regs[reg] = data;
        if (reg == 1) {
            tps65185_update_pwrgood(s);
        } else if (reg == 4 && (data & 0x40)) {
            /*
             * Programming VCOM EEPROM completes asynchronously in hardware.
             * The Kobo driver waits for the active-low PRGC interrupt before
             * it reloads and verifies the programmed VCOM value.
             */
            s->regs[4] &= ~0x40;
            s->regs[7] |= 0x01;
            qemu_set_irq(s->irq, 0);
        }
    }
    return 0;
}

static uint8_t tps65185_recv(I2CSlave *i2c)
{
    TPS65185State *s = TPS65185(i2c);
    uint8_t reg;
    uint8_t value;

    if (s->pointer >= sizeof(s->regs)) {
        return 0;
    }

    reg = s->pointer++;
    value = s->regs[reg];
    if (reg == 7 || reg == 8) {
        s->regs[reg] = 0;
        if (reg == 7) {
            qemu_set_irq(s->irq, 1);
        }
    }
    return value;
}

static int tps65185_event(I2CSlave *i2c, enum i2c_event event)
{
    TPS65185State *s = TPS65185(i2c);

    if (event == I2C_START_SEND) {
        s->have_pointer = false;
    }
    return 0;
}

static void tps65185_reset(DeviceState *dev)
{
    TPS65185State *s = TPS65185(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = 25;   /* temperature, degrees C */
    s->regs[13] = 0x20; /* TMST1 conversion complete */
    s->regs[15] = 0xfa; /* all display rails power-good */
    s->regs[16] = 0x65; /* TPS65185 pass 2 */
    s->pointer = 0;
    s->have_pointer = false;
    s->powerup = false;
    s->power_enable = false;
    qemu_set_irq(s->pwrgood, 0);
    qemu_set_irq(s->irq, 1);
}

static void tps65185_powerup(void *opaque, int line, int level)
{
    TPS65185State *s = opaque;

    s->powerup = level;
    tps65185_update_pwrgood(s);
}

static void tps65185_power_enable(void *opaque, int line, int level)
{
    TPS65185State *s = opaque;

    s->power_enable = level;
    tps65185_update_pwrgood(s);
}

static void tps65185_init(Object *obj)
{
    TPS65185State *s = TPS65185(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->pwrgood, "pwrgood", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
    qdev_init_gpio_in_named(DEVICE(obj), tps65185_powerup, "powerup", 1);
    qdev_init_gpio_in_named(DEVICE(obj), tps65185_power_enable,
                            "power-enable", 1);
}

static void tps65185_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, tps65185_reset);
    sc->send = tps65185_send;
    sc->recv = tps65185_recv;
    sc->event = tps65185_event;
}

static const TypeInfo zforce_types[] = {
    {
        .name = TYPE_ZFORCE,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(ZForceState),
        .instance_init = zforce_init,
        .class_init = zforce_class_init,
    }, {
        .name = TYPE_KINDLE_ZFORCE,
        .parent = TYPE_ZFORCE,
        .instance_init = kindle_zforce_init,
    }, {
        .name = TYPE_KINDLE_ZFORCE2,
        .parent = TYPE_ZFORCE,
        .instance_init = kindle_zforce2_init,
    }, {
        .name = TYPE_KINDLE_ZFORCE2_TI,
        .parent = TYPE_ZFORCE,
        .instance_init = kindle_zforce2_ti_init,
    }, {
        .name = TYPE_TPS65185,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(TPS65185State),
        .instance_init = tps65185_init,
        .class_init = tps65185_class_init,
    },
};

DEFINE_TYPES(zforce_types)
