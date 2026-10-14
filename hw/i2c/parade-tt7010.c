/*
 * Parade PIP application transport and host touch controller.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Protocol: vendor drivers/input/touchscreen/pt5/{pt_core,pt_i2c,pt_regs}.
 * Factory application identity is read from the supplied firmware header;
 * this model does not execute controller firmware or acknowledge flashing.
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/parade-tt7010.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "ui/input.h"

#define PARADE_TOUCH_QUEUE_SIZE 64
#define PARADE_LEGACY_ROW_SIZE 128

typedef struct ParadeTouch {
    uint16_t x, y;
    uint8_t event;
} ParadeTouch;

OBJECT_DECLARE_SIMPLE_TYPE(ParadeTT7010State, PARADE_TT7010)
struct ParadeTT7010State {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *timer;
    QemuInputHandlerState *input_handler;
    char *firmware_image;
    uint8_t firmware_header[23];
    uint16_t config_crc;
    uint8_t *config_block;
    uint32_t config_length;
    uint16_t physical_x, physical_y;
    bool config_ready;
    uint8_t panel_id;
    uint16_t resolution_x, resolution_y;
    uint8_t tx[512], rx[512], pending[512];
    unsigned tx_len, rx_len, rx_pos, pending_len;
    bool reset_level, scanning, sleeping, bootloader;
    bool pip1_only, invert_x, invert_y, swap_axes;
    bool pressed, reported, input_dirty, rx_is_touch, pending_is_touch;
    int input_x, input_y;
    uint8_t sequence, touch_head, touch_count;
    ParadeTouch touches[PARADE_TOUCH_QUEUE_SIZE];
    uint32_t parameters[256];
    uint8_t parameter_size[256];
};
static void parade_deliver_touch(ParadeTT7010State *s);

static uint16_t pip_crc(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (unsigned b = 0; b < 8; b++) {
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        }
    }
    return crc;
}

static uint32_t parade_silicon_id(ParadeTT7010State *s)
{
    /* Gen5/6's legacy header stores the complete JTAG silicon ID big-endian.
     * The TT7010 header instead has the PIP2 chip ID and revision LE16 fields.
     */
    return s->pip1_only ? ldl_be_p(s->firmware_header + 13) :
                         (uint32_t)lduw_le_p(s->firmware_header + 13) << 16;
}

static bool parade_parse_legacy_config(ParadeTT7010State *s,
                                       const uint8_t *image, size_t len,
                                       Error **errp)
{
    enum { FLASH_SIZE = 65536 };
    g_autofree uint8_t *flash = g_malloc0(FLASH_SIZE);
    g_autofree uint8_t *present = g_malloc0(FLASH_SIZE);
    size_t pos = image[0] + 1;
    unsigned found = 0;
    size_t config_address = 0, config_size = 0;

    /* pt_loader's PIP1 format: array byte, BE16 row, BE16 size, row bytes. */
    while (pos < len) {
        size_t address, size;

        if (len - pos < 5) {
            goto invalid;
        }
        address = lduw_be_p(image + pos + 1) * PARADE_LEGACY_ROW_SIZE;
        size = lduw_be_p(image + pos + 3);
        if (image[pos] || size != PARADE_LEGACY_ROW_SIZE ||
            size > len - pos - 5 ||
            address > FLASH_SIZE - size || present[address]) {
            goto invalid;
        }
        memcpy(flash + address, image + pos + 5, size);
        memset(present + address, 1, size);
        pos += 5 + size;
    }

    /* Recover the original touch-parameter block through its documented
     * size/max-size, axes and config version fields, then verify its CRC.
     * Require one complete matching table rather than inventing a checksum.
     */
    for (size_t address = 0; address + 12 <= FLASH_SIZE; address += 2) {
        size_t size = lduw_le_p(flash + address);
        uint16_t stored;

        if (size < 14 || size != lduw_le_p(flash + address + 2) ||
            size > FLASH_SIZE - address - 2 ||
            lduw_le_p(flash + address + 4) != s->resolution_x ||
            lduw_le_p(flash + address + 6) != s->resolution_y ||
            lduw_le_p(flash + address + 8) !=
            lduw_be_p(s->firmware_header + 17)) {
            continue;
        }
        if (memchr(present + address, 0, size + 2)) {
            continue;
        }
        stored = lduw_le_p(flash + address + size);
        if (stored != pip_crc(flash + address, size)) {
            continue;
        }
        s->config_crc = stored;
        /* Factory X_LEN_PHY/Y_LEN_PHY are in hundredths of a millimeter.
         * The original Thumb code at 0x4bce reads these as EE04+6/+8. */
        s->physical_x = lduw_le_p(flash + address + 10);
        s->physical_y = lduw_le_p(flash + address + 12);
        config_address = address;
        config_size = size;
        found++;
    }
    if (found != 1 || !s->physical_x || !s->physical_y) {
        error_setg(errp, "Parade TT5 image must contain one intact factory touch config");
        return false;
    }
    s->config_length = config_size + 2;
    s->config_block = g_memdup2(flash + config_address, s->config_length);
    s->config_ready = true;
    return true;
invalid:
    error_setg(errp, "Parade TT5 image has an invalid legacy flash row");
    return false;
}

static void parade_deliver(void *opaque)
{
    ParadeTT7010State *s = opaque;
    memcpy(s->rx, s->pending, s->pending_len);
    s->rx_len = s->pending_len;
    s->pending_len = 0;
    s->rx_pos = 0;
    s->rx_is_touch = s->pending_is_touch;
    s->pending_is_touch = false;
    qemu_set_irq(s->irq, 0); /* active-low physical INT pin */
}

static void parade_queue_packet(ParadeTT7010State *s, const uint8_t *data,
                                size_t len, bool touch)
{
    /* Commands can interrupt an unread touch, whose contact stays queued. */
    if (!touch) {
        s->rx_is_touch = false;
    }
    memcpy(s->pending, data, len);
    s->pending_len = len;
    s->pending_is_touch = touch;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 50000);
}

static void parade_queue(ParadeTT7010State *s, const uint8_t *data, size_t len)
{
    parade_queue_packet(s, data, len, false);
}

static void parade_descriptor(ParadeTT7010State *s)
{
    /* pt_hid_desc, with the exact 32-byte length checked by pt_core. */
    uint8_t data[32] = { 32, 0, 0xf7, 0, 0, 1 };
    data[2] = s->bootloader ? 0xff : 0xf7;
    stw_le_p(data + 6, 0); /* PIP parser uses fixed fields, no HID report desc */
    stw_le_p(data + 8, 2);
    stw_le_p(data + 10, 3);
    stw_le_p(data + 12, 254);
    stw_le_p(data + 14, 4);
    stw_le_p(data + 16, 254);
    stw_le_p(data + 18, 5);
    stw_le_p(data + 20, 6);
    stw_le_p(data + 22, 0x04b4);
    stw_le_p(data + 24, s->bootloader ? 0xc100 : 0xc101);
    stw_le_p(data + 26, 0x0100);
    parade_queue(s, data, sizeof(data));
}

static void parade_sysinfo(ParadeTT7010State *s, uint8_t *data)
{
    const uint8_t *h = s->firmware_header;
    data[5] = 1;
    data[6] = s->pip1_only ? 6 : 12;
    stw_le_p(data + 7, lduw_be_p(h + 1));
    data[9] = h[3];
    data[10] = h[4];
    stl_le_p(data + 11, ldl_be_p(h + 9));
    stw_le_p(data + 15, lduw_be_p(h + 17));
    stl_le_p(data + 19, parade_silicon_id(s));
    stw_le_p(data + 32, s->config_ready ? 2 : 0); /* config CRC POST bit */
    /* Electrode geometry and factory trim are not decoded. Input axes come
     * from the signed board DT and physical lengths from the legacy config. */
    stw_le_p(data + 35, s->physical_x);
    stw_le_p(data + 37, s->physical_y);
    stw_le_p(data + 39, s->resolution_x);
    stw_le_p(data + 41, s->resolution_y);
    stw_le_p(data + 43, 255);
    data[47] = s->panel_id;
    data[49] = s->scanning;
    data[50] = 2;
}

static void parade_pip1(ParadeTT7010State *s)
{
    uint8_t cmd = s->tx[6] & 0x7f;
    uint8_t data[12 + PARADE_LEGACY_ROW_SIZE] = { 5, 0, 0x1f, 0, cmd };
    unsigned len = 5;
    uint8_t id = s->tx_len > 7 ? s->tx[7] : 0;
    switch (cmd) {
    case 0: /* NULL / watchdog */
        break;
    case 1: { /* START_BOOTLOADER: reset sentinel */
        const uint8_t sentinel[2] = { 0, 0 };
        s->bootloader = true;
        s->scanning = false;
        parade_queue(s, sentinel, sizeof(sentinel));
        return;
    }
    case 2:
        len = 51;
        parade_sysinfo(s, data);
        break;
    case 3:
        s->scanning = false;
        break;
    case 4:
        s->scanning = true;
        s->sleeping = false;
        break;
    case 5: /* GET_PARAMETER: id, size, little-endian value */
        if (!s->parameter_size[id]) {
            goto unsupported;
        }
        data[5] = id;
        data[6] = s->parameter_size[id];
        stl_le_p(data + 7, s->parameters[id]);
        len = 7 + data[6];
        break;
    case 6: /* SET_PARAMETER */
        if (s->tx_len < 9 || !s->tx[8] || s->tx[8] > 4 ||
            s->tx_len != 9 + s->tx[8]) {
            goto unsupported;
        }
        s->parameter_size[id] = s->tx[8];
        s->parameters[id] = 0;
        for (unsigned i = 0; i < s->tx[8]; i++) {
            s->parameters[id] |= (uint32_t)s->tx[9 + i] << (8 * i);
        }
        data[5] = id;
        data[6] = s->tx[8];
        len = 7;
        break;
    case 0x20:
        if (s->scanning) {
            /* PIP1 uses rejection of this TEST-mode-only command to
             * distinguish an actively scanning application. */
            goto unsupported;
        }
        if (s->config_ready && s->tx_len == 8 && id == 0) {
            stw_le_p(data + 6, s->config_crc);
            stw_le_p(data + 8, s->config_crc);
        } else {
            /* No integrity claim for a block absent from the supplied image. */
            data[5] = 1;
        }
        len = 10;
        break;
    case 0x21: /* GET_CONFIG_ROW_SIZE */
        if (!s->pip1_only || s->scanning || s->tx_len != 7) {
            goto unsupported;
        }
        stw_le_p(data + 5, PARADE_LEGACY_ROW_SIZE);
        len = 7;
        break;
    case 0x22: { /* READ_DATA_BLOCK: row, length, EBID */
        unsigned row, length, ebid, offset, block_length;
        const uint8_t *block = NULL;

        if (!s->pip1_only || s->scanning || s->tx_len != 12) {
            goto unsupported;
        }
        row = lduw_le_p(s->tx + 7);
        length = lduw_le_p(s->tx + 9);
        ebid = s->tx[11];
        offset = row * PARADE_LEGACY_ROW_SIZE;
        data[6] = ebid;
        len = 10;

        /* The TT5 firmware maps EBID0 to its touch config and EBID1 to
         * the 128-byte manufacturing row at flash 0xff00. The supplied
         * update image omits that per-chip factory row; this synthetic
         * controller has erased bytes there, without a factory identity.
         */
        if (ebid == 0 && s->config_ready) {
            block = s->config_block;
            block_length = s->config_length;
        } else if (ebid == 1) {
            block_length = PARADE_LEGACY_ROW_SIZE;
        } else {
            data[5] = 1;
            break;
        }
        /* Firmware accepts at most one row per transfer and rejects
         * requests extending past the block, including its stored CRC.
         * A zero-length request returns the remaining block length.
         */
        if (length > PARADE_LEGACY_ROW_SIZE ||
            offset > block_length || length > block_length - offset) {
            data[5] = 1;
            break;
        }
        stw_le_p(data + 7, length ? length : block_length - offset);
        if (length) {
            if (block) {
                memcpy(data + 10, block + offset, length);
            } else {
                memset(data + 10, 0xff, length);
            }
            stw_le_p(data + 10 + length, pip_crc(data + 10, length));
            len = 12 + length;
        }
        break;
    }
    default:
unsupported:
        /* PIP1 invalid-command response: command zero plus offending ID. */
        data[4] = 0;
        data[5] = cmd;
        len = 6;
        qemu_log_mask(LOG_UNIMP, "TT7010 unsupported PIP1 command %#x\n", cmd);
        break;
    }
    stw_le_p(data, len);
    parade_queue(s, data, len);
}

static void parade_bootloader(ParadeTT7010State *s)
{
    uint8_t data[24] = { 0, 0, 0x30, 0, 1 };
    uint8_t cmd = s->tx[7];
    unsigned payload = 0, len;

    if (s->tx_len < 13 || s->tx[6] != 1 ||
        s->tx[s->tx_len - 1] != 0x17 ||
        lduw_le_p(s->tx + 8) + 13 != s->tx_len ||
        lduw_le_p(s->tx + s->tx_len - 3) !=
        pip_crc(s->tx + 6, s->tx_len - 9)) {
        data[5] = 1;
    } else if (s->bootloader && cmd == 0x3b) {
        const uint8_t sentinel[2] = { 0, 0 };
        s->bootloader = false;
        s->scanning = true;
        s->sleeping = false;
        parade_queue(s, sentinel, sizeof(sentinel));
        return;
    } else if (s->bootloader && cmd == 0x3e) {
        data[8] = s->panel_id;
        payload = 1;
    } else if (s->bootloader && cmd == 0x38) {
        /* Public chip identity from the image header; unavailable silicon
         * revision and bootloader version remain unknown. */
        stl_le_p(data + 8, parade_silicon_id(s));
        payload = 8;
    } else {
        /* No programming, erase or fabricated integrity success. */
        data[5] = 1;
        qemu_log_mask(LOG_UNIMP, "Parade unsupported bootloader command %#x\n",
                      cmd);
    }
    len = 11 + payload;
    stw_le_p(data, len);
    stw_le_p(data + 6, payload);
    stw_le_p(data + len - 3, pip_crc(data + 4, len - 7));
    data[len - 1] = 0x17;
    parade_queue(s, data, len);
}

static void parade_pip2(ParadeTT7010State *s)
{
    uint8_t cmd = s->tx[5] & 0x7f;
    uint8_t data[64] = { 0, 0, s->tx[4], cmd | 0x80, 0 };
    const uint8_t *h = s->firmware_header;
    unsigned len = 7;
    uint16_t crc;
    if (lduw_be_p(s->tx + s->tx_len - 2) !=
        pip_crc(s->tx + 2, s->tx_len - 4)) {
        data[4] = 0x0d;
        goto done;
    }
    switch (cmd) {
    case 0: /* PING */
        break;
    case 1: /* STATUS: executing application, then system mode */
        data[5] = 1;
        data[6] = s->sleeping ? 2 : s->scanning ? 1 : 3;
        len = 9;
        break;
    case 7: /* VERSION, legacy-size response layout in pt_pip2_version */
        data[5] = 12; data[6] = 1; /* application PIP1.12 over PIP2 transport */
        data[7] = 0; data[8] = 0; /* unknown bootloader version */
        data[9] = h[4]; data[10] = h[3];
        stw_le_p(data + 11, lduw_le_p(h + 13));
        stw_le_p(data + 13, lduw_le_p(h + 15));
        len = 17;
        break;
    default:
        data[4] = 0x11; /* UNKNOWN_COMMAND, including firmware file writes */
        qemu_log_mask(LOG_UNIMP, "TT7010 unsupported PIP2 command %#x\n", cmd);
    }
done:
    stw_le_p(data, len);
    crc = pip_crc(data, len - 2);
    stw_be_p(data + len - 2, crc);
    parade_queue(s, data, len);
}

static void parade_command(ParadeTT7010State *s)
{
    unsigned len = s->tx_len;
    if (!len) {
        return;
    }
    if (len == 2 && lduw_le_p(s->tx) == 1) {
        parade_descriptor(s);
    } else if (len >= 8 && lduw_le_p(s->tx) == 0x0101 &&
               lduw_le_p(s->tx + 2) + 2 == len) {
        if (s->pip1_only) {
            /* The legacy application does not speak the PIP2 transport. */
            qemu_log_mask(LOG_UNIMP, "TT5 unsupported PIP2 command %#x\n",
                          s->tx[5] & 0x7f);
        } else {
            parade_pip2(s);
        }
    } else if (len >= 7 && lduw_le_p(s->tx) == 4 &&
               lduw_le_p(s->tx + 2) + 2 == len && s->tx[4] == 0x2f) {
        parade_pip1(s);
    } else if (len >= 8 && lduw_le_p(s->tx) == 4 &&
               s->tx[4] == 0x40) {
        parade_bootloader(s);
    } else if (len >= 4 && lduw_le_p(s->tx) == 5 && s->tx[3] == 1) {
        const uint8_t sentinel[2] = { 0, 0 };
        s->bootloader = false;
        s->scanning = true;
        s->sleeping = false;
        parade_queue(s, sentinel, sizeof(sentinel));
    } else if (len >= 4 && lduw_le_p(s->tx) == 5 && s->tx[3] == 8) {
        /* HID SET_POWER, response state at byte3 and opcode at byte4. */
        uint8_t response[5] = { 5, 0, 0xf0, s->tx[2] & 3, 8 };
        s->sleeping = (s->tx[2] & 3) != 0;
        parade_queue(s, response, sizeof(response));
    } else {
        qemu_log_mask(LOG_UNIMP, "TT7010 unknown transfer reg=%#x len=%u\n",
                      len > 1 ? lduw_le_p(s->tx) : s->tx[0], len);
    }
    s->tx_len = 0;
}

static int parade_send(I2CSlave *slave, uint8_t data)
{
    ParadeTT7010State *s = PARADE_TT7010(slave);
    if (s->tx_len == sizeof(s->tx)) {
        return -1;
    }
    s->tx[s->tx_len++] = data;
    return 0;
}

static uint8_t parade_recv(I2CSlave *slave)
{
    ParadeTT7010State *s = PARADE_TT7010(slave);
    if (!s->rx_len) {
        return s->rx_pos++ == 1 ? 0xff : 0;
    }
    return s->rx_pos < s->rx_len ? s->rx[s->rx_pos++] : 0;
}

static int parade_event(I2CSlave *slave, enum i2c_event event)
{
    ParadeTT7010State *s = PARADE_TT7010(slave);
    switch (event) {
    case I2C_START_SEND:
        s->tx_len = 0;
        break;
    case I2C_START_RECV:
        parade_command(s);
        s->rx_pos = 0;
        break;
    case I2C_FINISH:
        parade_command(s);
        /* Reading two bytes peeks the length; the second transaction starts
         * at byte zero. Consume only the complete packet/reset sentinel. */
        if (s->rx_len && s->rx_pos >= s->rx_len) {
            if (s->rx_is_touch) {
                s->reported = s->touches[s->touch_head].event != 3;
                s->touch_head = (s->touch_head + 1) % PARADE_TOUCH_QUEUE_SIZE;
                s->touch_count--;
            }
            s->rx_len = s->rx_pos = 0;
            s->rx_is_touch = false;
            qemu_set_irq(s->irq, 1);
            parade_deliver_touch(s);
        }
        break;
    default:
        break;
    }
    return 0;
}

static void parade_input_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *event)
{
    ParadeTT7010State *s = PARADE_TT7010(dev);

    if (event->type == INPUT_EVENT_KIND_ABS) {
        InputMoveEvent *move = event->u.abs.data;
        if (move->axis == INPUT_AXIS_X) {
            s->input_dirty |= s->input_x != move->value;
            s->input_x = move->value;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->input_dirty |= s->input_y != move->value;
            s->input_y = move->value;
        }
    } else if (event->type == INPUT_EVENT_KIND_BTN &&
               event->u.btn.data->button == INPUT_BUTTON_LEFT) {
        s->input_dirty |= s->pressed != event->u.btn.data->down;
        s->pressed = event->u.btn.data->down;
    }
}

static void parade_input_sync(DeviceState *dev)
{
    ParadeTT7010State *s = PARADE_TT7010(dev);
    ParadeTouch touch;
    bool previous = s->reported;
    unsigned tail = 0;

    if (!s->input_dirty || !s->scanning || s->sleeping) {
        return;
    }
    s->input_dirty = false;
    if (s->touch_count) {
        tail = (s->touch_head + s->touch_count - 1) % PARADE_TOUCH_QUEUE_SIZE;
        previous = s->touches[tail].event != 3;
    }
    if (!s->pressed && !previous) {
        return;
    }
    touch.x = qemu_input_scale_axis(s->swap_axes ? s->input_y : s->input_x,
                                   INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
                                   0, s->resolution_x - 1);
    touch.y = qemu_input_scale_axis(s->swap_axes ? s->input_x : s->input_y,
                                   INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
                                   0, s->resolution_y - 1);
    touch.event = s->pressed ? (previous ? 2 : 1) : 3;
    if (touch.event == 2 && s->touch_count > 1 &&
        s->touches[tail].event == 2) {
        s->touches[tail] = touch;
    } else {
        if (s->touch_count == PARADE_TOUCH_QUEUE_SIZE) {
            /* Keep the active contact report and reconcile current input. */
            s->touch_count = 1;
            previous = s->touches[s->touch_head].event != 3;
            if (!s->pressed && !previous) {
                return;
            }
            touch.event = s->pressed ? (previous ? 2 : 1) : 3;
        }
        tail = (s->touch_head + s->touch_count) % PARADE_TOUCH_QUEUE_SIZE;
        s->touches[tail] = touch;
        s->touch_count++;
    }
    parade_deliver_touch(s);
}

static void parade_deliver_touch(ParadeTT7010State *s)
{
    uint8_t data[17] = { 0, 0, 1 };
    ParadeTouch *touch;
    unsigned len;

    if (!s->scanning || s->sleeping || s->rx_len || s->pending_len ||
        !s->touch_count) {
        return;
    }
    touch = &s->touches[s->touch_head];
    len = touch->event == 3 ? 7 : sizeof(data);
    stw_le_p(data, len);
    stw_le_p(data + 3, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
    data[5] = touch->event != 3;
    data[6] = (s->sequence++ & 3) << 6;
    if (touch->event != 3) {
        /* pt_core's fixed record: object, ID/event, X, Y, pressure,
         * major, minor, orientation. An empty frame releases the slot. */
        data[8] = touch->event << 5;
        stw_le_p(data + 9, s->invert_x ?
                 s->resolution_x - 1 - touch->x : touch->x);
        stw_le_p(data + 11, s->invert_y ?
                 s->resolution_y - 1 - touch->y : touch->y);
        data[13] = 64;
        data[14] = data[15] = 2;
    }
    parade_queue_packet(s, data, len, true);
}

static const QemuInputHandler parade_input = {
    .name = "Parade PIP touchscreen",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = parade_input_event,
    .sync = parade_input_sync,
};

static void parade_reset_input(void *opaque, int line, int level)
{
    ParadeTT7010State *s = opaque;
    if (level && !s->reset_level) {
        const uint8_t sentinel[2] = { 0, 0 };
        s->scanning = true;
        s->sleeping = false;
        s->bootloader = false;
        s->rx_len = s->rx_pos = 0;
        s->touch_head = s->touch_count = 0;
        s->pressed = s->reported = s->input_dirty = false;
        s->rx_is_touch = s->pending_is_touch = false;
        qemu_set_irq(s->irq, 1);
        parade_queue(s, sentinel, sizeof(sentinel));
    }
    s->reset_level = level;
}

static void parade_reset(DeviceState *dev)
{
    ParadeTT7010State *s = PARADE_TT7010(dev);
    timer_del(s->timer);
    s->tx_len = s->rx_len = s->rx_pos = s->pending_len = 0;
    s->scanning = true;
    s->sleeping = false;
    s->bootloader = false;
    s->reset_level = true;
    s->touch_head = s->touch_count = 0;
    s->pressed = s->reported = s->input_dirty = false;
    s->rx_is_touch = s->pending_is_touch = false;
    s->input_x = s->input_y = 0;
    s->sequence = 0;
    memset(s->parameters, 0, sizeof(s->parameters));
    memset(s->parameter_size, 0, sizeof(s->parameter_size));
    qemu_set_irq(s->irq, 1);
}

static void parade_realize(DeviceState *dev, Error **errp)
{
    ParadeTT7010State *s = PARADE_TT7010(dev);
    g_autofree char *image = NULL;
    gsize len;
    if (!s->resolution_x || !s->resolution_y) {
        error_setg(errp, "Parade touch axes must have nonzero resolution");
        return;
    }
    if (!s->firmware_image || !g_file_get_contents(s->firmware_image,
                                                   &image, &len, NULL) ||
        len < sizeof(s->firmware_header) ||
        (s->pip1_only ? (uint8_t)image[0] != 18 :
         (uint8_t)image[0] != 22 || lduw_le_p(image + 13) != 0x7010)) {
        error_setg(errp, "Parade controller requires its original touch firmware image");
        return;
    }
    memcpy(s->firmware_header, image, sizeof(s->firmware_header));
    if (s->pip1_only &&
        !parade_parse_legacy_config(s, (uint8_t *)image, len, errp)) {
        return;
    }
    if (s->pip1_only) {
        s->input_handler = qemu_input_handler_register(dev, &parade_input);
    }
}

static const Property parade_properties[] = {
    DEFINE_PROP_STRING("firmware-image", ParadeTT7010State, firmware_image),
    DEFINE_PROP_UINT8("panel-id", ParadeTT7010State, panel_id, 3),
    DEFINE_PROP_UINT16("resolution-x", ParadeTT7010State, resolution_x, 1986),
    DEFINE_PROP_UINT16("resolution-y", ParadeTT7010State, resolution_y, 2648),
    DEFINE_PROP_BOOL("invert-x", ParadeTT7010State, invert_x, false),
    DEFINE_PROP_BOOL("invert-y", ParadeTT7010State, invert_y, false),
    DEFINE_PROP_BOOL("swap-axes", ParadeTT7010State, swap_axes, false),
};

static void parade_init(Object *obj)
{
    ParadeTT7010State *s = PARADE_TT7010(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in_named(DEVICE(obj), parade_reset_input, "reset", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, parade_deliver, s);
}

static void parade_finalize(Object *obj)
{
    ParadeTT7010State *s = PARADE_TT7010(obj);
    if (s->input_handler) {
        qemu_input_handler_unregister(s->input_handler);
    }
    g_free(s->config_block);
    timer_free(s->timer);
}

static int parade_post_load(void *opaque, int version_id)
{
    ParadeTT7010State *s = opaque;

    if (s->tx_len > sizeof(s->tx) || s->rx_len > sizeof(s->rx) ||
        s->rx_pos > sizeof(s->rx) || s->pending_len > sizeof(s->pending) ||
        s->touch_head >= PARADE_TOUCH_QUEUE_SIZE ||
        s->touch_count > PARADE_TOUCH_QUEUE_SIZE ||
        ((s->rx_is_touch || s->pending_is_touch) && !s->touch_count)) {
        return -EINVAL;
    }
    qemu_set_irq(s->irq, s->rx_len ? 0 : 1);
    return 0;
}

static const VMStateDescription parade_touch_vmstate = {
    .name = "parade/touch",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(x, ParadeTouch),
        VMSTATE_UINT16(y, ParadeTouch),
        VMSTATE_UINT8(event, ParadeTouch),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription parade_vmstate = {
    .name = TYPE_PARADE_TT7010,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = parade_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, ParadeTT7010State),
        VMSTATE_UINT8_ARRAY(tx, ParadeTT7010State, 512),
        VMSTATE_UINT8_ARRAY(rx, ParadeTT7010State, 512),
        VMSTATE_UINT8_ARRAY(pending, ParadeTT7010State, 512),
        VMSTATE_UINT32(tx_len, ParadeTT7010State),
        VMSTATE_UINT32(rx_len, ParadeTT7010State),
        VMSTATE_UINT32(rx_pos, ParadeTT7010State),
        VMSTATE_UINT32(pending_len, ParadeTT7010State),
        VMSTATE_BOOL(reset_level, ParadeTT7010State),
        VMSTATE_BOOL(scanning, ParadeTT7010State),
        VMSTATE_BOOL(sleeping, ParadeTT7010State),
        VMSTATE_BOOL(bootloader, ParadeTT7010State),
        VMSTATE_BOOL(pressed, ParadeTT7010State),
        VMSTATE_BOOL(reported, ParadeTT7010State),
        VMSTATE_BOOL(input_dirty, ParadeTT7010State),
        VMSTATE_BOOL(rx_is_touch, ParadeTT7010State),
        VMSTATE_BOOL(pending_is_touch, ParadeTT7010State),
        VMSTATE_INT32(input_x, ParadeTT7010State),
        VMSTATE_INT32(input_y, ParadeTT7010State),
        VMSTATE_UINT8(sequence, ParadeTT7010State),
        VMSTATE_UINT8(touch_head, ParadeTT7010State),
        VMSTATE_UINT8(touch_count, ParadeTT7010State),
        VMSTATE_STRUCT_ARRAY(touches, ParadeTT7010State, PARADE_TOUCH_QUEUE_SIZE,
                             0, parade_touch_vmstate, ParadeTouch),
        VMSTATE_UINT32_ARRAY(parameters, ParadeTT7010State, 256),
        VMSTATE_UINT8_ARRAY(parameter_size, ParadeTT7010State, 256),
        VMSTATE_TIMER_PTR(timer, ParadeTT7010State),
        VMSTATE_END_OF_LIST()
    },
};

static void parade_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(klass);
    ic->send = parade_send;
    ic->recv = parade_recv;
    ic->event = parade_event;
    dc->realize = parade_realize;
    dc->vmsd = &parade_vmstate;
    device_class_set_legacy_reset(dc, parade_reset);
    device_class_set_props(dc, parade_properties);
}

static const TypeInfo parade_type = {
    .name = TYPE_PARADE_TT7010,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ParadeTT7010State),
    .instance_init = parade_init,
    .instance_finalize = parade_finalize,
    .class_init = parade_class_init,
};

static void parade_tt5_init(Object *obj)
{
    PARADE_TT7010(obj)->pip1_only = true;
}

static const TypeInfo parade_tt5_type = {
    .name = TYPE_PARADE_TT5,
    .parent = TYPE_PARADE_TT7010,
    .instance_init = parade_tt5_init,
};

static void parade_register(void)
{
    type_register_static(&parade_type);
    type_register_static(&parade_tt5_type);
}
type_init(parade_register);
