/*
 * Parade TT7010 PIP application transport and idle touch controller.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Protocol: vendor drivers/input/touchscreen/pt5/{pt_core,pt_i2c,pt_regs}.
 * Factory application identity is read from the supplied firmware header;
 * this model does not execute the encrypted firmware or acknowledge flashing.
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

OBJECT_DECLARE_SIMPLE_TYPE(ParadeTT7010State, PARADE_TT7010)
struct ParadeTT7010State {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *timer;
    char *firmware_image;
    uint8_t firmware_header[23];
    uint8_t panel_id;
    uint16_t resolution_x, resolution_y;
    uint8_t tx[512], rx[512], pending[512];
    unsigned tx_len, rx_len, rx_pos, pending_len;
    bool reset_level, scanning, sleeping;
    uint32_t parameters[256];
    uint8_t parameter_size[256];
};

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

static void parade_deliver(void *opaque)
{
    ParadeTT7010State *s = opaque;
    memcpy(s->rx, s->pending, s->pending_len);
    s->rx_len = s->pending_len;
    s->pending_len = 0;
    s->rx_pos = 0;
    qemu_set_irq(s->irq, 0); /* active-low physical INT pin */
}

static void parade_queue(ParadeTT7010State *s, const uint8_t *data, size_t len)
{
    memcpy(s->pending, data, len);
    s->pending_len = len;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 50000);
}

static void parade_descriptor(ParadeTT7010State *s)
{
    /* pt_hid_desc, with the exact 32-byte length checked by pt_core. */
    uint8_t data[32] = { 32, 0, 0xf7, 0, 0, 1 };
    stw_le_p(data + 6, 0); /* PIP parser uses fixed fields, no HID report desc */
    stw_le_p(data + 8, 2);
    stw_le_p(data + 10, 3);
    stw_le_p(data + 12, 254);
    stw_le_p(data + 14, 4);
    stw_le_p(data + 16, 254);
    stw_le_p(data + 18, 5);
    stw_le_p(data + 20, 6);
    stw_le_p(data + 22, 0x04b4);
    stw_le_p(data + 24, 0xc101);
    stw_le_p(data + 26, 0x0100);
    parade_queue(s, data, sizeof(data));
}

static void parade_sysinfo(ParadeTT7010State *s, uint8_t *data)
{
    const uint8_t *h = s->firmware_header;
    data[5] = 1;
    data[6] = 12; /* modeled PIP1 capability level with PIP2 commands */
    stw_le_p(data + 7, lduw_be_p(h + 1));
    data[9] = h[3];
    data[10] = h[4];
    stl_le_p(data + 11, ldl_be_p(h + 9));
    stw_le_p(data + 15, lduw_be_p(h + 17));
    stw_le_p(data + 21, lduw_le_p(h + 13));
    /* Electrode geometry and factory trim are not available in the encrypted
     * image. Keep these unknown; input axes come from the signed board DT. */
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
    uint8_t data[80] = { 5, 0, 0x1f, 0, cmd };
    unsigned len = 5;
    uint8_t id = s->tx_len > 7 ? s->tx[7] : 0;
    switch (cmd) {
    case 0: /* NULL / watchdog */
        break;
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
        /* A real config CRC is not recoverable from the encrypted image.
         * Report verification unavailable; never fabricate integrity success.
         * pt_enum_with_dut_ still registers input after its bounded retries. */
        data[5] = 1;
        len = 10;
        break;
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
        parade_pip2(s);
    } else if (len >= 7 && lduw_le_p(s->tx) == 4 &&
               lduw_le_p(s->tx + 2) + 2 == len && s->tx[4] == 0x2f) {
        parade_pip1(s);
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
            s->rx_len = s->rx_pos = 0;
            qemu_set_irq(s->irq, 1);
        }
        break;
    default:
        break;
    }
    return 0;
}

static void parade_reset_input(void *opaque, int line, int level)
{
    ParadeTT7010State *s = opaque;
    if (level && !s->reset_level) {
        const uint8_t sentinel[2] = { 0, 0 };
        s->scanning = true;
        s->sleeping = false;
        s->rx_len = s->rx_pos = 0;
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
    s->reset_level = true;
    memset(s->parameters, 0, sizeof(s->parameters));
    memset(s->parameter_size, 0, sizeof(s->parameter_size));
    qemu_set_irq(s->irq, 1);
}

static void parade_realize(DeviceState *dev, Error **errp)
{
    ParadeTT7010State *s = PARADE_TT7010(dev);
    g_autofree char *image = NULL;
    gsize len;
    if (!s->firmware_image || !g_file_get_contents(s->firmware_image,
                                                   &image, &len, NULL) ||
        len < sizeof(s->firmware_header) || (uint8_t)image[0] != 22 ||
        lduw_le_p(image + 13) != 0x7010) {
        error_setg(errp, "TT7010 requires an original 7010 touch firmware image");
        return;
    }
    memcpy(s->firmware_header, image, sizeof(s->firmware_header));
}

static const Property parade_properties[] = {
    DEFINE_PROP_STRING("firmware-image", ParadeTT7010State, firmware_image),
    DEFINE_PROP_UINT8("panel-id", ParadeTT7010State, panel_id, 3),
    DEFINE_PROP_UINT16("resolution-x", ParadeTT7010State, resolution_x, 1986),
    DEFINE_PROP_UINT16("resolution-y", ParadeTT7010State, resolution_y, 2648),
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
    timer_free(PARADE_TT7010(obj)->timer);
}

static void parade_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(klass);
    ic->send = parade_send;
    ic->recv = parade_recv;
    ic->event = parade_event;
    dc->realize = parade_realize;
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
static void parade_register(void)
{
    type_register_static(&parade_type);
}
type_init(parade_register);
