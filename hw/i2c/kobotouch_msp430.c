/*
 * Netronix board controller, including the PocketBook 611 E916 firmware.
 *
 * The Kobo Labs driver uses an 8-bit register address and a big-endian
 * 16-bit value. Register 0x90 accepts the system reset command 0xff00.
 */

#include "qemu/osdep.h"
#include "hw/i2c/zforce.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/bswap.h"
#include "qemu/crc-ccitt.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/rtc.h"
#include "system/runstate.h"

#define NTXEC_REG_VERSION       0x00
#define NTXEC_REG_BATTERY_ADC   0x41
#define NTXEC_REG_SYSTEM_FLAGS  0x60
#define NTXEC_REG_RESET         0x90
#define NTXEC_RESET_VALUE       0xff00
#define NTXEC_VERSION_KOBO_MINI 0xb83a
#define NTXEC_VERSION_E916      0xe916
#define NTXEC_BATTERY_4000MV    993

struct KoboTouchMSP430State {
    I2CSlave parent_obj;
    uint16_t regs[256];
    uint8_t pointer;
    uint8_t write_data[32];
    uint8_t reply[32];
    uint8_t reply_len;
    uint8_t write_len;
    uint8_t read_pos;
    bool have_pointer;
    uint16_t version;
    uint32_t rtc_seconds;
    uint32_t alarm_seconds;
    int64_t rtc_base_ns;
};

/* E916 uses little-endian command payloads and CRC-CCITT-FALSE, while
 * retaining the two-byte big-endian identity read of the older MCU. */
static void ntxec_e916_reply(KoboTouchMSP430State *s)
{
    uint16_t crc;

    memset(s->reply, 0, sizeof(s->reply));
    s->reply_len = 0;
    switch (s->pointer) {
    case 0x00:
        stw_be_p(s->reply, NTXEC_VERSION_E916);
        stw_be_p(s->reply + 2, 0x0100);
        s->reply_len = 8;
        break;
    case 0x04: /* RTC time, followed by the alarm when eight bytes are read. */
        stl_le_p(s->reply, s->rtc_seconds +
            (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->rtc_base_ns) /
            NANOSECONDS_PER_SECOND);
        stl_le_p(s->reply + 4, s->alarm_seconds);
        s->reply_len = 8;
        break;
    case 0x07: /* No critical battery or wake-alarm flags. */
        s->reply_len = 1;
        break;
    case 0x08: /* The kernel divides the little-endian ADC result by four. */
        stw_le_p(s->reply, NTXEC_BATTERY_4000MV * 4);
        s->reply_len = 2;
        break;
    case 0x09: /* Factory serial, 22 bytes followed by a little-endian CRC. */
        memset(s->reply, '0', 20);
        s->reply_len = 24;
        break;
    default:
        return;
    }
    if (s->pointer == 0 || s->pointer == 9) {
        crc = crc_ccitt_false(0xffff, s->reply, s->reply_len - 2);
        stw_le_p(s->reply + s->reply_len - 2, crc);
    }
}

static void ntxec_e916_write(KoboTouchMSP430State *s)
{
    uint8_t command[1 + sizeof(s->write_data)];
    uint16_t crc;

    if (s->write_len < 3) {
        return;
    }
    command[0] = s->pointer;
    memcpy(command + 1, s->write_data, s->write_len - 2);
    crc = crc_ccitt_false(0xffff, command, s->write_len - 1);
    if (crc != lduw_le_p(s->write_data + s->write_len - 2)) {
        return;
    }
    switch (s->pointer) {
    case 0x01:
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        break;
    case 0x02:
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        break;
    case 0x05:
        if (s->write_len == 7) {
            s->rtc_seconds = ldl_le_p(s->write_data + 1);
            s->rtc_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        break;
    case 0x06:
        if (s->write_len == 7) {
            s->alarm_seconds = ldl_le_p(s->write_data + 1);
        }
        break;
    default:
        /* Power-keep and watchdog configuration have no response data. */
        break;
    }
}

static int kobotouch_msp430_send(I2CSlave *i2c, uint8_t data)
{
    KoboTouchMSP430State *s = KOBOTOUCH_MSP430(i2c);

    if (!s->have_pointer) {
        s->pointer = data;
        s->have_pointer = true;
    } else if (s->write_len < sizeof(s->write_data)) {
        s->write_data[s->write_len++] = data;
    }
    return 0;
}

static uint8_t kobotouch_msp430_recv(I2CSlave *i2c)
{
    KoboTouchMSP430State *s = KOBOTOUCH_MSP430(i2c);
    uint16_t value = s->regs[s->pointer];

    if (s->version == NTXEC_VERSION_E916) {
        if (s->read_pos < s->reply_len) {
            return s->reply[s->read_pos++];
        }
        return 0;
    }
    if (s->read_pos++ == 0) {
        return value >> 8;
    }
    return value;
}

static int kobotouch_msp430_event(I2CSlave *i2c, enum i2c_event event)
{
    KoboTouchMSP430State *s = KOBOTOUCH_MSP430(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->have_pointer = false;
        s->write_len = 0;
        break;
    case I2C_START_RECV:
        s->read_pos = 0;
        if (s->version == NTXEC_VERSION_E916) {
            ntxec_e916_reply(s);
        }
        break;
    case I2C_FINISH:
        if (s->version == NTXEC_VERSION_E916) {
            ntxec_e916_write(s);
        } else if (s->have_pointer && s->write_len == 2) {
            uint16_t value = (s->write_data[0] << 8) | s->write_data[1];

            s->regs[s->pointer] = value;
            if (s->pointer == NTXEC_REG_RESET && value == NTXEC_RESET_VALUE) {
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            }
        }
        s->write_len = 0;
        break;
    default:
        break;
    }
    return 0;
}

static void kobotouch_msp430_reset(DeviceState *dev)
{
    KoboTouchMSP430State *s = KOBOTOUCH_MSP430(dev);
    struct tm tm;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[NTXEC_REG_VERSION] = s->version;
    s->regs[NTXEC_REG_BATTERY_ADC] = NTXEC_BATTERY_4000MV;
    s->regs[NTXEC_REG_SYSTEM_FLAGS] = 0;
    s->pointer = 0;
    s->write_len = 0;
    s->read_pos = 0;
    s->have_pointer = false;
    memset(s->reply, 0, sizeof(s->reply));
    s->reply_len = 0;
    qemu_get_timedate(&tm, 0);
    s->rtc_seconds = mktimegm(&tm);
    s->rtc_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->alarm_seconds = 0;
}

static int kobotouch_msp430_post_load(void *opaque, int version_id)
{
    KoboTouchMSP430State *s = opaque;

    if (s->write_len > sizeof(s->write_data) ||
        s->reply_len > sizeof(s->reply) ||
        (s->version == NTXEC_VERSION_E916 && s->read_pos > s->reply_len)) {
        return -EINVAL;
    }
    return 0;
}

static const VMStateDescription kobotouch_msp430_vmstate = {
    .name = TYPE_KOBOTOUCH_MSP430,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = kobotouch_msp430_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, KoboTouchMSP430State),
        VMSTATE_UINT16_ARRAY(regs, KoboTouchMSP430State, 256),
        VMSTATE_UINT8(pointer, KoboTouchMSP430State),
        VMSTATE_UINT8_ARRAY(write_data, KoboTouchMSP430State, 32),
        VMSTATE_UINT8_ARRAY(reply, KoboTouchMSP430State, 32),
        VMSTATE_UINT8(reply_len, KoboTouchMSP430State),
        VMSTATE_UINT8(write_len, KoboTouchMSP430State),
        VMSTATE_UINT8(read_pos, KoboTouchMSP430State),
        VMSTATE_BOOL(have_pointer, KoboTouchMSP430State),
        VMSTATE_UINT32(rtc_seconds, KoboTouchMSP430State),
        VMSTATE_UINT32(alarm_seconds, KoboTouchMSP430State),
        VMSTATE_INT64(rtc_base_ns, KoboTouchMSP430State),
        VMSTATE_END_OF_LIST()
    },
};

static void kobotouch_msp430_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    static const Property properties[] = {
        DEFINE_PROP_UINT16("version", KoboTouchMSP430State, version,
                           NTXEC_VERSION_KOBO_MINI),
    };

    device_class_set_props(dc, properties);
    dc->vmsd = &kobotouch_msp430_vmstate;
    device_class_set_legacy_reset(dc, kobotouch_msp430_reset);
    sc->send = kobotouch_msp430_send;
    sc->recv = kobotouch_msp430_recv;
    sc->event = kobotouch_msp430_event;
}

static const TypeInfo kobotouch_msp430_type = {
    .name = TYPE_KOBOTOUCH_MSP430,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(KoboTouchMSP430State),
    .class_init = kobotouch_msp430_class_init,
};

static void kobotouch_msp430_register_types(void)
{
    type_register_static(&kobotouch_msp430_type);
}

type_init(kobotouch_msp430_register_types)
