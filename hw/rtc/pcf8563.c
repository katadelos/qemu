/*
 * PCF8563 I2C clock/calendar.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/bcd.h"
#include "qemu/module.h"
#include "system/rtc.h"

#define TYPE_PCF8563 "pcf8563"
OBJECT_DECLARE_SIMPLE_TYPE(PCF8563State, PCF8563)

#define PCF8563_STOP BIT(5)

struct PCF8563State {
    I2CSlave parent_obj;
    int64_t offset;
    uint8_t regs[16];
    uint8_t pointer, weekday_offset;
    bool address_byte, clock_dirty;
};

static void pcf8563_capture(PCF8563State *s)
{
    struct tm now;

    if (s->regs[0] & PCF8563_STOP) {
        return;
    }
    qemu_get_timedate(&now, s->offset);
    s->regs[2] = to_bcd(now.tm_sec);
    s->regs[3] = to_bcd(now.tm_min);
    s->regs[4] = to_bcd(now.tm_hour);
    s->regs[5] = to_bcd(now.tm_mday);
    s->regs[6] = (now.tm_wday + s->weekday_offset) % 7;
    s->regs[7] = to_bcd(now.tm_mon + 1) | (now.tm_year < 100 ? 0x80 : 0);
    s->regs[8] = to_bcd((now.tm_year + 1900) % 100);
}

static void pcf8563_commit(PCF8563State *s)
{
    struct tm now;
    unsigned weekday = s->regs[6] & 7;

    if (!s->clock_dirty || (s->regs[0] & PCF8563_STOP)) {
        return;
    }
    s->clock_dirty = false;
    qemu_get_timedate(&now, s->offset);
    now.tm_sec = from_bcd(s->regs[2] & 0x7f);
    now.tm_min = from_bcd(s->regs[3] & 0x7f);
    now.tm_hour = from_bcd(s->regs[4] & 0x3f);
    now.tm_mday = from_bcd(s->regs[5] & 0x3f);
    now.tm_mon = from_bcd(s->regs[7] & 0x1f) - 1;
    now.tm_year = from_bcd(s->regs[8]) + ((s->regs[7] & 0x80) ? 0 : 100);
    if (now.tm_sec > 59 || now.tm_min > 59 || now.tm_hour > 23 ||
        now.tm_mday < 1 || now.tm_mday > 31 ||
        now.tm_mon < 0 || now.tm_mon > 11 || weekday > 6) {
        return;
    }
    s->offset = qemu_timedate_diff(&now);
    qemu_get_timedate(&now, s->offset);
    s->weekday_offset = (weekday + 7 - now.tm_wday) % 7;
}

static int pcf8563_event(I2CSlave *i2c, enum i2c_event event)
{
    PCF8563State *s = PCF8563(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->address_byte = true;
        break;
    case I2C_START_RECV:
        pcf8563_commit(s);
        pcf8563_capture(s);
        break;
    case I2C_FINISH:
        pcf8563_commit(s);
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t pcf8563_recv(I2CSlave *i2c)
{
    PCF8563State *s = PCF8563(i2c);
    uint8_t value = s->regs[s->pointer];

    s->pointer = (s->pointer + 1) & 15;
    return value;
}

static int pcf8563_send(I2CSlave *i2c, uint8_t data)
{
    PCF8563State *s = PCF8563(i2c);

    if (s->address_byte) {
        s->pointer = data & 15;
        s->address_byte = false;
        return 0;
    }
    if (s->pointer == 0) {
        if (!(s->regs[0] & PCF8563_STOP) && (data & PCF8563_STOP)) {
            pcf8563_capture(s);
        }
        if ((s->regs[0] & PCF8563_STOP) && !(data & PCF8563_STOP)) {
            s->clock_dirty = true;
        }
        s->regs[0] = data & 0xa8;
    } else if (s->pointer >= 2 && s->pointer <= 8) {
        static const uint8_t masks[] = { 0x7f, 0x7f, 0x3f, 0x3f, 7, 0x9f, 0xff };
        s->regs[s->pointer] = data & masks[s->pointer - 2];
        s->clock_dirty = true;
    } else {
        /* Alarm/timer registers are retained; their output pins are unwired. */
        s->regs[s->pointer] = data;
    }
    s->pointer = (s->pointer + 1) & 15;
    return 0;
}

static void pcf8563_reset(DeviceState *dev)
{
    PCF8563State *s = PCF8563(dev);

    s->offset = 0;
    s->pointer = s->weekday_offset = 0;
    s->address_byte = s->clock_dirty = false;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->regs + 9, 0x80, 4); /* Alarm comparisons disabled. */
    s->regs[13] = 0x80;          /* CLKOUT enabled at 32768 Hz. */
    pcf8563_capture(s);
}

static const VMStateDescription pcf8563_vmstate = {
    .name = TYPE_PCF8563,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, PCF8563State),
        VMSTATE_INT64(offset, PCF8563State),
        VMSTATE_UINT8_ARRAY(regs, PCF8563State, 16),
        VMSTATE_UINT8(pointer, PCF8563State),
        VMSTATE_UINT8(weekday_offset, PCF8563State),
        VMSTATE_BOOL(address_byte, PCF8563State),
        VMSTATE_BOOL(clock_dirty, PCF8563State),
        VMSTATE_END_OF_LIST()
    },
};

static void pcf8563_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->desc = "PCF8563 clock/calendar";
    dc->vmsd = &pcf8563_vmstate;
    device_class_set_legacy_reset(dc, pcf8563_reset);
    sc->event = pcf8563_event;
    sc->recv = pcf8563_recv;
    sc->send = pcf8563_send;
}

static const TypeInfo pcf8563_type = {
    .name = TYPE_PCF8563,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(PCF8563State),
    .class_init = pcf8563_class_init,
};

static void pcf8563_register_types(void)
{
    type_register_static(&pcf8563_type);
}
type_init(pcf8563_register_types)
