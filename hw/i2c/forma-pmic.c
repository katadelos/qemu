/* Kobo Forma power devices. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/bcd.h"
#include "qemu/cutils.h"
#include "system/rtc.h"
#include "system/system.h"

#define RTC_ALARM 0xa8
#define RTC_CTRL1 0xae
#define RTC_CTRL2 0xaf
#define IRQ_ENABLE 0x9d
#define IRQ_STATUS 0x9e
#define RTC_IRQ BIT(2)
#define RTC_ALARM_ENABLE BIT(6)
#define RTC_ALARM_PENDING BIT(0)

#define TYPE_FORMA_PMIC "forma-pmic"
OBJECT_DECLARE_SIMPLE_TYPE(FormaPMICState, FORMA_PMIC)
struct FormaPMICState {
    I2CSlave parent_obj;
    uint8_t regs[512];
    uint8_t pointer, bank;
    bool expect_pointer;
    int64_t rtc_offset;
    qemu_irq irq;
    QEMUTimer *alarm_timer;
};
static bool ricoh(FormaPMICState *s)
{
    return !strcmp(object_get_typename(OBJECT(s)), "forma-ricoh619");
}
static void forma_pmic_update_irq(FormaPMICState *s)
{
    bool pending = s->regs[RTC_CTRL2] & RTC_ALARM_PENDING;

    s->regs[IRQ_STATUS] = pending ? RTC_IRQ : 0;
    qemu_set_irq(s->irq, !(pending &&
                          (s->regs[RTC_CTRL1] & RTC_ALARM_ENABLE) &&
                          (s->regs[IRQ_ENABLE] & RTC_IRQ)));
}

static void forma_rtc_alarm(void *opaque)
{
    FormaPMICState *s = opaque;

    s->regs[RTC_CTRL2] |= RTC_ALARM_PENDING;
    forma_pmic_update_irq(s);
}

static void forma_rtc_schedule(FormaPMICState *s)
{
    const uint8_t *alarm = &s->regs[RTC_ALARM];
    struct tm now;
    struct tm when = {
        .tm_sec = from_bcd(alarm[0] & 0x7f),
        .tm_min = from_bcd(alarm[1] & 0x7f),
        .tm_hour = from_bcd(alarm[2] & 0x3f),
        .tm_mday = from_bcd(alarm[3] & 0x3f),
        .tm_mon = from_bcd(alarm[4] & 0x1f) - 1,
        .tm_year = from_bcd(alarm[5]) +
                   ((s->regs[0xa5] & 0x80) ? 100 : 0),
    };

    timer_del(s->alarm_timer);
    if (!(s->regs[RTC_CTRL1] & RTC_ALARM_ENABLE) ||
        (s->regs[RTC_CTRL2] & RTC_ALARM_PENDING) ||
        when.tm_mon < 0 || when.tm_mon > 11 || !when.tm_mday) {
        return;
    }
    qemu_get_timedate(&now, s->rtc_offset);
    int64_t delay = mktimegm(&when) - mktimegm(&now);
    if (delay > 0) {
        timer_mod(s->alarm_timer, qemu_clock_get_ms(rtc_clock) + delay * 1000);
    }
}

static void forma_rtc_capture(FormaPMICState *s)
{
    struct tm now;
    qemu_get_timedate(&now, s->rtc_offset);
    s->regs[0xa0] = to_bcd(now.tm_sec);
    s->regs[0xa1] = to_bcd(now.tm_min);
    s->regs[0xa2] = to_bcd(now.tm_hour);
    s->regs[0xa3] = to_bcd(now.tm_wday);
    s->regs[0xa4] = to_bcd(now.tm_mday);
    s->regs[0xa5] = to_bcd(now.tm_mon + 1) | 0x80;
    s->regs[0xa6] = to_bcd(now.tm_year % 100);
}
static int forma_pmic_send(I2CSlave *i2c, uint8_t data)
{
    FormaPMICState *s = FORMA_PMIC(i2c);
    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else if (ricoh(s) && s->pointer == 0xff) {
        s->bank = data & 1;
    } else {
        if (!strcmp(object_get_typename(OBJECT(s)), "forma-tps65185") &&
            s->pointer == 0x0d && (data & 0x80)) {
            data = 0x20; /* Temperature conversion complete. */
        }
        unsigned address = s->bank * 256 + s->pointer++;
        if (ricoh(s) && address == RTC_CTRL2) {
            /* Alarm/periodic status bits clear when written as zero. */
            data = (data & ~0x85) | (s->regs[address] & data & 0x85);
        }
        s->regs[address] = data;
        if (ricoh(s) && !s->bank && s->pointer == 0xa7) {
            struct tm tm = {
                .tm_sec = from_bcd(s->regs[0xa0]),
                .tm_min = from_bcd(s->regs[0xa1]),
                .tm_hour = from_bcd(s->regs[0xa2]),
                .tm_mday = from_bcd(s->regs[0xa4]),
                .tm_mon = from_bcd(s->regs[0xa5] & 0x1f) - 1,
                .tm_year = from_bcd(s->regs[0xa6]) +
                           ((s->regs[0xa5] & 0x80) ? 100 : 0),
            };
            s->rtc_offset = qemu_timedate_diff(&tm);
        }
        if (ricoh(s) && !s->bank) {
            forma_pmic_update_irq(s);
            if (address >= 0xa0 && address <= RTC_CTRL2) {
                forma_rtc_schedule(s);
            }
        }
    }
    return 0;
}
static uint8_t forma_pmic_recv(I2CSlave *i2c)
{
    FormaPMICState *s = FORMA_PMIC(i2c);
    return s->regs[s->bank * 256 + s->pointer++];
}
static int forma_pmic_event(I2CSlave *i2c, enum i2c_event event)
{
    FormaPMICState *s = FORMA_PMIC(i2c);
    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    } else if (event == I2C_START_RECV && ricoh(s) && !s->bank &&
               s->pointer >= 0xa0 && s->pointer <= 0xa6) {
        forma_rtc_capture(s);
    }
    return 0;
}
static void forma_pmic_reset(DeviceState *dev)
{
    FormaPMICState *s = FORMA_PMIC(dev);
    timer_del(s->alarm_timer);
    qemu_set_irq(s->irq, 1);
    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = s->bank = 0;
    s->expect_pointer = true;
    if (ricoh(s)) {
        s->regs[0x36] = 0x24; /* DC1 voltage / presence check */
        s->regs[0x07] = 80;
        s->regs[0x6a] = s->regs[0x70] = 0x0c;
        s->regs[0x6b] = s->regs[0x71] = 0xcc; /* 4.0 V ADC */
        s->regs[0xe0] = 0x11;
        s->regs[0xe1] = 80;
        s->regs[0xe2] = 0x03; s->regs[0xe3] = 0x20;
        s->regs[0xe4] = 0x03; s->regs[0xe5] = 0xe8;
        s->regs[0xeb] = 0x0c; s->regs[0xec] = 0xcc;
        s->regs[0xed] = 0x01; s->regs[0xee] = 0x90;
        s->rtc_offset = 0;
        forma_rtc_capture(s);
    } else if (!strcmp(object_get_typename(OBJECT(s)), "forma-tps65185")) {
        s->regs[0] = 25;
        s->regs[0x0d] = 0x20;
        s->regs[0x0f] = 0xfa;
        s->regs[0x10] = 0x65;
    }
}
static int forma_pmic_post_load(void *opaque, int version)
{
    FormaPMICState *s = opaque;

    if (ricoh(s)) {
        forma_pmic_update_irq(s);
    }
    return 0;
}

static const VMStateDescription forma_pmic_vmstate = {
    .name = TYPE_FORMA_PMIC, .version_id = 2, .minimum_version_id = 2,
    .post_load = forma_pmic_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FormaPMICState),
        VMSTATE_UINT8_ARRAY(regs, FormaPMICState, 512),
        VMSTATE_UINT8(pointer, FormaPMICState),
        VMSTATE_UINT8(bank, FormaPMICState),
        VMSTATE_BOOL(expect_pointer, FormaPMICState),
        VMSTATE_INT64(rtc_offset, FormaPMICState),
        VMSTATE_TIMER_PTR(alarm_timer, FormaPMICState),
        VMSTATE_END_OF_LIST()
    }
};
static void forma_pmic_init(Object *obj)
{
    FormaPMICState *s = FORMA_PMIC(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    s->alarm_timer = timer_new_ms(rtc_clock, forma_rtc_alarm, s);
}

static void forma_pmic_finalize(Object *obj)
{
    timer_free(FORMA_PMIC(obj)->alarm_timer);
}

static void forma_pmic_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    device_class_set_legacy_reset(dc, forma_pmic_reset);
    dc->vmsd = &forma_pmic_vmstate;
    sc->send = forma_pmic_send;
    sc->recv = forma_pmic_recv;
    sc->event = forma_pmic_event;
}
static const TypeInfo forma_pmic_types[] = {
    { .name = TYPE_FORMA_PMIC, .parent = TYPE_I2C_SLAVE,
      .instance_size = sizeof(FormaPMICState), .class_init = forma_pmic_class_init,
      .instance_init = forma_pmic_init,
      .instance_finalize = forma_pmic_finalize, .abstract = true },
    { .name = "forma-ricoh619", .parent = TYPE_FORMA_PMIC },
    { .name = "forma-tps65185", .parent = TYPE_FORMA_PMIC },
};
DEFINE_TYPES(forma_pmic_types)
