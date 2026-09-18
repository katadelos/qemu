/* Kobo Forma power devices. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/bcd.h"
#include "system/rtc.h"

#define TYPE_FORMA_PMIC "forma-pmic"
OBJECT_DECLARE_SIMPLE_TYPE(FormaPMICState, FORMA_PMIC)
struct FormaPMICState {
    I2CSlave parent_obj;
    uint8_t regs[512];
    uint8_t pointer, bank;
    bool expect_pointer;
    int64_t rtc_offset;
};
static bool ricoh(FormaPMICState *s)
{
    return !strcmp(object_get_typename(OBJECT(s)), "forma-ricoh619");
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
        s->regs[s->bank * 256 + s->pointer++] = data;
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
static const VMStateDescription forma_pmic_vmstate = {
    .name = TYPE_FORMA_PMIC, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FormaPMICState),
        VMSTATE_UINT8_ARRAY(regs, FormaPMICState, 512),
        VMSTATE_UINT8(pointer, FormaPMICState),
        VMSTATE_UINT8(bank, FormaPMICState),
        VMSTATE_BOOL(expect_pointer, FormaPMICState),
        VMSTATE_INT64(rtc_offset, FormaPMICState),
        VMSTATE_END_OF_LIST()
    }
};
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
      .abstract = true },
    { .name = "forma-ricoh619", .parent = TYPE_FORMA_PMIC },
    { .name = "forma-tps65185", .parent = TYPE_FORMA_PMIC },
};
DEFINE_TYPES(forma_pmic_types)
