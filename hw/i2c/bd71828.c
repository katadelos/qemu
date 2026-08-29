/* ROHM BD71828 PMIC register interface used by Bellatrix4. */

#include "qemu/osdep.h"
#include "hw/i2c/bd71828.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/bcd.h"
#include "qemu/module.h"
#include "system/rtc.h"

#define BD71828_REG_BOOTSRC          0x02
#define BD71828_REG_GPIO_CTRL1       0x47
#define BD71828_REG_GPIO_CTRL3       0x49
#define BD71828_REG_RTC_SEC          0x4c
#define BD71828_REG_RTC_MINUTE       0x4d
#define BD71828_REG_RTC_HOUR         0x4e
#define BD71828_REG_RTC_WEEK         0x4f
#define BD71828_REG_RTC_DAY          0x50
#define BD71828_REG_RTC_MONTH        0x51
#define BD71828_REG_RTC_YEAR         0x52
#define BD71828_REG_CHG_STATE        0x65
#define BD71828_REG_BAT_STAT         0x67
#define BD71828_REG_DCIN_STAT        0x68
#define BD71828_REG_BAT_TEMP         0x6c
#define BD71828_REG_VBAT_INITIAL1_U  0x86
#define BD71828_REG_VBAT_INITIAL2_U  0x88
#define BD71828_REG_OCV_PWRON_U      0x8a
#define BD71828_REG_VBAT_U           0x8c
#define BD71828_REG_VBAT_MIN_U       0x8e
#define BD71828_REG_VBAT_REX_U       0x92
#define BD71828_REG_VSYS_U           0x96
#define BD71828_REG_VSYS_MIN_U       0x98
#define BD71828_REG_BATID            0x9e
#define BD71828_REG_BTMP_U           0xa1
#define BD71828_REG_CC_CNT3          0xb5
#define BD71828_REG_CC_FULL3         0xbd
#define BD71828_REG_COULOMB_CTRL2    0xd2

#define BD71828_RTC_24H              BIT(7)
#define BD71828_BAT_DET              BIT(5)
#define BD71828_BAT_DET_DONE         BIT(4)
#define BD71828_FULL_CC_CLR           BIT(4)
#define BD71828_SUNWODA_BATID         0x40
#define BD71828_BTMP_25C              0x0af0

struct BD71828State {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer;
    uint8_t len;
    int64_t rtc_offset;
    qemu_irq gpio_out[3];
};

static void bd71828_update_gpio(BD71828State *s, unsigned gpio)
{
    qemu_set_irq(s->gpio_out[gpio],
                 s->regs[BD71828_REG_GPIO_CTRL1 + gpio] & BIT(0));
}

static void bd71828_store_be16(BD71828State *s, unsigned reg,
                               uint16_t value)
{
    s->regs[reg] = value >> 8;
    s->regs[reg + 1] = value;
}

static void bd71828_store_be32(BD71828State *s, unsigned reg,
                               uint32_t value)
{
    s->regs[reg] = value >> 24;
    s->regs[reg + 1] = value >> 16;
    s->regs[reg + 2] = value >> 8;
    s->regs[reg + 3] = value;
}

static void bd71828_rtc_capture(BD71828State *s)
{
    struct tm now;

    qemu_get_timedate(&now, s->rtc_offset);
    s->regs[BD71828_REG_RTC_SEC] = to_bcd(now.tm_sec);
    s->regs[BD71828_REG_RTC_MINUTE] = to_bcd(now.tm_min);
    s->regs[BD71828_REG_RTC_HOUR] =
        BD71828_RTC_24H | to_bcd(now.tm_hour);
    s->regs[BD71828_REG_RTC_WEEK] = to_bcd(now.tm_wday);
    s->regs[BD71828_REG_RTC_DAY] = to_bcd(now.tm_mday);
    s->regs[BD71828_REG_RTC_MONTH] = to_bcd(now.tm_mon + 1);
    s->regs[BD71828_REG_RTC_YEAR] = to_bcd(now.tm_year % 100);
}

static void bd71828_rtc_commit(BD71828State *s)
{
    struct tm now;

    qemu_get_timedate(&now, s->rtc_offset);
    now.tm_sec = from_bcd(s->regs[BD71828_REG_RTC_SEC] & 0x7f);
    now.tm_min = from_bcd(s->regs[BD71828_REG_RTC_MINUTE] & 0x7f);
    now.tm_hour = from_bcd(s->regs[BD71828_REG_RTC_HOUR] & 0x3f);
    now.tm_wday = from_bcd(s->regs[BD71828_REG_RTC_WEEK] & 0x07);
    now.tm_mday = from_bcd(s->regs[BD71828_REG_RTC_DAY] & 0x3f);
    now.tm_mon = from_bcd(s->regs[BD71828_REG_RTC_MONTH] & 0x1f) - 1;
    now.tm_year = from_bcd(s->regs[BD71828_REG_RTC_YEAR]) + 100;
    s->rtc_offset = qemu_timedate_diff(&now);
}

static int bd71828_send(I2CSlave *i2c, uint8_t data)
{
    BD71828State *s = BD71828(i2c);

    if (!s->len++) {
        s->pointer = data;
        return 0;
    }
    s->regs[s->pointer] = data;
    if (s->pointer == BD71828_REG_COULOMB_CTRL2 &&
        (data & BD71828_FULL_CC_CLR)) {
        memset(&s->regs[BD71828_REG_CC_FULL3], 0, sizeof(uint32_t));
    }
    if (s->pointer >= BD71828_REG_GPIO_CTRL1 &&
        s->pointer <= BD71828_REG_GPIO_CTRL3) {
        bd71828_update_gpio(s, s->pointer - BD71828_REG_GPIO_CTRL1);
    }
    if (s->pointer >= BD71828_REG_RTC_SEC &&
        s->pointer <= BD71828_REG_RTC_YEAR) {
        bd71828_rtc_commit(s);
    }
    s->pointer++;
    return 0;
}

static uint8_t bd71828_recv(I2CSlave *i2c)
{
    BD71828State *s = BD71828(i2c);

    if (s->pointer >= BD71828_REG_RTC_SEC &&
        s->pointer <= BD71828_REG_RTC_YEAR) {
        bd71828_rtc_capture(s);
    }
    return s->regs[s->pointer++];
}

static int bd71828_event(I2CSlave *i2c, enum i2c_event event)
{
    BD71828State *s = BD71828(i2c);

    if (event == I2C_START_SEND) {
        s->len = 0;
    }
    return 0;
}

static void bd71828_reset(DeviceState *dev)
{
    BD71828State *s = BD71828(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->len = 0;
    s->rtc_offset = 0;

    s->regs[BD71828_REG_BOOTSRC] = 1;
    s->regs[BD71828_REG_BAT_STAT] =
        BD71828_BAT_DET | BD71828_BAT_DET_DONE;
    s->regs[BD71828_REG_BAT_TEMP] = 0;
    s->regs[BD71828_REG_CHG_STATE] = 0;
    s->regs[BD71828_REG_DCIN_STAT] = 0;
    bd71828_store_be16(s, BD71828_REG_VBAT_INITIAL1_U, 4000);
    bd71828_store_be16(s, BD71828_REG_VBAT_INITIAL2_U, 4000);
    bd71828_store_be16(s, BD71828_REG_OCV_PWRON_U, 4000);
    bd71828_store_be16(s, BD71828_REG_VBAT_U, 4000);
    bd71828_store_be16(s, BD71828_REG_VBAT_MIN_U, 3950);
    bd71828_store_be16(s, BD71828_REG_VBAT_REX_U, 4000);
    bd71828_store_be16(s, BD71828_REG_VSYS_U, 4000);
    bd71828_store_be16(s, BD71828_REG_VSYS_MIN_U, 3950);
    s->regs[BD71828_REG_BATID] = BD71828_SUNWODA_BATID;
    bd71828_store_be16(s, BD71828_REG_BTMP_U, BD71828_BTMP_25C);
    bd71828_store_be32(s, BD71828_REG_CC_CNT3, 1296U << 16);
    bd71828_store_be32(s, BD71828_REG_CC_FULL3, 0);
    bd71828_rtc_capture(s);
    for (unsigned gpio = 0; gpio < ARRAY_SIZE(s->gpio_out); gpio++) {
        bd71828_update_gpio(s, gpio);
    }
}

static void bd71828_init(Object *obj)
{
    BD71828State *s = BD71828(obj);

    qdev_init_gpio_out_named(DEVICE(obj), s->gpio_out, "gpio",
                             ARRAY_SIZE(s->gpio_out));
}

static const VMStateDescription bd71828_vmstate = {
    .name = TYPE_BD71828,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, BD71828State),
        VMSTATE_UINT8_ARRAY(regs, BD71828State, 256),
        VMSTATE_UINT8(pointer, BD71828State),
        VMSTATE_UINT8(len, BD71828State),
        VMSTATE_INT64(rtc_offset, BD71828State),
        VMSTATE_END_OF_LIST()
    },
};

static void bd71828_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, bd71828_reset);
    dc->vmsd = &bd71828_vmstate;
    sc->send = bd71828_send;
    sc->recv = bd71828_recv;
    sc->event = bd71828_event;
}

static const TypeInfo bd71828_info = {
    .name = TYPE_BD71828,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BD71828State),
    .instance_init = bd71828_init,
    .class_init = bd71828_class_init,
};

static void bd71828_register_types(void)
{
    type_register_static(&bd71828_info);
}
type_init(bd71828_register_types)
