/* ROHM BD71828 PMIC register interface used by Bellatrix4. */

#include "qemu/osdep.h"
#include "hw/i2c/bd71828.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/bcd.h"
#include "qemu/module.h"
#include "system/rtc.h"
#include "system/runstate.h"

#define BD71828_REG_BOOTSRC          0x02
#define BD71828_REG_PS_CTRL1         0x04
#define BD71828_SHIP_MODE            BIT(0)
#define BD71828_HIBERNATE            BIT(1)
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
#define BD71828_REG_VBAT_U_MT8110    0x84
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
#define BD71828_REG_INT_MASK        0xd3
#define BD71828_REG_INT_MAIN        0xdf
#define BD71828_REG_INT_STATUS      0xe0
#define BD71828_REG_INT_DCIN1       0xe1
#define BD71828_REG_INT_DCIN2       0xe2
#define BD71828_REG_IO_STAT         0xed
#define BD71828_SHORTPUSH           BIT(4)
#define BD71828_PUSH                BIT(5)

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
    qemu_irq nirq;
    bool power_button_support;
    bool power_button;
    bool vbus_present;
    uint64_t power_presses;
    uint64_t power_releases;
    uint64_t power_short_acks;
};

static void bd71828_update_irq(BD71828State *s)
{
    static const uint8_t main_bit[12] = { 7, 6, 6, 5, 4, 3, 2, 2, 2, 2, 1, 0 };
    uint8_t pending = 0;

    if (!s->power_button_support) {
        return;
    }
    /* rohm-bd71828.c uses mask_invert: one enables an interrupt. */
    for (unsigned i = 0; i < 12; i++) {
        if (s->regs[BD71828_REG_INT_STATUS + i] &
            s->regs[BD71828_REG_INT_MASK + i]) {
            pending |= BIT(main_bit[i]);
        }
    }
    s->regs[BD71828_REG_INT_MAIN] = pending;
    qemu_set_irq(s->nirq, !pending);
}

static void bd71828_vbus(void *opaque, int input, int level)
{
    BD71828State *s = opaque;

    if (s->vbus_present != !!level) {
        /* DCIN detection/removal interrupts are latched until acknowledged. */
        s->regs[BD71828_REG_INT_DCIN1] |= level ? BIT(0) : BIT(1);
    }
    s->vbus_present = !!level;
    s->regs[BD71828_REG_DCIN_STAT] = level ? BIT(0) : 0;
    bd71828_update_irq(s);
}

static bool bd71828_get_power_button(Object *obj, Error **errp)
{
    return BD71828(obj)->power_button;
}

static void bd71828_set_power_button(Object *obj, bool value, Error **errp)
{
    BD71828State *s = BD71828(obj);

    if (!s->power_button_support || value == s->power_button) {
        return;
    }
    s->power_button = value;
    if (value) {
        s->power_presses++;
        s->regs[BD71828_REG_INT_DCIN2] |= BD71828_PUSH;
    } else {
        /* This host input represents a short button gesture. Long-press
         * emergency power removal is not synthesized by this interface. */
        s->power_releases++;
        s->regs[BD71828_REG_INT_DCIN2] |= BD71828_SHORTPUSH;
    }
    bd71828_update_irq(s);
}

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
    if (s->power_button_support && s->pointer >= BD71828_REG_INT_STATUS &&
        s->pointer < BD71828_REG_INT_STATUS + 12) {
        if (s->pointer == BD71828_REG_INT_DCIN2 &&
            (s->regs[s->pointer] & data & BD71828_SHORTPUSH)) {
            s->power_short_acks++;
        }
        s->regs[s->pointer] &= ~data;
    } else if (!s->power_button_support ||
               s->pointer != BD71828_REG_INT_MAIN) {
        s->regs[s->pointer] = data;
    }
    bd71828_update_irq(s);
    /* The stock MFD power-off callback selects HBNT (2) or SHPM (1).
     * Both remove the running SoC supply; RESERVED2 is only a reason latch.
     * Enable this rail-control behavior only on the fitted Scribe PMIC. */
    if (s->power_button_support && s->pointer == BD71828_REG_PS_CTRL1 &&
        ((data & 3) == BD71828_SHIP_MODE ||
         (data & 3) == BD71828_HIBERNATE)) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
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
    s->power_button = false;
    s->power_presses = 0;
    s->power_releases = 0;
    s->power_short_acks = 0;
    if (s->power_button_support) {
        s->regs[BD71828_REG_IO_STAT] = 1; /* Hall sensor: cover open. */
    }

    s->regs[BD71828_REG_BOOTSRC] = 1;
    s->regs[BD71828_REG_BAT_STAT] =
        BD71828_BAT_DET | BD71828_BAT_DET_DONE;
    s->regs[BD71828_REG_BAT_TEMP] = 0;
    s->regs[BD71828_REG_CHG_STATE] = 0;
    bd71828_vbus(s, 0, s->vbus_present);
    bd71828_store_be16(s, BD71828_REG_VBAT_U_MT8110, 4000);
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
    bd71828_update_irq(s);
}

static void bd71828_init(Object *obj)
{
    BD71828State *s = BD71828(obj);

    qdev_init_gpio_out_named(DEVICE(obj), s->gpio_out, "gpio",
                             ARRAY_SIZE(s->gpio_out));
    qdev_init_gpio_out_named(DEVICE(obj), &s->nirq, "irq", 1);
    qdev_init_gpio_in_named(DEVICE(obj), bd71828_vbus, "vbus", 1);
    object_property_add_bool(obj, "power-button", bd71828_get_power_button,
                              bd71828_set_power_button);
    object_property_add_uint64_ptr(obj, "power-presses", &s->power_presses,
                                    OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "power-releases", &s->power_releases,
                                    OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "power-short-acks", &s->power_short_acks,
                                    OBJ_PROP_FLAG_READ);
}

static const VMStateDescription bd71828_vmstate = {
    .name = TYPE_BD71828,
    .version_id = 3,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, BD71828State),
        VMSTATE_UINT8_ARRAY(regs, BD71828State, 256),
        VMSTATE_UINT8(pointer, BD71828State),
        VMSTATE_UINT8(len, BD71828State),
        VMSTATE_INT64(rtc_offset, BD71828State),
        VMSTATE_BOOL_V(power_button, BD71828State, 2),
        VMSTATE_UINT64_V(power_presses, BD71828State, 2),
        VMSTATE_UINT64_V(power_releases, BD71828State, 2),
        VMSTATE_UINT64_V(power_short_acks, BD71828State, 2),
        VMSTATE_BOOL_V(vbus_present, BD71828State, 3),
        VMSTATE_END_OF_LIST()
    },
};

static const Property bd71828_properties[] = {
    DEFINE_PROP_BOOL("power-button-support", BD71828State,
                     power_button_support, false),
};

static void bd71828_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, bd71828_reset);
    dc->vmsd = &bd71828_vmstate;
    device_class_set_props(dc, bd71828_properties);
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
