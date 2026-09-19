/*
 * MAX77796 companion register banks used by Lab126 Zelda (Kindle Oasis 2).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/max77796.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "system/rtc.h"
#include "system/runstate.h"
#include "ui/input.h"
#include "qemu/timer.h"

#define MAX77796_MAIN_ADDR       0x3c
#define MAX77796_UIC_ADDR        0x35
#define MAX77796_FG_ADDR         0x34
#define GLBLINT                  0x05
#define GLBLINTM                 0x06
#define GLBLSTAT                 0x07
#define GLBLINT_EN0_RISING       (1U << 7)
#define GLBLINT_EN0_FALLING      (1U << 6)
#define GLBLSTAT_EN0_S           (1U << 7)
#define INTTOP1                  0xa2
#define INTTOP1M                 0xa3
#define INTTOP1_TOPSYS           (1U << 7)
#define INTTOP2                  0xa4
#define INTTOP2M                 0xa5
#define INTTOP2_ADC              (1U << 7)
#define INTTOP2_EPD              (1U << 5)
#define UIC_STATUS2              0x04
#define UIC_STATUS2_ADC_MASK     0x1f
#define ADC_CNTL                 0x26
#define ADC_CNTL_CONV            (1U << 4)
#define ADC_CHSEL                0x2a
#define ADC_DATA_L               0x2b
#define ADC_DATA_H               0x2c
#define ADC_CHSEL_MASK           0x1f
#define ADC_CHANNEL_AIN0         9
#define ADC_CHANNEL_AIN2         12
#define ADC_INT                  0x2e
#define ADC_INTM                 0x2f
#define VREG_EPDCNFG             0x60
#define VREG_EPDCNFG_EPDEN       (1U << 7)
#define VREG_EPDINT              0x62
#define VREG_EPDINTM             0x63
#define VREG_EPDOKINTS           0x6a
#define VREG_EPDOKINT            0xb2
#define VREG_EPDOKINTM           0xb3
#define VREG_EPDPOK              (1U << 7)
#define VREG_EPDPDN              (1U << 6)

#define FG_TIMER                 0x3e
#define FG_MLOCK1                0x62
#define FG_MLOCK2                0x63
#define FG_MODEL_FIRST           0x80
#define FG_MODEL_LAST            0xaf

#define MAX77796_RTC_ADDR        0x68
#define RTC_UPDATE0              0x04
#define RTC_UPDATE0_RBUDR        (1U << 4)
#define RTC_UPDATE0_UDR          (1U << 0)
#define RTC_SEC                  0x07
#define RTC_MIN                  0x08
#define RTC_HOUR                 0x09
#define RTC_DOW                  0x0a
#define RTC_MONTH                0x0b
#define RTC_YEAR                 0x0c
#define RTC_DOM                  0x0d
#define RTC_HOUR_PM              (1U << 6)

struct MAX77796State {
    I2CSlave parent_obj;
    qemu_irq irq[2];
    qemu_irq page_button[2];
    uint8_t page_keys[2];
    uint8_t regs[512];
    uint8_t pointer;
    uint8_t byte;
    bool have_pointer;
    uint8_t width;
    int64_t rtc_offset;
    bool power_down;
    QemuInputHandlerState *input_handler;
};

static bool max77796_is_rtc(MAX77796State *s)
{
    return I2C_SLAVE(s)->address == MAX77796_RTC_ADDR;
}

static bool max77796_is_main(MAX77796State *s)
{
    return I2C_SLAVE(s)->address == MAX77796_MAIN_ADDR;
}

static void max77796_update_irq(MAX77796State *s)
{
    bool topsys_pending;
    bool root_pending;

    if (!max77796_is_main(s)) {
        return;
    }

    topsys_pending = s->regs[GLBLINT] & ~s->regs[GLBLINTM];
    if (topsys_pending) {
        s->regs[INTTOP1] |= INTTOP1_TOPSYS;
    } else {
        s->regs[INTTOP1] &= ~INTTOP1_TOPSYS;
    }
    s->regs[INTTOP2] = 0;
    if ((s->regs[VREG_EPDINT] & ~s->regs[VREG_EPDINTM]) ||
        (s->regs[VREG_EPDOKINT] & ~s->regs[VREG_EPDOKINTM])) {
        s->regs[INTTOP2] |= INTTOP2_EPD;
    }
    if (s->regs[ADC_INT] & ~s->regs[ADC_INTM]) {
        s->regs[INTTOP2] |= INTTOP2_ADC;
    }
    root_pending = (s->regs[INTTOP1] & ~s->regs[INTTOP1M]) ||
                   (s->regs[INTTOP2] & ~s->regs[INTTOP2M]);

    /* The physical PMIC interrupt output is active-low. */
    qemu_set_irq(s->irq[0], root_pending ? 0 : 1);
}

static void max77796_input_event(DeviceState *dev, QemuConsole *src,
                                 InputEvent *evt)
{
    MAX77796State *s = MAX77796(dev);
    InputKeyEvent *key = evt->u.key.data;
    int qcode = qemu_input_key_value_to_qcode(key->key);
    unsigned button, key_mask;

    switch (qcode) {
    case Q_KEY_CODE_LEFT:
    case Q_KEY_CODE_PGUP:
    case Q_KEY_CODE_BRACKET_LEFT:
        button = 0;
        key_mask = qcode == Q_KEY_CODE_LEFT ? 1 :
                   qcode == Q_KEY_CODE_PGUP ? 2 : 4;
        break;
    case Q_KEY_CODE_RIGHT:
    case Q_KEY_CODE_PGDN:
    case Q_KEY_CODE_BRACKET_RIGHT:
        button = 1;
        key_mask = qcode == Q_KEY_CODE_RIGHT ? 1 :
                   qcode == Q_KEY_CODE_PGDN ? 2 : 4;
        break;
    default:
        button = ARRAY_SIZE(s->page_button);
        key_mask = 0;
        break;
    }

    if (button < ARRAY_SIZE(s->page_button)) {
        if (key->down) {
            s->page_keys[button] |= key_mask;
        } else {
            s->page_keys[button] &= ~key_mask;
        }
        /* These board buttons share the PMIC's keyboard input handler. */
        qemu_set_irq(s->page_button[button], !s->page_keys[button]);
        return;
    }

    if (qcode != Q_KEY_CODE_POWER || s->power_down == key->down) {
        return;
    }

    s->power_down = key->down;
    if (key->down) {
        s->regs[GLBLSTAT] |= GLBLSTAT_EN0_S;
        s->regs[GLBLINT] |= GLBLINT_EN0_RISING;
    } else {
        s->regs[GLBLSTAT] &= ~GLBLSTAT_EN0_S;
        s->regs[GLBLINT] |= GLBLINT_EN0_FALLING;
    }
    max77796_update_irq(s);

    if (key->down) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
    }
}

static const QemuInputHandler max77796_input_handler = {
    .name = "MAX77796 power and Oasis page buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = max77796_input_event,
};

static uint16_t max77796_adc_sample(unsigned channel)
{
    switch (channel) {
    case 0:
        /* VSYS2: 4.0 V, at 8.192 V full scale. */
        return 2000;
    case 1:
        /* Internal die temperature: 25 degrees C. */
        return 1347;
    case 3:
        /* VSYS1: 4.0 V, at 5.120 V full scale. */
        return 3200;
    case ADC_CHANNEL_AIN0:
        /* Panel NTC lookup value for 25 degrees C. */
        return 782;
    case ADC_CHANNEL_AIN2:
        /* Analog-ground reference used to remove board noise. */
        return 0;
    default:
        return 0;
    }
}

static void max77796_adc_select_channel(MAX77796State *s, uint8_t value)
{
    uint16_t sample = max77796_adc_sample(value & ADC_CHSEL_MASK);

    s->regs[ADC_DATA_H] = sample >> 8;
    s->regs[ADC_DATA_L] = sample;
}

static void max77796_rtc_capture(MAX77796State *s)
{
    struct tm now;
    int hour;

    qemu_get_timedate(&now, s->rtc_offset);
    s->regs[RTC_SEC] = now.tm_sec;
    s->regs[RTC_MIN] = now.tm_min;

    hour = now.tm_hour % 12;
    if (hour == 0) {
        hour = 12;
    }
    s->regs[RTC_HOUR] = hour | (now.tm_hour >= 12 ? RTC_HOUR_PM : 0);
    s->regs[RTC_DOW] = 1U << now.tm_wday;
    s->regs[RTC_MONTH] = now.tm_mon + 1;
    s->regs[RTC_YEAR] = now.tm_year - 100;
    s->regs[RTC_DOM] = now.tm_mday;
}

static void max77796_rtc_commit(MAX77796State *s)
{
    struct tm now;
    int hour = s->regs[RTC_HOUR] & 0x3f;

    qemu_get_timedate(&now, s->rtc_offset);
    hour %= 12;
    if (s->regs[RTC_HOUR] & RTC_HOUR_PM) {
        hour += 12;
    }
    now.tm_sec = s->regs[RTC_SEC] & 0x7f;
    now.tm_min = s->regs[RTC_MIN] & 0x7f;
    now.tm_hour = hour;
    now.tm_mon = (s->regs[RTC_MONTH] & 0x1f) - 1;
    now.tm_year = (s->regs[RTC_YEAR] & 0xff) + 100;
    now.tm_mday = s->regs[RTC_DOM] & 0x3f;
    s->rtc_offset = qemu_timedate_diff(&now);
}

static unsigned max77796_index(MAX77796State *s)
{
    return s->pointer * s->width + s->byte;
}

static uint16_t max77796_fg_get(MAX77796State *s, unsigned reg)
{
    return s->regs[reg * 2] | s->regs[reg * 2 + 1] << 8;
}

static void max77796_fg_set(MAX77796State *s, unsigned reg, uint16_t value)
{
    s->regs[reg * 2] = value;
    s->regs[reg * 2 + 1] = value >> 8;
}

static bool max77796_fg_model_unlocked(MAX77796State *s)
{
    return max77796_fg_get(s, FG_MLOCK1) == 0x0059 &&
           max77796_fg_get(s, FG_MLOCK2) == 0x00c4;
}

static void max77796_advance(MAX77796State *s)
{
    if (++s->byte == s->width) {
        s->byte = 0;
        s->pointer++;
    }
}

static int max77796_send(I2CSlave *i2c, uint8_t data)
{
    MAX77796State *s = MAX77796(i2c);

    if (!s->have_pointer) {
        s->pointer = data;
        s->byte = 0;
        s->have_pointer = true;
    } else {
        unsigned index = max77796_index(s);

        if (max77796_is_rtc(s) && index == RTC_UPDATE0) {
            /* RBUDR and UDR are self-clearing update requests. */
            if (data & RTC_UPDATE0_RBUDR) {
                max77796_rtc_capture(s);
            }
            if (data & RTC_UPDATE0_UDR) {
                max77796_rtc_commit(s);
            }
            data &= ~(RTC_UPDATE0_RBUDR | RTC_UPDATE0_UDR);
        } else if (max77796_is_main(s) && index == ADC_CNTL) {
            /* ADCCONV is cleared by hardware when conversion completes. */
            if (data & ADC_CNTL_CONV) {
                s->regs[ADC_INT] |= 1;
            }
            data &= ~ADC_CNTL_CONV;
        } else if (max77796_is_main(s) && index == ADC_CHSEL) {
            max77796_adc_select_channel(s, data);
        }
        if (max77796_is_main(s) && index == VREG_EPDCNFG) {
            bool was_enabled = s->regs[index] & VREG_EPDCNFG_EPDEN;
            bool enabled = data & VREG_EPDCNFG_EPDEN;

            s->regs[index] = data;
            s->regs[VREG_EPDOKINTS] = enabled ? VREG_EPDPOK : VREG_EPDPDN;
            if (enabled != was_enabled) {
                s->regs[VREG_EPDOKINT] |= s->regs[VREG_EPDOKINTS];
                qemu_set_irq(s->irq[1], enabled);
            }
        } else if (s->width == 2 && s->pointer >= FG_MODEL_FIRST &&
                   s->pointer <= FG_MODEL_LAST &&
                   !max77796_fg_model_unlocked(s)) {
            /* The characterization table is inaccessible while locked. */
        } else {
            s->regs[index] = data;
        }
        if (max77796_is_main(s)) {
            max77796_update_irq(s);
        }
        max77796_advance(s);
    }
    return 0;
}

static uint8_t max77796_recv(I2CSlave *i2c)
{
    MAX77796State *s = MAX77796(i2c);
    unsigned index = max77796_index(s);
    uint8_t value;

    if (s->width == 2 && s->pointer == FG_TIMER && !s->byte) {
        /* The gauge timer has a 175.8 ms LSB. */
        max77796_fg_set(s, FG_TIMER,
                       qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 175800000);
    }
    value = s->regs[index];
    if (s->width == 2 && s->pointer >= FG_MODEL_FIRST &&
        s->pointer <= FG_MODEL_LAST && !max77796_fg_model_unlocked(s)) {
        value = 0;
    }

    /* TOPSYS edge status is latched until software reads GLBLINT. */
    if (max77796_is_main(s) &&
        (index == GLBLINT || index == VREG_EPDINT ||
         index == VREG_EPDOKINT || index == ADC_INT)) {
        s->regs[index] = 0;
        max77796_update_irq(s);
    }

    max77796_advance(s);
    return value;
}

static int max77796_event(I2CSlave *i2c, enum i2c_event event)
{
    MAX77796State *s = MAX77796(i2c);

    if (event == I2C_START_SEND) {
        s->have_pointer = false;
    }
    return 0;
}

static void max77796_realize(DeviceState *dev, Error **errp)
{
    MAX77796State *s = MAX77796(dev);
    I2CSlave *i2c = I2C_SLAVE(dev);

    s->width = i2c->address == MAX77796_FG_ADDR ? 2 : 1;
    if (s->width == 2) {
        static const uint8_t voltages[] = { 0x09, 0x19, 0xfb };
        static const uint8_t percentages[] = { 0x06, 0x07, 0x0d, 0x0e, 0xff };
        static const uint8_t temperatures[] = { 0x08, 0x16, 0x24 };
        static const uint8_t capacities[] = {
            0x05, 0x0f, 0x10, 0x18, 0x1f, 0x23, 0x35,
        };

        /* Charged 1012 mAh Zelda battery, 4.0 V, 100% SOC and 25 C. */
        for (unsigned i = 0; i < ARRAY_SIZE(voltages); i++) {
            max77796_fg_set(s, voltages[i], 0xc800);
        }
        for (unsigned i = 0; i < ARRAY_SIZE(percentages); i++) {
            max77796_fg_set(s, percentages[i], 0x6400);
        }
        for (unsigned i = 0; i < ARRAY_SIZE(temperatures); i++) {
            max77796_fg_set(s, temperatures[i], 0x1900);
        }
        for (unsigned i = 0; i < ARRAY_SIZE(capacities); i++) {
            max77796_fg_set(s, capacities[i], 2024);
        }
        for (unsigned i = FG_MODEL_FIRST; i <= FG_MODEL_LAST; i++) {
            max77796_fg_set(s, i, 0x0100);
        }
    } else if (i2c->address == MAX77796_UIC_ADDR) {
        /*
         * No cable is attached to the virtual USB port.  Zero is an OTG
         * accessory; an open-circuit code keeps host/gadget detection idle.
         */
        s->regs[UIC_STATUS2] = UIC_STATUS2_ADC_MASK;
    } else if (max77796_is_rtc(s)) {
        s->rtc_offset = 0;
        max77796_rtc_capture(s);
    }

    if (max77796_is_main(s)) {
        /* The physical PMIC IRQ pin is active-low. */
        s->regs[GLBLINTM] = 0xff;
        s->regs[INTTOP1M] = 0xff;
        s->regs[INTTOP2M] = 0xff;
        s->regs[ADC_INTM] = 0xff;
        s->regs[VREG_EPDINTM] = 0xff;
        s->regs[VREG_EPDOKINTM] = 0xff;
        s->regs[VREG_EPDOKINTS] = VREG_EPDPDN;
        /* Battery, thermistor and system supply okay; charger unplugged. */
        s->regs[0x0a] = 0x0d;
        s->regs[0x0c] = 0x38;
        s->power_down = false;
        s->input_handler = qemu_input_handler_register(
            dev, &max77796_input_handler);
        max77796_update_irq(s);
        qemu_set_irq(s->irq[1], 0);
        for (unsigned i = 0; i < ARRAY_SIZE(s->page_button); i++) {
            qemu_set_irq(s->page_button[i], 1);
        }
    }
}

static void max77796_reset(DeviceState *dev)
{
    MAX77796State *s = MAX77796(dev);

    if (max77796_is_main(s)) {
        s->regs[GLBLINT] = 0;
        s->regs[GLBLSTAT] &= ~GLBLSTAT_EN0_S;
        s->regs[INTTOP1] &= ~INTTOP1_TOPSYS;
        s->regs[VREG_EPDCNFG] &= ~VREG_EPDCNFG_EPDEN;
        s->regs[VREG_EPDOKINT] = 0;
        s->regs[VREG_EPDOKINTS] = VREG_EPDPDN;
        s->regs[ADC_INT] = 0;
        s->power_down = false;
        memset(s->page_keys, 0, sizeof(s->page_keys));
        max77796_update_irq(s);
        qemu_set_irq(s->irq[1], 0);
        for (unsigned i = 0; i < ARRAY_SIZE(s->page_button); i++) {
            qemu_set_irq(s->page_button[i], 1);
        }
    }
}

static void max77796_unrealize(DeviceState *dev)
{
    MAX77796State *s = MAX77796(dev);

    if (s->input_handler) {
        qemu_input_handler_unregister(s->input_handler);
    }
}

static int max77796_post_load(void *opaque, int version_id)
{
    MAX77796State *s = opaque;

    max77796_update_irq(s);
    if (max77796_is_main(s)) {
        qemu_set_irq(s->irq[1], !!(s->regs[VREG_EPDCNFG] &
                                 VREG_EPDCNFG_EPDEN));
        for (unsigned i = 0; i < ARRAY_SIZE(s->page_button); i++) {
            qemu_set_irq(s->page_button[i], !s->page_keys[i]);
        }
    }
    return 0;
}

static void max77796_init(Object *obj)
{
    MAX77796State *s = MAX77796(obj);

    qdev_init_gpio_out(DEVICE(obj), s->irq, ARRAY_SIZE(s->irq));
    qdev_init_gpio_out_named(DEVICE(obj), s->page_button, "page-button",
                            ARRAY_SIZE(s->page_button));
}

static const VMStateDescription max77796_vmstate = {
    .name = "max77796",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = max77796_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX77796State),
        VMSTATE_UINT8_ARRAY(regs, MAX77796State, 512),
        VMSTATE_UINT8(pointer, MAX77796State),
        VMSTATE_UINT8(byte, MAX77796State),
        VMSTATE_BOOL(have_pointer, MAX77796State),
        VMSTATE_UINT8(width, MAX77796State),
        VMSTATE_INT64(rtc_offset, MAX77796State),
        VMSTATE_BOOL(power_down, MAX77796State),
        VMSTATE_UINT8_ARRAY(page_keys, MAX77796State, 2),
        VMSTATE_END_OF_LIST()
    },
};

static void max77796_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = max77796_realize;
    dc->unrealize = max77796_unrealize;
    dc->vmsd = &max77796_vmstate;
    device_class_set_legacy_reset(dc, max77796_reset);
    sc->send = max77796_send;
    sc->recv = max77796_recv;
    sc->event = max77796_event;
}

static const TypeInfo max77796_info = {
    .name = TYPE_MAX77796,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAX77796State),
    .instance_init = max77796_init,
    .class_init = max77796_class_init,
};

static void max77796_register_types(void)
{
    type_register_static(&max77796_info);
}
type_init(max77796_register_types)
