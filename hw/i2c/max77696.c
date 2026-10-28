/* MAX77696 companion register banks used by Lab126 Wario. */

#include "qemu/osdep.h"
#include "hw/i2c/max77696.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/bcd.h"
#include "qemu/timer.h"
#include "system/rtc.h"
#include "system/system.h"
#include "system/runstate.h"
#include "ui/input.h"
#include "trace.h"

#define MAX77696_MAIN_ADDR       0x3c
#define MAX77696_UIC_ADDR        0x35
#define GLBLINT                  0x05
#define GLBLINTM                 0x06
#define GLBLSTAT                 0x07
#define GLBLINT_EN0_RISING       (1U << 7)
#define GLBLINT_EN0_FALLING      (1U << 6)
#define GLBLSTAT_EN0_S           (1U << 7)
#define CHG_INT                  0x08
#define CHG_INT_MASK             0x09
#define CHG_INT_OK               0x0a
#define CHG_DTLS_00              0x0b
#define CHG_CHGINA               (1U << 6)
#define CHG_DTLS_CHGINA_MASK     (3U << 5)
#define INTTOP1                  0xa2
#define INTTOP1M                 0xa3
#define INTTOP1_TOPSYS           (1U << 7)
#define INTTOP1_RTC              (1U << 3)
#define INTTOP1_CHGA             (1U << 2)
#define INTTOP1_UIC              (1U << 0)
#define UIC_INT1                 0x01
#define UIC_INT2                 0x02
#define UIC_INT1_CHGTYP          (1U << 0)
#define UIC_INT1_VBVOLT          (1U << 1)
#define UIC_STATUS1              0x03
#define UIC_STATUS1_CHGTYP_MASK   0x0f
#define UIC_STATUS1_CHGTYP_USB    1
#define UIC_STATUS1_VBVOLT       (1U << 4)
#define UIC_STATUS2              0x04
#define UIC_STATUS2_ADC_MASK     0x1f
#define UIC_INTMASK1             0x05
#define UIC_INTMASK2             0x06
#define UIC_SYSCTRL1             0x07
#define UIC_SYSCTRL1_INT_EN      (1U << 4)
#define ADC_CNTL                 0x26
#define ADC_CNTL_CONV            (1U << 4)
#define ADC_CHSEL                0x2a
#define ADC_DATA_L               0x2b
#define ADC_DATA_H               0x2c
#define ADC_CHSEL_MASK           0x1f
#define ADC_CHANNEL_AIN0         9
#define ADC_CHANNEL_AIN2         12
#define VREG_EPDCNFG             0x60
#define VREG_EPDCNFG_EPDEN       (1U << 7)

#define MAX77696_RTC_ADDR        0x68
#define RTC_INT                  0x00
#define RTC_INTM                 0x01
#define RTC_CONTROL              0x03
#define RTC_CONTROL_BCD          BIT(0)
#define RTC_CONTROL_24H          BIT(1)
#define RTC_INT_ALARM1           BIT(1)
#define RTC_INT_ALARM2           BIT(2)
#define RTC_INT_1S               BIT(4)
#define RTC_INT_60S              BIT(0)
#define RTC_AE1                  0x0e
#define RTC_ALARM1               0x0f
#define RTC_AE2                  0x16
#define RTC_ALARM2               0x17
#define RTC_FIELDS               7
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

struct MAX77696State {
    I2CSlave parent_obj;
    qemu_irq irq[2];
    qemu_irq rtc_irq;
    QEMUTimer *rtc_timer;
    uint8_t regs[512];
    uint8_t pointer;
    uint8_t byte;
    uint8_t len;
    uint8_t width;
    int64_t rtc_offset;
    int64_t rtc_last_tick;
    uint8_t rtc_alarm[RTC_FIELDS * 2];
    bool rtc_time_dirty;
    bool rtc_pending;
    bool rtc_alarm_match[2];
    bool power_down;
    QemuInputHandlerState *input_handler;
};

static bool max77696_is_rtc(MAX77696State *s)
{
    return I2C_SLAVE(s)->address == MAX77696_RTC_ADDR;
}

static bool max77696_is_main(MAX77696State *s)
{
    return I2C_SLAVE(s)->address == MAX77696_MAIN_ADDR;
}

static bool max77696_is_uic(MAX77696State *s)
{
    return I2C_SLAVE(s)->address == MAX77696_UIC_ADDR;
}

static void max77696_update_irq(MAX77696State *s)
{
    bool topsys_pending;
    bool root_pending;

    if (max77696_is_uic(s)) {
        /* UIC masks use one to enable, unlike the main PMIC masks. This
         * output supplies the UIC summary to the main interrupt bank. */
        bool pending = (s->regs[UIC_SYSCTRL1] & UIC_SYSCTRL1_INT_EN) &&
            ((s->regs[UIC_INT1] & s->regs[UIC_INTMASK1]) ||
             (s->regs[UIC_INT2] & s->regs[UIC_INTMASK2]));

        qemu_set_irq(s->irq[0], pending);
        return;
    }
    if (max77696_is_rtc(s)) {
        bool pending = s->regs[RTC_INT] & ~s->regs[RTC_INTM];
        qemu_set_irq(s->rtc_irq, pending);
        return;
    } else if (!max77696_is_main(s)) {
        return;
    }

    topsys_pending = s->regs[GLBLINT] & ~s->regs[GLBLINTM];
    if (topsys_pending) {
        s->regs[INTTOP1] |= INTTOP1_TOPSYS;
    } else {
        s->regs[INTTOP1] &= ~INTTOP1_TOPSYS;
    }
    if (s->regs[CHG_INT] & ~s->regs[CHG_INT_MASK]) {
        s->regs[INTTOP1] |= INTTOP1_CHGA;
    } else {
        s->regs[INTTOP1] &= ~INTTOP1_CHGA;
    }
    if (s->rtc_pending) {
        s->regs[INTTOP1] |= INTTOP1_RTC;
    } else {
        s->regs[INTTOP1] &= ~INTTOP1_RTC;
    }
    root_pending = s->regs[INTTOP1] & ~s->regs[INTTOP1M];

    /* The physical PMIC interrupt output is active-low. */
    trace_max77696_irq(root_pending, s->regs[GLBLINT],
                       s->regs[GLBLINTM], s->regs[INTTOP1M]);
    qemu_set_irq(s->irq[0], root_pending ? 0 : 1);
}

static void max77696_uic_irq(void *opaque, int n, int level)
{
    MAX77696State *s = opaque;

    if (level) {
        s->regs[INTTOP1] |= INTTOP1_UIC;
    } else {
        s->regs[INTTOP1] &= ~INTTOP1_UIC;
    }
    max77696_update_irq(s);
}

static void max77696_vbus(void *opaque, int n, int level)
{
    MAX77696State *s = opaque;
    bool connected = level != 0;

    if (max77696_is_main(s)) {
        bool previous = s->regs[CHG_INT_OK] & CHG_CHGINA;

        s->regs[CHG_INT_OK] &= ~CHG_CHGINA;
        s->regs[CHG_DTLS_00] &= ~CHG_DTLS_CHGINA_MASK;
        if (connected) {
            s->regs[CHG_INT_OK] |= CHG_CHGINA;
            s->regs[CHG_DTLS_00] |= CHG_DTLS_CHGINA_MASK;
        }
        if (previous != connected) {
            s->regs[CHG_INT] |= CHG_CHGINA;
        }
    } else if (max77696_is_uic(s)) {
        bool previous = s->regs[UIC_STATUS1] & UIC_STATUS1_VBVOLT;

        s->regs[UIC_STATUS1] &= ~(UIC_STATUS1_VBVOLT |
                                  UIC_STATUS1_CHGTYP_MASK);
        if (connected) {
            s->regs[UIC_STATUS1] |= UIC_STATUS1_VBVOLT |
                                    UIC_STATUS1_CHGTYP_USB;
        }
        if (previous != connected) {
            s->regs[UIC_INT1] |= UIC_INT1_CHGTYP | UIC_INT1_VBVOLT;
        }
    }
    max77696_update_irq(s);
}

static void max77696_power_button(void *opaque, int n, int level)
{
    MAX77696State *s = opaque;
    bool down = level != 0;

    if (s->power_down == down) {
        return;
    }

    s->power_down = down;
    if (down) {
        s->regs[GLBLSTAT] |= GLBLSTAT_EN0_S;
        s->regs[GLBLINT] |= GLBLINT_EN0_RISING;
    } else {
        s->regs[GLBLSTAT] &= ~GLBLSTAT_EN0_S;
        s->regs[GLBLINT] |= GLBLINT_EN0_FALLING;
    }
    trace_max77696_power(down, s->regs[GLBLSTAT],
                         s->regs[GLBLINT], s->regs[GLBLINTM]);
    max77696_update_irq(s);

    if (down) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
    }
}

static void max77696_input_event(DeviceState *dev, QemuConsole *src,
                                 InputEvent *evt)
{
    InputKeyEvent *key = evt->u.key.data;

    if (qemu_input_key_value_to_qcode(key->key) == Q_KEY_CODE_POWER) {
        max77696_power_button(dev, 0, key->down);
    }
}

static const QemuInputHandler max77696_input_handler = {
    .name = "MAX77696 EN0 power button",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = max77696_input_event,
};

static uint16_t max77696_adc_sample(unsigned channel)
{
    switch (channel) {
    case ADC_CHANNEL_AIN0:
        /* Panel NTC lookup value for 25 degrees C. */
        return 0x0295;
    case ADC_CHANNEL_AIN2:
        /* Analog-ground reference used to remove board noise. */
        return 0;
    default:
        return 0;
    }
}

static void max77696_adc_select_channel(MAX77696State *s, uint8_t value)
{
    uint16_t sample = max77696_adc_sample(value & ADC_CHSEL_MASK);

    s->regs[ADC_DATA_H] = sample >> 8;
    s->regs[ADC_DATA_L] = sample;
}

static void max77696_rtc_irq(void *opaque, int n, int level)
{
    MAX77696State *s = opaque;

    s->rtc_pending = level != 0;
    max77696_update_irq(s);
}

static uint8_t max77696_rtc_encode(MAX77696State *s, unsigned value)
{
    return s->regs[RTC_CONTROL] & RTC_CONTROL_BCD ? to_bcd(value) : value;
}

static unsigned max77696_rtc_decode(MAX77696State *s, uint8_t value)
{
    return s->regs[RTC_CONTROL] & RTC_CONTROL_BCD ? from_bcd(value) : value;
}

static void max77696_rtc_format(MAX77696State *s, struct tm *now,
                                uint8_t *regs)
{
    unsigned hour = now->tm_hour;

    if (!(s->regs[RTC_CONTROL] & RTC_CONTROL_24H)) {
        hour = hour % 12 ? hour % 12 : 12;
    }
    regs[0] = max77696_rtc_encode(s, now->tm_sec);
    regs[1] = max77696_rtc_encode(s, now->tm_min);
    regs[2] = max77696_rtc_encode(s, hour);
    if (!(s->regs[RTC_CONTROL] & RTC_CONTROL_24H) && now->tm_hour >= 12) {
        regs[2] |= RTC_HOUR_PM;
    }
    regs[3] = 1U << now->tm_wday;
    regs[4] = max77696_rtc_encode(s, now->tm_mon + 1);
    regs[5] = max77696_rtc_encode(s, now->tm_year - 100);
    regs[6] = max77696_rtc_encode(s, now->tm_mday);
}

static void max77696_rtc_capture(MAX77696State *s)
{
    struct tm now;

    qemu_get_timedate(&now, s->rtc_offset);
    max77696_rtc_format(s, &now, &s->regs[RTC_SEC]);
}

static void max77696_rtc_schedule(MAX77696State *s)
{
    int64_t now = qemu_clock_get_ms(rtc_clock);

    if ((s->regs[RTC_AE1] | s->regs[RTC_AE2]) & 0x7f ||
        (~s->regs[RTC_INTM] & (RTC_INT_1S | RTC_INT_60S))) {
        timer_mod(s->rtc_timer, (now / 1000 + 1) * 1000);
    } else {
        timer_del(s->rtc_timer);
    }
}

static void max77696_rtc_tick(void *opaque)
{
    MAX77696State *s = opaque;
    struct tm now;
    uint8_t current[RTC_FIELDS];
    unsigned alarm, field;
    int64_t clock_now = qemu_clock_get_ms(rtc_clock) / 1000;

    qemu_get_timedate(&now, s->rtc_offset);
    max77696_rtc_format(s, &now, current);
    for (alarm = 0; alarm < 2; alarm++) {
        uint8_t enabled = s->regs[alarm ? RTC_AE2 : RTC_AE1] & 0x7f;
        bool match = enabled != 0;

        for (field = 0; field < RTC_FIELDS; field++) {
            if (enabled & BIT(field)) {
                uint8_t value = s->rtc_alarm[alarm * RTC_FIELDS + field];
                match &= field == 3 ? (current[field] & value) != 0 :
                                      current[field] == value;
            }
        }
        if (enabled == 0x77) {
            const uint8_t *value = &s->rtc_alarm[alarm * RTC_FIELDS];
            struct tm target = now;
            unsigned hour = max77696_rtc_decode(s, value[2] & 0x3f);
            int64_t deadline;

            if (!(s->regs[RTC_CONTROL] & RTC_CONTROL_24H)) {
                hour = hour % 12 + ((value[2] & RTC_HOUR_PM) ? 12 : 0);
            }
            target.tm_sec = max77696_rtc_decode(s, value[0] & 0x7f);
            target.tm_min = max77696_rtc_decode(s, value[1] & 0x7f);
            target.tm_hour = hour;
            target.tm_mon = max77696_rtc_decode(s, value[4] & 0x1f) - 1;
            target.tm_year = max77696_rtc_decode(s, value[5]) + 100;
            target.tm_mday = max77696_rtc_decode(s, value[6] & 0x3f);
            deadline = clock_now + qemu_timedate_diff(&target) - s->rtc_offset;
            /* A delayed host callback must still latch an elapsed alarm. */
            match = deadline > s->rtc_last_tick && deadline <= clock_now;
        }
        if (match && !s->rtc_alarm_match[alarm]) {
            s->regs[RTC_INT] |= alarm ? RTC_INT_ALARM2 : RTC_INT_ALARM1;
            trace_max77696_rtc_alarm(alarm + 1, s->regs[RTC_INT],
                                     s->regs[RTC_INTM]);
            if (s->regs[RTC_INT] & ~s->regs[RTC_INTM]) {
                qemu_system_wakeup_request(QEMU_WAKEUP_REASON_RTC, NULL);
            }
        }
        s->rtc_alarm_match[alarm] = match;
    }
    /* Periodic flags share the RTC bank's clear-on-read interrupt register. */
    s->regs[RTC_INT] |= RTC_INT_1S;
    if (now.tm_sec == 0) {
        s->regs[RTC_INT] |= RTC_INT_60S;
    }
    s->rtc_last_tick = clock_now;
    max77696_update_irq(s);
    max77696_rtc_schedule(s);
}

static void max77696_rtc_commit(MAX77696State *s)
{
    if (s->rtc_time_dirty) {
        struct tm now;
        unsigned hour = max77696_rtc_decode(s, s->regs[RTC_HOUR] & 0x3f);

        qemu_get_timedate(&now, s->rtc_offset);
        if (!(s->regs[RTC_CONTROL] & RTC_CONTROL_24H)) {
            hour %= 12;
            if (s->regs[RTC_HOUR] & RTC_HOUR_PM) {
                hour += 12;
            }
        }
        now.tm_sec = max77696_rtc_decode(s, s->regs[RTC_SEC] & 0x7f);
        now.tm_min = max77696_rtc_decode(s, s->regs[RTC_MIN] & 0x7f);
        now.tm_hour = hour;
        now.tm_mon = max77696_rtc_decode(s, s->regs[RTC_MONTH] & 0x1f) - 1;
        now.tm_year = max77696_rtc_decode(s, s->regs[RTC_YEAR]) + 100;
        now.tm_mday = max77696_rtc_decode(s, s->regs[RTC_DOM] & 0x3f);
        s->rtc_offset = qemu_timedate_diff(&now);
        s->rtc_time_dirty = false;
    }
    /* An alarm-only UDR must not set the clock from its old read buffer. */
    memcpy(s->rtc_alarm, &s->regs[RTC_ALARM1], RTC_FIELDS);
    memcpy(s->rtc_alarm + RTC_FIELDS, &s->regs[RTC_ALARM2], RTC_FIELDS);
    memset(s->rtc_alarm_match, 0, sizeof(s->rtc_alarm_match));
    s->rtc_last_tick = qemu_clock_get_ms(rtc_clock) / 1000;
    max77696_rtc_schedule(s);
}

static unsigned max77696_index(MAX77696State *s)
{
    return s->pointer * s->width + s->byte;
}

static void max77696_advance(MAX77696State *s)
{
    if (++s->byte == s->width) {
        s->byte = 0;
        s->pointer++;
    }
}

static int max77696_send(I2CSlave *i2c, uint8_t data)
{
    MAX77696State *s = MAX77696(i2c);

    if (!s->len++) {
        s->pointer = data;
        s->byte = 0;
    } else {
        unsigned index = max77696_index(s);

        if ((max77696_is_main(s) &&
             (index == CHG_INT || index == CHG_INT_OK || index == CHG_DTLS_00)) ||
            (max77696_is_uic(s) &&
             (index == UIC_INT1 || index == UIC_INT2 ||
              index == UIC_STATUS1 || index == UIC_STATUS2))) {
            max77696_advance(s);
            return 0;
        }
        if (max77696_is_rtc(s) && index == RTC_UPDATE0) {
            /* RBUDR and UDR are self-clearing update requests. */
            if (data & RTC_UPDATE0_RBUDR) {
                max77696_rtc_capture(s);
            }
            if (data & RTC_UPDATE0_UDR) {
                max77696_rtc_commit(s);
            }
            data &= ~(RTC_UPDATE0_RBUDR | RTC_UPDATE0_UDR);
        } else if (max77696_is_main(s) && index == ADC_CNTL) {
            /* ADCCONV is cleared by hardware when conversion completes. */
            data &= ~ADC_CNTL_CONV;
        } else if (max77696_is_main(s) && index == ADC_CHSEL) {
            max77696_adc_select_channel(s, data);
        }
        if (max77696_is_main(s) && index == VREG_EPDCNFG) {
            bool was_enabled = s->regs[index] & VREG_EPDCNFG_EPDEN;
            bool enabled = data & VREG_EPDCNFG_EPDEN;

            s->regs[index] = data;
            if (enabled != was_enabled) {
                qemu_set_irq(s->irq[1], enabled);
            }
        } else {
            s->regs[index] = data;
        }
        if (max77696_is_rtc(s)) {
            if (index >= RTC_SEC && index <= RTC_DOM) {
                s->rtc_time_dirty = true;
            }
            if (index == RTC_AE1 || index == RTC_AE2 || index == RTC_INTM) {
                max77696_rtc_schedule(s);
                max77696_update_irq(s);
            }
        }
        if (max77696_is_main(s) &&
            (index == GLBLINTM || index == INTTOP1M || index == CHG_INT_MASK)) {
            max77696_update_irq(s);
        } else if (max77696_is_uic(s) &&
                   (index == UIC_INTMASK1 || index == UIC_INTMASK2 ||
                    index == UIC_SYSCTRL1)) {
            max77696_update_irq(s);
        }
        max77696_advance(s);
    }
    return 0;
}

static uint8_t max77696_recv(I2CSlave *i2c)
{
    MAX77696State *s = MAX77696(i2c);
    unsigned index = max77696_index(s);
    uint8_t value = s->regs[index];

    /* TOPSYS edge status is latched until software reads GLBLINT. */
    if (max77696_is_main(s) && index == GLBLINT) {
        s->regs[GLBLINT] = 0;
        max77696_update_irq(s);
    } else if (max77696_is_main(s) && index == CHG_INT) {
        s->regs[CHG_INT] = 0;
        max77696_update_irq(s);
    } else if (max77696_is_uic(s) &&
               (index == UIC_INT1 || index == UIC_INT2)) {
        s->regs[index] = 0;
        max77696_update_irq(s);
    }

    if (max77696_is_rtc(s) && index == RTC_INT) {
        s->regs[RTC_INT] = 0;
        max77696_update_irq(s);
    }

    max77696_advance(s);
    return value;
}

static int max77696_event(I2CSlave *i2c, enum i2c_event event)
{
    MAX77696State *s = MAX77696(i2c);

    if (event == I2C_START_SEND) {
        s->len = 0;
    }
    return 0;
}

static void max77696_realize(DeviceState *dev, Error **errp)
{
    MAX77696State *s = MAX77696(dev);
    I2CSlave *i2c = I2C_SLAVE(dev);

    s->width = i2c->address == 0x34 ? 2 : 1;
    if (s->width == 2) {
        /* A charged 4.0 V virtual battery, 100% SOC, 25 C. */
        s->regs[0x09 * 2] = 0x00;
        s->regs[0x09 * 2 + 1] = 0xc8;
        s->regs[0x06 * 2] = 0x00;
        s->regs[0x06 * 2 + 1] = 0x64;
        s->regs[0x0d * 2] = 0x00;
        s->regs[0x0d * 2 + 1] = 0x64;
        s->regs[0x0e * 2] = 0x00;
        s->regs[0x0e * 2 + 1] = 0x64;
        s->regs[0x08 * 2] = 0x00;
        s->regs[0x08 * 2 + 1] = 0x19;
    } else if (max77696_is_uic(s)) {
        /*
         * An ordinary USB cable leaves the accessory ID pin open. Zero is
         * reserved for an OTG accessory that makes the Kindle a USB host.
         */
        s->regs[UIC_STATUS2] = UIC_STATUS2_ADC_MASK;
    } else if (max77696_is_rtc(s)) {
        s->rtc_offset = 0;
        s->rtc_last_tick = qemu_clock_get_ms(rtc_clock) / 1000;
        s->regs[RTC_INTM] = 0xff;
        s->rtc_timer = timer_new_ms(rtc_clock, max77696_rtc_tick, s);
        max77696_rtc_capture(s);
    }

    if (max77696_is_main(s) || max77696_is_uic(s)) {
        qdev_init_gpio_in_named(dev, max77696_vbus, "vbus", 1);
    }
    if (max77696_is_main(s)) {
        /* The physical PMIC IRQ pin is active-low. */
        qdev_init_gpio_in_named(dev, max77696_power_button, "power-button", 1);
        qdev_init_gpio_in_named(dev, max77696_uic_irq, "uic-irq", 1);
        qdev_init_gpio_in_named(dev, max77696_rtc_irq, "rtc-irq-in", 1);
        s->regs[GLBLINTM] = 0xff;
        s->regs[INTTOP1M] = 0xff;
        s->regs[CHG_INT_MASK] = 0xff;
        s->power_down = false;
        s->input_handler = qemu_input_handler_register(
            dev, &max77696_input_handler);
        max77696_update_irq(s);
        qemu_set_irq(s->irq[1], 0);
    }
}

static void max77696_reset(DeviceState *dev)
{
    MAX77696State *s = MAX77696(dev);

    if (max77696_is_main(s)) {
        s->regs[GLBLINT] = 0;
        s->regs[CHG_INT] = 0;
        s->regs[GLBLSTAT] &= ~GLBLSTAT_EN0_S;
        s->regs[INTTOP1] &= ~INTTOP1_TOPSYS;
        s->power_down = false;
        max77696_update_irq(s);
        qemu_set_irq(s->irq[1], 0);
    } else if (max77696_is_uic(s)) {
        s->regs[UIC_INT1] = 0;
        s->regs[UIC_INT2] = 0;
        max77696_update_irq(s);
    }
}

static void max77696_unrealize(DeviceState *dev)
{
    MAX77696State *s = MAX77696(dev);

    if (s->input_handler) {
        qemu_input_handler_unregister(s->input_handler);
    }
    if (s->rtc_timer) {
        timer_free(s->rtc_timer);
    }
}

static int max77696_post_load(void *opaque, int version_id)
{
    MAX77696State *s = opaque;

    max77696_update_irq(s);
    if (max77696_is_rtc(s)) {
        max77696_rtc_schedule(s);
    }
    return 0;
}

static void max77696_init(Object *obj)
{
    MAX77696State *s = MAX77696(obj);

    qdev_init_gpio_out(DEVICE(obj), s->irq, ARRAY_SIZE(s->irq));
    qdev_init_gpio_out_named(DEVICE(obj), &s->rtc_irq, "rtc-irq", 1);
}

static const VMStateDescription max77696_vmstate = {
    .name = "max77696",
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = max77696_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX77696State),
        VMSTATE_UINT8_ARRAY(regs, MAX77696State, 512),
        VMSTATE_UINT8(pointer, MAX77696State),
        VMSTATE_UINT8(byte, MAX77696State),
        VMSTATE_UINT8(len, MAX77696State),
        VMSTATE_UINT8(width, MAX77696State),
        VMSTATE_INT64(rtc_offset, MAX77696State),
        VMSTATE_INT64(rtc_last_tick, MAX77696State),
        VMSTATE_BOOL(power_down, MAX77696State),
        VMSTATE_UINT8_ARRAY(rtc_alarm, MAX77696State, RTC_FIELDS * 2),
        VMSTATE_BOOL(rtc_time_dirty, MAX77696State),
        VMSTATE_BOOL(rtc_pending, MAX77696State),
        VMSTATE_BOOL_ARRAY(rtc_alarm_match, MAX77696State, 2),
        VMSTATE_END_OF_LIST()
    },
};

static void max77696_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = max77696_realize;
    dc->unrealize = max77696_unrealize;
    dc->vmsd = &max77696_vmstate;
    device_class_set_legacy_reset(dc, max77696_reset);
    sc->send = max77696_send;
    sc->recv = max77696_recv;
    sc->event = max77696_event;
}

static const TypeInfo max77696_info = {
    .name = TYPE_MAX77696,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAX77696State),
    .instance_init = max77696_init,
    .class_init = max77696_class_init,
};

static void max77696_register_types(void)
{
    type_register_static(&max77696_info);
}
type_init(max77696_register_types)
