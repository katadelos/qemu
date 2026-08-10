/* MAX77696 companion register banks used by Lab126 Wario. */

#include "qemu/osdep.h"
#include "hw/i2c/max77696.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "system/rtc.h"

#define MAX77696_MAIN_ADDR       0x3c
#define MAX77696_UIC_ADDR        0x35
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
#define VREG_EPDCNFG             0x60
#define VREG_EPDCNFG_EPDEN       (1U << 7)

#define MAX77696_RTC_ADDR        0x68
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
    uint8_t regs[512];
    uint8_t pointer;
    uint8_t byte;
    uint8_t len;
    uint8_t width;
    int64_t rtc_offset;
};

static bool max77696_is_rtc(MAX77696State *s)
{
    return I2C_SLAVE(s)->address == MAX77696_RTC_ADDR;
}

static bool max77696_is_main(MAX77696State *s)
{
    return I2C_SLAVE(s)->address == MAX77696_MAIN_ADDR;
}

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

static void max77696_rtc_capture(MAX77696State *s)
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

static void max77696_rtc_commit(MAX77696State *s)
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
        max77696_advance(s);
    }
    return 0;
}

static uint8_t max77696_recv(I2CSlave *i2c)
{
    MAX77696State *s = MAX77696(i2c);
    uint8_t value = s->regs[max77696_index(s)];

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
    } else if (i2c->address == MAX77696_UIC_ADDR) {
        /*
         * No cable is attached to the virtual USB port.  The Wario OTG
         * glue interprets an ADC code of zero as an OTG accessory and then
         * endlessly alternates host and gadget mode while trying to
         * enumerate it.  A non-zero open-circuit code keeps the port idle,
         * matching the physical PMIC when no cable is present.
         */
        s->regs[UIC_STATUS2] = UIC_STATUS2_ADC_MASK;
    } else if (max77696_is_rtc(s)) {
        s->rtc_offset = 0;
        max77696_rtc_capture(s);
    }

    if (max77696_is_main(s)) {
        /* The physical PMIC IRQ pin is active-low. */
        qemu_set_irq(s->irq[0], 1);
        qemu_set_irq(s->irq[1], 0);
    }
}

static void max77696_reset(DeviceState *dev)
{
    MAX77696State *s = MAX77696(dev);

    if (max77696_is_main(s)) {
        qemu_set_irq(s->irq[0], 1);
        qemu_set_irq(s->irq[1], 0);
    }
}

static void max77696_init(Object *obj)
{
    MAX77696State *s = MAX77696(obj);

    qdev_init_gpio_out(DEVICE(obj), s->irq, ARRAY_SIZE(s->irq));
}

static const VMStateDescription max77696_vmstate = {
    .name = "max77696",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX77696State),
        VMSTATE_UINT8_ARRAY(regs, MAX77696State, 512),
        VMSTATE_UINT8(pointer, MAX77696State),
        VMSTATE_UINT8(byte, MAX77696State),
        VMSTATE_UINT8(len, MAX77696State),
        VMSTATE_UINT8(width, MAX77696State),
        VMSTATE_INT64(rtc_offset, MAX77696State),
        VMSTATE_END_OF_LIST()
    },
};

static void max77696_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = max77696_realize;
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
