/*
 * Freescale MC13892 power-management IC
 *
 * The register transport is a 32-bit, active-high-CS SPI frame containing a
 * write flag, a six-bit register number, and 24 bits of register data.
 */
#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/ssi/mc13892.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/runstate.h"
#include "trace.h"

#define MC13892_NUM_REGS       64
#define MC13892_REG_INT_STAT0   0
#define MC13892_REG_INT_MASK0   1
#define MC13892_REG_INT_STAT1   3
#define MC13892_REG_INT_MASK1   4
#define MC13892_REG_INT_SENSE1  5
#define MC13892_REG_IDENT       7
#define MC13892_REG_RTCTOD      20
#define MC13892_REG_RTCTODA     21
#define MC13892_REG_RTCDAY      22
#define MC13892_REG_RTCDAYA     23
#define MC13892_REG_ADC1       44
#define MC13892_REG_ADC2       45
#define MC13892_REG_MASK        0x00ffffff
#define MC13892_ADC1_ASC        0x00100000
#define MC13892_ADC_BATT_RAW    832
#define MC13892_TODA            (1U << 1)
#define MC13892_PWRON1          (1U << 3)
#define MC13892_RTC_DAY_SECONDS (24 * 60 * 60)

struct MC13892State {
    SSIPeripheral parent_obj;
    qemu_irq irq;
    uint32_t regs[MC13892_NUM_REGS];
    uint32_t tx_frame;
    uint32_t read_value;
    uint8_t byte;
    uint8_t reg;
    bool write;
    bool power_down;
    int64_t rtc_offset;
    QEMUTimer *rtc_timer;
};

static void mc13892_update_irq(MC13892State *s);

static int64_t mc13892_rtc_count(MC13892State *s)
{
    return s->rtc_offset +
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / NANOSECONDS_PER_SECOND;
}

static void mc13892_rtc_reschedule(MC13892State *s)
{
    uint32_t alarm_tod = s->regs[MC13892_REG_RTCTODA];
    uint32_t alarm_day = s->regs[MC13892_REG_RTCDAYA] & 0x7fff;
    int64_t alarm;
    int64_t expiry;

    timer_del(s->rtc_timer);
    if (alarm_tod >= MC13892_RTC_DAY_SECONDS) {
        return;
    }

    alarm = (int64_t)alarm_day * MC13892_RTC_DAY_SECONDS + alarm_tod;
    expiry = (alarm - s->rtc_offset) * NANOSECONDS_PER_SECOND;
    if (expiry > qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)) {
        timer_mod(s->rtc_timer, expiry);
    }
}

static void mc13892_rtc_set_count(MC13892State *s, int64_t count)
{
    s->rtc_offset = count -
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / NANOSECONDS_PER_SECOND;
    mc13892_rtc_reschedule(s);
}

static uint32_t mc13892_read_register(MC13892State *s, uint8_t reg)
{
    int64_t count = mc13892_rtc_count(s);

    switch (reg) {
    case MC13892_REG_RTCTOD:
        return count % MC13892_RTC_DAY_SECONDS;
    case MC13892_REG_RTCDAY:
        return (count / MC13892_RTC_DAY_SECONDS) & 0x7fff;
    default:
        return s->regs[reg] & MC13892_REG_MASK;
    }
}

static void mc13892_rtc_alarm(void *opaque)
{
    MC13892State *s = opaque;

    s->regs[MC13892_REG_INT_STAT1] |= MC13892_TODA;
    mc13892_update_irq(s);
    if (!(s->regs[MC13892_REG_INT_MASK1] & MC13892_TODA)) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_RTC, NULL);
    }
}

static void mc13892_power_input(void *opaque, int line, int level)
{
    MC13892State *s = opaque;
    bool down = level;

    if (s->power_down == down) {
        return;
    }

    s->power_down = down;
    if (down) {
        s->regs[MC13892_REG_INT_SENSE1] &= ~MC13892_PWRON1;
    } else {
        s->regs[MC13892_REG_INT_SENSE1] |= MC13892_PWRON1;
    }
    s->regs[MC13892_REG_INT_STAT1] |= MC13892_PWRON1;
    trace_whitney_mc13892_power(down,
                                s->regs[MC13892_REG_INT_SENSE1],
                                s->regs[MC13892_REG_INT_STAT1],
                                s->regs[MC13892_REG_INT_MASK1]);
    mc13892_update_irq(s);
    if (down && !(s->regs[MC13892_REG_INT_MASK1] & MC13892_PWRON1)) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
    }
}

static void mc13892_update_irq(MC13892State *s)
{
    bool pending = (s->regs[MC13892_REG_INT_STAT0] &
                    ~s->regs[MC13892_REG_INT_MASK0]) ||
                   (s->regs[MC13892_REG_INT_STAT1] &
                    ~s->regs[MC13892_REG_INT_MASK1]);

    trace_whitney_mc13892_irq(pending,
                              s->regs[MC13892_REG_INT_STAT1],
                              s->regs[MC13892_REG_INT_MASK1]);
    qemu_set_irq(s->irq, pending);
}

static void mc13892_write_register(MC13892State *s, uint8_t reg,
                                   uint32_t value)
{
    int64_t count;

    value &= MC13892_REG_MASK;
    switch (reg) {
    case MC13892_REG_INT_STAT0:
    case MC13892_REG_INT_STAT1:
        s->regs[reg] &= ~value;
        break;
    case MC13892_REG_IDENT:
        break;
    case MC13892_REG_RTCTOD:
        count = mc13892_rtc_count(s);
        mc13892_rtc_set_count(s,
            count / MC13892_RTC_DAY_SECONDS * MC13892_RTC_DAY_SECONDS +
            value);
        break;
    case MC13892_REG_RTCDAY:
        count = mc13892_rtc_count(s);
        mc13892_rtc_set_count(s,
            (int64_t)(value & 0x7fff) * MC13892_RTC_DAY_SECONDS +
            count % MC13892_RTC_DAY_SECONDS);
        break;
    case MC13892_REG_RTCTODA:
    case MC13892_REG_RTCDAYA:
        s->regs[reg] = value;
        mc13892_rtc_reschedule(s);
        break;
    default:
        s->regs[reg] = value;
        break;
    }

    if (reg == MC13892_REG_ADC1 && (value & MC13892_ADC1_ASC)) {
        /* Complete the single battery-voltage conversion immediately. */
        s->regs[MC13892_REG_ADC1] &= ~MC13892_ADC1_ASC;
        s->regs[MC13892_REG_ADC2] = MC13892_ADC_BATT_RAW << 2;
    }
    mc13892_update_irq(s);
}

static uint32_t mc13892_transfer(SSIPeripheral *peripheral, uint32_t value)
{
    MC13892State *s = MC13892(peripheral);
    uint8_t input = value;
    uint8_t output = 0;

    if (s->byte == 0) {
        s->write = input & 0x80;
        s->reg = (input >> 1) & 0x3f;
        s->tx_frame = input;
        s->read_value = mc13892_read_register(s, s->reg);
    } else {
        s->tx_frame = (s->tx_frame << 8) | input;
        output = s->read_value >> (8 * (3 - s->byte));
    }

    if (++s->byte == 4) {
        if (s->write) {
            mc13892_write_register(s, s->reg, s->tx_frame);
        }
        s->byte = 0;
    }

    return output;
}

static int mc13892_set_cs(SSIPeripheral *peripheral, bool select)
{
    MC13892State *s = MC13892(peripheral);

    if (!select) {
        s->byte = 0;
    }
    return 0;
}

static void mc13892_reset(DeviceState *dev)
{
    MC13892State *s = MC13892(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[MC13892_REG_INT_MASK0] = MC13892_REG_MASK;
    s->regs[MC13892_REG_INT_MASK1] = MC13892_REG_MASK;
    s->regs[MC13892_REG_RTCTODA] = 0x1ffff;
    s->regs[MC13892_REG_RTCDAYA] = 0x7fff;
    /* PWRON1 is active low and the physical button powers up released. */
    s->regs[MC13892_REG_INT_SENSE1] = MC13892_PWRON1;
    /* MC13892 revision 2.0A, as fitted to production Whitney boards. */
    s->regs[MC13892_REG_IDENT] = 0x0045d0;
    s->tx_frame = 0;
    s->read_value = 0;
    s->byte = 0;
    s->reg = 0;
    s->write = false;
    s->power_down = false;
    s->rtc_offset =
        -qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / NANOSECONDS_PER_SECOND;
    timer_del(s->rtc_timer);
    mc13892_update_irq(s);
}

static void mc13892_realize(SSIPeripheral *peripheral, Error **errp)
{
    MC13892State *s = MC13892(peripheral);

    qdev_init_gpio_out_named(DEVICE(peripheral), &s->irq, "irq", 1);
    qdev_init_gpio_in_named(DEVICE(peripheral), mc13892_power_input,
                            "power-button", 1);
    s->rtc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                mc13892_rtc_alarm, s);
}

static void mc13892_unrealize(DeviceState *dev)
{
    MC13892State *s = MC13892(dev);

    timer_free(s->rtc_timer);
}

static int mc13892_post_load(void *opaque, int version_id)
{
    mc13892_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_mc13892 = {
    .name = TYPE_MC13892,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = mc13892_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, MC13892State),
        VMSTATE_UINT32_ARRAY(regs, MC13892State, MC13892_NUM_REGS),
        VMSTATE_UINT32(tx_frame, MC13892State),
        VMSTATE_UINT32(read_value, MC13892State),
        VMSTATE_UINT8(byte, MC13892State),
        VMSTATE_UINT8(reg, MC13892State),
        VMSTATE_BOOL(write, MC13892State),
        VMSTATE_BOOL(power_down, MC13892State),
        VMSTATE_INT64(rtc_offset, MC13892State),
        VMSTATE_TIMER_PTR(rtc_timer, MC13892State),
        VMSTATE_END_OF_LIST()
    }
};

static void mc13892_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *ssc = SSI_PERIPHERAL_CLASS(klass);

    ssc->realize = mc13892_realize;
    ssc->transfer = mc13892_transfer;
    ssc->set_cs = mc13892_set_cs;
    ssc->cs_polarity = SSI_CS_HIGH;
    dc->unrealize = mc13892_unrealize;
    dc->vmsd = &vmstate_mc13892;
    device_class_set_legacy_reset(dc, mc13892_reset);
}

static const TypeInfo mc13892_info = {
    .name = TYPE_MC13892,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(MC13892State),
    .class_init = mc13892_class_init,
};

static void mc13892_register_types(void)
{
    type_register_static(&mc13892_info);
}

type_init(mc13892_register_types)
