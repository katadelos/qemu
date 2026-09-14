/*
 * TI OPT3001/OPT3006 and OPT4001 ambient light conversion register interfaces.
 * Sources: vendor drivers/iio/light/opt{3001,4001}.c;
 * TI SBOS681C sections7.4-7.6 and SBOS993A sections8.3-8.6:
 * https://www.ti.com/lit/ds/symlink/opt3001.pdf
 * https://www.ti.com/lit/ds/symlink/opt4001.pdf
 * Illumination is an explicit virtual environment input, not captured hardware.
 * Noise, spectral response, glass/frontlight coupling, hardware triggering,
 * general-call reset and SMBus ARA are absent.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/ti-opt-sensors.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qemu/bitops.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define TYPE_TI_OPT_SENSOR "ti-opt-sensor"
OBJECT_DECLARE_SIMPLE_TYPE(TIOptSensorState, TI_OPT_SENSOR)
struct TIOptSensorState {
    I2CSlave parent_obj;
    uint16_t regs[128];
    uint16_t sample_snapshot[8], read_word, write_word;
    uint8_t pointer, byte;
    uint8_t counter, high_faults, low_faults;
    uint32_t millilux;
    bool opt4001, picostar, expect_pointer, alarm, pulse;
    qemu_irq irq;
    QEMUTimer *conversion, *pulse_end;
};

static void opt_irq(TIOptSensorState *s)
{
    uint16_t cfg = s->regs[s->opt4001 ? 10 : 1];
    bool polarity = cfg & (s->opt4001 ? 4 : 8);
    bool active = s->alarm;
    if (s->opt4001) {
        unsigned mechanism = (s->regs[11] >> 2) & 3;
        active = mechanism ? s->pulse : s->alarm;
        if (s->picostar || !(s->regs[11] & 16)) {
            active = false;
        }
    }
    qemu_set_irq(s->irq, active ? polarity : !polarity);
}

static void opt_pulse_end(void *opaque)
{
    TIOptSensorState *s = opaque;
    s->pulse = false;
    opt_irq(s);
}

static int64_t opt_period_ns(TIOptSensorState *s)
{
    static const unsigned usec[] = {
        600, 1000, 1800, 3400, 6500, 12700,
        25000, 50000, 100000, 200000, 400000, 800000,
    };
    if (s->opt4001) {
        unsigned ct = (s->regs[10] >> 6) & 15;
        return ct < ARRAY_SIZE(usec) ? usec[ct] * 1000LL : 0;
    }
    return (s->regs[1] & 0x800) ? 800000000 : 100000000;
}

static void opt_schedule(TIOptSensorState *s, bool first)
{
    int64_t duration = opt_period_ns(s);
    if (s->opt4001 && first && ((s->regs[10] >> 4) & 3) < 3 &&
        !(s->regs[10] & 0x8000)) {
        duration += 500000; /* Tss: circuits recover from low-power standby. */
    }
    if (s->opt4001 && first && ((s->regs[10] >> 4) & 3) == 1) {
        duration += 500000; /* Documented forced auto-range reset. */
    }
    if (duration) {
        timer_mod(s->conversion, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + duration);
    }
}

static unsigned opt_crc(unsigned exponent, uint32_t mantissa, unsigned count)
{
    unsigned all = ctpop32(mantissa) + ctpop32(exponent) + ctpop32(count);
    unsigned alt = ctpop32(mantissa & 0xaaaaa) +
                   ctpop32(exponent & 0xa) + ctpop32(count & 0xa);
    unsigned fourth = ctpop32(mantissa & 0x88888) +
                      ctpop32(exponent & 8) + ctpop32(count & 8);
    return (all & 1) | ((alt & 1) << 1) | ((fourth & 1) << 2) |
           ((ctpop32(mantissa & 0x80808) & 1) << 3);
}

static uint64_t opt_threshold(uint16_t reg, unsigned shift)
{
    return (uint64_t)(reg & 0xfff) << ((reg >> 12) + shift);
}

static void opt_comparison(TIOptSensorState *s, uint64_t sample)
{
    unsigned ctrl = s->opt4001 ? 10 : 1;
    unsigned lowreg = s->opt4001 ? 8 : 2;
    unsigned shift = s->opt4001 ? 8 : 0;
    unsigned status = s->opt4001 ? 12 : 1;
    unsigned lo = s->opt4001 ? 1 : 0x20;
    unsigned hi = s->opt4001 ? 2 : 0x40;
    unsigned faults = 1 << (s->regs[ctrl] & 3);
    bool latch = s->regs[ctrl] & (s->opt4001 ? 8 : 16);
    bool low = sample < opt_threshold(s->regs[lowreg], shift);
    bool high = sample > opt_threshold(s->regs[lowreg + 1], shift);
    s->low_faults = low ? MIN(s->low_faults + 1, 8) : 0;
    s->high_faults = high ? MIN(s->high_faults + 1, 8) : 0;
    if (s->high_faults >= faults) {
        if (!latch) { s->regs[status] &= ~lo; }
        s->regs[status] |= hi;
        s->alarm = true;
    }
    if (s->low_faults >= faults) {
        if (!latch) { s->regs[status] &= ~hi; }
        s->regs[status] |= lo;
        s->alarm = latch;
    }
}

static void opt_convert(void *opaque)
{
    TIOptSensorState *s = opaque;
    unsigned mode, exponent, max_exponent, max_mantissa;
    uint64_t sample;
    uint32_t mantissa;
    if (s->opt4001) {
        mode = (s->regs[10] >> 4) & 3;
        exponent = (s->regs[10] >> 10) & 15;
        max_exponent = 8;
        max_mantissa = 0xfffff;
        /* millilux / (5/16 or7/16 millilux per count). */
        sample = (uint64_t)s->millilux * 16 / (s->picostar ? 5 : 7);
    } else {
        mode = (s->regs[1] >> 9) & 3;
        exponent = s->regs[1] >> 12;
        max_exponent = 11;
        max_mantissa = 0xfff;
        sample = s->millilux / 10; /* 0.01lux/count at exponent0. */
    }
    if (!mode) {
        return;
    }
    if (exponent == 12 || (s->opt4001 && mode == 1)) {
        exponent = 0;
        while (exponent < max_exponent &&
               (sample >> exponent) > max_mantissa) {
            exponent++;
        }
    }
    if (exponent > max_exponent) {
        return; /* Reserved range encoding cannot produce a conversion. */
    }
    bool overflow = (sample >> exponent) > max_mantissa;
    mantissa = MIN(sample >> exponent, max_mantissa);
    if (s->opt4001) {
        /* Shift historical register pairs only on a completed conversion. */
        memmove(&s->regs[2], &s->regs[0], 6 * sizeof(s->regs[0]));
        s->counter = (s->counter + 1) & 15;
        s->regs[0] = (exponent << 12) | (mantissa >> 8);
        s->regs[1] = ((mantissa & 255) << 8) | (s->counter << 4) |
                     opt_crc(exponent, mantissa, s->counter);
        s->regs[12] = (s->regs[12] & ~8) | 4 | (overflow ? 8 : 0);
        opt_comparison(s, (uint64_t)mantissa << exponent);
        unsigned mechanism = (s->regs[11] >> 2) & 3;
        if (mechanism == 1 || (mechanism == 3 && !(s->counter & 3))) {
            s->pulse = true;
            timer_mod(s->pulse_end, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
        }
        if (mode == 3) {
            opt_schedule(s, false);
        } else {
            s->regs[10] &= ~0x30;
        }
    } else {
        bool mask_exponent = (s->regs[1] & 4) && (s->regs[1] >> 12) != 12;
        s->regs[0] = (mask_exponent ? 0 : (exponent << 12)) | mantissa;
        s->regs[1] = (s->regs[1] & ~0x100) | 0x80 | (overflow ? 0x100 : 0);
        if ((s->regs[2] & 0xc000) == 0xc000) {
            s->alarm = true;
        } else {
            opt_comparison(s, (uint64_t)mantissa << exponent);
        }
        if (mode >= 2) {
            opt_schedule(s, false);
        } else {
            s->regs[1] &= ~0x600;
        }
    }
    opt_irq(s);
}

static uint16_t opt_read(TIOptSensorState *s, unsigned reg)
{
    uint16_t value = s->regs[reg & 127];
    if (s->opt4001 && reg < 8) {
        return s->sample_snapshot[reg];
    }
    if (s->opt4001 && reg == 12) {
        s->regs[12] &= ~4;
        if (s->regs[10] & 8) {
            s->regs[12] &= ~3;
            s->alarm = false;
        }
    } else if (!s->opt4001 && reg == 1) {
        s->regs[1] &= ~0x80;
        if ((s->regs[1] & 16) || (s->regs[2] & 0xc000) == 0xc000) {
            s->regs[1] &= ~0x60;
            s->alarm = false;
        }
    }
    opt_irq(s);
    return value;
}

static void opt_write(TIOptSensorState *s, unsigned reg, uint16_t value)
{
    if (s->opt4001) {
        if (reg == 8 || reg == 9) {
            s->regs[reg] = value;
            s->high_faults = s->low_faults = 0;
        } else if (reg == 10) {
            bool busy = timer_pending(s->conversion);
            unsigned mode = (value >> 4) & 3;
            if (busy && (mode == 1 || mode == 2)) {
                return; /* Trigger received during conversion is ignored. */
            }
            timer_del(s->conversion);
            s->regs[10] = value & ~0x4000;
            s->high_faults = s->low_faults = 0;
            if (mode) { opt_schedule(s, true); }
        } else if (reg == 11) {
            s->regs[11] = 0x8000 | (value & 0x1d);
        } else if (reg == 12 && value) {
            s->regs[12] &= ~4;
        }
    } else {
        if (reg == 2 || reg == 3) {
            s->regs[reg] = value;
            s->high_faults = s->low_faults = 0;
        } else if (reg == 1) {
            timer_del(s->conversion);
            s->regs[1] = (s->regs[1] & 0x1e0) | (value & ~0x1e0);
            s->high_faults = s->low_faults = 0;
            if (value & 0x600) {
                s->regs[1] &= ~0x80;
                if (!(value & 16) && (s->regs[2] & 0xc000) == 0xc000) {
                    s->alarm = false;
                }
                opt_schedule(s, true);
            }
        }
    }
    opt_irq(s);
}

static int opt_send(I2CSlave *dev, uint8_t value)
{
    TIOptSensorState *s = TI_OPT_SENSOR(dev);
    if (s->expect_pointer) {
        s->pointer = value & 127;
        s->expect_pointer = false;
        s->byte = 0;
    } else if (!s->byte++) {
        s->write_word = value << 8;
    } else {
        opt_write(s, s->pointer, s->write_word | value);
        s->byte = 0;
    }
    return 0;
}
static uint8_t opt_recv(I2CSlave *dev)
{
    TIOptSensorState *s = TI_OPT_SENSOR(dev);
    if (!s->byte++) {
        s->read_word = opt_read(s, s->pointer);
        return s->read_word >> 8;
    }
    s->byte = 0;
    if (s->opt4001 && (s->regs[11] & 1)) {
        s->pointer = (s->pointer + 1) & 127;
    }
    return s->read_word & 255;
}
static int opt_event(I2CSlave *dev, enum i2c_event event)
{
    TIOptSensorState *s = TI_OPT_SENSOR(dev);
    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
        s->byte = 0;
    } else if (event == I2C_START_RECV) {
        s->byte = 0;
        memcpy(s->sample_snapshot, s->regs, sizeof(s->sample_snapshot));
    }
    return 0;
}
static void opt_reset(DeviceState *dev)
{
    TIOptSensorState *s = TI_OPT_SENSOR(dev);
    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = s->byte = s->counter = 0;
    s->high_faults = s->low_faults = 0;
    s->alarm = s->pulse = false;
    s->expect_pointer = true;
    timer_del(s->conversion);
    timer_del(s->pulse_end);
    if (s->opt4001) {
        s->regs[9] = 0xbfff;
        s->regs[10] = 0x3208;
        s->regs[11] = 0x8011;
        s->regs[17] = 0x0121;
    } else {
        s->regs[1] = 0xc810;
        s->regs[3] = 0xbfff;
        s->regs[0x7e] = 0x5449;
        s->regs[0x7f] = 0x3001;
    }
    opt_irq(s);
}
static const Property opt_properties[] = {
    DEFINE_PROP_UINT32("millilux", TIOptSensorState, millilux, 100000),
    DEFINE_PROP_BOOL("picostar", TIOptSensorState, picostar, true),
};
static void opt_init(Object *obj)
{
    TIOptSensorState *s = TI_OPT_SENSOR(obj);
    s->conversion = timer_new_ns(QEMU_CLOCK_VIRTUAL, opt_convert, s);
    s->pulse_end = timer_new_ns(QEMU_CLOCK_VIRTUAL, opt_pulse_end, s);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
}
static void opt4001_init(Object *obj)
{ TI_OPT_SENSOR(obj)->opt4001 = true; }
static void opt_finalize(Object *obj)
{
    TIOptSensorState *s = TI_OPT_SENSOR(obj);
    timer_free(s->conversion);
    timer_free(s->pulse_end);
}
static void opt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);
    device_class_set_legacy_reset(dc, opt_reset);
    device_class_set_props(dc, opt_properties);
    sc->send = opt_send;
    sc->recv = opt_recv;
    sc->event = opt_event;
}
static const TypeInfo opt_types[] = {
    { .name = TYPE_TI_OPT_SENSOR, .parent = TYPE_I2C_SLAVE,
      .instance_size = sizeof(TIOptSensorState), .abstract = true,
      .instance_init = opt_init, .instance_finalize = opt_finalize,
      .class_init = opt_class_init },
    { .name = TYPE_TI_OPT3001, .parent = TYPE_TI_OPT_SENSOR },
    { .name = TYPE_TI_OPT4001, .parent = TYPE_TI_OPT_SENSOR,
      .instance_init = opt4001_init },
};
DEFINE_TYPES(opt_types)
