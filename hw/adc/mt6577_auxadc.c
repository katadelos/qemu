/*
 * MT6577-compatible AUXADC used by MT8512/MT8113.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Contract: Amazon Linux 4.9 drivers/iio/adc/mt6577_auxadc.c and
 * drivers/misc/mediatek/thermal/mt8512/mtk_ts_bts.c. The driver starts
 * conversion with a rising CON1 channel bit, checks CON2.BUSY, then reads
 * DATn.READY plus 12-bit data. Clearing CON1 clears the previous READY.
 * The 25 us conversion and 1 ms power-up intervals follow driver waits;
 * analog noise, automatic scan, and clock-rate variation are unmodeled.
 */
#include "qemu/osdep.h"
#include "hw/adc/mt6577_auxadc.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define CON1 0x04
#define CON2 0x10
#define DAT0 0x14
#define MISC 0x94
#define READY BIT(12)
#define POWER BIT(14)

static void adc_schedule(MT6577AuxADCState *s)
{
    int64_t next = INT64_MAX;
    for (unsigned i = 0; i < 16; i++) {
        if (s->pending & BIT(i)) {
            next = MIN(next, s->complete_ns[i]);
        }
    }
    if (next == INT64_MAX) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, next);
    }
}

static void adc_complete(void *opaque)
{
    MT6577AuxADCState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (unsigned i = 0; i < 16; i++) {
        if ((s->pending & BIT(i)) && s->complete_ns[i] <= now) {
            s->regs[DAT0 / 4 + i] = READY | s->sampled_raw[i];
            s->pending &= ~BIT(i);
            s->conversions++;
        }
    }
    adc_schedule(s);
}

static uint64_t adc_read(void *opaque, hwaddr offset, unsigned size)
{
    MT6577AuxADCState *s = opaque;
    if (offset == CON2) {
        return (s->regs[CON2 / 4] & ~1U) | !!s->pending;
    }
    if (offset == 8 || offset == 12) {
        return 0; /* CON1 SET/CLR write aliases. */
    }
    return s->regs[offset / 4];
}

static void adc_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    MT6577AuxADCState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (offset >= DAT0 && offset < DAT0 + 16 * 4) {
        return; /* Conversion result and READY are hardware-owned. */
    }
    if (offset == CON1 || offset == 8 || offset == 12) {
        uint16_t old = s->regs[CON1 / 4];
        uint16_t channels = offset == 8 ? old | value :
                            offset == 12 ? old & ~value : value;
        uint16_t trigger = channels & ~old;
        int64_t deadline = MAX(now, s->power_ready_ns);

        s->regs[CON1 / 4] = channels;
        for (unsigned i = 0; i < 16; i++) {
            if (!(channels & BIT(i))) {
                s->regs[DAT0 / 4 + i] &= ~READY;
                s->pending &= ~BIT(i);
            }
            if (s->pending & BIT(i)) {
                deadline = MAX(deadline, s->complete_ns[i]);
            }
        }
        if (s->regs[MISC / 4] & POWER) {
            for (unsigned i = 0; i < 16; i++) {
                if (trigger & BIT(i)) {
                    s->regs[DAT0 / 4 + i] &= ~READY;
                    s->pending |= BIT(i);
                    /* Clamp input before multiplication, including QOM edits. */
                    s->sampled_raw[i] = MIN(4095U,
                        (MIN(s->input_mv[i], 1500U) * 4096U + 750U) / 1500U);
                    deadline += 25000;
                    s->complete_ns[i] = deadline;
                }
            }
        }
        adc_schedule(s);
        return;
    }
    if (offset == MISC) {
        if ((value & POWER) && !(s->regs[MISC / 4] & POWER)) {
            s->power_ready_ns = now + 1000000;
        }
        if (!(value & POWER)) {
            s->pending = 0;
            for (unsigned i = 0; i < 16; i++) {
                s->regs[DAT0 / 4 + i] &= ~READY;
            }
            timer_del(s->timer);
        }
    }
    s->regs[offset / 4] = offset == CON2 ? value & ~1U : value;
}

static const MemoryRegionOps adc_ops = {
    .read = adc_read,
    .write = adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void adc_reset(DeviceState *dev)
{
    MT6577AuxADCState *s = MT6577_AUXADC(dev);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->sampled_raw, 0, sizeof(s->sampled_raw));
    memset(s->complete_ns, 0, sizeof(s->complete_ns));
    s->pending = 0;
    s->power_ready_ns = 0;
    s->conversions = 0;
    timer_del(s->timer);
}

static const VMStateDescription vmstate_adc = {
    .name = TYPE_MT6577_AUXADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MT6577AuxADCState, 0x1000 / 4),
        VMSTATE_UINT32_ARRAY(input_mv, MT6577AuxADCState, 16),
        VMSTATE_UINT32_ARRAY(sampled_raw, MT6577AuxADCState, 16),
        VMSTATE_INT64_ARRAY(complete_ns, MT6577AuxADCState, 16),
        VMSTATE_INT64(power_ready_ns, MT6577AuxADCState),
        VMSTATE_UINT16(pending, MT6577AuxADCState),
        VMSTATE_UINT64(conversions, MT6577AuxADCState),
        VMSTATE_TIMER_PTR(timer, MT6577AuxADCState),
        VMSTATE_END_OF_LIST()
    },
};

/* Barolo/Pisco have 100k pull-ups at 1800 mV and 100k NTCs at 25 C:
 * 900 mV input produces raw2458, then the stock integer divider gives900 mV.
 * All channels remain configurable analog inputs, not forced temperatures.
 */
#define ADC_CHANNEL(n) DEFINE_PROP_UINT32("channel" #n "-millivolts", \
                                        MT6577AuxADCState, input_mv[n], 900)
static const Property adc_properties[] = {
    ADC_CHANNEL(0), ADC_CHANNEL(1), ADC_CHANNEL(2), ADC_CHANNEL(3),
    ADC_CHANNEL(4), ADC_CHANNEL(5), ADC_CHANNEL(6), ADC_CHANNEL(7),
    ADC_CHANNEL(8), ADC_CHANNEL(9), ADC_CHANNEL(10), ADC_CHANNEL(11),
    ADC_CHANNEL(12), ADC_CHANNEL(13), ADC_CHANNEL(14), ADC_CHANNEL(15),
};

static void adc_init(Object *obj)
{
    MT6577AuxADCState *s = MT6577_AUXADC(obj);
    memory_region_init_io(&s->iomem, obj, &adc_ops, s,
                          TYPE_MT6577_AUXADC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, adc_complete, s);
    object_property_add_uint64_ptr(obj, "conversions", &s->conversions,
                                   OBJ_PROP_FLAG_READ);
}

static void adc_finalize(Object *obj)
{
    timer_free(MT6577_AUXADC(obj)->timer);
}

static void adc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, adc_reset);
    device_class_set_props(dc, adc_properties);
    dc->vmsd = &vmstate_adc;
}

static const TypeInfo adc_type = {
    .name = TYPE_MT6577_AUXADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT6577AuxADCState),
    .instance_init = adc_init,
    .instance_finalize = adc_finalize,
    .class_init = adc_class_init,
};

static void adc_register(void)
{
    type_register_static(&adc_type);
}
type_init(adc_register);
