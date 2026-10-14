/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TI TMP103 single-byte temperature/register interface.
 * Register map, reset state and conversion modes: TI SBOS545D, sections7.5-7.6
 * https://www.ti.com/lit/ds/symlink/tmp103.pdf
 * Temperature is an explicit virtual environment input. MDA/global access,
 * general-call reset, bus timeout and analog sensor error are not modeled.
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/tmp103.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define TMP103_CONVERSION_NS (26 * SCALE_MS)
OBJECT_DECLARE_SIMPLE_TYPE(TMP103State, TMP103)

struct TMP103State {
    I2CSlave parent_obj;
    uint8_t regs[4], pointer;
    bool expect_pointer, converting;
    int32_t temperature;
    QEMUTimer *conversion;
};

static void tmp103_start_conversion(TMP103State *s)
{
    s->converting = true;
    timer_mod(s->conversion, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              TMP103_CONVERSION_NS);
}

static void tmp103_convert(void *opaque)
{
    TMP103State *s = opaque;
    static const int64_t periods[] = {
        4 * NANOSECONDS_PER_SECOND, NANOSECONDS_PER_SECOND,
        NANOSECONDS_PER_SECOND / 4, NANOSECONDS_PER_SECOND / 8,
    };
    int64_t degrees;
    uint8_t flags = 0, mode;

    if (!s->converting) {
        tmp103_start_conversion(s);
        return;
    }
    degrees = ((int64_t)s->temperature +
               (s->temperature < 0 ? -500 : 500)) / 1000;
    s->regs[0] = (int8_t)MAX(-128, MIN(127, degrees));
    if ((int8_t)s->regs[0] > (int8_t)s->regs[3]) {
        flags |= 0x10;
    }
    if ((int8_t)s->regs[0] < (int8_t)s->regs[2]) {
        flags |= 0x08;
    }
    s->regs[1] = (s->regs[1] & ((s->regs[1] & 4) ? 0xff : 0xe7)) | flags;
    s->converting = false;
    mode = s->regs[1] & 3;
    if (mode == 1) {
        s->regs[1] &= ~3;
    } else if (mode & 2) {
        timer_mod(s->conversion, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  periods[(s->regs[1] >> 5) & 3] - TMP103_CONVERSION_NS);
    }
}

static int tmp103_send(I2CSlave *i2c, uint8_t data)
{
    TMP103State *s = TMP103(i2c);

    if (s->expect_pointer) {
        if (data >= ARRAY_SIZE(s->regs)) {
            return -1;
        }
        s->pointer = data;
        s->expect_pointer = false;
    } else if (s->pointer == 1) {
        s->regs[1] = (s->regs[1] & 0x98) | (data & 0x67);
        if (!(s->regs[1] & 3) && !s->converting) {
            timer_del(s->conversion);
        } else if ((s->regs[1] & 3) && !timer_pending(s->conversion)) {
            tmp103_start_conversion(s);
        }
    } else if (s->pointer > 1) {
        s->regs[s->pointer] = data;
    }
    return 0;
}

static uint8_t tmp103_recv(I2CSlave *i2c)
{
    TMP103State *s = TMP103(i2c);
    uint8_t value = s->regs[s->pointer];

    if (s->pointer == 1) {
        s->regs[1] &= ~0x18;
    }
    return value;
}

static int tmp103_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        TMP103(i2c)->expect_pointer = true;
    }
    return 0;
}

static void tmp103_reset(DeviceState *dev)
{
    TMP103State *s = TMP103(dev);

    s->regs[0] = 0;
    s->regs[1] = 2;
    s->regs[2] = (uint8_t)-10;
    s->regs[3] = 60;
    s->pointer = 0;
    s->expect_pointer = true;
    tmp103_start_conversion(s);
}

static const VMStateDescription tmp103_vmstate = {
    .name = TYPE_TMP103,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, TMP103State),
        VMSTATE_UINT8_ARRAY(regs, TMP103State, 4),
        VMSTATE_UINT8(pointer, TMP103State),
        VMSTATE_BOOL(expect_pointer, TMP103State),
        VMSTATE_BOOL(converting, TMP103State),
        VMSTATE_INT32(temperature, TMP103State),
        VMSTATE_TIMER_PTR(conversion, TMP103State),
        VMSTATE_END_OF_LIST()
    },
};

static const Property tmp103_properties[] = {
    DEFINE_PROP_INT32("temperature-millidegrees", TMP103State, temperature, 25000),
};

static void tmp103_init(Object *obj)
{
    TMP103(obj)->conversion = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         tmp103_convert, obj);
}

static void tmp103_finalize(Object *obj)
{
    timer_free(TMP103(obj)->conversion);
}

static void tmp103_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);

    device_class_set_props(dc, tmp103_properties);
    device_class_set_legacy_reset(dc, tmp103_reset);
    dc->vmsd = &tmp103_vmstate;
    ic->send = tmp103_send;
    ic->recv = tmp103_recv;
    ic->event = tmp103_event;
}

static const TypeInfo tmp103_type = {
    .name = TYPE_TMP103,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TMP103State),
    .instance_init = tmp103_init,
    .instance_finalize = tmp103_finalize,
    .class_init = tmp103_class_init,
};

static void tmp103_register_types(void)
{
    type_register_static(&tmp103_type);
}
type_init(tmp103_register_types)
