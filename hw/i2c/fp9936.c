/* SPDX-License-Identifier: GPL-2.0-or-later */
/* FP9936 display PMIC: vendor FT9936/fiti_core.{c,h} register contract.
 * Virtual 25 C temperature; no analog rail/fault or waveform-dependent model.
 * Enable comes from BD72720 GPIO5, power-good feeds PA6 GPIO/EINT135. */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "qemu/timer.h"
#define TYPE_FP9936 "fp9936"
OBJECT_DECLARE_SIMPLE_TYPE(FP9936State, FP9936)
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define FP9936_REGISTER_COUNT 0x1c

struct FP9936State {
    I2CSlave parent_obj;
    uint8_t regs[FP9936_REGISTER_COUNT];
    uint8_t pointer;
    bool expect_pointer;
    bool enabled;
    qemu_irq power_good;
    QEMUTimer *settle;
};

static void fp9936_settled(void *opaque)
{
    FP9936State *s = opaque;
    qemu_set_irq(s->power_good, s->enabled);
}

static void fp9936_set_enable(void *opaque, int line, int level)
{
    FP9936State *s = opaque;
    s->enabled = level;
    /* Nominal digital settling, not captured analog rail timing. */
    if (level) {
        timer_mod(s->settle, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
    } else {
        timer_del(s->settle);
        qemu_set_irq(s->power_good, 0);
    }
}

static int fp9936_send(I2CSlave *i2c, uint8_t data)
{
    FP9936State *s = FP9936(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else if (s->pointer < sizeof(s->regs)) {
        unsigned reg = s->pointer++;
        if (reg == 0x1a || reg == 0x1b) {
            s->regs[reg] &= data; /* Vendor init clears fault flags with zero. */
        } else if (reg) {
            s->regs[reg] = data;
        }
    }
    return 0;
}

static uint8_t fp9936_recv(I2CSlave *i2c)
{
    FP9936State *s = FP9936(i2c);

    return s->pointer < sizeof(s->regs) ? s->regs[s->pointer++] : 0;
}

static int fp9936_event(I2CSlave *i2c, enum i2c_event event)
{
    FP9936State *s = FP9936(i2c);

    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static void fp9936_reset(DeviceState *dev)
{
    FP9936State *s = FP9936(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x00] = 25;
    s->pointer = 0;
    s->expect_pointer = true;
    s->enabled = false;
    timer_del(s->settle);
    qemu_set_irq(s->power_good, 0);
}

static const VMStateDescription fp9936_vmstate = {
    .name = TYPE_FP9936,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FP9936State),
        VMSTATE_UINT8_ARRAY(regs, FP9936State, FP9936_REGISTER_COUNT),
        VMSTATE_UINT8(pointer, FP9936State),
        VMSTATE_BOOL(expect_pointer, FP9936State),
        VMSTATE_BOOL(enabled, FP9936State),
        VMSTATE_TIMER_PTR(settle, FP9936State),
        VMSTATE_END_OF_LIST()
    },
};

static void fp9936_init(Object *obj)
{
    FP9936State *s = FP9936(obj);

    s->settle = timer_new_ns(QEMU_CLOCK_VIRTUAL, fp9936_settled, s);
    qdev_init_gpio_in_named(DEVICE(obj), fp9936_set_enable, "enable", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->power_good,
                             "power-good", 1);
}

static void fp9936_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, fp9936_reset);
    dc->vmsd = &fp9936_vmstate;
    sc->send = fp9936_send;
    sc->recv = fp9936_recv;
    sc->event = fp9936_event;
}

static void fp9936_finalize(Object *obj)
{
    timer_free(FP9936(obj)->settle);
}

static const TypeInfo fp9936_info = {
    .name = TYPE_FP9936,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FP9936State),
    .instance_init = fp9936_init,
    .instance_finalize = fp9936_finalize,
    .class_init = fp9936_class_init,
};

static void fp9936_register_types(void)
{
    type_register_static(&fp9936_info);
}
type_init(fp9936_register_types)
