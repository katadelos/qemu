/*
 * JD9930 EPD power supply, as wired on Netronix E70T04.
 * Register contract: Kobo Linux 4.9 include/linux/mfd/jd9930.h.
 * Models the digital interface and ideal rails; analog faults are unmodeled.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_JD9930 "jd9930"
OBJECT_DECLARE_SIMPLE_TYPE(JD9930State, JD9930)

struct JD9930State {
    I2CSlave parent_obj;
    uint8_t regs[13];
    uint8_t pointer;
    bool expect_pointer;
    bool powered, enabled;
    qemu_irq power_good;
};

static void jd9930_update(JD9930State *s)
{
    qemu_set_irq(s->power_good, s->powered && s->enabled);
}

static void jd9930_enable(void *opaque, int line, int level)
{
    JD9930State *s = opaque;
    s->enabled = level;
    jd9930_update(s);
}

static void jd9930_power(void *opaque, int line, int level)
{
    JD9930State *s = opaque;
    s->powered = level;
    if (!level) {
        memset(s->regs, 0, sizeof(s->regs));
        s->regs[0] = 25;
    }
    jd9930_update(s);
}

static int jd9930_send(I2CSlave *i2c, uint8_t value)
{
    JD9930State *s = JD9930(i2c);
    if (s->expect_pointer) {
        s->pointer = value;
        s->expect_pointer = false;
    } else {
        unsigned reg = s->pointer++;
        if (reg && reg < sizeof(s->regs)) {
            s->regs[reg] = value;
        }
    }
    return 0;
}

static uint8_t jd9930_recv(I2CSlave *i2c)
{
    JD9930State *s = JD9930(i2c);
    unsigned reg = s->pointer++;
    return reg < sizeof(s->regs) ? s->regs[reg] : 0;
}

static int jd9930_event(I2CSlave *i2c, enum i2c_event event)
{
    JD9930State *s = JD9930(i2c);
    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return (event == I2C_START_SEND || event == I2C_START_RECV) &&
           !s->powered ? -1 : 0;
}

static void jd9930_reset(DeviceState *dev)
{
    JD9930State *s = JD9930(dev);
    s->pointer = 0;
    s->expect_pointer = true;
    s->enabled = false;
    jd9930_power(s, 0, 0);
}

static int jd9930_post_load(void *opaque, int version)
{
    jd9930_update(opaque);
    return 0;
}

static const VMStateDescription jd9930_vmstate = {
    .name = TYPE_JD9930,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = jd9930_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, JD9930State),
        VMSTATE_UINT8_ARRAY(regs, JD9930State, 13),
        VMSTATE_UINT8(pointer, JD9930State),
        VMSTATE_BOOL(expect_pointer, JD9930State),
        VMSTATE_BOOL(powered, JD9930State),
        VMSTATE_BOOL(enabled, JD9930State),
        VMSTATE_END_OF_LIST()
    },
};

static void jd9930_init(Object *obj)
{
    JD9930State *s = JD9930(obj);
    qdev_init_gpio_out_named(DEVICE(obj), &s->power_good, "power-good", 1);
    qdev_init_gpio_in_named(DEVICE(obj), jd9930_power, "power", 1);
    qdev_init_gpio_in_named(DEVICE(obj), jd9930_enable, "enable", 1);
}

static void jd9930_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);
    dc->vmsd = &jd9930_vmstate;
    device_class_set_legacy_reset(dc, jd9930_reset);
    ic->send = jd9930_send;
    ic->recv = jd9930_recv;
    ic->event = jd9930_event;
}

static const TypeInfo jd9930_type = {
    .name = TYPE_JD9930,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(JD9930State),
    .instance_init = jd9930_init,
    .class_init = jd9930_class_init,
};

static void jd9930_register_types(void)
{
    type_register_static(&jd9930_type);
}
type_init(jd9930_register_types)
