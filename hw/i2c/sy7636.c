/*
 * SY7636 EPD power supply, as wired on Netronix E60T04 and TPV E60V00.
 * Register contract: Kobo Linux 4.9 include/linux/mfd/sy7636.h.
 * Models the digital interface and ideal rails; analog faults are unmodeled.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_SY7636 "sy7636"
OBJECT_DECLARE_SIMPLE_TYPE(SY7636State, SY7636)

struct SY7636State {
    I2CSlave parent_obj;
    uint8_t regs[9];
    uint8_t pointer;
    bool expect_pointer;
    bool powered, enabled;
    qemu_irq power_good;
};

static void sy7636_update(SY7636State *s)
{
    bool good = s->powered && s->enabled && (s->regs[0] & 0x80) &&
                !(s->regs[0] & 0x1f);

    s->regs[7] = good; /* FAULTFLAGS: PG bit, no analog faults. */
    qemu_set_irq(s->power_good, good);
}

static void sy7636_enable(void *opaque, int line, int level)
{
    SY7636State *s = opaque;
    s->enabled = level;
    if (!level) {
        memset(s->regs, 0, sizeof(s->regs));
        s->regs[8] = 25;
    }
    sy7636_update(s);
}

static void sy7636_power(void *opaque, int line, int level)
{
    SY7636State *s = opaque;
    s->powered = level;
    if (!level) {
        memset(s->regs, 0, sizeof(s->regs));
        s->regs[8] = 25;
    }
    sy7636_update(s);
}

static int sy7636_send(I2CSlave *i2c, uint8_t value)
{
    SY7636State *s = SY7636(i2c);
    if (s->expect_pointer) {
        s->pointer = value;
        s->expect_pointer = false;
    } else {
        unsigned reg = s->pointer++;
        if (reg < 7) {
            s->regs[reg] = value;
            sy7636_update(s);
        }
    }
    return 0;
}

static uint8_t sy7636_recv(I2CSlave *i2c)
{
    SY7636State *s = SY7636(i2c);
    unsigned reg = s->pointer++;
    return reg < sizeof(s->regs) ? s->regs[reg] : 0;
}

static int sy7636_event(I2CSlave *i2c, enum i2c_event event)
{
    SY7636State *s = SY7636(i2c);
    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return (event == I2C_START_SEND || event == I2C_START_RECV) &&
           !(s->powered && s->enabled) ? -1 : 0;
}

static void sy7636_reset(DeviceState *dev)
{
    SY7636State *s = SY7636(dev);
    s->pointer = 0;
    s->expect_pointer = true;
    s->enabled = false;
    sy7636_power(s, 0, 0);
}

static int sy7636_post_load(void *opaque, int version)
{
    sy7636_update(opaque);
    return 0;
}

static const VMStateDescription sy7636_vmstate = {
    .name = TYPE_SY7636,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = sy7636_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, SY7636State),
        VMSTATE_UINT8_ARRAY(regs, SY7636State, 9),
        VMSTATE_UINT8(pointer, SY7636State),
        VMSTATE_BOOL(expect_pointer, SY7636State),
        VMSTATE_BOOL(powered, SY7636State),
        VMSTATE_BOOL(enabled, SY7636State),
        VMSTATE_END_OF_LIST()
    },
};

static void sy7636_init(Object *obj)
{
    SY7636State *s = SY7636(obj);
    qdev_init_gpio_out_named(DEVICE(obj), &s->power_good, "power-good", 1);
    qdev_init_gpio_in_named(DEVICE(obj), sy7636_power, "power", 1);
    qdev_init_gpio_in_named(DEVICE(obj), sy7636_enable, "enable", 1);
}

static void sy7636_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);
    dc->vmsd = &sy7636_vmstate;
    device_class_set_legacy_reset(dc, sy7636_reset);
    ic->send = sy7636_send;
    ic->recv = sy7636_recv;
    ic->event = sy7636_event;
}

static const TypeInfo sy7636_type = {
    .name = TYPE_SY7636,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(SY7636State),
    .instance_init = sy7636_init,
    .class_init = sy7636_class_init,
};

static void sy7636_register_types(void)
{
    type_register_static(&sy7636_type);
}
type_init(sy7636_register_types)
