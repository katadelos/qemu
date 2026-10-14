/*
 * TI LM3630A dual-bank frontlight register interface used by Kobo Clara HD.
 * Models register access, hardware enable and fault-free operation. The
 * brightness/current registers are retained; the e-paper surface is unlit.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_LM3630A "lm3630a"
OBJECT_DECLARE_SIMPLE_TYPE(LM3630AState, LM3630A)

struct LM3630AState {
    I2CSlave parent_obj;
    uint8_t regs[0x51];
    uint8_t pointer;
    bool expect_pointer;
    bool enabled;
};

static void lm3630a_reset_registers(LM3630AState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x02] = 0x38; /* Boost control reset value. */
    s->regs[0x05] = s->regs[0x06] = 0x1f; /* Full-scale current. */
    s->pointer = 0;
    s->expect_pointer = true;
}

static void lm3630a_enable(void *opaque, int line, int level)
{
    LM3630AState *s = opaque;

    if (s->enabled && !level) {
        lm3630a_reset_registers(s);
    }
    s->enabled = level != 0;
}

static int lm3630a_event(I2CSlave *i2c, enum i2c_event event)
{
    LM3630AState *s = LM3630A(i2c);

    if (!s->enabled) {
        return -1;
    }
    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static int lm3630a_send(I2CSlave *i2c, uint8_t data)
{
    LM3630AState *s = LM3630A(i2c);
    uint8_t reg;

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
        return 0;
    }
    reg = s->pointer++;
    if (reg < sizeof(s->regs) && reg != 0x09 && reg != 0x0b &&
        reg != 0x12 && reg != 0x13 && reg != 0x1f) {
        s->regs[reg] = data;
    }
    return 0;
}

static uint8_t lm3630a_recv(I2CSlave *i2c)
{
    LM3630AState *s = LM3630A(i2c);
    uint8_t reg = s->pointer++;

    return reg < sizeof(s->regs) ? s->regs[reg] : 0;
}

static void lm3630a_reset(DeviceState *dev)
{
    LM3630AState *s = LM3630A(dev);

    lm3630a_reset_registers(s);
    s->enabled = false;
}

static const VMStateDescription lm3630a_vmstate = {
    .name = TYPE_LM3630A,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, LM3630AState),
        VMSTATE_UINT8_ARRAY(regs, LM3630AState, 0x51),
        VMSTATE_UINT8(pointer, LM3630AState),
        VMSTATE_BOOL(expect_pointer, LM3630AState),
        VMSTATE_BOOL(enabled, LM3630AState),
        VMSTATE_END_OF_LIST()
    },
};

static void lm3630a_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), lm3630a_enable, "enable", 1);
}

static void lm3630a_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->vmsd = &lm3630a_vmstate;
    device_class_set_legacy_reset(dc, lm3630a_reset);
    sc->send = lm3630a_send;
    sc->recv = lm3630a_recv;
    sc->event = lm3630a_event;
}

static const TypeInfo lm3630a_type = {
    .name = TYPE_LM3630A,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LM3630AState),
    .instance_init = lm3630a_init,
    .class_init = lm3630a_class_init,
};

static void lm3630a_register_types(void)
{
    type_register_static(&lm3630a_type);
}
type_init(lm3630a_register_types)
