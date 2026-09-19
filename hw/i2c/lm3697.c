/*
 * TI LM3697 dual-bank frontlight controller used by Kindle Oasis 3.
 *
 * Models the register interface, hardware enable and open/short status.
 * Brightness is retained for the guest; the e-paper surface is unlit.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_LM3697 "lm3697"
OBJECT_DECLARE_SIMPLE_TYPE(LM3697State, LM3697)

struct LM3697State {
    I2CSlave parent_obj;
    uint8_t regs[0xb5];
    uint8_t pointer;
    bool expect_pointer;
    bool enabled;
};

static void lm3697_register_reset(LM3697State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x00] = 0x01; /* Silicon revision */
    s->regs[0x10] = 0x06; /* HVLED1 bank A, HVLED2/3 bank B */
    s->regs[0x15] = 0x33; /* Reserved power-on value */
    s->regs[0x1b] = 0xcf; /* Auto-frequency threshold */
    s->regs[0x1c] = 0x0c; /* PWM polarity and zero detection */
    s->regs[0x17] = 0x13; /* Full-scale current A/B: 20.2 mA */
    s->regs[0x18] = 0x13;
    s->regs[0x19] = 0x07; /* All feedback channels enabled */
}

static void lm3697_enable(void *opaque, int line, int level)
{
    LM3697State *s = opaque;

    if (s->enabled && !level) {
        lm3697_register_reset(s);
    }
    s->enabled = !!level;
}

static int lm3697_event(I2CSlave *i2c, enum i2c_event event)
{
    LM3697State *s = LM3697(i2c);

    if (!s->enabled) {
        return -1;
    }
    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static int lm3697_send(I2CSlave *i2c, uint8_t data)
{
    LM3697State *s = LM3697(i2c);
    uint8_t reg;

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
        return 0;
    }
    reg = s->pointer++;
    if (reg == 0x01 && (data & 1)) {
        lm3697_register_reset(s);
    } else if (reg < sizeof(s->regs) && reg != 0 &&
               reg != 0xb0 && reg != 0xb2) {
        s->regs[reg] = data;
    }
    return 0;
}

static uint8_t lm3697_recv(I2CSlave *i2c)
{
    LM3697State *s = LM3697(i2c);
    uint8_t reg = s->pointer++;

    return reg < sizeof(s->regs) ? s->regs[reg] : 0;
}

static void lm3697_reset(DeviceState *dev)
{
    LM3697State *s = LM3697(dev);

    lm3697_register_reset(s);
    s->pointer = 0;
    s->expect_pointer = true;
    s->enabled = false;
}

static const VMStateDescription vmstate_lm3697 = {
    .name = TYPE_LM3697,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, LM3697State),
        VMSTATE_UINT8_ARRAY(regs, LM3697State, 0xb5),
        VMSTATE_UINT8(pointer, LM3697State),
        VMSTATE_BOOL(expect_pointer, LM3697State),
        VMSTATE_BOOL(enabled, LM3697State),
        VMSTATE_END_OF_LIST()
    },
};

static void lm3697_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), lm3697_enable, "enable", 1);
}

static void lm3697_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);

    ic->event = lm3697_event;
    ic->send = lm3697_send;
    ic->recv = lm3697_recv;
    device_class_set_legacy_reset(dc, lm3697_reset);
    dc->vmsd = &vmstate_lm3697;
}

static const TypeInfo lm3697_info = {
    .name = TYPE_LM3697,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LM3697State),
    .instance_init = lm3697_init,
    .class_init = lm3697_class_init,
};

static void lm3697_register_types(void)
{
    type_register_static(&lm3697_info);
}
type_init(lm3697_register_types)
