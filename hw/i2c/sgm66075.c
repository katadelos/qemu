/* SPDX-License-Identifier: GPL-2.0-or-later */
/* SGM66075 digital regulator controls, from the PA6 vendor driver.
 * Analog loop, efficiency and faults are not modeled. */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_SGM66075 "sgm66075"
OBJECT_DECLARE_SIMPLE_TYPE(SGM66075State, SGM66075)

struct SGM66075State {
    I2CSlave parent_obj;
    uint8_t regs[5], pointer;
    bool expect_pointer;
};

static void sgm66075_reset(DeviceState *dev)
{
    SGM66075State *s = SGM66075(dev);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = 0x21;
    s->regs[2] = 15; /* Default selector: 2.85 V + 15 * 50 mV = 3.6 V. */
    s->pointer = 0;
    s->expect_pointer = true;
}

static int sgm66075_send(I2CSlave *i2c, uint8_t data)
{
    SGM66075State *s = SGM66075(i2c);
    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else {
        uint8_t reg = s->pointer++;
        if (reg == 1 && (data & 0x80)) {
            sgm66075_reset(DEVICE(s));
        } else if (reg >= 1 && reg <= 3) {
            s->regs[reg] = data;
        }
    }
    return 0;
}

static uint8_t sgm66075_recv(I2CSlave *i2c)
{
    SGM66075State *s = SGM66075(i2c);
    unsigned reg = s->pointer++;
    return reg < sizeof(s->regs) ? s->regs[reg] : 0;
}

static int sgm66075_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        SGM66075(i2c)->expect_pointer = true;
    }
    return 0;
}

static const VMStateDescription sgm66075_vmstate = {
    .name = TYPE_SGM66075,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, SGM66075State),
        VMSTATE_UINT8_ARRAY(regs, SGM66075State, 5),
        VMSTATE_UINT8(pointer, SGM66075State),
        VMSTATE_BOOL(expect_pointer, SGM66075State),
        VMSTATE_END_OF_LIST()
    },
};

static void sgm66075_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    device_class_set_legacy_reset(dc, sgm66075_reset);
    dc->vmsd = &sgm66075_vmstate;
    sc->send = sgm66075_send;
    sc->recv = sgm66075_recv;
    sc->event = sgm66075_event;
}

static const TypeInfo sgm66075_info = {
    .name = TYPE_SGM66075,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(SGM66075State),
    .class_init = sgm66075_class_init,
};

static void sgm66075_register_types(void)
{
    type_register_static(&sgm66075_info);
}
type_init(sgm66075_register_types)
