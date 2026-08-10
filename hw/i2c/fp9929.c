/* Fitipower FP9929 E Ink panel power-supply register model. */

#include "qemu/osdep.h"
#include "hw/i2c/fp9929.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

struct FP9929State {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer;
    bool expect_pointer;
};

static int fp9929_send(I2CSlave *i2c, uint8_t data)
{
    FP9929State *s = FP9929(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t fp9929_recv(I2CSlave *i2c)
{
    FP9929State *s = FP9929(i2c);

    return s->regs[s->pointer++];
}

static int fp9929_event(I2CSlave *i2c, enum i2c_event event)
{
    FP9929State *s = FP9929(i2c);

    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static void fp9929_reset(DeviceState *dev)
{
    FP9929State *s = FP9929(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[2] = 116;
    s->pointer = 0;
    s->expect_pointer = true;
}

static const VMStateDescription fp9929_vmstate = {
    .name = TYPE_FP9929,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FP9929State),
        VMSTATE_UINT8_ARRAY(regs, FP9929State, 256),
        VMSTATE_UINT8(pointer, FP9929State),
        VMSTATE_BOOL(expect_pointer, FP9929State),
        VMSTATE_END_OF_LIST()
    },
};

static void fp9929_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, fp9929_reset);
    dc->vmsd = &fp9929_vmstate;
    sc->send = fp9929_send;
    sc->recv = fp9929_recv;
    sc->event = fp9929_event;
}

static const TypeInfo fp9929_info = {
    .name = TYPE_FP9929,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FP9929State),
    .class_init = fp9929_class_init,
};

static void fp9929_register_types(void)
{
    type_register_static(&fp9929_info);
}
type_init(fp9929_register_types)
