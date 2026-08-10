/* Maxim MAX14656 USB charger detector register model. */

#include "qemu/osdep.h"
#include "hw/i2c/max14656.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

struct MAX14656State {
    I2CSlave parent_obj;
    uint8_t regs[10];
    uint8_t pointer;
    bool expect_pointer;
};

static int max14656_send(I2CSlave *i2c, uint8_t data)
{
    MAX14656State *s = MAX14656(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else if (s->pointer < sizeof(s->regs)) {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t max14656_recv(I2CSlave *i2c)
{
    MAX14656State *s = MAX14656(i2c);

    return s->pointer < sizeof(s->regs) ? s->regs[s->pointer++] : 0;
}

static int max14656_event(I2CSlave *i2c, enum i2c_event event)
{
    MAX14656State *s = MAX14656(i2c);

    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static void max14656_reset(DeviceState *dev)
{
    MAX14656State *s = MAX14656(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = 0x21;
    s->regs[7] = 0x0d;
    s->regs[8] = 0x8e;
    s->regs[9] = 0x8d;
    s->pointer = 0;
    s->expect_pointer = true;
}

static const VMStateDescription max14656_vmstate = {
    .name = TYPE_MAX14656,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX14656State),
        VMSTATE_UINT8_ARRAY(regs, MAX14656State, 10),
        VMSTATE_UINT8(pointer, MAX14656State),
        VMSTATE_BOOL(expect_pointer, MAX14656State),
        VMSTATE_END_OF_LIST()
    },
};

static void max14656_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, max14656_reset);
    dc->vmsd = &max14656_vmstate;
    sc->send = max14656_send;
    sc->recv = max14656_recv;
    sc->event = max14656_event;
}

static const TypeInfo max14656_info = {
    .name = TYPE_MAX14656,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAX14656State),
    .class_init = max14656_class_init,
};

static void max14656_register_types(void)
{
    type_register_static(&max14656_info);
}
type_init(max14656_register_types)
