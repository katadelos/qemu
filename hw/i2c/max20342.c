/* Maxim MAX20342 USB-C charger detector register model. */

#include "qemu/osdep.h"
#include "hw/i2c/max20342.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

struct MAX20342State {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer;
    bool expect_pointer;
};

static int max20342_send(I2CSlave *i2c, uint8_t data)
{
    MAX20342State *s = MAX20342(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t max20342_recv(I2CSlave *i2c)
{
    MAX20342State *s = MAX20342(i2c);

    return s->regs[s->pointer++];
}

static int max20342_event(I2CSlave *i2c, enum i2c_event event)
{
    MAX20342State *s = MAX20342(i2c);

    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static void max20342_reset(DeviceState *dev)
{
    MAX20342State *s = MAX20342(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x00] = 0x01;
    s->pointer = 0;
    s->expect_pointer = true;
}

static const VMStateDescription max20342_vmstate = {
    .name = TYPE_MAX20342,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX20342State),
        VMSTATE_UINT8_ARRAY(regs, MAX20342State, 256),
        VMSTATE_UINT8(pointer, MAX20342State),
        VMSTATE_BOOL(expect_pointer, MAX20342State),
        VMSTATE_END_OF_LIST()
    },
};

static void max20342_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, max20342_reset);
    dc->vmsd = &max20342_vmstate;
    sc->send = max20342_send;
    sc->recv = max20342_recv;
    sc->event = max20342_event;
}

static const TypeInfo max20342_info = {
    .name = TYPE_MAX20342,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAX20342State),
    .class_init = max20342_class_init,
};

static void max20342_register_types(void)
{
    type_register_static(&max20342_info);
}
type_init(max20342_register_types)
