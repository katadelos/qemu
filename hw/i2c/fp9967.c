/* Fitipower FP9967 dual-channel frontlight register model. */

#include "qemu/osdep.h"
#include "hw/i2c/fp9967.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

struct FP9967State {
    I2CSlave parent_obj;
    uint8_t regs[0x14];
    uint8_t pointer;
    bool expect_pointer;
};

static int fp9967_send(I2CSlave *i2c, uint8_t data)
{
    FP9967State *s = FP9967(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else if (s->pointer < sizeof(s->regs)) {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t fp9967_recv(I2CSlave *i2c)
{
    FP9967State *s = FP9967(i2c);

    return s->pointer < sizeof(s->regs) ? s->regs[s->pointer++] : 0;
}

static int fp9967_event(I2CSlave *i2c, enum i2c_event event)
{
    FP9967State *s = FP9967(i2c);

    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static void fp9967_reset(DeviceState *dev)
{
    FP9967State *s = FP9967(dev);
    static const uint8_t defaults[0x14] = {
        [0x00] = 0x11,
        [0x02] = 0x22,
        [0x03] = 0x76,
        [0x08] = 0x55,
        [0x09] = 0x0d,
        [0x0a] = 0x55,
        [0x0b] = 0x0d,
        [0x0e] = 0xdd,
        [0x10] = 0x01,
        [0x12] = 0xf5,
        [0x13] = 0x33,
    };

    memcpy(s->regs, defaults, sizeof(s->regs));
    s->pointer = 0;
    s->expect_pointer = true;
}

static const VMStateDescription fp9967_vmstate = {
    .name = TYPE_FP9967,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FP9967State),
        VMSTATE_UINT8_ARRAY(regs, FP9967State, 0x14),
        VMSTATE_UINT8(pointer, FP9967State),
        VMSTATE_BOOL(expect_pointer, FP9967State),
        VMSTATE_END_OF_LIST()
    },
};

static void fp9967_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, fp9967_reset);
    dc->vmsd = &fp9967_vmstate;
    sc->send = fp9967_send;
    sc->recv = fp9967_recv;
    sc->event = fp9967_event;
}

static const TypeInfo fp9967_info = {
    .name = TYPE_FP9967,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FP9967State),
    .class_init = fp9967_class_init,
};

static void fp9967_register_types(void)
{
    type_register_static(&fp9967_info);
}
type_init(fp9967_register_types)
