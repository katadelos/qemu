/* TI LM3695 frontlight controller register interface used by Lab126 Rex. */

#include "qemu/osdep.h"
#include "hw/i2c/lm3695.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

struct LM3695State {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer;
    uint8_t len;
};

static int lm3695_send(I2CSlave *i2c, uint8_t data)
{
    LM3695State *s = LM3695(i2c);

    if (!s->len++) {
        s->pointer = data;
    } else {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t lm3695_recv(I2CSlave *i2c)
{
    LM3695State *s = LM3695(i2c);

    return s->regs[s->pointer++];
}

static int lm3695_event(I2CSlave *i2c, enum i2c_event event)
{
    LM3695State *s = LM3695(i2c);

    if (event == I2C_START_SEND) {
        s->len = 0;
    }
    return 0;
}

static void lm3695_reset(DeviceState *dev)
{
    LM3695State *s = LM3695(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->len = 0;
}

static const VMStateDescription lm3695_vmstate = {
    .name = TYPE_LM3695,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, LM3695State),
        VMSTATE_UINT8_ARRAY(regs, LM3695State, 256),
        VMSTATE_UINT8(pointer, LM3695State),
        VMSTATE_UINT8(len, LM3695State),
        VMSTATE_END_OF_LIST()
    },
};

static void lm3695_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, lm3695_reset);
    dc->vmsd = &lm3695_vmstate;
    sc->send = lm3695_send;
    sc->recv = lm3695_recv;
    sc->event = lm3695_event;
}

static const TypeInfo lm3695_info = {
    .name = TYPE_LM3695,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LM3695State),
    .class_init = lm3695_class_init,
};

static void lm3695_register_types(void)
{
    type_register_static(&lm3695_info);
}
type_init(lm3695_register_types)
