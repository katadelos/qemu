/* Maxim MAX20342 USB-C charger detector register model. */

#include "qemu/osdep.h"
#include "hw/i2c/max20342.h"
#include "migration/vmstate.h"
#include "hw/core/irq.h"
#include "qemu/module.h"

struct MAX20342State {
    I2CSlave parent_obj;
    qemu_irq nirq;
    uint8_t regs[256];
    uint8_t pointer;
    bool expect_pointer;
};

static void max20342_reset(DeviceState *dev);

static void max20342_update_irq(MAX20342State *s)
{
    unsigned pending = 0;
    for (unsigned reg = 1; reg <= 6; reg++) {
        /* A set mask bit enables this source (vendor driver defaults). */
        pending |= s->regs[reg] & s->regs[reg + 11];
    }
    qemu_set_irq(s->nirq, !(pending && (s->regs[0x15] & 0x80)));
}

static int max20342_send(I2CSlave *i2c, uint8_t data)
{
    MAX20342State *s = MAX20342(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else {
        uint8_t reg = s->pointer++;
        if (reg == 0x19 && (data & 2)) {
            max20342_reset(DEVICE(s));
        } else if (reg >= 0x0c && reg != 0x18 &&
                   !(reg >= 0x2c && reg <= 0x2f) &&
                   !(reg >= 0x54 && reg <= 0x5b) &&
                   !(reg >= 0x60 && reg <= 0x62)) {
            s->regs[reg] = data;
        }
        max20342_update_irq(s);
    }
    return 0;
}

static uint8_t max20342_recv(I2CSlave *i2c)
{
    MAX20342State *s = MAX20342(i2c);

    uint8_t reg = s->pointer++;
    uint8_t value = s->regs[reg];
    if (reg >= 1 && reg <= 6) {
        s->regs[reg] = 0; /* Interrupt latches clear on read. */
        max20342_update_irq(s);
    }
    return value;
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
    s->regs[0x00] = 0x01; /* Virtual silicon revision. */
    s->regs[0x07] = 0x02; /* Unplugged: VBUS at safe 0 V. */
    s->regs[0x0b] = 0x80; /* I2C interface ready, OVP switch open. */
    s->pointer = 0;
    s->expect_pointer = true;
    max20342_update_irq(s);
}

static void max20342_init(Object *obj)
{
    MAX20342State *s = MAX20342(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->nirq, 1);
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
    .instance_init = max20342_init,
};

static void max20342_register_types(void)
{
    type_register_static(&max20342_info);
}
type_init(max20342_register_types)
