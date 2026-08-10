/* TI DRV2667 piezo haptic controller used by Lab126 Icewine. */

#include "qemu/osdep.h"
#include "hw/i2c/drv2667.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define DRV2667_ID              7
#define DRV2667_REG_CONTROL     0x01
#define DRV2667_REG_CONTROL_2   0x02
#define DRV2667_REG_PAGE        0xff
#define DRV2667_RESET           0x80

struct DRV2667State {
    I2CSlave parent_obj;
    uint8_t regs[2][256];
    uint8_t pointer;
    uint8_t page;
    bool have_pointer;
};

static int drv2667_send(I2CSlave *i2c, uint8_t data)
{
    DRV2667State *s = DRV2667(i2c);

    if (!s->have_pointer) {
        s->pointer = data;
        s->have_pointer = true;
        return 0;
    }

    if (s->pointer == DRV2667_REG_PAGE) {
        s->page = data & 1;
    } else if (!s->page && s->pointer == DRV2667_REG_CONTROL_2 &&
               (data & DRV2667_RESET)) {
        memset(s->regs, 0, sizeof(s->regs));
        s->regs[0][DRV2667_REG_CONTROL] = DRV2667_ID << 3;
        s->page = 0;
    } else {
        s->regs[s->page][s->pointer] = data;
    }
    s->pointer++;
    return 0;
}

static uint8_t drv2667_recv(I2CSlave *i2c)
{
    DRV2667State *s = DRV2667(i2c);

    if (s->pointer == DRV2667_REG_PAGE) {
        return s->page;
    }
    return s->regs[s->page][s->pointer++];
}

static int drv2667_event(I2CSlave *i2c, enum i2c_event event)
{
    DRV2667State *s = DRV2667(i2c);

    if (event == I2C_START_SEND) {
        s->have_pointer = false;
    }
    return 0;
}

static void drv2667_reset(DeviceState *dev)
{
    DRV2667State *s = DRV2667(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0][DRV2667_REG_CONTROL] = DRV2667_ID << 3;
    s->pointer = 0;
    s->page = 0;
    s->have_pointer = false;
}

static const VMStateDescription drv2667_vmstate = {
    .name = TYPE_DRV2667,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, DRV2667State),
        VMSTATE_UINT8_2DARRAY(regs, DRV2667State, 2, 256),
        VMSTATE_UINT8(pointer, DRV2667State),
        VMSTATE_UINT8(page, DRV2667State),
        VMSTATE_BOOL(have_pointer, DRV2667State),
        VMSTATE_END_OF_LIST()
    },
};

static void drv2667_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, drv2667_reset);
    dc->vmsd = &drv2667_vmstate;
    sc->send = drv2667_send;
    sc->recv = drv2667_recv;
    sc->event = drv2667_event;
}

static const TypeInfo drv2667_info = {
    .name = TYPE_DRV2667,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(DRV2667State),
    .class_init = drv2667_class_init,
};

static void drv2667_register_types(void)
{
    type_register_static(&drv2667_info);
}

type_init(drv2667_register_types)
