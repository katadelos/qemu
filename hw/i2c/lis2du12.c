/* STMicroelectronics LIS2DU12 accelerometer register model. */

#include "qemu/osdep.h"
#include "hw/i2c/lis2du12.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define LIS2DU12_CTRL1          0x10
#define LIS2DU12_CTRL5          0x14
#define LIS2DU12_STATUS         0x25
#define LIS2DU12_OUT_Z_L        0x2c
#define LIS2DU12_WHO_AM_I       0x43
#define LIS2DU12_WHO_AM_I_VALUE 0x45
#define LIS2DU12_SW_RESET       BIT(5)
#define LIS2DU12_ODR_MASK       0xf0
#define LIS2DU12_DATA_READY     BIT(0)

struct LIS2DU12State {
    I2CSlave parent_obj;
    uint8_t regs[0x60];
    uint8_t pointer;
    bool expect_pointer;
};

static void lis2du12_register_reset(LIS2DU12State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[LIS2DU12_WHO_AM_I] = LIS2DU12_WHO_AM_I_VALUE;

    /* Stationary, face-up device: Z = +1 g in the default +/-2 g range. */
    s->regs[LIS2DU12_OUT_Z_L] = 0x00;
    s->regs[LIS2DU12_OUT_Z_L + 1] = 0x40;
}

static int lis2du12_send(I2CSlave *i2c, uint8_t data)
{
    LIS2DU12State *s = LIS2DU12(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
        return 0;
    }

    if (s->pointer < sizeof(s->regs)) {
        uint8_t reg = s->pointer++;

        if (reg == LIS2DU12_CTRL1 && (data & LIS2DU12_SW_RESET)) {
            lis2du12_register_reset(s);
        } else if (reg != LIS2DU12_WHO_AM_I) {
            s->regs[reg] = data;
        }
    }
    return 0;
}

static uint8_t lis2du12_recv(I2CSlave *i2c)
{
    LIS2DU12State *s = LIS2DU12(i2c);
    uint8_t reg = s->pointer++;

    if (reg >= sizeof(s->regs)) {
        return 0;
    }
    if (reg == LIS2DU12_STATUS &&
        (s->regs[LIS2DU12_CTRL5] & LIS2DU12_ODR_MASK)) {
        return s->regs[reg] | LIS2DU12_DATA_READY;
    }
    return s->regs[reg];
}

static int lis2du12_event(I2CSlave *i2c, enum i2c_event event)
{
    LIS2DU12State *s = LIS2DU12(i2c);

    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static void lis2du12_reset(DeviceState *dev)
{
    LIS2DU12State *s = LIS2DU12(dev);

    lis2du12_register_reset(s);
    s->pointer = 0;
    s->expect_pointer = true;
}

static const VMStateDescription lis2du12_vmstate = {
    .name = TYPE_LIS2DU12,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, LIS2DU12State),
        VMSTATE_UINT8_ARRAY(regs, LIS2DU12State, 0x60),
        VMSTATE_UINT8(pointer, LIS2DU12State),
        VMSTATE_BOOL(expect_pointer, LIS2DU12State),
        VMSTATE_END_OF_LIST()
    },
};

static void lis2du12_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, lis2du12_reset);
    dc->vmsd = &lis2du12_vmstate;
    sc->send = lis2du12_send;
    sc->recv = lis2du12_recv;
    sc->event = lis2du12_event;
}

static const TypeInfo lis2du12_info = {
    .name = TYPE_LIS2DU12,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LIS2DU12State),
    .class_init = lis2du12_class_init,
};

static void lis2du12_register_types(void)
{
    type_register_static(&lis2du12_info);
}
type_init(lis2du12_register_types)
