/* Fitipower FP9935/FP9937 electrophoretic display PMIC register model. */

#include "qemu/osdep.h"
#include "hw/i2c/fp9935.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define FP9935_REGISTER_COUNT 0x1c

struct FP9935State {
    I2CSlave parent_obj;
    uint8_t regs[FP9935_REGISTER_COUNT];
    uint8_t pointer;
    bool expect_pointer;
    bool enabled;
    bool temperature_enabled;
    qemu_irq power_good;
};

static void fp9935_update_power_good(FP9935State *s)
{
    qemu_set_irq(s->power_good, s->enabled);
}

static void fp9935_set_enable(void *opaque, int line, int level)
{
    FP9935State *s = opaque;

    s->enabled = level;
    fp9935_update_power_good(s);
}

static void fp9935_set_temperature_enable(void *opaque, int line, int level)
{
    FP9935State *s = opaque;

    /* EN_TS controls the ADC independently of the display rail enable. */
    s->temperature_enabled = level;
}

static int fp9935_send(I2CSlave *i2c, uint8_t data)
{
    FP9935State *s = FP9935(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else if (s->pointer < sizeof(s->regs)) {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t fp9935_recv(I2CSlave *i2c)
{
    FP9935State *s = FP9935(i2c);

    return s->pointer < sizeof(s->regs) ? s->regs[s->pointer++] : 0;
}

static int fp9935_event(I2CSlave *i2c, enum i2c_event event)
{
    FP9935State *s = FP9935(i2c);

    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static void fp9935_reset(DeviceState *dev)
{
    FP9935State *s = FP9935(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x00] = 25;
    s->pointer = 0;
    s->expect_pointer = true;
    s->enabled = false;
    s->temperature_enabled = false;
    qemu_set_irq(s->power_good, 0);
}

static int fp9935_post_load(void *opaque, int version_id)
{
    fp9935_update_power_good(opaque);
    return 0;
}

static const VMStateDescription fp9935_vmstate = {
    .name = TYPE_FP9935,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = fp9935_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FP9935State),
        VMSTATE_UINT8_ARRAY(regs, FP9935State, FP9935_REGISTER_COUNT),
        VMSTATE_UINT8(pointer, FP9935State),
        VMSTATE_BOOL(expect_pointer, FP9935State),
        VMSTATE_BOOL(enabled, FP9935State),
        VMSTATE_BOOL(temperature_enabled, FP9935State),
        VMSTATE_END_OF_LIST()
    },
};

static void fp9935_init(Object *obj)
{
    FP9935State *s = FP9935(obj);

    qdev_init_gpio_in_named(DEVICE(obj), fp9935_set_enable, "enable", 1);
    qdev_init_gpio_in_named(DEVICE(obj), fp9935_set_temperature_enable,
                             "temperature-enable", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->power_good,
                             "power-good", 1);
}

static void fp9935_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, fp9935_reset);
    dc->vmsd = &fp9935_vmstate;
    sc->send = fp9935_send;
    sc->recv = fp9935_recv;
    sc->event = fp9935_event;
}

static const TypeInfo fp9935_info = {
    .name = TYPE_FP9935,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FP9935State),
    .instance_init = fp9935_init,
    .class_init = fp9935_class_init,
};

/*
 * The AO5 FP9937 driver uses the same byte-addressed register interface,
 * signed temperature at register 0, EN/EN_TS inputs and power-good output.
 * Its additional voltage channels are programmed in registers 0x14..0x19;
 * voltage encodings are interpreted by the guest, so the register backing
 * and rail sequencing can be shared with FP9935.
 */
static const TypeInfo fp9937_info = {
    .name = TYPE_FP9937,
    .parent = TYPE_FP9935,
};

static void fp9935_register_types(void)
{
    type_register_static(&fp9935_info);
    type_register_static(&fp9937_info);
}
type_init(fp9935_register_types)
