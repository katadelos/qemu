/*
 * Fitipower FP9966 dual-channel frontlight control.
 * Register map and 11-bit brightness packing follow Amazon's
 * U-Boot drivers/video/backlight_fp9966.c. Electrical fault generation,
 * analog ramp shape and LED illumination are not modeled.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/fp9966.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

struct FP9966State {
    I2CSlave parent_obj;
    uint8_t regs[0x12];
    uint8_t pointer;
    bool expect_pointer;
    bool enabled;
};

static int fp9966_send(I2CSlave *i2c, uint8_t data)
{
    FP9966State *s = FP9966(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else if (s->pointer < sizeof(s->regs)) {
        unsigned reg = s->pointer++;

        /* Device-owned fault flags; no electrical/thermal fault is injected. */
        if (reg != 0x0f && reg != 0x11) {
            s->regs[reg] = reg == 4 || reg == 6 ? data & 7 : data;
        }
    }
    return 0;
}

static uint8_t fp9966_recv(I2CSlave *i2c)
{
    FP9966State *s = FP9966(i2c);

    return s->pointer < sizeof(s->regs) ? s->regs[s->pointer++] : 0;
}

static int fp9966_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        FP9966(i2c)->expect_pointer = true;
    }
    return 0;
}

static void fp9966_enable(void *opaque, int line, int level)
{
    FP9966State *s = opaque;
    s->enabled = level != 0;
}

static void fp9966_reset(DeviceState *dev)
{
    FP9966State *s = FP9966(dev);

    /* Uncalibrated power-on storage. Stock firmware programs all operating
     * thresholds and brightness before enabling either channel. */
    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->expect_pointer = true;
    s->enabled = false;
}

static bool fp9966_get_enabled(Object *obj, Error **errp)
{
    return FP9966(obj)->enabled;
}

static void fp9966_init(Object *obj)
{
    FP9966State *s = FP9966(obj);

    qdev_init_gpio_in_named(DEVICE(obj), fp9966_enable, "enable", 1);
    object_property_add_bool(obj, "enabled", fp9966_get_enabled, NULL);
    object_property_add_uint8_ptr(obj, "channel1-brightness-low", &s->regs[4],
                                  OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "channel1-brightness-high", &s->regs[5],
                                  OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "channel2-brightness-low", &s->regs[6],
                                  OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "channel2-brightness-high", &s->regs[7],
                                  OBJ_PROP_FLAG_READ);
}

static const VMStateDescription fp9966_vmstate = {
    .name = TYPE_FP9966,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FP9966State),
        VMSTATE_UINT8_ARRAY(regs, FP9966State, 0x12),
        VMSTATE_UINT8(pointer, FP9966State),
        VMSTATE_BOOL(expect_pointer, FP9966State),
        VMSTATE_BOOL(enabled, FP9966State),
        VMSTATE_END_OF_LIST()
    },
};

static void fp9966_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, fp9966_reset);
    dc->vmsd = &fp9966_vmstate;
    sc->send = fp9966_send;
    sc->recv = fp9966_recv;
    sc->event = fp9966_event;
}

static const TypeInfo fp9966_info = {
    .name = TYPE_FP9966,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FP9966State),
    .class_init = fp9966_class_init,
    .instance_init = fp9966_init,
};

static void fp9966_register_types(void)
{
    type_register_static(&fp9966_info);
}
type_init(fp9966_register_types)
