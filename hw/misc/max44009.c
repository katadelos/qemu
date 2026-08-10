/* Maxim MAX44009 ambient light sensor. SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/max44009.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define REG_INT_STATUS  0x00
#define REG_INT_ENABLE  0x01
#define REG_CONFIG      0x02
#define REG_LUX_HIGH    0x03
#define REG_LUX_LOW     0x04
#define REG_THRESH_HIGH 0x05
#define REG_THRESH_LOW  0x06
#define REG_THRESH_TIME 0x07
#define REG_CLOCK1      0x09
#define REG_CLOCK2      0x0a
#define REG_GAIN1       0x0b
#define REG_GAIN2       0x0c
#define REG_CONTROL     0x0d

static uint16_t max44009_count_from_code(uint8_t code)
{
    return (code & 0xf) << (code >> 4);
}

static void max44009_encode_lux(MAX44009State *s)
{
    uint64_t count = s->lux_millilux / 45;
    unsigned exponent = 0;

    while (count > 255 && exponent < 14) {
        count = (count + 1) >> 1;
        exponent++;
    }
    count = MIN(count, 255);
    s->regs[REG_LUX_HIGH] = (exponent << 4) | (count >> 4);
    s->regs[REG_LUX_LOW] = count & 0xf;
}

static void max44009_update_irq(MAX44009State *s)
{
    uint16_t count = (((s->regs[REG_LUX_HIGH] & 0xf) << 4) |
                      (s->regs[REG_LUX_LOW] & 0xf))
                     << (s->regs[REG_LUX_HIGH] >> 4);
    uint16_t high = max44009_count_from_code(s->regs[REG_THRESH_HIGH]);
    uint16_t low = max44009_count_from_code(s->regs[REG_THRESH_LOW]);

    s->interrupt = (count > high || count < low) &&
                   (s->regs[REG_INT_ENABLE] & 1);
    s->regs[REG_INT_STATUS] = s->interrupt;
    /* INT is open-drain and active low. */
    qemu_set_irq(s->irq, !s->interrupt);
}

static uint8_t max44009_recv(I2CSlave *i2c)
{
    MAX44009State *s = MAX44009(i2c);
    uint8_t value;

    max44009_encode_lux(s);
    value = s->regs[s->pointer & 0xf];
    if (s->pointer >= REG_CLOCK1 && s->pointer <= REG_GAIN2) {
        value = ~value;
    }
    if (s->pointer == REG_INT_STATUS) {
        s->interrupt = false;
        s->regs[REG_INT_STATUS] = 0;
        qemu_set_irq(s->irq, 1);
    }
    return value;
}

static int max44009_send(I2CSlave *i2c, uint8_t data)
{
    MAX44009State *s = MAX44009(i2c);

    if (s->tx_len++ == 0) {
        s->pointer = data & 0xf;
        return 0;
    }
    if (s->pointer != REG_INT_STATUS && s->pointer != REG_LUX_HIGH &&
        s->pointer != REG_LUX_LOW) {
        s->regs[s->pointer] = data;
        max44009_update_irq(s);
    }
    return 0;
}

static int max44009_event(I2CSlave *i2c, enum i2c_event event)
{
    MAX44009State *s = MAX44009(i2c);

    if (event == I2C_START_SEND) {
        s->tx_len = 0;
    }
    return 0;
}

static void max44009_reset(DeviceState *dev)
{
    MAX44009State *s = MAX44009(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[REG_THRESH_HIGH] = 0xff;
    s->regs[REG_THRESH_LOW] = 0x00;
    s->regs[REG_THRESH_TIME] = 0xff;
    /* Factory timing words are active-low in the Lab126 driver. */
    s->regs[REG_CLOCK1] = 0;
    s->regs[REG_CLOCK2] = 0;
    s->regs[REG_GAIN1] = 0;
    s->regs[REG_GAIN2] = 0;
    s->pointer = 0;
    s->tx_len = 0;
    s->interrupt = false;
    max44009_encode_lux(s);
    qemu_set_irq(s->irq, 1);
}

static const VMStateDescription vmstate_max44009 = {
    .name = TYPE_MAX44009,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX44009State),
        VMSTATE_UINT8_ARRAY(regs, MAX44009State, 16),
        VMSTATE_UINT8(pointer, MAX44009State),
        VMSTATE_UINT8(tx_len, MAX44009State),
        VMSTATE_UINT32(lux_millilux, MAX44009State),
        VMSTATE_BOOL(interrupt, MAX44009State),
        VMSTATE_END_OF_LIST()
    },
};

static const Property max44009_properties[] = {
    DEFINE_PROP_UINT32("lux-millilux", MAX44009State, lux_millilux, 100000),
};

static void max44009_realize(DeviceState *dev, Error **errp)
{
    MAX44009State *s = MAX44009(dev);

    qdev_init_gpio_out(dev, &s->irq, 1);
}

static void max44009_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(klass);

    dc->realize = max44009_realize;
    device_class_set_legacy_reset(dc, max44009_reset);
    device_class_set_props(dc, max44009_properties);
    dc->vmsd = &vmstate_max44009;
    ic->event = max44009_event;
    ic->recv = max44009_recv;
    ic->send = max44009_send;
}

static const TypeInfo max44009_info = {
    .name = TYPE_MAX44009,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAX44009State),
    .class_init = max44009_class_init,
};

static void max44009_register_types(void)
{
    type_register_static(&max44009_info);
}
type_init(max44009_register_types)
