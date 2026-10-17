/* MAX77818 PMIC, charger and ModelGauge fuel gauge.
 * Register interface: reMarkable Linux's max77818 and max17042 drivers.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#define TYPE_MAX77818 "max77818"
OBJECT_DECLARE_SIMPLE_TYPE(MAX77818State, MAX77818)
struct MAX77818State {
    I2CSlave parent_obj;
    uint16_t regs[256];
    uint8_t pointer, byte;
    bool have_pointer;
    qemu_irq irq;
};
static bool max77818_word(MAX77818State *s)
{
    return I2C_SLAVE(s)->address == 0x36;
}
static void max77818_advance(MAX77818State *s)
{
    if (max77818_word(s) && !s->byte) {
        s->byte = 1;
    } else {
        s->byte = 0;
        s->pointer++;
    }
}
static int max77818_send(I2CSlave *i2c, uint8_t value)
{
    MAX77818State *s = MAX77818(i2c);
    if (!s->have_pointer) {
        s->pointer = value;
        s->byte = 0;
        s->have_pointer = true;
        return 0;
    }
    s->regs[s->pointer] = deposit32(s->regs[s->pointer], s->byte * 8, 8, value);
    if (max77818_word(s) && s->pointer == 0xbb && s->byte) {
        s->regs[0xbb] &= ~BIT(5); /* Load Model completes. */
    }
    max77818_advance(s);
    return 0;
}
static uint8_t max77818_recv(I2CSlave *i2c)
{
    MAX77818State *s = MAX77818(i2c);
    uint16_t value = s->regs[s->pointer];
    /* Locked ModelGauge characterization RAM reads as zero. */
    if (max77818_word(s) && s->pointer >= 0x80 && s->pointer < 0xb0 &&
        (s->regs[0x62] != 0x59 || s->regs[0x63] != 0xc4)) {
        value = 0;
    }
    value >>= s->byte * 8;
    max77818_advance(s);
    return value;
}
static int max77818_event(I2CSlave *i2c, enum i2c_event event)
{
    MAX77818State *s = MAX77818(i2c);
    if (event == I2C_START_SEND) {
        s->have_pointer = false;
    }
    return 0;
}
static void max77818_reset(DeviceState *dev)
{
    MAX77818State *s = MAX77818(dev);
    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = s->byte = 0;
    s->have_pointer = false;
    if (max77818_word(s)) {
        s->regs[0x05] = s->regs[0x0f] = 5241; /* 85% of 3083 mAh. */
        s->regs[0x06] = s->regs[0x0d] = s->regs[0x0e] = s->regs[0xff] = 85 << 8;
        s->regs[0x07] = 100 << 8;
        s->regs[0x08] = s->regs[0x16] = 25 << 8;
        s->regs[0x09] = s->regs[0x19] = s->regs[0xee] = s->regs[0xfb] = 51200;
        s->regs[0x10] = s->regs[0x18] = s->regs[0x23] = 0x1816;
        s->regs[0x11] = 12800;
        s->regs[0x1b] = 0xc8c8;
        s->regs[0x1d] = 0x0a14;
        s->regs[0x21] = 0x4010;
        s->regs[0x3a] = 0x9157;
    } else if (I2C_SLAVE(s)->address == 0x66) {
        s->regs[0x20] = 0x18;
        s->regs[0x21] = 0x03;
    } else {
        s->regs[0xb2] = BIT(2) | BIT(3); /* battery present, input absent. */
        s->regs[0xb3] = 0;
        s->regs[0xb4] = 0x35; /* healthy battery, charger off. */
    }
    qemu_set_irq(s->irq, 1);
}
static const VMStateDescription max77818_vmstate = {
    .name = TYPE_MAX77818, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX77818State),
        VMSTATE_UINT16_ARRAY(regs, MAX77818State, 256),
        VMSTATE_UINT8(pointer, MAX77818State),
        VMSTATE_UINT8(byte, MAX77818State),
        VMSTATE_BOOL(have_pointer, MAX77818State),
        VMSTATE_END_OF_LIST()
    },
};
static void max77818_init(Object *obj)
{
    qdev_init_gpio_out_named(DEVICE(obj), &MAX77818(obj)->irq, "irq", 1);
}
static void max77818_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);
    dc->vmsd = &max77818_vmstate;
    device_class_set_legacy_reset(dc, max77818_reset);
    ic->send = max77818_send;
    ic->recv = max77818_recv;
    ic->event = max77818_event;
}
static const TypeInfo max77818_info = {
    .name = TYPE_MAX77818, .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAX77818State), .instance_init = max77818_init,
    .class_init = max77818_class_init,
};
static void max77818_register(void) { type_register_static(&max77818_info); }
type_init(max77818_register)
