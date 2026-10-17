/* Register interfaces for the Paper Pro's power and display support chips.
 * Protocols: vendor PCA9450, MAX1726x, MAX77963, G2194 and Cumulus drivers.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define TYPE_RM3_I2C "remarkable3-i2c"
OBJECT_DECLARE_TYPE(RM3I2CState, RM3I2CClass, RM3_I2C)
enum { PCA9450, MAX17261, MAX77963, G2194, FUSB303B, CUMULUS, LIS2DW12 };
struct RM3I2CClass { I2CSlaveClass parent_class; unsigned chip; };
struct RM3I2CState {
    I2CSlave parent_obj;
    uint16_t regs[256];
    uint8_t pointer, byte;
    bool have_pointer;
    qemu_irq irq;
    QEMUTimer event_timer;
};

static bool rm3_i2c_word(RM3I2CState *s)
{
    return RM3_I2C_GET_CLASS(s)->chip == MAX17261;
}
static void rm3_i2c_irq(RM3I2CState *s)
{
    if (RM3_I2C_GET_CLASS(s)->chip == MAX77963) {
        qemu_set_irq(s->irq, !(s->regs[3] & ~s->regs[4]));
    } else if (RM3_I2C_GET_CLASS(s)->chip == LIS2DW12) {
        qemu_set_irq(s->irq, (s->regs[0x3b] & BIT(4)) &&
                     (s->regs[0x23] & BIT(7)) && (s->regs[0x3f] & BIT(5)));
    }
}
static bool lis2dw12_orientation_enabled(RM3I2CState *s)
{
    return (s->regs[0x20] & 0xf0) && (s->regs[0x23] & BIT(7)) &&
           (s->regs[0x3f] & BIT(5));
}
static void rm3_i2c_event_done(void *opaque)
{
    RM3I2CState *s = opaque;
    if (RM3_I2C_GET_CLASS(s)->chip == MAX77963) {
        s->regs[0x31] &= ~1;
        s->regs[3] |= BIT(5);
        s->regs[5] |= BIT(5);
        rm3_i2c_irq(s);
        return;
    }
    if (lis2dw12_orientation_enabled(s)) {
        /* The board's mount matrix exchanges X and Y. */
        s->regs[0x3a] = BIT(6) | BIT(1);
        s->regs[0x3b] |= BIT(4);
        s->regs[0x27] |= BIT(2);
        rm3_i2c_irq(s);
    }
}
static void rm3_i2c_advance(RM3I2CState *s)
{
    if (rm3_i2c_word(s) && !s->byte) {
        s->byte = 1;
    } else {
        s->byte = 0;
        s->pointer++;
    }
}
static int rm3_i2c_send(I2CSlave *i2c, uint8_t value)
{
    RM3I2CState *s = RM3_I2C(i2c);
    if (!s->have_pointer) {
        s->pointer = value;
        s->byte = 0;
        s->have_pointer = true;
    } else {
        bool orientation_enabled = lis2dw12_orientation_enabled(s);
        s->regs[s->pointer] = deposit32(s->regs[s->pointer], s->byte * 8, 8, value);
        if (rm3_i2c_word(s) && s->pointer == 0xbb && s->byte) {
            s->regs[0xbb] &= ~BIT(5);
        }
        if (RM3_I2C_GET_CLASS(s)->chip == MAX77963) {
            if (s->pointer == 0x31 && (value & 1)) {
                s->regs[5] &= ~BIT(5);
                timer_mod_ns(&s->event_timer,
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000000);
            }
            rm3_i2c_irq(s);
        } else if (RM3_I2C_GET_CLASS(s)->chip == LIS2DW12) {
            if (s->pointer == 0x21) {
                s->regs[0x21] &= ~BIT(6); /* Software reset completes. */
            }
            if (lis2dw12_orientation_enabled(s) && !orientation_enabled) {
                timer_mod_ns(&s->event_timer,
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000000);
            }
            rm3_i2c_irq(s);
        }
        rm3_i2c_advance(s);
    }
    return 0;
}
static uint8_t rm3_i2c_recv(I2CSlave *i2c)
{
    RM3I2CState *s = RM3_I2C(i2c);
    uint16_t value = s->regs[s->pointer];
    if (RM3_I2C_GET_CLASS(s)->chip == MAX77963 &&
        (s->pointer == 3 || s->pointer == 0x10)) {
        s->regs[s->pointer] = 0; /* Interrupt status clears on read. */
        rm3_i2c_irq(s);
    } else if (RM3_I2C_GET_CLASS(s)->chip == LIS2DW12 && s->pointer == 0x3a) {
        s->regs[0x3a] &= ~BIT(6);
        s->regs[0x3b] &= ~BIT(4);
        s->regs[0x27] &= ~BIT(2);
        rm3_i2c_irq(s);
    }
    if (rm3_i2c_word(s) && s->pointer >= 0x80 && s->pointer < 0xb0 &&
        (s->regs[0x62] != 0x59 || s->regs[0x63] != 0xc4)) {
        value = 0;
    }
    value >>= s->byte * 8;
    rm3_i2c_advance(s);
    return value;
}
static int rm3_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        RM3_I2C(i2c)->have_pointer = false;
    }
    return 0;
}
static void rm3_i2c_reset(DeviceState *dev)
{
    RM3I2CState *s = RM3_I2C(dev);
    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = s->byte = 0;
    s->have_pointer = false;
    timer_del(&s->event_timer);
    switch (RM3_I2C_GET_CLASS(s)->chip) {
    case PCA9450:
        s->regs[0] = 0x10;
        s->regs[0x10] = s->regs[0x13] = s->regs[0x16] = 0x59;
        s->regs[0x11] = s->regs[0x14] = s->regs[0x17] = 0x20;
        break;
    case MAX17261:
        s->regs[0x05] = s->regs[0x0f] = 6800;
        s->regs[0x06] = s->regs[0x0d] = s->regs[0x0e] = s->regs[0xff] = 85 << 8;
        s->regs[0x07] = 100 << 8;
        s->regs[0x08] = s->regs[0x16] = 25 << 8;
        s->regs[0x09] = s->regs[0x19] = s->regs[0xee] = s->regs[0xfb] = 51200;
        s->regs[0x10] = s->regs[0x18] = s->regs[0x23] = 8000;
        s->regs[0x11] = 12800;
        s->regs[0x1d] = 0x2210;
        s->regs[0x21] = 0x4031;
        break;
    case MAX77963:
        s->regs[0] = 0x63;
        s->regs[0x14] = 0x38; /* Healthy battery, charger disconnected. */
        s->regs[4] = s->regs[0x11] = 0xff;
        s->regs[5] = 0x29;
        /* 8.0 V battery and system rail, 25 C die. */
        s->regs[0x36] = s->regs[0x38] = 8;
        s->regs[0x37] = s->regs[0x39] = 0x88;
        break;
    case FUSB303B:
        s->regs[0] = 0x10;
        s->regs[1] = 0x10;
        break;
    case G2194:
        s->regs[0] = 0x94;
        break;
    case CUMULUS:
        s->regs[0] = 1;
        break;
    case LIS2DW12:
        s->regs[0x0f] = 0x44;
        s->regs[0x21] = BIT(2);
        s->regs[0x27] = BIT(0);
        s->regs[0x29] = 0x40; /* +1 g on X at the default +/-2 g range. */
        s->regs[0x3a] = BIT(1);
        break;
    }
    qemu_set_irq(s->irq, RM3_I2C_GET_CLASS(s)->chip != LIS2DW12);
}
static int rm3_i2c_post_load(void *opaque, int version)
{
    rm3_i2c_irq(opaque);
    return 0;
}
static const VMStateDescription rm3_i2c_vmstate = {
    .name = TYPE_RM3_I2C, .version_id = 1, .minimum_version_id = 1,
    .post_load = rm3_i2c_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, RM3I2CState),
        VMSTATE_UINT16_ARRAY(regs, RM3I2CState, 256),
        VMSTATE_UINT8(pointer, RM3I2CState),
        VMSTATE_UINT8(byte, RM3I2CState),
        VMSTATE_BOOL(have_pointer, RM3I2CState),
        VMSTATE_TIMER(event_timer, RM3I2CState),
        VMSTATE_END_OF_LIST()
    },
};
static void rm3_i2c_init(Object *obj)
{
    RM3I2CState *s = RM3_I2C(obj);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
    timer_init_ns(&s->event_timer, QEMU_CLOCK_VIRTUAL,
                  rm3_i2c_event_done, s);
}
static void rm3_i2c_class_init(ObjectClass *oc, const void *data)
{
    RM3I2CClass *rc = RM3_I2C_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);
    rc->chip = (uintptr_t)data;
    dc->vmsd = &rm3_i2c_vmstate;
    device_class_set_legacy_reset(dc, rm3_i2c_reset);
    ic->send = rm3_i2c_send;
    ic->recv = rm3_i2c_recv;
    ic->event = rm3_i2c_event;
}
static const TypeInfo rm3_i2c_types[] = {
    { .name = TYPE_RM3_I2C, .parent = TYPE_I2C_SLAVE,
      .instance_size = sizeof(RM3I2CState), .class_size = sizeof(RM3I2CClass),
      .instance_init = rm3_i2c_init, .abstract = true },
    { .name = "pca9450", .parent = TYPE_RM3_I2C, .class_init = rm3_i2c_class_init,
      .class_data = (void *)PCA9450 },
    { .name = "max17261", .parent = TYPE_RM3_I2C, .class_init = rm3_i2c_class_init,
      .class_data = (void *)MAX17261 },
    { .name = "max77963", .parent = TYPE_RM3_I2C, .class_init = rm3_i2c_class_init,
      .class_data = (void *)MAX77963 },
    { .name = "g2194", .parent = TYPE_RM3_I2C, .class_init = rm3_i2c_class_init,
      .class_data = (void *)G2194 },
    { .name = "fusb303b", .parent = TYPE_RM3_I2C, .class_init = rm3_i2c_class_init,
      .class_data = (void *)FUSB303B },
    { .name = "cumulus-bridge-i2c", .parent = TYPE_RM3_I2C, .class_init = rm3_i2c_class_init,
      .class_data = (void *)CUMULUS },
    { .name = "lis2dw12", .parent = TYPE_RM3_I2C, .class_init = rm3_i2c_class_init,
      .class_data = (void *)LIS2DW12 },
};
DEFINE_TYPES(rm3_i2c_types)
