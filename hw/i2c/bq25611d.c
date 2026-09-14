/*
 * TI BQ25611D charging control, using the stock bq256xx_charger.c register
 * contract. No input supply is connected initially; battery charge physics
 * are not synthesized. Explicit VBUS presence exposes an SDP source.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/bq25611d.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"

struct BQ25611DState {
    I2CSlave parent_obj;
    uint8_t regs[13];
    uint8_t pointer;
    bool expect_pointer;
    bool vbus;
    qemu_irq nirq;
    QEMUTimer *watchdog;
};

static void bq25611d_irq(BQ25611DState *s)
{
    /* The driver consumes status on the falling edge, without a W1C ACK. */
    qemu_set_irq(s->nirq, 0);
    qemu_set_irq(s->nirq, 1);
}

static void bq25611d_update_status(BQ25611DState *s)
{
    s->regs[8] = s->vbus ? BIT(5) | BIT(2) : 0;
    if (s->vbus && (s->regs[1] & BIT(4)) && !(s->regs[7] & BIT(5))) {
        s->regs[8] |= BIT(4); /* Charge enabled with an input supply. */
    }
    /* VBUS_GD; no DPM threshold or electrical fault is asserted. */
    s->regs[10] = (s->regs[10] & 3) | (s->vbus ? BIT(7) : 0);
}

static void bq25611d_watchdog_arm(BQ25611DState *s)
{
    static const unsigned timeout_ms[] = { 0, 40000, 80000, 160000 };
    unsigned ms = timeout_ms[(s->regs[5] >> 4) & 3];

    if (s->vbus && ms) {
        timer_mod(s->watchdog, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + ms);
    } else {
        timer_del(s->watchdog);
    }
}

static void bq25611d_register_reset(BQ25611DState *s)
{
    static const uint8_t defaults[13] = {
        0x17, 0x1a, 0x91, 0x12, 0x40, 0x9e, 0xe6, 0x4c,
        0, 0, 0, 0x54, 0x75,
    };
    memcpy(s->regs, defaults, sizeof(s->regs));
    bq25611d_update_status(s);
    bq25611d_watchdog_arm(s);
}

static void bq25611d_watchdog_expire(void *opaque)
{
    BQ25611DState *s = opaque;
    bq25611d_register_reset(s);
    s->regs[9] |= BIT(7);
    bq25611d_irq(s);
}

static int bq25611d_send(I2CSlave *i2c, uint8_t data)
{
    BQ25611DState *s = BQ25611D(i2c);
    unsigned reg;

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
        return 0;
    }
    reg = s->pointer++;
    if (reg >= sizeof(s->regs)) {
        return 0;
    }
    if (reg == 11) {
        if (data & BIT(7)) {
            bq25611d_register_reset(s);
        }
    } else if (reg == 10) {
        s->regs[reg] = (s->regs[reg] & ~3) | (data & 3);
    } else if (reg != 8 && reg != 9) {
        s->regs[reg] = reg == 1 ? data & ~BIT(6) : data;
        if (reg == 5 || (reg == 1 && (data & BIT(6)))) {
            bq25611d_watchdog_arm(s);
        }
        bq25611d_update_status(s);
    }
    return 0;
}

static uint8_t bq25611d_recv(I2CSlave *i2c)
{
    BQ25611DState *s = BQ25611D(i2c);
    unsigned reg = s->pointer++;
    uint8_t value = reg < sizeof(s->regs) ? s->regs[reg] : 0;
    if (reg == 9) {
        s->regs[9] &= ~BIT(7); /* Read out the latched watchdog event. */
    }
    return value;
}

static int bq25611d_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        BQ25611D(i2c)->expect_pointer = true;
    }
    return 0;
}

static bool bq25611d_get_vbus(Object *obj, Error **errp)
{
    return BQ25611D(obj)->vbus;
}

static void bq25611d_set_vbus(Object *obj, bool value, Error **errp)
{
    BQ25611DState *s = BQ25611D(obj);
    if (s->vbus != value) {
        s->vbus = value;
        bq25611d_update_status(s);
        bq25611d_watchdog_arm(s);
        bq25611d_irq(s);
    }
}

static void bq25611d_reset(DeviceState *dev)
{
    BQ25611DState *s = BQ25611D(dev);
    s->pointer = 0;
    s->expect_pointer = true;
    bq25611d_register_reset(s);
    qemu_set_irq(s->nirq, 1);
}

static void bq25611d_init(Object *obj)
{
    BQ25611DState *s = BQ25611D(obj);
    s->watchdog = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                               bq25611d_watchdog_expire, s);
    qdev_init_gpio_out_named(DEVICE(obj), &s->nirq, "irq", 1);
    object_property_add_bool(obj, "vbus", bq25611d_get_vbus, bq25611d_set_vbus);
}

static void bq25611d_finalize(Object *obj)
{
    timer_free(BQ25611D(obj)->watchdog);
}

static const VMStateDescription bq25611d_vmstate = {
    .name = TYPE_BQ25611D, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, BQ25611DState),
        VMSTATE_UINT8_ARRAY(regs, BQ25611DState, 13),
        VMSTATE_UINT8(pointer, BQ25611DState),
        VMSTATE_BOOL(expect_pointer, BQ25611DState),
        VMSTATE_BOOL(vbus, BQ25611DState),
        VMSTATE_TIMER_PTR(watchdog, BQ25611DState),
        VMSTATE_END_OF_LIST()
    },
};

static void bq25611d_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    dc->vmsd = &bq25611d_vmstate;
    device_class_set_legacy_reset(dc, bq25611d_reset);
    sc->send = bq25611d_send;
    sc->recv = bq25611d_recv;
    sc->event = bq25611d_event;
}

static const TypeInfo bq25611d_info = {
    .name = TYPE_BQ25611D, .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BQ25611DState),
    .instance_init = bq25611d_init, .instance_finalize = bq25611d_finalize,
    .class_init = bq25611d_class_init,
};
static void bq25611d_register_types(void) { type_register_static(&bq25611d_info); }
type_init(bq25611d_register_types)
