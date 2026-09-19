/*
 * Bosch BMA222E/BMA253 stationary accelerometer for Kindle Oasis.
 *
 * Models the register interface and a face-up, motionless device. There is
 * no motion source or FIFO producer; orientation stays fixed and motion
 * interrupts remain inactive. Samples are range-scaled and left-aligned in
 * the register pair for both the 8-bit BMA222E and 12-bit BMA253.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/bma2x2.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define BMA_RANGE       0x0f
#define BMA_BANDWIDTH   0x10
#define BMA_MODE        0x11
#define BMA_RESET       0x14
#define BMA_INT_OUTPUT  0x20
#define BMA_INT_LATCH   0x21

struct BMA2X2State {
    I2CSlave parent_obj;
    uint8_t regs[0x40];
    uint8_t pointer;
    uint8_t new_data;
    uint8_t chip_id;
    bool expect_pointer;
    int64_t sample_time_ns;
    qemu_irq irq;
};

static void bma2x2_update_irq(BMA2X2State *s)
{
    /* INT1's reset polarity is active-high; honor software inversion. */
    qemu_set_irq(s->irq, !(s->regs[BMA_INT_OUTPUT] & 1));
}

static void bma2x2_register_reset(BMA2X2State *s)
{
    static const uint8_t defaults[0x40] = {
        [0x00] = 0xf8, /* BMA222E chip ID */
        [0x0c] = 0x80, /* face-up, flat */
        [0x0f] = 0x03, [0x10] = 0x0f, [0x20] = 0x05,
        [0x22] = 0x09, [0x23] = 0x30, [0x24] = 0x81,
        [0x25] = 0x0f, [0x26] = 0xc0, [0x28] = 0x14,
        [0x29] = 0x14, [0x2a] = 0x04, [0x2b] = 0x0a,
        [0x2c] = 0x18, [0x2d] = 0x48, [0x2e] = 0x08,
        [0x2f] = 0x11, [0x31] = 0xff, [0x33] = 0xf4,
        [0x36] = 0x10, [0x3d] = 0xff,
    };

    memcpy(s->regs, defaults, sizeof(s->regs));
    s->regs[0] = s->chip_id;
    s->new_data = 7;
    s->sample_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bma2x2_update_irq(s);
}

static void bma2x2_sample(BMA2X2State *s)
{
    unsigned bw = s->regs[BMA_BANDWIDTH] & 0x1f;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t interval = 64000000 >> (CLAMP(bw, 8, 15) - 8);

    /* Normal and low-power modes sample; suspend/deep suspend retain data. */
    if (!(s->regs[BMA_MODE] & 0xa0) &&
        now - s->sample_time_ns >= interval) {
        s->sample_time_ns = now;
        s->new_data = 7;
    }
}

static int bma2x2_send(I2CSlave *i2c, uint8_t data)
{
    BMA2X2State *s = BMA2X2(i2c);
    uint8_t reg;

    if (s->expect_pointer) {
        s->pointer = data & 0x3f;
        s->expect_pointer = false;
        return 0;
    }
    reg = s->pointer;
    s->pointer = (s->pointer + 1) & 0x3f;
    if (reg < BMA_RANGE || reg == 0x3f) {
        return 0;
    }
    if (reg == BMA_RESET) {
        if (data == 0xb6) {
            uint8_t gp0 = s->regs[0x3b];
            uint8_t gp1 = s->regs[0x3c];

            bma2x2_register_reset(s);
            /* GP0/GP1 retain their application data across soft reset. */
            s->regs[0x3b] = gp0;
            s->regs[0x3c] = gp1;
        }
    } else if (reg == BMA_INT_LATCH) {
        /* reset_int is a write-only command; no motion is latched. */
        s->regs[reg] = data & 0x0f;
    } else {
        s->regs[reg] = data;
    }
    bma2x2_update_irq(s);
    return 0;
}

static uint8_t bma2x2_recv(I2CSlave *i2c)
{
    BMA2X2State *s = BMA2X2(i2c);
    uint8_t reg = s->pointer;
    unsigned axis;

    /* FIFO reads stay at the data port; the stationary FIFO remains empty. */
    if (reg != 0x3f) {
        s->pointer = (s->pointer + 1) & 0x3f;
    }
    bma2x2_sample(s);
    if (reg >= 2 && reg <= 7) {
        axis = (reg - 2) / 2;
        if (!(reg & 1)) {
            return (s->new_data >> axis) & 1;
        }
        s->new_data &= ~BIT(axis);
        if (axis != 2) {
            return 0;
        }
        /* +1 g along Z: high byte 64/32/16/8 at +/-2/4/8/16 g.
         * The low sample bits are zero, including on the 12-bit BMA253.
         */
        switch (s->regs[BMA_RANGE] & 0x0f) {
        case 0x05:
            return 32;
        case 0x08:
            return 16;
        case 0x0c:
            return 8;
        default:
            return 64;
        }
    }
    return s->regs[reg];
}

static int bma2x2_event(I2CSlave *i2c, enum i2c_event event)
{
    BMA2X2State *s = BMA2X2(i2c);

    bma2x2_update_irq(s);
    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    }
    return 0;
}

static void bma2x2_reset(DeviceState *dev)
{
    BMA2X2State *s = BMA2X2(dev);

    s->pointer = 0;
    s->expect_pointer = true;
    bma2x2_register_reset(s);
}

static int bma2x2_post_load(void *opaque, int version_id)
{
    bma2x2_update_irq(opaque);
    return 0;
}

static const VMStateDescription bma2x2_vmstate = {
    .name = TYPE_BMA2X2,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = bma2x2_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, BMA2X2State),
        VMSTATE_UINT8_ARRAY(regs, BMA2X2State, 0x40),
        VMSTATE_UINT8(pointer, BMA2X2State),
        VMSTATE_UINT8(new_data, BMA2X2State),
        VMSTATE_BOOL(expect_pointer, BMA2X2State),
        VMSTATE_INT64(sample_time_ns, BMA2X2State),
        VMSTATE_END_OF_LIST()
    },
};

static void bma2x2_init(Object *obj)
{
    BMA2X2State *s = BMA2X2(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
}

static void bma2x2_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    static const Property properties[] = {
        DEFINE_PROP_UINT8("chip-id", BMA2X2State, chip_id, 0xf8),
    };

    device_class_set_props(dc, properties);
    device_class_set_legacy_reset(dc, bma2x2_reset);
    dc->vmsd = &bma2x2_vmstate;
    sc->send = bma2x2_send;
    sc->recv = bma2x2_recv;
    sc->event = bma2x2_event;
}

static const TypeInfo bma2x2_type = {
    .name = TYPE_BMA2X2,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BMA2X2State),
    .instance_init = bma2x2_init,
    .class_init = bma2x2_class_init,
};

static void bma2x2_register_types(void)
{
    type_register_static(&bma2x2_type);
}
type_init(bma2x2_register_types)
