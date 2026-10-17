/*
 * Freescale MMA7660FC stationary accelerometer.
 *
 * Implements the register interface, active/standby sampling, orientation
 * interrupts and test-mode axis writes. Physical acceleration stays at
 * X=Y=0, Z=+1g; tap, shake and automatic sleep/wake are not produced.
 * Register definitions follow the MMA7660FC data sheet, Rev. 8.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/i2c/mma7660.h"
#include "migration/vmstate.h"
#include "qemu/bitops.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define MMA_TILT     0x03
#define MMA_SRST     0x04
#define MMA_INTSU    0x06
#define MMA_MODE     0x07
#define MMA_SR       0x08
#define MMA_PD       0x0a
#define MMA_ACTIVE   BIT(0)
#define MMA_TEST     BIT(2)
#define MMA_IAH      BIT(7)

struct MMA7660State {
    I2CSlave parent_obj;
    uint8_t regs[32];
    uint8_t pointer;
    uint8_t tilt_count;
    bool expect_pointer;
    bool pending;
    QEMUTimer *sample_timer;
    qemu_irq irq;
};

static void mma7660_update_irq(MMA7660State *s)
{
    bool active_high = s->regs[MMA_MODE] & MMA_IAH;

    qemu_set_irq(s->irq, active_high ? s->pending : !s->pending);
}

static int64_t mma7660_sample_period(MMA7660State *s)
{
    static const unsigned rates[] = { 120, 64, 32, 16, 8, 4, 2, 1 };

    return NANOSECONDS_PER_SECOND / rates[s->regs[MMA_SR] & 7];
}

static void mma7660_update_tilt(MMA7660State *s)
{
    int x = sextract32(s->regs[0], 0, 6);
    int y = sextract32(s->regs[1], 0, 6);
    int z = sextract32(s->regs[2], 0, 6);
    uint8_t old = s->regs[MMA_TILT];
    uint8_t tilt = z > 5 ? 1 : z < -5 ? 2 : 0;

    /* 21 counts/g: portrait/landscape is valid below 0.8g on Z. */
    if (abs(z) < 17) {
        if (abs(x) > abs(y)) {
            tilt |= (x < 0 ? 6 : 5) << 2;
        } else if (abs(y) > abs(x)) {
            tilt |= (y < 0 ? 2 : 1) << 2;
        }
    }
    s->regs[MMA_TILT] = tilt;
    if ((((old ^ tilt) & 3) && (s->regs[MMA_INTSU] & BIT(0))) ||
        (((old ^ tilt) & 0x1c) && (s->regs[MMA_INTSU] & BIT(1)))) {
        s->pending = true;
    }
    mma7660_update_irq(s);
}

static void mma7660_sample(void *opaque)
{
    MMA7660State *s = opaque;

    if (!(s->regs[MMA_MODE] & MMA_ACTIVE)) {
        return;
    }
    s->regs[0] = 0;
    s->regs[1] = 0;
    s->regs[2] = 21;
    s->regs[MMA_SRST] = 1;
    if (++s->tilt_count >= MAX(s->regs[MMA_SR] >> 5, 1)) {
        mma7660_update_tilt(s);
        s->tilt_count = 0;
    }
    if (s->regs[MMA_INTSU] & BIT(4)) {
        s->pending = true;
    }
    mma7660_update_irq(s);
    timer_mod(s->sample_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                              mma7660_sample_period(s));
}

static int mma7660_send(I2CSlave *i2c, uint8_t data)
{
    MMA7660State *s = MMA7660(i2c);
    uint8_t reg;

    if (s->expect_pointer) {
        s->pointer = data & 0x1f;
        s->expect_pointer = false;
        return 0;
    }
    reg = s->pointer;
    s->pointer = (reg + 1) & 0x1f;
    if (reg == MMA_MODE) {
        timer_del(s->sample_timer);
        s->regs[reg] = data & ~BIT(1);
        if (data & MMA_ACTIVE) {
            s->regs[reg] &= ~MMA_TEST;
            timer_mod(s->sample_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      mma7660_sample_period(s));
        }
        memset(s->regs, 0, MMA_SRST + 1);
        s->tilt_count = 0;
    } else if ((s->regs[MMA_MODE] & MMA_TEST) && reg < MMA_TILT) {
        s->regs[reg] = data & 0x3f;
        mma7660_update_tilt(s);
    } else if (!(s->regs[MMA_MODE] & MMA_ACTIVE) &&
               reg > MMA_SRST && reg <= MMA_PD) {
        s->regs[reg] = data;
    }
    mma7660_update_irq(s);
    return 0;
}

static uint8_t mma7660_recv(I2CSlave *i2c)
{
    MMA7660State *s = MMA7660(i2c);
    uint8_t data = s->regs[s->pointer];

    s->pointer = (s->pointer + 1) & 0x1f;
    return data;
}

static int mma7660_event(I2CSlave *i2c, enum i2c_event event)
{
    MMA7660State *s = MMA7660(i2c);

    if (event == I2C_START_SEND || event == I2C_START_RECV) {
        /* INT is acknowledged by the next slave-address transfer. */
        s->pending = false;
        mma7660_update_irq(s);
    }
    if (event == I2C_START_SEND) {
        s->expect_pointer = true;
    } else if (event == I2C_FINISH) {
        /* STOP resets the address pointer; repeated START preserves it. */
        s->pointer = 0;
    }
    return 0;
}

static void mma7660_reset(DeviceState *dev)
{
    MMA7660State *s = MMA7660(dev);

    timer_del(s->sample_timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->tilt_count = 0;
    s->expect_pointer = true;
    s->pending = false;
    mma7660_update_irq(s);
}

static int mma7660_post_load(void *opaque, int version_id)
{
    MMA7660State *s = opaque;

    if (s->pointer >= ARRAY_SIZE(s->regs)) {
        return -EINVAL;
    }
    mma7660_update_irq(s);
    return 0;
}

static const VMStateDescription mma7660_vmstate = {
    .name = TYPE_MMA7660,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = mma7660_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MMA7660State),
        VMSTATE_UINT8_ARRAY(regs, MMA7660State, 32),
        VMSTATE_UINT8(pointer, MMA7660State),
        VMSTATE_UINT8(tilt_count, MMA7660State),
        VMSTATE_BOOL(expect_pointer, MMA7660State),
        VMSTATE_BOOL(pending, MMA7660State),
        VMSTATE_TIMER_PTR(sample_timer, MMA7660State),
        VMSTATE_END_OF_LIST()
    },
};

static void mma7660_init(Object *obj)
{
    MMA7660State *s = MMA7660(obj);

    s->sample_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mma7660_sample, s);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
}

static void mma7660_finalize(Object *obj)
{
    timer_free(MMA7660(obj)->sample_timer);
}

static void mma7660_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, mma7660_reset);
    dc->vmsd = &mma7660_vmstate;
    sc->send = mma7660_send;
    sc->recv = mma7660_recv;
    sc->event = mma7660_event;
}

static const TypeInfo mma7660_type = {
    .name = TYPE_MMA7660,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MMA7660State),
    .instance_init = mma7660_init,
    .instance_finalize = mma7660_finalize,
    .class_init = mma7660_class_init,
};

static void mma7660_register_types(void)
{
    type_register_static(&mma7660_type);
}

type_init(mma7660_register_types)
