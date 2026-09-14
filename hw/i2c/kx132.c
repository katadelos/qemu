/*
 * Kionix KX132-1211 digital acceleration interface.
 * The stock kx132_1211_registers.h defines this map, ODR, range, FIFO,
 * command-test and interrupt behavior. Samples represent explicit static
 * acceleration inputs, not MEMS noise, analog self-test or tap physics.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/kx132.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define KX132_FIFO_SIZE 2048

struct KX132State {
    I2CSlave parent_obj;
    uint8_t regs[0x77];
    uint8_t pointer;
    bool expect_pointer;
    int32_t acceleration[3]; /* milligravity, physical sensor axes */
    uint8_t fifo[KX132_FIFO_SIZE];
    uint16_t fifo_head, fifo_count;
    uint64_t samples;
    qemu_irq irq[2];
    QEMUTimer *sample_timer;
};

static void kx132_update_irq(KX132State *s)
{
    bool any = false;
    unsigned size = s->regs[0x5f] & BIT(6) ? 6 : 3;
    unsigned capacity = (size == 6 ? 339 : 681) * size;
    unsigned watermark = MAX(2, s->regs[0x5e]) * size;
    s->regs[0x17] &= ~(BIT(5) | BIT(6));
    if (s->regs[0x5f] & BIT(7)) {
        if (s->fifo_count >= watermark) { s->regs[0x17] |= BIT(5); }
        if (s->fifo_count >= capacity && (s->regs[0x5f] & BIT(5))) {
            s->regs[0x17] |= BIT(6);
        }
    }
    for (unsigned i = 0; i < 2; i++) {
        uint8_t control = s->regs[i ? 0x26 : 0x22];
        bool pending = (control & BIT(5)) &&
            (s->regs[0x17] & s->regs[i ? 0x27 : 0x25]);
        bool active_high = control & BIT(4);
        qemu_set_irq(s->irq[i], pending == active_high);
        any |= pending;
    }
    s->regs[0x19] = (s->regs[0x19] & ~BIT(4)) | (any ? BIT(4) : 0);
}

static void kx132_schedule(KX132State *s)
{
    if (s->regs[0x1b] & BIT(7)) {
        /* 0.78125 Hz at OSA0, doubling through 25.6 kHz at OSA15. */
        int64_t ns = 1280000000LL >> (s->regs[0x21] & 15);
        timer_mod(s->sample_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
    } else {
        timer_del(s->sample_timer);
    }
}

static void kx132_sample(void *opaque)
{
    KX132State *s = opaque;
    unsigned range = 2U << ((s->regs[0x1b] >> 3) & 3);
    uint8_t sample[6];
    unsigned largest = 0;

    if (!(s->regs[0x1b] & BIT(7))) { return; }
    for (unsigned axis = 0; axis < 3; axis++) {
        int64_t raw = (int64_t)s->acceleration[axis] * 32768 / (1000 * range);
        stw_le_p(sample + axis * 2, (int16_t)CLAMP(raw, INT16_MIN, INT16_MAX));
        if (llabs(s->acceleration[axis]) > llabs(s->acceleration[largest])) {
            largest = axis;
        }
    }
    memcpy(s->regs + 8, sample, sizeof(sample));
    s->samples++;
    if (s->regs[0x1b] & BIT(5)) { s->regs[0x17] |= BIT(4); }

    /* A static pose has one dominant gravity axis. Physical tilt filtering
     * and arbitrary moving poses are outside this numerical input model. */
    if ((s->regs[0x1b] & 1) && s->acceleration[largest]) {
        uint8_t pose = BIT((2 - largest) * 2 + (s->acceleration[largest] < 0));
        if (pose != s->regs[0x14] && (pose & s->regs[0x1c])) {
            s->regs[0x15] = s->regs[0x14];
            s->regs[0x14] = pose;
            s->regs[0x17] |= 1;
        }
    }

    if ((s->regs[0x5f] & BIT(7)) && (s->regs[0x5f] & 3) < 2) {
        unsigned bytes = s->regs[0x5f] & BIT(6) ? 6 : 3;
        unsigned capacity = (bytes == 6 ? 339 : 681) * bytes;
        if (s->fifo_count + bytes > capacity && (s->regs[0x5f] & 1)) {
            s->fifo_head = (s->fifo_head + bytes) % KX132_FIFO_SIZE;
            s->fifo_count -= bytes;
        }
        if (s->fifo_count + bytes <= capacity) {
            for (unsigned i = 0; i < bytes; i++) {
                s->fifo[(s->fifo_head + s->fifo_count++) % KX132_FIFO_SIZE] =
                    sample[bytes == 6 ? i : i * 2 + 1];
            }
        }
    }
    kx132_update_irq(s);
    kx132_schedule(s);
}

static void kx132_reset(DeviceState *dev)
{
    KX132State *s = KX132(dev);
    timer_del(s->sample_timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x12] = 0x55;
    s->regs[0x13] = 0x3d;
    s->pointer = 0;
    s->expect_pointer = true;
    s->fifo_head = s->fifo_count = 0;
    s->samples = 0;
    kx132_update_irq(s);
}

static int kx132_send(I2CSlave *i2c, uint8_t data)
{
    KX132State *s = KX132(i2c);
    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else {
        unsigned reg = s->pointer++;
        if (reg == 0x1c && (data & BIT(7))) {
            kx132_reset(DEVICE(s));
            s->expect_pointer = false;
        } else if (reg == 0x62) {
            s->fifo_head = s->fifo_count = 0;
        } else if (reg >= 0x1b && reg < sizeof(s->regs) &&
                   (reg < 0x60 || reg > 0x63)) {
            if (reg == 0x5f && ((s->regs[reg] ^ data) & 0xc3)) {
                s->fifo_head = s->fifo_count = 0;
            }
            s->regs[reg] = data;
            if (reg == 0x1c && (data & BIT(6))) { s->regs[0x12] = 0xaa; }
            if (reg == 0x5f && (data & BIT(7)) && (data & 3) >= 2) {
                qemu_log_mask(LOG_UNIMP, "kx132: trigger FIFO mode unsupported\n");
            }
        }
        kx132_update_irq(s);
        kx132_schedule(s);
    }
    return 0;
}

static uint8_t kx132_recv(I2CSlave *i2c)
{
    KX132State *s = KX132(i2c);
    unsigned reg = s->pointer;
    uint8_t value = reg < sizeof(s->regs) ? s->regs[reg] : 0;
    if (reg == 0x63) {
        if (s->fifo_count) {
            value = s->fifo[s->fifo_head++];
            s->fifo_head %= KX132_FIFO_SIZE;
            s->fifo_count--;
        }
    } else {
        s->pointer++;
        if (reg == 0x60 || reg == 0x61) { value = s->fifo_count >> ((reg - 0x60) * 8); }
        if (reg == 0x12) { s->regs[0x12] = 0x55; s->regs[0x1c] &= ~BIT(6); }
        if (reg == 0x0d) { s->regs[0x17] &= ~BIT(4); }
        if (reg == 0x1a) { memset(s->regs + 0x16, 0, 3); }
    }
    kx132_update_irq(s);
    return value;
}

static int kx132_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) { KX132(i2c)->expect_pointer = true; }
    return 0;
}

static void kx132_init(Object *obj)
{
    KX132State *s = KX132(obj);
    qdev_init_gpio_out_named(DEVICE(obj), s->irq, "irq", 2);
    s->sample_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, kx132_sample, s);
    object_property_add_uint64_ptr(obj, "samples", &s->samples, OBJ_PROP_FLAG_READ);
}

static void kx132_finalize(Object *obj) { timer_free(KX132(obj)->sample_timer); }

static const Property kx132_properties[] = {
    DEFINE_PROP_INT32("x-mg", KX132State, acceleration[0], 0),
    DEFINE_PROP_INT32("y-mg", KX132State, acceleration[1], 0),
    DEFINE_PROP_INT32("z-mg", KX132State, acceleration[2], 1000),
};

static const VMStateDescription kx132_vmstate = {
    .name = TYPE_KX132, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, KX132State),
        VMSTATE_UINT8_ARRAY(regs, KX132State, 0x77),
        VMSTATE_UINT8(pointer, KX132State),
        VMSTATE_BOOL(expect_pointer, KX132State),
        VMSTATE_UINT8_ARRAY(fifo, KX132State, KX132_FIFO_SIZE),
        VMSTATE_UINT16(fifo_head, KX132State),
        VMSTATE_UINT16(fifo_count, KX132State),
        VMSTATE_UINT64(samples, KX132State),
        VMSTATE_TIMER_PTR(sample_timer, KX132State),
        VMSTATE_END_OF_LIST()
    },
};

static void kx132_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    dc->vmsd = &kx132_vmstate;
    device_class_set_props(dc, kx132_properties);
    device_class_set_legacy_reset(dc, kx132_reset);
    sc->send = kx132_send; sc->recv = kx132_recv; sc->event = kx132_event;
}
static const TypeInfo kx132_info = {
    .name = TYPE_KX132, .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(KX132State), .instance_init = kx132_init,
    .instance_finalize = kx132_finalize, .class_init = kx132_class_init,
};
static void kx132_register_types(void) { type_register_static(&kx132_info); }
type_init(kx132_register_types)
