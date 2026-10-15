/*
 * Kionix KX122 digital acceleration interface.
 * The stock kx122_registers.h defines this map, ODR, range, FIFO,
 * command-test and interrupt behavior. Samples represent explicit static
 * acceleration inputs, not MEMS noise, analog self-test or tap physics.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"

#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define TYPE_KX122 "kx122"
OBJECT_DECLARE_SIMPLE_TYPE(KX122State, KX122)

#define KX122_FIFO_SIZE 2048

struct KX122State {
    I2CSlave parent_obj;
    uint8_t regs[0x61];
    uint8_t pointer;
    bool expect_pointer;
    int32_t acceleration[3]; /* milligravity, physical sensor axes */
    uint8_t fifo[KX122_FIFO_SIZE];
    uint16_t fifo_head, fifo_count;
    uint64_t samples;
    qemu_irq irq[2];
    QEMUTimer *sample_timer;
};

static void kx122_update_irq(KX122State *s)
{
    bool any = false;
    unsigned size = s->regs[0x3b] & BIT(6) ? 6 : 3;
    unsigned capacity = (size == 6 ? 339 : 681) * size;
    unsigned watermark = MAX(2, s->regs[0x3a]) * size;
    s->regs[0x13] &= ~(BIT(5) | BIT(6));
    if (s->regs[0x3b] & BIT(7)) {
        if (s->fifo_count >= watermark) { s->regs[0x13] |= BIT(5); }
        if (s->fifo_count >= capacity && (s->regs[0x3b] & BIT(5))) {
            s->regs[0x13] |= BIT(6);
        }
    }
    for (unsigned i = 0; i < 2; i++) {
        uint8_t control = s->regs[i ? 0x20 : 0x1c];
        bool pending = (control & BIT(5)) &&
            (s->regs[0x13] & s->regs[i ? 0x21 : 0x1f]);
        bool active_high = control & BIT(4);
        qemu_set_irq(s->irq[i], pending == active_high);
        any |= pending;
    }
    s->regs[0x15] = (s->regs[0x15] & ~BIT(4)) | (any ? BIT(4) : 0);
}

static void kx122_schedule(KX122State *s)
{
    if (s->regs[0x18] & BIT(7)) {
        unsigned odr = s->regs[0x1b] & 15;
        /* OSA0..7: 12.5..1600 Hz; 8..11: 0.78125..6.25 Hz. */
        int64_t ns = odr < 8 ? 80000000LL >> odr :
                     odr < 12 ? 1280000000LL >> (odr - 8) :
                                312500LL >> (odr - 12);
        timer_mod(s->sample_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
    } else {
        timer_del(s->sample_timer);
    }
}

static void kx122_sample(void *opaque)
{
    KX122State *s = opaque;
    unsigned range = 2U << ((s->regs[0x18] >> 3) & 3);
    uint8_t sample[6];
    unsigned largest = 0;

    if (!(s->regs[0x18] & BIT(7))) { return; }
    for (unsigned axis = 0; axis < 3; axis++) {
        int64_t raw = (int64_t)s->acceleration[axis] * 32768 / (1000 * range);
        stw_le_p(sample + axis * 2, (int16_t)CLAMP(raw, INT16_MIN, INT16_MAX));
        if (llabs(s->acceleration[axis]) > llabs(s->acceleration[largest])) {
            largest = axis;
        }
    }
    memcpy(s->regs + 6, sample, sizeof(sample));
    s->samples++;
    if (s->regs[0x18] & BIT(5)) { s->regs[0x13] |= BIT(4); }

    /* A static pose has one dominant gravity axis. Physical tilt filtering
     * and arbitrary moving poses are outside this numerical input model. */
    if ((s->regs[0x18] & 1) && s->acceleration[largest]) {
        uint8_t pose = BIT((2 - largest) * 2 + (s->acceleration[largest] < 0));
        if (pose != s->regs[0x10] && (pose & s->regs[0x19])) {
            s->regs[0x11] = s->regs[0x10];
            s->regs[0x10] = pose;
            s->regs[0x13] |= 1;
        }
    }

    if ((s->regs[0x3b] & BIT(7)) && (s->regs[0x3b] & 3) < 2) {
        unsigned bytes = s->regs[0x3b] & BIT(6) ? 6 : 3;
        unsigned capacity = (bytes == 6 ? 339 : 681) * bytes;
        if (s->fifo_count + bytes > capacity && (s->regs[0x3b] & 1)) {
            s->fifo_head = (s->fifo_head + bytes) % KX122_FIFO_SIZE;
            s->fifo_count -= bytes;
        }
        if (s->fifo_count + bytes <= capacity) {
            for (unsigned i = 0; i < bytes; i++) {
                s->fifo[(s->fifo_head + s->fifo_count++) % KX122_FIFO_SIZE] =
                    sample[bytes == 6 ? i : i * 2 + 1];
            }
        }
    }
    kx122_update_irq(s);
    kx122_schedule(s);
}

static void kx122_reset(DeviceState *dev)
{
    KX122State *s = KX122(dev);
    timer_del(s->sample_timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x0c] = 0x55;
    s->regs[0x0f] = 0x1b;
    s->regs[0x19] = 0x3f; /* All six tilt directions enabled. */
    s->pointer = 0;
    s->expect_pointer = true;
    s->fifo_head = s->fifo_count = 0;
    s->samples = 0;
    kx122_update_irq(s);
}

static int kx122_send(I2CSlave *i2c, uint8_t data)
{
    KX122State *s = KX122(i2c);
    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else {
        unsigned reg = s->pointer++;
        if (reg == 0x19 && (data & BIT(7))) {
            kx122_reset(DEVICE(s));
            s->expect_pointer = false;
        } else if (reg == 0x3e) {
            s->fifo_head = s->fifo_count = 0;
        } else if (reg >= 0x18 && reg < sizeof(s->regs) &&
                   (reg < 0x3c || reg > 0x3f)) {
            if (reg == 0x3b && ((s->regs[reg] ^ data) & 0xc3)) {
                s->fifo_head = s->fifo_count = 0;
            }
            s->regs[reg] = data;
            if (reg == 0x19 && (data & BIT(6))) { s->regs[0x0c] = 0xaa; }
            if (reg == 0x3b && (data & BIT(7)) && (data & 3) >= 2) {
                qemu_log_mask(LOG_UNIMP, "kx122: trigger FIFO mode unsupported\n");
            }
        }
        kx122_update_irq(s);
        kx122_schedule(s);
    }
    return 0;
}

static uint8_t kx122_recv(I2CSlave *i2c)
{
    KX122State *s = KX122(i2c);
    unsigned reg = s->pointer;
    uint8_t value = reg < sizeof(s->regs) ? s->regs[reg] : 0;
    if (reg == 0x3f) {
        if (s->fifo_count) {
            value = s->fifo[s->fifo_head++];
            s->fifo_head %= KX122_FIFO_SIZE;
            s->fifo_count--;
        }
    } else {
        s->pointer++;
        if (reg == 0x3c || reg == 0x3d) { value = s->fifo_count >> ((reg - 0x3c) * 8); }
        if (reg == 0x0c) { s->regs[0x0c] = 0x55; s->regs[0x19] &= ~BIT(6); }
        if (reg == 0x0b) { s->regs[0x13] &= ~BIT(4); }
        if (reg == 0x17) { memset(s->regs + 0x12, 0, 3); }
    }
    kx122_update_irq(s);
    return value;
}

static int kx122_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) { KX122(i2c)->expect_pointer = true; }
    return 0;
}

static void kx122_init(Object *obj)
{
    KX122State *s = KX122(obj);
    qdev_init_gpio_out_named(DEVICE(obj), s->irq, "irq", 2);
    s->sample_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, kx122_sample, s);
    object_property_add_uint64_ptr(obj, "samples", &s->samples, OBJ_PROP_FLAG_READ);
}

static void kx122_finalize(Object *obj) { timer_free(KX122(obj)->sample_timer); }

static const Property kx122_properties[] = {
    DEFINE_PROP_INT32("x-mg", KX122State, acceleration[0], 0),
    DEFINE_PROP_INT32("y-mg", KX122State, acceleration[1], 0),
    DEFINE_PROP_INT32("z-mg", KX122State, acceleration[2], 1000),
};

static const VMStateDescription kx122_vmstate = {
    .name = TYPE_KX122, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, KX122State),
        VMSTATE_UINT8_ARRAY(regs, KX122State, 0x61),
        VMSTATE_UINT8(pointer, KX122State),
        VMSTATE_BOOL(expect_pointer, KX122State),
        VMSTATE_UINT8_ARRAY(fifo, KX122State, KX122_FIFO_SIZE),
        VMSTATE_UINT16(fifo_head, KX122State),
        VMSTATE_UINT16(fifo_count, KX122State),
        VMSTATE_UINT64(samples, KX122State),
        VMSTATE_TIMER_PTR(sample_timer, KX122State),
        VMSTATE_END_OF_LIST()
    },
};

static void kx122_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    dc->vmsd = &kx122_vmstate;
    device_class_set_props(dc, kx122_properties);
    device_class_set_legacy_reset(dc, kx122_reset);
    sc->send = kx122_send; sc->recv = kx122_recv; sc->event = kx122_event;
}
static const TypeInfo kx122_info = {
    .name = TYPE_KX122, .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(KX122State), .instance_init = kx122_init,
    .instance_finalize = kx122_finalize, .class_init = kx122_class_init,
};
static void kx122_register_types(void) { type_register_static(&kx122_info); }
type_init(kx122_register_types)
