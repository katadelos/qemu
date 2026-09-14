/*
 * LSM6DSO16IS primary I2C interface and sampled inertial data.
 * Contract: ST DS13892 Rev2, sections 5, 8-11, and the stock Lab126
 * drivers/iio/imu/st_lsm6dso16is driver. Virtual physical inputs are explicit.
 * ISPU program/data uploads are retained in separate real-sized RAM banks.
 * The proprietary processor is not executed: BOOT_END, algorithm outputs,
 * version and orientation/tap interrupts are never invented. Sensor-hub
 * transactions, analog filters, noise and self-test actuation are absent.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define TYPE_LSM6DSO16IS "lsm6dso16is"
OBJECT_DECLARE_SIMPLE_TYPE(LSM6DSO16ISState, LSM6DSO16IS)

typedef struct LSM6SampleTimer {
    LSM6DSO16ISState *owner;
    QEMUTimer *timer;
    unsigned channel;
} LSM6SampleTimer;

struct LSM6DSO16ISState {
    I2CSlave parent_obj;
    uint8_t main[128], ispu[128], shub[128];
    uint8_t program[32768], data[8192];
    uint16_t memory_address, read_mask;
    uint8_t pointer;
    bool expect_pointer;
    int32_t accel[3], gyro[3], temperature;
    qemu_irq irq[2];
    LSM6SampleTimer sample[3];
    uint64_t samples, program_bytes, data_bytes, memory_errors;
};

static void update_irq(LSM6DSO16ISState *s)
{
    unsigned ready = s->main[0x1e];
    bool irq1 = !!(s->main[0x0d] & ready & 3);
    bool irq2 = !!(s->main[0x0e] & ready & 7);
    bool low = s->main[0x12] & 0x20;
    if (s->main[0x13] & 0x20) { irq1 |= irq2; }
    qemu_set_irq(s->irq[0], irq1 ^ low);
    qemu_set_irq(s->irq[1], irq2 ^ low);
}

static int64_t period(LSM6DSO16ISState *s, unsigned channel)
{
    static const uint32_t mhz[] = {
        0, 12500, 26000, 52000, 104000, 208000,
        416000, 833000, 1667000, 3333000, 6667000,
    };
    unsigned odr = s->main[channel ? 0x11 : 0x10] >> 4;
    if (channel == 2) {
        return ((s->main[0x10] | s->main[0x11]) & 0xf0) ? 80000000 : 0;
    }
    if (channel == 1 && (s->main[0x13] & 0x40)) { return 0; }
    return odr && odr < ARRAY_SIZE(mhz) ? 1000000000000LL / mhz[odr] : 0;
}

static void reschedule(LSM6DSO16ISState *s)
{
    for (unsigned i = 0; i < 3; i++) {
        int64_t ns = period(s, i);
        if (ns) {
            timer_mod(s->sample[i].timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
        } else {
            timer_del(s->sample[i].timer);
            s->main[0x1e] &= ~(1 << i);
        }
    }
    update_irq(s);
}

static void sample(void *opaque)
{
    LSM6SampleTimer *t = opaque;
    LSM6DSO16ISState *s = t->owner;
    static const unsigned ug_per_lsb[] = {61, 488, 122, 244};
    static const unsigned udps_per_lsb[] = {
        8750, 4375, 17500, 4375, 35000, 4375, 70000, 4375,
    };
    unsigned start = t->channel == 0 ? 0x28 :
                     t->channel == 1 ? 0x22 : 0x20;
    unsigned words = t->channel == 2 ? 1 : 3;
    for (unsigned i = 0; i < words; i++) {
        unsigned address = start + i * 2;
        unsigned mask = 3U << (address - 0x20);
        int64_t raw;
        /* BDU retains each output word until both of its bytes are read. */
        if ((s->main[0x12] & 0x40) &&
            (s->read_mask & mask) != mask) { continue; }
        if (t->channel == 0) {
            raw = (int64_t)s->accel[i] /
                  ug_per_lsb[(s->main[0x10] >> 2) & 3];
        } else if (t->channel == 1) {
            raw = (int64_t)s->gyro[i] /
                  udps_per_lsb[(s->main[0x11] >> 1) & 7];
        } else {
            raw = ((int64_t)s->temperature - 25000) * 256 / 1000;
        }
        stw_le_p(&s->main[address], (int16_t)CLAMP(raw, -32768, 32767));
        s->read_mask &= ~mask;
    }
    s->main[0x1e] |= 1 << t->channel;
    s->samples++;
    update_irq(s);
    int64_t ns = period(s, t->channel);
    if (ns) {
        timer_mod(t->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
    }
}

static void register_reset(LSM6DSO16ISState *s)
{
    for (unsigned i = 0; i < 3; i++) { timer_del(s->sample[i].timer); }
    memset(s->main, 0, sizeof(s->main));
    memset(s->ispu, 0, sizeof(s->ispu));
    memset(s->shub, 0, sizeof(s->shub));
    s->main[2] = 0x3f;
    s->main[0x0f] = 0x22;
    s->main[0x12] = 4; /* IF_INC defaults enabled. */
    s->main[0x18] = 0xe0;
    s->read_mask = 0x3fff;
    s->memory_address = 0;
    update_irq(s);
}

static uint8_t *bank(LSM6DSO16ISState *s)
{
    return (s->main[1] & 0x80) ? s->ispu :
           (s->main[1] & 0x40) ? s->shub : s->main;
}

static uint8_t *memory_byte(LSM6DSO16ISState *s)
{
    bool program = s->ispu[8] & 1;
    if (s->memory_address >= (program ? sizeof(s->program) : sizeof(s->data))) {
        s->memory_errors++;
        return NULL;
    }
    return (program ? s->program : s->data) + s->memory_address;
}

static void advance_pointer(LSM6DSO16ISState *s)
{
    /* A burst at the ISPU memory port streams RAM, not adjacent registers. */
    if ((s->main[1] & 0x80) && s->pointer == 0x0b) {
        s->memory_address++;
        s->ispu[9] = s->memory_address >> 8;
        s->ispu[10] = s->memory_address;
    } else if (s->main[0x12] & 4) {
        s->pointer = (s->pointer + 1) & 0x7f;
    }
}

static int lsm6_send(I2CSlave *i2c, uint8_t value)
{
    LSM6DSO16ISState *s = LSM6DSO16IS(i2c);
    if (s->expect_pointer) {
        s->pointer = value & 0x7f;
        s->expect_pointer = false;
        return 0;
    }
    uint8_t *r = bank(s);
    unsigned reg = s->pointer;
    if (reg == 1) {
        if (value & 2) { memset(s->ispu, 0, sizeof(s->ispu)); }
        s->main[1] = value & 0xc0;
    } else if (r == s->ispu) {
        if (reg == 0x0b) {
            uint8_t *destination = memory_byte(s);
            if (!destination) { return -1; }
            *destination = value;
            if (s->ispu[8] & 1) { s->program_bytes++; }
            else { s->data_bytes++; }
        } else if (reg == 9 || reg == 10) {
            r[reg] = value;
            s->memory_address = (r[9] << 8) | r[10];
        } else if (reg == 0x0c || reg == 0x0d) {
            r[reg] |= value; /* IF2S flags are interface-set-only. */
        } else if (reg == 0x0e || reg == 0x0f) {
            r[reg] &= ~value; /* S2IF flags are interface-clear-only. */
        } else if (reg != 4 && !(reg >= 0x10 && reg <= 0x4f)) {
            r[reg] = value;
        }
    } else if (r == s->main) {
        if (reg == 0x12 && (value & 1)) {
            register_reset(s);
        } else if (reg != 0x0f && !(reg >= 0x1a && reg <= 0x2d)) {
            r[reg] = value;
            if (reg == 0x10 || reg == 0x11 || reg == 0x13) { reschedule(s); }
            update_irq(s);
        }
    } else {
        r[reg] = value; /* Retain sensor-hub configuration; no attached slaves. */
    }
    advance_pointer(s);
    return 0;
}

static uint8_t lsm6_recv(I2CSlave *i2c)
{
    LSM6DSO16ISState *s = LSM6DSO16IS(i2c);
    uint8_t *r = bank(s), value = r[s->pointer];
    unsigned reg = s->pointer;
    if (reg == 1) {
        /* FUNC_CFG_ACCESS remains visible in every bank. regmap uses its
         * readback to clear ISPU_REG_ACCESS after reading algorithm output. */
        value = s->main[1];
    } else if (r == s->ispu && reg == 0x0b) {
        uint8_t *source = memory_byte(s);
        value = source && (s->ispu[8] & 0x40) ? *source : 0;
    } else if (r == s->main && reg >= 0x20 && reg <= 0x2d) {
        s->read_mask |= 1 << (reg - 0x20);
        if ((s->read_mask & 3) == 3) { s->main[0x1e] &= ~4; }
        if ((s->read_mask & 0xfc) == 0xfc) { s->main[0x1e] &= ~2; }
        if ((s->read_mask & 0x3f00) == 0x3f00) { s->main[0x1e] &= ~1; }
        update_irq(s);
    }
    advance_pointer(s);
    return value;
}

static int event(I2CSlave *i2c, enum i2c_event ev)
{
    LSM6DSO16ISState *s = LSM6DSO16IS(i2c);
    if (s->main[0x13] & 4) { return -1; }
    if (ev == I2C_START_SEND) { s->expect_pointer = true; }
    update_irq(s);
    return 0;
}

static void reset(DeviceState *dev)
{
    LSM6DSO16ISState *s = LSM6DSO16IS(dev);
    register_reset(s);
    memset(s->program, 0, sizeof(s->program));
    memset(s->data, 0, sizeof(s->data));
    s->pointer = 0;
    s->expect_pointer = true;
    s->samples = s->program_bytes = s->data_bytes = s->memory_errors = 0;
}

static const Property properties[] = {
    DEFINE_PROP_INT32("accel-x-micro-g", LSM6DSO16ISState, accel[0], 0),
    DEFINE_PROP_INT32("accel-y-micro-g", LSM6DSO16ISState, accel[1], 0),
    DEFINE_PROP_INT32("accel-z-micro-g", LSM6DSO16ISState, accel[2], -1000000),
    DEFINE_PROP_INT32("gyro-x-micro-dps", LSM6DSO16ISState, gyro[0], 0),
    DEFINE_PROP_INT32("gyro-y-micro-dps", LSM6DSO16ISState, gyro[1], 0),
    DEFINE_PROP_INT32("gyro-z-micro-dps", LSM6DSO16ISState, gyro[2], 0),
    DEFINE_PROP_INT32("temperature-millicelsius", LSM6DSO16ISState, temperature, 25000),
};

static void init(Object *obj)
{
    LSM6DSO16ISState *s = LSM6DSO16IS(obj);
    qdev_init_gpio_out(DEVICE(obj), s->irq, 2);
    for (unsigned i = 0; i < 3; i++) {
        s->sample[i].owner = s;
        s->sample[i].channel = i;
        s->sample[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sample, &s->sample[i]);
    }
    object_property_add_uint64_ptr(obj, "samples", &s->samples, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "ispu-program-bytes", &s->program_bytes, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "ispu-data-bytes", &s->data_bytes, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "ispu-memory-errors", &s->memory_errors, OBJ_PROP_FLAG_READ);
}

static void finalize(Object *obj)
{
    LSM6DSO16ISState *s = LSM6DSO16IS(obj);
    for (unsigned i = 0; i < 3; i++) { timer_free(s->sample[i].timer); }
}

static void class_init(ObjectClass *klass, const void *data)
{
    I2CSlaveClass *i2c = I2C_SLAVE_CLASS(klass);
    i2c->send = lsm6_send;
    i2c->recv = lsm6_recv;
    i2c->event = event;
    device_class_set_props(DEVICE_CLASS(klass), properties);
    device_class_set_legacy_reset(DEVICE_CLASS(klass), reset);
}

static const TypeInfo info = {
    .name = TYPE_LSM6DSO16IS,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LSM6DSO16ISState),
    .instance_init = init,
    .instance_finalize = finalize,
    .class_init = class_init,
};

static void register_type(void) { type_register_static(&info); }
type_init(register_type)
