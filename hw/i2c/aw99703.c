/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Awinic AW99703 frontlight register interface.
 *
 * Prosecco U-Boot probes register 0 for ID 3 and programs the 11-bit
 * brightness through registers 6/7. The register defaults and writable
 * fields follow AW99703 datasheet V1.6, pp. 28-33:
 * https://doc.awinic.com/doc/202403/4635fa96-003b-4dd8-ad18-b47173a9429f.pdf
 * LED current, boost conversion and physical fault conditions are not modeled.
 */
#include "qemu/osdep.h"
#include "hw/i2c/aw99703.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define AW99703_REG_COUNT 0x10
#define AW99703_SFTRST 0x01
#define AW99703_RESET_TIME_NS (2 * SCALE_MS)
#define AW99703_STARTUP_TIME_NS (250 * SCALE_US)

struct AW99703State {
    I2CSlave parent_obj;
    uint8_t regs[AW99703_REG_COUNT];
    uint8_t pointer;
    bool expect_pointer;
    bool enabled;
    int64_t ready_ns;
};

static const uint8_t aw99703_defaults[AW99703_REG_COUNT] = {
    0x03, 0x00, 0x00, 0x9f, 0x2e, 0x00, 0x07, 0xff,
    0x93, 0x44, 0x00, 0x58, 0x80, 0x10, 0x00, 0x00,
};

static const uint8_t aw99703_writable[AW99703_REG_COUNT] = {
    0x00, 0x01, 0x17, 0xff, 0xff, 0xf8, 0x07, 0xff,
    0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x00, 0x00,
};

static void aw99703_reset(DeviceState *dev)
{
    AW99703State *s = AW99703(dev);

    memcpy(s->regs, aw99703_defaults, sizeof(s->regs));
    s->pointer = 0;
    s->expect_pointer = true;
    s->enabled = false;
    s->ready_ns = 0;
}

static void aw99703_enable(void *opaque, int line, int level)
{
    AW99703State *s = opaque;

    if (s->enabled == !!level) {
        return;
    }
    if (!level) {
        aw99703_reset(DEVICE(s));
    } else {
        s->enabled = true;
        s->ready_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      AW99703_STARTUP_TIME_NS;
    }
}

static int aw99703_send(I2CSlave *i2c, uint8_t data)
{
    AW99703State *s = AW99703(i2c);
    uint8_t reg, mask;

    if (!s->enabled || qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->ready_ns) {
        return -1;
    }
    if (s->expect_pointer) {
        if (data >= AW99703_REG_COUNT) {
            return -1;
        }
        s->pointer = data;
        s->expect_pointer = false;
        return 0;
    }
    reg = s->pointer++;
    if (reg >= AW99703_REG_COUNT) {
        return -1;
    }
    if (reg == AW99703_SFTRST && (data & 1)) {
        memcpy(s->regs, aw99703_defaults, sizeof(s->regs));
        s->ready_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      AW99703_RESET_TIME_NS;
        return 0;
    }
    mask = aw99703_writable[reg];
    s->regs[reg] = (s->regs[reg] & ~mask) | (data & mask);
    return 0;
}

static uint8_t aw99703_recv(I2CSlave *i2c)
{
    AW99703State *s = AW99703(i2c);
    unsigned reg = s->pointer++;

    return reg < AW99703_REG_COUNT ? s->regs[reg] : 0xff;
}

static int aw99703_event(I2CSlave *i2c, enum i2c_event event)
{
    AW99703State *s = AW99703(i2c);

    if (event == I2C_START_SEND || event == I2C_START_RECV) {
        if (!s->enabled || qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->ready_ns) {
            return -1;
        }
        if (event == I2C_START_SEND) {
            s->expect_pointer = true;
        }
    }
    return 0;
}

static const VMStateDescription aw99703_vmstate = {
    .name = TYPE_AW99703,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, AW99703State),
        VMSTATE_UINT8_ARRAY(regs, AW99703State, AW99703_REG_COUNT),
        VMSTATE_UINT8(pointer, AW99703State),
        VMSTATE_BOOL(expect_pointer, AW99703State),
        VMSTATE_BOOL(enabled, AW99703State),
        VMSTATE_INT64(ready_ns, AW99703State),
        VMSTATE_END_OF_LIST()
    },
};

static void aw99703_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->desc = "Awinic AW99703 frontlight controller";
    device_class_set_legacy_reset(dc, aw99703_reset);
    dc->vmsd = &aw99703_vmstate;
    sc->send = aw99703_send;
    sc->recv = aw99703_recv;
    sc->event = aw99703_event;
}

static void aw99703_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), aw99703_enable, "enable", 1);
}

static const TypeInfo aw99703_info = {
    .name = TYPE_AW99703,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(AW99703State),
    .instance_init = aw99703_init,
    .class_init = aw99703_class_init,
};

static void aw99703_register_types(void)
{
    type_register_static(&aw99703_info);
}
type_init(aw99703_register_types)
