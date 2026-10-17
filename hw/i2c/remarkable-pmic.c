/*
 * PFUZE100, MAX17135 and BQ27441 fitted to reMarkable 1.
 * Register interfaces: reMarkable's zero-gravitas U-Boot and Linux BSP.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_RM_REGS "remarkable-i2c-registers"
OBJECT_DECLARE_SIMPLE_TYPE(RMRegsState, RM_REGS)

struct RMRegsState {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer, start;
    bool have_pointer;
    bool wakeup;
    qemu_irq pwrgood, irq;
};

static void rm_max17135_update(RMRegsState *s)
{
    bool enabled = (s->regs[9] & 1) || s->wakeup;
    s->regs[10] = enabled ? 0x80 : 0;
    qemu_set_irq(s->pwrgood, enabled);
    qemu_set_irq(s->irq, 1);
}

static int rm_regs_send(I2CSlave *i2c, uint8_t value)
{
    RMRegsState *s = RM_REGS(i2c);
    if (!s->have_pointer) {
        s->start = s->pointer = value;
        s->have_pointer = true;
    } else {
        uint8_t reg = s->pointer++;
        s->regs[reg] = value;
        if (object_dynamic_cast(OBJECT(s), "max17135")) {
            /* EEPROM programming completes synchronously; temperature is fixed. */
            s->regs[0x0c] = 0;
            rm_max17135_update(s);
        } else if (object_dynamic_cast(OBJECT(s), "bq27441") && reg == 1) {
            uint16_t command = lduw_le_p(s->regs);
            if (command == 1) {
                stw_le_p(s->regs, 0x0421); /* DEVICE_TYPE */
            } else if (command == 2) {
                stw_le_p(s->regs, 0x0109); /* FW_VERSION */
            } else if (command == 4) {
                stw_le_p(s->regs, 7); /* configured battery data-memory revision */
            } else if (command == 0x13) {
                s->regs[6] |= 0x10; /* CFGUPMODE */
            } else if (command == 0x42 || command == 0x41) {
                s->regs[6] &= ~0x10;
            } else if (command == 0) {
                stw_le_p(s->regs, 0x2000); /* sealed, initialized */
            }
        }
    }
    return 0;
}

static uint8_t rm_regs_recv(I2CSlave *i2c)
{
    RMRegsState *s = RM_REGS(i2c);
    return s->regs[s->pointer++];
}

static int rm_regs_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        RM_REGS(i2c)->have_pointer = false;
    }
    return 0;
}

static void rm_regs_reset(DeviceState *dev)
{
    RMRegsState *s = RM_REGS(dev);
    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = s->start = 0;
    s->have_pointer = false;
}

static void rm_pfuze_reset(DeviceState *dev)
{
    RMRegsState *s = RM_REGS(dev);
    rm_regs_reset(dev);
    s->regs[0] = 0x10;
    s->regs[3] = 0x11;
    s->regs[0x20] = s->regs[0x2e] = s->regs[0x35] = 0x2d;
}

static void rm_max17135_reset(DeviceState *dev)
{
    RMRegsState *s = RM_REGS(dev);
    rm_regs_reset(dev);
    s->wakeup = false;
    /* Temperature words are big endian, in half degrees Celsius. */
    s->regs[0] = s->regs[4] = 25;
    s->regs[7] = 0x4d;
    s->regs[8] = 180; /* 180 x 15 mV VCOM magnitude */
    rm_max17135_update(s);
}

static void rm_bq27441_reset(DeviceState *dev)
{
    RMRegsState *s = RM_REGS(dev);
    rm_regs_reset(dev);
    stw_le_p(s->regs, 0x2000);
    stw_le_p(s->regs + 2, 2981); /* 25 C, tenths Kelvin */
    stw_le_p(s->regs + 4, 4000); /* mV */
    stw_le_p(s->regs + 6, 0x0008); /* battery detected */
    stw_le_p(s->regs + 8, 3000); /* nominal available mAh */
    stw_le_p(s->regs + 10, 3000);
    stw_le_p(s->regs + 12, 2550); /* remaining mAh */
    stw_le_p(s->regs + 14, 3000); /* full mAh */
    stw_le_p(s->regs + 0x1c, 85); /* state of charge */
    stw_le_p(s->regs + 0x1e, 2981); /* internal temperature */
    stw_le_p(s->regs + 0x20, 100); /* state of health */
    s->regs[0x41] = 4; /* BATLOWEN already configured */
}

static int rm_regs_post_load(void *opaque, int version)
{
    RMRegsState *s = opaque;
    if (object_dynamic_cast(OBJECT(s), "max17135")) {
        rm_max17135_update(s);
    }
    return 0;
}

static const VMStateDescription rm_regs_vmstate = {
    .name = TYPE_RM_REGS,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rm_regs_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, RMRegsState),
        VMSTATE_UINT8_ARRAY(regs, RMRegsState, 256),
        VMSTATE_UINT8(pointer, RMRegsState),
        VMSTATE_UINT8(start, RMRegsState),
        VMSTATE_BOOL(have_pointer, RMRegsState),
        VMSTATE_BOOL(wakeup, RMRegsState),
        VMSTATE_END_OF_LIST()
    },
};

static void rm_regs_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);
    dc->vmsd = &rm_regs_vmstate;
    ic->send = rm_regs_send;
    ic->recv = rm_regs_recv;
    ic->event = rm_regs_event;
}

static void rm_pfuze_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), rm_pfuze_reset);
}

static void rm_max17135_wakeup(void *opaque, int line, int level)
{
    RMRegsState *s = opaque;
    s->wakeup = !!level;
    rm_max17135_update(s);
}

static void rm_max17135_init(Object *obj)
{
    RMRegsState *s = RM_REGS(obj);
    qdev_init_gpio_in_named(DEVICE(obj), rm_max17135_wakeup, "wakeup", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->pwrgood, "pwrgood", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
}

static void rm_max17135_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), rm_max17135_reset);
}

static void rm_bq27441_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), rm_bq27441_reset);
}

static const TypeInfo rm_pmic_types[] = {
    {
        .name = TYPE_RM_REGS,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(RMRegsState),
        .abstract = true,
        .class_init = rm_regs_class_init,
    }, {
        .name = "pfuze100",
        .parent = TYPE_RM_REGS,
        .class_init = rm_pfuze_class_init,
    }, {
        .name = "max17135",
        .parent = TYPE_RM_REGS,
        .instance_init = rm_max17135_init,
        .class_init = rm_max17135_class_init,
    }, {
        .name = "bq27441",
        .parent = TYPE_RM_REGS,
        .class_init = rm_bq27441_class_init,
    },
};
DEFINE_TYPES(rm_pmic_types)
