/*
 * MT8113 pinctrl/GPIO register block
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/gpio/mt8113_gpio.h"
#include "hw/core/irq.h"
#include "qemu/bitops.h"
#include "qemu/module.h"

#define MT8113_EINT_PORTS       6
#define MT8113_EINT_STAT        0x000
#define MT8113_EINT_ACK         0x040
#define MT8113_EINT_MASK        0x080
#define MT8113_EINT_MASK_SET    0x0c0
#define MT8113_EINT_MASK_CLR    0x100
#define MT8113_EINT_SENS        0x140
#define MT8113_EINT_SENS_SET    0x180
#define MT8113_EINT_SENS_CLR    0x1c0
#define MT8113_EINT_SOFT        0x200
#define MT8113_EINT_SOFT_SET    0x240
#define MT8113_EINT_SOFT_CLR    0x280
#define MT8113_EINT_POL         0x300
#define MT8113_EINT_POL_SET     0x340
#define MT8113_EINT_POL_CLR     0x380
#define MT8113_EINT_DOM_EN      0x400
#define MT8113_GPIO_DOUT        0x0a0
#define MT8113_GPIO_PORT_STRIDE 0x010

static bool mt8113_gpio_is_din(hwaddr offset)
{
    return offset <= 0x30 && !(offset & 0xf);
}

static uint64_t mt8113_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8113GPIOState *s = opaque;

    if (mt8113_gpio_is_din(offset)) {
        return s->input_levels[offset / 0x10];
    }
    return s->regs[offset / sizeof(uint32_t)];
}

static void mt8113_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    MT8113GPIOState *s = opaque;
    hwaddr alias = offset & 0xf;
    hwaddr base = offset - alias;
    uint32_t *reg = &s->regs[base / sizeof(uint32_t)];

    if (mt8113_gpio_is_din(base)) {
        return;
    }
    if (alias == 4) {
        *reg |= value;
    } else if (alias == 8) {
        *reg &= ~value;
    } else {
        s->regs[offset / sizeof(uint32_t)] = value;
    }

    if (base >= MT8113_GPIO_DOUT &&
        base < MT8113_GPIO_DOUT +
               DIV_ROUND_UP(MT8113_GPIO_NUM_PINS, 32) *
               MT8113_GPIO_PORT_STRIDE) {
        unsigned port = (base - MT8113_GPIO_DOUT) /
                        MT8113_GPIO_PORT_STRIDE;

        for (unsigned bit = 0; bit < 32; bit++) {
            unsigned pin = port * 32 + bit;

            if (pin < MT8113_GPIO_NUM_PINS) {
                qemu_set_irq(s->output[pin], !!(*reg & BIT(bit)));
            }
        }
    }
}

static const MemoryRegionOps mt8113_gpio_ops = {
    .read = mt8113_gpio_read,
    .write = mt8113_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static bool mt8113_eint_port(hwaddr offset, hwaddr base, unsigned *port)
{
    if (offset < base || offset >= base + MT8113_EINT_PORTS * 4) {
        return false;
    }
    *port = (offset - base) / 4;
    return true;
}

static uint32_t *mt8113_eint_reg(MT8113GPIOState *s, hwaddr base,
                                 unsigned port)
{
    return &s->eint_regs[(base + port * 4) / sizeof(uint32_t)];
}

static bool mt8113_eint_level_active(MT8113GPIOState *s, unsigned pin)
{
    unsigned port = pin / 32;
    uint32_t bit = BIT(pin % 32);
    bool level = s->input_levels[port] & bit;
    bool polarity = *mt8113_eint_reg(s, MT8113_EINT_POL, port) & bit;

    return level == polarity;
}

static void mt8113_eint_update_irq(MT8113GPIOState *s)
{
    bool pending = false;

    for (unsigned port = 0; port < MT8113_EINT_PORTS; port++) {
        uint32_t status = *mt8113_eint_reg(s, MT8113_EINT_STAT, port);
        uint32_t mask = *mt8113_eint_reg(s, MT8113_EINT_MASK, port);
        uint32_t domain = *mt8113_eint_reg(s, MT8113_EINT_DOM_EN, port);

        pending |= status & ~mask & domain;
    }
    qemu_set_irq(s->irq, pending);
}

static void mt8113_eint_latch_active_levels(MT8113GPIOState *s,
                                            unsigned port, uint32_t bits)
{
    uint32_t sensitivity =
        *mt8113_eint_reg(s, MT8113_EINT_SENS, port);

    bits &= sensitivity;
    while (bits) {
        unsigned bit = ctz32(bits);
        unsigned pin = port * 32 + bit;

        if (pin < MT8113_GPIO_NUM_PINS &&
            mt8113_eint_level_active(s, pin)) {
            *mt8113_eint_reg(s, MT8113_EINT_STAT, port) |= BIT(bit);
        }
        bits &= ~BIT(bit);
    }
}

static uint64_t mt8113_eint_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8113GPIOState *s = opaque;

    return s->eint_regs[offset / sizeof(uint32_t)];
}

static void mt8113_eint_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    MT8113GPIOState *s = opaque;
    uint32_t val = value;
    uint32_t *reg;
    unsigned port;

    if (mt8113_eint_port(offset, MT8113_EINT_ACK, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_STAT, port) &= ~val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_MASK, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_MASK, port) = val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_MASK_SET, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_MASK, port) |= val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_MASK_CLR, &port)) {
        reg = mt8113_eint_reg(s, MT8113_EINT_MASK, port);
        *reg &= ~val;
        mt8113_eint_latch_active_levels(s, port, val);
    } else if (mt8113_eint_port(offset, MT8113_EINT_SENS, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_SENS, port) = val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_SENS_SET, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_SENS, port) |= val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_SENS_CLR, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_SENS, port) &= ~val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_SOFT, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_SOFT, port) = val;
        *mt8113_eint_reg(s, MT8113_EINT_STAT, port) |= val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_SOFT_SET, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_SOFT, port) |= val;
        *mt8113_eint_reg(s, MT8113_EINT_STAT, port) |= val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_SOFT_CLR, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_SOFT, port) &= ~val;
        *mt8113_eint_reg(s, MT8113_EINT_STAT, port) &= ~val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_POL, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_POL, port) = val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_POL_SET, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_POL, port) |= val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_POL_CLR, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_POL, port) &= ~val;
    } else if (mt8113_eint_port(offset, MT8113_EINT_DOM_EN, &port)) {
        *mt8113_eint_reg(s, MT8113_EINT_DOM_EN, port) = val;
    } else {
        s->eint_regs[offset / sizeof(uint32_t)] = val;
    }
    mt8113_eint_update_irq(s);
}

static const MemoryRegionOps mt8113_eint_ops = {
    .read = mt8113_eint_read,
    .write = mt8113_eint_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mt8113_gpio_set_input(void *opaque, int pin, int level)
{
    MT8113GPIOState *s = opaque;
    unsigned port = pin / 32;
    uint32_t bit = BIT(pin % 32);
    bool old_level;
    bool polarity;
    bool sensitive;

    if (pin >= MT8113_GPIO_NUM_PINS) {
        return;
    }
    old_level = s->input_levels[port] & bit;
    if (level) {
        s->input_levels[port] |= bit;
    } else {
        s->input_levels[port] &= ~bit;
    }

    polarity = *mt8113_eint_reg(s, MT8113_EINT_POL, port) & bit;
    sensitive = *mt8113_eint_reg(s, MT8113_EINT_SENS, port) & bit;
    if ((sensitive && level == polarity) ||
        (!sensitive && old_level != level && level == polarity)) {
        *mt8113_eint_reg(s, MT8113_EINT_STAT, port) |= bit;
    }
    mt8113_eint_update_irq(s);
}

static void mt8113_gpio_reset(DeviceState *dev)
{
    MT8113GPIOState *s = MT8113_GPIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->eint_regs, 0, sizeof(s->eint_regs));
    for (unsigned port = 0; port < MT8113_EINT_PORTS; port++) {
        *mt8113_eint_reg(s, MT8113_EINT_MASK, port) = UINT32_MAX;
    }
    qemu_set_irq(s->irq, 0);
}

static void mt8113_gpio_init(Object *obj)
{
    MT8113GPIOState *s = MT8113_GPIO(obj);

    for (unsigned port = 0; port < ARRAY_SIZE(s->input_levels); port++) {
        s->input_levels[port] = UINT32_MAX;
    }
    memory_region_init_io(&s->iomem, obj, &mt8113_gpio_ops, s,
                          TYPE_MT8113_GPIO, MT8113_GPIO_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    memory_region_init_io(&s->eint_iomem, obj, &mt8113_eint_ops, s,
                          "mt8113.eint", MT8113_GPIO_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->eint_iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), mt8113_gpio_set_input,
                            "gpio-in", MT8113_GPIO_NUM_PINS);
    qdev_init_gpio_out_named(DEVICE(obj), s->output,
                             "gpio-out", MT8113_GPIO_NUM_PINS);
}

static void mt8113_gpio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->desc = "MediaTek MT8113 pinctrl/GPIO";
    device_class_set_legacy_reset(dc, mt8113_gpio_reset);
}

static const TypeInfo mt8113_gpio_info = {
    .name = TYPE_MT8113_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8113GPIOState),
    .instance_init = mt8113_gpio_init,
    .class_init = mt8113_gpio_class_init,
};

static void mt8113_gpio_register_types(void)
{
    type_register_static(&mt8113_gpio_info);
}
type_init(mt8113_gpio_register_types)
