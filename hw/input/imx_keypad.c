/* i.MX keypad port. Matrix contacts are row * 8 + column GPIO inputs.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_IMX_KEYPAD "imx-keypad"
OBJECT_DECLARE_SIMPLE_TYPE(IMXKeypadState, IMX_KEYPAD)
struct IMXKeypadState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint16_t regs[4];
    uint64_t contacts;
    uint8_t rows;
    qemu_irq irq;
};

static void imx_keypad_irq(IMXKeypadState *s)
{
    qemu_set_irq(s->irq, (s->regs[1] & (s->regs[1] >> 8) & 3) != 0);
}

static uint8_t imx_keypad_rows(IMXKeypadState *s)
{
    uint8_t rows = 0xff;
    uint8_t columns = (s->regs[2] & ~s->regs[3]) >> 8;
    unsigned row;

    for (row = 0; row < 8; row++) {
        if ((s->regs[0] & BIT(row)) &&
            ((s->contacts >> (row * 8)) & columns)) {
            rows &= ~BIT(row);
        }
    }
    return rows;
}

static void imx_keypad_update(IMXKeypadState *s)
{
    uint8_t rows = imx_keypad_rows(s);

    if (s->rows & ~rows) {
        s->regs[1] |= 1; /* KPKD: a row went low. */
    }
    if (s->rows != 0xff && rows == 0xff) {
        s->regs[1] |= 2; /* KPKR: all rows were released. */
    }
    s->rows = rows;
    imx_keypad_irq(s);
}

static void imx_keypad_contact(void *opaque, int contact, int level)
{
    IMXKeypadState *s = opaque;

    s->contacts = deposit64(s->contacts, contact, 1, level != 0);
    imx_keypad_update(s);
}
static uint64_t imx_keypad_read(void *opaque, hwaddr offset, unsigned size)
{
    IMXKeypadState *s = opaque;
    if (offset == 6) {
        return (s->regs[3] & s->regs[2]) |
               ((0xff00 | imx_keypad_rows(s)) & ~s->regs[2]);
    }
    return s->regs[offset / 2];
}
static void imx_keypad_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    IMXKeypadState *s = opaque;
    if (offset == 2) {
        /* KPKD/KPKR are W1C, synchronizer clear bits are self-clearing. */
        s->regs[1] = (value & 0x700) | (s->regs[1] & ~value & 3);
    } else {
        s->regs[offset / 2] = value;
    }
    imx_keypad_update(s);
}
static const MemoryRegionOps imx_keypad_ops = {
    .read = imx_keypad_read, .write = imx_keypad_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 2 },
};
static void imx_keypad_reset(DeviceState *dev)
{
    IMXKeypadState *s = IMX_KEYPAD(dev);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[3] = 0xffff;
    s->contacts = 0;
    s->rows = 0xff;
    imx_keypad_irq(s);
}
static int imx_keypad_post_load(void *opaque, int version_id)
{
    imx_keypad_irq(opaque);
    return 0;
}
static const VMStateDescription imx_keypad_vmstate = {
    .name = TYPE_IMX_KEYPAD, .version_id = 1, .minimum_version_id = 1,
    .post_load = imx_keypad_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, IMXKeypadState, 4),
        VMSTATE_UINT64(contacts, IMXKeypadState),
        VMSTATE_UINT8(rows, IMXKeypadState),
        VMSTATE_END_OF_LIST()
    },
};
static void imx_keypad_init(Object *obj)
{
    IMXKeypadState *s = IMX_KEYPAD(obj);
    memory_region_init_io(&s->iomem, obj, &imx_keypad_ops, s,
                          TYPE_IMX_KEYPAD, 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), imx_keypad_contact, "key", 64);
}
static void imx_keypad_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->vmsd = &imx_keypad_vmstate;
    device_class_set_legacy_reset(dc, imx_keypad_reset);
}
static const TypeInfo imx_keypad_type = {
    .name = TYPE_IMX_KEYPAD, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXKeypadState), .instance_init = imx_keypad_init,
    .class_init = imx_keypad_class_init,
};
static void imx_keypad_register(void) { type_register_static(&imx_keypad_type); }
type_init(imx_keypad_register)
