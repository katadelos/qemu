/* Freescale i.MX TrustZone Interrupt Controller */
#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/intc/imx_tzic.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

static void imx_tzic_update(IMXTZICState *s)
{
    unsigned i;
    bool active = false;

    for (i = 0; i < ARRAY_SIZE(s->pending); i++) {
        active |= s->pending[i] & s->enabled[i];
    }
    qemu_set_irq(s->irq, active);
}

static void imx_tzic_set_irq(void *opaque, int irq, int level)
{
    IMXTZICState *s = opaque;
    unsigned word = irq / 32;
    uint32_t bit = 1u << (irq % 32);

    if (level) {
        s->pending[word] |= bit;
    } else {
        s->pending[word] &= ~bit;
    }
    imx_tzic_update(s);
}

static uint64_t imx_tzic_read(void *opaque, hwaddr offset, unsigned size)
{
    IMXTZICState *s = opaque;
    unsigned index;

    switch (offset) {
    case 0x000: return s->intcntl;
    case 0x004: return 0x00000003; /* 128 interrupt inputs */
    case 0x008: return 0x00000051; /* Freescale implementer */
    case 0x00c: return 0x1f;
    default:
        if (offset >= 0x100 && offset < 0x110) {
            index = (offset - 0x100) / 4;
            return s->enabled[index];
        }
        if (offset >= 0x400 && offset < 0x480) {
            return s->priority[(offset - 0x400) / 4];
        }
        if (offset >= 0xd00 && offset < 0xd10) {
            index = (offset - 0xd00) / 4;
            return s->pending[index];
        }
        if (offset >= 0xd80 && offset < 0xd90) {
            index = (offset - 0xd80) / 4;
            return s->pending[index] & s->enabled[index];
        }
        return 0;
    }
}

static void imx_tzic_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    IMXTZICState *s = opaque;
    unsigned index;

    switch (offset) {
    case 0x000:
        s->intcntl = value;
        break;
    default:
        if (offset >= 0x100 && offset < 0x110) {
            index = (offset - 0x100) / 4;
            s->enabled[index] |= value;
        } else if (offset >= 0x180 && offset < 0x190) {
            index = (offset - 0x180) / 4;
            s->enabled[index] &= ~value;
        } else if (offset >= 0x200 && offset < 0x210) {
            index = (offset - 0x200) / 4;
            s->pending[index] |= value;
        } else if (offset >= 0x280 && offset < 0x290) {
            index = (offset - 0x280) / 4;
            s->pending[index] &= ~value;
        } else if (offset >= 0x400 && offset < 0x480) {
            s->priority[(offset - 0x400) / 4] = value;
        }
        break;
    }
    imx_tzic_update(s);
}

static const MemoryRegionOps imx_tzic_ops = {
    .read = imx_tzic_read, .write = imx_tzic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void imx_tzic_init(Object *obj)
{
    IMXTZICState *s = IMX_TZIC(obj);
    memory_region_init_io(&s->iomem, obj, &imx_tzic_ops, s,
                          TYPE_IMX_TZIC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in(DEVICE(obj), imx_tzic_set_irq, IMX_TZIC_NUM_IRQS);
}

static const VMStateDescription vmstate_imx_tzic = {
    .name = TYPE_IMX_TZIC, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(enabled, IMXTZICState, 4),
        VMSTATE_UINT32_ARRAY(pending, IMXTZICState, 4),
        VMSTATE_UINT32_ARRAY(priority, IMXTZICState, 32),
        VMSTATE_UINT32(intcntl, IMXTZICState), VMSTATE_END_OF_LIST()
    },
};

static void imx_tzic_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->vmsd = &vmstate_imx_tzic;
    dc->desc = "Freescale i.MX TrustZone Interrupt Controller";
}

static const TypeInfo imx_tzic_info = {
    .name = TYPE_IMX_TZIC, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXTZICState), .instance_init = imx_tzic_init,
    .class_init = imx_tzic_class_init,
};
static void imx_tzic_register_types(void)
{
    type_register_static(&imx_tzic_info);
}
type_init(imx_tzic_register_types)
