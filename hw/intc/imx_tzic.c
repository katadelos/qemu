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
        uint32_t enabled = s->enabled[i];

        if (s->wake_filter) {
            enabled &= s->wakeup[i];
        }
        active |= s->pending[i] & enabled;
    }
    qemu_set_irq(s->irq, active);
    qemu_set_irq(s->deep_wake[0], active && s->wake_filter);
    qemu_set_irq(s->deep_wake[1], active && s->wake_filter);
}

static void imx_tzic_update_wake_filter(IMXTZICState *s)
{
    unsigned i;

    s->wake_filter = false;
    for (i = 0; i < ARRAY_SIZE(s->wakeup); i++) {
        if (s->wakeup[i] != s->enabled[i]) {
            s->wake_filter = true;
            return;
        }
    }
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
    case 0x014: return s->dsmint;
    default:
        if (offset >= 0x080 && offset < 0x090) {
            index = (offset - 0x080) / 4;
            return s->intsec[index];
        }
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
        if (offset >= 0xe00 && offset < 0xe10) {
            index = (offset - 0xe00) / 4;
            return s->wakeup[index];
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
    case 0x014:
        s->dsmint = value & 1;
        break;
    default:
        if (offset >= 0x080 && offset < 0x090) {
            index = (offset - 0x080) / 4;
            s->intsec[index] = value;
        } else if (offset >= 0x100 && offset < 0x110) {
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
        } else if (offset >= 0xe00 && offset < 0xe10) {
            index = (offset - 0xe00) / 4;
            s->wakeup[index] = value;
            /*
             * Linux copies ENSET into all four WAKEUP banks for ordinary
             * idle, but installs a different wake-only set for system
             * suspend.  Treat that completed register state as the TZIC's
             * deep-sleep input filter so a non-wake timer cannot release a
             * guest blocked in WFI.
             */
            imx_tzic_update_wake_filter(s);
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
    qdev_init_gpio_out_named(DEVICE(obj), s->deep_wake, "deep-wake", 2);
    qdev_init_gpio_in(DEVICE(obj), imx_tzic_set_irq, IMX_TZIC_NUM_IRQS);
}

static const VMStateDescription vmstate_imx_tzic = {
    .name = TYPE_IMX_TZIC, .version_id = 2, .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(enabled, IMXTZICState, 4),
        VMSTATE_UINT32_ARRAY(pending, IMXTZICState, 4),
        VMSTATE_UINT32_ARRAY(intsec, IMXTZICState, 4),
        VMSTATE_UINT32_ARRAY(priority, IMXTZICState, 32),
        VMSTATE_UINT32(intcntl, IMXTZICState),
        VMSTATE_UINT32(dsmint, IMXTZICState),
        VMSTATE_UINT32_ARRAY(wakeup, IMXTZICState, 4),
        VMSTATE_BOOL(wake_filter, IMXTZICState),
        VMSTATE_END_OF_LIST()
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
