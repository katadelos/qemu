/* i.MX5/i.MX6 USB OTG wakeup control. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/usb/imx-usb-misc.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "trace.h"

#define IMX5_USBCTRL_OWIE BIT(27)
#define IMX6_USBCTRL_OWIE BIT(10)
#define USBCTRL_OWIR BIT(31)
#define IMX5_USBCTRL_H1WIE BIT(11)
#define IMX5_USBCTRL_H1WIR BIT(15)

OBJECT_DECLARE_SIMPLE_TYPE(IMXUSBMiscState, IMX_USB_MISC)

struct IMXUSBMiscState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[2];
    uint32_t ctrl;
    bool imx6_layout;
};

static uint32_t imx_usb_misc_wakeup_enable(IMXUSBMiscState *s)
{
    return s->imx6_layout ? IMX6_USBCTRL_OWIE : IMX5_USBCTRL_OWIE;
}

static void imx_usb_misc_update_irq(IMXUSBMiscState *s)
{
    uint32_t mask = imx_usb_misc_wakeup_enable(s) | USBCTRL_OWIR;

    qemu_set_irq(s->irq[0], (s->ctrl & mask) == mask);
    mask = IMX5_USBCTRL_H1WIE | IMX5_USBCTRL_H1WIR;
    qemu_set_irq(s->irq[1], !s->imx6_layout && (s->ctrl & mask) == mask);
}

static void imx_usb_misc_resume(void *opaque, int n, int level)
{
    IMXUSBMiscState *s = opaque;

    uint32_t enable = n ? IMX5_USBCTRL_H1WIE : imx_usb_misc_wakeup_enable(s);
    uint32_t request = n ? IMX5_USBCTRL_H1WIR : USBCTRL_OWIR;

    trace_imx_usb_misc_resume(n, level, s->ctrl);
    if (level && (s->ctrl & enable)) {
        s->ctrl |= request;
        imx_usb_misc_update_irq(s);
    }
}

static uint64_t imx_usb_misc_read(void *opaque, hwaddr addr, unsigned size)
{
    trace_imx_usb_misc_ctrl(false, IMX_USB_MISC(opaque)->ctrl);
    return IMX_USB_MISC(opaque)->ctrl;
}

static void imx_usb_misc_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    IMXUSBMiscState *s = opaque;

    /* The read-only request latch clears when wakeup interrupts are disabled. */
    uint32_t requests = USBCTRL_OWIR |
        (s->imx6_layout ? 0 : IMX5_USBCTRL_H1WIR);

    s->ctrl = (value & ~requests) | (s->ctrl & requests);
    trace_imx_usb_misc_ctrl(true, value);
    if (!(s->ctrl & imx_usb_misc_wakeup_enable(s))) {
        s->ctrl &= ~USBCTRL_OWIR;
    }
    if (!(s->ctrl & IMX5_USBCTRL_H1WIE) && !s->imx6_layout) {
        s->ctrl &= ~IMX5_USBCTRL_H1WIR;
    }
    imx_usb_misc_update_irq(s);
}

static const MemoryRegionOps imx_usb_misc_ops = {
    .read = imx_usb_misc_read,
    .write = imx_usb_misc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void imx_usb_misc_reset(DeviceState *dev)
{
    IMXUSBMiscState *s = IMX_USB_MISC(dev);

    s->ctrl = 0;
    imx_usb_misc_update_irq(s);
}

static const VMStateDescription vmstate_imx_usb_misc = {
    .name = TYPE_IMX_USB_MISC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, IMXUSBMiscState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property imx_usb_misc_properties[] = {
    DEFINE_PROP_BOOL("imx6-layout", IMXUSBMiscState, imx6_layout, false),
};

static void imx_usb_misc_init(Object *obj)
{
    IMXUSBMiscState *s = IMX_USB_MISC(obj);

    memory_region_init_io(&s->iomem, obj, &imx_usb_misc_ops, s,
                          TYPE_IMX_USB_MISC, 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[0]);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[1]);
    qdev_init_gpio_in(DEVICE(obj), imx_usb_misc_resume, 2);
}

static void imx_usb_misc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, imx_usb_misc_reset);
    device_class_set_props(dc, imx_usb_misc_properties);
    dc->vmsd = &vmstate_imx_usb_misc;
    dc->desc = "i.MX USB OTG wakeup control";
}

static const TypeInfo imx_usb_misc_info = {
    .name = TYPE_IMX_USB_MISC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXUSBMiscState),
    .instance_init = imx_usb_misc_init,
    .class_init = imx_usb_misc_class_init,
};

static void imx_usb_misc_register_types(void)
{
    type_register_static(&imx_usb_misc_info);
}
type_init(imx_usb_misc_register_types)
