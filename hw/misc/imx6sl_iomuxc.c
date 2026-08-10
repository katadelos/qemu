/*
 * i.MX6 SoloLite IOMUX controller register file
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/imx6sl_iomuxc.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

static uint64_t imx6sl_iomuxc_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    IMX6SLIOMUXCState *s = opaque;

    return s->regs[offset >> 2];
}

static void imx6sl_iomuxc_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    IMX6SLIOMUXCState *s = opaque;

    s->regs[offset >> 2] = value;
}

static const MemoryRegionOps imx6sl_iomuxc_ops = {
    .read = imx6sl_iomuxc_read,
    .write = imx6sl_iomuxc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void imx6sl_iomuxc_reset(DeviceState *dev)
{
    IMX6SLIOMUXCState *s = IMX6SL_IOMUXC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static const VMStateDescription vmstate_imx6sl_iomuxc = {
    .name = TYPE_IMX6SL_IOMUXC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMX6SLIOMUXCState,
                             IMX6SL_IOMUXC_SIZE / sizeof(uint32_t)),
        VMSTATE_END_OF_LIST()
    },
};

static void imx6sl_iomuxc_init(Object *obj)
{
    IMX6SLIOMUXCState *s = IMX6SL_IOMUXC(obj);

    memory_region_init_io(&s->iomem, obj, &imx6sl_iomuxc_ops, s,
                          TYPE_IMX6SL_IOMUXC, IMX6SL_IOMUXC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void imx6sl_iomuxc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, imx6sl_iomuxc_reset);
    dc->vmsd = &vmstate_imx6sl_iomuxc;
    dc->desc = "i.MX6 SoloLite IOMUX controller";
}

static const TypeInfo imx6sl_iomuxc_info = {
    .name = TYPE_IMX6SL_IOMUXC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX6SLIOMUXCState),
    .instance_init = imx6sl_iomuxc_init,
    .class_init = imx6sl_iomuxc_class_init,
};

static void imx6sl_iomuxc_register_types(void)
{
    type_register_static(&imx6sl_iomuxc_info);
}

type_init(imx6sl_iomuxc_register_types)
