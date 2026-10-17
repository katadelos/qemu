/* i.MX6 GPC register interface. Interrupt delivery is handled by the GIC;
 * power-domain requests complete without electrically gating emulated devices.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#define TYPE_IMX6_GPC "imx6-gpc"
OBJECT_DECLARE_SIMPLE_TYPE(IMX6GPCState, IMX6_GPC)
struct IMX6GPCState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[0x1000 / 4];
};
static uint64_t imx6_gpc_read(void *opaque, hwaddr offset, unsigned size)
{
    return IMX6_GPC(opaque)->regs[offset / 4];
}
static void imx6_gpc_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    IMX6GPCState *s = opaque;
    /* GPU/VPU, display and PCI power request strobes clear on completion. */
    s->regs[offset / 4] = offset ? value : value & ~0xf3U;
}
static const MemoryRegionOps imx6_gpc_ops = {
    .read = imx6_gpc_read, .write = imx6_gpc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static void imx6_gpc_reset(DeviceState *dev)
{
    IMX6GPCState *s = IMX6_GPC(dev);
    memset(s->regs, 0, sizeof(s->regs));
    for (unsigned i = 2; i < 6; i++) {
        s->regs[i] = UINT32_MAX;
    }
}
static const VMStateDescription imx6_gpc_vmstate = {
    .name = TYPE_IMX6_GPC, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMX6GPCState, 0x1000 / 4),
        VMSTATE_END_OF_LIST()
    },
};
static void imx6_gpc_init(Object *obj)
{
    IMX6GPCState *s = IMX6_GPC(obj);
    memory_region_init_io(&s->iomem, obj, &imx6_gpc_ops, s, TYPE_IMX6_GPC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}
static void imx6_gpc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->vmsd = &imx6_gpc_vmstate;
    device_class_set_legacy_reset(dc, imx6_gpc_reset);
}
static const TypeInfo imx6_gpc_type = {
    .name = TYPE_IMX6_GPC, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX6GPCState), .instance_init = imx6_gpc_init,
    .class_init = imx6_gpc_class_init,
};
static void imx6_gpc_register(void) { type_register_static(&imx6_gpc_type); }
type_init(imx6_gpc_register)
