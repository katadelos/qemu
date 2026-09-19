/*
 * Copyright (c) 2018, Impinj, Inc.
 *
 * i.MX7 GPCv2 block emulation code
 *
 * Author: Andrey Smirnov <andrew.smirnov@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/intc/imx_gpcv2.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define GPC_CPU_PGC_SW_PUP_REQ      0x0f0
#define GPC_CPU_PGC_SW_PDN_REQ      0x0fc

#define GPC_PU_PGC_SW_PUP_REQ       0x0f8
#define GPC_PU_PGC_SW_PDN_REQ       0x104

#define GPC_LPCR_A7_BSC             0x000
#define GPC_IMR1_CORE0              0x030
#define GPC_IMR1_M4                 0x050

/* The external interrupt wake path remains powered when the GIC is off. */
static void imx_gpcv2_update_wake(IMXGPCv2State *s)
{
    uint32_t lpcr = s->regs[GPC_LPCR_A7_BSC / 4];

    for (unsigned cpu = 0; cpu < GPC_NUM_CPUS; cpu++) {
        bool pending = false;

        if ((lpcr & (3 << (cpu * 2))) && (lpcr & BIT(28 + cpu))) {
            for (unsigned word = 0; word < GPC_NUM_IRQS / 32; word++) {
                unsigned imr = GPC_IMR1_CORE0 / 4 + cpu * 4 + word;
                pending |= (s->irq_levels[word] & ~s->regs[imr]) != 0;
            }
        }
        qemu_set_irq(s->wake[cpu], pending);
    }
}

static void imx_gpcv2_set_irq(void *opaque, int irq, int level)
{
    IMXGPCv2State *s = opaque;
    uint32_t mask = BIT(irq % 32);

    if (level) {
        s->irq_levels[irq / 32] |= mask;
    } else {
        s->irq_levels[irq / 32] &= ~mask;
    }
    qemu_set_irq(s->irq_out[irq], level);
    imx_gpcv2_update_wake(s);
}

#define USB_HSIC_PHY_SW_Pxx_REQ     BIT(4)
#define USB_OTG2_PHY_SW_Pxx_REQ     BIT(3)
#define USB_OTG1_PHY_SW_Pxx_REQ     BIT(2)
#define PCIE_PHY_SW_Pxx_REQ         BIT(1)
#define MIPI_PHY_SW_Pxx_REQ         BIT(0)


static void imx_gpcv2_reset(DeviceState *dev)
{
    IMXGPCv2State *s = IMX_GPCV2(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->irq_levels, 0, sizeof(s->irq_levels));
    for (unsigned offset = GPC_IMR1_CORE0; offset < GPC_IMR1_M4 + 16;
         offset += 4) {
        s->regs[offset / 4] = UINT32_MAX;
    }
    for (unsigned irq = 0; irq < GPC_NUM_IRQS; irq++) {
        qemu_set_irq(s->irq_out[irq], 0);
    }
    imx_gpcv2_update_wake(s);
}

static uint64_t imx_gpcv2_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    IMXGPCv2State *s = opaque;

    return s->regs[offset / sizeof(uint32_t)];
}

static void imx_gpcv2_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    IMXGPCv2State *s = opaque;
    const size_t idx = offset / sizeof(uint32_t);

    s->regs[idx] = value;

    /*
     * Real HW will clear those bits once as a way to indicate that
     * power up request is complete
     */
    if (offset == GPC_CPU_PGC_SW_PUP_REQ ||
        offset == GPC_CPU_PGC_SW_PDN_REQ) {
        s->regs[idx] &= ~BIT(1);
    }
    if (offset == GPC_PU_PGC_SW_PUP_REQ ||
        offset == GPC_PU_PGC_SW_PDN_REQ) {
        s->regs[idx] &= ~(USB_HSIC_PHY_SW_Pxx_REQ |
                          USB_OTG2_PHY_SW_Pxx_REQ |
                          USB_OTG1_PHY_SW_Pxx_REQ |
                          PCIE_PHY_SW_Pxx_REQ     |
                          MIPI_PHY_SW_Pxx_REQ);
    }
    imx_gpcv2_update_wake(s);
}

static const struct MemoryRegionOps imx_gpcv2_ops = {
    .read = imx_gpcv2_read,
    .write = imx_gpcv2_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        /*
         * Our device would not work correctly if the guest was doing
         * unaligned access. This might not be a limitation on the real
         * device but in practice there is no reason for a guest to access
         * this device unaligned.
         */
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void imx_gpcv2_init(Object *obj)
{
    SysBusDevice *sd = SYS_BUS_DEVICE(obj);
    IMXGPCv2State *s = IMX_GPCV2(obj);

    memory_region_init_io(&s->iomem,
                          obj,
                          &imx_gpcv2_ops,
                          s,
                          TYPE_IMX_GPCV2 ".iomem",
                          sizeof(s->regs));
    sysbus_init_mmio(sd, &s->iomem);
    qdev_init_gpio_in(DEVICE(obj), imx_gpcv2_set_irq, GPC_NUM_IRQS);
    qdev_init_gpio_out_named(DEVICE(obj), s->irq_out, "irq", GPC_NUM_IRQS);
    qdev_init_gpio_out_named(DEVICE(obj), s->wake, "wake", GPC_NUM_CPUS);
}

static int imx_gpcv2_post_load(void *opaque, int version_id)
{
    IMXGPCv2State *s = opaque;

    for (unsigned irq = 0; irq < GPC_NUM_IRQS; irq++) {
        qemu_set_irq(s->irq_out[irq],
                     (s->irq_levels[irq / 32] >> (irq % 32)) & 1);
    }
    imx_gpcv2_update_wake(s);
    return 0;
}

static const VMStateDescription vmstate_imx_gpcv2 = {
    .name = TYPE_IMX_GPCV2,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = imx_gpcv2_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMXGPCv2State, GPC_NUM),
        VMSTATE_UINT32_ARRAY(irq_levels, IMXGPCv2State, GPC_NUM_IRQS / 32),
        VMSTATE_END_OF_LIST()
    },
};

static void imx_gpcv2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, imx_gpcv2_reset);
    dc->vmsd  = &vmstate_imx_gpcv2;
    dc->desc  = "i.MX GPCv2 Module";
}

static const TypeInfo imx_gpcv2_info = {
    .name          = TYPE_IMX_GPCV2,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXGPCv2State),
    .instance_init = imx_gpcv2_init,
    .class_init    = imx_gpcv2_class_init,
};

static void imx_gpcv2_register_type(void)
{
    type_register_static(&imx_gpcv2_info);
}
type_init(imx_gpcv2_register_type)
