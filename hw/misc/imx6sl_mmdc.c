/*
 * Minimal i.MX6 SoloLite MMDC register model
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/imx6sl_mmdc.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/units.h"

#define MMDC_SIZE   0x4000
#define MDCTL       0x0000
#define MDSCR       0x001c
#define MDMISC      0x0018
#define MDMRR       0x0034
#define MAPSR       0x0404
#define MPRDDLCTL   0x083c
#define MPMUR0      0x08b8
#define MDSCR_CON_REQ   (1U << 15)
#define MDSCR_CON_ACK   (1U << 14)
#define MDSCR_MRR_VALID (1U << 10)
#define MAPSR_DVFS      (1U << 21)
#define MAPSR_DVACK     (1U << 25)
#define MAPSR_PSS       (1U << 4)
#define MPMUR0_FRC_MSR  (1U << 11)
#define MPRDDLCTL_RST_RD_FIFO (1U << 31)

static uint64_t imx6sl_mmdc_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX6SLMMDCState *s = opaque;
    uint32_t value = s->regs[offset >> 2];

    if (offset == MDSCR) {
        /* Check the programmed command before adding synthesized status. */
        if ((value & 0xffff) == 0x8060) {
            value |= MDSCR_MRR_VALID;
        }
        if (value & MDSCR_CON_REQ) {
            value |= MDSCR_CON_ACK;
        }
    } else if (offset == MDMRR) {
        /* JEDEC manufacturer ID 0x06 (SK hynix). */
        value = 0x06;
    } else if (offset == MAPSR) {
        /*
         * The SoloLite IRAM idle routine explicitly requests DDR
         * self-refresh with DVFS and polls DVACK before reducing clocks.
         * There is no DRAM timing delay in the model, so acknowledge (and
         * deassert) synchronously with the request bit.
         */
        if (value & MAPSR_DVFS) {
            value |= MAPSR_DVACK;
        } else {
            value &= ~MAPSR_DVACK;
        }
    }
    return value;
}

static void imx6sl_mmdc_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    IMX6SLMMDCState *s = opaque;

    if (offset == MPMUR0) {
        /*
         * FRC_MSR starts a PHY measurement and self-clears when it is done.
         * The SoloLite IRAM DDR-frequency routine polls this bit with caches
         * disabled, so retaining the written bit deadlocks the only CPU.
         * Timing has no observable effect in the current DRAM model; complete
         * the measurement synchronously while preserving the bypass/count
         * fields in MPMUR0.
         */
        value &= ~MPMUR0_FRC_MSR;
    } else if (offset == MPRDDLCTL) {
        /* RST_RD_FIFO is another write-one command/self-clearing status bit. */
        value &= ~MPRDDLCTL_RST_RD_FIFO;
    }
    s->regs[offset >> 2] = value;
}

static const MemoryRegionOps imx6sl_mmdc_ops = {
    .read = imx6sl_mmdc_read,
    .write = imx6sl_mmdc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const VMStateDescription vmstate_imx6sl_mmdc = {
    .name = TYPE_IMX6SL_MMDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMX6SLMMDCState, 0x1000),
        VMSTATE_END_OF_LIST()
    },
};

static void imx6sl_mmdc_reset(DeviceState *dev)
{
    IMX6SLMMDCState *s = IMX6SL_MMDC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    if (s->ram_size) {
        /* Match the Wario DCD state that a real i.MX boot ROM applies. */
        s->regs[MDCTL >> 2] = s->ram_size <= 256 * MiB ?
                              0x83010000 : 0x83110000;
        s->regs[MDMISC >> 2] = 0x00001688;
    } else {
        /* Rex boot firmware configures Hynix LPDDR3 with this MDMISC value. */
        s->regs[MDMISC >> 2] = 0x00201718;
    }
    /* Automatic power saving is immediately active in the timing-free model. */
    s->regs[MAPSR >> 2] = MAPSR_PSS;
}

static const Property imx6sl_mmdc_properties[] = {
    DEFINE_PROP_UINT64("ram-size", IMX6SLMMDCState, ram_size, 0),
};

static void imx6sl_mmdc_init(Object *obj)
{
    IMX6SLMMDCState *s = IMX6SL_MMDC(obj);

    memory_region_init_io(&s->iomem, obj, &imx6sl_mmdc_ops, s,
                          TYPE_IMX6SL_MMDC, MMDC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void imx6sl_mmdc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, imx6sl_mmdc_reset);
    device_class_set_props(dc, imx6sl_mmdc_properties);
    dc->vmsd = &vmstate_imx6sl_mmdc;
}

static const TypeInfo imx6sl_mmdc_info = {
    .name = TYPE_IMX6SL_MMDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX6SLMMDCState),
    .instance_init = imx6sl_mmdc_init,
    .class_init = imx6sl_mmdc_class_init,
};

static void imx6sl_mmdc_register_types(void)
{
    type_register_static(&imx6sl_mmdc_info);
}

type_init(imx6sl_mmdc_register_types)
