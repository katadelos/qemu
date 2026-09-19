/*
 * i.MX7 DDR controller and PHY control interface.
 *
 * RAM accesses use QEMU's system memory directly.  This models the register
 * handshakes used for frequency changes and self-refresh, not DRAM timing,
 * training, or contention.  Pending operations complete immediately.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/imx7_ddrc.h"
#include "migration/vmstate.h"
#include "qemu/bitops.h"
#include "qemu/module.h"

#define DDRC_MSTR       0x000
#define DDRC_STAT       0x004
#define DDRC_MRCTRL0    0x010
#define DDRC_MRSTAT     0x018
#define DDRC_PWRCTL     0x030
#define DDRC_DFISTAT    0x1bc
#define DDRC_DBGCAM     0x308
#define DDRC_SWCTL      0x320
#define DDRC_SWSTAT     0x324
#define DDRC_PSTAT      0x3fc
#define DDRC_PCTRL0     0x490
#define DDRPHY_MDLL_CON1 0x0b4

static uint64_t imx7_ddrc_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX7DDRCState *s = opaque;

    switch (offset) {
    case DDRC_STAT:
        /* Operating mode 3 / self-refresh type 2 for software requests. */
        return (s->controller_regs[DDRC_PWRCTL / 4] & BIT(5)) ? 0x23 : 1;
    case DDRC_MRSTAT:
    case DDRC_PSTAT:
        return 0; /* No mode-register command or AXI port request is busy. */
    case DDRC_DFISTAT:
        return 1; /* PHY initialization complete. */
    case DDRC_DBGCAM:
        /* All read/write queues and pipelines are empty. */
        return 0x36000000;
    case DDRC_SWSTAT:
        return s->controller_regs[DDRC_SWCTL / 4] & 1;
    default:
        return s->controller_regs[offset / 4];
    }
}

static void imx7_ddrc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    IMX7DDRCState *s = opaque;

    switch (offset) {
    case DDRC_STAT:
    case DDRC_MRSTAT:
    case DDRC_DFISTAT:
    case DDRC_DBGCAM:
    case DDRC_SWSTAT:
    case DDRC_PSTAT:
        return;
    case DDRC_MRCTRL0:
        /* mr_wr initiates an operation and clears when accepted. */
        value &= ~BIT(31);
        break;
    }
    s->controller_regs[offset / 4] = value;
}

static uint64_t imx7_ddrphy_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX7DDRCState *s = opaque;
    uint32_t value = s->phy_regs[offset / 4];

    if (offset == DDRPHY_MDLL_CON1) {
        value |= BIT(2); /* Master delay-locked loop is locked. */
    }
    return value;
}

static void imx7_ddrphy_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    IMX7DDRCState *s = opaque;

    s->phy_regs[offset / 4] = value;
}

static const MemoryRegionOps imx7_ddrc_ops = {
    .read = imx7_ddrc_read,
    .write = imx7_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps imx7_ddrphy_ops = {
    .read = imx7_ddrphy_read,
    .write = imx7_ddrphy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void imx7_ddrc_reset(DeviceState *dev)
{
    IMX7DDRCState *s = IMX7_DDRC(dev);

    memset(s->controller_regs, 0, sizeof(s->controller_regs));
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    /* Firmware starts with the RAM already initialized by the boot ROM. */
    s->controller_regs[DDRC_MSTR / 4] = 0x03040008; /* LPDDR3 */
    s->controller_regs[DDRC_SWCTL / 4] = 1;
    s->controller_regs[DDRC_PCTRL0 / 4] = 1;
}

static const VMStateDescription vmstate_imx7_ddrc = {
    .name = TYPE_IMX7_DDRC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(controller_regs, IMX7DDRCState,
                             IMX7_DDRC_REG_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(phy_regs, IMX7DDRCState,
                             IMX7_DDRC_REG_SIZE / sizeof(uint32_t)),
        VMSTATE_END_OF_LIST()
    },
};

static void imx7_ddrc_init(Object *obj)
{
    IMX7DDRCState *s = IMX7_DDRC(obj);

    memory_region_init_io(&s->controller, obj, &imx7_ddrc_ops, s,
                          TYPE_IMX7_DDRC, IMX7_DDRC_REG_SIZE);
    memory_region_init_io(&s->phy, obj, &imx7_ddrphy_ops, s,
                          TYPE_IMX7_DDRC ".phy", IMX7_DDRC_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->controller);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->phy);
}

static void imx7_ddrc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "i.MX7 DDR controller and PHY";
    dc->vmsd = &vmstate_imx7_ddrc;
    device_class_set_legacy_reset(dc, imx7_ddrc_reset);
}

static const TypeInfo imx7_ddrc_info = {
    .name = TYPE_IMX7_DDRC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX7DDRCState),
    .instance_init = imx7_ddrc_init,
    .class_init = imx7_ddrc_class_init,
};

static void imx7_ddrc_register_types(void)
{
    type_register_static(&imx7_ddrc_info);
}
type_init(imx7_ddrc_register_types)
