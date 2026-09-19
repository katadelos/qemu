/*
 * i.MX7 CoreSight debug access port.
 *
 * Model the component identification and programming registers of the two
 * ETMs, two funnels, ETF, ETR and TPIU. No instruction trace is produced.
 * The AMBA identities are required even when tracing is unused: Linux
 * constructs the complete CoreSight endpoint graph during device probing.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/imx7_dap.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

static const struct {
    hwaddr offset;
    uint32_t periphid;
    uint32_t devid;
} components[IMX7_DAP_COMPONENTS] = {
    { 0x41000, 0x000bb908, 2 },          /* Cortex-A7 trace funnel */
    { 0x7c000, 0x000bb956, 0 },          /* CPU0 ETM 3.5 */
    { 0x7d000, 0x000bb956, 0 },          /* CPU1 ETM 3.5 */
    { 0x83000, 0x000bb908, 2 },          /* System trace funnel */
    { 0x84000, 0x000bb961, 0x880 },      /* TMC embedded trace FIFO */
    { 0x86000, 0x000bb961, 0x840 },      /* TMC embedded trace router */
    { 0x87000, 0x000bb912, 0 },          /* Trace port interface */
};

static int imx7_dap_component(hwaddr offset)
{
    for (unsigned i = 0; i < ARRAY_SIZE(components); i++) {
        if ((offset & ~0xfff) == components[i].offset) {
            return i;
        }
    }
    return -1;
}

static uint64_t imx7_dap_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX7DAPState *s = opaque;
    int component = imx7_dap_component(offset);
    unsigned reg = offset & 0xfff;
    uint32_t pid;

    if (component < 0) {
        return 0;
    }
    pid = components[component].periphid;
    if (reg >= 0xfe0 && reg <= 0xfec) {
        return (pid >> (8 * ((reg - 0xfe0) / 4))) & 0xff;
    }
    if (reg >= 0xff0) {
        return (0xb105900dU >> (8 * ((reg - 0xff0) / 4))) & 0xff;
    }
    if (reg == 0xfd0) {
        return 4; /* JEP106 continuation code: ARM */
    }
    if (reg == 0xfc8) {
        return components[component].devid;
    }
    if (reg == 0xfb4) {
        return s->regs[component][0xfb0 / 4] == 0xc5acce55 ? 1 : 3;
    }
    if (pid == 0xbb956) {
        switch (reg) {
        case 0x004: /* ETMCCR: 8 address comparators, 2 counters */
            return 4 | (2 << 13);
        case 0x010: /* ETMSR: programming bit follows ETMCR */
            return (s->regs[component][0] & BIT(10)) ? BIT(1) : 0;
        case 0x1e4: /* ETMIDR: ETM architecture 3.5 */
            return 0x250;
        case 0x304: /* ETMOSLSR: OS lock released */
            return 0;
        case 0x314: /* ETMPDSR: powered up */
            return 1;
        }
    } else if (pid == 0xbb961) {
        switch (reg) {
        case 0x004: /* TMC_RSZ: 16 KiB buffer */
            return 0x1000;
        case 0x00c: /* TMC_STS: ready, empty */
            return BIT(2) | BIT(4);
        case 0x010: /* TMC_RRD: no trace data */
            return 0xffffffff;
        }
    }
    return s->regs[component][reg / 4];
}

static void imx7_dap_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    IMX7DAPState *s = opaque;
    int component = imx7_dap_component(offset);
    unsigned reg = offset & 0xfff;

    if (component < 0 || reg >= 0xfc8) {
        return;
    }
    /* Manual formatter flush completes immediately with no trace producer. */
    if (reg == 0x304 && components[component].periphid != 0xbb956) {
        value &= ~BIT(6);
    }
    s->regs[component][reg / 4] = value;
}

static const MemoryRegionOps imx7_dap_ops = {
    .read = imx7_dap_read,
    .write = imx7_dap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void imx7_dap_reset(DeviceState *dev)
{
    IMX7DAPState *s = IMX7_DAP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[1][0] = BIT(10) | BIT(0);
    s->regs[2][0] = BIT(10) | BIT(0);
}

static const VMStateDescription vmstate_imx7_dap = {
    .name = TYPE_IMX7_DAP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(regs, IMX7DAPState, IMX7_DAP_COMPONENTS, 1024),
        VMSTATE_END_OF_LIST()
    },
};

static void imx7_dap_init(Object *obj)
{
    IMX7DAPState *s = IMX7_DAP(obj);

    memory_region_init_io(&s->iomem, obj, &imx7_dap_ops, s,
                          TYPE_IMX7_DAP, 0x100000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void imx7_dap_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, imx7_dap_reset);
    dc->vmsd = &vmstate_imx7_dap;
}

static const TypeInfo imx7_dap_info = {
    .name = TYPE_IMX7_DAP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX7DAPState),
    .instance_init = imx7_dap_init,
    .class_init = imx7_dap_class_init,
};

static void imx7_dap_register_types(void)
{
    type_register_static(&imx7_dap_info);
}

type_init(imx7_dap_register_types)
