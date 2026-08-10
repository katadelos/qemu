/* Freescale i.MX50 Clock Control Module and DPLLs */
#include "qemu/osdep.h"
#include "hw/misc/imx50_ccm.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define CCM_CSCMR1 0x1c
#define CCM_CS1CDR 0x28
#define CCM_CS2CDR 0x2c
#define CCM_CSCDR2 0x38

static void imx50_ccm_reset(DeviceState *dev)
{
    IMX50CCMState *s = IMX50_CCM(dev);

    memset(s->ccm, 0, sizeof(s->ccm));
    memset(s->pll, 0, sizeof(s->pll));

    /*
     * SSI_EXT1 and SSI_EXT2 use their respective common SSI clocks after
     * reset.  The SSI and CSPI root clocks reset with divide-by-two
     * predividers.  Linux propagates the reset clock tree before programming
     * its preferred dividers, and treats a divide-by-one value as invalid.
     */
    s->ccm[CCM_CSCMR1 / 4] = 0x3;
    s->ccm[CCM_CS1CDR / 4] = (1 << 22) | (1 << 6);
    s->ccm[CCM_CS2CDR / 4] = (1 << 22) | (1 << 6);
    s->ccm[CCM_CSCDR2 / 4] = 1 << 25;
}

static uint64_t imx50_ccm_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX50CCMState *s = opaque;

    /* CDHIPR clock-domain handshakes complete immediately. */
    if (offset == 0x48) {
        return 0;
    }
    return s->ccm[offset / 4];
}

static void imx50_ccm_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    IMX50CCMState *s = opaque;

    s->ccm[offset / 4] = value;
}

static uint64_t imx50_pll_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX50PLLRegion *r = opaque;
    uint32_t value = r->ccm->pll[r->index][offset / 4];

    /* DPLL lock (LRF) is asserted once software enables the PLL. */
    return offset == 0 ? value | 1 : value;
}

static void imx50_pll_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    IMX50PLLRegion *r = opaque;

    r->ccm->pll[r->index][offset / 4] = value;
}

static const MemoryRegionOps imx50_ccm_ops = {
    .read = imx50_ccm_read,
    .write = imx50_ccm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps imx50_pll_ops = {
    .read = imx50_pll_read,
    .write = imx50_pll_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint32_t imx50_ccm_get_clock(IMXCCMState *ccm, IMXClk clock)
{
    switch (clock) {
    case CLK_NONE:
        return 0;
    case CLK_32k:
        return 32768;
    case CLK_IPG:
        return 66000000;
    case CLK_IPG_HIGH:
    case CLK_HIGH:
        return 133000000;
    default:
        return 24000000;
    }
}

static void imx50_ccm_init(Object *obj)
{
    IMX50CCMState *s = IMX50_CCM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    unsigned i;

    memory_region_init_io(&s->ccm_iomem, obj, &imx50_ccm_ops, s,
                          TYPE_IMX50_CCM ".ccm", 0x1000);
    sysbus_init_mmio(sbd, &s->ccm_iomem);
    for (i = 0; i < 3; i++) {
        IMX50PLLRegion *r = &s->pll_region[i];

        r->ccm = s;
        r->index = i;
        memory_region_init_io(&s->pll_iomem[i], obj, &imx50_pll_ops, r,
                              TYPE_IMX50_CCM ".pll", 0x100);
        sysbus_init_mmio(sbd, &s->pll_iomem[i]);
    }
}

static const VMStateDescription vmstate_imx50_ccm = {
    .name = TYPE_IMX50_CCM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ccm, IMX50CCMState, 0x1000 / 4),
        VMSTATE_UINT32_2DARRAY(pll, IMX50CCMState, 3, 0x100 / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void imx50_ccm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    IMXCCMClass *cc = IMX_CCM_CLASS(oc);

    dc->desc = "Freescale i.MX50 Clock Control Module";
    dc->vmsd = &vmstate_imx50_ccm;
    device_class_set_legacy_reset(dc, imx50_ccm_reset);
    cc->get_clock_frequency = imx50_ccm_get_clock;
}

static const TypeInfo imx50_ccm_info = {
    .name = TYPE_IMX50_CCM,
    .parent = TYPE_IMX_CCM,
    .instance_size = sizeof(IMX50CCMState),
    .instance_init = imx50_ccm_init,
    .class_init = imx50_ccm_class_init,
};
static void imx50_ccm_register_types(void)
{
    type_register_static(&imx50_ccm_info);
}
type_init(imx50_ccm_register_types)
