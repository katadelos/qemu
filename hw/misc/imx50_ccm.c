/* Freescale i.MX50 Clock Control Module and DPLLs */
#include "qemu/osdep.h"
#include "hw/misc/imx50_ccm.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define CCM_CCSR   0x0c
#define CCM_CBCDR  0x14
#define CCM_CBCMR  0x18
#define CCM_CSCMR1 0x1c
#define CCM_CS1CDR 0x28
#define CCM_CS2CDR 0x2c
#define CCM_CSCDR2 0x38

#define PLL_DP_CTL      0x00
#define PLL_DP_CTL_LRF  (1U << 0)
#define PLL_DP_CTL_RST  (1U << 4)
#define PLL_DP_CTL_UPEN (1U << 5)

static void imx50_ccm_reset(DeviceState *dev)
{
    IMX50CCMState *s = IMX50_CCM(dev);

    memset(s->ccm, 0, sizeof(s->ccm));
    memset(s->pll, 0, sizeof(s->pll));
    s->stop_wake_asserted = false;

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

    return r->ccm->pll[r->index][offset / 4];
}

static void imx50_pll_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    IMX50PLLRegion *r = opaque;
    uint32_t old = r->ccm->pll[r->index][offset / 4];

    if (offset == PLL_DP_CTL) {
        /*
         * LRF is read-only. Powering up a PLL starts acquisition, as does
         * a manual RST write. K4's _clk_pll_enable only raises UPEN.
         * An already enabled PLL retains its existing lock.
         */
        value &= ~PLL_DP_CTL_LRF;
        if ((value & PLL_DP_CTL_UPEN) &&
            (!(old & PLL_DP_CTL_UPEN) || (value & PLL_DP_CTL_RST) ||
             (old & PLL_DP_CTL_LRF))) {
            value |= PLL_DP_CTL_LRF;
        }
    }

    /* DP_CONFIG.LDREQ clears when the new DPLL factors are accepted. */
    if (offset == 0x04) {
        value &= ~1U;
    }
    r->ccm->pll[r->index][offset / 4] = value;
}

static void imx50_ccm_stop_wake(void *opaque, int line, int level)
{
    IMX50CCMState *s = opaque;

    /*
     * The i.MX50 STOP workaround restarts PLL1 before WFI.  Hardware holds
     * the CPU on step_clk until PLL1 has relocked at the staged rate, so the
     * first instruction after WFI observes LRF set without another restart.
     */
    if (level && !s->stop_wake_asserted &&
        (s->pll[0][PLL_DP_CTL / 4] & PLL_DP_CTL_UPEN)) {
        s->pll[0][PLL_DP_CTL / 4] |= PLL_DP_CTL_LRF;
    }
    s->stop_wake_asserted = level;
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

static uint32_t imx50_pll_clock(IMX50CCMState *s, unsigned pll)
{
    uint32_t ctl = s->pll[pll][PLL_DP_CTL / 4];
    unsigned op_offset = (ctl & BIT(7)) ? 0x1c : 0x08;
    uint32_t op = s->pll[pll][op_offset / 4];
    uint32_t mfd = s->pll[pll][(op_offset + 4) / 4] & 0x07ffffff;
    int32_t mfn = sextract32(s->pll[pll][(op_offset + 8) / 4], 0, 27);
    uint32_t mfi = MAX(extract32(op, 4, 4), 5);
    uint32_t reference = 24000000 * ((ctl & BIT(12)) ? 4 : 2);

    reference /= extract32(op, 0, 4) + 1;
    return (int64_t)reference * mfi +
           (int64_t)reference * mfn / (mfd + 1);
}

static uint32_t imx50_lp_apm_clock(IMX50CCMState *s)
{
    return (s->ccm[CCM_CCSR / 4] & BIT(10)) ? 480000000 : 24000000;
}

static uint32_t imx50_pll_switch_clock(IMX50CCMState *s, unsigned pll)
{
    uint32_t ccsr = s->ccm[CCM_CCSR / 4];

    if (!(ccsr & BIT(2 - pll))) {
        return imx50_pll_clock(s, pll);
    }
    if (pll != 0) {
        return 24000000;
    }
    switch (extract32(ccsr, 7, 2)) {
    case 2:
        return imx50_pll_switch_clock(s, 1) /
               (extract32(ccsr, 5, 2) + 1);
    case 3:
        return imx50_pll_switch_clock(s, 2) /
               (extract32(ccsr, 3, 2) + 1);
    default:
        return imx50_lp_apm_clock(s);
    }
}

static uint32_t imx50_per_clock(IMX50CCMState *s)
{
    uint32_t cbcdr = s->ccm[CCM_CBCDR / 4];
    uint32_t cbcmr = s->ccm[CCM_CBCMR / 4];
    unsigned parent = extract32(cbcdr, 25, 2);
    uint32_t main_bus = parent == 3 ? imx50_lp_apm_clock(s) :
                                   imx50_pll_switch_clock(s, parent);

    if (cbcmr & BIT(0)) {
        return main_bus / (extract32(cbcdr, 10, 3) + 1) /
                          (extract32(cbcdr, 8, 2) + 1);
    }
    return ((cbcmr & BIT(1)) ? imx50_lp_apm_clock(s) : main_bus) /
           ((extract32(cbcdr, 6, 2) + 1) *
            (extract32(cbcdr, 3, 3) + 1) *
            (extract32(cbcdr, 0, 3) + 1));
}

static uint32_t imx50_ccm_get_clock(IMXCCMState *ccm, IMXClk clock)
{
    switch (clock) {
    case CLK_NONE:
        return 0;
    case CLK_32k:
        return 32768;
    case CLK_PER:
        /* GPT and Linux's clocksource must use the programmed perclk tree. */
        return imx50_per_clock(IMX50_CCM(ccm));
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
    qdev_init_gpio_in_named(DEVICE(obj), imx50_ccm_stop_wake,
                            "stop-wake", 1);
}

static const VMStateDescription vmstate_imx50_ccm = {
    .name = TYPE_IMX50_CCM,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ccm, IMX50CCMState, 0x1000 / 4),
        VMSTATE_UINT32_2DARRAY(pll, IMX50CCMState, 3, 0x100 / 4),
        VMSTATE_BOOL(stop_wake_asserted, IMX50CCMState),
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
