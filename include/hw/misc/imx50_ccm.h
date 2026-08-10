/* Freescale i.MX50 Clock Control Module and DPLLs */
#ifndef HW_MISC_IMX50_CCM_H
#define HW_MISC_IMX50_CCM_H

#include "hw/misc/imx_ccm.h"

#define TYPE_IMX50_CCM "imx50.ccm"
OBJECT_DECLARE_SIMPLE_TYPE(IMX50CCMState, IMX50_CCM)

typedef struct IMX50PLLRegion {
    IMX50CCMState *ccm;
    unsigned index;
} IMX50PLLRegion;

struct IMX50CCMState {
    IMXCCMState parent_obj;
    MemoryRegion ccm_iomem;
    MemoryRegion pll_iomem[3];
    IMX50PLLRegion pll_region[3];
    uint32_t ccm[0x1000 / 4];
    uint32_t pll[3][0x100 / 4];
};
#endif
