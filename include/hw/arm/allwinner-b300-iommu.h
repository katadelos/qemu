/* Allwinner sun8iw15p1 IOMMU. SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_ALLWINNER_B300_IOMMU_H
#define HW_ARM_ALLWINNER_B300_IOMMU_H

#include "hw/core/sysbus.h"

#define TYPE_AW_B300_IOMMU "allwinner-b300-iommu"
OBJECT_DECLARE_SIMPLE_TYPE(AWB300IOMMUState, AW_B300_IOMMU)

/* Native master IDs: DE0, EDMA1, E_FE2, VE3, CSI4, G2D5, E_BE6. */
AddressSpace *aw_b300_iommu_as(AWB300IOMMUState *s, unsigned master);

#endif
