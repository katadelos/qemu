/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_IMX_CAAM_H
#define HW_IMX_CAAM_H
#include "hw/core/sysbus.h"
#define TYPE_IMX_CAAM "imx-caam"
OBJECT_DECLARE_SIMPLE_TYPE(IMXCAAMState, IMX_CAAM)
struct IMXCAAMState {
    SysBusDevice parent_obj;
    MemoryRegion memory;
    qemu_irq irq[4];
    uint32_t registers[0x10000 / 4];
    uint32_t input_index[4], output_index[4], pending[4];
    uint8_t jdkey[16];
    uint8_t mpkey[66], mpcurve;
};
#endif
