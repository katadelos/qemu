/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MT8171_MCUPM_H
#define HW_MT8171_MCUPM_H
#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_MT8171_MCUPM "mt8171-mcupm"
OBJECT_DECLARE_SIMPLE_TYPE(MT8171MCUPMState, MT8171_MCUPM)
struct MT8171MCUPMState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[8];
    QEMUTimer *timer;
    uint32_t regs[0x3000 / 4];
    uint32_t tx[8][20];
    uint32_t tx_pending, rx_pending;
    uint32_t shared_base, shared_size, logger_base;
    uint32_t ssc_rate[11], ssc_dt[11], ssc_df[11];
    uint32_t tr_id, tr_value;
    uint64_t tr_begin, tr_end;
    bool firmware_loaded, service_ready;
};
/* Board has replayed the original BL2 bootstrap into the SRAM mapping. */
void mt8171_mcupm_set_loaded(MT8171MCUPMState *s, bool loaded);
#endif
