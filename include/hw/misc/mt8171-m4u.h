/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MT8171_M4U_H
#define HW_MT8171_M4U_H
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_MT8171_M4U "mt8171-m4u"
OBJECT_DECLARE_SIMPLE_TYPE(MT8171M4UState, MT8171_M4U)
struct MT8171M4UState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[5][0x1000 / 4];
    qemu_irq irq[5];
};
/* Stream routing supplies the bank and hardware interrupt-ID encoding.
 * No physical-address bypass: every enabled stream walks its programmed PT. */
bool mt8171_m4u_translate(MT8171M4UState *s, unsigned bank, uint16_t stream,
                         hwaddr iova, bool write, hwaddr *physical,
                         size_t *span);
/* Read-only page-table observation for the diagnostic framebuffer console.
 * Does not consult a stream's MMU enable, record faults or raise interrupts. */
bool mt8171_m4u_debug_translate(MT8171M4UState *s, unsigned bank, hwaddr iova,
                               hwaddr *physical, size_t *span);
#endif
