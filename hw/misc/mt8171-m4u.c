/*
 * MT8171 multimedia M4U: five context banks, 34-bit IOVA, 35-bit PA.
 * Sources: vendor mtk_iommu_mt8xxx.c and Linux io-pgtable-arm-v7s.c.
 * No translation cache is retained, so invalidations finish synchronously.
 * DMA engines must explicitly use the translator with their SMI bank/port.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/misc/mt8171-m4u.h"
#include "hw/core/irq.h"
#include "qemu/module.h"
#include "system/address-spaces.h"

static void m4u_irq(MT8171M4UState *s, unsigned bank)
{
    uint32_t *r = s->regs[bank];
    qemu_set_irq(s->irq[bank], !!(r[0x134 / 4] & r[0x124 / 4]));
}

static uint64_t m4u_read(void *opaque, hwaddr offset, unsigned size)
{
    MT8171M4UState *s = opaque;
    return s->regs[offset / 0x1000][(offset % 0x1000) / 4];
}

static void m4u_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    MT8171M4UState *s = opaque;
    unsigned bank = offset / 0x1000;
    uint32_t *r = s->regs[bank];
    unsigned reg = offset % 0x1000;

    if (reg == 0x60 || (reg >= 0x130 && reg <= 0x154)) {
        return; /* outstanding count, faults and diagnostic addresses */
    }
    if (reg == 0x20) {
        if (value & 3) {
            r[0x12c / 4] = 1;
        }
        return; /* invalidate command strobe */
    }
    if (reg == 0x120 && (value & (1 << 12))) {
        r[0x130 / 4] = r[0x134 / 4] = 0;
        value &= ~(1 << 12);
    }
    if (reg == 0x12c) {
        r[reg / 4] &= value; /* software writes zero after CPE completion */
    } else {
        r[reg / 4] = value;
    }
    m4u_irq(s, bank);
}

static hwaddr m4u_pte_pa(uint32_t pte, uint32_t mask)
{
    return (pte & mask) | ((hwaddr)!!(pte & (1 << 9)) << 32) |
           ((hwaddr)!!(pte & (1 << 4)) << 33) |
           ((hwaddr)!!(pte & (1 << 5)) << 34);
}

static uint32_t m4u_read_pte(hwaddr address, bool hardware, MemTxResult *result)
{
    if (hardware) {
        return address_space_ldl_le(&address_space_memory, address,
                                   MEMTXATTRS_UNSPECIFIED, result);
    }
    hwaddr translated, length = 4;
    MemoryRegion *region = address_space_translate(&address_space_memory,
             address, &translated, &length, false, MEMTXATTRS_UNSPECIFIED);
    if (!memory_region_is_ram(region) || length < 4) {
        *result = MEMTX_DECODE_ERROR;
        return 0;
    }
    *result = MEMTX_OK;
    return ldl_le_p((uint8_t *)memory_region_get_ram_ptr(region) + translated);
}

static bool m4u_translate(MT8171M4UState *s, unsigned bank, uint16_t stream,
                         hwaddr iova, bool write, hwaddr *physical, size_t *span,
                         bool report_fault)
{
    MemTxResult result;
    uint32_t pte, mask, *r;
    hwaddr table;
    unsigned layer = 0;

    if (bank >= 5) {
        return false;
    }
    r = s->regs[bank];
    table = (r[0] & 0xffffff80) | ((hwaddr)(r[0] & 7) << 32);
    if (iova >= (1ULL << 34) || !table) {
        goto fault;
    }
    pte = m4u_read_pte(table + (iova >> 20) * 4, report_fault, &result);
    if (result != MEMTX_OK || !(pte & 3)) {
        goto fault;
    }
    if ((pte & 3) == 1) {
        layer = 1;
        table = m4u_pte_pa(pte, 0xfffffc00);
        pte = m4u_read_pte(table + ((iova >> 12) & 255) * 4,
                          report_fault, &result);
        if (result != MEMTX_OK || !(pte & 3)) {
            goto fault;
        }
        mask = (pte & 3) == 1 ? 0xffff0000 : 0xfffff000;
    } else if ((pte & 3) == 2) {
        mask = (pte & (1 << 18)) ? 0xff000000 : 0xfff00000;
    } else {
        goto fault;
    }
    /* This MTK format uses NO_PERMS: AP bits carry extended PA instead. */
    *physical = m4u_pte_pa(pte, mask) | (iova & (uint32_t)~mask);
    *span = (uint64_t)(uint32_t)~mask + 1 - (iova & (uint32_t)~mask);
    return true;

fault:
    if (!report_fault) {
        return false;
    }
    r[0x134 / 4] |= 1;
    r[0x13c / 4] = (iova & 0xfffff000) | (((iova >> 32) & 7) << 9) |
                    (write ? 2 : 0) | layer;
    r[0x140 / 4] = 0;
    r[0x150 / 4] = stream;
    m4u_irq(s, bank);
    return false;
}

bool mt8171_m4u_translate(MT8171M4UState *s, unsigned bank, uint16_t stream,
                         hwaddr iova, bool write, hwaddr *physical, size_t *span)
{
    return m4u_translate(s, bank, stream, iova, write, physical, span, true);
}

bool mt8171_m4u_debug_translate(MT8171M4UState *s, unsigned bank, hwaddr iova,
                               hwaddr *physical, size_t *span)
{
    return m4u_translate(s, bank, 0, iova, false, physical, span, false);
}

static const MemoryRegionOps m4u_ops = {
    .read = m4u_read,
    .write = m4u_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void m4u_reset(DeviceState *dev)
{
    MT8171M4UState *s = MT8171_M4U(dev);
    memset(s->regs, 0, sizeof(s->regs));
    for (unsigned i = 0; i < 5; i++) {
        qemu_irq_lower(s->irq[i]);
    }
}

static void m4u_init(Object *obj)
{
    MT8171M4UState *s = MT8171_M4U(obj);
    memory_region_init_io(&s->iomem, obj, &m4u_ops, s, TYPE_MT8171_M4U, 0x5000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (unsigned i = 0; i < 5; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
}

static void m4u_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), m4u_reset);
}

static const TypeInfo m4u_type = {
    .name = TYPE_MT8171_M4U,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MT8171M4UState),
    .instance_init = m4u_init,
    .class_init = m4u_class_init,
};
static void m4u_register(void)
{
    type_register_static(&m4u_type);
}
type_init(m4u_register);
