/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_MT8171_CONSYS_H
#define HW_MISC_MT8171_CONSYS_H
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#define TYPE_MT8171_CONSYS "mt8171-consys"
OBJECT_DECLARE_SIMPLE_TYPE(MT8171ConsysState, MT8171_CONSYS)
bool mt8171_consys_mcu_reg(MT8171ConsysState *s, uint32_t address,
                          uint32_t *value, uint32_t mask, bool write);
bool mt8171_consys_mcu_adie(MT8171ConsysState *s, uint32_t *identity);
bool mt8171_consys_boot_ready(MT8171ConsysState *s);
/* Model-owned scratch in the last page of the DT's 8 MiB CONSYS EMI.
 * This offset is an emulator protocol allocation, not a physical RF layout. */
#define MT8171_SYNTH_CAL_OFFSET 0x7ff000
bool mt8171_consys_calibration_memory(MT8171ConsysState *s, void *data,
                                     unsigned size, bool write);
struct MT8171ConsysState {
    SysBusDevice parent_obj;
    MemoryRegion iomem, reset_iomem;
    uint32_t regs[0x100000 / 4];
    /* Shared SPM owner supplies its register bank; primary/secondary rails
     * are requested by CONN_PWR_CON304 bits2/3. */
    uint32_t *spm_regs;
    uint32_t *infracfg_regs;
    uint32_t reset_control;
    QEMUTimer *spi_timer, *rom_timer;
    char *mcu_firmware;
    uint8_t *mcu_image;
    size_t mcu_image_size;
    bool rom_attempted;
    qemu_irq irq[3];
    qemu_irq subsystem_reset;
    uint64_t identity_reads, spi_transactions, unsupported_spi, reset_assertions;
    uint64_t rom_boots, firmware_mismatches, firmware_bytes_verified;
};
#endif
