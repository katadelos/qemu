/*
 * Amazon/Lab126 Coloursoft (Bellatrix4) board emulation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <libfdt.h>
#include "hw/arm/mt8113.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/bd71828.h"
#include "hw/i2c/fp9935.h"
#include "hw/i2c/fp9967.h"
#include "hw/i2c/ft5536g.h"
#include "hw/i2c/lis2du12.h"
#include "hw/i2c/max20342.h"
#include "hw/sd/sd.h"
#include "qapi/error.h"
#include "exec/target_page.h"
#include "exec/tb-flush.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/reset.h"
#include "system/tcg.h"

#define COLOURSOFT_HANDOFF_ADDR      0x40001000
#define COLOURSOFT_BOOTARGS_ADDR     0x40000100
#define COLOURSOFT_UBOOT_ADDR        0x44e00000

#define COLOURSOFT_BL2_GFH_OFFSET    0x800
#define COLOURSOFT_BL2_LOAD_ADDR     0x00110d00
#define COLOURSOFT_BL2_ENTRY         0x00111000

#define COLOURSOFT_IDME_SIZE         (10 * 512)
#define COLOURSOFT_IDME_VERSION      "2.1"

#define COLOURSOFT_CFA_MODULE_START  0xbf000000
#define COLOURSOFT_CFA_MODULE_END    0xc0000000
#define COLOURSOFT_CFA_KNOWN_ADDR    0xbf055000
#define COLOURSOFT_CFA_SCAN_PAGES    512
#define COLOURSOFT_CFA_DATA_SCAN_SIZE 0x20000
#define COLOURSOFT_CFA_RETRY_NS      (100 * SCALE_MS)

#define TYPE_COLOURSOFT_MACHINE MACHINE_TYPE_NAME("mt8113-coloursoft")
OBJECT_DECLARE_SIMPLE_TYPE(ColoursoftMachineState, COLOURSOFT_MACHINE)

typedef struct ColoursoftMachineState {
    MachineState parent_obj;
    MT8113State *soc;
    char *bl2;
    char *tee;
    char *quickboot;
    char *boot_stage;
    char *idme_board_id;
    char *idme_serial;
    char *idme_mac;
    char *idme_mfg;
    char *idme_device_type;
    char *idme_bootmode;
    char *idme_postmode;
    char *idme_fos_flags;
    char *idme_hwid;
    char *idme_bcm_stress;
    QEMUTimer *cfa_bypass_timer;
    bool cfa_bypass;
    bool cfa_bypass_inflight;
    bool cfa_bypass_applied;
    uint32_t cfa_bypass_address;
    uint32_t cfa_bypass_workers_address;
    uint32_t cfa_scan_address;
    uint64_t cfa_bypass_attempts;
} ColoursoftMachineState;

typedef struct ColoursoftIdmeField {
    const char *name;
    const char *value;
    size_t size;
    bool exportable;
} ColoursoftIdmeField;

/*
 * The stock CFA module converts the complete 1272x1696 RGBA source frame in
 * two kernel threads before every e-ink update.  That NEON-heavy software
 * conversion is both unnecessary for QEMU's RGB console and prohibitively
 * slow under TCG.  Match the stock function prologue before replacing it with
 * a hook that reports its actual input pointer to HWTCON and returns.  This
 * preserves the driver's first/alternate fbdev page selection without running
 * the converter whose calling contract was verified from shipped source.
 */
static const uint8_t coloursoft_cfa_signature[] = {
    0xf0, 0x43, 0x2d, 0xe9, /* push {r4-r9, lr} */
    0x00, 0x80, 0xa0, 0xe1,
    0x01, 0x50, 0xa0, 0xe1,
    0x24, 0xd0, 0x4d, 0xe2,
    0x02, 0x60, 0xa0, 0xe1,
    0x03, 0x90, 0xa0, 0xe1,
    0x40, 0x70, 0x9d, 0xe5,
    0x44, 0x40, 0x9d, 0xe5,
};

static const uint8_t coloursoft_cfa_hook[] = {
    0xd0, 0xc0, 0x00, 0xe3, /* movw ip, #0x00d0 */
    0x00, 0xc0, 0x4c, 0xe3, /* movt ip, #0xc000 */
    0x00, 0x00, 0x8c, 0xe5, /* str r0, [ip]       source pointer */
    0x04, 0x20, 0x8c, 0xe5, /* str r2, [ip, #4]   source width */
    0x08, 0x30, 0x8c, 0xe5, /* str r3, [ip, #8]   source height */
    0x00, 0x10, 0x9d, 0xe5, /* ldr r1, [sp]       left */
    0x0c, 0x10, 0x8c, 0xe5, /* str r1, [ip, #12] */
    0x04, 0x10, 0x9d, 0xe5, /* ldr r1, [sp, #4]   top */
    0x10, 0x10, 0x8c, 0xe5, /* str r1, [ip, #16] */
    0x08, 0x10, 0x9d, 0xe5, /* ldr r1, [sp, #8]   update width */
    0x14, 0x10, 0x8c, 0xe5, /* str r1, [ip, #20] */
    0x0c, 0x10, 0x9d, 0xe5, /* ldr r1, [sp, #12]  update height */
    0x18, 0x10, 0x8c, 0xe5, /* str r1, [ip, #24] */
    0x10, 0x10, 0x9d, 0xe5, /* ldr r1, [sp, #16]  rotation */
    0x1c, 0x10, 0x8c, 0xe5, /* str r1, [ip, #28] */
    0x1e, 0xff, 0x2f, 0xe1, /* bx lr */
};

/*
 * Stock cfa.ko .data starts with cfa_handle_threads followed by its CFA
 * method and base-LUT configuration.  Match enough immutable data to resolve
 * the independently relocated writable section without depending on a fixed
 * module layout.
 */
static const char coloursoft_cfa_data_path[] =
    "/data/init_bin/cfa/base.bin.gz";

static bool coloursoft_cfa_signature_at(CPUState *cpu, uint32_t address)
{
    uint8_t signature[sizeof(coloursoft_cfa_signature)];

    return cpu_memory_rw_debug(cpu, address, signature, sizeof(signature),
                               false) == 0 &&
           !memcmp(signature, coloursoft_cfa_signature, sizeof(signature));
}

static uint32_t coloursoft_find_cfa_signature(ColoursoftMachineState *cms,
                                               CPUState *cpu)
{
    size_t page_size = TARGET_PAGE_SIZE;
    g_autofree uint8_t *page = g_malloc(page_size);

    if (coloursoft_cfa_signature_at(cpu, COLOURSOFT_CFA_KNOWN_ADDR)) {
        return COLOURSOFT_CFA_KNOWN_ADDR;
    }

    for (unsigned page_index = 0;
         page_index < COLOURSOFT_CFA_SCAN_PAGES; page_index++) {
        uint32_t address = cms->cfa_scan_address;

        cms->cfa_scan_address += page_size;
        if (cms->cfa_scan_address >= COLOURSOFT_CFA_MODULE_END) {
            cms->cfa_scan_address = COLOURSOFT_CFA_MODULE_START;
        }
        if (address == COLOURSOFT_CFA_KNOWN_ADDR ||
            cpu_memory_rw_debug(cpu, address, page, page_size, false)) {
            continue;
        }
        for (unsigned offset = 0;
             offset <= page_size - sizeof(coloursoft_cfa_signature);
             offset += sizeof(uint32_t)) {
            if (!memcmp(page + offset, coloursoft_cfa_signature,
                        sizeof(coloursoft_cfa_signature))) {
                return address + offset;
            }
        }
    }
    return 0;
}

static uint32_t coloursoft_find_cfa_workers(CPUState *cpu,
                                             uint32_t text_address)
{
    size_t page_size = TARGET_PAGE_SIZE;
    g_autofree uint8_t *page = g_malloc(page_size);
    uint32_t start = text_address & TARGET_PAGE_MASK;

    for (uint32_t address = start;
         address - start < COLOURSOFT_CFA_DATA_SCAN_SIZE;
         address += page_size) {
        if (cpu_memory_rw_debug(cpu, address, page, page_size, false)) {
            continue;
        }
        for (unsigned offset = 0x20;
             offset <= page_size - sizeof(coloursoft_cfa_data_path);
             offset += sizeof(uint32_t)) {
            uint8_t *candidate = page + offset - 0x20;

            /* Relocated pointers occupy the deliberately ignored words. */
            if (!memcmp(page + offset, coloursoft_cfa_data_path,
                        sizeof(coloursoft_cfa_data_path)) &&
                ldl_le_p(candidate) == 2 &&
                ldl_le_p(candidate + 0x0c) == 9 &&
                ldl_le_p(candidate + 0x1c) == 0x01400000) {
                return address + offset - 0x20;
            }
        }
    }
    return 0;
}

static void coloursoft_cfa_bypass_on_cpu(CPUState *cpu, run_on_cpu_data data)
{
    ColoursoftMachineState *cms = data.host_ptr;
    uint32_t address = coloursoft_find_cfa_signature(cms, cpu);
    uint32_t workers_address = address ?
        coloursoft_find_cfa_workers(cpu, address) : 0;
    const uint32_t one = 1;

    if (address && workers_address &&
        !cpu_memory_rw_debug(cpu, workers_address, (void *)&one,
                             sizeof(one), true) &&
        !cpu_memory_rw_debug(cpu, address,
                             (void *)coloursoft_cfa_hook,
                             sizeof(coloursoft_cfa_hook), true)) {
        /* This callback holds every vCPU outside translated code. */
        if (tcg_enabled()) {
            tb_flush__exclusive_or_serial();
        }
        qatomic_set(&cms->cfa_bypass_address, address);
        qatomic_set(&cms->cfa_bypass_workers_address, workers_address);
        qatomic_set(&cms->cfa_bypass_applied, true);
        info_report("Coloursoft CFA bypass applied at 0x%08x (one worker at "
                    "0x%08x)", address, workers_address);
    }

    qatomic_set(&cms->cfa_bypass_inflight, false);
    if (!qatomic_read(&cms->cfa_bypass_applied) &&
        qatomic_read(&cms->cfa_bypass)) {
        timer_mod_ns(cms->cfa_bypass_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     COLOURSOFT_CFA_RETRY_NS);
    }
}

static void coloursoft_cfa_bypass_tick(void *opaque)
{
    ColoursoftMachineState *cms = opaque;

    if (!qatomic_read(&cms->cfa_bypass) ||
        qatomic_read(&cms->cfa_bypass_applied) ||
        qatomic_xchg(&cms->cfa_bypass_inflight, true)) {
        return;
    }
    if (!first_cpu) {
        qatomic_set(&cms->cfa_bypass_inflight, false);
        timer_mod_ns(cms->cfa_bypass_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     COLOURSOFT_CFA_RETRY_NS);
        return;
    }

    qatomic_inc(&cms->cfa_bypass_attempts);
    async_safe_run_on_cpu(first_cpu, coloursoft_cfa_bypass_on_cpu,
                          RUN_ON_CPU_HOST_PTR(cms));
}

static void coloursoft_cfa_bypass_reset(void *opaque)
{
    ColoursoftMachineState *cms = opaque;

    timer_del(cms->cfa_bypass_timer);
    qatomic_set(&cms->cfa_bypass_inflight, false);
    qatomic_set(&cms->cfa_bypass_applied, false);
    qatomic_set(&cms->cfa_bypass_address, 0);
    qatomic_set(&cms->cfa_bypass_workers_address, 0);
    qatomic_set(&cms->cfa_bypass_attempts, 0);
    cms->cfa_scan_address = COLOURSOFT_CFA_MODULE_START;
    if (qatomic_read(&cms->cfa_bypass)) {
        timer_mod_ns(cms->cfa_bypass_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     COLOURSOFT_CFA_RETRY_NS);
    }
}

static void coloursoft_firmware_reset(void *opaque)
{
    ARMCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

/*
 * Enter the 32-bit BL2 or U-Boot payload from the Cortex-A53 reset state.
 * The real BootROM/BL2 performs this EL3 width transition.
 */
static const uint32_t coloursoft_handoff[] = {
    0x58000180, /* ldr x0, bootargs */
    0xd2803a61, /* mov x1, #0x1d3 (AArch32 SVC, AIF masked) */
    0xd51e4001, /* msr spsr_el3, x1 */
    0xd53e1101, /* mrs x1, scr_el3 */
    0x9275f821, /* bic x1, x1, #SCR_RW */
    0xb2400021, /* orr x1, x1, #SCR_NS */
    0xd51e1101, /* msr scr_el3, x1 */
    0x580000e1, /* ldr x1, entry */
    0xd51e4021, /* msr elr_el3, x1 */
    0xd5033fdf, /* isb */
    0xd69f03e0, /* eret */
    0xd503201f, /* nop/alignment */
    COLOURSOFT_BOOTARGS_ADDR, 0,
    COLOURSOFT_UBOOT_ADDR, 0,
};

static void coloursoft_load_fit_image(const char *filename,
                                      const char *node, const char *name)
{
    g_autofree uint8_t *fit = NULL;
    gsize fit_size;
    const void *data;
    const fdt32_t *load_prop;
    int data_size;
    int load_size;
    int image;
    hwaddr load;

    if (!g_file_get_contents(filename, (char **)&fit, &fit_size, NULL) ||
        fdt_check_header(fit)) {
        error_report("%s is not a valid FIT image", filename);
        exit(EXIT_FAILURE);
    }
    image = fdt_path_offset(fit, node);
    data = fdt_getprop(fit, image, "data", &data_size);
    load_prop = fdt_getprop(fit, image, "load", &load_size);
    if (image < 0 || !data || data_size <= 0 || !load_prop || load_size != 4) {
        error_report("%s has no loadable %s payload", filename, node);
        exit(EXIT_FAILURE);
    }
    load = fdt32_to_cpu(*load_prop);
    if (load < MT8113_RAM_BASE ||
        load + data_size > MT8113_RAM_BASE + MT8113_RAM_MAX) {
        error_report("%s payload has invalid load range", name);
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed(name, data, data_size, load);
}

static void coloursoft_load_bl2(ColoursoftMachineState *cms)
{
    g_autofree uint8_t *image = NULL;
    gsize size;

    if (!cms->bl2) {
        error_report("boot-stage=bl2 requires the bl2 property");
        exit(EXIT_FAILURE);
    }
    if (!g_file_get_contents(cms->bl2, (char **)&image, &size, NULL) ||
        size <= COLOURSOFT_BL2_GFH_OFFSET) {
        error_report("unable to load Coloursoft BL2 '%s'", cms->bl2);
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("coloursoft.bl2",
                       image + COLOURSOFT_BL2_GFH_OFFSET,
                       size - COLOURSOFT_BL2_GFH_OFFSET,
                       COLOURSOFT_BL2_LOAD_ADDR);
}

static void coloursoft_load_firmware(ColoursoftMachineState *cms,
                                     MachineState *machine)
{
    uint32_t handoff[ARRAY_SIZE(coloursoft_handoff)];
    g_autofree uint8_t *bootargs = g_malloc0(0x2dc);
    hwaddr entry = COLOURSOFT_UBOOT_ADDR;

    if (!machine->firmware) {
        error_report("Coloursoft requires -bios with the stock u-boot.bin FIT");
        exit(EXIT_FAILURE);
    }

    coloursoft_load_fit_image(machine->firmware, "/images/kernel",
                              "coloursoft.u-boot");
    if (cms->tee) {
        coloursoft_load_fit_image(cms->tee, "/images/tee", "coloursoft.tee");
        coloursoft_load_fit_image(cms->tee, "/images/trustedos",
                                  "coloursoft.trustedos");
    }
    if (cms->quickboot) {
        coloursoft_load_fit_image(cms->quickboot, "/images/fbios",
                                  "coloursoft.fbios");
        coloursoft_load_fit_image(cms->quickboot, "/images/sbios",
                                  "coloursoft.sbios");
    }

    if (!strcmp(cms->boot_stage, "bl2")) {
        coloursoft_load_bl2(cms);
        entry = COLOURSOFT_BL2_ENTRY;
    } else if (strcmp(cms->boot_stage, "u-boot")) {
        error_report("invalid boot-stage '%s' (expected u-boot or bl2)",
                     cms->boot_stage);
        exit(EXIT_FAILURE);
    }

    stl_le_p(bootargs, 0x504c504c);
    stl_le_p(bootargs + 4, machine->ram_size);
    stl_le_p(bootargs + 0x254, 0x504c504c);
    stl_le_p(bootargs + 0x2d8, 0x504c504c);
    rom_add_blob_fixed("coloursoft.bootargs", bootargs, 0x2dc,
                       COLOURSOFT_BOOTARGS_ADDR);

    memcpy(handoff, coloursoft_handoff, sizeof(handoff));
    handoff[14] = entry;
    rom_add_blob_fixed("coloursoft.handoff", handoff, sizeof(handoff),
                       COLOURSOFT_HANDOFF_ADDR);
}

static void coloursoft_populate_idme(DeviceState *card,
                                     ColoursoftMachineState *cms)
{
    g_autofree uint8_t *contents = g_malloc0(COLOURSOFT_IDME_SIZE);
    ColoursoftIdmeField fields[] = {
        { "board_id",       cms->idme_board_id,   16, true },
        { "serial",         cms->idme_serial,     16, true },
        { "mac_addr",       cms->idme_mac,        16, true },
        { "mac_sec",        "0",                 32, true },
        { "bt_mac_addr",    "0",                 16, true },
        { "product_name",   "se9_p",             32, true },
        { "productid",      "0",                 32, true },
        { "productid2",     "0",                 32, true },
        { "region",         "US",                 4, true },
        { "bootmode",       cms->idme_bootmode,    4, true },
        { "postmode",       cms->idme_postmode,    4, true },
        { "bootcount",      "0",                  8, true },
        { "manufacturing",  cms->idme_mfg,       512, true },
        { "unlock_code",    "",                1024, true },
        { "device_type_id", cms->idme_device_type, 32, true },
        { "dev_flags",      "0",                  8, true },
        { "fos_flags",      cms->idme_fos_flags,   8, true },
        { "usr_flags",      "0",                  8, true },
        { "hwid",           cms->idme_hwid,        4, true },
        { "bcm_stress",     cms->idme_bcm_stress, 1024, true },
    };
    uint8_t *cursor = contents;

    memcpy(cursor, "beefdeed", 8);
    memcpy(cursor + 8, COLOURSOFT_IDME_VERSION,
           sizeof(COLOURSOFT_IDME_VERSION));
    stl_le_p(cursor + 12, ARRAY_SIZE(fields));
    cursor += 16;

    for (size_t i = 0; i < ARRAY_SIZE(fields); i++) {
        ColoursoftIdmeField *field = &fields[i];
        size_t length = strlen(field->value);
        size_t item_size = ROUND_UP(28 + field->size, 4);

        if (strlen(field->name) > 16 || !g_str_is_ascii(field->value) ||
            length > field->size || cursor + item_size > contents +
            COLOURSOFT_IDME_SIZE) {
            error_report("IDME %s must contain at most %zu ASCII bytes",
                         field->name, field->size);
            exit(EXIT_FAILURE);
        }
        memcpy(cursor, field->name, strlen(field->name));
        stl_le_p(cursor + 16, field->size);
        stl_le_p(cursor + 20, field->exportable);
        stl_le_p(cursor + 24, 0444);
        memcpy(cursor + 28, field->value, length);
        cursor += item_size;
    }

    /* UFBL's MediaTek platform reads ten blocks from boot area 2, block 0. */
    emmc_boot_partition_write(card, 2, 0, contents, COLOURSOFT_IDME_SIZE,
                              &error_fatal);
}

static void coloursoft_write_boot_component(DeviceState *card,
                                             unsigned int partition,
                                             uint64_t start_block,
                                             uint64_t block_count,
                                             const char *filename,
                                             const char *name)
{
    g_autofree uint8_t *contents = NULL;
    gsize size;

    if (!filename) {
        return;
    }
    if (!g_file_get_contents(filename, (char **)&contents, &size, NULL)) {
        error_report("unable to load Coloursoft %s '%s'", name, filename);
        exit(EXIT_FAILURE);
    }
    if (size > block_count * 512) {
        error_report("Coloursoft %s exceeds its eMMC boot-area slot", name);
        exit(EXIT_FAILURE);
    }
    emmc_boot_partition_write(card, partition, start_block * 512,
                              contents, size, &error_fatal);
}

static void coloursoft_attach_emmc(ColoursoftMachineState *cms)
{
    MachineState *machine = MACHINE(cms);
    DriveInfo *di = drive_get(IF_SD, 0, 0);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus = qdev_get_child_bus(DEVICE(&cms->soc->msdc0), "sd-bus");
    DeviceState *card = qdev_new(TYPE_EMMC);

    qdev_prop_set_uint64(card, "boot-partition-size", 4 * MiB);
    qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);

    /* Fixed boot-area slots from stock U-Boot's g_custom_partitions table. */
    coloursoft_write_boot_component(card, 1, 0x000, 0x200,
                                    cms->bl2, "BL2");
    coloursoft_write_boot_component(card, 1, 0x400, 0x800,
                                    machine->firmware, "U-Boot");
    coloursoft_write_boot_component(card, 1, 0xc00, 0x600,
                                    cms->tee, "TEE");
    coloursoft_write_boot_component(card, 2, 0x400, 0x400,
                                    cms->quickboot, "quickboot");
    coloursoft_populate_idme(card, cms);
    object_unref(OBJECT(card));
}

static void coloursoft_init(MachineState *machine)
{
    ColoursoftMachineState *cms = COLOURSOFT_MACHINE(machine);
    MT8113State *soc;
    I2CSlave *fp9935;
    I2CSlave *bd71828;
    I2CSlave *ft5536g;
    I2CSlave *lis2du12;

    if (machine->ram_size > MT8113_RAM_MAX) {
        error_report("Coloursoft RAM exceeds the MT8113 2 GiB address map");
        exit(EXIT_FAILURE);
    }

    soc = MT8113(object_new(TYPE_MT8113));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_property_set_uint(OBJECT(soc), "reset-vector",
                             COLOURSOFT_HANDOFF_ADDR, &error_fatal);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);
    cms->soc = soc;
    for (int cpu = 0; cpu < MT8113_NUM_CPUS; cpu++) {
        qemu_register_reset(coloursoft_firmware_reset, &soc->cpu[cpu]);
    }

    i2c_slave_create_simple(soc->i2c[0].bus, TYPE_FP9967, 0x26);
    lis2du12 = i2c_slave_create_simple(soc->i2c[0].bus,
                                      TYPE_LIS2DU12, 0x19);
    fp9935 = i2c_slave_create_simple(soc->i2c[0].bus, TYPE_FP9935, 0x30);
    i2c_slave_create_simple(soc->i2c[0].bus, TYPE_MAX20342, 0x35);
    bd71828 = i2c_slave_create_simple(soc->i2c[1].bus, TYPE_BD71828, 0x4b);
    i2c_slave_create_simple(soc->i2c[1].bus, TYPE_BD71828, 0x4d);
    ft5536g = i2c_slave_create_simple(soc->i2c[2].bus, TYPE_FT5536G, 0x38);

    qdev_connect_gpio_out_named(DEVICE(bd71828), "gpio", 1,
        qdev_get_gpio_in_named(DEVICE(fp9935), "enable", 0));
    qdev_connect_gpio_out_named(DEVICE(fp9935), "power-good", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 1));
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 1),
                 0);
    qdev_connect_gpio_out_named(DEVICE(ft5536g), "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 15));
    qdev_connect_gpio_out_named(DEVICE(&soc->gpio), "gpio-out", 20,
        qdev_get_gpio_in_named(DEVICE(ft5536g), "reset", 0));
    qdev_connect_gpio_out_named(DEVICE(lis2du12), "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 70));
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&soc->gpio), "gpio-in", 70),
                 0);

    memory_region_add_subregion(get_system_memory(), MT8113_RAM_BASE,
                                machine->ram);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&soc->hwtcon), 2,
                            MT8113_HWTCON_CFA_MAILBOX_ADDR, 1);
    coloursoft_attach_emmc(cms);
    coloursoft_load_firmware(cms, machine);
    qemu_register_reset(coloursoft_cfa_bypass_reset, cms);
}

static char *coloursoft_get_string(char **value)
{
    return g_strdup(*value);
}

static void coloursoft_set_string(char **field, const char *value)
{
    g_free(*field);
    *field = g_strdup(value);
}

#define COLOURSOFT_STRING_PROPERTY(_member)                              \
    static char *coloursoft_get_##_member(Object *obj, Error **errp)     \
    {                                                                    \
        return coloursoft_get_string(&COLOURSOFT_MACHINE(obj)->_member); \
    }                                                                    \
    static void coloursoft_set_##_member(Object *obj, const char *value, \
                                         Error **errp)                    \
    {                                                                    \
        coloursoft_set_string(&COLOURSOFT_MACHINE(obj)->_member, value); \
    }

COLOURSOFT_STRING_PROPERTY(bl2)
COLOURSOFT_STRING_PROPERTY(tee)
COLOURSOFT_STRING_PROPERTY(quickboot)
COLOURSOFT_STRING_PROPERTY(boot_stage)
COLOURSOFT_STRING_PROPERTY(idme_board_id)
COLOURSOFT_STRING_PROPERTY(idme_serial)
COLOURSOFT_STRING_PROPERTY(idme_mac)
COLOURSOFT_STRING_PROPERTY(idme_mfg)
COLOURSOFT_STRING_PROPERTY(idme_device_type)
COLOURSOFT_STRING_PROPERTY(idme_bootmode)
COLOURSOFT_STRING_PROPERTY(idme_postmode)
COLOURSOFT_STRING_PROPERTY(idme_fos_flags)
COLOURSOFT_STRING_PROPERTY(idme_hwid)
COLOURSOFT_STRING_PROPERTY(idme_bcm_stress)

static bool coloursoft_get_cfa_bypass(Object *obj, Error **errp)
{
    return qatomic_read(&COLOURSOFT_MACHINE(obj)->cfa_bypass);
}

static void coloursoft_set_cfa_bypass(Object *obj, bool value, Error **errp)
{
    ColoursoftMachineState *cms = COLOURSOFT_MACHINE(obj);

    qatomic_set(&cms->cfa_bypass, value);
    if (!cms->cfa_bypass_timer) {
        return;
    }
    if (!value) {
        timer_del(cms->cfa_bypass_timer);
    } else if (!qatomic_read(&cms->cfa_bypass_applied) &&
               !qatomic_read(&cms->cfa_bypass_inflight)) {
        timer_mod_ns(cms->cfa_bypass_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     COLOURSOFT_CFA_RETRY_NS);
    }
}

static bool coloursoft_get_cfa_bypass_applied(Object *obj, Error **errp)
{
    return qatomic_read(&COLOURSOFT_MACHINE(obj)->cfa_bypass_applied);
}

static void coloursoft_machine_instance_init(Object *obj)
{
    ColoursoftMachineState *cms = COLOURSOFT_MACHINE(obj);

    cms->boot_stage = g_strdup("u-boot");
    /* Bytes 3..5 are a Sangria Color tattoo recognized by stock U-Boot. */
    cms->idme_board_id = g_strdup("0003RW0000000000");
    /* Modern serials encode the three-byte device code at bytes 3..5. */
    cms->idme_serial = g_strdup("G003H9000000000");
    cms->idme_mac = g_strdup("020000000005");
    cms->idme_mfg = g_strdup("COLOURSOFTQEMU00000");
    cms->idme_device_type = g_strdup("3H9");
    cms->idme_bootmode = g_strdup("1");
    cms->idme_postmode = g_strdup("0");
    cms->idme_fos_flags = g_strdup("0");
    cms->idme_hwid = g_strdup("0");
    cms->idme_bcm_stress = g_strdup("");
    cms->cfa_bypass = true;
    cms->cfa_scan_address = COLOURSOFT_CFA_MODULE_START;
    cms->cfa_bypass_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          coloursoft_cfa_bypass_tick, cms);
    object_property_add_bool(obj, "cfa-bypass-applied",
                             coloursoft_get_cfa_bypass_applied, NULL);
    object_property_add_uint32_ptr(obj, "cfa-bypass-address",
                                   &cms->cfa_bypass_address,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "cfa-bypass-workers-address",
                                   &cms->cfa_bypass_workers_address,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "cfa-bypass-attempts",
                                   &cms->cfa_bypass_attempts,
                                   OBJ_PROP_FLAG_READ);
}

static void coloursoft_machine_instance_finalize(Object *obj)
{
    ColoursoftMachineState *cms = COLOURSOFT_MACHINE(obj);

    if (cms->soc) {
        for (int cpu = 0; cpu < MT8113_NUM_CPUS; cpu++) {
            qemu_unregister_reset(coloursoft_firmware_reset,
                                  &cms->soc->cpu[cpu]);
        }
    }
    qemu_unregister_reset(coloursoft_cfa_bypass_reset, cms);
    timer_free(cms->cfa_bypass_timer);
    g_free(cms->bl2);
    g_free(cms->tee);
    g_free(cms->quickboot);
    g_free(cms->boot_stage);
    g_free(cms->idme_board_id);
    g_free(cms->idme_serial);
    g_free(cms->idme_mac);
    g_free(cms->idme_mfg);
    g_free(cms->idme_device_type);
    g_free(cms->idme_bootmode);
    g_free(cms->idme_postmode);
    g_free(cms->idme_fos_flags);
    g_free(cms->idme_hwid);
    g_free(cms->idme_bcm_stress);
}

#define COLOURSOFT_ADD_PROPERTY(_name, _member)                         \
    object_class_property_add_str(oc, _name,                            \
                                  coloursoft_get_##_member,             \
                                  coloursoft_set_##_member)

static void coloursoft_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Amazon Kindle Coloursoft (MediaTek MT8113/Bellatrix4)";
    mc->init = coloursoft_init;
    mc->max_cpus = MT8113_NUM_CPUS;
    mc->default_cpus = MT8113_NUM_CPUS;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a53");
    mc->default_ram_size = 2 * GiB;
    mc->default_ram_id = "coloursoft.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;

    COLOURSOFT_ADD_PROPERTY("bl2", bl2);
    COLOURSOFT_ADD_PROPERTY("tee", tee);
    COLOURSOFT_ADD_PROPERTY("quickboot", quickboot);
    COLOURSOFT_ADD_PROPERTY("boot-stage", boot_stage);
    COLOURSOFT_ADD_PROPERTY("idme-board-id", idme_board_id);
    COLOURSOFT_ADD_PROPERTY("idme-serial", idme_serial);
    COLOURSOFT_ADD_PROPERTY("idme-mac", idme_mac);
    COLOURSOFT_ADD_PROPERTY("idme-mfg", idme_mfg);
    COLOURSOFT_ADD_PROPERTY("idme-device-type", idme_device_type);
    COLOURSOFT_ADD_PROPERTY("idme-bootmode", idme_bootmode);
    COLOURSOFT_ADD_PROPERTY("idme-postmode", idme_postmode);
    COLOURSOFT_ADD_PROPERTY("idme-fos-flags", idme_fos_flags);
    COLOURSOFT_ADD_PROPERTY("idme-hwid", idme_hwid);
    COLOURSOFT_ADD_PROPERTY("idme-bcm-stress", idme_bcm_stress);
    object_class_property_add_bool(oc, "cfa-bypass",
                                   coloursoft_get_cfa_bypass,
                                   coloursoft_set_cfa_bypass);
}

static const TypeInfo coloursoft_machine_type = {
    .name = TYPE_COLOURSOFT_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(ColoursoftMachineState),
    .instance_init = coloursoft_machine_instance_init,
    .instance_finalize = coloursoft_machine_instance_finalize,
    .class_init = coloursoft_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void coloursoft_machine_register_types(void)
{
    type_register_static(&coloursoft_machine_type);
}
type_init(coloursoft_machine_register_types)
