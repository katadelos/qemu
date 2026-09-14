/*
 * Amazon MediaTek MT8115 PA6/CS8 platform.
 * Shared SoC model; fitted Calvados/CS8 and Paloma/PA6 board descriptions
 * are declared separately in mt8115-board.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <libfdt.h>
#include "hw/arm/bsa.h"
#include "hw/arm/mt8115-preload.h"
#include "hw/arm/mt8115-board.h"
#include "hw/arm/machines-qom.h"
#include "hw/char/serial-mm.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/intc/arm_gicv3.h"
#include "hw/i2c/mtk_i2c.h"
#include "hw/i2c/ti-opt-sensors.h"
#include "hw/i2c/parade-tt7010.h"
#include "hw/misc/unimp.h"
#include "hw/misc/mt8171-m4u.h"
#include "hw/misc/mt8171-gce.h"
#include "hw/misc/mt8171-mcupm.h"
#include "hw/misc/mt8171-consys.h"
#include "hw/misc/mt8171-wlan.h"
#include "hw/misc/mt8171-btif.h"
#include "hw/display/mt8171-mdp.h"
#include "hw/display/mt8171-hwtcon.h"
#include "hw/display/mt8171-panel.h"
#include "hw/sd/mtk-msdc.h"
#include "hw/usb/mtu3.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/main-loop.h"
#include "qemu/log.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qobject/qlist.h"
#include "system/block-backend.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/cpregs.h"

#define TYPE_PA6_CS8 MACHINE_TYPE_NAME("mt8115-pa6-cs8")
OBJECT_DECLARE_SIMPLE_TYPE(PA6CS8State, PA6_CS8)

#define PA6_RAM_BASE 0x40000000
#define PA6_NUM_SPIS 640
#define PA6_NUM_CPUS 4

/* BL2 0x204840/0x204af0: fixed allocations, sorted for the free RAM list. */
static const struct {
    uint64_t start, size;
    const char *name;
} pa6_reserved[] = {
    { 0x42fc0000, 0x040000, "atf-log-reserved" },
    { 0x43000000, 0x100000, "atf-reserved-memory" },
    { 0x43100000, 0x600000, "optee-reserved-memory" },
    { 0x44b80000, 0x100000, "dpm-reserved-memory" },
    { 0x45000000, 0x010000, "spmfw-reserved-memory" },
    { 0x46000000, 0x020000, "mcupm-reserved-memory" },
    { 0x47000000, 0x200000, "sspm-reserved-memory" },
    { 0x47200000, 0x200000, "pi-img-reserved-memory" },
    { 0x48100000, 0x010000, "aee_debug_kinfo" },
    { 0x48110000, 0x010000, "minirdump" },
    { 0x49000000, 0x800000, "consys_emi_reserved" },
    { 0x49800000, 0x600000, "shared-dma-pool_wifi-reserve-memory_dma" },
    { 0x6ec00000, 0x080000, "pstore" },
    { 0x70000000, 0x100000, "dfd-mcu-reserved-memory" },
    /* BL2 0x21cd40, allocator 0x20f470: below 0x9fffffff, 64 KiB aligned. */
    { 0x9fee0000, 0x110000, "sspm_ap-shared" },
};

typedef enum PA6ClockKind {
    PA6_CLOCK_TOPCKGEN,
    PA6_CLOCK_APMIXEDSYS,
    PA6_CLOCK_INFRACFG_AO,
    PA6_CLOCK_IIC_WRAP,
    PA6_CLOCK_REGS,
    PA6_SMI_LARB,
    PA6_SMI_COMMON,
    PA6_CLOCK_GATE0,
    PA6_CLOCK_MDP,
    PA6_CLOCK_VDEC,
} PA6ClockKind;

static const struct {
    const char *name;
    hwaddr base;
    PA6ClockKind kind;
    unsigned size;
} pa6_clock_map[] = {
    { "mt8115.topckgen", 0x10000000, PA6_CLOCK_TOPCKGEN },
    { "mt8115.apmixedsys", 0x1000c000, PA6_CLOCK_APMIXEDSYS },
    { "mt8115.dvfsrc-top", 0x10012000, PA6_CLOCK_REGS },
    { "mt8115.debugtop-ao", 0x00d0b000, PA6_CLOCK_REGS },
    { "mt8115.debugtop", 0x0d0a0000, PA6_CLOCK_REGS },
    { "mt8115.ecc-top", 0x1021f000, PA6_CLOCK_REGS },
    { "mt8115.infracfg-ao", 0x10001000, PA6_CLOCK_INFRACFG_AO },
    { "mt8115.infracfg", 0x1020e000, PA6_CLOCK_REGS },
    { "mt8115.infracfg-mem", 0x1021c000, PA6_CLOCK_REGS },
    { "mt8115.iic-wrap-c", 0x11298000, PA6_CLOCK_IIC_WRAP },
    { "mt8115.iic-wrap-e", 0x11fc2000, PA6_CLOCK_IIC_WRAP },
    /* Remaining MT8171 providers, from clk-mt8171-{mmsys,mdpsys,
     * vcodec,cam,img,adsp}.c. These own the peripheral configuration banks;
     * processing engines will share them when implemented. */
    { "mt8115.smi-1401c000", 0x1401c000, PA6_SMI_LARB },
    { "mt8115.smi-1401d000", 0x1401d000, PA6_SMI_LARB },
    { "mt8115.smi-1401f000", 0x1401f000, PA6_SMI_COMMON },
    { "mt8115.smi-14025000", 0x14025000, PA6_SMI_COMMON },
    { "mt8115.smi-14026000", 0x14026000, PA6_SMI_COMMON },
    { "mt8115.smi-14027000", 0x14027000, PA6_SMI_COMMON },
    { "mt8115.smi-14028000", 0x14028000, PA6_SMI_COMMON },
    { "mt8115.smi-14029000", 0x14029000, PA6_SMI_COMMON },
    { "mt8115.smi-1502e000", 0x1502e000, PA6_SMI_LARB },
    { "mt8115.smi-1502f000", 0x1502f000, PA6_SMI_COMMON },
    { "mt8115.smi-1582e000", 0x1582e000, PA6_SMI_LARB },
    { "mt8115.smi-1602e000", 0x1602e000, PA6_SMI_LARB },
    { "mt8115.smi-17010000", 0x17010000, PA6_SMI_LARB },
    { "mt8115.smi-1a001000", 0x1a001000, PA6_SMI_LARB },
    { "mt8115.smi-1a002000", 0x1a002000, PA6_SMI_LARB },
    { "mt8115.smi-1a00c000", 0x1a00c000, PA6_SMI_COMMON },
    { "mt8115.smi-1a00d000", 0x1a00d000, PA6_SMI_COMMON },
    { "mt8115.smi-1a00f000", 0x1a00f000, PA6_SMI_LARB },
    { "mt8115.smi-1b00e000", 0x1b00e000, PA6_SMI_COMMON },
    { "mt8115.smi-1b00f000", 0x1b00f000, PA6_SMI_LARB },
    { "mt8115.smi-1c001000", 0x1c001000, PA6_SMI_LARB },
    { "mt8115.smi-1c003000", 0x1c003000, PA6_SMI_LARB },
    { "mt8115.smi-1f002000", 0x1f002000, PA6_SMI_LARB },
    { "mt8115.mfgcfg", 0x13fbf000, PA6_CLOCK_GATE0 },
    { "mt8115.dispsys", 0x14000000, PA6_CLOCK_REGS },
    { "mt8115.epdsys", 0x1c000000, PA6_CLOCK_REGS },
    { "mt8115.dptx", 0x11ec1400, PA6_CLOCK_REGS },
    { "mt8115.imgsys1", 0x15020000, PA6_CLOCK_GATE0 },
    { "mt8115.imgsys2", 0x15820000, PA6_CLOCK_GATE0 },
    { "mt8115.ipesys", 0x1b000000, PA6_CLOCK_GATE0 },
    { "mt8115.camsys-main", 0x1a000000, PA6_CLOCK_GATE0 },
    { "mt8115.camsys-rawa", 0x1a04f000, PA6_CLOCK_GATE0 },
    { "mt8115.vdec", 0x1602f000, PA6_CLOCK_VDEC },
    { "mt8115.venc", 0x17000000, PA6_CLOCK_GATE0 },
    { "mt8115.vadsys-ck", 0x1e001000, PA6_CLOCK_REGS },
    { "mt8115.vadsys-tmbist", 0x1e010000, PA6_CLOCK_REGS },
};

typedef struct PA6ClockBank {
    MemoryRegion iomem;
    PA6CS8State *board;
    hwaddr address;
    PA6ClockKind kind;
    uint32_t regs[0x1000 / 4];
} PA6ClockBank;

typedef struct PA6DPM {
    MemoryRegion pm, dm, config;
    uint32_t regs[0x8000 / 4];
    unsigned pm_size;
} PA6DPM;

typedef struct PA6DevAPC {
    MemoryRegion pd, ao;
    PA6CS8State *board;
    unsigned modules;
    uint32_t regs[0x1000 / 4];
    uint32_t permissions[0x1000 / 4];
} PA6DevAPC;

typedef struct PA6DSUPMU {
    uint32_t control, enabled, overflow, inten, selector;
    uint32_t event[6], count[6];
    uint64_t cycles;
    int64_t last_ns;
    QEMUTimer *alarm;
    qemu_irq irq;
} PA6DSUPMU;

typedef struct PA6PinBank {
    MemoryRegion iomem;
    uint32_t regs[0x1000 / 4];
    bool gpio;
    qemu_irq touch_reset;
    unsigned touch_reset_pin;
} PA6PinBank;

typedef struct PA6LVTS {
    MemoryRegion iomem;
    uint32_t regs[0x1000 / 4];
    uint8_t device[3][256];
    uint32_t count_rc[3];
    int64_t access_done[3], sensing_done[3];
    unsigned first_id, controllers;
    QEMUTimer *alarm;
    qemu_irq irq;
} PA6LVTS;

typedef struct PA6DRAMC {
    MemoryRegion iomem;
    uint32_t regs[0xc000 / 4];
} PA6DRAMC;

/* Register/status indices from mtk-scpsys-mt8171.c. */
static const struct {
    uint16_t control;
    uint8_t status_bit, sram_bit;
} pa6_power_domains[] = {
    { 0x304, 0, 0 }, /* conn */
    { 0x308, 1, 8 }, /* mfg0 */
    { 0x30c, 2, 8 }, /* mfg1 */
    { 0x310, 3, 8 }, /* mfg2 */
    { 0x314, 4, 8 }, /* mfg3 */
    { 0x340, 23, 8 }, /* vdec */
    { 0x348, 24, 8 }, /* venc */
    { 0x358, 7, 8 }, /* audio */
    { 0x35c, 13, 8 }, /* cam */
    { 0x360, 14, 8 }, /* cam_rawa */
    { 0x3ac, 9, 8 }, /* dp_tx */
    { 0x3d4, 12, 8 }, /* pcie */
    { 0x3e0, 15, 8 }, /* isp_img1 */
    { 0x3e4, 16, 8 }, /* isp_img2 */
    { 0x3e8, 17, 8 }, /* isp_ipe */
    { 0x3ec, 18, 8 }, /* mmlsys_shutdown */
    { 0x3f0, 19, 8 }, /* disp */
    { 0x3f4, 20, 8 }, /* epd */
    { 0x408, 25, 0 }, /* csi_rx */
    { 0x40c, 26, 9 }, /* vadsp_top_dormant */
    { 0x410, 27, 8 }, /* vadsp_infra */
    { 0x414, 28, 0 }, /* vadsp_ao */
};

typedef struct PA6SPM {
    MemoryRegion iomem;
    uint32_t regs[0x1000 / 4];
    int64_t settled[ARRAY_SIZE(pa6_power_domains)];
    qemu_irq conn_power;
} PA6SPM;

typedef struct PA6TinyMbox {
    MemoryRegion iomem, shared;
    uint32_t tx_pending, rx_pending;
    QEMUTimer *service_timer;
    qemu_irq irq;
} PA6TinyMbox;

typedef struct PA6AuxADC {
    MemoryRegion iomem;
    uint32_t regs[0x1000 / 4];
    uint16_t input_mv[16];
    int64_t ready_ns, complete_ns[16];
    uint16_t pending;
    QEMUTimer *alarm;
} PA6AuxADC;

typedef struct PA6AFE {
    MemoryRegion iomem;
    uint32_t regs[0x10000 / 4];
    qemu_irq irq;
} PA6AFE;

typedef struct PA6TPHY {
    MemoryRegion iomem;
    uint32_t regs[0x700 / 4];
} PA6TPHY;

typedef struct PA6USBIPPC {
    MemoryRegion iomem;
    uint32_t regs[0x100 / 4];
    int64_t settle_ns;
    int64_t resource_settle_ns[2];
    uint32_t resource_ack[2];
    bool usb3;
    qemu_irq mac_reset, mac_power;
} PA6USBIPPC;

struct PA6CS8State {
    MachineState parent_obj;
    ARMCPU *cpu[PA6_NUM_CPUS];
    GICv3State gic;
    MTKMSDCState msdc;
    MT8171M4UState m4u;
    MT8171GCEState gce;
    MT8171MDPState mdp;
    MT8171HWTCONState hwtcon;
    MT8171PanelState panel;
    MTKI2CState i2c[10];
    PA6ClockBank clocks[ARRAY_SIZE(pa6_clock_map)];
    PA6DPM dpm[2];
    PA6DevAPC devapc[4];
    qemu_irq devapc_irq;
    PA6DSUPMU dsu;
    PA6PinBank pins[7];
    PA6DRAMC dramc[2];
    PA6LVTS lvts[2];
    PA6TPHY u2phy[3];
    PA6USBIPPC usb_ippc[3];
    MTU3State usb;
    PA6AuxADC auxadc;
    PA6SPM spm;
    PA6AFE afe;
    MemoryRegion sspm_sram, sspm_config, sspm_shared;
    uint32_t sspm_regs[0x10000 / 4];
    PA6TinyMbox tiny_mbox[2];
    MemoryRegion eint;
    uint32_t eint_regs[0x1000 / 4];
    qemu_irq eint_irq;
    uint32_t eint_inputs[7], eint_connected[7];
    MemoryRegion mcupm_sram;
    MT8171MCUPMState mcupm;
    MT8171ConsysState consys;
    MT8171WlanState wlan;
    MT8171BtifState btif;
    MemoryRegion sram;
    MemoryRegion bootrom;
    MemoryRegion preloader_sram;
    MemoryRegion efuse;
    MemoryRegion timer;
    MemoryRegion systimer;
    QEMUTimer *systimer_alarm;
    qemu_irq systimer_irq;
    uint32_t systimer_control, systimer_ticks;
    bool systimer_pending;
    uint64_t systimer_expirations, systimer_masked_expirations;
    uint32_t timer_regs[0x1000 / 4];
    uint64_t entry;
    char *quickboot;
    char *bl2;
    char *boot_stage;
    char *tee;
    char *aux_firmware;
    char *touch_firmware;
    char *touch_controller;
    char *consys_firmware;
    bool development_mode;
    char *device_profile;
    const MT8115BoardConfig *board;
};

/* SPM register-level power transitions. Supply voltages, PCM firmware and
 * peripheral clock propagation remain separate providers. Nominal settling
 * is 1 us; cold-reset domains start isolated and powered down, not a captured
 * preloader power state. No resource-request ACKs are invented for USB.
 */
static void pa6_spm_update(PA6SPM *p)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (unsigned i = 0; i < ARRAY_SIZE(pa6_power_domains); i++) {
        uint32_t *ctrl = &p->regs[pa6_power_domains[i].control / 4];
        uint32_t bit = BIT(pa6_power_domains[i].status_bit);
        unsigned sram = pa6_power_domains[i].sram_bit;

        if (now < p->settled[i]) {
            continue;
        }
        p->regs[0x16c / 4] = (p->regs[0x16c / 4] & ~bit) |
                             ((*ctrl & BIT(2)) ? bit : 0);
        p->regs[0x170 / 4] = (p->regs[0x170 / 4] & ~bit) |
                             ((*ctrl & BIT(3)) ? bit : 0);
        if (sram) {
            *ctrl = (*ctrl & ~BIT(sram + 4)) |
                    ((*ctrl & BIT(sram)) ? BIT(sram + 4) : 0);
        }
    }
}

static uint64_t pa6_spm_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6SPM *p = opaque;
    pa6_spm_update(p);
    return p->regs[offset / 4];
}

static void pa6_spm_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    PA6SPM *p = opaque;

    pa6_spm_update(p);
    if (offset == 0x16c || offset == 0x170) {
        return;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(pa6_power_domains); i++) {
        if (offset == pa6_power_domains[i].control) {
            unsigned bit = pa6_power_domains[i].sram_bit;
            uint32_t ack = bit ? BIT(bit + 4) : 0;
            value = (value & ~ack) | (p->regs[offset / 4] & ack);
            if (p->regs[offset / 4] != value) {
                p->settled[i] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000;
            }
            break;
        }
    }
    p->regs[offset / 4] = value;
    if (offset == 0x304) {
        /* Match the CONSYS model's primary/secondary rail predicate. */
        qemu_set_irq(p->conn_power, (value & 0xc) == 0xc);
    }
}

static const MemoryRegionOps pa6_spm_ops = {
    .read = pa6_spm_read,
    .write = pa6_spm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* SCMI server model of the AP-visible firmware interface, based on the
 * vendor kernel's arm_scmi/{base,shmem}.c and tinysys_scmi/tinysys.c.
 * This does not execute the SSPM image. Unimplemented tinysys operations
 * return NOT_SUPPORTED; do not acknowledge arbitrary power requests.
 */
static void pa6_scmi_service(void *opaque)
{
    PA6TinyMbox *m = opaque;
    uint8_t *sh = memory_region_get_ram_ptr(&m->shared);
    uint32_t hdr = ldl_le_p(sh + 24);
    unsigned protocol = (hdr >> 10) & 255;
    unsigned command = hdr & 255;
    uint32_t length = ldl_le_p(sh + 20);
    uint32_t arg = ldl_le_p(sh + 28);
    uint8_t response[32] = { 0 };
    unsigned bytes = 0;
    int status = -1; /* SCMI_NOT_SUPPORTED */

    if ((ldl_le_p(sh + 4) & 1) || !(m->tx_pending & 1)) {
        m->tx_pending &= ~1;
        return;
    }
    if (length < 4 || length > 0x80 - 24 || (hdr & (3 << 8))) {
        status = -2; /* INVALID_PARAMETERS */
    } else if (protocol == 0x10) {
        switch (command) {
        case 0: /* protocol version */
            stl_le_p(response, 0x20000);
            bytes = 4;
            status = 0;
            break;
        case 1: /* one non-base protocol, platform + OSPM agents */
            stl_le_p(response, 0x201);
            bytes = 4;
            status = 0;
            break;
        case 2: /* message attributes */
            status = length < 8 ? -2 : (arg <= 7 ? 0 : -5);
            bytes = status ? 0 : 4;
            break;
        case 3:
        case 4:
            pstrcpy((char *)response, 16, command == 3 ? "QEMU" : "PA6-CS8");
            bytes = 16;
            status = 0;
            break;
        case 5:
            stl_le_p(response, 1);
            bytes = 4;
            status = 0;
            break;
        case 6:
            status = length < 8 ? -2 : 0;
            if (!status) {
                stl_le_p(response, arg == 0 ? 1 : 0);
                response[4] = arg == 0 ? 0x80 : 0;
                bytes = arg == 0 ? 8 : 4;
            }
            break;
        case 7:
            if (arg == UINT32_MAX) {
                arg = 1;
            }
            status = length < 8 ? -2 : (arg < 2 ? 0 : -5);
            if (!status) {
                stl_le_p(response, arg);
                memcpy(response + 4, arg ? "OSPM" : "QEMU", 5);
                bytes = 20;
            }
            break;
        }
    } else if (protocol == 0x80) {
        switch (command) {
        case 0:
            stl_le_p(response, 0x10000);
            bytes = 4;
            status = 0;
            break;
        case 1:
            /* DT feature IDs 1..12, with source zero reserved. */
            stl_le_p(response, 13);
            bytes = 4;
            status = 0;
            break;
        case 2:
            status = length < 8 ? -2 : (arg < 3 ? 0 : -5);
            bytes = status ? 0 : 4;
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "mt8115.scmi: unsupported tinysys cmd=%u fid=%u\n",
                          command, length >= 12 ? ldl_le_p(sh + 32) : 0);
            break;
        }
    }
    stl_le_p(sh + 28, status);
    memcpy(sh + 32, response, bytes);
    stl_le_p(sh + 20, 8 + bytes);
    stl_le_p(sh + 4, 1); /* response published before channel release/IRQ */
    m->tx_pending &= ~1;
    if (ldl_le_p(sh + 16) & 1) {
        m->rx_pending |= 1;
        qemu_irq_raise(m->irq);
    }
}

/* AFE control plane shared by audiosys clocks and the sound driver.
 * Layout: vendor mt8171-reg.h, irq_data[] and runtime PM in afe-pcm.c.
 * DMA/audio sample clocks are not implemented; position/counter monitors
 * remain idle and no stream-completion IRQs are generated.
 */
static const uint16_t pa6_afe_readonly[] = {
    0x0034, 0x003c, 0x006c, 0x0078, 0x007c, 0x0080, 0x00b0, 0x00b8,
    0x00c8, 0x00e8, 0x0474, 0x0478, 0x04c0, 0x04d8, 0x04fc, 0x0500,
    0x0504, 0x0a20, 0x0ba4, 0x0ba8, 0x0bc4, 0x0bc8, 0x0be0, 0x0be4,
    0x0be8, 0x0c4c, 0x0ce0, 0x0ce4, 0x0ce8, 0x0d4c, 0x0d60, 0x0d64,
    0x0d68, 0x0dcc, 0x0de0, 0x0de4, 0x0de8, 0x0e4c, 0x0e60, 0x0e64,
    0x0e68, 0x0ecc, 0x1180, 0x1188, 0x1208, 0x13e8, 0x1418, 0x1448,
    0x15a8, 0x15e8, 0x1628, 0x1c4c, 0x1e60, 0x1ea4, 0x1ecc, 0x1ed0,
    0x1ee4, 0x1ee8, 0x1eec, 0x1ef8, 0x1f00, 0x1f38, 0x1f3c, 0x1f9c,
    0x1fa0, 0x1fa4, 0x1fa8, 0x1fac, 0x1fb0, 0x1fc8, 0x1ff0, 0x1ff4,
    0x1ff8, 0x2044, 0x2054, 0x207c, 0x4084, 0x4088, 0x408c, 0x4090,
    0x4094, 0x4098, 0x4120, 0x4384, 0x4388, 0x4390, 0x4394, 0x4398,
    0x4408, 0x440c, 0x4410, 0x4418, 0x441c, 0x4448, 0x444c, 0x4458,
    0x445c, 0x4464, 0x4478, 0x447c, 0x4488, 0x448c, 0x4494, 0x44a8,
    0x44ac, 0x44b8, 0x44bc, 0x44c4, 0x44d8, 0x44dc, 0x44e8, 0x44ec,
    0x44f4, 0x4508, 0x450c, 0x4518, 0x451c, 0x4524, 0x4538, 0x453c,
    0x4548, 0x454c, 0x4554, 0x4568, 0x456c, 0x4578, 0x457c, 0x4584,
    0x4598, 0x459c, 0x45a8, 0x45ac, 0x45b4, 0x4628, 0x462c, 0x4644,
    0x4658, 0x465c, 0x4674, 0x4d68, 0x4d6c, 0x4d78, 0x4d7c, 0x4d84,
    0x4d98, 0x4d9c, 0x4da8, 0x4dac, 0x4db4, 0x4dc8, 0x4dcc, 0x4dd8,
    0x4ddc, 0x4de4, 0x4df8, 0x4dfc, 0x4e08, 0x4e0c, 0x4e14, 0x4ee8,
    0x4eec, 0x4ef8, 0x4efc, 0x4f04, 0x4f48, 0x4f4c, 0x4f58, 0x4f5c,
    0x4f64, 0x51c8, 0x51cc, 0x51dc, 0x5208, 0x520c, 0x521c, 0x52a8,
    0x52ac, 0x52bc, 0x52c8, 0x52cc, 0x52dc, 0x52e8, 0x52ec, 0x52fc,
    0x5308, 0x530c, 0x531c, 0x5368, 0x536c, 0x537c, 0x5388, 0x538c,
    0x53c8, 0x53cc, 0x53dc, 0x5458, 0x545c, 0x5468, 0x546c, 0x5490,
    0x5494, 0x5498, 0x549c, 0x54a0, 0x54a4, 0x5508, 0x550c, 0x5510,
    0x5514, 0x5518, 0x551c, 0x5520, 0x5524, 0x5528, 0x552c, 0x5530,
    0x5534, 0x5538, 0x553c, 0x5540, 0x5544, 0x5548, 0x554c, 0x5550,
    0x5554, 0x5558, 0x555c, 0x5560, 0x556c, 0x5588, 0x558c, 0x559c,
    0x576c, 0x5770, 0x5774, 0x5778, 0x577c, 0x5780, 0x5784, 0x5788,
    0x578c, 0x5790, 0x5794, 0x5798, 0x579c, 0x57a0, 0x57a4, 0x57a8,
    0x57ac, 0x57b0, 0x57b4, 0x57b8, 0x57bc, 0x57c0, 0x57c4, 0x57c8,
    0x57cc, 0x57d0, 0x57d4, 0x57d8, 0x57dc, 0x57e0, 0x57e4, 0x57e8,
    0x57ec, 0x57f0, 0x57f4, 0x57f8, 0x57fc, 0x5800, 0x5804, 0x5808,
    0x580c, 0x5810, 0x5814, 0x5818, 0x581c, 0x5820, 0x5824, 0x5828,
    0x783c, 0x787c, 0x78bc, 0x78fc, 0x793c, 0x797c, 0x79bc, 0x79fc,
    0x7a3c, 0x7a7c, 0x7abc, 0x7afc, 0x7b3c, 0x7b7c, 0x7bbc, 0x7bfc,
    0x7c3c, 0x7c7c, 0x7cbc, 0x7cfc, 0x7d3c, 0x7d7c, 0x7dbc, 0x9d20,
    0x9d24, 0x9f10, 0x9f14, 0x9f18, 0x9f1c, 0x9f20, 0x9f24, 0x9f28,
    0x9f2c, 0x9f30, 0x9f34, 0x9f38, 0x9f3c, 0x9f40, 0x9f44, 0x9f48,
    0x9f4c, 0x9f50, 0x9f54, 0x9f58, 0x9f5c, 0x9f60, 0x9f64, 0x9f68,
    0x9f6c, 0x9f70, 0x9f74, 0x9f78, 0x9f7c, 0x9f80, 0x9f84, 0x9f88,
    0x9f8c, 0x9f90, 0x9f98, 0x9f9c, 0xa01c, 0xa024, 0xa028, 0xa02c,
    0xa034, 0xa03c, 0xa048, 0xa058, 0xa068, 0xa078, 0xa0c8, 0xa0e8,
    0xa1f8, 0xa218, 0xa268, 0xa278, 0xa288, 0xa2c8, 0xa2e8, 0xa440,
    0xa448, 0xa46c,
};

static uint64_t pa6_afe_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6AFE *a = opaque;
    if (offset == 0x7c) { /* AUDIO_ENGEN_CON0_MON */
        return a->regs[0x14 / 4];
    }
    return a->regs[offset / 4];
}

static void pa6_afe_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    PA6AFE *a = opaque;
    static const struct { uint16_t cfg, status; uint8_t bit; } irqs[] = {
        { 0x9d44, 0x9d20, 0 },
        { 0x9d4c, 0x9d20, 1 },
        { 0x9d54, 0x9d20, 2 },
        { 0x9d5c, 0x9d20, 3 },
        { 0x9d64, 0x9d20, 4 },
        { 0x9d6c, 0x9d20, 5 },
        { 0x9d74, 0x9d20, 6 },
        { 0x9d7c, 0x9d20, 7 },
        { 0x9d84, 0x9d20, 8 },
        { 0x9d8c, 0x9d20, 9 },
        { 0x9d94, 0x9d20, 10 },
        { 0x9d9c, 0x9d20, 11 },
        { 0x9da4, 0x9d20, 12 },
        { 0x9dac, 0x9d20, 13 },
        { 0x9db4, 0x9d20, 14 },
        { 0x9dbc, 0x9d20, 15 },
        { 0x9dc4, 0x9d20, 16 },
        { 0x9dcc, 0x9d20, 17 },
        { 0x9dd4, 0x9d20, 18 },
        { 0x9ddc, 0x9d20, 19 },
        { 0x9de4, 0x9d20, 20 },
        { 0x9dec, 0x9d20, 21 },
        { 0x9df4, 0x9d20, 22 },
        { 0x9dfc, 0x9d20, 23 },
        { 0x9e04, 0x9d20, 24 },
        { 0x9e0c, 0x9d20, 25 },
        { 0x9e14, 0x9d20, 26 },
        { 0x9e1c, 0x9d20, 27 },
        { 0x9e24, 0x9d20, 28 },
        { 0x9e2c, 0x9d20, 29 },
        { 0x9e34, 0x9d20, 30 },
        { 0x9e3c, 0x9d20, 31 },
        { 0x9ecc, 0x9d24, 22 },
        { 0x9ed4, 0x9d24, 23 },
        { 0x9fdc, 0x9d24, 0 },
        { 0x9fe4, 0x9d24, 2 },
        { 0x9fe8, 0x9d24, 3 },
        { 0x9fec, 0x9d24, 4 },
        { 0x9ff0, 0x9d24, 5 },
        { 0x9ff4, 0x9d24, 6 },
        { 0xa044, 0xa03c, 0 },
        { 0xa054, 0xa03c, 1 },
        { 0xa064, 0xa03c, 2 },
        { 0xa074, 0xa03c, 3 },
        { 0xa0c4, 0xa03c, 8 },
        { 0xa0e4, 0xa03c, 10 },
        { 0xa1f4, 0xa03c, 27 },
        { 0xa214, 0xa03c, 29 },
        { 0xa264, 0xa448, 2 },
        { 0xa274, 0xa448, 3 },
        { 0xa284, 0xa448, 4 },
        { 0xa2c4, 0xa448, 8 },
        { 0xa2e4, 0xa448, 10 },
    };
    static const struct { uint16_t status, enable; } banks[] = {
        { 0x9d20, 0x9d00 }, { 0x9d24, 0x9d10 }, { 0xa03c, 0xa008 },
        { 0xa448, 0xa44c }, { 0xa46c, 0xa470 },
    };
    bool pending = false;

    if (offset == 0x64) { /* SPM ACK requires a resource provider. */
        return;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(pa6_afe_readonly); i++) {
        if (offset == pa6_afe_readonly[i]) {
            return;
        }
    }
    for (unsigned i = 0; i < ARRAY_SIZE(irqs); i++) {
        if (offset == irqs[i].cfg &&
            ((a->regs[offset / 4] ^ value) & 0xc0000000)) {
            a->regs[irqs[i].status / 4] &= ~BIT(irqs[i].bit);
        }
    }
    a->regs[offset / 4] = value;
    for (unsigned i = 0; i < ARRAY_SIZE(banks); i++) {
        pending |= a->regs[banks[i].status / 4] & a->regs[banks[i].enable / 4];
    }
    qemu_set_irq(a->irq, pending);
}

static const MemoryRegionOps pa6_afe_ops = {
    .read = pa6_afe_read,
    .write = pa6_afe_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t pa6_tiny_mbox_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6TinyMbox *m = opaque;

    switch (offset) {
    case 0: return m->tx_pending;
    case 4: return m->rx_pending;
    default: return 0;
    }
}

static void pa6_tiny_mbox_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    PA6TinyMbox *m = opaque;

    if (offset == 0) {
        m->tx_pending |= value;
        if ((value & 1) && !timer_pending(m->service_timer)) {
            timer_mod(m->service_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10 * 1000);
        }
    } else if (offset == 4) {
        m->rx_pending &= ~value;
        qemu_set_irq(m->irq, !!m->rx_pending);
    }
}

static const MemoryRegionOps pa6_tiny_mbox_ops = {
    .read = pa6_tiny_mbox_read,
    .write = pa6_tiny_mbox_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t pa6_sspm_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6CS8State *s = opaque;
    return s->sspm_regs[offset / 4];
}

static void pa6_sspm_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    PA6CS8State *s = opaque;
    /* Control storage only: reset release does not execute the SSPM core. */
    s->sspm_regs[offset / 4] = value;
}

static const MemoryRegionOps pa6_sspm_ops = {
    .read = pa6_sspm_read,
    .write = pa6_sspm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* AUXADC conversion interface from vendor mt6577_auxadc.c. The MT8171 DT
 * matches the mt8173 variant: 16 channels, 12 bits, 1500 mV full scale and
 * no software eFuse calibration. Time models the driver's 1 ms power-up
 * and 25 us sample intervals; analog noise and automatic scan are unmodeled.
 */
static void pa6_auxadc_schedule(PA6AuxADC *a)
{
    int64_t next = INT64_MAX;

    for (unsigned i = 0; i < 16; i++) {
        if (a->pending & BIT(i)) {
            next = MIN(next, a->complete_ns[i]);
        }
    }
    if (next == INT64_MAX) {
        timer_del(a->alarm);
    } else {
        timer_mod(a->alarm, next);
    }
}

static void pa6_auxadc_complete(void *opaque)
{
    PA6AuxADC *a = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (unsigned i = 0; i < 16; i++) {
        if ((a->pending & BIT(i)) && a->complete_ns[i] <= now) {
            unsigned raw = MIN(4095, (a->input_mv[i] * 4096 + 750) / 1500);
            a->regs[(0x14 + i * 4) / 4] = BIT(12) | raw;
            a->pending &= ~BIT(i);
        }
    }
    pa6_auxadc_schedule(a);
}

static uint64_t pa6_auxadc_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6AuxADC *a = opaque;

    if (offset == 0x10) {
        return (a->regs[0x10 / 4] & ~1U) | !!a->pending;
    }
    if (offset == 8 || offset == 12) {
        return 0; /* Write-only channel trigger aliases. */
    }
    return a->regs[offset / 4];
}

static void pa6_auxadc_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    PA6AuxADC *a = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (offset >= 0x14 && offset < 0x54) {
        return; /* Sample words and ready flags belong to the converter. */
    }
    if (offset == 4 || offset == 8 || offset == 12) {
        uint16_t old = a->regs[1];
        uint16_t channels = offset == 8 ? old | value :
                            offset == 12 ? old & ~value : value;
        uint16_t trigger = channels & ~old;
        int64_t deadline = MAX(now, a->ready_ns);

        a->regs[1] = channels;
        for (unsigned i = 0; i < 16; i++) {
            if (!(channels & BIT(i))) {
                a->regs[(0x14 + i * 4) / 4] &= ~BIT(12);
                a->pending &= ~BIT(i);
            }
            if (a->pending & BIT(i)) {
                deadline = MAX(deadline, a->complete_ns[i]);
            }
        }
        if (a->regs[0x94 / 4] & BIT(14)) {
            for (unsigned i = 0; i < 16; i++) {
                if (trigger & BIT(i)) {
                    a->regs[(0x14 + i * 4) / 4] &= ~BIT(12);
                    a->pending |= BIT(i);
                    deadline += 25000;
                    a->complete_ns[i] = deadline;
                }
            }
        }
        pa6_auxadc_schedule(a);
    } else if (offset == 0x94) {
        if ((value & BIT(14)) && !(a->regs[offset / 4] & BIT(14))) {
            a->ready_ns = now + 1000000;
        }
        a->regs[offset / 4] = value;
        if (!(value & BIT(14))) {
            a->pending = 0;
            for (unsigned i = 0; i < 16; i++) {
                a->regs[(0x14 + i * 4) / 4] &= ~BIT(12);
            }
            timer_del(a->alarm);
        }
    } else {
        a->regs[offset / 4] = offset == 0x10 ? value & ~1U : value;
    }
}

static const MemoryRegionOps pa6_auxadc_ops = {
    .read = pa6_auxadc_read,
    .write = pa6_auxadc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* USB IP power/reset controls, from the vendor mtu3_hw_regs.h. Status
 * follows software reset, power-down and port selection with a nominal
 * 1 us settling delay. Upstream clock-tree timing is not yet propagated.
 */
static void pa6_usb_ippc_outputs(PA6USBIPPC *p)
{
    uint32_t *r = p->regs;
    bool reset = (r[0] | r[0x98 / 4]) & 1;
    bool powered = !reset && !(r[0x08 / 4] & 1) && !(r[0x50 / 4] & 7);

    qemu_set_irq(p->mac_reset, reset);
    qemu_set_irq(p->mac_power, powered);
}

static void pa6_usb_ippc_reset(PA6USBIPPC *p)
{
    memset(p->regs, 0, sizeof(p->regs));
    memset(p->resource_ack, 0, sizeof(p->resource_ack));
    memset(p->resource_settle_ns, 0, sizeof(p->resource_settle_ns));
    p->regs[0x04 / 4] = 1; /* Host powered down. */
    p->regs[0x08 / 4] = 1; /* Device powered down. */
    p->regs[0x0c / 4] = 1;
    p->regs[0x30 / 4] = 3; /* Port disabled and powered down. */
    p->regs[0x50 / 4] = 3;
    p->settle_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000;
    pa6_usb_ippc_outputs(p);
}

static uint64_t pa6_usb_ippc_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6USBIPPC *p = opaque;
    uint32_t *r = p->regs;
    bool ready = !(r[0] & 1) &&
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= p->settle_ns;
    bool host = ready && !(r[0x04 / 4] & 1);
    bool device = ready && !(r[0x08 / 4] & 1) && !(r[0x98 / 4] & 1);
    bool u2 = !(r[0x50 / 4] & 3) &&
              ((r[0x50 / 4] & BIT(2)) ? host : device);
    bool u3 = p->usb3 && !(r[0x30 / 4] & 3) &&
              ((r[0x30 / 4] & BIT(2)) ? host : device);

    switch (offset) {
    case 0x10:
        return ((!host && !device) ? BIT(30) : 0) |
               ((host || device) ? BIT(0) | BIT(8) | BIT(10) : 0) |
               (host ? BIT(11) : 0) | (u3 ? BIT(16) : 0);
    case 0x14:
        return u2 ? 1 : 0;
    case 0x20:
    case 0x28:
        return p->usb3 ? 1 : 0;
    case 0x24:
        return 0x100 | (p->usb3 ? 1 : 0);
    case 0x44:
    case 0xe8: {
        unsigned version = offset == 0xe8;
        unsigned request = version ? 0xe0 : 0x40;
        /* mtu3_hwrscs_req_v2_v3 waits for SPM resource requests to settle.
         * The board supplies these digital clock/DDR resources; reflect only
         * their defined request bits, never FORCE_HW_REQ or reserved bits.
         * V3 adds VCORE bit 5 to the V1/V2 mask 0x5f. */
        if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >=
            p->resource_settle_ns[version]) {
            p->resource_ack[version] = r[request / 4] &
                                      (version ? 0x7f : 0x5f);
        }
        return p->resource_ack[version];
    }
    default:
        return r[offset / 4];
    }
}

static void pa6_usb_ippc_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    PA6USBIPPC *p = opaque;

    if ((offset >= 0x10 && offset <= 0x28) || offset == 0x44 ||
        offset == 0xa0 || offset == 0xa4 || offset == 0xe8) {
        return; /* Status/capabilities; no external VBUS transition yet. */
    }
    if (offset == 0 && (value & 1) && !(p->regs[0] & 1)) {
        pa6_usb_ippc_reset(p);
    }
    if ((offset <= 0x0c || offset == 0x30 || offset == 0x50 || offset == 0x98) &&
        p->regs[offset / 4] != value) {
        p->settle_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000;
    }
    if ((offset == 0x40 || offset == 0xe0) &&
        p->regs[offset / 4] != value) {
        /* Nominal digital handshake latency, not measured physical timing. */
        p->resource_settle_ns[offset == 0xe0] =
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000;
    }
    p->regs[offset / 4] = value;
    pa6_usb_ippc_outputs(p);
}

static const MemoryRegionOps pa6_usb_ippc_ops = {
    .read = pa6_usb_ippc_read,
    .write = pa6_usb_ippc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/* TPHY v3 USB2 miscellaneous and common analog-control banks. The v3
 * frequency-meter aperture is reserved. Electrical signaling, factory
 * calibration and the USB controller itself are separate, unfinished work.
 */
static bool pa6_tphy_control(hwaddr offset)
{
    return offset < 0x100 || (offset >= 0x300 && offset < 0x400);
}

static uint64_t pa6_tphy_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6TPHY *p = opaque;

    return pa6_tphy_control(offset) ? p->regs[offset / 4] : 0;
}

static void pa6_tphy_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    PA6TPHY *p = opaque;

    if (pa6_tphy_control(offset)) {
        p->regs[offset / 4] = value;
    }
}

static const MemoryRegionOps pa6_tphy_ops = {
    .read = pa6_tphy_read,
    .write = pa6_tphy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * MT8171 LVTS v1 controller / v6 sensor programming, from soc_temp_lvts.c.
 * Nominal uncalibrated sensors: 25 C, RC count 2750. These describe virtual
 * sensors, not factory eFuses. Analog filtering and thermal reset are not yet
 * modeled. Device transactions take 2 us and RC measurements take 20 us.
 */
static void pa6_lvts_irq(PA6LVTS *l)
{
    uint32_t summary = UINT32_MAX;
    bool pending = false;

    for (unsigned i = 0; i < l->controllers; i++) {
        uint32_t *r = &l->regs[i * 0x100 / 4];

        if (r[0x10 / 4] & r[0x0c / 4]) {
            summary &= ~BIT(i + 1); /* Active-low domain summary. */
            pending = true;
        }
    }
    l->regs[0xf04 / 4] = summary;
    qemu_set_irq(l->irq, pending);
}

static void pa6_lvts_sample(void *opaque)
{
    PA6LVTS *l = opaque;
    static const unsigned cold_bits[] = { 0, 5, 10, 22 };
    bool active = false;

    for (unsigned i = 0; i < l->controllers; i++) {
        uint32_t *r = &l->regs[i * 0x100 / 4];
        unsigned sensors = l->first_id == 0x83 && i == 0 ? 2 : 4;

        for (unsigned j = 0; j < sensors; j++) {
            /* v1 normalized count at 25 C plus the protection offset. */
            uint32_t raw = MIN(0xffff, 16384 + (r[0xc0 / 4] & 0xffff));
            uint32_t protection = r[0xc0 / 4];
            bool dominator = (protection & BIT(16)) ||
                             ((protection & BIT(17)) &&
                              ((protection >> 18) & 3) == j);

            if (!(r[0xe4 / 4] & 1) || !(r[0] & BIT(j))) {
                r[(0x90 + j * 4) / 4] = 0;
                r[(0xa0 + j * 4) / 4] = 0;
                continue;
            }
            active = true;
            r[(0x90 + j * 4) / 4] = BIT(16) | raw;
            r[(0xa0 + j * 4) / 4] = BIT(16) | raw;
            if (raw > r[0x2c / 4]) {
                r[0x10 / 4] |= BIT(cold_bits[j]);
            }
            if (raw < r[0x28 / 4]) {
                r[0x10 / 4] |= BIT(cold_bits[j] + 1);
            }
            if (dominator && raw < r[0xcc / 4]) {
                r[0x10 / 4] |= BIT(31);
            }
        }
    }
    pa6_lvts_irq(l);
    if (active) {
        timer_mod(l->alarm, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2500000);
    }
}

static uint64_t pa6_lvts_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6LVTS *l = opaque;
    unsigned tc = offset / 0x100;
    uint32_t value = l->regs[offset / 4];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (tc < l->controllers && (offset & 0xff) == 0x50) {
        value &= ~(BIT(24) | BIT(25));
        if (now < l->access_done[tc]) {
            value |= BIT(24);
        }
        if (now < l->sensing_done[tc]) {
            value |= BIT(25);
        }
    }
    return value;
}

static void pa6_lvts_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    PA6LVTS *l = opaque;
    unsigned tc = offset / 0x100;
    unsigned reg = offset & 0xff;
    uint32_t *r;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (tc >= l->controllers) {
        return;
    }
    r = &l->regs[tc * 0x100 / 4];
    if (reg == 0x10) {
        r[reg / 4] &= ~value;
    } else if (reg == 0x4c || (reg >= 0x90 && reg <= 0xbc)) {
        return; /* ID, measurements and serial device readback are read-only. */
    } else if (reg == 0x50 && (value & BIT(24))) {
        unsigned index = (value >> 8) & 0xff;
        uint8_t data = value;

        r[reg / 4] = value & ~(BIT(24) | BIT(25));
        l->access_done[tc] = now + 2000;
        if (value & BIT(16)) {
            if (index == 0xff && data == 0xff) {
                memset(l->device[tc], 0, sizeof(l->device[tc]));
                l->count_rc[tc] = 0;
                l->sensing_done[tc] = 0;
            } else {
                l->device[tc][index] = data;
                if (index == 3 && data == 2) {
                    l->count_rc[tc] = 2750;
                    l->sensing_done[tc] = now + 20000;
                }
            }
        } else {
            uint32_t data = index == 0 ? l->count_rc[tc] :
                            index == 0xfc ? l->first_id + tc :
                            l->device[tc][index];

            r[0xb0 / 4] = data;
            if (value & BIT(26)) {
                r[0x4c / 4] = data;
            }
        }
    } else {
        /* MSRCTL1 idle status bits are not software writable. */
        r[reg / 4] = reg == 0x3c ? value & ~0x481 : value;
    }
    pa6_lvts_irq(l);
    if (reg == 0 || reg == 0xe4) {
        timer_mod(l->alarm, now + 20000);
    }
}

static const MemoryRegionOps pa6_lvts_ops = {
    .read = pa6_lvts_read,
    .write = pa6_lvts_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void pa6_eint_update(PA6CS8State *s)
{
    bool pending = false;

    for (unsigned port = 0; port < 7; port++) {
        uint32_t active = ~(s->eint_inputs[port] ^
                            s->eint_regs[0x300 / 4 + port]) &
                          s->eint_connected[port];
        uint32_t level = s->eint_regs[0x140 / 4 + port];

        s->eint_regs[port] = (s->eint_regs[port] & ~level) | (active & level);
        pending |= (s->eint_regs[port] | s->eint_regs[0x200 / 4 + port]) &
                   ~s->eint_regs[0x80 / 4 + port] &
                   s->eint_regs[0x400 / 4 + port];
    }
    qemu_set_irq(s->eint_irq, pending);
}

static void pa6_eint_input(void *opaque, int pin, int level)
{
    PA6CS8State *s = opaque;
    unsigned port = pin / 32;
    uint32_t bit = 1u << (pin % 32);
    bool old = !!(s->eint_inputs[port] & bit);
    bool high_active = !!(s->eint_regs[0x300 / 4 + port] & bit);

    if (pin < 192) {
        uint32_t *din = &s->pins[0].regs[0x200 / 4 + port * 4];
        *din = (*din & ~bit) | (level ? bit : 0);
    }
    s->eint_connected[port] |= bit;
    s->eint_inputs[port] = (s->eint_inputs[port] & ~bit) | (level ? bit : 0);
    if (old != !!level && !!level == high_active) {
        s->eint_regs[port] |= bit;
    }
    pa6_eint_update(s);
}

static uint64_t pa6_eint_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6CS8State *s = opaque;

    if (offset < 7 * 4 || (offset >= 0xa00 && offset < 0xa00 + 7 * 4)) {
        unsigned port = (offset & 0x3f) / 4;

        return s->eint_regs[port] | s->eint_regs[0x200 / 4 + port];
    }
    return s->eint_regs[offset / 4];
}

static void pa6_eint_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    PA6CS8State *s = opaque;
    static const unsigned bases[] = { 0x80, 0x140, 0x200, 0x300 };
    unsigned port = (offset & 0x3f) / 4;

    /* Last port contains EINT192..205. Status is hardware-owned, ACK W1C. */
    if (port == 6 && offset < 0x500) {
        value &= 0x3fff;
    }
    if (offset < 0x40 || offset >= 0xa00) {
        return;
    }
    if (offset < 0x40 + 7 * 4) {
        s->eint_regs[port] &= ~value;
    } else {
        for (unsigned i = 0; i < ARRAY_SIZE(bases); i++) {
            unsigned base = bases[i];

            if (offset >= base && offset < base + 0xc0 && port < 7) {
                uint32_t *reg = &s->eint_regs[base / 4 + port];

                switch ((offset - base) / 0x40) {
                case 0: *reg = value; break;
                case 1: *reg |= value; break;
                case 2: *reg &= ~value; break;
                }
                pa6_eint_update(s);
                return;
            }
        }
        if (offset >= 0x500 && offset < 0x800) {
            /* 52 debounce byte fields, with SET/CLR and self-clearing reset. */
            unsigned word = (offset & 0xff) / 4;
            uint32_t *reg = &s->eint_regs[0x500 / 4 + word];

            if (word >= 13) {
                return;
            }
            switch (offset >> 8) {
            case 5: *reg = value; break;
            case 6: *reg |= value; break;
            case 7: *reg &= ~value; break;
            }
            *reg &= ~0x02020202;
        } else {
            s->eint_regs[offset / 4] = value;
        }
    }
    pa6_eint_update(s);
}

static const MemoryRegionOps pa6_eint_ops = {
    .read = pa6_eint_read,
    .write = pa6_eint_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t pa6_dramc_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6DRAMC *d = opaque;

    return d->regs[offset / 4];
}

static void pa6_dramc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    PA6DRAMC *d = opaque;

    d->regs[offset / 4] = value;
}

static const MemoryRegionOps pa6_dramc_ops = {
    .read = pa6_dramc_read,
    .write = pa6_dramc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* MT8171 GPIO and IOCFG words have VAL/SET/CLR at +0/+4/+8. */
static void pa6_touch_reset_update(PA6PinBank *p)
{
    /* Reset is pulled high when the AP is not driving it as a GPIO output. */
    if (p->touch_reset) {
        unsigned pin = p->touch_reset_pin;
        unsigned bank = (pin / 32) * 0x10;
        unsigned mode = 0x300 + (pin / 8) * 0x10;
        uint32_t bit = BIT(pin % 32);
        bool driven = (p->regs[bank / 4] & bit) &&
                      !(p->regs[mode / 4] & (15U << ((pin % 8) * 4)));
        qemu_set_irq(p->touch_reset,
                     !driven || (p->regs[(0x100 + bank) / 4] & bit));
    }
}

static uint64_t pa6_pin_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6PinBank *p = opaque;

    if (p->gpio && offset >= 0x200 && offset < 0x260 && !(offset & 0xf)) {
        unsigned port = (offset - 0x200) / 0x10;
        uint32_t dir = p->regs[port * 4];
        uint32_t out = p->regs[0x100 / 4 + port * 4];

        /* External peripheral pins and driven GPIO outputs share DIN. */
        return (dir & out) | (~dir & p->regs[offset / 4]);
    }
    return p->regs[offset / 4];
}

static void pa6_pin_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    PA6PinBank *p = opaque;
    unsigned alias = offset & 0xf;
    uint32_t *reg = &p->regs[(offset & ~0xf) / 4];

    if (p->gpio && offset >= 0x200 && offset < 0x260) {
        return;
    }
    if (alias == 4) {
        *reg |= value;
    } else if (alias == 8) {
        *reg &= ~value;
    } else if (!alias) {
        *reg = value;
    }
    if (p->gpio) {
        pa6_touch_reset_update(p);
    }
}

static const MemoryRegionOps pa6_pin_ops = {
    .read = pa6_pin_read,
    .write = pa6_pin_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * DSU cluster PMU, shared by all four cores (Arm 100453, chapter B3).
 * Like QEMU's core PMU, cycles use virtual time rather than a simulated
 * pipeline. Only cycles and chaining have event producers at present.
 */
static void pa6_dsu_sync(PA6DSUPMU *p)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t delta = now - p->last_ns;
    uint64_t carry = 0;

    p->last_ns = now;
    if (p->control & 1) {
        if (p->enabled & BIT(31)) {
            uint64_t old = p->cycles;
            p->cycles += delta;
            if (p->cycles < old) {
                p->overflow |= BIT(31);
            }
        }
        for (unsigned i = 0; i < ARRAY_SIZE(p->count); i++) {
            uint64_t add = 0, value;

            if (p->enabled & BIT(i)) {
                if (p->event[i] == 0x11) {
                    add = delta;
                } else if (p->event[i] == 0x1e && (i & 1)) {
                    add = carry;
                }
            }
            value = p->count[i] + add;
            carry = value >> 32;
            p->count[i] = value;
            if (carry) {
                p->overflow |= BIT(i);
            }
        }
    }
    qemu_set_irq(p->irq, (p->overflow & p->inten) != 0);
}

static void pa6_dsu_schedule(PA6DSUPMU *p)
{
    uint64_t next = INT64_MAX;

    timer_del(p->alarm);
    if (!(p->control & 1)) {
        return;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(p->count); i++) {
        if ((p->enabled & BIT(i)) && p->event[i] == 0x11) {
            next = MIN(next, (1ULL << 32) - p->count[i]);
        }
    }
    if ((p->enabled & BIT(31)) && p->cycles) {
        next = MIN(next, -p->cycles);
    }
    if (next < INT64_MAX - p->last_ns) {
        timer_mod(p->alarm, p->last_ns + next);
    }
}

static void pa6_dsu_alarm(void *opaque)
{
    PA6DSUPMU *p = opaque;

    pa6_dsu_sync(p);
    pa6_dsu_schedule(p);
}

static uint64_t pa6_dsu_read(CPUARMState *env, const ARMCPRegInfo *ri)
{
    PA6DSUPMU *p = &PA6_CS8(qdev_get_machine())->dsu;
    BQL_LOCK_GUARD();

    pa6_dsu_sync(p);
    if (ri->crm == 5) {
        switch (ri->opc2) {
        case 0: return 0x41413040 | p->control;
        case 1: case 2: return p->enabled;
        case 3: case 4: return p->overflow;
        case 5: return p->selector;
        case 6: case 7: return p->inten;
        }
    } else {
        switch (ri->opc2) {
        case 0: return p->cycles;
        case 1: return p->selector < 6 ? p->event[p->selector] : 0;
        case 2: return p->selector < 6 ? p->count[p->selector] : 0;
        case 4: return BIT(17) | BIT(30); /* CPU_CYCLES, CHAIN */
        case 5: return 0;
        }
    }
    g_assert_not_reached();
}

static void pa6_dsu_write(CPUARMState *env, const ARMCPRegInfo *ri,
                          uint64_t value)
{
    PA6DSUPMU *p = &PA6_CS8(qdev_get_machine())->dsu;
    uint32_t mask = value & (BIT(31) | 0x3f);
    BQL_LOCK_GUARD();

    pa6_dsu_sync(p);
    if (ri->crm == 5) {
        switch (ri->opc2) {
        case 0:
            p->control = value & 1;
            if (value & 2) {
                memset(p->count, 0, sizeof(p->count));
            }
            if (value & 4) {
                p->cycles = 0;
            }
            break;
        case 1: p->enabled |= mask; break;
        case 2: p->enabled &= ~mask; break;
        case 3: p->overflow |= mask; break;
        case 4: p->overflow &= ~mask; break;
        case 5: p->selector = value & 0x1f; break;
        case 6: p->inten |= mask; break;
        case 7: p->inten &= ~mask; break;
        }
    } else {
        switch (ri->opc2) {
        case 0: p->cycles = value; break;
        case 1:
            if (p->selector < 6) {
                p->event[p->selector] = value & 0xffff;
            }
            break;
        case 2:
            if (p->selector < 6) {
                p->count[p->selector] = value;
            }
            break;
        }
    }
    qemu_set_irq(p->irq, (p->overflow & p->inten) != 0);
    pa6_dsu_schedule(p);
}

static void pa6_dsu_register(ARMCPU *cpu)
{
    static const char * const names[2][8] = {
        { "CLUSTERPMCR_EL1", "CLUSTERPMCNTENSET_EL1",
          "CLUSTERPMCNTENCLR_EL1", "CLUSTERPMOVSSET_EL1",
          "CLUSTERPMOVSCLR_EL1", "CLUSTERPMSELR_EL1",
          "CLUSTERPMINTENSET_EL1", "CLUSTERPMINTENCLR_EL1" },
        { "CLUSTERPMCCNTR_EL1", "CLUSTERPMXEVTYPER_EL1",
          "CLUSTERPMXEVCNTR_EL1", NULL,
          "CLUSTERPMCEID0_EL1", "CLUSTERPMCEID1_EL1" },
    };

    for (unsigned bank = 0; bank < 2; bank++) {
        for (unsigned op = 0; op < 8; op++) {
            if (names[bank][op]) {
                ARMCPRegInfo reg = {
                    .name = names[bank][op], .state = ARM_CP_STATE_AA64,
                    .opc0 = 3, .opc1 = 0, .crn = 15,
                    .crm = 5 + bank, .opc2 = op, .access = PL1_RW,
                    .type = ARM_CP_IO | ARM_CP_NO_RAW,
                    .readfn = pa6_dsu_read, .writefn = pa6_dsu_write,
                };
                define_one_arm_cp_reg(cpu, &reg);
            }
        }
    }
}

/*
 * MT8171 clock register layout used by the MT8115 vendor driver.  Mux and
 * PLL programming is retained for readback; peripheral timing is still
 * supplied by the individual device models, not a simulated clock tree.
 */
static bool pa6_smi_busy(PA6ClockBank *bank)
{
    /* DMA is synchronous except the outstanding MDP LUT transfer. A
     * processing engine waiting for input has no SMI transaction to drain. */
    /* Stock DT connects larb2 directly to common0. The subcommons at
     * 0x14027000..0x14029000 serve other larbs (including HWTCON larb3/8);
     * an MDP LUT transfer must not make their independent buses busy. */
    bool mdp_bus = bank->address == 0x1f002000 ||
                   bank->address == 0x1401f000;
    return mdp_bus && timer_pending(bank->board->mdp.lut_timer);
}

static uint64_t pa6_clock_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6ClockBank *bank = opaque;

    if (bank->kind == PA6_SMI_LARB) {
        bool busy = pa6_smi_busy(bank);
        if (offset == 0) {
            return busy; /* LARB_STAT: outstanding transactions. */
        }
        if (offset == 0xc) {
            return (bank->regs[3] & ~BIT(16)) |
                   ((bank->regs[3] & 1) && !busy ? BIT(16) : 0);
        }
    } else if (bank->kind == PA6_SMI_COMMON && offset == 0x440) {
        return (bank->regs[offset / 4] & ~1U) | !pa6_smi_busy(bank);
    }
    return bank->regs[offset / 4];
}

static void pa6_clock_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    PA6ClockBank *bank = opaque;
    hwaddr base = offset & ~0xf;
    bool aliases = false;
    static const struct {
        unsigned set, clear, status;
    } infra_gates[] = {
        { 0x10, 0x14, 0x18 }, { 0x1c, 0x20, 0x24 },
        { 0x80, 0x84, 0x90 }, { 0x88, 0x8c, 0x94 },
        { 0xa4, 0xa8, 0xac }, { 0xc0, 0xc4, 0xc8 },
        { 0xf0, 0xf4, 0xf8 },
    };

    switch (bank->kind) {
    case PA6_SMI_LARB:
        if (((offset >= 0x380 && offset < 0x400) ||
             (offset >= 0xf80 && offset < 0x1000)) &&
            bank->regs[offset / 4] != value) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mt8115-smi: base=%" HWADDR_PRIx
                          " reg=%03" HWADDR_PRIx " %08x->%08" PRIx64
                          " pc=%" PRIx64 "\n",
                          bank->address, offset, bank->regs[offset / 4], value,
                          (uint64_t)(current_cpu ? current_cpu->cc->get_pc(current_cpu) : 0));
        }
        if (!offset) {
            return;
        }
        if (offset == 0xc) {
            value &= ~BIT(16); /* Sleep ACK is device-owned. */
        }
        break;
    case PA6_SMI_COMMON:
        if (offset == 0x440) {
            return;
        }
        if (offset == 0x3c4 || offset == 0x3c8) {
            if (offset == 0x3c4) {
                bank->regs[0x3c0 / 4] |= value;
            } else {
                bank->regs[0x3c0 / 4] &= ~value;
            }
            return;
        }
        break;
    case PA6_CLOCK_TOPCKGEN:
        /* CLK_CFG_UPDATE[0..2]: latch mux selections, then self-clear. */
        if (offset >= 0x4 && offset <= 0xc) {
            return;
        }
        aliases = (base >= 0x10 && base <= 0x130) ||
                  (base >= 0x240 && base <= 0x260);
        if (offset >= 0x504 && offset <= 0x50c) {
            base = 0x504;
            aliases = true;
        }
        break;
    case PA6_CLOCK_APMIXEDSYS:
        /* Shared PLL enable and reset-bar registers. */
        aliases = base == 0x70 || base == 0x80;
        break;
    case PA6_CLOCK_INFRACFG_AO: {
        /* Bus-protection SET/CLR, enable and idle-ACK registers. There
         * are no modeled in-flight multimedia transactions to drain yet. */
        static const unsigned protection[][4] = {
            { 0x2a0, 0x2a4, 0x220, 0x228 },
            { 0x2a8, 0x2ac, 0x250, 0x258 },
            { 0x2d4, 0x2d8, 0x2d0, 0x2ec },
            { 0x714, 0x718, 0x710, 0x724 },
            { 0x758, 0x75c, 0x754, 0x764 },
        };
        for (unsigned i = 0; i < ARRAY_SIZE(protection); i++) {
            const unsigned *r = protection[i];
            if (offset == r[3]) {
                return;
            }
            if (offset == r[0] || offset == r[1] || offset == r[2]) {
                uint32_t *en = &bank->regs[r[2] / 4];
                *en = offset == r[0] ? *en | value :
                      offset == r[1] ? *en & ~value : value;
                bank->regs[r[3] / 4] = i == 4 ?
                    ((*en & BIT(3)) ? BIT(4) | BIT(10) | BIT(13) : 0) : *en;
                return;
            }
        }
        for (unsigned i = 0; i < ARRAY_SIZE(infra_gates); i++) {
            if (offset == infra_gates[i].set) {
                bank->regs[infra_gates[i].status / 4] |= value;
                return;
            }
            if (offset == infra_gates[i].clear) {
                bank->regs[infra_gates[i].status / 4] &= ~value;
                return;
            }
        }
        break;
    }
    case PA6_CLOCK_IIC_WRAP:
        /* These wrappers place CLR before SET, unlike TOPCKGEN. */
        if (offset == 0xe04) {
            bank->regs[0xe00 / 4] &= ~value;
            return;
        }
        if (offset == 0xe08) {
            bank->regs[0xe00 / 4] |= value;
            return;
        }
        break;
    case PA6_CLOCK_GATE0:
    case PA6_CLOCK_MDP:
        base = bank->kind == PA6_CLOCK_MDP ? 0x100 : 0;
        if (offset == base + 4 || offset == base + 8) {
            if (offset == base + 4) {
                bank->regs[base / 4] |= value;
            } else {
                bank->regs[base / 4] &= ~value;
            }
            return;
        }
        break;
    case PA6_CLOCK_VDEC:
        /* VDEC and LARB use the status address itself as the SET port. */
        if (offset == 0 || offset == 4 || offset == 8 || offset == 12 ||
            offset == 0x200 || offset == 0x204) {
            base = offset & ~7;
            if (offset == base) {
                bank->regs[base / 4] |= value;
            } else {
                bank->regs[base / 4] &= ~value;
            }
            return;
        }
        break;
    case PA6_CLOCK_REGS:
        break;
    default:
        g_assert_not_reached();
    }

    if (aliases && offset == base + 4) {
        bank->regs[base / 4] |= value;
    } else if (aliases && offset == base + 8) {
        bank->regs[base / 4] &= ~value;
    } else {
        bank->regs[offset / 4] = value;
    }
}

static MemTxResult pa6_clock_write_attrs(void *opaque, hwaddr offset,
                                         uint64_t value, unsigned size,
                                         MemTxAttrs attrs)
{
    PA6ClockBank *bank = opaque;

    /* MT8171 uses the secure-world SMI MMU configuration service. Linux's
     * mtk-smi and MediaTek TF-A describe SEC_CON as the effective MMU_EN
     * source; NONSEC_CON retains bank selection. Protect the secure port
     * configuration from the vendor HAPS helper's nonsecure zero writes.
     * Ignored writes (rather than external aborts) are our access-policy
     * model; the exact silicon violation response is not documented. */
    if (bank->kind == PA6_SMI_LARB && offset >= 0xf80 && offset < 0x1000 &&
        !attrs.secure) {
        return MEMTX_OK;
    }
    pa6_clock_write(opaque, offset, value, size);
    return MEMTX_OK;
}

static const MemoryRegionOps pa6_clock_ops = {
    .read = pa6_clock_read,
    .write_with_attrs = pa6_clock_write_attrs,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t pa6_dpm_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6DPM *dpm = opaque;

    return dpm->regs[offset / 4];
}

static void pa6_dpm_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    PA6DPM *dpm = opaque;

    dpm->regs[offset / 4] = value;
}

static const MemoryRegionOps pa6_dpm_ops = {
    .read = pa6_dpm_read,
    .write = pa6_dpm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* MT8171 DEVAPC v3 mask/status interface. Bus access filtering is not modeled. */
static void pa6_devapc_update_irq(PA6CS8State *s)
{
    bool pending = false;

    for (unsigned i = 0; i < ARRAY_SIZE(s->devapc); i++) {
        PA6DevAPC *apc = &s->devapc[i];

        if (apc->regs[0xf00 / 4] & BIT(2)) {
            continue;
        }
        for (unsigned j = 0; j < DIV_ROUND_UP(apc->modules, 32); j++) {
            pending |= (apc->regs[0x400 / 4 + j] & ~apc->regs[j]) != 0;
        }
    }
    qemu_set_irq(s->devapc_irq, pending);
}

static uint64_t pa6_devapc_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6DevAPC *apc = opaque;

    return apc->regs[offset / 4];
}

static void pa6_devapc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    PA6DevAPC *apc = opaque;

    if ((offset >= 0x400 && offset < 0x450) || offset == 0xf20) {
        /* VIO_STA and VIO_SHIFT_STA are write-one-to-clear. */
        apc->regs[offset / 4] &= ~value;
    } else if (offset >= 0x900 && offset <= 0x90c) {
        /* Violation debug words are hardware-owned. */
        return;
    } else if (offset == 0xf10) {
        apc->regs[offset / 4] = value & 1 ? 3 : 0;
    } else {
        apc->regs[offset / 4] = value;
    }
    pa6_devapc_update_irq(apc->board);
}

static const MemoryRegionOps pa6_devapc_ops = {
    .read = pa6_devapc_read,
    .write = pa6_devapc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t pa6_devapc_ao_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6DevAPC *apc = opaque;

    return apc->permissions[offset / 4];
}

static void pa6_devapc_ao_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    PA6DevAPC *apc = opaque;

    apc->permissions[offset / 4] = value;
}

static const MemoryRegionOps pa6_devapc_ao_ops = {
    .read = pa6_devapc_ao_read,
    .write = pa6_devapc_ao_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* GPT4 is the free-running 26 MHz counter used by the stock bootloader. */
static uint64_t pa6_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6CS8State *s = opaque;

    if (offset == 0x48 || offset == 0x28) {
        return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                        26000000, NANOSECONDS_PER_SECOND);
    }
    return s->timer_regs[offset / 4];
}

static void pa6_timer_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    PA6CS8State *s = opaque;

    s->timer_regs[offset / 4] = value;
}

static const MemoryRegionOps pa6_timer_ops = {
    .read = pa6_timer_read,
    .write = pa6_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void pa6_systimer_update_irq(PA6CS8State *s)
{
    qemu_set_irq(s->systimer_irq,
                 s->systimer_pending && (s->systimer_control & 3) == 3);
}

static void pa6_systimer_expire(void *opaque)
{
    PA6CS8State *s = opaque;
    if (s->systimer_control & 1) {
        /* Countdown and interrupt delivery are separate. Linux writes
         * CON=EN, VAL=ticks, CON=EN|IRQ_EN; a short countdown may expire
         * before the last write. Preserve that event until IRQ_CLR. */
        s->systimer_pending = true;
        s->systimer_expirations++;
        if (!(s->systimer_control & 2)) { s->systimer_masked_expirations++; }
    }
    pa6_systimer_update_irq(s);
}

static uint64_t pa6_systimer_read(void *opaque, hwaddr offset, unsigned size)
{
    PA6CS8State *s = opaque;
    return offset == 0x40 ? s->systimer_control :
           offset == 0x44 ? s->systimer_ticks : 0;
}

static void pa6_systimer_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    PA6CS8State *s = opaque;
    if (offset == 0x40) {
        s->systimer_control = value & 3;
        /* The source register contract requires EN for status changes. */
        if ((value & 0x11) == 0x11) {
            s->systimer_pending = false;
        }
        if (!(value & 1)) {
            timer_del(s->systimer_alarm);
        }
    } else if (offset == 0x44) {
        if (s->systimer_control & 1) {
            s->systimer_ticks = value;
            timer_mod(s->systimer_alarm, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      muldiv64(s->systimer_ticks, NANOSECONDS_PER_SECOND,
                               13000000));
        }
    }
    pa6_systimer_update_irq(s);
}

static const MemoryRegionOps pa6_systimer_ops = {
    .read = pa6_systimer_read,
    .write = pa6_systimer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t pa6_load_fit(PA6CS8State *s, const void *fit,
                             const char *path)
{
    int node = fdt_path_offset(fit, path);
    int len, address_len;
    const void *data = fdt_getprop(fit, node, "data", &len);
    const fdt32_t *address = fdt_getprop(fit, node, "load", &address_len);
    uint64_t load;

    if (node < 0 || !data || len <= 0 || !address || address_len != 4) {
        error_report("PA6/CS8 firmware has no loadable %s", path);
        exit(EXIT_FAILURE);
    }
    load = fdt32_to_cpu(*address);
    if (load < PA6_RAM_BASE ||
        load + len > PA6_RAM_BASE + MACHINE(s)->ram_size) {
        error_report("PA6/CS8 FIT payload %s is outside RAM", path);
        exit(EXIT_FAILURE);
    }
    if (!strcmp(path, "/images/fdt-u-boot")) {
        g_autofree void *dt = g_malloc(len + 8192);
        fdt64_t memory[] = {
            cpu_to_fdt64(PA6_RAM_BASE),
            cpu_to_fdt64(MACHINE(s)->ram_size),
        };
        memcpy(dt, data, len);
        int ret = fdt_open_into(dt, dt, len + 8192);
        int node = ret ? ret : fdt_path_offset(dt, "/memory@40000000");

        /* The preloader supplies a pair of 64-bit DRAM values. */
        ret = node < 0 ? node : fdt_setprop(dt, node, "reg", memory, sizeof(memory));
        if (ret) {
            error_report("PA6/CS8 cannot construct the DRAM handoff: %s", fdt_strerror(ret));
            exit(EXIT_FAILURE);
        }
        uint8_t devinfo[0x1400] = { 0 };
        stl_le_p(devinfo, 319);
        /* A separate opt-in virtual development board can use U-Boot's
         * normal maintenance console and bootargs_append. Never change the
         * production handoff implicitly for a driver or setup failure. */
        devinfo[4 + 0x5c] = s->development_mode ? 0 : 2; /* SBC_EN */
        devinfo[4 + 0x5e] = s->development_mode ? 0 : 8; /* AR_EN */
        memcpy(devinfo + 4 + 0x30, "ScribeColorQEMU01", 16);
        node = fdt_path_offset(dt, "/devinfo");
        ret = fdt_setprop(dt, node, "devinfo,data", devinfo, sizeof(devinfo));
        if (ret) {
            error_report("PA6/CS8 cannot construct devinfo: %s", fdt_strerror(ret));
            exit(EXIT_FAILURE);
        }
        /* BL2 0x262278 publishes the reserved connectivity base as one cell. */
        node = fdt_path_offset(dt, "/consys");
        ret = node < 0 ? node : fdt_setprop_u32(dt, node, "emi-addr", 0x49000000);
        if (ret) {
            error_report("PA6/CS8 cannot construct connectivity handoff: %s",
                         fdt_strerror(ret));
            exit(EXIT_FAILURE);
        }
        rom_add_blob_fixed(path, dt, fdt_totalsize(dt), load);
    } else {
        rom_add_blob_fixed(path, data, len, load);
    }
    address = fdt_getprop(fit, node, "entry", &address_len);
    return address && address_len == 4 ? fdt32_to_cpu(*address) : load;
}

/* UFBL IDME v2 records occupy the first ten sectors of boot area 2. */
static void pa6_populate_idme(PA6CS8State *s, DeviceState *card)
{
    char hwid[] = { '0' + mt8115_board_profile_hwid(s->device_profile), 0 };
    uint8_t data[5120] = { 0 };
    uint8_t *cursor = data + 16;
    const struct {
        const char *name, *value;
        unsigned size;
    } fields[] = {
        { "board_id", s->board->board_id, 16 },
        { "serial", s->board->serial, 16 },
        { "mac_addr", "020000000001", 16 },
        { "mac_sec", "0", 32 },
        { "bt_mac_addr", "020000000002", 16 },
        { "product_name", s->board->product_name, 32 },
        { "productid", "0", 32 },
        { "productid2", "0", 32 },
        { "region", "US", 4 },
        { "bootmode", "main", 4 },
        { "postmode", "0", 4 },
        { "bootcount", "0", 8 },
        { "manufacturing", s->board->manufacturing, 512 },
        { "unlock_code", "", 1024 },
        { "device_type_id", s->board->device_type_id, 32 },
        { "dev_flags", "0", 8 },
        { "fos_flags", "0", 8 },
        { "usr_flags", "0", 8 },
        { "hwid", hwid, 4 },
        { "bcm_stress", "0", 1024 },
    };

    memcpy(data, "beefdeed2.1", 11);
    stl_le_p(data + 12, ARRAY_SIZE(fields));
    for (unsigned i = 0; i < ARRAY_SIZE(fields); i++) {
        memcpy(cursor, fields[i].name, strlen(fields[i].name));
        stl_le_p(cursor + 16, fields[i].size);
        stl_le_p(cursor + 20, 1);
        stl_le_p(cursor + 24, 0444);
        memcpy(cursor + 28, fields[i].value, strlen(fields[i].value));
        cursor += ROUND_UP(28 + fields[i].size, 4);
    }
    emmc_boot_partition_write(card, 2, 0, data, sizeof(data), &error_fatal);
}

static void pa6_boot_tags(PA6CS8State *s)
{
    const size_t memory_size = 0x1e2a8;
    const size_t size = 0x98 + 0x68 + 8 + memory_size + 44 + 8;
    g_autofree uint8_t *tags = g_malloc0(size);
    uint8_t *mem = tags + 0x108;
    uint8_t *reason = mem + memory_size;
    uint64_t cursor = PA6_RAM_BASE;
    uint64_t end = cursor + MACHINE(s)->ram_size;
    unsigned free_count = 0;

    /* Byte-sized vendor tags, reconstructed from the shipped ARM64 U-Boot. */
    stl_le_p(tags, 0x98);
    stl_le_p(tags + 4, 0x88610020);
    /*
     * BL2 0x21f8c0 builds this DRAM tag. Its type query returns 10,
     * translated to Linux LPDDR4X (6); supported channels, MR count and
     * frequency list are constants in 0x25b130/1c0/1d0/3a0. Describe the
     * model's single rank consistently with the memory tag below. Mode
     * register manufacturer/revision values await a physical dump.
     */
    tags[8] = 6;
    tags[9] = 2;
    tags[10] = 2;
    tags[11] = 1;
    stw_le_p(tags + 12, 4);
    stw_le_p(tags + 14, 3);
    stq_le_p(tags + 16, MACHINE(s)->ram_size);
    stl_le_p(tags + 0x20, 3733);
    stl_le_p(tags + 0x24, 3200);
    stl_le_p(tags + 0x28, 1600);
    for (unsigned i = 0; i < 4; i++) {
        stw_le_p(tags + 0x34 + i * 4, 5 + i);
    }
    stl_le_p(tags + 0x98, 0x68);
    stl_le_p(tags + 0x9c, 0x88610040);
    /* BL2 0x21ce40 -> U-Boot 0x18640 -> sspm-res-ram-start DT fixup. */
    stl_le_p(tags + 0xf0, 0x9fee0000);
    stl_le_p(tags + 0x100, memory_size + 8);
    stl_le_p(tags + 0x104, 0x88610005);
    stl_le_p(mem, 1);
    stq_le_p(mem + 8, PA6_RAM_BASE);
    stq_le_p(mem + 16, MACHINE(s)->ram_size);
    /*
     * mblock v2: 24-byte free extents and 152-byte reservations. U-Boot
     * appends the latter to the kernel DT itself (0x74ddc), including
     * compatible/no-map properties; leave the signed kernel DT intact.
     */
    for (unsigned i = 0; i < ARRAY_SIZE(pa6_reserved); i++) {
        uint64_t start = pa6_reserved[i].start;
        uint64_t length = pa6_reserved[i].size;
        uint8_t *reserved = mem + 0xc40 + i * 0x98;

        if (start < cursor || start + length > end) {
            error_report("PA6/CS8 DRAM cannot hold preloader reservations");
            exit(EXIT_FAILURE);
        }
        if (start > cursor) {
            uint8_t *free = mem + 0x30 + free_count++ * 24;

            stq_le_p(free, cursor);
            stq_le_p(free + 8, start - cursor);
        }
        stq_le_p(reserved, start);
        stq_le_p(reserved + 8, length);
        memcpy(reserved + 0x14, pa6_reserved[i].name,
               strlen(pa6_reserved[i].name) + 1);
        cursor = start + length;
    }
    if (cursor < end) {
        uint8_t *free = mem + 0x30 + free_count++ * 24;

        stq_le_p(free, cursor);
        stq_le_p(free + 8, end - cursor);
    }
    stl_le_p(mem + 0x28, free_count);
    stl_le_p(mem + 0xc30, 0x99999999);
    stl_le_p(mem + 0xc34, 2);
    stl_le_p(mem + 0xc38, ARRAY_SIZE(pa6_reserved));
    /* BL2 mblock_set_dtb_node (0x2104a0): count then 128-byte path/name pairs. */
    static const char *const links[][2] = {
        { "/debug-kinfo", "aee_debug_kinfo" },
        { "/soc/wifi", "shared-dma-pool_wifi-reserve-memory_dma" },
    };
    uint8_t *dt_links = mem + 0x162a0;

    stl_le_p(dt_links, ARRAY_SIZE(links));
    for (unsigned i = 0; i < ARRAY_SIZE(links); i++) {
        memcpy(dt_links + 4 + i * 256, links[i][0], strlen(links[i][0]) + 1);
        memcpy(dt_links + 4 + i * 256 + 128, links[i][1],
               strlen(links[i][1]) + 1);
    }
    /* Cold-boot context copied by the hibernation initialization path. */
    stl_le_p(reason, 44);
    stl_le_p(reason + 4, 0x88890001);
    rom_add_blob_fixed("mt8115.preloader-tags", tags, size, 0x48600000);
}

static void pa6_boot_component(DeviceState *card, unsigned partition,
                               uint64_t offset, uint64_t capacity,
                               const char *filename)
{
    g_autofree char *data = NULL;
    gsize size;
    if (!filename) {
        return;
    }
    if (!g_file_get_contents(filename, &data, &size, NULL) || size > capacity) {
        error_report("PA6/CS8 boot component does not fit its slot: %s", filename);
        exit(EXIT_FAILURE);
    }
    emmc_boot_partition_write(card, partition, offset, data, size, &error_fatal);
}

static void pa6_aux_load(PA6CS8State *s, const char *filename, bool tee)
{
    g_autofree char *fit = NULL;
    gsize size;
    int node;
    if (!filename) {
        return;
    }
    if (!g_file_get_contents(filename, &fit, &size, NULL) ||
        fdt_check_full(fit, size)) {
        error_report("PA6/CS8 invalid firmware FIT: %s", filename);
        exit(EXIT_FAILURE);
    }
    fdt_for_each_subnode(node, fit, fdt_path_offset(fit, "/images")) {
        const char *name = fdt_get_name(fit, node, NULL);
        g_autofree char *path = g_strdup_printf("/images/%s", name);
        /* ATF and OP-TEE are encrypted; the monitor supplies their ABI. */
        if (tee && strcmp(name, "teeloader")) {
            continue;
        }
        if (!tee && !strcmp(name, "dpm")) {
            int len;
            const uint8_t *data = fdt_getprop(fit, node, "data", &len);

            if (!data || len <= 0) {
                error_report("PA6/CS8 firmware has no DPM payload");
                exit(EXIT_FAILURE);
            }
            pa6_load_fit(s, fit, path);
            uint32_t pm_size = mt8115_preload_dpm(data, len);
            for (unsigned i = 0; i < ARRAY_SIZE(s->dpm); i++) {
                s->dpm[i].pm_size = pm_size;
            }
            continue;
        }
        if (!tee && !strcmp(name, "sspm")) {
            int len;
            const uint8_t *data = fdt_getprop(fit, node, "data", &len);

            if (!data || len <= 0) {
                error_report("PA6/CS8 firmware has no SSPM payload");
                exit(EXIT_FAILURE);
            }
            /* The allocated BL2 virtual xfile pointer is not yet known. */
            mt8115_preload_sspm(data, len, 0x47000000, 0, s->sspm_regs);
            continue;
        }
        pa6_load_fit(s, fit, path);
        if (!tee && !strcmp(name, "pi")) {
            int len;
            const uint8_t *data = fdt_getprop(fit, node, "data", &len);
            if (!data || len <= 0) {
                error_report("PA6/CS8 firmware has no PI payload");
                exit(EXIT_FAILURE);
            }
            mt8115_preload_pi(data, len, 0); /* Current virtual devinfo fuse. */
        }
        if (!tee && !strcmp(name, "mcupm")) {
            int len, address_len;
            const uint8_t *data = fdt_getprop(fit, node, "data", &len);
            const fdt32_t *load = fdt_getprop(fit, node, "load", &address_len);
            uint8_t bootstrap[0x2000], shared[8];
            uint32_t header_size, payload;

            if (!data || len < 0x200 || !load || address_len != 4 ||
                fdt32_to_cpu(*load) != 0x46000000 ||
                ldl_le_p(data) != 0x58881688) {
                error_report("PA6/CS8 invalid MCUPM image");
                exit(EXIT_FAILURE);
            }
            header_size = ldl_le_p(data + 0x34);
            if (header_size > len || len - header_size < sizeof(bootstrap)) {
                error_report("PA6/CS8 truncated MCUPM bootstrap");
                exit(EXIT_FAILURE);
            }
            payload = fdt32_to_cpu(*load) + header_size;
            memcpy(bootstrap, data + header_size, sizeof(bootstrap));
            /* BL2 0x264ba0 copies the bootstrap and installs three segments. */
            stl_le_p(bootstrap + 4, payload);
            stl_le_p(bootstrap + 8, 0x2000);
            stl_le_p(bootstrap + 12, payload + 0x2000);
            stl_le_p(bootstrap + 16, 0x1da88);
            stl_le_p(bootstrap + 20, payload + 0x20000);
            stl_le_p(bootstrap + 24, 0x20000);
            rom_add_blob_fixed("mt8115.mcupm-bootstrap", bootstrap,
                               sizeof(bootstrap), 0x0c540000);
            stl_le_p(shared, payload + 0x20000);
            stl_le_p(shared + 4, 0x20000);
            rom_add_blob_fixed("mt8115.mcupm-shared-info", shared,
                               sizeof(shared), 0x0c55fad4);
            mt8171_mcupm_set_loaded(&s->mcupm, true);
        }
    }
}

static void pa6_reset(void *opaque)
{
    PA6CS8State *s = opaque;

    timer_del(s->systimer_alarm);
    s->systimer_control = s->systimer_ticks = 0;
    s->systimer_pending = false;
    s->systimer_expirations = s->systimer_masked_expirations = 0;
    pa6_systimer_update_irq(s);

    timer_del(s->dsu.alarm);
    memset(&s->dsu, 0, offsetof(PA6DSUPMU, alarm));
    s->dsu.last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    qemu_set_irq(s->dsu.irq, 0);
    memset(s->eint_regs, 0, sizeof(s->eint_regs));
    memset(&s->eint_regs[0x80 / 4], 0xff, 7 * 4);
    pa6_eint_update(s);
    for (unsigned i = 0; i < ARRAY_SIZE(s->usb_ippc); i++) {
        pa6_usb_ippc_reset(&s->usb_ippc[i]);
    }
    memset(s->afe.regs, 0, sizeof(s->afe.regs));
    qemu_irq_lower(s->afe.irq);
    memset(s->auxadc.regs, 0, sizeof(s->auxadc.regs));
    s->auxadc.pending = 0;
    s->auxadc.ready_ns = 0;
    timer_del(s->auxadc.alarm);
    for (unsigned i = 0; i < ARRAY_SIZE(s->u2phy); i++) {
        memset(s->u2phy[i].regs, 0, sizeof(s->u2phy[i].regs));
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->lvts); i++) {
        PA6LVTS *l = &s->lvts[i];

        timer_del(l->alarm);
        memset(l->regs, 0, sizeof(l->regs));
        memset(l->device, 0, sizeof(l->device));
        memset(l->count_rc, 0, sizeof(l->count_rc));
        memset(l->access_done, 0, sizeof(l->access_done));
        memset(l->sensing_done, 0, sizeof(l->sensing_done));
        pa6_lvts_irq(l);
    }
    memset(s->spm.regs, 0, sizeof(s->spm.regs));
    memset(s->spm.settled, 0, sizeof(s->spm.settled));
    for (unsigned i = 0; i < ARRAY_SIZE(pa6_power_domains); i++) {
        unsigned sram = pa6_power_domains[i].sram_bit;
        s->spm.regs[pa6_power_domains[i].control / 4] = BIT(1) | BIT(4) |
            (sram == 8 ? BIT(8) | BIT(12) : 0);
    }
    /* BL2 0x266d40 clock init calls 0x266850/0x266710 when devinfo[4]
     * bits 17/16 are clear (unfused VADSP). Their final PWR_CON is 0x0d.
     * Linux KEEP_DEFAULT_OFF on VADSP_INFRA preserves this preloader state;
     * AUDIO is an enabled child. Physical devinfo still needs capturing. */
    s->spm.regs[0x410 / 4] = 0x0d;
    s->spm.regs[0x414 / 4] = 0x0d;
    pa6_spm_update(&s->spm);
    memset(s->sspm_regs, 0, sizeof(s->sspm_regs));
    for (unsigned i = 0; i < ARRAY_SIZE(s->tiny_mbox); i++) {
        PA6TinyMbox *m = &s->tiny_mbox[i];
        timer_del(m->service_timer);
        m->tx_pending = m->rx_pending = 0;
        qemu_irq_lower(m->irq);
        memset(memory_region_get_ram_ptr(&m->shared), 0, 0x80);
        stl_le_p(memory_region_get_ram_ptr(&m->shared) + 4, 1);
    }

    for (unsigned i = 0; i < ARRAY_SIZE(s->pins); i++) {
        memset(s->pins[i].regs, 0, sizeof(s->pins[i].regs));
    }
    /* gpio_get_values_as_int() consumes this order, not ascending GPIOs.
     * U-Boot detects the phase itself and writes that HWID back to IDME. */
    unsigned hwid = mt8115_board_profile_hwid(s->device_profile);
    for (unsigned i = 0; i < ARRAY_SIZE(s->board->hwid_pins); i++) {
        unsigned pin = s->board->hwid_pins[i];
        if (hwid & BIT(i)) {
            s->pins[0].regs[0x200 / 4 + (pin / 32) * 4] |= BIT(pin % 32);
        }
    }
    /* Explicit fitted-controller strap, sampled as GPIO653 by stock touch.
     * Both variants occur in the signed CS8 DT; this is a virtual profile,
     * not a claim about an uncaptured physical unit's resistor population. */
    if (!g_strcmp0(s->touch_controller, "focaltech")) {
        unsigned pin = s->board->touch_id_pin;
        s->pins[0].regs[0x200 / 4 + (pin / 32) * 4] |= BIT(pin % 32);
    }
    pa6_touch_reset_update(&s->pins[0]);
    for (unsigned i = 0; i < ARRAY_SIZE(s->dramc); i++) {
        /*
         * Reference operating point, not a captured training result:
         * BL2 0x2292d0 programs PCW = rate * 256 / 26 below 3800 MT/s.
         * The vendor fmeter truncates this to 3731 MT/s. Full training,
         * shuffle selection and DPM-driven DVFS are not implemented yet.
         */
        PA6DRAMC *d = &s->dramc[i];
        uint32_t pcw = (3733 * 256 / 26) << 3;

        memset(d->regs, 0, sizeof(d->regs));
        d->regs[0x6908 / 4] = pcw;
        d->regs[0x6928 / 4] = pcw;
        d->regs[0x6910 / 4] = 8;
        d->regs[0x6930 / 4] = 8;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->clocks); i++) {
        memset(s->clocks[i].regs, 0, sizeof(s->clocks[i].regs));
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->dpm); i++) {
        PA6DPM *dpm = &s->dpm[i];

        memset(dpm->regs, 0, sizeof(dpm->regs));
        if (dpm->pm_size) {
            /* BL2's PM SRAM bank selection and reset release. */
            dpm->regs[0] = dpm->pm_size <= 0x4000 ? BIT(29) :
                           dpm->pm_size <= 0x8000 ? BIT(28) : 0;
            dpm->regs[0x7074 / 4] = 1;
        }
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->devapc); i++) {
        PA6DevAPC *apc = &s->devapc[i];

        memset(apc->regs, 0, sizeof(apc->regs));
        memset(apc->permissions, 0, sizeof(apc->permissions));
        memset(apc->regs, 0xff, DIV_ROUND_UP(apc->modules, 32) * 4);
    }
    pa6_devapc_update_irq(s);

    for (unsigned i = 0; i < PA6_NUM_CPUS; i++) {
        cpu_reset(CPU(s->cpu[i]));
        if (!s->boot_stage || !strcmp(s->boot_stage, "u-boot")) {
            /* Stock U-Boot initializes Falcon through its EL1 SVC interface. */
            arm_emulate_firmware_reset(CPU(s->cpu[i]), 1);
            arm_rebuild_hflags(&s->cpu[i]->env);
        }
    }
    cpu_set_pc(CPU(s->cpu[0]), s->entry);
}

/* Local mtk-smi.c packs command 3 in bits[15:8] and larbid in[7:0].
 * SEC_CON fields match the MediaTek TF-A mtk_iommu_smc.c contract:
 * MMU_EN[0], SEC_EN[1], domain[8:4]. Other fields are preserved.
 * The exact encrypted CS8 ATF is not executed by this monitor model. */
static int32_t pa6_iommu_config(uint64_t command, uint64_t ports)
{
    PA6CS8State *s = PA6_CS8(qdev_get_machine());
    BQL_LOCK_GUARD();
    static const struct { hwaddr base; unsigned ports; } larbs[21] = {
        [0] = {0x1401c000, 4}, [1] = {0x1401d000, 3},
        [2] = {0x1f002000, 9}, [3] = {0x1c003000, 12},
        [4] = {0x1602e000, 10}, [7] = {0x17010000, 16},
        [8] = {0x1c001000, 15}, [9] = {0x1502e000, 29},
        [11] = {0x1582e000, 29}, [13] = {0x1a001000, 15},
        [14] = {0x1a002000, 10}, [16] = {0x1a00f000, 17},
        [20] = {0x1b00f000, 6},
    };
    unsigned id = command & 255;
    if ((command >> 8) != 3) {
        return -1;
    }
    if (id >= ARRAY_SIZE(larbs) || !larbs[id].base) {
        return -2;
    }
    /* Virtual MT8171 firmware policy: the stock legacy MDP driver attaches
     * only RDMA1 as its shared DMA client, but programs the same IOVA domain
     * into COLOR and WROT. The stock mdpsys_config DT node declares the full
     * group below; its separate mdpsyscon driver is absent from this kernel.
     * Configure that group's secure ports when the shared client is enabled.
     * This is inferred from the stock DT and DMA usage, not recovered CS8
     * ATF behavior or the newer public TF-A per-port bitmap implementation. */
    if (id == 2 && (ports & BIT_ULL(2))) {
        ports |= s->board->mdp_shared_dma_ports;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(pa6_clock_map); i++) {
        if (pa6_clock_map[i].base != larbs[id].base) {
            continue;
        }
        for (unsigned port = 0; port < larbs[id].ports; port++) {
            uint32_t *reg = &s->clocks[i].regs[0xf80 / 4 + port];
            *reg = (*reg & ~0x1f3U) | ((ports >> port) & 1);
        }
        return 0;
    }
    return -2;
}

static void pa6_init(MachineState *machine)
{
    PA6CS8State *s = PA6_CS8(machine);
    DeviceState *gic;
    QList *redist = qlist_new();
    g_autofree char *firmware = NULL;
    gsize firmware_size;
    DriveInfo *di;
    DeviceState *card;
    static const struct {
        hwaddr pd, ao;
        unsigned modules;
    } devapc_map[] = {
        { 0x10207000, 0x10030000, 291 },
        { 0x10274000, 0x10034000, 39 },
        { 0x10275000, 0x10038000, 97 },
        { 0x11020000, 0x1003c000, 45 },
    };
    static const int timer_irqs[] = {
        [GTIMER_PHYS] = ARCH_TIMER_NS_EL1_IRQ,
        [GTIMER_VIRT] = ARCH_TIMER_VIRT_IRQ,
        [GTIMER_HYP] = ARCH_TIMER_NS_EL2_IRQ,
        [GTIMER_SEC] = ARCH_TIMER_S_EL1_IRQ,
    };
    static const struct {
        hwaddr base, dma;
        unsigned irq;
    } i2c_map[] = {
        { 0x11fc0000, 0x10217080, 152 },
        { 0x11290000, 0x10217280, 144 },
        { 0x11291000, 0x10217480, 145 },
        { 0x11292000, 0x10217500, 146 },
        { 0x11fc1000, 0x10217580, 153 },
        { 0x11293000, 0x10217600, 147 },
        { 0x11294000, 0x10217680, 148 },
        { 0x11295000, 0x10217700, 149 },
        { 0x11296000, 0x10217780, 150 },
        { 0x11297000, 0x10217800, 151 },
    };

    if (!machine->firmware ||
        !g_file_get_contents(machine->firmware, &firmware,
                             &firmware_size, NULL) ||
        fdt_check_full(firmware, firmware_size)) {
        error_report("PA6/CS8 requires -bios with the stock U-Boot FIT");
        exit(EXIT_FAILURE);
    }
    memory_region_add_subregion(get_system_memory(), PA6_RAM_BASE,
                                machine->ram);
    memory_region_init_rom(&s->bootrom, NULL, "mt8115.bootrom", 64 * KiB,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0, &s->bootrom);
    memory_region_init_ram(&s->sram, NULL, "mt8115.sram", MiB,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x100000, &s->sram);
    memory_region_init_ram(&s->preloader_sram, NULL, "mt8115.preloader-sram", MiB,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x200000, &s->preloader_sram);
    memory_region_init_ram(&s->mcupm_sram, NULL, "mt8115.mcupm-sram",
                           0x20000, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x0c540000, &s->mcupm_sram);


    memory_region_init_rom(&s->efuse, NULL, "mt8115.efuse", 0x1000,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x11fb0000, &s->efuse);
    uint8_t *efuse = memory_region_get_ram_ptr(&s->efuse);
    efuse[0x5c] = s->development_mode ? 0 : 2;
    efuse[0x5e] = s->development_mode ? 0 : 8;
    memcpy(efuse + 0x30, "ScribeColorQEMU01", 16);

    for (unsigned i = 0; i < PA6_NUM_CPUS; i++) {
        g_autofree char *name = g_strdup_printf("cpu%u", i);
        Object *cpuobj = object_new(machine->cpu_type);

        s->cpu[i] = ARM_CPU(cpuobj);
        s->cpu[i]->mtk_iommu_config = pa6_iommu_config;
        object_property_add_child(OBJECT(s), name, cpuobj);
        object_property_set_bool(cpuobj, "has_el3",
            s->boot_stage && !strcmp(s->boot_stage, "bl2"), &error_abort);
        /* The stock DT uses affinity level 1 for the four enabled cores. */
        object_property_set_int(cpuobj, "mp-affinity", i << 8, &error_abort);
        object_property_set_bool(cpuobj, "start-powered-off", i != 0,
                                 &error_abort);
        object_property_set_int(cpuobj, "cntfrq", 13000000, &error_abort);
        object_property_set_int(cpuobj, "psci-conduit",
                                QEMU_PSCI_CONDUIT_SMC, &error_abort);
        /* The emulated monitor hands both cold and secondary boot to EL1. */
        object_property_set_int(cpuobj, "psci-target-el", 1, &error_abort);
        object_property_set_int(cpuobj, "mtk-optee-fbe", 1, &error_abort);
        object_property_set_int(cpuobj, "mtk-sip-vcorefs", 1, &error_abort);
        pa6_dsu_register(s->cpu[i]);
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    }

    object_initialize_child(OBJECT(s), "gic", &s->gic, TYPE_ARM_GICV3);
    gic = DEVICE(&s->gic);
    qdev_prop_set_uint32(gic, "num-cpu", PA6_NUM_CPUS);
    qdev_prop_set_uint32(gic, "num-irq", PA6_NUM_SPIS + GIC_INTERNAL);
    qlist_append_int(redist, PA6_NUM_CPUS);
    qdev_prop_set_array(gic, "redist-region-count", redist);
    object_property_set_link(OBJECT(gic), "sysmem",
                             OBJECT(get_system_memory()), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(gic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(gic), 0, 0x0c000000);
    sysbus_mmio_map(SYS_BUS_DEVICE(gic), 1, 0x0c040000);
    for (unsigned cpu = 0; cpu < PA6_NUM_CPUS; cpu++) {
        DeviceState *cpudev = DEVICE(s->cpu[cpu]);
        unsigned ppi_base = PA6_NUM_SPIS + cpu * GIC_INTERNAL;

        for (int i = 0; i < ARRAY_SIZE(timer_irqs); i++) {
            qdev_connect_gpio_out(cpudev, i,
                qdev_get_gpio_in(gic, ppi_base + timer_irqs[i]));
        }
        qdev_connect_gpio_out_named(cpudev, "gicv3-maintenance-interrupt", 0,
            qdev_get_gpio_in(gic, ppi_base + ARCH_GIC_MAINT_IRQ));
        qdev_connect_gpio_out_named(cpudev, "pmu-interrupt", 0,
            qdev_get_gpio_in(gic, ppi_base + 16 + 7));
        for (int i = 0; i < 4; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(gic), cpu + i * PA6_NUM_CPUS,
                               qdev_get_gpio_in(cpudev, i));
        }
    }
    memory_region_init_io(&s->spm.iomem, OBJECT(s), &pa6_spm_ops, &s->spm,
                          "mt8115.spm", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x10006000, &s->spm.iomem);
    memory_region_init_ram(&s->sspm_sram, NULL, "mt8115.sspm-sram",
                           0x2c000, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x10400000, &s->sspm_sram);
    memory_region_init_io(&s->sspm_config, OBJECT(s), &pa6_sspm_ops, s,
                          "mt8115.sspm-config", sizeof(s->sspm_regs));
    memory_region_add_subregion(get_system_memory(), 0x10440000, &s->sspm_config);
    memory_region_init_ram(&s->sspm_shared, NULL, "mt8115.sspm-shared",
                           0x80, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x10480000, &s->sspm_shared);
    for (unsigned i = 0; i < ARRAY_SIZE(s->tiny_mbox); i++) {
        PA6TinyMbox *m = &s->tiny_mbox[i];
        hwaddr base = 0x10450000 + i * 0x10000;
        g_autofree char *name = g_strdup_printf("mt8115.tiny-mbox%u", i);
        g_autofree char *shmem = g_strdup_printf("mt8115.tiny-shmem%u", i);

        m->service_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pa6_scmi_service, m);
        m->irq = qdev_get_gpio_in(gic, 276 + i);
        memory_region_init_io(&m->iomem, OBJECT(s), &pa6_tiny_mbox_ops, m,
                              name, 0x1000);
        memory_region_add_subregion(get_system_memory(), base + 0x1000,
                                    &m->iomem);
        memory_region_init_ram(&m->shared, NULL, shmem, 0x80, &error_fatal);
        memory_region_add_subregion(get_system_memory(), base, &m->shared);
    }
    object_initialize_child(OBJECT(s), "usb-device", &s->usb, TYPE_MTU3);
    sysbus_realize(SYS_BUS_DEVICE(&s->usb), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->usb), 0, 0x11201000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->usb), 0, qdev_get_gpio_in(gic, 128));
    s->usb_ippc[0].mac_reset = qdev_get_gpio_in_named(DEVICE(&s->usb), "reset", 0);
    s->usb_ippc[0].mac_power = qdev_get_gpio_in_named(DEVICE(&s->usb), "power", 0);
    for (unsigned i = 0; i < ARRAY_SIZE(s->usb_ippc); i++) {
        PA6USBIPPC *p = &s->usb_ippc[i];
        g_autofree char *name = g_strdup_printf("mt8115.usb-ippc%u", i);

        p->usb3 = i == 0;
        memory_region_init_io(&p->iomem, OBJECT(s), &pa6_usb_ippc_ops, p,
                              name, sizeof(p->regs));
        memory_region_add_subregion(get_system_memory(),
                                    0x11203e00 + i * 0x10000, &p->iomem);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->u2phy); i++) {
        static const hwaddr addresses[] = { 0x11e80000, 0x11ef0000, 0x11e40000 };
        PA6TPHY *p = &s->u2phy[i];
        g_autofree char *name = g_strdup_printf("mt8115.u2phy%u", i);

        memory_region_init_io(&p->iomem, OBJECT(s), &pa6_tphy_ops, p,
                              name, sizeof(p->regs));
        memory_region_add_subregion(get_system_memory(), addresses[i], &p->iomem);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->lvts); i++) {
        PA6LVTS *l = &s->lvts[i];
        g_autofree char *name = g_strdup_printf("mt8115.lvts%u", i);

        l->first_id = i == 0 ? 0x83 : 0x81;
        l->controllers = i == 0 ? 3 : 2;
        l->irq = qdev_get_gpio_in(gic, 201 + i);
        l->alarm = timer_new_ns(QEMU_CLOCK_VIRTUAL, pa6_lvts_sample, l);
        memory_region_init_io(&l->iomem, OBJECT(s), &pa6_lvts_ops, l,
                              name, 0x1000);
        memory_region_add_subregion(get_system_memory(),
                                    i == 0 ? 0x1100b000 : 0x11278000, &l->iomem);
    }
    s->dsu.irq = qdev_get_gpio_in(gic, 18);
    s->dsu.alarm = timer_new_ns(QEMU_CLOCK_VIRTUAL, pa6_dsu_alarm, &s->dsu);
    s->eint_irq = qdev_get_gpio_in(gic, 244);
    memory_region_init_io(&s->eint, OBJECT(s), &pa6_eint_ops, s,
                          "mt8115.eint", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x1000b000, &s->eint);
    serial_mm_init(get_system_memory(), 0x11002000, 2,
                    qdev_get_gpio_in(gic, 141), 921600,
                    serial_hd(0), DEVICE_LITTLE_ENDIAN);
    s->devapc_irq = qdev_get_gpio_in(gic, 219);
    for (unsigned i = 0; i < ARRAY_SIZE(s->devapc); i++) {
        PA6DevAPC *apc = &s->devapc[i];
        g_autofree char *pd = g_strdup_printf("mt8115.devapc%u.pd", i);
        g_autofree char *ao = g_strdup_printf("mt8115.devapc%u.ao", i);

        apc->board = s;
        apc->modules = devapc_map[i].modules;
        memory_region_init_io(&apc->pd, OBJECT(s), &pa6_devapc_ops, apc,
                              pd, sizeof(apc->regs));
        memory_region_init_io(&apc->ao, OBJECT(s), &pa6_devapc_ao_ops, apc,
                              ao, sizeof(apc->permissions));
        memory_region_add_subregion(get_system_memory(), devapc_map[i].pd,
                                    &apc->pd);
        memory_region_add_subregion(get_system_memory(), devapc_map[i].ao,
                                    &apc->ao);
    }
    create_unimplemented_device("mt8115.uart0-config", 0x11002020, 0xfe0);
    static const hwaddr pin_base[] = {
        0x10005000, 0x11c30000, 0x11d40000, 0x11e20000,
        0x11ea0000, 0x11f10000, 0x11f20000,
    };
    for (unsigned i = 0; i < ARRAY_SIZE(s->pins); i++) {
        g_autofree char *name = g_strdup_printf("mt8115.pinctrl%u", i);

        s->pins[i].gpio = i == 0;
        memory_region_init_io(&s->pins[i].iomem, OBJECT(s), &pa6_pin_ops,
                              &s->pins[i], name, 0x1000);
        memory_region_add_subregion(get_system_memory(), pin_base[i],
                                    &s->pins[i].iomem);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->dramc); i++) {
        g_autofree char *name = g_strdup_printf("mt8115.dramc%u", i);

        memory_region_init_io(&s->dramc[i].iomem, OBJECT(s), &pa6_dramc_ops,
                              &s->dramc[i], name, 0xc000);
        memory_region_add_subregion(get_system_memory(), 0x10230000 + i * 0x10000,
                                    &s->dramc[i].iomem);
    }
    memory_region_init_io(&s->auxadc.iomem, OBJECT(s), &pa6_auxadc_ops,
                          &s->auxadc, "mt8115.auxadc", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x11001000,
                                &s->auxadc.iomem);
    s->afe.irq = qdev_get_gpio_in(gic, 485);
    memory_region_init_io(&s->afe.iomem, OBJECT(s), &pa6_afe_ops, &s->afe,
                          "mt8115.afe", sizeof(s->afe.regs));
    memory_region_add_subregion(get_system_memory(), 0x1e290000, &s->afe.iomem);
    s->auxadc.alarm = timer_new_ns(QEMU_CLOCK_VIRTUAL, pa6_auxadc_complete,
                                   &s->auxadc);
    /* DT thermal tables map channels 0..3 at 900 mV to 25000 mC. These
     * are nominal virtual thermistors, not factory measurements. */
    for (unsigned i = 0; i < 4; i++) {
        s->auxadc.input_mv[i] = 900;
    }
    create_unimplemented_device("mt8115.watchdog", 0x10007000, 0x1000);
    object_initialize_child(OBJECT(s), "consys", &s->consys, TYPE_MT8171_CONSYS);
    s->consys.spm_regs = s->spm.regs;
    for (unsigned i = 0; i < ARRAY_SIZE(pa6_clock_map); i++) {
        if (pa6_clock_map[i].base == 0x10001000) {
            s->consys.infracfg_regs = s->clocks[i].regs;
        }
    }
    if (s->consys_firmware) {
        qdev_prop_set_string(DEVICE(&s->consys), "mcu-firmware", s->consys_firmware);
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->consys), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->consys), 0, 0x18000000);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->consys), 1, 0x10007018, 1);
    static const unsigned consys_irqs[] = {410, 128, 411};
    for (unsigned i = 0; i < ARRAY_SIZE(consys_irqs); i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->consys), i,
                          qdev_get_gpio_in(gic, consys_irqs[i]));
    }
    object_initialize_child(OBJECT(s), "wlan", &s->wlan, TYPE_MT8171_WLAN);
    s->wlan.consys = &s->consys;
    sysbus_realize(SYS_BUS_DEVICE(&s->wlan), &error_fatal);
    qdev_connect_gpio_out_named(DEVICE(&s->consys), "reset", 0,
        qdev_get_gpio_in_named(DEVICE(&s->wlan), "reset", 0));
    s->spm.conn_power = qdev_get_gpio_in_named(DEVICE(&s->consys), "power", 0);
    qemu_set_irq(s->spm.conn_power, (s->spm.regs[0x304 / 4] & 0xc) == 0xc);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->wlan), 0, 0x18004000, 1);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->wlan), 1, 0x180b1000, 1);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->wlan), 2, 0x180c1140, 1);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->wlan), 3, 0x18007000, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->wlan), 0,
                      qdev_get_gpio_in(gic, 409));
    object_initialize_child(OBJECT(s), "btif", &s->btif, TYPE_MT8171_BTIF);
    s->btif.consys = &s->consys;
    sysbus_realize(SYS_BUS_DEVICE(&s->btif), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->btif), 0, 0x1100c000);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->btif), 1, 0x10217c80);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->btif), 2, 0x10217d00);
    static const unsigned btif_irqs[] = {190, 184, 185};
    for (unsigned i = 0; i < ARRAY_SIZE(btif_irqs); i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->btif), i,
                           qdev_get_gpio_in(gic, btif_irqs[i]));
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->clocks); i++) {
        PA6ClockBank *bank = &s->clocks[i];

        bank->kind = pa6_clock_map[i].kind;
        bank->board = s;
        bank->address = pa6_clock_map[i].base;
        memory_region_init_io(&bank->iomem, OBJECT(s), &pa6_clock_ops, bank,
                              pa6_clock_map[i].name,
                              pa6_clock_map[i].size ?: sizeof(bank->regs));
        memory_region_add_subregion(get_system_memory(), pa6_clock_map[i].base,
                                    &bank->iomem);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->dpm); i++) {
        PA6DPM *dpm = &s->dpm[i];
        hwaddr base = 0x10900000 + i * 0x100000;
        g_autofree char *pm = g_strdup_printf("mt8115.dpm%u.pm", i);
        g_autofree char *dm = g_strdup_printf("mt8115.dpm%u.dm", i);
        g_autofree char *config = g_strdup_printf("mt8115.dpm%u.config", i);

        memory_region_init_ram(&dpm->pm, NULL, pm, 0xc000, &error_fatal);
        memory_region_init_ram(&dpm->dm, NULL, dm, 0x4000, &error_fatal);
        memory_region_init_io(&dpm->config, OBJECT(s), &pa6_dpm_ops, dpm,
                              config, sizeof(dpm->regs));
        memory_region_add_subregion(get_system_memory(), base, &dpm->pm);
        memory_region_add_subregion(get_system_memory(), base + 0x20000,
                                    &dpm->dm);
        memory_region_add_subregion(get_system_memory(), base + 0x40000,
                                    &dpm->config);
    }
    for (int i = 0; i < ARRAY_SIZE(i2c_map); i++) {
        g_autofree char *name = g_strdup_printf("i2c%d", i);

        object_initialize_child(OBJECT(s), name, &s->i2c[i], TYPE_MTK_I2C_V2);
        qdev_prop_set_uint32(DEVICE(&s->i2c[i]), "ap-offset", i == 0 ? 0x100 : 0);
        sysbus_realize(SYS_BUS_DEVICE(&s->i2c[i]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 0, i2c_map[i].base);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 1, i2c_map[i].dma);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                          qdev_get_gpio_in(gic, i2c_map[i].irq));
    }
    I2CSlave *pmic_slave = i2c_slave_new("bd72720", 0x4b);
    DeviceState *pmic = DEVICE(pmic_slave);
    object_property_add_child(OBJECT(s), "pmic", OBJECT(pmic_slave));
    i2c_slave_realize_and_unref(pmic_slave, s->i2c[0].bus, &error_fatal);
    qdev_connect_gpio_out(pmic, 0, qemu_allocate_irq(pa6_eint_input, s, 11));
    I2CSlave *charger = i2c_slave_new("bd72720", 0x4c);
    qdev_prop_set_bit(DEVICE(charger), "charger", true);
    i2c_slave_realize_and_unref(charger, s->i2c[0].bus, &error_fatal);

    if (s->touch_firmware) {
        bool focaltech = !g_strcmp0(s->touch_controller, "focaltech");
        if (!focaltech && g_strcmp0(s->touch_controller, "parade")) {
            error_report("PA6 touch-firmware requires explicit touch-controller=parade or focaltech");
            exit(EXIT_FAILURE);
        }
        I2CSlave *touch = i2c_slave_new(
            focaltech ? "focaltech-ft3a81" : TYPE_PARADE_TT7010,
            focaltech ? 0x38 : 0x24);
        qdev_prop_set_string(DEVICE(touch), "firmware-image", s->touch_firmware);
        qdev_prop_set_uint16(DEVICE(touch), "resolution-x", s->board->touch_width);
        qdev_prop_set_uint16(DEVICE(touch), "resolution-y", s->board->touch_height);
        if (focaltech) {
            qdev_prop_set_uint8(DEVICE(touch), "vendor-id", s->board->touch_vendor_id);
        }
        i2c_slave_realize_and_unref(touch, s->i2c[s->board->touch_bus].bus,
                                   &error_fatal);
        qdev_connect_gpio_out(DEVICE(touch), 0,
            qemu_allocate_irq(pa6_eint_input, s, s->board->touch_eint));
        s->pins[0].touch_reset =
            qdev_get_gpio_in_named(DEVICE(touch), "reset", 0);
        s->pins[0].touch_reset_pin = s->board->touch_reset_pin;
        pa6_touch_reset_update(&s->pins[0]);
    }

    for (unsigned i = 0; i < s->board->num_i2c_devices; i++) {
        const MT8115I2CFittedDevice *fitted = &s->board->i2c_devices[i];
        DeviceState *dev = DEVICE(i2c_slave_create_simple(
            s->i2c[fitted->bus].bus, fitted->type, fitted->address));
        for (unsigned irq = 0; irq < ARRAY_SIZE(fitted->eint); irq++) {
            if (fitted->eint[irq] >= 0) {
                qdev_connect_gpio_out_named(dev, fitted->irq_name, irq,
                    qemu_allocate_irq(pa6_eint_input, s, fitted->eint[irq]));
            }
        }
        if (fitted->pmic_enable_gpio >= 0) {
            qdev_connect_gpio_out_named(pmic, "gpio", fitted->pmic_enable_gpio,
                qdev_get_gpio_in_named(dev, "enable", 0));
        }
    }

    memory_region_init_io(&s->timer, OBJECT(s), &pa6_timer_ops, s,
                          "mt8115.apxgpt", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x10008000, &s->timer);

    s->systimer_irq = qdev_get_gpio_in(gic, 265);
    s->systimer_alarm = timer_new_ns(QEMU_CLOCK_VIRTUAL, pa6_systimer_expire, s);
    object_property_add_uint64_ptr(OBJECT(s), "systimer-expirations",
        &s->systimer_expirations, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(OBJECT(s), "systimer-masked-expirations",
        &s->systimer_masked_expirations, OBJ_PROP_FLAG_READ);
    memory_region_init_io(&s->systimer, OBJECT(s), &pa6_systimer_ops, s,
                          "mt8115.systimer", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x10017000, &s->systimer);

    object_initialize_child(OBJECT(s), "mcupm", &s->mcupm, TYPE_MT8171_MCUPM);
    sysbus_realize(SYS_BUS_DEVICE(&s->mcupm), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->mcupm), 0, 0x0c560000);
    for (unsigned i = 0; i < 8; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->mcupm), i,
                          qdev_get_gpio_in(gic, 33 + i));
    }

    object_initialize_child(OBJECT(s), "m4u", &s->m4u, TYPE_MT8171_M4U);
    sysbus_realize(SYS_BUS_DEVICE(&s->m4u), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->m4u), 0, 0x14020000);
    for (unsigned i = 0; i < 5; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->m4u), i,
                          qdev_get_gpio_in(gic, 319 + i));
    }
    object_initialize_child(OBJECT(s), "gce", &s->gce, TYPE_MT8171_GCE);
    sysbus_realize(SYS_BUS_DEVICE(&s->gce), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gce), 0, 0x10228000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gce), 0, qdev_get_gpio_in(gic, 235));

    object_initialize_child(OBJECT(s), "mdp", &s->mdp, TYPE_MT8171_MDP);
    s->mdp.m4u = &s->m4u;
    for (unsigned i = 0; i < ARRAY_SIZE(pa6_clock_map); i++) {
        if (pa6_clock_map[i].base == 0x1f002000) {
            s->mdp.larb_regs = s->clocks[i].regs;
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->mdp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->mdp), 0, 0x1f000000);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->mdp), 1, 0x1f003000);
    for (unsigned i = 0; i < 20; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->mdp), i,
                          qdev_get_gpio_in(gic, 584 + i));
    }
    for (unsigned i = 0; i < 1024; i++) {
        qdev_connect_gpio_out_named(DEVICE(&s->mdp), "event", i,
            qdev_get_gpio_in_named(DEVICE(&s->gce), "event", i));
    }

    object_initialize_child(OBJECT(s), "hwtcon", &s->hwtcon, TYPE_MT8171_HWTCON);
    s->hwtcon.m4u = &s->m4u;
    for (unsigned i = 0; i < ARRAY_SIZE(pa6_clock_map); i++) {
        if (pa6_clock_map[i].base == 0x1c001000) {
            s->hwtcon.larb8_regs = s->clocks[i].regs;
        }
        if (pa6_clock_map[i].base == 0x1c003000) {
            s->hwtcon.larb3_regs = s->clocks[i].regs;
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->hwtcon), &error_fatal);
    /* The existing SMI larbs retain their own register ownership. */
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->hwtcon), 0, 0x1c001000, -1);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->hwtcon), 1, 0x14001000, -1);
    static const unsigned hwtcon_irqs[] = {414, 498, 518, 517, 519, 421, 422, 425, 412};
    for (unsigned i = 0; i < ARRAY_SIZE(hwtcon_irqs); i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->hwtcon), i,
                          qdev_get_gpio_in(gic, hwtcon_irqs[i]));
    }
    static const unsigned hwtcon_events[] = {331, 333, 169};
    for (unsigned i = 0; i < ARRAY_SIZE(hwtcon_events); i++) {
        qdev_connect_gpio_out_named(DEVICE(&s->hwtcon), "event", i,
            qdev_get_gpio_in_named(DEVICE(&s->gce), "event", hwtcon_events[i]));
    }

    object_initialize_child(OBJECT(s), "panel-preview", &s->panel, TYPE_MT8171_PANEL);
    s->panel.hwtcon_regs = s->hwtcon.bank[0].regs;
    qdev_prop_set_uint32(DEVICE(&s->panel), "width", s->board->panel_width);
    qdev_prop_set_uint32(DEVICE(&s->panel), "height", s->board->panel_height);
    sysbus_realize(SYS_BUS_DEVICE(&s->panel), &error_fatal);
    s->mdp.panel = &s->panel;

    object_initialize_child(OBJECT(s), "msdc0", &s->msdc, TYPE_MTK_MSDC);
    sysbus_realize(SYS_BUS_DEVICE(&s->msdc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->msdc), 0, 0x11230000);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->msdc), 1, 0x11ed0000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->msdc), 0,
                        qdev_get_gpio_in(gic, 131));
    di = drive_get(IF_SD, 0, 0);
    card = qdev_new(TYPE_EMMC);
    qdev_prop_set_uint64(card, "boot-partition-size", 4 * MiB);
    qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    qdev_prop_set_drive_err(card, "drive",
                            di ? blk_by_legacy_dinfo(di) : NULL, &error_fatal);
    qdev_realize(card, qdev_get_child_bus(DEVICE(&s->msdc), "sd-bus"),
                  &error_fatal);
    /* Hardware boot-area geometry from stock g_custom_partitions. */
    pa6_boot_component(card, 1, 0, MiB, s->bl2);
    pa6_boot_component(card, 1, MiB, MiB, machine->firmware);
    pa6_boot_component(card, 2, MiB, 3 * MiB, s->tee);
    pa6_aux_load(s, s->tee, true);
    pa6_aux_load(s, s->aux_firmware, false);
    if (s->quickboot) {
        g_autofree char *quickboot = NULL;
        gsize size;
        if (!g_file_get_contents(s->quickboot, &quickboot, &size, NULL) ||
            size > MiB || fdt_check_full(quickboot, size)) {
            error_report("PA6/CS8 quickboot must be a FIT image fitting its 1 MiB slot");
            exit(EXIT_FAILURE);
        }
        emmc_boot_partition_write(card, 1, 0x1000 * 512, quickboot,
                                  size, &error_fatal);
        pa6_load_fit(s, quickboot, "/images/fbios");
        pa6_load_fit(s, quickboot, "/images/sbios");
    }
    pa6_populate_idme(s, card);
    object_unref(OBJECT(card));

    s->entry = pa6_load_fit(s, firmware, "/images/kernel");
    pa6_load_fit(s, firmware, "/images/fdt-u-boot");
    if (s->boot_stage && !strcmp(s->boot_stage, "bl2")) {
        g_autofree char *bl2 = NULL;
        gsize size;
        if (!s->bl2 || !g_file_get_contents(s->bl2, &bl2, &size, NULL) ||
            size < 0x8f4 || memcmp(bl2 + 0x800, "MMM", 3)) {
            error_report("PA6/CS8 boot-stage=bl2 requires a MediaTek BL2 image");
            exit(EXIT_FAILURE);
        }
        uint32_t load = ldl_le_p(bl2 + 0x81c);
        uint32_t entry_offset = ldl_le_p(bl2 + 0x830);
        if (load < 0x200000 || load + size - 0x800 > 0x300000 ||
            entry_offset >= size - 0x800 || (entry_offset & 3)) {
            error_report("PA6/CS8 BL2 does not fit preloader SRAM");
            exit(EXIT_FAILURE);
        }
        rom_add_blob_fixed("mt8115.bl2", bl2 + 0x800, size - 0x800, load);
        s->entry = load + entry_offset;
    } else if (s->boot_stage && strcmp(s->boot_stage, "u-boot")) {
        error_report("PA6/CS8 boot-stage must be bl2 or u-boot");
        exit(EXIT_FAILURE);
    }
    pa6_boot_tags(s);
    qemu_register_reset(pa6_reset, s);
}

#define PA6_STRING_PROPERTY(member) \
static char *pa6_get_##member(Object *obj, Error **errp) \
{ \
    return g_strdup(PA6_CS8(obj)->member); \
} \
static void pa6_set_##member(Object *obj, const char *value, Error **errp) \
{ \
    PA6CS8State *s = PA6_CS8(obj); \
    g_free(s->member); \
    s->member = g_strdup(value); \
}

PA6_STRING_PROPERTY(quickboot)
PA6_STRING_PROPERTY(bl2)
PA6_STRING_PROPERTY(boot_stage)
PA6_STRING_PROPERTY(tee)
PA6_STRING_PROPERTY(aux_firmware)
PA6_STRING_PROPERTY(touch_firmware)
PA6_STRING_PROPERTY(touch_controller)
PA6_STRING_PROPERTY(consys_firmware)

static char *pa6_get_device_profile(Object *obj, Error **errp)
{
    return g_strdup(PA6_CS8(obj)->device_profile);
}

static void pa6_set_device_profile(Object *obj, const char *value, Error **errp)
{
    if (mt8115_board_profile_hwid(value) < 0) {
        error_setg(errp, "Invalid MT8115 device-profile '%s'; expected production, "
                   "dvt, evt, or hvt1.1", value);
        return;
    }
    g_free(PA6_CS8(obj)->device_profile);
    PA6_CS8(obj)->device_profile = g_strdup(value);
}

static char *pa6_get_board(Object *obj, Error **errp)
{
    return g_strdup(PA6_CS8(obj)->board->name);
}

static void pa6_set_board(Object *obj, const char *value, Error **errp)
{
    const MT8115BoardConfig *board = mt8115_board_config(value);
    if (!board) {
        error_setg(errp, "Invalid MT8115 board '%s'; expected calvados or paloma",
                   value);
        return;
    }
    PA6_CS8(obj)->board = board;
}

static void pa6_instance_init(Object *obj)
{
    PA6_CS8(obj)->device_profile = g_strdup("production");
    PA6_CS8(obj)->board = mt8115_board_config("calvados");
}

static bool pa6_get_development_mode(Object *obj, Error **errp)
{
    return PA6_CS8(obj)->development_mode;
}

static void pa6_set_development_mode(Object *obj, bool value, Error **errp)
{
    PA6_CS8(obj)->development_mode = value;
}

static void pa6_finalize(Object *obj)
{
    PA6CS8State *s = PA6_CS8(obj);

    g_free(s->device_profile);
    g_free(s->bl2);
    g_free(s->tee);
    g_free(s->aux_firmware);
    g_free(s->touch_firmware);
    g_free(s->touch_controller);
    g_free(s->consys_firmware);
    g_free(s->quickboot);
    g_free(s->boot_stage);
    if (s->systimer_alarm) {
        timer_free(s->systimer_alarm);
    }
}

static void pa6_class_init(ObjectClass *klass, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(klass);

    object_class_property_add_str(klass, "board", pa6_get_board, pa6_set_board);
    object_class_property_set_description(klass, "board",
        "Fitted board: calvados (Scribe Colorsoft, default) or paloma (Scribe 3)");
    object_class_property_add_str(klass, "quickboot", pa6_get_quickboot,
                                  pa6_set_quickboot);
    object_class_property_add_str(klass, "boot-stage", pa6_get_boot_stage,
                                  pa6_set_boot_stage);
    object_class_property_add_str(klass, "bl2", pa6_get_bl2, pa6_set_bl2);
    object_class_property_add_str(klass, "tee", pa6_get_tee, pa6_set_tee);
    object_class_property_add_str(klass, "aux-firmware", pa6_get_aux_firmware,
                                  pa6_set_aux_firmware);
    object_class_property_add_str(klass, "touch-firmware", pa6_get_touch_firmware,
                                  pa6_set_touch_firmware);
    object_class_property_add_str(klass, "touch-controller", pa6_get_touch_controller,
                                  pa6_set_touch_controller);
    object_class_property_add_str(klass, "consys-firmware", pa6_get_consys_firmware,
                                  pa6_set_consys_firmware);
    object_class_property_add_str(klass, "device-profile",
                                  pa6_get_device_profile, pa6_set_device_profile);
    object_class_property_set_description(klass, "device-profile",
        "Hardware phase: production (DVT), dvt, evt, or hvt1.1");
    object_class_property_add_bool(klass, "development-mode",
                                   pa6_get_development_mode,
                                   pa6_set_development_mode);
    object_class_property_set_description(klass, "development-mode",
        "Unfused virtual SBC/AR policy; independent of hardware device-profile");
    mc->desc = "Amazon Kindle Scribe 3 / Colorsoft (MT8115 PA6/CS8)";
    mc->init = pa6_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a55");
    mc->default_ram_size = 4 * GiB;
    mc->default_ram_id = "mt8115.ram";
    mc->min_cpus = PA6_NUM_CPUS;
    mc->max_cpus = PA6_NUM_CPUS;
    mc->default_cpus = PA6_NUM_CPUS;
    mc->no_cdrom = true;
    mc->auto_create_sdcard = false;
}

static const TypeInfo pa6_type = {
    .name = TYPE_PA6_CS8,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(PA6CS8State),
    .instance_init = pa6_instance_init,
    .instance_finalize = pa6_finalize,
    .class_init = pa6_class_init,
    .interfaces = aarch64_machine_interfaces,
};

static void pa6_register_types(void)
{
    type_register_static(&pa6_type);
}

type_init(pa6_register_types)
