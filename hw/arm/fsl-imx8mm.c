/*
 * NXP i.MX8M Mini application processor and peripheral interconnect.
 * CPU/GIC wiring follows fsl-imx8mp.c (Bernhard Beschow, 2024).
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/fsl-imx8mm.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/unimp.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qobject/qlist.h"
#include "system/address-spaces.h"
#include "qemu/module.h"
#include "system/system.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/internals.h"

enum {
    CTRL_SRC, CTRL_GPC, CTRL_OCOTP, CTRL_IOMUXC, CTRL_GPR, CTRL_DDRC,
    CTRL_MIPI, CTRL_DISPMIX, CTRL_TZASC, CTRL_RDC, CTRL_DDR_PERF,
    CTRL_DDR_GPR, CTRL_CSU, CTRL_NOC,
    CTRL_TMU, CTRL_USBMISC1, CTRL_USBMISC2,
};
static const hwaddr control_bases[] = {
    0x30390000, 0x303a0000, 0x30350000, 0x30330000, 0x30340000,
    0x3d400000, 0x32e10000, 0x32e28000, 0x32f80000,
    0x303d0000, 0x3d800000, 0x3d000000, 0x32df0000, 0x32700000,
    0x30260000, 0x32e40200, 0x32e50200,
};

static hwaddr imx8mm_reset_vector(FslIMX8MMState *s, unsigned core)
{
    uint32_t *src = s->registers[CTRL_SRC];
    return ((uint64_t)(src[(0x74 + core * 8) / 4] & 0xffff) << 24) |
           ((uint64_t)(src[(0x78 + core * 8) / 4] & 0x3fffff) << 2);
}

static void imx8mm_warm_reset(CPUState *cpu, run_on_cpu_data data)
{
    FslIMX8MMState *s = data.host_ptr;
    unsigned core = cpu->cpu_index;
    hwaddr entry = imx8mm_reset_vector(s, core);
    cpu_reset(cpu);
    cpu_set_pc(cpu, entry);
    ARM_CPU(cpu)->power_state = PSCI_ON;
    cpu->halted = false;
    for (unsigned line = 0; line < 4; line++) {
        qemu_set_irq(qdev_get_gpio_in(DEVICE(cpu), line),
                     !!(s->cpu_irq_levels & BIT(core + line * 4)));
    }
}

static void imx8mm_wake(FslIMX8MMState *s, unsigned core)
{
    if (s->sleeping[core] && ((s->wake_levels & BIT(core)) ||
        (s->cpu_irq_levels & (0x1111 << core)))) {
        s->sleeping[core] = s->powerdown[core] = false;
        async_run_on_cpu(CPU(&s->cpu[core]), imx8mm_warm_reset,
                         RUN_ON_CPU_HOST_PTR(s));
    }
}

static void imx8mm_cpu_wakeup(void *opaque, int n, int level)
{
    FslIMX8MMState *s = opaque;
    s->wake_levels = deposit32(s->wake_levels, n, 1, level);
    imx8mm_wake(s, n);
}

static void imx8mm_power_tick(void *opaque)
{
    FslIMX8MMState *s = opaque;
    bool pending = false;
    for (unsigned core = 0; core < 4; core++) {
        CPUState *cpu = CPU(&s->cpu[core]);
        if (s->powerdown[core] && !s->sleeping[core]) {
            /* The GPC gates an armed core when its firmware executes WFI. */
            uint32_t previous = 0;
            if (arm_current_el(&s->cpu[core].env) == 3) {
                cpu_physical_memory_read(s->cpu[core].env.pc - 4, &previous, 4);
                s->sleeping[core] = cpu->halted || le32_to_cpu(previous) == 0xd503207f;
                if (s->sleeping[core]) {
                    s->cpu[core].power_state = PSCI_OFF;
                    cpu->halted = true;
                }
            }
            pending |= !s->sleeping[core];
        }
        imx8mm_wake(s, core);
    }
    if (pending) {
        timer_mod_ns(&s->power_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
    }
}

static void imx8mm_cpu_irq(void *opaque, int n, int level)
{
    FslIMX8MMState *s = opaque;
    unsigned core = n % 4, line = n / 4;
    s->cpu_irq_levels = deposit32(s->cpu_irq_levels, n, 1, level);
    imx8mm_wake(s, core);
    qemu_set_irq(qdev_get_gpio_in(DEVICE(&s->cpu[core]), line), level);
}

static uint64_t imx8mm_control_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX8MMControlRegion *r = opaque;
    FslIMX8MMState *s = r->soc;
    uint32_t *regs = s->registers[r->index];
    if (r->index == CTRL_TMU && (offset == 0x20 || offset == 0x24)) {
        return (regs[0] & BIT(31)) ? BIT(31) | 25 : 0;
    }
    if (r->index == CTRL_TZASC && offset == 0) {
        return 0x2307; /* TZC-380: 36-bit addresses, eight regions. */
    }
    if (r->index == CTRL_DDRC) {
        switch (offset) {
        case 0x004:
            if (regs[0x030 / 4] & BIT(6)) {
                return regs[0x030 / 4] & BIT(5) ? 0x123 : 0x323;
            }
            return regs[0x030 / 4] & BIT(5) ? 0x223 : 1;
        case 0x018: case 0x3fc: return 0;
        case 0x308: return 0x36000000;
        case 0x324: return regs[0x320 / 4] & 1;
        }
    }
    if (r->index == CTRL_GPC &&
        (offset == 0xf0 || offset == 0xf4 || offset == 0xf8 || offset == 0x104)) {
        return 0; /* Power-domain transition requests complete. */
    }
    if (r->index == CTRL_GPC && offset == 0x1fc) {
        uint32_t request = regs[offset / 4] & 0x3fff;
        return request | (request << 18); /* Domain-clock handshake ACKs. */
    }
    if (r->index == CTRL_MIPI) {
        switch (offset) {
        case 0x00: return 0x01060200;
        case 0x04: return BIT(31) | BIT(20) | BIT(10) | BIT(8) | 15;
        case 0x4c: return regs[offset / 4] | BIT(24) | BIT(22) | BIT(20) | BIT(10) | BIT(8);
        }
    }
    return extract32(regs[offset / 4], (offset & 3) * 8, size * 8);
}

static void imx8mm_control_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    IMX8MMControlRegion *r = opaque;
    uint32_t *reg = &r->soc->registers[r->index][offset / 4];
    if (r->index == CTRL_MIPI) {
        uint32_t *regs = r->soc->registers[CTRL_MIPI];
        if (offset == 0x34) {
            regs[0x34 / 4] &= ~value;
        } else {
            *reg = value;
            if (offset == 0x0c) {
                *reg = 0;
                regs[0x34 / 4] |= BIT(30);
            } else if (offset == 0x94 && (value & BIT(23))) {
                regs[0x34 / 4] |= BIT(31);
            } else if (offset == 0x3c) {
                regs[0x34 / 4] |= BIT(29) | BIT(28);
            }
        }
        qemu_set_irq(qdev_get_gpio_in(DEVICE(&r->soc->gic), 18),
                     (regs[0x34 / 4] & ~regs[0x38 / 4]) != 0);
        return;
    }
    if (r->index == CTRL_DDRC && offset == 0x10) {
        value &= ~BIT(31); /* Mode-register command accepted. */
    }
    if (r->index == CTRL_DDRC && offset == 0x1b0) {
        FslIMX8MMState *s = r->soc;
        /*
         * The first DFI init request after DDR reset completes PHY startup.
         * Later requests drop init_complete until init_start is released:
         * firmware uses this handshake for frequency changes and retention.
         */
        if (value & BIT(5)) {
            if (!(*reg & BIT(5))) {
                s->registers[CTRL_DDRC][0x1bc / 4] = !s->ddr_initialized;
                s->ddr_initialized = true;
            }
        } else {
            s->registers[CTRL_DDRC][0x1bc / 4] = s->ddr_initialized;
        }
    }
    if (r->index == CTRL_SRC && offset == 0x1000 && (value & BIT(0))) {
        r->soc->ddr_initialized = false;
        r->soc->registers[CTRL_DDRC][0x1bc / 4] = 0;
    }
    if (r->index == CTRL_GPC && offset == 0x1fc) {
        value &= 0x3fff; /* ACK fields are read-only. */
    }
    if (r->index == CTRL_SRC && offset == 8) {
        uint32_t released = value & ~*reg;
        for (unsigned core = 1; core < IMX8MM_NUM_CPUS; core++) {
            if (released & BIT(core)) {
                /* TF-A installs the secure reset vector before releasing POR. */
                arm_set_cpu_on(core, imx8mm_reset_vector(r->soc, core), 0, 3, true);
            }
        }
    }
    *reg = deposit32(*reg, (offset & 3) * 8, size * 8, value);
    if (r->index == CTRL_GPC && offset >= 0x800 && offset <= 0x8c0 &&
        !(offset & 0x3f)) {
        FslIMX8MMState *s = r->soc;
        unsigned core = (offset - 0x800) / 0x40;
        unsigned wfi_bit = core < 2 ? core * 2 : (core - 2) * 2 + 16;
        s->powerdown[core] = (value & 1) &&
            (s->registers[CTRL_GPC][4 / 4] & BIT(wfi_bit));
        s->sleeping[core] = false;
        if (s->powerdown[core]) {
            timer_mod_ns(&s->power_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
        }
    }
}

static uint64_t imx8mm_phy_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX8MMControlRegion *r = opaque;
    FslIMX8MMState *s = r->soc;
    switch (offset / 4) {
    case 0xd0004: return s->phy[0xd0031] ? 0 : 1;
    case 0xd0032: return 7; /* Training firmware's completion mailbox. */
    }
    return extract32(s->phy[offset / 4], (offset & 3) * 8, size * 8);
}

static void imx8mm_phy_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    IMX8MMControlRegion *r = opaque;
    uint32_t *reg = &r->soc->phy[offset / 4];
    *reg = deposit32(*reg, (offset & 3) * 8, size * 8, value);
}

static const MemoryRegionOps imx8mm_control_ops = {
    .read = imx8mm_control_read, .write = imx8mm_control_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};
static const MemoryRegionOps imx8mm_phy_ops = {
    .read = imx8mm_phy_read, .write = imx8mm_phy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 4 },
};

static void imx8mm_map(SysBusDevice *dev, hwaddr base, DeviceState *gic, int irq)
{
    sysbus_realize(dev, &error_fatal);
    sysbus_mmio_map(dev, 0, base);
    if (irq >= 0) {
        sysbus_connect_irq(dev, 0, qdev_get_gpio_in(gic, irq));
    }
}

static void fsl_imx8mm_init(Object *obj)
{
    FslIMX8MMState *s = FSL_IMX8MM(obj);
    qdev_init_gpio_in_named(DEVICE(obj), imx8mm_cpu_irq, "cpu-irq", 16);
    qdev_init_gpio_in_named(DEVICE(obj), imx8mm_cpu_wakeup, "cpu-wakeup", 4);
    timer_init_ns(&s->power_timer, QEMU_CLOCK_VIRTUAL, imx8mm_power_tick, s);
    object_initialize_child(obj, "gic", &s->gic, gicv3_class_name());
    object_initialize_child(obj, "ccm", &s->ccm, TYPE_IMX8MP_CCM);
    object_initialize_child(obj, "analog", &s->analog, TYPE_IMX8MP_ANALOG);
    object_initialize_child(obj, "snvs", &s->snvs, TYPE_IMX7_SNVS);
    object_initialize_child(obj, "lcdif", &s->lcdif, TYPE_IMX_LCDIF);
    object_initialize_child(obj, "caam", &s->caam, TYPE_IMX_CAAM);
    for (unsigned i = 0; i < 2; i++) {
        g_autofree char *name = g_strdup_printf("usb%u", i + 1);
        object_initialize_child(obj, name, &s->usb[i], TYPE_CHIPIDEA);
    }
    for (unsigned i = 0; i < 5; i++) {
        g_autofree char *name = g_strdup_printf("gpio%u", i + 1);
        object_initialize_child(obj, name, &s->gpio[i], TYPE_IMX_GPIO);
    }
    for (unsigned i = 0; i < 4; i++) {
        g_autofree char *uart = g_strdup_printf("uart%u", i + 1);
        g_autofree char *i2c = g_strdup_printf("i2c%u", i + 1);
        object_initialize_child(obj, uart, &s->uart[i], TYPE_IMX_SERIAL);
        object_initialize_child(obj, i2c, &s->i2c[i], TYPE_IMX_I2C);
    }
    for (unsigned i = 0; i < 3; i++) {
        g_autofree char *sd = g_strdup_printf("usdhc%u", i + 1);
        g_autofree char *spi = g_strdup_printf("spi%u", i + 1);
        g_autofree char *wdt = g_strdup_printf("wdt%u", i + 1);
        g_autofree char *sdma = g_strdup_printf("sdma%u", i + 1);
        object_initialize_child(obj, sd, &s->usdhc[i], TYPE_IMX_USDHC);
        object_initialize_child(obj, spi, &s->spi[i], TYPE_IMX_SPI);
        object_initialize_child(obj, wdt, &s->wdt[i], TYPE_IMX2_WDT);
        object_initialize_child(obj, sdma, &s->sdma[i], TYPE_IMX_SDMA);
    }
}

static void fsl_imx8mm_realize(DeviceState *dev, Error **errp)
{
    FslIMX8MMState *s = FSL_IMX8MM(dev);
    MachineState *ms = MACHINE(qdev_get_machine());
    DeviceState *gic = DEVICE(&s->gic);
    SysBusDevice *gicsbd = SYS_BUS_DEVICE(gic);
    QList *redists = qlist_new();
    static const hwaddr uart_bases[] = {0x30860000, 0x30890000, 0x30880000, 0x30a60000};
    static const unsigned uart_irqs[] = {26, 27, 28, 29};
    for (unsigned i = 0; i < ms->smp.cpus; i++) {
        g_autofree char *name = g_strdup_printf("cpu%u", i);
        object_initialize_child(OBJECT(s), name, &s->cpu[i], ARM_CPU_TYPE_NAME("cortex-a53"));
        qdev_prop_set_uint64(DEVICE(&s->cpu[i]), "cntfrq", 8000000);
        qdev_prop_set_bit(DEVICE(&s->cpu[i]), "start-powered-off", i != 0);
        if (!qdev_realize(DEVICE(&s->cpu[i]), NULL, errp)) {
            return;
        }
    }
    qdev_prop_set_uint32(gic, "num-cpu", ms->smp.cpus);
    qdev_prop_set_bit(gic, "has-security-extensions", true);
    qdev_prop_set_uint32(gic, "num-irq", IMX8MM_NUM_IRQS + GIC_INTERNAL);
    qlist_append_int(redists, ms->smp.cpus);
    qdev_prop_set_array(gic, "redist-region-count", redists);
    object_property_set_link(OBJECT(gic), "sysmem", OBJECT(get_system_memory()), &error_fatal);
    sysbus_realize(gicsbd, &error_fatal);
    sysbus_mmio_map(gicsbd, 0, 0x38800000);
    sysbus_mmio_map(gicsbd, 1, 0x38880000);
    for (unsigned i = 0; i < ms->smp.cpus; i++) {
        DeviceState *cpu = DEVICE(&s->cpu[i]);
        unsigned base = IMX8MM_NUM_IRQS + i * GIC_INTERNAL;
        static const unsigned timer_irqs[] = {
            [GTIMER_PHYS] = ARCH_TIMER_NS_EL1_IRQ,
            [GTIMER_VIRT] = ARCH_TIMER_VIRT_IRQ,
            [GTIMER_HYP] = ARCH_TIMER_NS_EL2_IRQ,
            [GTIMER_SEC] = ARCH_TIMER_S_EL1_IRQ,
        };
        for (unsigned j = 0; j < ARRAY_SIZE(timer_irqs); j++) {
            qdev_connect_gpio_out(cpu, j, qdev_get_gpio_in(gic, base + timer_irqs[j]));
        }
        qdev_connect_gpio_out_named(cpu, "gicv3-maintenance-interrupt", 0,
            qdev_get_gpio_in(gic, base + ARCH_GIC_MAINT_IRQ));
        qdev_connect_gpio_out_named(cpu, "pmu-interrupt", 0,
            qdev_get_gpio_in(gic, base + VIRTUAL_PMU_IRQ));
        for (unsigned line = 0; line < 4; line++) {
            sysbus_connect_irq(gicsbd, i + line * ms->smp.cpus,
                qdev_get_gpio_in_named(dev, "cpu-irq", i + line * 4));
        }
        sysbus_connect_irq(gicsbd, i + 6 * ms->smp.cpus,
            qdev_get_gpio_in_named(dev, "cpu-wakeup", i));
    }
    imx8mm_map(SYS_BUS_DEVICE(&s->ccm), 0x30380000, gic, -1);
    qdev_prop_set_uint32(DEVICE(&s->analog), "digprog", 0x00824110);
    imx8mm_map(SYS_BUS_DEVICE(&s->analog), 0x30360000, gic, -1);
    imx8mm_map(SYS_BUS_DEVICE(&s->snvs), 0x30370000, gic, 4);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->snvs), 1, qdev_get_gpio_in(gic, 19));
    imx8mm_map(SYS_BUS_DEVICE(&s->lcdif), 0x32e00000, gic, 5);
    imx8mm_map(SYS_BUS_DEVICE(&s->caam), 0x30900000, gic, -1);
    for (unsigned i = 0; i < 4; i++) {
        static const unsigned caam_irqs[] = {105, 106, 114, 115};
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->caam), i,
                            qdev_get_gpio_in(gic, caam_irqs[i]));
    }
    for (unsigned i = 0; i < 4; i++) {
        if (!qemu_chr_fe_get_driver(&s->uart[i].chr)) {
            qdev_prop_set_chr(DEVICE(&s->uart[i]), "chardev", serial_hd(i));
        }
        imx8mm_map(SYS_BUS_DEVICE(&s->uart[i]), uart_bases[i], gic, uart_irqs[i]);
        imx8mm_map(SYS_BUS_DEVICE(&s->i2c[i]), 0x30a20000 + 0x10000 * i, gic, 35 + i);
    }
    for (unsigned i = 0; i < 5; i++) {
        imx8mm_map(SYS_BUS_DEVICE(&s->gpio[i]), 0x30200000 + 0x10000 * i, gic, 64 + i * 2);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio[i]), 1,
                            qdev_get_gpio_in(gic, 65 + i * 2));
    }
    for (unsigned i = 0; i < 3; i++) {
        static const hwaddr sdma_bases[] = {0x30bd0000, 0x302c0000, 0x302b0000};
        static const unsigned sdma_irqs[] = {2, 103, 34};
        imx8mm_map(SYS_BUS_DEVICE(&s->sdma[i]), sdma_bases[i], gic, sdma_irqs[i]);
        qdev_prop_set_bit(DEVICE(&s->usdhc[i]), "busy-completion-irq", true);
        imx8mm_map(SYS_BUS_DEVICE(&s->usdhc[i]), 0x30b40000 + 0x10000 * i, gic, 22 + i);
        imx8mm_map(SYS_BUS_DEVICE(&s->spi[i]), 0x30820000 + 0x10000 * i, gic, 31 + i);
        qdev_prop_set_bit(DEVICE(&s->wdt[i]), "pretimeout-support", true);
        imx8mm_map(SYS_BUS_DEVICE(&s->wdt[i]), 0x30280000 + 0x10000 * i, gic, 78 + i);
    }
    for (unsigned i = 0; i < IMX8MM_CONTROL_BANKS; i++) {
        IMX8MMControlRegion *r = &s->control[i];
        g_autofree char *name = g_strdup_printf("imx8mm.control.%u", i);
        r->soc = s; r->index = i; r->base = control_bases[i];
        memory_region_init_io(&r->memory, OBJECT(s), &imx8mm_control_ops, r,
                              name, i == CTRL_DISPMIX ? 0x100 :
                              i >= CTRL_USBMISC1 ? 0x200 : 0x10000);
        memory_region_add_subregion(get_system_memory(), r->base, &r->memory);
    }
    for (unsigned i = 0; i < 2; i++) {
        qdev_prop_set_bit(DEVICE(&s->usb[i]), "gadget", true);
        imx8mm_map(SYS_BUS_DEVICE(&s->usb[i]), 0x32e40000 + i * 0x10000, gic, 40 + i);
    }
    s->phy_region.soc = s;
    memory_region_init_io(&s->phy_region.memory, OBJECT(s), &imx8mm_phy_ops,
                          &s->phy_region, "imx8mm.ddrphy", 0x1000000);
    memory_region_add_subregion(get_system_memory(), 0x3c000000, &s->phy_region.memory);
    memory_region_init_rom(&s->rom, OBJECT(s), "imx8mm.rom", 0x20000, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0, &s->rom);
    {
        /* HAB ROM API for an unfused part. Authentication returns NULL:
         * the native bootloader decides whether its open policy permits it. */
        uint8_t rvt[0x60] = {0xdd, 0, 0x60, 0x44};
        uint8_t code[0x60] = {0};
        static const uint32_t instructions[] = {
            0x52801e00, 0xd65f03c0, /* entry/exit/check-target: HAB_SUCCESS */
            0xd2800000, 0xd65f03c0, /* authenticate: no verified entry */
            0x52800660, 0xd65f03c0, /* report-event: no events */
            0x52801e02, 0xb9000002, /* report-status: OPEN */
            0x52800cc2, 0xb9000022, /* NONSECURE */
            0x52801e00, 0xd65f03c0,
        };
        for (unsigned i = 0; i < ARRAY_SIZE(instructions); i++) {
            stl_le_p(code + i * 4, instructions[i]);
        }
        stq_le_p(rvt + 0x08, 0x1400);
        stq_le_p(rvt + 0x10, 0x1400);
        stq_le_p(rvt + 0x18, 0x1400);
        stq_le_p(rvt + 0x20, 0x1408);
        stq_le_p(rvt + 0x40, 0x1410);
        stq_le_p(rvt + 0x48, 0x1418);
        stq_le_p(rvt + 0x50, 0x1410);
        rom_add_blob_fixed("imx8mm.hab-rvt", rvt, sizeof(rvt), 0x900);
        rom_add_blob_fixed("imx8mm.hab-open-api", code, sizeof(code), 0x1400);
    }
    memory_region_init_ram(&s->ocram_s, OBJECT(s), "imx8mm.ocram-s", 0x8000, &error_fatal);
    memory_region_init_ram(&s->tcm, OBJECT(s), "imx8mm.tcm", 0x80000, &error_fatal);
    memory_region_init_ram(&s->ocram, OBJECT(s), "imx8mm.ocram", 0x40000, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x180000, &s->ocram_s);
    memory_region_add_subregion(get_system_memory(), 0x7c0000, &s->tcm);
    memory_region_add_subregion(get_system_memory(), 0x900000, &s->ocram);
    {
        DeviceState *counter = qdev_new("imx7-syscnt");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(counter), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(counter), 0, 0x306c0000);
        sysbus_mmio_map(SYS_BUS_DEVICE(counter), 1, 0x306a0000);
        sysbus_mmio_map(SYS_BUS_DEVICE(counter), 2, 0x306b0000);
        sysbus_connect_irq(SYS_BUS_DEVICE(counter), 0, qdev_get_gpio_in(gic, 47));
    }
    static const hwaddr unimplemented[] = {
        0x301f0000, 0x305f0000, 0x309f0000, 0x30270000,
        0x303e0000, 0x307f0000, 0x33000000,
    };
    for (unsigned i = 0; i < ARRAY_SIZE(unimplemented); i++) {
        g_autofree char *name = g_strdup_printf("imx8mm.unimplemented.%x", (unsigned)unimplemented[i]);
        create_unimplemented_device(name, unimplemented[i], 0x10000);
    }
}

static void fsl_imx8mm_reset(DeviceState *dev)
{
    FslIMX8MMState *s = FSL_IMX8MM(dev);
    memset(s->registers, 0, sizeof(s->registers));
    memset(s->phy, 0, sizeof(s->phy));
    memset(s->powerdown, 0, sizeof(s->powerdown));
    memset(s->sleeping, 0, sizeof(s->sleeping));
    s->cpu_irq_levels = 0;
    s->wake_levels = 0;
    s->ddr_initialized = false;
    timer_del(&s->power_timer);
    s->phy[0xd0031] = 1;
    s->registers[CTRL_SRC][0x58 / 4] = 0x2000; /* eMMC1 boot configuration. */
}
static const VMStateDescription fsl_imx8mm_vmstate = {
    .name = TYPE_FSL_IMX8MM, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(registers, FslIMX8MMState, IMX8MM_CONTROL_BANKS, 0x10000 / 4),
        VMSTATE_UINT32_ARRAY(phy, FslIMX8MMState, 0x1000000 / 4),
        VMSTATE_UINT16(cpu_irq_levels, FslIMX8MMState),
        VMSTATE_UINT8(wake_levels, FslIMX8MMState),
        VMSTATE_BOOL(ddr_initialized, FslIMX8MMState),
        VMSTATE_BOOL_ARRAY(powerdown, FslIMX8MMState, 4),
        VMSTATE_BOOL_ARRAY(sleeping, FslIMX8MMState, 4),
        VMSTATE_TIMER(power_timer, FslIMX8MMState),
        VMSTATE_END_OF_LIST()
    },
};
static void fsl_imx8mm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = fsl_imx8mm_realize;
    dc->vmsd = &fsl_imx8mm_vmstate;
    dc->user_creatable = false;
    device_class_set_legacy_reset(dc, fsl_imx8mm_reset);
}
static const TypeInfo fsl_imx8mm_info = {
    .name = TYPE_FSL_IMX8MM, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FslIMX8MMState),
    .instance_init = fsl_imx8mm_init, .class_init = fsl_imx8mm_class_init,
};
static void fsl_imx8mm_register(void) { type_register_static(&fsl_imx8mm_info); }
type_init(fsl_imx8mm_register)
