/*
 * Allwinner B300 / sun8iw15p1 peripherals used by PocketBook readers.
 * Register maps follow PocketBook's kernel-b300 and uboot-b300 sources.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "hw/arm/allwinner-b300.h"
#include "hw/arm/allwinner-b300-iommu.h"
#include "hw/char/serial-mm.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/allwinner-b300-eink.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/allwinner-b300-power.h"
#include "hw/timer/allwinner-a10-pit.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/watchdog/allwinner-wdt.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/rtc.h"
#include "system/system.h"
#include "target/arm/gtimer.h"
#include "target/arm/arm-powerctl.h"

#define TYPE_B300_IO "allwinner-b300-io"
#define TYPE_B300_EHCI "allwinner-b300-ehci"
#define B300_SYSTEM_SIZE 0x10000
#define B300_R_SIZE 0x24000
#define B300_PERIPHERAL_SIZE 0x80000
#define B300_CPUCFG_SIZE 0x1000
#define B300_USB_PHY_SIZE 0x1000
#define B300_USB_HOST_PHY_SIZE 0x28
#define B300_CODEC_SIZE 0x800

OBJECT_DECLARE_SIMPLE_TYPE(B300IOState, B300_IO)

typedef struct B300RegisterBank {
    B300IOState *state;
    hwaddr base;
    uint32_t *regs;
    MemoryRegion iomem;
} B300RegisterBank;

struct B300IOState {
    SysBusDevice parent_obj;
    B300RegisterBank bank[8];
    uint32_t system_regs[B300_SYSTEM_SIZE / 4];
    uint32_t r_regs[B300_R_SIZE / 4];
    uint32_t peripheral_regs[B300_PERIPHERAL_SIZE / 4];
    uint32_t cpu_regs[B300_CPUCFG_SIZE / 4];
    uint32_t usb_phy_regs[B300_USB_PHY_SIZE / 4];
    uint32_t usb_host_phy_regs[2][B300_USB_HOST_PHY_SIZE / 4];
    uint32_t codec_regs[B300_CODEC_SIZE / 4];
    uint32_t gpio_input[12];
    int64_t rtc_offset;
    uint32_t rtc_access;
    QEMUTimer *rtc_timer, *ths_timer;
    qemu_irq ths_irq;
    qemu_irq gpio_irq[5];
    qemu_irq gpio_output[AW_B300_NUM_GPIO];
    AWB300PowerState *power;
};

static void b300_rtc_sync_done(void *opaque)
{
    B300_IO(opaque)->rtc_access = 0;
}

static void b300_ths_irq(B300IOState *s)
{
    uint32_t *ths = &s->peripheral_regs[0x70400 / 4];

    qemu_set_irq(s->ths_irq, !!((ths[0x10 / 4] & ths[0x20 / 4]) |
                               (ths[0x14 / 4] & ths[0x24 / 4]) |
                               (ths[0x18 / 4] & ths[0x28 / 4])));
}

static void b300_ths_schedule(B300IOState *s)
{
    uint32_t *ths = &s->peripheral_regs[0x70400 / 4];

    if (ths[0x04 / 4] & 3) {
        uint64_t period = MAX(ths[0x08 / 4], 1);
        timer_mod(s->ths_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  muldiv64(period, NANOSECONDS_PER_SECOND, 24000000));
    } else {
        timer_del(s->ths_timer);
    }
}

static void b300_ths_sample(void *opaque)
{
    B300IOState *s = opaque;
    uint32_t *ths = &s->peripheral_regs[0x70400 / 4];

    /* Native SUN8IW15 conversion: (2794000 - raw * 1000) / 14882 = 25 C. */
    for (unsigned i = 0; i < 2; i++) {
        if (ths[0x04 / 4] & BIT(i)) {
            ths[(0xc0 + i * 4) / 4] = 2421;
            ths[0x20 / 4] |= BIT(i);
        }
    }
    b300_ths_irq(s);
    b300_ths_schedule(s);
}

static uint32_t *b300_gpio_regs(B300IOState *s, unsigned bank)
{
    return bank == 11 ? &s->r_regs[0x22000 / 4] :
                        &s->system_regs[(0xb000 + bank * 0x24) / 4];
}

static uint32_t *b300_gpio_irq_regs(B300IOState *s, unsigned bank)
{
    return bank == 11 ? &s->r_regs[0x22200 / 4] :
                        &s->system_regs[(0xb200 + bank * 0x20) / 4];
}

static void b300_gpio_update(B300IOState *s)
{
    static const unsigned banks[] = { 1, 5, 6, 7, 11 };

    for (unsigned i = 0; i < ARRAY_SIZE(banks); i++) {
        unsigned bank = banks[i];
        uint32_t *regs = b300_gpio_irq_regs(s, bank);
        uint32_t *gpio = b300_gpio_regs(s, bank);

        for (unsigned pin = 0; pin < 32; pin++) {
            unsigned mux = extract32(gpio[pin / 8], pin % 8 * 4, 4);
            unsigned mode = extract32(regs[pin / 8], pin % 8 * 4, 4);
            bool high = !!(s->gpio_input[bank] & BIT(pin));

            if (mux == 6 && ((mode == 2 && high) || (mode == 3 && !high))) {
                regs[0x14 / 4] |= BIT(pin);
            }
        }
        qemu_set_irq(s->gpio_irq[i], !!(regs[0x10 / 4] & regs[0x14 / 4]));
    }
}

static void b300_gpio_outputs(B300IOState *s)
{
    for (unsigned bank = 0; bank < 12; bank++) {
        uint32_t *regs = b300_gpio_regs(s, bank);
        for (unsigned pin = 0; pin < 32; pin++) {
            bool output = extract32(regs[pin / 8], pin % 8 * 4, 4) == 1;
            uint32_t data = output ? regs[0x10 / 4] : s->gpio_input[bank];

            qemu_set_irq(s->gpio_output[bank * 32 + pin], !!(data & BIT(pin)));
        }
    }
}

static void b300_gpio_input(void *opaque, int number, int level)
{
    B300IOState *s = opaque;
    unsigned bank = number / 32, pin = number % 32;
    bool old = !!(s->gpio_input[bank] & BIT(pin));

    s->gpio_input[bank] = deposit32(s->gpio_input[bank], pin, 1, level);
    if (bank == 1 || bank == 5 || bank == 6 || bank == 7 || bank == 11) {
        uint32_t *regs = b300_gpio_irq_regs(s, bank);
        uint32_t *gpio = b300_gpio_regs(s, bank);
        unsigned mode = extract32(regs[pin / 8], pin % 8 * 4, 4);
        unsigned mux = extract32(gpio[pin / 8], pin % 8 * 4, 4);
        bool pending = (mode == 0 && !old && level) ||
                       (mode == 1 && old && !level) ||
                       (mode == 4 && old != level);

        if (mux == 6 && pending) {
            regs[0x14 / 4] |= BIT(pin);
        }
    }
    b300_gpio_update(s);
}

static uint64_t b300_io_read_word(void *opaque, hwaddr offset, unsigned size)
{
    B300RegisterBank *bank = opaque;
    B300IOState *s = bank->state;
    hwaddr addr = bank->base + offset;
    uint32_t value = bank->regs[offset / 4];

    if (addr == 0x09010080) {
        for (unsigned i = 0; i < AW_B300_NUM_CPUS; i++) {
            CPUState *cpu = arm_get_cpu_by_id(i);

            value = deposit32(value, 16 + i, 1, cpu && cpu->halted);
        }
        return value;
    }

    if (addr == 0x07000010 || addr == 0x07000014) {
        struct tm now;

        qemu_get_timedate(&now, s->rtc_offset);
        if (addr == 0x07000010) {
            unsigned year = now.tm_year + 1900;
            bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);

            return ((year - 1970) << 16) | (leap << 23) |
                   ((now.tm_mon + 1) << 8) | now.tm_mday;
        }
        return (now.tm_hour << 16) | (now.tm_min << 8) | now.tm_sec;
    }
    /* PLL factors are readable, with hardware's lock and update completion. */
    if (addr >= 0x03001000 && addr <= 0x03001078 && !(addr & 7)) {
        value &= ~BIT(30);
        if (value & BIT(31)) {
            value |= BIT(28);
        }
        return value;
    }
    if ((addr >= 0x0300b000 && addr < 0x0300b1b0 &&
         (addr - 0x0300b000) % 0x24 == 0x10) || addr == 0x07022010) {
        unsigned port = addr == 0x07022010 ? 11 :
                        (addr - 0x0300b000) / 0x24;
        uint32_t *gpio = b300_gpio_regs(s, port);
        uint32_t outputs = 0;

        for (unsigned pin = 0; pin < 32; pin++) {
            if (extract32(gpio[pin / 8], pin % 8 * 4, 4) == 1) {
                outputs |= BIT(pin);
            }
        }
        return (value & outputs) | (s->gpio_input[port] & ~outputs);
    }
    if (addr >= 0x05000000 && addr <= 0x0500107c &&
        (addr & 0x3ff) == 0x7c) {
        return 6; /* DesignWare UART USR: TX not full, transmitter idle. */
    }
    /* The RTC clock domain asserts access status while synchronizing a write. */
    if (addr == 0x07000000) {
        return (value & ~(BIT(7) | BIT(8))) | s->rtc_access;
    }
    return value;
}

static void b300_io_write_word(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    B300RegisterBank *bank = opaque;
    B300IOState *s = bank->state;
    hwaddr addr = bank->base + offset;
    uint32_t old = bank->regs[offset / 4];

    if ((addr >= 0x0300b220 && addr <= 0x0300b2f4 &&
         (addr - 0x0300b200) % 0x20 == 0x14) || addr == 0x07022214) {
        bank->regs[offset / 4] &= ~value;
        b300_gpio_update(s);
        return;
    }
    if (addr >= 0x05070420 && addr <= 0x0507042c) {
        bank->regs[offset / 4] &= ~value;
        b300_ths_irq(s);
        return;
    }
    if (addr == 0x05101824 || addr == 0x05200824 || addr == 0x05100424 ||
        addr == 0x050704c0 || addr == 0x050704c4) {
        return; /* PHY status and thermal samples are read-only. */
    }
    bank->regs[offset / 4] = value;
    if (addr == 0x05070404 || addr == 0x05070408) {
        b300_ths_schedule(s);
    }
    if (addr >= 0x05070410 && addr <= 0x05070418) {
        b300_ths_irq(s);
    }
    if (addr >= 0x05000000 && addr <= 0x050010a4 &&
        (addr & 0x3ff) == 0xa4) {
        /* UART HALT.LCRUP clears when the new divisor/LCR is accepted. */
        bank->regs[offset / 4] &= ~BIT(2);
    }
    if (addr == 0x07000400 && !(old & BIT(0)) && (value & BIT(0))) {
        aw_b300_power_scp_start(s->power);
    }
    if (addr == 0x09010000) {
        /* ARM32 Boot ROM uses each core's CPU_SOFT_ENTRY_REG(cpu). */
        for (unsigned i = 1; i < AW_B300_NUM_CPUS; i++) {
            uint32_t entry = s->r_regs[(0x5c4 + i * 4) / 4];

            if (entry && !(old & BIT(i)) && (value & BIT(i))) {
                arm_set_cpu_on(i, entry, 0, 3, false);
            }
        }
    }
    if (addr >= 0x07000454 && addr <= 0x0700045c && value == 0xff) {
        arm_set_cpu_off((addr - 0x07000450) / 4);
    }
    if ((addr >= 0x0300b000 && addr < 0x0300b400) ||
        (addr >= 0x07022000 && addr < 0x07022400)) {
        b300_gpio_outputs(s);
        b300_gpio_update(s);
    }
    if (addr == 0x07000010 || addr == 0x07000014) {
        struct tm now;

        qemu_get_timedate(&now, s->rtc_offset);
        if (addr == 0x07000010) {
            now.tm_year = extract32(value, 16, 7) + 1970 - 1900;
            now.tm_mon = extract32(value, 8, 4) - 1;
            now.tm_mday = extract32(value, 0, 5);
        } else {
            now.tm_hour = extract32(value, 16, 5);
            now.tm_min = extract32(value, 8, 6);
            now.tm_sec = extract32(value, 0, 6);
        }
        if (now.tm_mon >= 0 && now.tm_mon < 12 && now.tm_mday >= 1 &&
            now.tm_mday <= 31 && now.tm_hour < 24 && now.tm_min < 60 &&
            now.tm_sec < 60) {
            s->rtc_offset = qemu_timedate_diff(&now);
            s->rtc_access |= BIT(addr == 0x07000010 ? 7 : 8);
            timer_mod(s->rtc_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
        }
    }
}

static uint64_t b300_io_read(void *opaque, hwaddr offset, unsigned size)
{
    return extract32(b300_io_read_word(opaque, offset & ~3, 4),
                      (offset & 3) * 8, size * 8);
}

static void b300_io_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    B300RegisterBank *bank = opaque;
    hwaddr addr = bank->base + (offset & ~3);
    bool w1c = (addr >= 0x0300b220 && addr <= 0x0300b2f4 &&
                (addr - 0x0300b200) % 0x20 == 0x14) || addr == 0x07022214 ||
                (addr >= 0x05070420 && addr <= 0x0507042c);
    uint32_t word = w1c ? value << ((offset & 3) * 8) :
        deposit32(bank->regs[offset / 4], (offset & 3) * 8, size * 8, value);

    b300_io_write_word(opaque, offset & ~3, word, 4);
}

static const MemoryRegionOps b300_io_ops = {
    .read = b300_io_read,
    .write = b300_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void b300_io_reset(DeviceState *dev)
{
    B300IOState *s = B300_IO(dev);

    memset(s->system_regs, 0, sizeof(s->system_regs));
    memset(s->r_regs, 0, sizeof(s->r_regs));
    memset(s->peripheral_regs, 0, sizeof(s->peripheral_regs));
    memset(s->cpu_regs, 0, sizeof(s->cpu_regs));
    memset(s->usb_phy_regs, 0, sizeof(s->usb_phy_regs));
    memset(s->usb_host_phy_regs, 0, sizeof(s->usb_host_phy_regs));
    memset(s->codec_regs, 0, sizeof(s->codec_regs));
    s->rtc_access = 0;
    timer_del(s->rtc_timer);
    timer_del(s->ths_timer);
    b300_ths_irq(s);
    memset(s->gpio_input, 0xff, sizeof(s->gpio_input));
    /* Boot0 leaves the CPU and peripheral PLLs running before U-Boot entry. */
    s->system_regs[0x1000 / 4] = 0x90002900;
    s->system_regs[0x1020 / 4] = 0x90006300;
    s->system_regs[0x1028 / 4] = 0x90006300;
    s->system_regs[0x1500 / 4] = 0x03000301;
    s->system_regs[0x1510 / 4] = 0x03000002;
    s->system_regs[0x1520 / 4] = 0x03000002;
    s->system_regs[0x1524 / 4] = 0x03000002;
    s->r_regs[0x10000 / 4] = 0x01000000;
    s->r_regs[0x440 / 4] = 1;
    s->r_regs[0x444 / 4] = 0x0e;
    s->r_regs[0x454 / 4] = s->r_regs[0x458 / 4] =
        s->r_regs[0x45c / 4] = 0xff;
    s->cpu_regs[0] = 1;
    s->system_regs[0x6200 / 4] = 0x03000001;
    s->system_regs[0x6204 / 4] = 0x00003000;
    s->system_regs[0x6208 / 4] = 0x20240001;
    s->system_regs[0x620c / 4] = 0x00000001;
    b300_gpio_update(s);
    b300_gpio_outputs(s);
    b300_ths_irq(s);
}

static int b300_io_post_load(void *opaque, int version_id)
{
    B300IOState *s = opaque;

    b300_gpio_update(s);
    b300_gpio_outputs(s);
    return 0;
}

static const VMStateDescription b300_io_vmstate = {
    .name = TYPE_B300_IO,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = b300_io_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(system_regs, B300IOState, B300_SYSTEM_SIZE / 4),
        VMSTATE_UINT32_ARRAY(r_regs, B300IOState, B300_R_SIZE / 4),
        VMSTATE_UINT32_ARRAY(peripheral_regs, B300IOState,
                             B300_PERIPHERAL_SIZE / 4),
        VMSTATE_UINT32_ARRAY(cpu_regs, B300IOState, B300_CPUCFG_SIZE / 4),
        VMSTATE_UINT32_ARRAY(usb_phy_regs, B300IOState, B300_USB_PHY_SIZE / 4),
        VMSTATE_UINT32_2DARRAY(usb_host_phy_regs, B300IOState, 2,
                               B300_USB_HOST_PHY_SIZE / 4),
        VMSTATE_UINT32_ARRAY(codec_regs, B300IOState, B300_CODEC_SIZE / 4),
        VMSTATE_UINT32_ARRAY(gpio_input, B300IOState, 12),
        VMSTATE_INT64(rtc_offset, B300IOState),
        VMSTATE_UINT32(rtc_access, B300IOState),
        VMSTATE_TIMER_PTR(rtc_timer, B300IOState),
        VMSTATE_TIMER_PTR(ths_timer, B300IOState),
        VMSTATE_END_OF_LIST()
    },
};

static void b300_io_init(Object *obj)
{
    B300IOState *s = B300_IO(obj);
    static const hwaddr base[] = {
        0x03000000, 0x07000000, 0x05000000, 0x09010000, 0x05100000,
        0x05101800, 0x05200800,
        0x05096000,
    };
    static const unsigned size[] = {
        B300_SYSTEM_SIZE, B300_R_SIZE, B300_PERIPHERAL_SIZE, B300_CPUCFG_SIZE,
        B300_USB_PHY_SIZE,
        B300_USB_HOST_PHY_SIZE, B300_USB_HOST_PHY_SIZE,
        B300_CODEC_SIZE,
    };
    uint32_t *regs[] = {
        s->system_regs, s->r_regs, s->peripheral_regs, s->cpu_regs,
        s->usb_phy_regs,
        s->usb_host_phy_regs[0], s->usb_host_phy_regs[1],
        s->codec_regs,
    };

    s->rtc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, b300_rtc_sync_done, s);
    s->ths_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, b300_ths_sample, s);

    for (unsigned i = 0; i < ARRAY_SIZE(s->bank); i++) {
        s->bank[i].state = s;
        s->bank[i].base = base[i];
        s->bank[i].regs = regs[i];
        memory_region_init_io(&s->bank[i].iomem, obj, &b300_io_ops,
                               &s->bank[i], "b300-registers", size[i]);
        sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->bank[i].iomem);
    }
    qdev_init_gpio_in_named(DEVICE(s), b300_gpio_input, "gpio-in",
                            AW_B300_NUM_GPIO);
    qdev_init_gpio_out_named(DEVICE(s), s->gpio_output, "gpio-out",
                             AW_B300_NUM_GPIO);
    for (unsigned i = 0; i < ARRAY_SIZE(s->gpio_irq); i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->gpio_irq[i]);
    }
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->ths_irq);
}

static void b300_io_finalize(Object *obj)
{
    timer_free(B300_IO(obj)->rtc_timer);
    timer_free(B300_IO(obj)->ths_timer);
}

static void b300_io_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    static const Property props[] = {
        DEFINE_PROP_LINK("power", B300IOState, power,
                          TYPE_AW_B300_POWER, AWB300PowerState *),
    };

    dc->vmsd = &b300_io_vmstate;
    device_class_set_props(dc, props);
    device_class_set_legacy_reset(dc, b300_io_reset);
}

static void allwinner_b300_init(Object *obj)
{
    AwB300State *s = AW_B300(obj);

    for (unsigned i = 0; i < AW_B300_NUM_CPUS; i++) {
        object_initialize_child(obj, "cpu[*]", &s->cpus[i],
                                 ARM_CPU_TYPE_NAME("cortex-a7"));
    }
    object_initialize_child(obj, "gic", &s->gic, TYPE_ARM_GIC);
    s->display = qdev_new(TYPE_AW_B300_EINK);
    object_property_add_child(obj, "display", OBJECT(s->display));
    object_unref(OBJECT(s->display));
    s->power = qdev_new(TYPE_AW_B300_POWER);
    object_property_add_child(obj, "power", OBJECT(s->power));
    object_unref(OBJECT(s->power));
    s->iommu = qdev_new(TYPE_AW_B300_IOMMU);
    object_property_add_child(obj, "iommu", OBJECT(s->iommu));
    object_unref(OBJECT(s->iommu));
    for (unsigned i = 0; i < AW_B300_NUM_MMC; i++) {
        object_initialize_child(obj, "mmc[*]", &s->mmc[i],
                                 TYPE_AW_SDHOST_SUN50I_A64);
    }
    for (unsigned i = 0; i < AW_B300_NUM_I2C; i++) {
        object_initialize_child(obj, "i2c[*]", &s->i2c[i], TYPE_AW_I2C_SUN6I);
    }
}

static void allwinner_b300_realize(DeviceState *device, Error **errp)
{
    AwB300State *s = AW_B300(device);
    DeviceState *dev;
    SysBusDevice *busdev;

    /* Both native secure/nonsecure BROM nodes begin at physical address zero.
     * Boot0's package handoff starts after ROM execution. */
    memory_region_init_rom(&s->brom, OBJECT(s), "b300.brom", 64 * KiB,
                            &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0, &s->brom);
    memory_region_init_ram(&s->sram_a1, OBJECT(s), "b300.sram-a1", 96 * KiB,
                            &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x00020000, &s->sram_a1);
    memory_region_init_ram(&s->sram_c, OBJECT(s), "b300.sram-c", 128 * KiB,
                            &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x00038000, &s->sram_c);
    memory_region_init_ram(&s->sram_a2, OBJECT(s), "b300.sram-a2", 112 * KiB,
                            &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x00100000, &s->sram_a2);

    for (unsigned i = 0; i < AW_B300_NUM_CPUS; i++) {
        dev = DEVICE(&s->cpus[i]);
        qdev_prop_set_uint32(dev, "cntfrq", 24000000);
        qdev_prop_set_uint32(dev, "reset-cbar", 0x03020000);
        qdev_prop_set_bit(dev, "has_el3", true);
        qdev_prop_set_bit(dev, "has_el2", true);
        qdev_prop_set_bit(dev, "start-powered-off", i != 0);
        qdev_realize(dev, NULL, &error_fatal);
    }
    dev = DEVICE(&s->gic);
    qdev_prop_set_uint32(dev, "num-irq", AW_B300_NUM_SPI + GIC_INTERNAL);
    qdev_prop_set_uint32(dev, "num-cpu", AW_B300_NUM_CPUS);
    qdev_prop_set_uint32(dev, "revision", 2);
    qdev_prop_set_bit(dev, "has-security-extensions", true);
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x03021000);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, 0x03022000);
    for (unsigned i = 0; i < AW_B300_NUM_CPUS; i++) {
        static const unsigned timer_ppi[] = {
            [GTIMER_PHYS] = 14, [GTIMER_VIRT] = 11,
            [GTIMER_HYP] = 10, [GTIMER_SEC] = 13,
        };

        for (unsigned j = 0; j < ARRAY_SIZE(timer_ppi); j++) {
            qdev_connect_gpio_out(DEVICE(&s->cpus[i]), j,
                qdev_get_gpio_in(dev, AW_B300_NUM_SPI + i * GIC_INTERNAL +
                                 GIC_NR_SGIS + timer_ppi[j]));
        }
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i,
                            qdev_get_gpio_in(DEVICE(&s->cpus[i]), ARM_CPU_IRQ));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i + AW_B300_NUM_CPUS,
                            qdev_get_gpio_in(DEVICE(&s->cpus[i]), ARM_CPU_FIQ));
    }

    busdev = SYS_BUS_DEVICE(s->power);
    sysbus_realize(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0x07083000);
    sysbus_mmio_map_overlap(busdev, 1, 0x03003000, 1);
    sysbus_mmio_map_overlap(busdev, 2, 0x03004000, 1);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->gic), 140));
    sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(DEVICE(&s->gic), 11));
    sysbus_connect_irq(busdev, 2, qdev_get_gpio_in(DEVICE(&s->gic), 84));

    dev = qdev_new(TYPE_B300_IO);
    object_property_add_child(OBJECT(s), "registers", OBJECT(dev));
    object_property_set_link(OBJECT(dev), "power", OBJECT(s->power),
                              &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, 0x03000000, -1);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 1, 0x07000000, -1);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 2, 0x05000000, -1);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 3, 0x09010000);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 4, 0x05100000);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 5, 0x05101800, 2);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 6, 0x05200800, 2);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 7, 0x05096000);
    qdev_pass_gpios(dev, device, "gpio-in");
    qdev_pass_gpios(dev, device, "gpio-out");
    for (unsigned i = 0; i < 5; i++) {
        static const unsigned irq[] = { 58, 59, 60, 61, 93 };

        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i,
                            qdev_get_gpio_in(DEVICE(&s->gic), irq[i]));
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 5,
                        qdev_get_gpio_in(DEVICE(&s->gic), 1));

    busdev = SYS_BUS_DEVICE(s->iommu);
    sysbus_realize(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0x030f0000);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->gic), 18));

    object_property_set_link(OBJECT(s->display), "iommu", OBJECT(s->iommu),
                              &error_fatal);
    busdev = SYS_BUS_DEVICE(s->display);
    sysbus_realize(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0x01000000);
    sysbus_mmio_map(busdev, 1, 0x06511000);
    sysbus_mmio_map(busdev, 2, 0x01400000);
    sysbus_mmio_map(busdev, 3, 0x06504000);
    sysbus_mmio_map(busdev, 4, 0x06510000);
    for (unsigned i = 0; i < 4; i++) {
        static const unsigned irq[] = { 29, 52, 20, 37 };

        sysbus_connect_irq(busdev, i,
                            qdev_get_gpio_in(DEVICE(&s->gic), irq[i]));
    }

    dev = qdev_new(TYPE_AW_A10_PIT);
    qdev_prop_set_uint32(dev, "clk0-freq", 32768);
    qdev_prop_set_uint32(dev, "clk1-freq", 24000000);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map_overlap(busdev, 0, 0x03009000, 1);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->gic), 50));
    dev = qdev_new(TYPE_AW_WDT_SUN6I);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, 0x030090a0, 2);

    for (unsigned i = 0; i < 2; i++) {
        hwaddr base = i ? 0x05200000 : 0x05101000;
        DeviceState *ehci = qdev_new(TYPE_B300_EHCI);

        qdev_prop_set_bit(ehci, "companion-enable", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ehci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ehci), 0, base);
        sysbus_connect_irq(SYS_BUS_DEVICE(ehci), 0,
                            qdev_get_gpio_in(DEVICE(&s->gic), 54 + i * 2));

        dev = qdev_new(TYPE_SYSBUS_OHCI);
        qdev_prop_set_uint32(dev, "num-ports", 1);
        qdev_prop_set_string(dev, "masterbus",
                             SYS_BUS_EHCI(ehci)->ehci.bus.qbus.name);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, base + 0x400, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                            qdev_get_gpio_in(DEVICE(&s->gic), 55 + i * 2));
    }

    for (unsigned i = 0; i < AW_B300_NUM_MMC; i++) {
        dev = DEVICE(&s->mmc[i]);
        object_property_set_link(OBJECT(dev), "dma-memory",
                                  OBJECT(get_system_memory()), &error_fatal);
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_realize(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, 0x04020000 + i * 0x1000);
        sysbus_connect_irq(busdev, 0,
                            qdev_get_gpio_in(DEVICE(&s->gic), 38 + i));
    }
    for (unsigned i = 0; i < AW_B300_NUM_I2C; i++) {
        busdev = SYS_BUS_DEVICE(&s->i2c[i]);
        sysbus_realize(busdev, &error_fatal);
        sysbus_mmio_map_overlap(busdev, 0, 0x05002000 + i * 0x400, 1);
        sysbus_connect_irq(busdev, 0,
                            qdev_get_gpio_in(DEVICE(&s->gic), 34 + i));
    }
    serial_mm_init(get_system_memory(), 0x05000000, 2,
                    qdev_get_gpio_in(DEVICE(&s->gic), 41), 115200,
                    serial_hd(0), DEVICE_LITTLE_ENDIAN);
}

static void allwinner_b300_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = allwinner_b300_realize;
    dc->user_creatable = false; /* Owns CPUs and uses serial_hd(). */
}

static void b300_ehci_class_init(ObjectClass *oc, const void *data)
{
    SYS_BUS_EHCI_CLASS(oc)->portnr = 1;
}

static const TypeInfo b300_types[] = {
    {
        .name = TYPE_B300_EHCI,
        .parent = TYPE_AW_H3_EHCI,
        .class_init = b300_ehci_class_init,
    }, {
        .name = TYPE_B300_IO,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(B300IOState),
        .instance_init = b300_io_init,
        .instance_finalize = b300_io_finalize,
        .class_init = b300_io_class_init,
    }, {
        .name = TYPE_AW_B300,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AwB300State),
        .instance_init = allwinner_b300_init,
        .class_init = allwinner_b300_class_init,
    },
};
DEFINE_TYPES(b300_types)
