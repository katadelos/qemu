/*
 * Allwinner B288 / sun8iw10p1 SoC.
 * Register definitions follow PocketBook's kernel-b288 and uboot_b288 trees.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/arm/allwinner-b288.h"
#include "qemu/timer.h"
#include "hw/char/serial-mm.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/display/allwinner-b288-eink.h"
#include "hw/i2c/allwinner-i2c.h"
#include "hw/i2c/i2c.h"
#include "hw/intc/arm_gic.h"
#include "hw/sd/allwinner-sdhost.h"
#include "hw/timer/allwinner-a10-pit.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/watchdog/allwinner-wdt.h"
#include "hw/misc/unimp.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/rtc.h"
#include "system/system.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/cpu.h"
#include "target/arm/gtimer.h"

#define B288_IO_SIZE 0x40000

#define TYPE_B288_IO "allwinner-b288-io"
#define TYPE_B288_EHCI "allwinner-b288-ehci"
OBJECT_DECLARE_SIMPLE_TYPE(B288IOState, B288_IO)

struct B288IOState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion avs;
    uint32_t regs[B288_IO_SIZE / 4];
    int64_t avs_epoch[2];
    uint32_t avs_count[2];
    int64_t rtc_offset;
    uint32_t gpio_input[9];
    uint32_t lradc_mv;
    QEMUTimer *lradc_timer;
    qemu_irq lradc_irq;
    qemu_irq gpio_irq[4];
    qemu_irq gpio_output[9 * 32];
};

static void b288_lradc_update(B288IOState *s)
{
    qemu_set_irq(s->lradc_irq,
                 !!(s->regs[0x21804 / 4] & s->regs[0x21808 / 4]));
}

static void b288_lradc_sample(void *opaque)
{
    B288IOState *s = opaque;
    uint32_t control = s->regs[0x21800 / 4];

    if ((control & 1) && s->lradc_mv < 2000) {
        s->regs[0x2180c / 4] = MIN(s->lradc_mv / 31, 63);
        s->regs[0x21808 / 4] |= 1;
        b288_lradc_update(s);
        timer_mod(s->lradc_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  (4000000 << extract32(control, 2, 2)));
    }
}

static void b288_lradc_input(void *opaque, int channel, int millivolts)
{
    B288IOState *s = opaque;
    bool pressed = millivolts < 2000;

    if (pressed != (s->lradc_mv < 2000)) {
        s->regs[0x21808 / 4] |= pressed ? BIT(1) : BIT(4);
    }
    s->lradc_mv = CLAMP(millivolts, 0, 2000);
    timer_del(s->lradc_timer);
    b288_lradc_sample(s);
    b288_lradc_update(s);
}

static void b288_gpio_update(B288IOState *s)
{
    static const unsigned banks[] = { 1, 4, 5, 6 };

    for (unsigned i = 0; i < 4; i++) {
        unsigned base = 0x20a00 + i * 0x20;
        for (unsigned pin = 0; pin < 32; pin++) {
            unsigned mode = extract32(s->regs[(base + pin / 8 * 4) / 4],
                                       pin % 8 * 4, 4);
            bool level = !!(s->gpio_input[banks[i]] & BIT(pin));
            if ((mode == 2 && level) || (mode == 3 && !level)) {
                s->regs[(base + 0x14) / 4] |= BIT(pin);
            }
        }
        qemu_set_irq(s->gpio_irq[i],
                     !!(s->regs[(base + 0x10) / 4] &
                        s->regs[(base + 0x14) / 4]));
    }
}

static void b288_gpio_outputs(B288IOState *s)
{
    for (unsigned bank = 0; bank < 9; bank++) {
        unsigned base = 0x20800 + bank * 0x24;
        for (unsigned pin = 0; pin < 32; pin++) {
            unsigned cfg = s->regs[(base + pin / 8 * 4) / 4];
            uint32_t data = extract32(cfg, pin % 8 * 4, 3) == 1 ?
                            s->regs[(base + 0x10) / 4] : s->gpio_input[bank];
            qemu_set_irq(s->gpio_output[bank * 32 + pin], !!(data & BIT(pin)));
        }
    }
}

static void b288_gpio_input(void *opaque, int number, int level)
{
    static const int irq_bank[] = { -1, 0, -1, -1, 1, 2, 3, -1, -1 };
    B288IOState *s = opaque;
    unsigned bank = number / 32, pin = number % 32;
    bool old = !!(s->gpio_input[bank] & BIT(pin));
    s->gpio_input[bank] = deposit32(s->gpio_input[bank], pin, 1, level);
    if (irq_bank[bank] >= 0) {
        unsigned base = 0x20a00 + irq_bank[bank] * 0x20;
        unsigned mode = extract32(s->regs[(base + (pin / 8) * 4) / 4],
                                   (pin % 8) * 4, 4);
        bool pending = (mode == 0 && !old && level) ||
                       (mode == 1 && old && !level) ||
                       (mode == 2 && level) || (mode == 3 && !level) ||
                       (mode == 4 && old != level);
        if (pending) {
            s->regs[(base + 0x14) / 4] |= BIT(pin);
        }
    }
    b288_gpio_update(s);
}

static uint64_t b288_avs_read(void *opaque, hwaddr offset, unsigned size)
{
    B288IOState *s = opaque;
    if (offset == 4 || offset == 8) {
        unsigned i = offset / 4 - 1;
        uint32_t control = s->regs[0x20c80 / 4];
        uint32_t divisor = extract32(s->regs[0x20c8c / 4], i * 16, 16) + 1;
        uint64_t ticks = 0;
        if ((control & BIT(i)) && !(control & BIT(i + 8))) {
            ticks = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                             s->avs_epoch[i], 24000000, NANOSECONDS_PER_SECOND);
        }
        return s->avs_count[i] + ticks / divisor;
    }
    return s->regs[(0x20c80 + offset) / 4];
}

static void b288_avs_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    B288IOState *s = opaque;
    if (offset == 4 || offset == 8) {
        unsigned i = offset / 4 - 1;
        s->avs_count[i] = value;
        s->avs_epoch[i] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
    s->regs[(0x20c80 + offset) / 4] = value;
}

static const MemoryRegionOps b288_avs_ops = {
    .read = b288_avs_read,
    .write = b288_avs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t b288_io_read(void *opaque, hwaddr offset, unsigned size)
{
    B288IOState *s = opaque;
    uint32_t value = s->regs[offset / 4];

    if (offset == 0x20400) {
        return value;
    }
    if (offset == 0x20410 || offset == 0x20414) {
        struct tm now;
        qemu_get_timedate(&now, s->rtc_offset);
        if (offset == 0x20410) {
            unsigned year = now.tm_year + 1900;
            bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
            return ((year - 2010) << 15) | (leap << 22) |
                   ((now.tm_mon + 1) << 8) | now.tm_mday;
        }
        return (now.tm_hour << 16) | (now.tm_min << 8) | now.tm_sec;
    }

    if (offset >= 0x20800 && offset < 0x20944 &&
        (offset - 0x20800) % 0x24 == 0x10) {
        unsigned bank = (offset - 0x20800) / 0x24;
        uint32_t outputs = 0;
        for (unsigned pin = 0; pin < 32; pin++) {
            unsigned cfg = s->regs[(0x20800 + bank * 0x24 + pin / 8 * 4) / 4];
            if (extract32(cfg, pin % 8 * 4, 3) == 1) {
                outputs |= BIT(pin);
            }
        }
        return (value & outputs) | (s->gpio_input[bank] & ~outputs);
    }

    /* DesignWare UART USR: TX FIFO not full, transmitter empty. */
    if (offset == 0x2807c) {
        return 6;
    }

    /* PLLs report lock once enabled. DDR update strobes clear in hardware. */
    if (offset >= 0x20000 && offset <= 0x20048 && !(offset & 7)) {
        value &= ~BIT(20);
        if (value & BIT(31)) {
            value |= BIT(28);
        }
    }
    /* B288 core count and revision, selected through SRAM controller. */
    if (offset == 0x24) {
        return value & ~0xffff;
    }
    return value;
}

static void b288_io_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    B288IOState *s = opaque;
    if (offset == 0x20410 || offset == 0x20414) {
        struct tm now;
        s->regs[0x20400 / 4] |= offset == 0x20410 ? BIT(7) : BIT(8);
        qemu_get_timedate(&now, s->rtc_offset);
        if (offset == 0x20410) {
            now.tm_year = extract32(value, 15, 7) + 110;
            now.tm_mon = extract32(value, 8, 4) - 1;
            now.tm_mday = value & 31;
        } else {
            now.tm_hour = extract32(value, 16, 5);
            now.tm_min = extract32(value, 8, 6);
            now.tm_sec = value & 63;
        }
        if (now.tm_mon >= 0 && now.tm_mon <= 11 && now.tm_mday >= 1 &&
            now.tm_mday <= 31 && now.tm_hour < 24 && now.tm_min < 60 && now.tm_sec < 60) {
            s->rtc_offset = qemu_timedate_diff(&now);
        }
        return;
    }
    if (offset >= 0x20a00 && offset < 0x20a80 &&
        (offset - 0x20a00) % 0x20 == 0x14) {
        s->regs[offset / 4] &= ~value;
        b288_gpio_update(s);
        return;
    }
    if (offset == 0x21808) {
        s->regs[offset / 4] &= ~value;
        b288_lradc_update(s);
        return;
    }
    s->regs[offset / 4] = value;
    if (offset == 0x280a4) {
        s->regs[offset / 4] &= ~BIT(2); /* UART HALT, LCR update complete. */
    }
    if (offset >= 0x20a00 && offset < 0x20a80) {
        b288_gpio_update(s);
    }
    if (offset >= 0x20800 && offset < 0x20944) {
        b288_gpio_outputs(s);
    }
    if (offset == 0x21800 || offset == 0x21804) {
        timer_del(s->lradc_timer);
        b288_lradc_sample(s);
        b288_lradc_update(s);
    }
    /* Secondary reset vector lives in RTC GP registers on sun8iw10p1. */
    if (offset == 0x25c80 && (value & 3) == 3) {
        arm_set_cpu_on(1, s->regs[0x205e4 / 4], 0, 3, false);
    }
}

static const MemoryRegionOps b288_io_ops = {
    .read = b288_io_read,
    .write = b288_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void b288_io_reset(DeviceState *dev)
{
    B288IOState *s = B288_IO(dev);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x20000 / 4] = 0x90002900; /* CPU PLL, 1008 MHz */
    s->regs[0x20008 / 4] = 0x90035514;
    s->regs[0x20010 / 4] = 0x93006207;
    s->regs[0x20018 / 4] = 0x93006207;
    s->regs[0x20020 / 4] = 0x90001b00;
    s->regs[0x20028 / 4] = 0x90041811;
    s->regs[0x20044 / 4] = 0x90001800;
    s->regs[0x20048 / 4] = 0x93006207;
    s->regs[0x20050 / 4] = 0x00020001;
    s->regs[0x20054 / 4] = 0x00003180;
    s->regs[0x20058 / 4] = 0x01000000;
    s->regs[0x20060 / 4] = 0xffffffff;
    s->regs[0x20064 / 4] = 0xffffffff;
    s->regs[0x20c8c / 4] = 0x05db5dbf;
    s->avs_epoch[0] = s->avs_epoch[1] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->avs_count[0] = s->avs_count[1] = 0;
    s->lradc_mv = 2000;
    s->regs[0x2180c / 4] = 63;
    timer_del(s->lradc_timer);
    b288_lradc_update(s);
    b288_gpio_update(s);
}

static int b288_io_post_load(void *opaque, int version_id)
{
    B288IOState *s = opaque;
    b288_gpio_update(s);
    b288_gpio_outputs(s);
    b288_lradc_update(s);
    return 0;
}

static const VMStateDescription b288_io_vmstate = {
    .name = TYPE_B288_IO,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = b288_io_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, B288IOState, B288_IO_SIZE / 4),
        VMSTATE_INT64_ARRAY(avs_epoch, B288IOState, 2),
        VMSTATE_UINT32_ARRAY(avs_count, B288IOState, 2),
        VMSTATE_INT64(rtc_offset, B288IOState),
        VMSTATE_UINT32_ARRAY(gpio_input, B288IOState, 9),
        VMSTATE_UINT32(lradc_mv, B288IOState),
        VMSTATE_TIMER_PTR(lradc_timer, B288IOState),
        VMSTATE_END_OF_LIST()
    },
};

static void b288_io_init(Object *obj)
{
    B288IOState *s = B288_IO(obj);
    memory_region_init_io(&s->iomem, obj, &b288_io_ops, s,
                          "b288-registers", B288_IO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    memory_region_init_io(&s->avs, obj, &b288_avs_ops, s, "b288-avs", 16);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->avs);
    qdev_init_gpio_in_named(DEVICE(s), b288_gpio_input, "gpio-in", 9 * 32);
    qdev_init_gpio_out_named(DEVICE(s), s->gpio_output, "gpio-out", 9 * 32);
    qdev_init_gpio_in_named(DEVICE(s), b288_lradc_input, "lradc-in", 1);
    s->lradc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, b288_lradc_sample, s);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->lradc_irq);
    for (unsigned i = 0; i < 4; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->gpio_irq[i]);
    }
}

static void b288_io_finalize(Object *obj)
{
    timer_free(B288_IO(obj)->lradc_timer);
}

static void b288_io_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->vmsd = &b288_io_vmstate;
    device_class_set_legacy_reset(dc, b288_io_reset);
}

static void allwinner_b288_init(Object *obj)
{
    AwB288State *s = AW_B288(obj);
    for (unsigned i = 0; i < AW_B288_NUM_CPUS; i++) {
        object_initialize_child(obj, "cpu[*]", &s->cpus[i],
                                ARM_CPU_TYPE_NAME("cortex-a7"));
    }
    object_initialize_child(obj, "gic", &s->gic, TYPE_ARM_GIC);
    s->display = qdev_new(TYPE_AW_B288_EINK);
    object_property_add_child(obj, "display", OBJECT(s->display));
    object_unref(OBJECT(s->display));
    for (unsigned i = 0; i < AW_B288_NUM_MMC; i++) {
        object_initialize_child(obj, "mmc[*]", &s->mmc[i],
                                TYPE_AW_SDHOST_B288);
    }
    for (unsigned i = 0; i < AW_B288_NUM_I2C; i++) {
        object_initialize_child(obj, "i2c[*]", &s->i2c[i], TYPE_AW_I2C_SUN6I);
    }
}

static void allwinner_b288_realize(DeviceState *device, Error **errp)
{
    AwB288State *s = AW_B288(device);
    DeviceState *dev;
    SysBusDevice *busdev;

    memory_region_init_ram(&s->sram_a, OBJECT(s), "b288.sram-a", 64 * KiB,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0, &s->sram_a);
    memory_region_init_ram(&s->sram_c, OBJECT(s), "b288.sram-c", 128 * KiB,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x10000, &s->sram_c);

    for (unsigned i = 0; i < AW_B288_NUM_CPUS; i++) {
        qdev_prop_set_uint32(DEVICE(&s->cpus[i]), "cntfrq", 24000000);
        qdev_prop_set_uint32(DEVICE(&s->cpus[i]), "midr", 0x410fc075);
        qdev_prop_set_uint32(DEVICE(&s->cpus[i]), "reset-cbar", 0x01c80000);
        qdev_prop_set_bit(DEVICE(&s->cpus[i]), "start-powered-off", i != 0);
        qdev_realize(DEVICE(&s->cpus[i]), NULL, &error_fatal);
    }
    dev = DEVICE(&s->gic);
    qdev_prop_set_uint32(dev, "num-irq", AW_B288_NUM_SPI + GIC_INTERNAL);
    qdev_prop_set_uint32(dev, "num-cpu", AW_B288_NUM_CPUS);
    qdev_prop_set_uint32(dev, "revision", 2);
    qdev_prop_set_bit(dev, "has-security-extensions", false);
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x01c81000);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, 0x01c82000);
    for (unsigned i = 0; i < AW_B288_NUM_CPUS; i++) {
        static const unsigned timer_ppi[] = {
            [GTIMER_PHYS] = 14, [GTIMER_VIRT] = 11,
            [GTIMER_HYP] = 10, [GTIMER_SEC] = 13,
        };
        for (unsigned j = 0; j < ARRAY_SIZE(timer_ppi); j++) {
            qdev_connect_gpio_out(DEVICE(&s->cpus[i]), j,
                qdev_get_gpio_in(dev, AW_B288_NUM_SPI + i * GIC_INTERNAL +
                                 GIC_NR_SGIS + timer_ppi[j]));
        }
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i,
                            qdev_get_gpio_in(DEVICE(&s->cpus[i]), ARM_CPU_IRQ));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i + AW_B288_NUM_CPUS,
                            qdev_get_gpio_in(DEVICE(&s->cpus[i]), ARM_CPU_FIQ));
    }

    dev = qdev_new(TYPE_B288_IO);
    object_property_add_child(OBJECT(s), "registers", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, 0x01c00000, -1);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 1, 0x01c20c80, 2);
    qdev_pass_gpios(dev, device, "gpio-in");
    qdev_pass_gpios(dev, device, "gpio-out");
    qdev_pass_gpios(dev, device, "lradc-in");
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                        qdev_get_gpio_in(DEVICE(&s->gic), 30));
    for (unsigned i = 0; i < 4; i++) {
        static const unsigned irq[] = { 15, 16, 21, 17 };
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i + 1,
                            qdev_get_gpio_in(DEVICE(&s->gic), irq[i]));
    }
    create_unimplemented_device("b288-cpu-debug", 0x01700000, 16 * KiB);
    busdev = SYS_BUS_DEVICE(s->display);
    sysbus_realize(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0x01000000);
    sysbus_mmio_map_overlap(busdev, 1, 0x01c0c000, 1);
    sysbus_mmio_map(busdev, 2, 0x01400000);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->gic), 86));
    sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(DEVICE(&s->gic), 95));
    sysbus_connect_irq(busdev, 2, qdev_get_gpio_in(DEVICE(&s->gic), 106));

    dev = qdev_new(TYPE_AW_A10_PIT);
    qdev_prop_set_uint32(dev, "clk0-freq", 32768);
    qdev_prop_set_uint32(dev, "clk1-freq", 24000000);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map_overlap(busdev, 0, 0x01c20c00, 1);
    for (unsigned i = 0; i < 2; i++) {
        sysbus_connect_irq(busdev, i, qdev_get_gpio_in(DEVICE(&s->gic), 18 + i));
    }
    dev = qdev_new(TYPE_AW_WDT_SUN6I);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, 0x01c20ca0, 2);

    /* The single host port has EHCI and OHCI companion controllers. */
    DeviceState *ehci = qdev_new(TYPE_B288_EHCI);
    object_property_add_child(OBJECT(s), "ehci0", OBJECT(ehci));
    qdev_prop_set_bit(ehci, "companion-enable", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(ehci), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(ehci), 0, 0x01c1a000, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(ehci), 0,
                       qdev_get_gpio_in(DEVICE(&s->gic), 72));

    dev = qdev_new(TYPE_SYSBUS_OHCI);
    object_property_add_child(OBJECT(s), "ohci0", OBJECT(dev));
    qdev_prop_set_uint32(dev, "num-ports", 1);
    qdev_prop_set_string(dev, "masterbus",
                         SYS_BUS_EHCI(ehci)->ehci.bus.qbus.name);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, 0x01c1a400, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(DEVICE(&s->gic), 73));

    for (unsigned i = 0; i < 4; i++) {
        dev = DEVICE(&s->mmc[i]);
        object_property_set_link(OBJECT(dev), "dma-memory",
                                  OBJECT(get_system_memory()), &error_fatal);
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_realize(busdev, &error_fatal);
        sysbus_mmio_map_overlap(busdev, 0, 0x01c0f000 + i * 0x1000, 1);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->gic), 60 + i));
    }
    for (unsigned i = 0; i < 3; i++) {
        dev = DEVICE(&s->i2c[i]);
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_realize(busdev, &error_fatal);
        sysbus_mmio_map_overlap(busdev, 0, 0x01c2ac00 + i * 0x400, 1);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(DEVICE(&s->gic), 6 + i));
    }
    serial_mm_init(get_system_memory(), 0x01c28000, 2,
                    qdev_get_gpio_in(DEVICE(&s->gic), 0), 115200,
                    serial_hd(0), DEVICE_LITTLE_ENDIAN);
}

static void allwinner_b288_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = allwinner_b288_realize;
    dc->user_creatable = false; /* Owns CPUs and uses serial_hd(). */
}

static void b288_ehci_class_init(ObjectClass *oc, const void *data)
{
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(oc);

    sec->portnr = 1;
}

static const TypeInfo b288_types[] = {
    {
        .name = TYPE_B288_EHCI,
        .parent = TYPE_AW_H3_EHCI,
        .class_init = b288_ehci_class_init,
    },
    {
        .name = TYPE_B288_IO,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(B288IOState),
        .instance_init = b288_io_init,
        .instance_finalize = b288_io_finalize,
        .class_init = b288_io_class_init,
    }, {
        .name = TYPE_AW_B288,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AwB288State),
        .instance_init = allwinner_b288_init,
        .class_init = allwinner_b288_class_init,
    },
};
DEFINE_TYPES(b288_types)
