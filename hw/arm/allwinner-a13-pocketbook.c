/*
 * PocketBook Touch Lux 4 (627), Allwinner A13 / sun5i.
 * The update contains a native Linux Image and filesystems, without U-Boot.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "hw/arm/machines-qom.h"
#include "hw/char/serial-mm.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/display/allwinner-a13-eink.h"
#include "hw/i2c/allwinner-i2c.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/elan-ktf2k.h"
#include "hw/i2c/zforce.h"
#include "hw/intc/allwinner-a10-pic.h"
#include "hw/sd/allwinner-sdhost.h"
#include "hw/timer/allwinner-a10-pit.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "ui/input.h"

#define A13_RAM_BASE 0x40000000
#define A13_KERNEL_BASE 0x40008000
#define A13_SCRIPT_BASE 0x43000000
#define A13_IO_SIZE 0x40000

#define TYPE_A13PB_IO "allwinner-a13-pocketbook-io"
OBJECT_DECLARE_SIMPLE_TYPE(A13PBIOState, A13PB_IO)

struct A13PBIOState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion counter;
    MemoryRegion pwm;
    uint32_t regs[A13_IO_SIZE / 4];
    uint32_t gpio_input[9];
    qemu_irq gpio_irq;
    qemu_irq lradc_irq;
    qemu_irq lcd_drive;
    qemu_irq gpio_output[9 * 32];
    QEMUTimer *key_timer;
    uint32_t key_sample;
    int64_t counter_epoch;
    uint64_t counter_latched;
};

static void a13_key_update(A13PBIOState *s)
{
    qemu_set_irq(s->lradc_irq, !!(s->regs[0x22804 / 4] &
                                 s->regs[0x22808 / 4]));
}

static void a13_key_sample(void *opaque)
{
    A13PBIOState *s = opaque;
    if ((s->regs[0x22800 / 4] & 1) && s->key_sample < 63) {
        s->regs[0x2280c / 4] = s->key_sample;
        s->regs[0x22808 / 4] |= 1;
        a13_key_update(s);
        timer_mod(s->key_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 8000000);
    }
}

static void a13_key_input(void *opaque, int number, int sample)
{
    A13PBIOState *s = opaque;
    bool pressed = sample < 63;
    if (pressed != (s->key_sample < 63)) {
        s->regs[0x22808 / 4] |= pressed ? BIT(1) : BIT(4);
    }
    s->key_sample = CLAMP(sample, 0, 63);
    timer_del(s->key_timer);
    a13_key_sample(s);
    a13_key_update(s);
}

static uint64_t a13_counter_read(void *opaque, hwaddr offset, unsigned size)
{
    A13PBIOState *s = opaque;
    return offset == 4 ? (uint32_t)s->counter_latched :
           offset == 8 ? s->counter_latched >> 32 : 0;
}

static void a13_counter_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    A13PBIOState *s = opaque;
    if (offset == 0) {
        if (value & 1) {
            s->counter_epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        if (value & 2) {
            s->counter_latched = muldiv64(
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->counter_epoch,
                24000000, NANOSECONDS_PER_SECOND);
        }
    }
}

static const MemoryRegionOps a13_counter_ops = {
    .read = a13_counter_read,
    .write = a13_counter_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void a13_gpio_update(A13PBIOState *s)
{
    for (unsigned pin = 0; pin < 32; pin++) {
        unsigned mode = extract32(s->regs[(0x20a00 + pin / 8 * 4) / 4],
                                   pin % 8 * 4, 4);
        unsigned mux = extract32(s->regs[(0x208d8 + pin / 8 * 4) / 4],
                                  pin % 8 * 4, 3);
        bool high = !!(s->gpio_input[6] & BIT(pin));
        if (mux == 6 && ((mode == 2 && high) || (mode == 3 && !high))) {
            s->regs[0x20a14 / 4] |= BIT(pin);
        }
    }
    qemu_set_irq(s->gpio_irq, !!(s->regs[0x20a10 / 4] &
                                s->regs[0x20a14 / 4]));
}

static void a13_gpio_outputs(A13PBIOState *s)
{
    for (unsigned bank = 0; bank < 9; bank++) {
        unsigned base = 0x20800 + bank * 0x24;
        for (unsigned pin = 0; pin < 32; pin++) {
            qemu_set_irq(s->gpio_output[bank * 32 + pin],
                !!(s->regs[(base + 0x10) / 4] & BIT(pin)));
        }
    }
    qemu_set_irq(s->lcd_drive,
        extract32(s->regs[0x2086c / 4], 12, 3) == 2);
}

static void a13_gpio_input(void *opaque, int number, int level)
{
    A13PBIOState *s = opaque;
    unsigned bank = number / 32, pin = number % 32;
    bool old = !!(s->gpio_input[bank] & BIT(pin));

    s->gpio_input[bank] = deposit32(s->gpio_input[bank], pin, 1, !!level);
    /* A13 external interrupts are multiplexed onto port G. */
    if (bank == 6) {
        unsigned mode = extract32(s->regs[(0x20a00 + pin / 8 * 4) / 4],
                                   pin % 8 * 4, 4);
        unsigned mux = extract32(s->regs[(0x208d8 + pin / 8 * 4) / 4],
                                  pin % 8 * 4, 3);
        if (mux == 6 && ((mode == 0 && !old && level) ||
            (mode == 1 && old && !level) ||
            (mode == 2 && level) || (mode == 3 && !level) ||
            (mode == 4 && old != !!level))) {
            s->regs[0x20a14 / 4] |= BIT(pin);
        }
    }
    a13_gpio_update(s);
}

static uint64_t a13_io_read(void *opaque, hwaddr offset, unsigned size)
{
    A13PBIOState *s = opaque;
    if (offset == 0x1004) {
        /* SDR_DCR: one 512 MiB rank. */
        return 4 << 3;
    }
    if ((offset & 0x3ff) == 0x7c &&
        offset >= 0x28000 && offset < 0x2a000) {
        return 6; /* DesignWare UART: TX not full, transmitter empty. */
    }
    if (offset >= 0x20800 && offset < 0x20944 &&
        (offset - 0x20800) % 0x24 == 0x10) {
        unsigned bank = (offset - 0x20800) / 0x24;
        uint32_t outputs = 0;
        for (unsigned pin = 0; pin < 32; pin++) {
            uint32_t cfg = s->regs[(0x20800 + bank * 0x24 + pin / 8 * 4) / 4];
            if (extract32(cfg, pin % 8 * 4, 3) == 1) {
                outputs |= BIT(pin);
            }
        }
        return (s->regs[offset / 4] & outputs) |
               (s->gpio_input[bank] & ~outputs);
    }
    return s->regs[offset / 4];
}

static void a13_io_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
    A13PBIOState *s = opaque;
    if (offset == 0x20a14 || offset == 0x22808) {
        s->regs[offset / 4] &= ~value;
    } else {
        s->regs[offset / 4] = value;
    }
    if (offset >= 0x20a00 && offset <= 0x20a14) {
        a13_gpio_update(s);
    }
    if (offset >= 0x20800 && offset < 0x20944) {
        a13_gpio_outputs(s);
        a13_gpio_update(s);
    }
    if (offset == 0x22800 || offset == 0x22804 || offset == 0x22808) {
        timer_del(s->key_timer);
        a13_key_sample(s);
        a13_key_update(s);
    }
}

static const MemoryRegionOps a13_io_ops = {
    .read = a13_io_read,
    .write = a13_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void a13_io_reset(DeviceState *dev)
{
    A13PBIOState *s = A13PB_IO(dev);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->gpio_input, 0xff, sizeof(s->gpio_input));
    s->key_sample = 63;
    s->counter_epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->counter_latched = 0;
    timer_del(s->key_timer);
    s->regs[0x20000 / 4] = 0xa1005000;
    s->regs[0x20008 / 4] = 0x08100010;
    s->regs[0x20010 / 4] = 0x8010d063;
    s->regs[0x20018 / 4] = 0xa1009911;
    s->regs[0x20020 / 4] = 0x91049280;
    s->regs[0x20028 / 4] = 0xa1009911;
    s->regs[0x20030 / 4] = 0x8010d063;
    s->regs[0x20050 / 4] = 0x00138013;
    s->regs[0x20054 / 4] = 0x00020010;
    s->regs[0x15000 / 4] = 1 << 16; /* A13 silicon family. */
    a13_gpio_update(s);
    a13_key_update(s);
    a13_gpio_outputs(s);
}

static void a13_io_init(Object *obj)
{
    A13PBIOState *s = A13PB_IO(obj);
    memory_region_init_io(&s->iomem, obj, &a13_io_ops, s,
                          TYPE_A13PB_IO, A13_IO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    memory_region_init_io(&s->counter, obj, &a13_counter_ops, s,
                          "pb627.counter", 12);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->counter);
    memory_region_init_alias(&s->pwm, obj, "pb627.pwm", &s->iomem,
                             0x20e00, 0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->pwm);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->gpio_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->lradc_irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->lcd_drive, "lcd-drive", 1);
    qdev_init_gpio_in_named(DEVICE(obj), a13_gpio_input, "gpio-in", 9 * 32);
    qdev_init_gpio_in_named(DEVICE(obj), a13_key_input, "key-sample", 1);
    qdev_init_gpio_out_named(DEVICE(obj), s->gpio_output, "gpio-out", 9 * 32);
    s->key_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, a13_key_sample, s);
}

static int a13_io_post_load(void *opaque, int version_id)
{
    A13PBIOState *s = opaque;
    a13_gpio_update(s);
    a13_key_update(s);
    a13_gpio_outputs(s);
    return 0;
}

static const VMStateDescription vmstate_a13_io = {
    .name = TYPE_A13PB_IO,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = a13_io_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, A13PBIOState, A13_IO_SIZE / 4),
        VMSTATE_UINT32_ARRAY(gpio_input, A13PBIOState, 9),
        VMSTATE_UINT32(key_sample, A13PBIOState),
        VMSTATE_INT64(counter_epoch, A13PBIOState),
        VMSTATE_UINT64(counter_latched, A13PBIOState),
        VMSTATE_TIMER_PTR(key_timer, A13PBIOState),
        VMSTATE_END_OF_LIST()
    },
};

static void a13_io_finalize(Object *obj)
{
    timer_free(A13PB_IO(obj)->key_timer);
}

static void a13_io_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_legacy_reset(dc, a13_io_reset);
    dc->vmsd = &vmstate_a13_io;
}

#define TYPE_A13PB_MACHINE MACHINE_TYPE_NAME("allwinner-a13-pocketbook-627")
OBJECT_DECLARE_SIMPLE_TYPE(A13PBMachineState, A13PB_MACHINE)

struct A13PBMachineState {
    MachineState parent_obj;
    ARMCPU cpu;
    AwA10PICState intc;
    AwA10PITState timer;
    AwSdHostState mmc;
    AWI2CState i2c[3];
    EHCISysBusState ehci;
    OHCISysBusState ohci;
    A13PBIOState io;
    MemoryRegion sram;
    DeviceState *touch;
    DeviceState *display;
    DeviceState *wifi;
    QemuInputHandlerState *input;
};

static void a13_buttons_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    A13PBIOState *s = A13PB_IO(dev);
    InputKeyEvent *key = evt->u.key.data;
    switch (qemu_input_key_value_to_qcode(key->key)) {
    case Q_KEY_CODE_PGDN:
    case Q_KEY_CODE_RIGHT:
        a13_gpio_input(s, 6 * 32 + 9, !key->down);
        break;
    case Q_KEY_CODE_PGUP:
    case Q_KEY_CODE_LEFT:
        a13_gpio_input(s, 6 * 32 + 10, !key->down);
        break;
    case Q_KEY_CODE_HOME:
        a13_key_input(s, 0, key->down ? 8 : 63);
        break;
    case Q_KEY_CODE_RET:
    case Q_KEY_CODE_ESC:
    case Q_KEY_CODE_BACKSPACE:
    case Q_KEY_CODE_MENU:
        a13_key_input(s, 0, key->down ? 43 : 63);
        break;
    default:
        break;
    }
}

static const QemuInputHandler a13_buttons_handler = {
    .name = "PocketBook 627 buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = a13_buttons_event,
};

typedef struct A13ScriptEntry {
    const char *section, *name;
    int32_t value;
    unsigned type;
    int32_t gpio[6];
    const char *text;
} A13ScriptEntry;

#define A13_PIN(sec, key, bank, pin, function, pull, drive, level) \
    { sec, key, 0, 4, { bank, pin, function, pull, drive, level } }
#define A13_TEXT(sec, key, str) { sec, key, 0, 2, { 0 }, str }

static void a13_script(void)
{
    /* Linux's old script parser uses word offsets and 40-byte key records. */
    static const A13ScriptEntry entries[] = {
        { "uart_para", "uart_debug_port", 1 },
        { "uart_para1", "uart_used", 1 },
        { "uart_para1", "uart_port", 1 },
        { "uart_para1", "uart_type", 2 },
        A13_PIN("uart_para1", "uart_tx", 7, 3, 4, 1, 0, 1),
        A13_PIN("uart_para1", "uart_rx", 7, 4, 4, 1, 0, 1),
        { "twi0_para", "twi0_used", 1 },
        A13_PIN("twi0_para", "twi0_scl", 2, 0, 2, 1, 0, 1),
        A13_PIN("twi0_para", "twi0_sda", 2, 1, 2, 1, 0, 1),
        { "twi1_para", "twi1_used", 1 },
        A13_PIN("twi1_para", "twi1_scl", 2, 15, 2, 1, 0, 1),
        A13_PIN("twi1_para", "twi1_sda", 2, 16, 2, 1, 0, 1),
        { "twi2_para", "twi2_used", 1 },
        A13_PIN("twi2_para", "twi2_scl", 2, 17, 2, 1, 0, 1),
        A13_PIN("twi2_para", "twi2_sda", 2, 18, 2, 1, 0, 1),
        { "mmc0_para", "sdc_used", 0 },
        { "mmc1_para", "sdc_used", 0 },
        { "mmc2_para", "sdc_used", 1 },
        { "mmc2_para", "sdc_detmode", 3 },
        { "mmc2_para", "sdc_use_wp", 0 },
        { "mmc2_para", "sdc_buswidth", 4 },
        A13_PIN("mmc2_para", "sdc_cmd", 3, 6, 3, 1, 2, 1),
        A13_PIN("mmc2_para", "sdc_clk", 3, 7, 3, 1, 2, 1),
        A13_PIN("mmc2_para", "sdc_d0", 3, 8, 3, 1, 2, 1),
        A13_PIN("mmc2_para", "sdc_d1", 3, 9, 3, 1, 2, 1),
        A13_PIN("mmc2_para", "sdc_d2", 3, 10, 3, 1, 2, 1),
        A13_PIN("mmc2_para", "sdc_d3", 3, 11, 3, 1, 2, 1),
        { "dram_para", "dram_clk", 408 },
        { "clock", "pll6", 600 },
        { "clock", "apb", 24 },
        { "pmu_para", "pmu_used", 1 },
        { "pmu_para", "pmu_twi_id", 0 },
        { "pmu_para", "pmu_twi_addr", 0x34 },
        { "pmu_para", "pmu_irq_id", 0 },
        { "pmu_para", "pmu_battery_cap", 1500 },
        { "rtc_para", "rtc_used", 1 },
        A13_TEXT("rtc_para", "rtc_name", "pcf8563"),
        { "rtc_para", "rtc_twi_id", 1 },
        { "rtc_para", "rtc_twi_addr", 0x51 },
        { "usbc0", "usb_used", 0 },
        { "usbc0", "usb_port_type", 0 },
        { "usbc0", "usb_detect_type", 0 },
        { "usbc1", "usb_used", 1 },
        { "usbc1", "usb_port_type", 1 },
        { "usbc1", "usb_detect_type", 0 },
        { "usbc1", "usb_controller_type", 0 },
        { "usbc1", "usb_host_init_state", 1 },
        { "GPIOKEY_para", "GPIOKEY_used", 1 },
        A13_PIN("GPIOKEY_para", "GPIO_KEY_THREE", 7, 9, 6, 1, 0, 1),
        A13_PIN("GPIOKEY_para", "GPIO_KEY_FOUR", 7, 10, 6, 1, 0, 1),
        { "ctp_para", "ctp_used", 1 },
        A13_TEXT("ctp_para", "ctp_name", "ektf2k_elan"),
        { "ctp_para", "ctp_twi_id", 2 },
        { "ctp_para", "ctp_screen_max_x", 758 },
        { "ctp_para", "ctp_screen_max_y", 1024 },
        { "ctp_para", "ctp_revert_x_flag", 0 },
        { "ctp_para", "ctp_revert_y_flag", 0 },
        { "ctp_para", "ctp_exchange_x_y_flag", 0 },
        A13_PIN("ctp_para", "ctp_int_port", 7, 11, 6, 1, 0, 1),
        A13_PIN("ctp_para", "ctp_reset", 7, 12, 1, 1, 0, 1),
        A13_PIN("ctp_para", "ctp_wakeup", 3, 13, 1, 1, 0, 1),
        A13_PIN("ctp_para", "ctp_powerpin", 3, 13, 1, 1, 0, 1),
    };
    g_autofree uint8_t *blob = g_malloc0(64 * KiB);
    unsigned sections = 0;
    for (unsigned i = 0; i < ARRAY_SIZE(entries); i++) {
        if (!i || strcmp(entries[i].section, entries[i - 1].section)) {
            sections++;
        }
    }
    stl_le_p(blob, sections);
    stl_le_p(blob + 4, 1);
    unsigned offset = 16 + sections * 40, section = 0;
    for (unsigned i = 0; i < ARRAY_SIZE(entries);) {
        unsigned end = i + 1;
        while (end < ARRAY_SIZE(entries) &&
               !strcmp(entries[i].section, entries[end].section)) {
            end++;
        }
        uint8_t *main = blob + 16 + section++ * 40;
        pstrcpy((char *)main, 32, entries[i].section);
        stl_le_p(main + 32, end - i);
        stl_le_p(main + 36, offset / 4);
        unsigned values = offset + (end - i) * 40;
        for (unsigned j = i; j < end; j++) {
            uint8_t *key = blob + offset;
            unsigned type = entries[j].type ? entries[j].type : 1;
            unsigned size = type == 4 ? 24 : type == 2 ?
                             ROUND_UP(strlen(entries[j].text) + 1, 4) : 4;
            pstrcpy((char *)key, 32, entries[j].name);
            stl_le_p(key + 32, values / 4);
            stl_le_p(key + 36, (type << 16) | size / 4);
            if (type == 4) {
                for (unsigned k = 0; k < 6; k++) {
                    stl_le_p(blob + values + k * 4, entries[j].gpio[k]);
                }
            } else if (type == 2) {
                memcpy(blob + values, entries[j].text, strlen(entries[j].text));
            } else {
                stl_le_p(blob + values, entries[j].value);
            }
            values += size;
            offset += 40;
        }
        offset = values;
        i = end;
    }
    rom_add_blob_fixed("pb627.script", blob, 64 * KiB, A13_SCRIPT_BASE);
}

static void a13_pb_reset(void *opaque)
{
    A13PBMachineState *s = opaque;
    cpu_reset(CPU(&s->cpu));
    qemu_set_irq(qdev_get_gpio_in_named(s->touch, "power", 0), 1);
    qemu_set_irq(qdev_get_gpio_in_named(s->touch, "reset", 0), 1);
    s->cpu.env.regs[13] = A13_RAM_BASE + 0x8000;
    cpu_set_pc(CPU(&s->cpu), A13_KERNEL_BASE);
}

static void a13_pb_init(MachineState *machine)
{
    A13PBMachineState *s = A13PB_MACHINE(machine);
    if (machine->ram_size != 512 * MiB || !machine->kernel_filename) {
        error_report("PocketBook 627 requires 512 MiB RAM and its native kernel (-kernel)");
        exit(EXIT_FAILURE);
    }
    object_initialize_child(OBJECT(s), "cpu", &s->cpu,
                            ARM_CPU_TYPE_NAME("cortex-a8"));
    qdev_realize(DEVICE(&s->cpu), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), A13_RAM_BASE, machine->ram);
    memory_region_init_ram(&s->sram, NULL, "pb627.sram", 192 * KiB,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0, &s->sram);
    object_initialize_child(OBJECT(s), "io", &s->io, TYPE_A13PB_IO);
    sysbus_realize(SYS_BUS_DEVICE(&s->io), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->io), 0, 0x01c00000, -1);
    object_initialize_child(OBJECT(s), "intc", &s->intc, TYPE_AW_A10_PIC);
    /* The sun5i IRQ entry scans only enabled, unmasked pending sources. */
    qdev_prop_set_bit(DEVICE(&s->intc), "filtered-pending", true);
    sysbus_realize(SYS_BUS_DEVICE(&s->intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0, 0x01c20400);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 1,
                       qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_FIQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->io), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 28));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->io), 1,
                       qdev_get_gpio_in(DEVICE(&s->intc), 31));
    object_initialize_child(OBJECT(s), "timer", &s->timer, TYPE_AW_A10_PIT);
    qdev_prop_set_uint32(DEVICE(&s->timer), "clk0-freq", 32768);
    qdev_prop_set_uint32(DEVICE(&s->timer), "clk1-freq", 24000000);
    sysbus_realize(SYS_BUS_DEVICE(&s->timer), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer), 0, 0x01c20c00);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->io), 1, 0x01c20ca0, 10);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->io), 2, 0x01c20e00, 10);
    for (unsigned i = 0; i < 6; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer), i,
            qdev_get_gpio_in(DEVICE(&s->intc), i < 4 ? 22 + i : 63 + i));
    }
    serial_mm_init(get_system_memory(), 0x01c28400, 2,
        qdev_get_gpio_in(DEVICE(&s->intc), 2), 115200, serial_hd(0),
        DEVICE_LITTLE_ENDIAN);
    for (unsigned i = 0; i < 3; i++) {
        object_initialize_child(OBJECT(s), "i2c[*]", &s->i2c[i], TYPE_AW_I2C);
        sysbus_realize(SYS_BUS_DEVICE(&s->i2c[i]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 0, 0x01c2ac00 + i * 0x400);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 7 + i));
    }
    DeviceState *pmu = qdev_new("axp209_pmu");
    qdev_prop_set_uint8(pmu, "address", 0x34);
    qdev_prop_set_bit(pmu, "battery-present", true);
    qdev_realize_and_unref(pmu, BUS(s->i2c[0].bus), &error_fatal);
    i2c_slave_create_simple(s->i2c[1].bus, "pcf8563", 0x51);
    DeviceState *epd_pmu = qdev_new(TYPE_TPS65185);
    object_property_add_child(OBJECT(s), "display-pmic", OBJECT(epd_pmu));
    qdev_prop_set_uint8(epd_pmu, "address", 0x68);
    qdev_prop_set_bit(epd_pmu, "gpio-rails", true);
    qdev_realize_and_unref(epd_pmu, BUS(s->i2c[1].bus), &error_fatal);
    qdev_connect_gpio_out_named(epd_pmu, "pwrgood", 0,
        qdev_get_gpio_in_named(DEVICE(&s->io), "gpio-in", 4 * 32 + 6));
    qdev_connect_gpio_out_named(DEVICE(&s->io), "gpio-out", 3 * 32 + 18,
        qdev_get_gpio_in_named(epd_pmu, "powerup", 0));
    qdev_connect_gpio_out_named(DEVICE(&s->io), "gpio-out", 4 * 32 + 7,
        qdev_get_gpio_in_named(epd_pmu, "power-enable", 0));
    s->touch = qdev_new(TYPE_ELAN_KTF2K);
    object_property_add_child(OBJECT(s), "touchscreen", OBJECT(s->touch));
    qdev_prop_set_uint8(s->touch, "address", 0x15);
    qdev_prop_set_uint16(s->touch, "x-resolution", 758);
    qdev_prop_set_uint16(s->touch, "y-resolution", 1024);
    qdev_prop_set_bit(s->touch, "swap-axes", false);
    qdev_prop_set_bit(s->touch, "invert-x", false);
    qdev_prop_set_bit(s->touch, "swap-coordinate-ranges", true);
    qdev_prop_set_uint8(s->touch, "contact-mask", 1);
    qdev_prop_set_uint16(s->touch, "firmware-version", 0x12ae);
    qdev_realize_and_unref(s->touch, BUS(s->i2c[2].bus), &error_fatal);
    qdev_connect_gpio_out_named(s->touch, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&s->io), "gpio-in", 6 * 32 + 11));
    qdev_connect_gpio_out_named(DEVICE(&s->io), "gpio-out", 6 * 32 + 12,
        qdev_get_gpio_in_named(s->touch, "reset", 0));
    qdev_connect_gpio_out_named(DEVICE(&s->io), "gpio-out", 2 * 32 + 13,
        qdev_get_gpio_in_named(s->touch, "power", 0));
    s->input = qemu_input_handler_register(DEVICE(&s->io), &a13_buttons_handler);
    s->display = qdev_new(TYPE_AW_A13_EINK);
    object_property_add_child(OBJECT(s), "display", OBJECT(s->display));
    /* The native driver starts its bootloader pixel index at gray level two. */
    qdev_prop_set_uint8(s->display, "initial-grey", 2 * 17);
    sysbus_realize(SYS_BUS_DEVICE(s->display), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->display), 0, 0x01e00000);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->display), 1, 0x01c0c000);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->display), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 44));
    qdev_connect_gpio_out_named(DEVICE(&s->io), "lcd-drive", 0,
        qdev_get_gpio_in_named(s->display, "drive-enable", 0));
    object_initialize_child(OBJECT(s), "ehci", &s->ehci, "allwinner-a13-ehci");
    qdev_prop_set_bit(DEVICE(&s->ehci), "companion-enable", true);
    sysbus_realize(SYS_BUS_DEVICE(&s->ehci), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ehci), 0, 0x01c14000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->ehci), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 39));
    object_initialize_child(OBJECT(s), "ohci", &s->ohci, TYPE_SYSBUS_OHCI);
    qdev_prop_set_uint32(DEVICE(&s->ohci), "num-ports", 1);
    object_property_set_str(OBJECT(&s->ohci), "masterbus", "usb-bus.0", &error_fatal);
    sysbus_realize(SYS_BUS_DEVICE(&s->ohci), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ohci), 0, 0x01c14400);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->ohci), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 40));
    s->wifi = qdev_new("rtl8188eu");
    qdev_prop_set_string(s->wifi, "port", "1");
    qemu_configure_nic_device(s->wifi, true, "rtl8188eu");
    usb_realize_and_unref(USB_DEVICE(s->wifi), &s->ehci.ehci.bus, &error_fatal);
    qdev_connect_gpio_out_named(pmu, "ldo3-enable", 0,
        qdev_get_gpio_in_named(s->wifi, "power", 0));
    object_initialize_child(OBJECT(s), "mmc2", &s->mmc, "allwinner-a13-sdhost");
    object_property_set_link(OBJECT(&s->mmc), "dma-memory",
                             OBJECT(get_system_memory()), &error_fatal);
    sysbus_realize(SYS_BUS_DEVICE(&s->mmc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->mmc), 0, 0x01c11000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->mmc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 34));
    DriveInfo *di = drive_get(IF_SD, 0, 0);
    if (di) {
        DeviceState *card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di), &error_fatal);
        qdev_realize_and_unref(card, qdev_get_child_bus(DEVICE(&s->mmc), "sd-bus"),
                               &error_fatal);
    }
    if (load_image_targphys(machine->kernel_filename, A13_KERNEL_BASE,
                            16 * MiB, &error_fatal) < 0) {
        error_report("Cannot load PocketBook 627 kernel");
        exit(EXIT_FAILURE);
    }
    const char *command = machine->kernel_cmdline && *machine->kernel_cmdline ?
        machine->kernel_cmdline :
        "console=ttyS0,115200 root=/dev/mmcblk0p7 rootfstype=ext2 rootwait rw loglevel=5";
    unsigned words = (strlen(command) + 4) / 4;
    g_autofree uint8_t *tags = g_malloc0(36 + 8 + words * 4 + 8);
    stl_le_p(tags, 5); stl_le_p(tags + 4, 0x54410001);
    stl_le_p(tags + 8, 1); stl_le_p(tags + 12, 4096);
    stl_le_p(tags + 20, 4); stl_le_p(tags + 24, 0x54410002);
    stl_le_p(tags + 28, machine->ram_size); stl_le_p(tags + 32, A13_RAM_BASE);
    stl_le_p(tags + 36, words + 2); stl_le_p(tags + 40, 0x54410009);
    memcpy(tags + 44, command, strlen(command));
    rom_add_blob_fixed("pb627.atags", tags, 52 + words * 4, A13_RAM_BASE);
    a13_script();
    qemu_register_reset(a13_pb_reset, s);
}

static void a13_pb_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "PocketBook Touch Lux 4 (627, Allwinner A13)";
    mc->init = a13_pb_init;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "pb627.ram";
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a8");
    mc->default_nic = "rtl8188eu";
    mc->ignore_memory_transaction_failures = true;
    mc->units_per_default_bus = 1;
}

static void a13_ehci_class_init(ObjectClass *oc, const void *data)
{
    SysBusEHCIClass *ec = SYS_BUS_EHCI_CLASS(oc);
    ec->capsbase = 0;
    ec->opregbase = 0x10;
    ec->portscbase = 0x44;
    ec->portnr = 1;
}

static void a13_sdhost_class_init(ObjectClass *oc, const void *data)
{
    /* A13 keeps the FIFO at 0x100, with 16-bit DMA descriptor lengths. */
    AW_SDHOST_CLASS(oc)->max_desc_size = 64 * KiB;
}

static const TypeInfo a13_pb_types[] = {
    { .name = "allwinner-a13-sdhost", .parent = TYPE_AW_SDHOST_SUN4I,
      .class_init = a13_sdhost_class_init },
    { .name = "allwinner-a13-ehci", .parent = TYPE_PLATFORM_EHCI,
      .class_init = a13_ehci_class_init },
    { .name = TYPE_A13PB_IO, .parent = TYPE_SYS_BUS_DEVICE,
      .instance_size = sizeof(A13PBIOState), .instance_init = a13_io_init,
      .instance_finalize = a13_io_finalize,
      .class_init = a13_io_class_init },
    { .name = TYPE_A13PB_MACHINE, .parent = TYPE_MACHINE,
      .instance_size = sizeof(A13PBMachineState), .class_init = a13_pb_class_init,
      .interfaces = arm_machine_interfaces },
};
DEFINE_TYPES(a13_pb_types)
