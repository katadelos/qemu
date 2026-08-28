/* Amazon Yoshi-family (Tequila and Whitney) board emulation */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/arm/fsl-imx50.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/cyttsp.h"
#include "hw/i2c/zforce.h"
#include "hw/input/tequila-keyboard.h"
#include "hw/sd/sd.h"
#include "hw/ssi/mc13892.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/reset.h"
#include "system/qtest.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "trace.h"

#define YOSHI_RAM_BASE 0x70000000
#define YOSHI_RAM_MAX  (512 * MiB)
#define YOSHI_MACHINE_ID 2955 /* MACH_TYPE_MX50_YOSHI */
/*
 * The Kindle image is CONFIG_IRAM_BOOT U-Boot, linked for the i.MX50
 * boot-ROM destination rather than the external SDRAM load address.
 */
#define YOSHI_UBOOT_ADDR 0xf8007000
#define YOSHI_UBOOT_MAX  0x00019000
#define YOSHI_IRAM_TOP   0xf8020000
#define YOSHI_IDME_BASE  0x0003f000
/*
 * The Kobo-derived Whitney image is a second-stage U-Boot linked for SDRAM,
 * matching the Kobo Touch loader contract rather than Lab126's OCRAM image.
 */
#define WHITNEY_KOBO_UBOOT_ADDR  0x77800000
#define WHITNEY_KOBO_UBOOT_ENTRY 0x77800ae0
#define WHITNEY_KOBO_UBOOT_MAX   0x000c0000
#define WHITNEY_KOBO_KERNEL_ADDR 0x70800000
#define WHITNEY_KOBO_KERNEL_MAX  0x00400000

#define TYPE_YOSHI_MACHINE MACHINE_TYPE_NAME("imx50-yoshi-base")
#define TYPE_TEQUILA_MACHINE MACHINE_TYPE_NAME("imx50-tequila")
#define TYPE_WHITNEY_MACHINE MACHINE_TYPE_NAME("imx50-whitney")
#define TYPE_CELESTE_MACHINE MACHINE_TYPE_NAME("imx50-celeste")
OBJECT_DECLARE_SIMPLE_TYPE(YoshiMachineState, YOSHI_MACHINE)

#define TYPE_YOSHI_BATTERY "yoshi-battery"
#define TYPE_WHITNEY_BATTERY "whitney-battery"
#define TYPE_CELESTE_BATTERY "celeste-battery"
OBJECT_DECLARE_SIMPLE_TYPE(YoshiBatteryState, YOSHI_BATTERY)

#define TYPE_YOSHI_PAPYRUS "yoshi-papyrus"
OBJECT_DECLARE_SIMPLE_TYPE(YoshiPapyrusState, YOSHI_PAPYRUS)

#define TYPE_WHITNEY_MMA8453 "whitney-mma8453"
OBJECT_DECLARE_SIMPLE_TYPE(WhitneyMMA8453State, WHITNEY_MMA8453)

struct YoshiBatteryState {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer;
    bool pointer_valid;
    bool whitney;
    bool celeste;
};

struct YoshiPapyrusState {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer;
    bool pointer_valid;
    qemu_irq pwrgood;
};

struct WhitneyMMA8453State {
    I2CSlave parent_obj;
    uint8_t regs[0x32];
    uint8_t pointer;
    bool pointer_valid;
    qemu_irq irq[2];
};

struct YoshiMachineState {
    MachineState parent_obj;
    char *idme_serial;
    char *idme_accel;
    char *idme_mac;
    char *idme_sec;
    char *idme_pcbsn;
    char *idme_bootmode;
    char *idme_postmode;
    bool whitney;
    bool celeste;
    bool idme_boot_partitions;
    FslIMX50State *soc;
    QEMUTimer *diagnostic_timer;
};

typedef struct YoshiIdmeField {
    const char *name;
    char *value;
    uint32_t offset;
    size_t size;
} YoshiIdmeField;

static struct arm_boot_info yoshi_binfo;

static void whitney_diagnostic_heartbeat(void *opaque)
{
    YoshiMachineState *tms = opaque;
    CPUState *cpu = CPU(&tms->soc->cpu);
    int64_t host_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

    trace_whitney_heartbeat(host_ns,
                            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                            cpu->cc->get_pc(cpu), cpu->halted);
    timer_mod(tms->diagnostic_timer, host_ns + 250 * SCALE_MS);
}

static void yoshi_battery_reset(DeviceState *dev)
{
    YoshiBatteryState *s = YOSHI_BATTERY(dev);
    uint16_t temperature = (25 + 273) * 4;
    uint16_t voltage = 3900;
    uint16_t capacity_raw = 3921;
    uint16_t learned_capacity_raw = 4986;

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->pointer_valid = false;

    /*
     * The Lab126 driver reads the TI gas gauge as individual SMBus byte
     * registers.  Supply a healthy, mostly charged Yoshi-family battery.
     */
    s->regs[0x06] = temperature;
    s->regs[0x07] = temperature >> 8;
    s->regs[0x08] = voltage;
    s->regs[0x09] = voltage >> 8;
    s->regs[0x0a] = 0x80; /* Positive-current sign; current itself is zero. */
    s->regs[0x0b] = 80;
    s->regs[0x0c] = capacity_raw;
    s->regs[0x0d] = capacity_raw >> 8;
    s->regs[0x0e] = learned_capacity_raw;
    s->regs[0x0f] = learned_capacity_raw >> 8;
    s->regs[0x10] = capacity_raw;
    s->regs[0x11] = capacity_raw >> 8;
    s->regs[0x2c] = 80;
    /* Production board families validate different one-wire resistor IDs. */
    s->regs[0x7e] = s->celeste ? 10 : (s->whitney ? 8 : 12);
}

static int yoshi_battery_event(I2CSlave *i2c, enum i2c_event event)
{
    YoshiBatteryState *s = YOSHI_BATTERY(i2c);

    if (event == I2C_START_SEND) {
        s->pointer_valid = false;
    }
    return 0;
}

static int yoshi_battery_send(I2CSlave *i2c, uint8_t data)
{
    YoshiBatteryState *s = YOSHI_BATTERY(i2c);

    if (!s->pointer_valid) {
        s->pointer = data;
        s->pointer_valid = true;
    } else {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t yoshi_battery_recv(I2CSlave *i2c)
{
    YoshiBatteryState *s = YOSHI_BATTERY(i2c);

    return s->regs[s->pointer++];
}

static const VMStateDescription yoshi_battery_vmstate = {
    .name = TYPE_YOSHI_BATTERY,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, YoshiBatteryState, 256),
        VMSTATE_UINT8(pointer, YoshiBatteryState),
        VMSTATE_BOOL(pointer_valid, YoshiBatteryState),
        VMSTATE_I2C_SLAVE(parent_obj, YoshiBatteryState),
        VMSTATE_END_OF_LIST()
    },
};

static void yoshi_battery_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, yoshi_battery_reset);
    dc->vmsd = &yoshi_battery_vmstate;
    sc->event = yoshi_battery_event;
    sc->send = yoshi_battery_send;
    sc->recv = yoshi_battery_recv;
}

static const TypeInfo yoshi_battery_type = {
    .name = TYPE_YOSHI_BATTERY,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(YoshiBatteryState),
    .class_init = yoshi_battery_class_init,
};

static void whitney_battery_init(Object *obj)
{
    YoshiBatteryState *s = YOSHI_BATTERY(obj);

    s->whitney = true;
}

static const TypeInfo whitney_battery_type = {
    .name = TYPE_WHITNEY_BATTERY,
    .parent = TYPE_YOSHI_BATTERY,
    .instance_init = whitney_battery_init,
};

static void celeste_battery_init(Object *obj)
{
    YoshiBatteryState *s = YOSHI_BATTERY(obj);

    s->celeste = true;
}

static const TypeInfo celeste_battery_type = {
    .name = TYPE_CELESTE_BATTERY,
    .parent = TYPE_YOSHI_BATTERY,
    .instance_init = celeste_battery_init,
};

static void yoshi_papyrus_reset(DeviceState *dev)
{
    YoshiPapyrusState *s = YOSHI_PAPYRUS(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x00] = 25; /* External temperature, degrees Celsius. */
    s->regs[0x05] = 25;
    s->regs[0x07] = 0x45; /* TPS65180 product ID. */
    s->regs[0x10] = 0x01;
    s->regs[0x0c] = 0x20;
    s->regs[0x0d] = 0x20;
    s->pointer = 0;
    s->pointer_valid = false;
    qemu_set_irq(s->pwrgood, 0);
}

static int yoshi_papyrus_event(I2CSlave *i2c, enum i2c_event event)
{
    YoshiPapyrusState *s = YOSHI_PAPYRUS(i2c);

    if (event == I2C_START_SEND) {
        s->pointer_valid = false;
    }
    return 0;
}

static int yoshi_papyrus_send(I2CSlave *i2c, uint8_t data)
{
    YoshiPapyrusState *s = YOSHI_PAPYRUS(i2c);

    if (!s->pointer_valid) {
        s->pointer = data;
        s->pointer_valid = true;
        return 0;
    }

    s->regs[s->pointer] = data;
    if (s->pointer == 0x01) {
        bool enabled = data & 0x80;

        /* TPS65185/Papyrus V2 reports all expected rails as 0xfa. */
        s->regs[0x0f] = enabled ? 0xfa : 0;
        qemu_set_irq(s->pwrgood, enabled);
    } else if ((s->pointer == 0x0c || s->pointer == 0x0d) &&
               (data & 0x80)) {
        /* Support both Papyrus V1 (0x0c) and Celeste V2 (0x0d). */
        s->regs[s->pointer] |= 0x20;
    }
    s->pointer++;
    return 0;
}

static uint8_t yoshi_papyrus_recv(I2CSlave *i2c)
{
    YoshiPapyrusState *s = YOSHI_PAPYRUS(i2c);

    return s->regs[s->pointer++];
}

static const VMStateDescription yoshi_papyrus_vmstate = {
    .name = TYPE_YOSHI_PAPYRUS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, YoshiPapyrusState, 256),
        VMSTATE_UINT8(pointer, YoshiPapyrusState),
        VMSTATE_BOOL(pointer_valid, YoshiPapyrusState),
        VMSTATE_I2C_SLAVE(parent_obj, YoshiPapyrusState),
        VMSTATE_END_OF_LIST()
    },
};

static void yoshi_papyrus_realize(DeviceState *dev, Error **errp)
{
    YoshiPapyrusState *s = YOSHI_PAPYRUS(dev);

    qdev_init_gpio_out_named(dev, &s->pwrgood, "pwrgood", 1);
}

static void yoshi_papyrus_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = yoshi_papyrus_realize;
    device_class_set_legacy_reset(dc, yoshi_papyrus_reset);
    dc->vmsd = &yoshi_papyrus_vmstate;
    sc->event = yoshi_papyrus_event;
    sc->send = yoshi_papyrus_send;
    sc->recv = yoshi_papyrus_recv;
}

static const TypeInfo yoshi_papyrus_type = {
    .name = TYPE_YOSHI_PAPYRUS,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(YoshiPapyrusState),
    .class_init = yoshi_papyrus_class_init,
};

static void whitney_mma8453_reset(DeviceState *dev)
{
    WhitneyMMA8453State *s = WHITNEY_MMA8453(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x00] = 0x08; /* XYZ data ready. */
    s->regs[0x06] = 0x40; /* Stationary: +1 g on Z in 2 g mode. */
    s->regs[0x0d] = 0x3a; /* WHO_AM_I. */
    s->pointer = 0;
    s->pointer_valid = false;

    /* Lab126 configures both MMA8453 interrupt outputs active-low. */
    qemu_set_irq(s->irq[0], 1);
    qemu_set_irq(s->irq[1], 1);
}

static int whitney_mma8453_event(I2CSlave *i2c, enum i2c_event event)
{
    WhitneyMMA8453State *s = WHITNEY_MMA8453(i2c);

    if (event == I2C_START_SEND) {
        s->pointer_valid = false;
    }
    return 0;
}

static int whitney_mma8453_send(I2CSlave *i2c, uint8_t data)
{
    WhitneyMMA8453State *s = WHITNEY_MMA8453(i2c);

    if (!s->pointer_valid) {
        s->pointer = data;
        s->pointer_valid = true;
    } else if (s->pointer < sizeof(s->regs)) {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t whitney_mma8453_recv(I2CSlave *i2c)
{
    WhitneyMMA8453State *s = WHITNEY_MMA8453(i2c);

    if (s->pointer >= sizeof(s->regs)) {
        return 0;
    }
    return s->regs[s->pointer++];
}

static const VMStateDescription whitney_mma8453_vmstate = {
    .name = TYPE_WHITNEY_MMA8453,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, WhitneyMMA8453State, 0x32),
        VMSTATE_UINT8(pointer, WhitneyMMA8453State),
        VMSTATE_BOOL(pointer_valid, WhitneyMMA8453State),
        VMSTATE_I2C_SLAVE(parent_obj, WhitneyMMA8453State),
        VMSTATE_END_OF_LIST()
    },
};

static void whitney_mma8453_realize(DeviceState *dev, Error **errp)
{
    WhitneyMMA8453State *s = WHITNEY_MMA8453(dev);

    qdev_init_gpio_out(dev, s->irq, ARRAY_SIZE(s->irq));
}

static void whitney_mma8453_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = whitney_mma8453_realize;
    device_class_set_legacy_reset(dc, whitney_mma8453_reset);
    dc->vmsd = &whitney_mma8453_vmstate;
    sc->event = whitney_mma8453_event;
    sc->send = whitney_mma8453_send;
    sc->recv = whitney_mma8453_recv;
}

static const TypeInfo whitney_mma8453_type = {
    .name = TYPE_WHITNEY_MMA8453,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(WhitneyMMA8453State),
    .class_init = whitney_mma8453_class_init,
};

static void yoshi_populate_idme(DeviceState *card,
                                  YoshiMachineState *tms)
{
    YoshiIdmeField fields[] = {
        { "serial", tms->idme_serial, 0x0000, 16 },
        { "accel", tms->idme_accel, 0x0020, 16 },
        { "mac", tms->idme_mac, 0x0030, 12 },
        { "sec", tms->idme_sec, 0x0040, 20 },
        { "pcbsn", tms->idme_pcbsn, 0x0060, 16 },
        { "bootmode", tms->idme_bootmode, 0x1000, 16 },
        { "postmode", tms->idme_postmode, 0x1010, 16 },
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(fields); i++) {
        YoshiIdmeField *field = &fields[i];
        g_autofree uint8_t *contents = g_malloc0(field->size);
        size_t len = strlen(field->value);

        if (!g_str_is_ascii(field->value) || len > field->size) {
            error_report("IDME %s must contain at most %zu ASCII bytes",
                         field->name, field->size);
            exit(EXIT_FAILURE);
        }
        memcpy(contents, field->value, len);
        emmc_boot_partition_write(card, 1,
                                  YOSHI_IDME_BASE + field->offset,
                                  contents, field->size, &error_fatal);
    }
}

static void yoshi_firmware_reset(void *opaque)
{
    YoshiMachineState *tms = opaque;
    ARMCPU *cpu = &tms->soc->cpu;

    cpu_reset(CPU(cpu));
    if (yoshi_binfo.entry == YOSHI_RAM_BASE) {
        cpu->env.regs[13] = YOSHI_IRAM_TOP;
    }
    cpu_set_pc(CPU(cpu), yoshi_binfo.entry);
}

static void yoshi_load_firmware(MachineState *machine,
                                YoshiMachineState *tms)
{
    hwaddr addr = tms->whitney ? WHITNEY_KOBO_UBOOT_ADDR : YOSHI_UBOOT_ADDR;
    hwaddr entry = tms->whitney ? WHITNEY_KOBO_UBOOT_ENTRY : YOSHI_UBOOT_ADDR;
    size_t max_size = tms->whitney ? WHITNEY_KOBO_UBOOT_MAX : YOSHI_UBOOT_MAX;
    ssize_t size;

    if (!machine->firmware) {
        return;
    }
    /*
     * Stock Yoshi U-Boot is an OCRAM-linked raw image.  Modern barebox is
     * larger than OCRAM and its i.MX image is position independent, so model
     * the boot ROM's SDRAM load for images that do not fit the stock window.
     */
    if (get_image_size(machine->firmware, NULL) > YOSHI_UBOOT_MAX) {
        addr = YOSHI_RAM_BASE;
        entry = YOSHI_RAM_BASE;
        max_size = machine->ram_size;
    }
    size = load_image_targphys(machine->firmware, addr, max_size, NULL);
    if (size < 0) {
        error_report("Unable to load Yoshi-family firmware '%s'",
                     machine->firmware);
        exit(EXIT_FAILURE);
    }
    if (tms->whitney && machine->kernel_filename) {
        size = load_image_targphys(machine->kernel_filename,
                                   WHITNEY_KOBO_KERNEL_ADDR,
                                   WHITNEY_KOBO_KERNEL_MAX, NULL);
        if (size < 0) {
            error_report("Unable to load Whitney kernel '%s'",
                         machine->kernel_filename);
            exit(EXIT_FAILURE);
        }
    }
    yoshi_binfo.entry = entry;
}

static void yoshi_attach_emmc(FslIMX50State *soc,
                                YoshiMachineState *tms,
                                unsigned controller, unsigned drive)
{
    DriveInfo *di = drive_get(IF_SD, 0, drive);
    BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
    BusState *bus;
    DeviceState *card = qdev_new(TYPE_EMMC);

    bus = qdev_get_child_bus(DEVICE(&soc->esdhc[controller]), "sd-bus");
    if (tms->idme_boot_partitions) {
        /* Reader identity is volatile when no eMMC backend was supplied. */
        qdev_prop_set_uint64(card, "boot-partition-size", 2 * MiB);
        qdev_prop_set_bit(card, "boot-partitions-in-memory", true);
    }
    qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
    qdev_realize(card, bus, &error_fatal);
    if (tms->idme_boot_partitions) {
        yoshi_populate_idme(card, tms);
    }
    object_unref(OBJECT(card));
}

static void yoshi_attach_wifi(FslIMX50State *soc)
{
    BusState *bus;
    DeviceState *wifi;

    /*
     * Tequila and Whitney route AR6003 to eSDHC2.  SD3_WP is repurposed as
     * the module's active-high power control (GPIO5_28).
     */
    bus = qdev_get_child_bus(DEVICE(&soc->esdhc[1]), "sd-bus");
    wifi = qdev_new(TYPE_AR6003_SDIO);
    qemu_configure_nic_device(wifi, true, "ar6003");
    qdev_realize(wifi, bus, &error_fatal);
    qdev_connect_gpio_out(DEVICE(&soc->gpio[4]), 28,
                          qdev_get_gpio_in_named(wifi, "power", 0));
    qdev_connect_gpio_out_named(
        wifi, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&soc->esdhc[1]), "sdio-irq", 0));
    object_unref(OBJECT(wifi));
}

static void yoshi_attach_panel_flash(FslIMX50State *soc)
{
    SSIBus *bus;
    DeviceState *flash;
    DriveInfo *di = drive_get(IF_MTD, 0, 0);
    qemu_irq cs;

    bus = (SSIBus *)qdev_get_child_bus(DEVICE(&soc->spi[1]), "spi");
    flash = qdev_new("mx25l4005a");
    if (di) {
        qdev_prop_set_drive_err(flash, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
    }
    qdev_realize_and_unref(flash, BUS(bus), &error_fatal);

    cs = qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0);
    if (!di) {
        /*
         * An erased panel flash is not a usable physical board state.  Seed
         * the waveform-format byte so the existing board-default panel mode
         * can be selected while leaving the rest of the synthetic flash
         * erased.  A supplied MTD image always takes precedence.
         */
        qemu_set_irq(cs, 1);
        qemu_set_irq(cs, 0);
        ssi_transfer(bus, 0x06); /* write enable */
        qemu_set_irq(cs, 1);
        qemu_set_irq(cs, 0);
        ssi_transfer(bus, 0x02); /* page program */
        ssi_transfer(bus, 0x00);
        ssi_transfer(bus, 0x08);
        ssi_transfer(bus, 0x99);
        ssi_transfer(bus, 0x15); /* WJ waveform layout */
        qemu_set_irq(cs, 1);
    }

    /* CSPI2 SS0 is the first sysbus output following the controller IRQ. */
    sysbus_connect_irq(SYS_BUS_DEVICE(&soc->spi[1]), 1,
                       cs);
}

static DeviceState *whitney_attach_pmic(FslIMX50State *soc)
{
    SSIBus *bus;
    DeviceState *pmic;

    /* Whitney's system PMIC is an MC13892 on CSPI3 SS0. */
    bus = (SSIBus *)qdev_get_child_bus(DEVICE(&soc->spi[2]), "spi");
    pmic = qdev_new(TYPE_MC13892);
    ssi_realize_and_unref(pmic, bus, &error_fatal);

    /* i.MX50 drives native CS low internally; MC13892 CS is active high. */
    sysbus_connect_irq(SYS_BUS_DEVICE(&soc->spi[2]), 1,
        qemu_irq_invert(qdev_get_gpio_in_named(pmic, SSI_GPIO_CS, 0)));
    qdev_connect_gpio_out_named(
        pmic, "irq", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[5]), 8));

    return pmic;
}

static void tequila_attach_keyboard(FslIMX50State *soc, DeviceState *pmic)
{
    DeviceState *keyboard = qdev_new(TYPE_TEQUILA_KEYBOARD);
    static const struct {
        unsigned output;
        unsigned bank;
        unsigned pin;
    } connections[] = {
        { TEQUILA_KEY_FIVEWAY_UP,     0, 20 }, /* EIM_EB1 */
        { TEQUILA_KEY_FIVEWAY_DOWN,   0, 19 }, /* EIM_EB0 */
        { TEQUILA_KEY_FIVEWAY_LEFT,   0, 16 }, /* EIM_CS2 */
        { TEQUILA_KEY_FIVEWAY_RIGHT,  0, 17 }, /* EIM_CS1 */
        { TEQUILA_KEY_FIVEWAY_SELECT, 0, 18 }, /* EIM_CS0 */
        { TEQUILA_KEY_PAGE_PREVIOUS,  3,  5 }, /* KEY_ROW2 */
        { TEQUILA_KEY_PAGE_NEXT,      3,  2 }, /* KEY_COL1, row 5 */
        { TEQUILA_KEY_KEYBOARD,       3,  1 }, /* KEY_ROW0 */
        { TEQUILA_KEY_MENU,           3,  3 }, /* KEY_ROW1 */
        { TEQUILA_KEY_HOME,           3,  4 }, /* KEY_COL2, row 6 */
        { TEQUILA_KEY_BACK,           3,  6 }, /* KEY_COL3, row 7 */
    };
    unsigned i;

    sysbus_realize_and_unref(SYS_BUS_DEVICE(keyboard), &error_fatal);
    for (i = 0; i < ARRAY_SIZE(connections); i++) {
        qdev_connect_gpio_out(
            keyboard, connections[i].output,
            qdev_get_gpio_in(DEVICE(&soc->gpio[connections[i].bank]),
                             connections[i].pin));
    }
    qdev_connect_gpio_out(
        keyboard, TEQUILA_KEY_POWER,
        qdev_get_gpio_in_named(pmic, "power-button", 0));
}

static void whitney_attach_input(FslIMX50State *soc, DeviceState *pmic)
{
    DeviceState *keyboard = qdev_new(TYPE_TEQUILA_KEYBOARD);
    I2CSlave *mma8453;
    I2CSlave *zforce;

    /* Production Whitney: MMA8453 on I2C1, INT1/INT2 on GPIO5_4/GPIO5_0. */
    mma8453 = i2c_slave_create_simple(soc->i2c[0].bus,
                                      TYPE_WHITNEY_MMA8453, 0x1c);
    qdev_connect_gpio_out(DEVICE(mma8453), 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[4]), 4));
    qdev_connect_gpio_out(DEVICE(mma8453), 1,
        qdev_get_gpio_in(DEVICE(&soc->gpio[4]), 0));

    /*
     * Production Whitney revisions put zForce on the dedicated 400 kHz
     * I2C3 bus.  DISP_RS is its active-low data-ready line (GPIO2_17);
     * DISP_BUSY is the controller reset line (GPIO2_18).
     */
    zforce = i2c_slave_create_simple(soc->i2c[2].bus,
                                     TYPE_KINDLE_ZFORCE, 0x50);
    qdev_connect_gpio_out(DEVICE(zforce), 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[1]), 17));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[1]), 18,
        qdev_get_gpio_in(DEVICE(zforce), 0));

    /*
     * Whitney has one physical Home button on EIM_DA0 (GPIO1_0).  Reuse the
     * Yoshi-family host keyboard adapter, but wire only its H-key output.
     */
    sysbus_realize_and_unref(SYS_BUS_DEVICE(keyboard), &error_fatal);
    qdev_connect_gpio_out(
        keyboard, TEQUILA_KEY_HOME,
        qemu_irq_invert(qdev_get_gpio_in(DEVICE(&soc->gpio[0]), 0)));
    qdev_connect_gpio_out(
        keyboard, TEQUILA_KEY_POWER,
        qdev_get_gpio_in_named(pmic, "power-button", 0));
}

static void celeste_attach_input(FslIMX50State *soc, DeviceState *pmic)
{
    DeviceState *keyboard = qdev_new(TYPE_TEQUILA_KEYBOARD);
    I2CSlave *cyttsp;

    /*
     * Production Celeste revisions use a Cypress Gen3 controller on the
     * 400 kHz I2C3 bus.  KEY_COL1 (GPIO4_2) is the active-low interrupt and
     * KEY_COL0 (GPIO4_0) drives the hardware reset sequence.
     */
    cyttsp = i2c_slave_create_simple(soc->i2c[2].bus, TYPE_CYTTSP, 0x24);
    qdev_connect_gpio_out(DEVICE(cyttsp), 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 2));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[3]), 0,
        qdev_get_gpio_in(DEVICE(cyttsp), 0));

    /* SD2_WP (GPIO5_16) is high while the magnetic cover is open. */
    qemu_set_irq(qdev_get_gpio_in(DEVICE(&soc->gpio[4]), 16), 1);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(keyboard), &error_fatal);
    qdev_connect_gpio_out(
        keyboard, TEQUILA_KEY_POWER,
        qdev_get_gpio_in_named(pmic, "power-button", 0));
}

static void yoshi_init(MachineState *machine)
{
    YoshiMachineState *tms = YOSHI_MACHINE(machine);
    FslIMX50State *soc;
    DeviceState *pmic;
    I2CSlave *papyrus;

    if (machine->ram_size > YOSHI_RAM_MAX) {
        error_report("RAM size exceeds Yoshi's 512 MiB addressable SDRAM");
        exit(EXIT_FAILURE);
    }
    soc = FSL_IMX50(object_new(TYPE_FSL_IMX50));
    tms->soc = soc;
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    if (tms->whitney) {
        /* Whitney's 256 MiB mobile-DDR is reported in DATAbahn CTL0. */
        object_property_set_uint(OBJECT(soc), "ddr-type", 0x100,
                                 &error_fatal);
    }
    if (tms->whitney || tms->celeste) {
        /* Whitney and Celeste mount the landscape panel scan clockwise. */
        object_property_set_bool(OBJECT(&soc->epdc), "rotate-ccw", false,
                                 &error_fatal);
    }
    if (tms->celeste) {
        /*
         * Celeste's stock stack keeps a portrait 8-bit shadow framebuffer;
         * use it for host presentation while EPDC still owns update timing.
         */
        object_property_set_bool(OBJECT(&soc->epdc), "direct-framebuffer",
                                 true, &error_fatal);
        object_property_set_uint(OBJECT(&soc->epdc), "framebuffer-address",
                                 0x75800000, &error_fatal);
        object_property_set_uint(OBJECT(&soc->epdc), "framebuffer-stride",
                                 768, &error_fatal);
        object_property_set_uint(OBJECT(&soc->epdc), "framebuffer-width",
                                 758, &error_fatal);
        object_property_set_uint(OBJECT(&soc->epdc), "framebuffer-height",
                                 1024, &error_fatal);
        DEVICE(&soc->epdc)->id = g_strdup("celeste-epdc");
    }
    qdev_realize(DEVICE(soc), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), YOSHI_RAM_BASE,
                                machine->ram);

    i2c_slave_create_simple(
        soc->i2c[1].bus,
        tms->celeste ? TYPE_CELESTE_BATTERY :
        (tms->whitney ? TYPE_WHITNEY_BATTERY : TYPE_YOSHI_BATTERY),
        0x55);
    papyrus = i2c_slave_create_simple(soc->i2c[1].bus,
                                      TYPE_YOSHI_PAPYRUS,
                                      tms->celeste ? 0x68 : 0x48);
    qdev_connect_gpio_out_named(DEVICE(papyrus), "pwrgood", 0,
                                qdev_get_gpio_in(DEVICE(&soc->gpio[2]), 28));

    /* The onboard flash is the first registered device, on eSDHC3. */
    yoshi_attach_emmc(soc, tms, 2, 0);
    yoshi_attach_wifi(soc);
    yoshi_attach_panel_flash(soc);
    /* The production Yoshi-family boards use MC13892 on CSPI3. */
    pmic = whitney_attach_pmic(soc);
    if (tms->whitney) {
        whitney_attach_input(soc, pmic);
    } else if (tms->celeste) {
        celeste_attach_input(soc, pmic);
    } else {
        tequila_attach_keyboard(soc, pmic);
    }
    yoshi_binfo = (struct arm_boot_info) {
        .loader_start = YOSHI_RAM_BASE,
        .ram_size = machine->ram_size,
        .board_id = YOSHI_MACHINE_ID,
    };
    yoshi_load_firmware(machine, tms);
    if (machine->firmware) {
        qemu_register_reset(yoshi_firmware_reset, tms);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&soc->cpu, machine, &yoshi_binfo);
    }
    if (tms->whitney) {
        tms->diagnostic_timer = timer_new_ns(
            QEMU_CLOCK_REALTIME, whitney_diagnostic_heartbeat, tms);
        timer_mod(tms->diagnostic_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 250 * SCALE_MS);
    }
}

static char *yoshi_idme_get(char **value)
{
    return g_strdup(*value);
}

static void yoshi_idme_set(char **field, const char *value)
{
    g_free(*field);
    *field = g_strdup(value);
}

static bool yoshi_get_idme_boot_partitions(Object *obj, Error **errp)
{
    return YOSHI_MACHINE(obj)->idme_boot_partitions;
}

static void yoshi_set_idme_boot_partitions(Object *obj, bool value,
                                           Error **errp)
{
    YOSHI_MACHINE(obj)->idme_boot_partitions = value;
}

#define YOSHI_IDME_PROPERTY(_member)                                    \
    static char *yoshi_get_##_member(Object *obj, Error **errp)         \
    {                                                                   \
        return yoshi_idme_get(&YOSHI_MACHINE(obj)->_member);            \
    }                                                                   \
    static void yoshi_set_##_member(Object *obj, const char *value,     \
                                    Error **errp)                        \
    {                                                                   \
        yoshi_idme_set(&YOSHI_MACHINE(obj)->_member, value);            \
    }

YOSHI_IDME_PROPERTY(idme_serial)
YOSHI_IDME_PROPERTY(idme_accel)
YOSHI_IDME_PROPERTY(idme_mac)
YOSHI_IDME_PROPERTY(idme_sec)
YOSHI_IDME_PROPERTY(idme_pcbsn)
YOSHI_IDME_PROPERTY(idme_bootmode)
YOSHI_IDME_PROPERTY(idme_postmode)

static void yoshi_machine_instance_init(Object *obj)
{
    YoshiMachineState *tms = YOSHI_MACHINE(obj);

    tms->idme_serial = g_strdup("");
    tms->idme_accel = g_strdup("");
    tms->idme_mac = g_strdup("");
    tms->idme_sec = g_strdup("");
    tms->idme_pcbsn = g_strdup("");
    tms->idme_bootmode = g_strdup("main");
    tms->idme_postmode = g_strdup("normal");
    tms->idme_boot_partitions = true;
}

static void tequila_machine_instance_init(Object *obj)
{
    YoshiMachineState *tms = YOSHI_MACHINE(obj);

    yoshi_idme_set(&tms->idme_serial, "B00E000000000001");
    yoshi_idme_set(&tms->idme_mac, "020000000001");
    yoshi_idme_set(&tms->idme_sec, "00000000000000000000");
    yoshi_idme_set(&tms->idme_pcbsn, "0031500000000001");
}

static void whitney_machine_instance_init(Object *obj)
{
    YoshiMachineState *tms = YOSHI_MACHINE(obj);

    tms->whitney = true;
    yoshi_idme_set(&tms->idme_serial, "B011000000000001");
    yoshi_idme_set(&tms->idme_accel, "0");
    yoshi_idme_set(&tms->idme_mac, "020000000003");
    yoshi_idme_set(&tms->idme_sec, "00000000000000000000");
    /*
     * B011 is a Wi-Fi-only Kindle Touch.  Its production board family is
     * Whitney WFO EVT3 (00606); 005xx denotes cellular Whitney and makes
     * userspace repeatedly probe Host 1 for a WAN modem.
     */
    yoshi_idme_set(&tms->idme_pcbsn, "0060600000000001");
}

static void celeste_machine_instance_init(Object *obj)
{
    YoshiMachineState *tms = YOSHI_MACHINE(obj);

    tms->celeste = true;
    yoshi_idme_set(&tms->idme_serial, "B024000000000001");
    yoshi_idme_set(&tms->idme_accel, "0");
    yoshi_idme_set(&tms->idme_mac, "020000000004");
    yoshi_idme_set(&tms->idme_sec, "00000000000000000000");
    /* 00A10 is the production Celeste WFO 256 MiB EVT3 board family. */
    yoshi_idme_set(&tms->idme_pcbsn, "00A1000000000001");
}

static void yoshi_machine_instance_finalize(Object *obj)
{
    YoshiMachineState *tms = YOSHI_MACHINE(obj);

    timer_free(tms->diagnostic_timer);
    g_free(tms->idme_serial);
    g_free(tms->idme_accel);
    g_free(tms->idme_mac);
    g_free(tms->idme_sec);
    g_free(tms->idme_pcbsn);
    g_free(tms->idme_bootmode);
    g_free(tms->idme_postmode);
}

static void yoshi_machine_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->init = yoshi_init;
    mc->default_ram_size = 256 * MiB;
    mc->auto_create_sdcard = true;
    mc->ignore_memory_transaction_failures = true;

    object_class_property_add_str(oc, "idme-serial",
                                  yoshi_get_idme_serial,
                                  yoshi_set_idme_serial);
    object_class_property_add_str(oc, "idme-accel",
                                  yoshi_get_idme_accel,
                                  yoshi_set_idme_accel);
    object_class_property_add_str(oc, "idme-mac", yoshi_get_idme_mac,
                                  yoshi_set_idme_mac);
    object_class_property_add_str(oc, "idme-sec", yoshi_get_idme_sec,
                                  yoshi_set_idme_sec);
    object_class_property_add_str(oc, "idme-pcbsn",
                                  yoshi_get_idme_pcbsn,
                                  yoshi_set_idme_pcbsn);
    object_class_property_add_str(oc, "idme-bootmode",
                                  yoshi_get_idme_bootmode,
                                  yoshi_set_idme_bootmode);
    object_class_property_add_str(oc, "idme-postmode",
                                  yoshi_get_idme_postmode,
                                  yoshi_set_idme_postmode);
    object_class_property_add_bool(oc, "idme-boot-partitions",
                                   yoshi_get_idme_boot_partitions,
                                   yoshi_set_idme_boot_partitions);
}

static const TypeInfo yoshi_machine_type = {
    .name = TYPE_YOSHI_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(YoshiMachineState),
    .instance_init = yoshi_machine_instance_init,
    .instance_finalize = yoshi_machine_instance_finalize,
    .class_init = yoshi_machine_init,
    .interfaces = arm_machine_interfaces,
    .abstract = true,
};

static void tequila_machine_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Amazon Kindle 4 Tequila (i.MX508)";
    mc->default_ram_id = "tequila.ram";
    mc->default_nic = TYPE_AR6003_SDIO;
}

static const TypeInfo tequila_machine_type = {
    .name = TYPE_TEQUILA_MACHINE,
    .parent = TYPE_YOSHI_MACHINE,
    .instance_init = tequila_machine_instance_init,
    .class_init = tequila_machine_init,
};

static void whitney_machine_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Amazon Kindle Touch Whitney (i.MX508)";
    mc->default_ram_id = "whitney.ram";
}

static const TypeInfo whitney_machine_type = {
    .name = TYPE_WHITNEY_MACHINE,
    .parent = TYPE_YOSHI_MACHINE,
    .instance_init = whitney_machine_instance_init,
    .class_init = whitney_machine_init,
};

static void celeste_machine_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Amazon Kindle Paperwhite Celeste (i.MX508)";
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "celeste.ram";
}

static const TypeInfo celeste_machine_type = {
    .name = TYPE_CELESTE_MACHINE,
    .parent = TYPE_YOSHI_MACHINE,
    .instance_init = celeste_machine_instance_init,
    .class_init = celeste_machine_init,
};

static void yoshi_machine_register_types(void)
{
    type_register_static(&yoshi_battery_type);
    type_register_static(&whitney_battery_type);
    type_register_static(&celeste_battery_type);
    type_register_static(&yoshi_papyrus_type);
    type_register_static(&whitney_mma8453_type);
    type_register_static(&yoshi_machine_type);
    type_register_static(&tequila_machine_type);
    type_register_static(&whitney_machine_type);
    type_register_static(&celeste_machine_type);
}

type_init(yoshi_machine_register_types)
