/*
 * PocketBook i.MX50 boards.
 *
 * Board wiring follows the matching pocketbook-free kernel/U-Boot trees;
 * the Netronix 512/611 wiring is recovered from their original kernels and BSP.
 * The updater's U-Boot payload begins at the first IVT, omitting the 1 KiB
 * prefix from the on-card ROM image. Enter the main image after its DDR plug-in.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/arm/fsl-imx50.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/split-irq.h"
#include "hw/i2c/cy8c-slider.h"
#include "hw/i2c/elan-ktf2k.h"
#include "hw/i2c/fp9928.h"
#include "hw/i2c/goodix_gt813.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/mma7660.h"
#include "hw/i2c/zforce.h"
#include "hw/sd/sd.h"
#include "hw/sensor/tmp105.h"
#include "hw/ssi/mc13892.h"
#include "hw/ssi/ssi.h"
#include "hw/usb/imx-usb-misc.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/qtest.h"
#include "system/reset.h"
#include "ui/input.h"

#define PB_RAM_BASE       0x70000000
#define PB_RAM_SIZE       (128 * MiB)
#define PB_MACHINE_ID     2988 /* MACH_TYPE_MX50_RDP */
#define PB_IVT_HEADER     0x402000d1

#define TYPE_PB_MACHINE MACHINE_TYPE_NAME("imx50-pocketbook")
OBJECT_DECLARE_TYPE(PBMachineState, PBMachineClass, PB_MACHINE)

typedef struct PBKeyPin {
    unsigned button;
    unsigned gpio;
    unsigned pin;
} PBKeyPin;

typedef struct PBProfile {
    const char *description;
    const char *ram_id;
    const PBKeyPin *keys;
    unsigned num_keys;
    uint64_t ram_size;
    uint32_t hwid_straps;
    uint8_t matrix_contacts[7];
    bool rotate_ccw;
    bool touch;
    bool wifi;
    bool frontlight;
    bool color;
    bool netronix;
    bool accelerometer;
} PBProfile;

struct PBMachineClass {
    MachineClass parent_class;
    const PBProfile *profile;
};

struct PBMachineState {
    MachineState parent_obj;
    FslIMX50State *soc;
    struct arm_boot_info boot;
};

#define TYPE_PB_BATTERY "pocketbook-max17043"
OBJECT_DECLARE_SIMPLE_TYPE(PBBatteryState, PB_BATTERY)

struct PBBatteryState {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer;
    bool expect_pointer;
};

static int pb_battery_send(I2CSlave *i2c, uint8_t data)
{
    PBBatteryState *s = PB_BATTERY(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else {
        s->regs[s->pointer++] = data;
    }
    return 0;
}

static uint8_t pb_battery_recv(I2CSlave *i2c)
{
    PBBatteryState *s = PB_BATTERY(i2c);

    return s->regs[s->pointer++];
}

static int pb_battery_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        PB_BATTERY(i2c)->expect_pointer = true;
    }
    return 0;
}

static void pb_battery_reset(DeviceState *dev)
{
    PBBatteryState *s = PB_BATTERY(dev);

    memset(s->regs, 0, sizeof(s->regs));
    /* VCELL has 1.25 mV steps in bits 15:4; SOC is an 8.8 percentage. */
    s->regs[2] = 0xc8; /* 4.0 V */
    s->regs[4] = 85;
    s->regs[9] = 3;    /* MAX17043 version */
    s->pointer = 0;
    s->expect_pointer = true;
}

static const VMStateDescription pb_battery_vmstate = {
    .name = TYPE_PB_BATTERY,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, PBBatteryState),
        VMSTATE_UINT8_ARRAY(regs, PBBatteryState, 256),
        VMSTATE_UINT8(pointer, PBBatteryState),
        VMSTATE_BOOL(expect_pointer, PBBatteryState),
        VMSTATE_END_OF_LIST()
    },
};

static void pb_battery_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->vmsd = &pb_battery_vmstate;
    device_class_set_legacy_reset(dc, pb_battery_reset);
    sc->send = pb_battery_send;
    sc->recv = pb_battery_recv;
    sc->event = pb_battery_event;
}

#define TYPE_PB_BUTTONS "pocketbook-buttons"
OBJECT_DECLARE_SIMPLE_TYPE(PBButtonsState, PB_BUTTONS)

struct PBButtonsState {
    DeviceState parent_obj;
    QemuInputHandlerState *input;
    DeviceState *slider;
    qemu_irq output[8];
    uint8_t pressed;
    bool touch;
    bool color;
};

static void pb_buttons_update(PBButtonsState *s)
{
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(s->output); i++) {
        qemu_set_irq(s->output[i], !!(s->pressed & BIT(i)));
    }
}

static void pb_buttons_event(DeviceState *dev, QemuConsole *src,
                             InputEvent *evt)
{
    PBButtonsState *s = PB_BUTTONS(dev);
    InputKeyEvent *key = evt->u.key.data;
    QKeyCode code = qemu_input_key_value_to_qcode(key->key);
    unsigned button;

    if (s->slider && (code == Q_KEY_CODE_BRACKET_LEFT ||
                      code == Q_KEY_CODE_BRACKET_RIGHT)) {
        if (key->down) {
            cy8c_slider_step(s->slider,
                             code == Q_KEY_CODE_BRACKET_RIGHT ? 1 : -1);
        }
        return;
    }
    if (s->touch) {
        switch (code) {
        case Q_KEY_CODE_HOME: button = 0; break;
        case Q_KEY_CODE_M: button = 1; break;
        case Q_KEY_CODE_ESC: button = s->color ? 0 : 1; break;
        case Q_KEY_CODE_UP:
            if (!s->color) {
                return;
            }
            button = 5;
            break;
        case Q_KEY_CODE_DOWN:
            if (!s->color) {
                return;
            }
            button = 6;
            break;
        case Q_KEY_CODE_PGUP: button = 5; break;
        case Q_KEY_CODE_PGDN: button = 6; break;
        case Q_KEY_CODE_POWER:
        case Q_KEY_CODE_P: button = 7; break;
        default: return;
        }
    } else {
        switch (code) {
        case Q_KEY_CODE_RET:  button = 0; break;
        case Q_KEY_CODE_LEFT: button = 1; break;
        case Q_KEY_CODE_RIGHT: button = 2; break;
        case Q_KEY_CODE_DOWN: button = 3; break;
        case Q_KEY_CODE_UP:   button = 4; break;
        case Q_KEY_CODE_PGUP: button = 5; break;
        case Q_KEY_CODE_PGDN: button = 6; break;
        case Q_KEY_CODE_POWER:
        case Q_KEY_CODE_P:    button = 7; break;
        default: return;
        }
    }
    s->pressed = deposit32(s->pressed, button, 1, key->down);
    pb_buttons_update(s);
}

static const QemuInputHandler pb_buttons_handler = {
    .name = "PocketBook buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = pb_buttons_event,
};

static void pb_buttons_reset(DeviceState *dev)
{
    PBButtonsState *s = PB_BUTTONS(dev);

    s->pressed = 0;
    pb_buttons_update(s);
}

static int pb_buttons_post_load(void *opaque, int version_id)
{
    pb_buttons_update(opaque);
    return 0;
}

static const VMStateDescription pb_buttons_vmstate = {
    .name = TYPE_PB_BUTTONS,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = pb_buttons_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(pressed, PBButtonsState),
        VMSTATE_END_OF_LIST()
    },
};

static void pb_buttons_realize(DeviceState *dev, Error **errp)
{
    PB_BUTTONS(dev)->input = qemu_input_handler_register(
        dev, &pb_buttons_handler);
}

static void pb_buttons_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(PB_BUTTONS(dev)->input);
}

static void pb_buttons_init(Object *obj)
{
    PBButtonsState *s = PB_BUTTONS(obj);

    qdev_init_gpio_out(DEVICE(obj), s->output, ARRAY_SIZE(s->output));
}

static void pb_buttons_class_init(ObjectClass *oc, const void *data)
{
    static const Property properties[] = {
        DEFINE_PROP_BOOL("touch", PBButtonsState, touch, false),
        DEFINE_PROP_BOOL("color", PBButtonsState, color, false),
        DEFINE_PROP_LINK("slider", PBButtonsState, slider, TYPE_CY8C_SLIDER,
                         DeviceState *),
    };
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = pb_buttons_realize;
    dc->unrealize = pb_buttons_unrealize;
    dc->vmsd = &pb_buttons_vmstate;
    device_class_set_props(dc, properties);
    device_class_set_legacy_reset(dc, pb_buttons_reset);
}

/* EP7A uses PWM1/GPIO6_9; Color Lux uses inverted PWM2/GPIO2_4. */
#define TYPE_PB_FRONTLIGHT "pocketbook-pwm-frontlight"
OBJECT_DECLARE_SIMPLE_TYPE(PBFrontlightState, PB_FRONTLIGHT)

struct PBFrontlightState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[6];
    uint32_t duty_percent;
    bool powered;
    bool inverted;
};

static void pb_frontlight_update(PBFrontlightState *s)
{
    unsigned period = s->regs[4] + 2;

    if (s->powered && (s->regs[0] & BIT(0)) &&
        (s->regs[0] & (3 << 16))) {
        unsigned duty = MIN(100, s->regs[3] * 100 / period);

        s->duty_percent = s->inverted ? 100 - duty : duty;
    } else {
        s->duty_percent = 0;
    }
}

static uint64_t pb_frontlight_read(void *opaque, hwaddr offset, unsigned size)
{
    PBFrontlightState *s = opaque;

    return offset < sizeof(s->regs) ? s->regs[offset / 4] : 0;
}

static void pb_frontlight_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    PBFrontlightState *s = opaque;

    if (offset == 0 && (value & BIT(3))) { /* PWMCR.SWR self-clears. */
        memset(s->regs, 0, sizeof(s->regs));
        s->regs[4] = 0xfffe;
    } else if (offset == 0 || offset == 8) {
        s->regs[offset / 4] = value;
    } else if (offset == 0xc || offset == 0x10) {
        s->regs[offset / 4] = value & 0xffff;
    }
    pb_frontlight_update(s);
}

static void pb_frontlight_power(void *opaque, int line, int level)
{
    PBFrontlightState *s = opaque;

    s->powered = level;
    pb_frontlight_update(s);
}

static void pb_frontlight_reset(DeviceState *dev)
{
    PBFrontlightState *s = PB_FRONTLIGHT(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[4] = 0xfffe;
    s->powered = false;
    pb_frontlight_update(s);
}

static int pb_frontlight_post_load(void *opaque, int version_id)
{
    pb_frontlight_update(opaque);
    return 0;
}

static const VMStateDescription pb_frontlight_vmstate = {
    .name = TYPE_PB_FRONTLIGHT,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = pb_frontlight_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, PBFrontlightState, 6),
        VMSTATE_BOOL(powered, PBFrontlightState),
        VMSTATE_END_OF_LIST()
    },
};

static const MemoryRegionOps pb_frontlight_ops = {
    .read = pb_frontlight_read,
    .write = pb_frontlight_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void pb_frontlight_init(Object *obj)
{
    PBFrontlightState *s = PB_FRONTLIGHT(obj);

    memory_region_init_io(&s->iomem, obj, &pb_frontlight_ops, s,
                          TYPE_PB_FRONTLIGHT, 0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_in_named(DEVICE(obj), pb_frontlight_power, "power", 1);
    object_property_add_uint32_ptr(obj, "duty-percent", &s->duty_percent,
                                   OBJ_PROP_FLAG_READ);
}

static void pb_frontlight_class_init(ObjectClass *oc, const void *data)
{
    static const Property properties[] = {
        DEFINE_PROP_BOOL("inverted", PBFrontlightState, inverted, false),
    };
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->vmsd = &pb_frontlight_vmstate;
    device_class_set_props(dc, properties);
    device_class_set_legacy_reset(dc, pb_frontlight_reset);
}

static size_t pb_boot_data(const uint8_t *image, size_t size, size_t ivt)
{
    uint32_t self, ptr;

    if (ivt > size || size - ivt < 0x20 ||
        ldl_le_p(image + ivt) != PB_IVT_HEADER) {
        error_report("PocketBook firmware has an invalid IVT");
        exit(EXIT_FAILURE);
    }
    self = ldl_le_p(image + ivt + 0x14);
    ptr = ldl_le_p(image + ivt + 0x10);
    if (ptr < self || ptr - self > size - ivt - 12) {
        error_report("PocketBook firmware boot data is outside the image");
        exit(EXIT_FAILURE);
    }
    return ivt + ptr - self;
}

static void pb_load_firmware(PBMachineState *s)
{
    MachineState *machine = MACHINE(s);
    g_autofree uint8_t *image = NULL;
    gsize size;
    size_t ivt = 0, data;
    uint32_t self, entry, start, length;
    hwaddr load, ram_end = PB_RAM_BASE + machine->ram_size;

    if (!g_file_get_contents(machine->firmware, (char **)&image, &size, NULL)) {
        error_report("Unable to read PocketBook firmware '%s'",
                     machine->firmware);
        exit(EXIT_FAILURE);
    }
    data = pb_boot_data(image, size, ivt);
    if (ldl_le_p(image + data + 8) == 1) {
        /* The ROM DDR plug-in precedes the main IVT. QEMU provides DDR. */
        ivt = data + 12;
        data = pb_boot_data(image, size, ivt);
    }
    self = ldl_le_p(image + ivt + 0x14);
    entry = ldl_le_p(image + ivt + 4);
    start = ldl_le_p(image + data);
    length = ldl_le_p(image + data + 4);
    load = self - ivt;
    if (ldl_le_p(image + data + 8) != 0 || self < ivt ||
        load < PB_RAM_BASE || load >= ram_end || size > ram_end - load ||
        start > load || !length || start < PB_RAM_BASE ||
        length > ram_end - start || entry < load || entry - load >= size) {
        error_report("PocketBook main IVT does not describe an SDRAM image");
        exit(EXIT_FAILURE);
    }
    if (load_image_targphys(machine->firmware, load, ram_end - load, NULL) < 0) {
        error_report("Unable to load PocketBook firmware '%s'",
                     machine->firmware);
        exit(EXIT_FAILURE);
    }
    s->boot.entry = entry;
}

static void pb_firmware_reset(void *opaque)
{
    PBMachineState *s = opaque;
    ARMCPU *cpu = &s->soc->cpu;

    cpu_reset(CPU(cpu));
    cpu->env.regs[13] = FSL_IMX50_IRAM_ADDR + FSL_IMX50_IRAM_SIZE;
    cpu_set_pc(CPU(cpu), s->boot.entry);
}

static void pb_attach_card(FslIMX50State *soc, unsigned unit, bool required,
                            bool emmc)
{
    DriveInfo *di = drive_get(IF_SD, 0, unit);
    DeviceState *card;

    if (!di) {
        if (required && !qtest_enabled()) {
            error_report("PocketBook requires its internal card at index %u",
                          unit);
            exit(EXIT_FAILURE);
        }
        return;
    }
    card = qdev_new(emmc ? TYPE_EMMC : TYPE_SD_CARD);
    qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di), &error_fatal);
    qdev_realize_and_unref(card,
        qdev_get_child_bus(DEVICE(&soc->esdhc[unit]), "sd-bus"), &error_fatal);
}

static void pb_create_lm75(FslIMX50State *soc)
{
    I2CSlave *sensor = i2c_slave_create_simple(soc->i2c[1].bus,
                                               TYPE_TMP105, 0x48);

    /* LM75A and TMP105 share the temperature/config/limit register ABI. */
    object_property_set_int(OBJECT(sensor), "temperature", 25000,
                             &error_fatal);
}

static DeviceState *pb_create_color_peripherals(FslIMX50State *soc)
{
    I2CSlave *touch = i2c_slave_create_simple(soc->i2c[0].bus,
                                              TYPE_GOODIX_GT813, 0x5d);
    I2CSlave *slider = i2c_slave_create_simple(soc->i2c[2].bus,
                                               TYPE_CY8C_SLIDER, 0x24);
    DeviceState *supply = qdev_new(TYPE_SPLIT_IRQ);

    pb_create_lm75(soc);
    qdev_connect_gpio_out_named(DEVICE(touch), "irq", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[4]), 13));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[4]), 14,
        qdev_get_gpio_in_named(DEVICE(touch), "reset", 0));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[4]), 13,
        qdev_get_gpio_in_named(DEVICE(touch), "wake", 0));
    qdev_connect_gpio_out_named(DEVICE(slider), "irq", 0,
        qdev_get_gpio_in(DEVICE(&soc->gpio[4]), 7));
    qdev_connect_gpio_out(DEVICE(&soc->gpio[3]), 2,
        qdev_get_gpio_in_named(DEVICE(slider), "reset", 0));
    qdev_prop_set_uint16(supply, "num-lines", 2);
    qdev_realize_and_unref(supply, NULL, &error_fatal);
    qdev_connect_gpio_out(DEVICE(&soc->gpio[3]), 0,
        qdev_get_gpio_in(supply, 0));
    qdev_connect_gpio_out(supply, 0,
        qdev_get_gpio_in_named(DEVICE(touch), "power", 0));
    qdev_connect_gpio_out(supply, 1,
        qdev_get_gpio_in_named(DEVICE(slider), "power", 0));
    /* USB Wi-Fi's supply remains the software-visible GPIO4_17. There is
     * no RTL8188EU USB radio model attached to this port. */
    return DEVICE(slider);
}

static void pb_create_peripherals(FslIMX50State *soc, const PBProfile *profile)
{
    DeviceState *pmic = qdev_new(TYPE_MC13892);
    DeviceState *buttons = qdev_new(TYPE_PB_BUTTONS);
    DeviceState *usb_misc = qdev_new(TYPE_IMX_USB_MISC);
    unsigned i;

    if (profile->netronix) {
        I2CSlave *mcu = i2c_slave_new(TYPE_KOBOTOUCH_MSP430, 0x43);
        DeviceState *keypad = sysbus_create_simple("imx-keypad", 0x53f94000,
            qdev_get_gpio_in(DEVICE(&soc->tzic), 60));

        pb_create_lm75(soc);
        qdev_prop_set_uint16(DEVICE(mcu), "version", 0xe916);
        i2c_slave_realize_and_unref(mcu, soc->i2c[2].bus, &error_fatal);
        for (i = 0; i < ARRAY_SIZE(profile->matrix_contacts); i++) {
            qdev_connect_gpio_out(buttons, i,
                qdev_get_gpio_in_named(keypad, "key",
                                        profile->matrix_contacts[i]));
        }
    } else if (profile->color) {
        DeviceState *slider = pb_create_color_peripherals(soc);

        object_property_set_link(OBJECT(buttons), "slider", OBJECT(slider),
                                  &error_fatal);
    } else {
        i2c_slave_create_simple(soc->i2c[0].bus, TYPE_FP9928, 0x48);
        i2c_slave_create_simple(soc->i2c[0].bus, TYPE_PB_BATTERY, 0x36);
    }
    ssi_realize_and_unref(pmic,
        (SSIBus *)qdev_get_child_bus(DEVICE(&soc->spi[2]), "spi"),
        &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(&soc->spi[2]), 1,
        qemu_irq_invert(qdev_get_gpio_in_named(pmic, SSI_GPIO_CS, 0)));
    qdev_connect_gpio_out_named(pmic, "irq", 0,
        profile->netronix ? qdev_get_gpio_in(DEVICE(&soc->gpio[5]), 8) :
        qdev_get_gpio_in(DEVICE(&soc->gpio[3]), profile->color ? 18 : 14));

    qdev_prop_set_bit(buttons, "touch", profile->touch);
    qdev_prop_set_bit(buttons, "color", profile->color);
    qdev_realize_and_unref(buttons, NULL, &error_fatal);
    for (i = 0; i < profile->num_keys; i++) {
        const PBKeyPin *key = &profile->keys[i];

        qemu_irq input = qdev_get_gpio_in(DEVICE(&soc->gpio[key->gpio]),
                                         key->pin);

        qdev_connect_gpio_out(buttons, key->button,
                              profile->color ? qemu_irq_invert(input) : input);
    }
    qdev_connect_gpio_out(buttons, 7,
        profile->netronix ? qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 10) :
        qdev_get_gpio_in_named(pmic, "power-button", 0));

    if (profile->touch && !profile->color) {
        I2CSlave *touch = i2c_slave_create_simple(soc->i2c[1].bus,
                                                 TYPE_ELAN_KTF2K, 0x10);

        qdev_connect_gpio_out_named(DEVICE(touch), "irq", 0,
            qdev_get_gpio_in(DEVICE(&soc->gpio[5]), 14));
        qdev_connect_gpio_out(DEVICE(&soc->gpio[5]), 15,
            qdev_get_gpio_in_named(DEVICE(touch), "reset", 0));
        qdev_connect_gpio_out(DEVICE(&soc->gpio[4]), 17,
            qdev_get_gpio_in_named(DEVICE(touch), "power", 0));
    }
    if (profile->accelerometer) {
        I2CSlave *sensor = i2c_slave_create_simple(soc->i2c[0].bus,
                                                   TYPE_MMA7660, 0x4c);

        qdev_connect_gpio_out(DEVICE(sensor), 0,
            qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 15));
    }
    if (profile->wifi) {
        DeviceState *wifi = qdev_new(TYPE_BCM43362_SDIO);
        unsigned controller = profile->netronix ? 2 : 1;

        qemu_configure_nic_device(wifi, true, "bcm43362");
        qdev_realize_and_unref(wifi,
            qdev_get_child_bus(DEVICE(&soc->esdhc[controller]), "sd-bus"),
            &error_fatal);
        /* The EP7 BSP switches the module supply through WIFI_EN. Its
         * separate WIFI_RST output remains deasserted at zero. */
        qdev_connect_gpio_out(DEVICE(&soc->gpio[profile->netronix ? 4 : 3]),
            profile->netronix ? 14 : 5,
            qdev_get_gpio_in_named(wifi, "power", 0));
        qdev_connect_gpio_out_named(wifi, "irq", 0,
            qdev_get_gpio_in_named(DEVICE(&soc->esdhc[controller]),
                                    "sdio-irq", 0));
        if (profile->netronix) {
            qdev_connect_gpio_out_named(wifi, "oob-irq", 0,
                qdev_get_gpio_in(DEVICE(&soc->gpio[3]), 8));
        }
    }
    if (profile->frontlight) {
        DeviceState *light = qdev_new(TYPE_PB_FRONTLIGHT);

        qdev_prop_set_bit(light, "inverted", profile->color);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(light), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(light), 0,
                        profile->color ? 0x53fb8000 : 0x53fb4000);
        qdev_connect_gpio_out(DEVICE(&soc->gpio[profile->color ? 1 : 5]),
            profile->color ? 4 : 9,
            qdev_get_gpio_in_named(light, "power", 0));
    }

    sysbus_realize_and_unref(SYS_BUS_DEVICE(usb_misc), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(usb_misc), 0, 0x53f80800, 2);
    sysbus_connect_irq(SYS_BUS_DEVICE(usb_misc), 0,
        qdev_get_gpio_in_named(DEVICE(&soc->usb_otg), "wakeup", 0));
    qdev_connect_gpio_out_named(DEVICE(&soc->usb_otg), "resume", 0,
                               qdev_get_gpio_in(usb_misc, 0));
}

static void pb_init(MachineState *machine)
{
    PBMachineState *s = PB_MACHINE(machine);
    const PBProfile *profile = PB_MACHINE_GET_CLASS(s)->profile;
    FslIMX50State *soc;

    if (machine->ram_size != profile->ram_size) {
        error_report("%s requires %" PRIu64 " MiB RAM", profile->description,
                      profile->ram_size / MiB);
        exit(EXIT_FAILURE);
    }
    soc = s->soc = FSL_IMX50(object_new(TYPE_FSL_IMX50));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_property_set_uint(OBJECT(soc), "ddr-type", 0x400, &error_fatal);
    object_property_set_uint(OBJECT(soc), "src-sbmr",
                              profile->netronix ? 0x40 : 0x00200040,
                              &error_fatal);
    object_property_set_bool(OBJECT(&soc->epdc), "rotate-ccw",
                             profile->rotate_ccw, &error_fatal);
    object_property_set_bool(OBJECT(&soc->epdc), "rgb-stripe", profile->color,
                              &error_fatal);
    /* SD1 is removable; SD3 carries the internal flash. Battery alert is idle. */
    qdev_prop_set_uint32(DEVICE(&soc->gpio[0]), "reset-psr",
                         drive_get(IF_SD, 0, 0) ? 0 : BIT(27));
    if (profile->color) {
        qdev_prop_set_uint32(DEVICE(&soc->gpio[1]), "reset-psr",
                             BIT(12) | BIT(15) | BIT(17));
    }
    qdev_prop_set_uint32(DEVICE(&soc->gpio[3]), "reset-psr", BIT(7));
    /* EP7 revision 0 straps select the wc121 BCM43362 module in netagent. */
    qdev_prop_set_uint32(DEVICE(&soc->gpio[4]), "reset-psr",
                         profile->color ? BIT(7) : 0);
    /* Pull-ups remain visible while the kernel recovers the I2C buses. */
    qdev_prop_set_uint32(DEVICE(&soc->gpio[5]), "reset-psr",
                         0x00fc0000 | (profile->touch ? BIT(14) : 0));
    if (profile->netronix) {
        /* PCB straps on GPIO6[14,15,17], idle MCU/charger and SD2 CD. */
        qdev_prop_set_uint32(DEVICE(&soc->gpio[0]), "reset-psr", 0);
        qdev_prop_set_uint32(DEVICE(&soc->gpio[3]), "reset-psr",
                             BIT(11) | BIT(18) | BIT(19));
        qdev_prop_set_uint32(DEVICE(&soc->gpio[4]), "reset-psr",
                             drive_get(IF_SD, 0, 1) ? 0 : BIT(17));
        qdev_prop_set_uint32(DEVICE(&soc->gpio[5]), "reset-psr",
                             0x00fc0000 | profile->hwid_straps);
    }
    qdev_realize(DEVICE(soc), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), PB_RAM_BASE,
                                machine->ram);
    pb_create_peripherals(soc, profile);
    if (profile->netronix) {
        pb_attach_card(soc, 0, true, false);
        pb_attach_card(soc, 1, false, false);
    } else {
        pb_attach_card(soc, 0, false, false);
        pb_attach_card(soc, 2, true, true);
    }
    s->boot = (struct arm_boot_info) {
        .loader_start = PB_RAM_BASE,
        .ram_size = machine->ram_size,
        .board_id = PB_MACHINE_ID,
    };
    if (machine->firmware) {
        pb_load_firmware(s);
        qemu_register_reset(pb_firmware_reset, s);
    } else if (!qtest_enabled()) {
        arm_load_kernel(&soc->cpu, machine, &s->boot);
    }
}

static void pb_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    PBMachineClass *pc = PB_MACHINE_CLASS(oc);
    const PBProfile *profile = data;

    pc->profile = profile;
    mc->desc = profile->description;
    mc->init = pb_init;
    mc->default_ram_size = profile->ram_size;
    mc->default_ram_id = profile->ram_id;
    mc->auto_create_sdcard = true;
    mc->ignore_memory_transaction_failures = true;
    if (profile->wifi) {
        mc->default_nic = TYPE_BCM43362_SDIO;
    }
}

static const PBKeyPin pb613_keys[] = {
    { 0, 5, 8 },  /* confirm */
    { 1, 3, 3 },  /* left */
    { 2, 3, 4 },  /* right */
    { 3, 5, 9 },  /* down */
    { 4, 1, 10 }, /* up */
    { 5, 1, 14 }, /* backward */
    { 6, 1, 9 },  /* forward */
};

static const PBKeyPin pb622_keys[] = {
    { 0, 3, 4 }, /* home */
    { 1, 3, 3 }, /* menu */
    { 5, 3, 6 }, /* backward */
    { 6, 3, 1 }, /* forward */
};

static const PBKeyPin pb801_keys[] = {
    { 0, 1, 12 }, /* back, active low */
    { 1, 5, 14 }, /* menu, active low */
    { 5, 1, 15 }, /* up, active low */
    { 6, 1, 17 }, /* down, active low */
};

static const PBProfile pb613_profile = {
    .description = "PocketBook Basic 613 (i.MX50 EP8)",
    .ram_id = "pocketbook-613.ram",
    .keys = pb613_keys,
    .num_keys = ARRAY_SIZE(pb613_keys),
    .ram_size = PB_RAM_SIZE,
    .rotate_ccw = true,
};

static const PBProfile pb611_profile = {
    .description = "PocketBook Basic 611 (i.MX50 Netronix E60630)",
    .ram_id = "pocketbook-611.ram",
    .ram_size = PB_RAM_SIZE,
    .hwid_straps = BIT(15) | BIT(17), /* E60630: HWID 6 */
    .matrix_contacts = { 8, 3, 2, 1, 0, 9, 10 },
    .rotate_ccw = true,
    .netronix = true,
    .wifi = true,
};

static const PBProfile pb622_profile = {
    .description = "PocketBook Touch 622 (i.MX50 EP7)",
    .ram_id = "pocketbook-622.ram",
    .keys = pb622_keys,
    .num_keys = ARRAY_SIZE(pb622_keys),
    .ram_size = PB_RAM_SIZE,
    .touch = true,
    .wifi = true,
};

static const PBProfile pb623_profile = {
    .description = "PocketBook Touch Lux 623 (i.MX50 EP7A)",
    .ram_id = "pocketbook-623.ram",
    .keys = pb622_keys,
    .num_keys = ARRAY_SIZE(pb622_keys),
    .ram_size = PB_RAM_SIZE,
    .touch = true,
    .wifi = true,
    .frontlight = true,
};

static const PBProfile pb801_profile = {
    .description = "PocketBook Color Lux 801 (i.MX50 EC080SC1)",
    .ram_id = "pocketbook-801.ram",
    .keys = pb801_keys,
    .num_keys = ARRAY_SIZE(pb801_keys),
    .ram_size = 256 * MiB,
    .touch = true,
    .frontlight = true,
    .color = true,
};

static const TypeInfo pb_types[] = {
    {
        .name = TYPE_PB_BATTERY,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(PBBatteryState),
        .class_init = pb_battery_class_init,
    }, {
        .name = TYPE_PB_BUTTONS,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(PBButtonsState),
        .instance_init = pb_buttons_init,
        .class_init = pb_buttons_class_init,
    }, {
        .name = TYPE_PB_FRONTLIGHT,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(PBFrontlightState),
        .instance_init = pb_frontlight_init,
        .class_init = pb_frontlight_class_init,
    }, {
        .name = TYPE_PB_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(PBMachineState),
        .class_size = sizeof(PBMachineClass),
        .abstract = true,
        .interfaces = arm_machine_interfaces,
    }, {
        .name = MACHINE_TYPE_NAME("imx50-pocketbook-611"),
        .parent = TYPE_PB_MACHINE,
        .class_init = pb_machine_class_init,
        .class_data = &pb611_profile,
    }, {
        .name = MACHINE_TYPE_NAME("imx50-pocketbook-613"),
        .parent = TYPE_PB_MACHINE,
        .class_init = pb_machine_class_init,
        .class_data = &pb613_profile,
    }, {
        .name = MACHINE_TYPE_NAME("imx50-pocketbook-622"),
        .parent = TYPE_PB_MACHINE,
        .class_init = pb_machine_class_init,
        .class_data = &pb622_profile,
    }, {
        .name = MACHINE_TYPE_NAME("imx50-pocketbook-623"),
        .parent = TYPE_PB_MACHINE,
        .class_init = pb_machine_class_init,
        .class_data = &pb623_profile,
    }, {
        .name = MACHINE_TYPE_NAME("imx50-pocketbook-801"),
        .parent = TYPE_PB_MACHINE,
        .class_init = pb_machine_class_init,
        .class_data = &pb801_profile,
    },
};

DEFINE_TYPES(pb_types)
