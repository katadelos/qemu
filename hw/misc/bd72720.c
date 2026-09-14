/*
 * ROHM BD72720 digital PMIC interface.
 * Register layout/semantics: Lab126 rohm-bd72720.h, bd71828-regulator.c,
 * gpio-bd72720.c and rtc-bd70528.c. Analog regulation, RTC alarms and the
 * charging state machine are not implemented. Reset values below describe
 * a virtual board, not a dump of the board's OTP configuration.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/bcd.h"
#include "qemu/module.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "system/rtc.h"
#include "system/runstate.h"

#define TYPE_BD72720 "bd72720"
OBJECT_DECLARE_SIMPLE_TYPE(BD72720State, BD72720)

#define BD72720_REG_PS_CTRL_1  0x1a
#define BD72720_ENTER_SHPM     0x01
#define BD72720_ENTER_HBNT     0x02
#define BD72720_REG_PS1_STAT   0xce
#define BD72720_REG_PS1_SRC    0xdb
#define BD72720_SHORTPUSH      BIT(2)
#define BD72720_PUSH           BIT(3)

struct BD72720State {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t ptr;
    bool addr_byte, rtc_dirty, charger;
    bool poweroff_pending;
    bool power_button, powerdown_registered;
    QEMUTimer *power_release_timer;
    Notifier powerdown_notifier;
    uint64_t power_presses, power_releases, power_short_acks;
    int64_t rtc_offset;
    qemu_irq irq;
    qemu_irq gpio[6];
};

static void bd72720_update(BD72720State *s)
{
    static const uint8_t group[] = { 0, 0, 1, 1, 2, 3, 4, 5, 5, 6, 7, 7 };
    uint8_t status = 0, source = 0;

    if (s->charger) {
        return;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(group); i++) {
        if (s->regs[0xce + i] & s->regs[0xc1 + i]) {
            status |= 1 << group[i];
        }
        if (s->regs[0xdb + i]) {
            source |= 1 << group[i];
        }
    }
    s->regs[0xcd] = status;
    s->regs[0xda] = source;
    /* Physical nIRQ output, including deasserted high level. */
    qemu_set_irq(s->irq, !(status & s->regs[0xc0]));
    for (unsigned i = 0; i < ARRAY_SIZE(s->gpio); i++) {
        qemu_set_irq(s->gpio[i], s->regs[0x90 + i] & 1);
    }
}

/* rohm-bd72720.h: PS1 PUSH bit 3, SHORTPUSH bit 2; *_SRC is current
 * function state and *_STAT is W1C interrupt state. rohm-bd71828.c maps
 * SHORTPUSH to gpio-keys KEY_POWER; bd71827-power.c also converts it to
 * POWERON_SHORT/KOBJ_ONLINE. Only the short-press decoder is modeled:
 * the published driver does not specify PWRON_CFG1/2 duration encodings
 * for MIDPUSH/LONGPUSH or their OTP thresholds. A release completes one
 * short press; holding this input does not synthesize a long-press reset.
 */
static void bd72720_power_button(BD72720State *s, bool pressed)
{
    if (s->charger || s->power_button == pressed) {
        return;
    }
    s->power_button = pressed;
    if (pressed) {
        s->regs[BD72720_REG_PS1_SRC] |= BD72720_PUSH;
        s->regs[BD72720_REG_PS1_STAT] |= BD72720_PUSH;
        s->power_presses++;
    } else {
        s->regs[BD72720_REG_PS1_SRC] &= ~BD72720_PUSH;
        s->regs[BD72720_REG_PS1_STAT] |= BD72720_SHORTPUSH;
        s->power_releases++;
    }
    bd72720_update(s);
}

static bool bd72720_get_power_button(Object *obj, Error **errp)
{
    return BD72720(obj)->power_button;
}

static void bd72720_set_power_button(Object *obj, bool pressed, Error **errp)
{
    BD72720State *s = BD72720(obj);

    if (s->charger) {
        error_setg(errp, "The BD72720 charger bank has no power-button input");
        return;
    }
    timer_del(s->power_release_timer);
    bd72720_power_button(s, pressed);
}

static void bd72720_power_release(void *opaque)
{
    bd72720_power_button(opaque, false);
}

static void bd72720_powerdown(Notifier *notifier, void *opaque)
{
    BD72720State *s = container_of(notifier, BD72720State, powerdown_notifier);

    if (!s->power_button) {
        bd72720_power_button(s, true);
        /* A host short click, not an assumed PMIC debounce/OTP threshold.
         * Virtual time continues in guest s2idle and the resulting nIRQ
         * wakes the CPU through the board's actual EINT connection. */
        timer_mod(s->power_release_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }
}

static void bd72720_gpio_input(void *opaque, int pin, int level)
{
    BD72720State *s = opaque;
    unsigned bit = 1 << (pin + 4);
    bool old = s->regs[0xe5] & bit;
    unsigned mode = (s->regs[0x90 + pin] >> 4) & 7;

    s->regs[0xe5] = (s->regs[0xe5] & ~bit) | (level ? bit : 0);
    if ((mode == 0 && old && !level) ||
        (mode == 1 && !old && level) ||
        (mode == 2 && old != !!level) ||
        (mode == 3 && level) || (mode == 4 && !level)) {
        s->regs[0xd8] |= bit;
    }
    bd72720_update(s);
}

static void bd72720_capture_time(BD72720State *s)
{
    struct tm now;

    qemu_get_timedate(&now, s->rtc_offset);
    s->regs[0xa0] = to_bcd(now.tm_sec);
    s->regs[0xa1] = to_bcd(now.tm_min);
    s->regs[0xa2] = 0x80 | to_bcd(now.tm_hour);
    s->regs[0xa3] = now.tm_wday;
    s->regs[0xa4] = to_bcd(now.tm_mday);
    s->regs[0xa5] = to_bcd(now.tm_mon + 1);
    s->regs[0xa6] = to_bcd(now.tm_year % 100);
}

static void bd72720_commit_time(BD72720State *s)
{
    struct tm now = { 0 };
    unsigned hour = s->regs[0xa2];

    now.tm_sec = from_bcd(s->regs[0xa0] & 0x7f);
    now.tm_min = from_bcd(s->regs[0xa1] & 0x7f);
    now.tm_hour = from_bcd(hour & 0x3f);
    if (!(hour & 0x80)) {
        now.tm_hour %= 12;
        now.tm_hour += (hour & 0x40) ? 12 : 0;
    }
    now.tm_mday = from_bcd(s->regs[0xa4] & 0x3f);
    now.tm_mon = from_bcd(s->regs[0xa5] & 0x1f) - 1;
    now.tm_year = 100 + from_bcd(s->regs[0xa6]);
    if (now.tm_mday && now.tm_mon >= 0 && now.tm_mon < 12) {
        s->rtc_offset = qemu_timedate_diff(&now);
    }
    s->rtc_dirty = false;
}

static int bd72720_event(I2CSlave *i2c, enum i2c_event event)
{
    BD72720State *s = BD72720(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->addr_byte = true;
        break;
    case I2C_START_RECV:
        if (!s->charger && !s->rtc_dirty) {
            bd72720_capture_time(s);
        }
        break;
    case I2C_FINISH:
        if (s->rtc_dirty) {
            bd72720_commit_time(s);
        }
        if (s->poweroff_pending) {
            s->poweroff_pending = false;
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t bd72720_recv(I2CSlave *i2c)
{
    BD72720State *s = BD72720(i2c);
    return s->regs[s->ptr++];
}

static int bd72720_send(I2CSlave *i2c, uint8_t value)
{
    BD72720State *s = BD72720(i2c);
    unsigned reg;

    if (s->addr_byte) {
        s->ptr = value;
        s->addr_byte = false;
        return 0;
    }
    reg = s->ptr++;
    if (s->charger) {
        /* Status and ADC results are hardware-owned; configuration and
         * coulomb accumulator preload registers are writable. */
        if (reg >= 8 && !(reg >= 0x30 && reg <= 0x69) &&
            !(reg >= 0x70 && reg <= 0x76)) {
            s->regs[reg] = value;
        }
    } else if (reg >= 0xce && reg <= 0xd9) {
        if (reg == BD72720_REG_PS1_STAT &&
            (s->regs[reg] & value & BD72720_SHORTPUSH)) {
            s->power_short_acks++;
        }
        s->regs[reg] &= ~value;
        if (reg == 0xd8) {
            for (unsigned pin = 0; pin < 2; pin++) {
                bool high = s->regs[0xe5] & (1 << (pin + 4));
                unsigned mode = (s->regs[0x90 + pin] >> 4) & 7;
                if ((mode == 3 && high) || (mode == 4 && !high)) {
                    s->regs[reg] |= 1 << (pin + 4);
                }
            }
        }
    } else if (reg >= 0x19 && reg != 0xcd && reg < 0xda) {
        s->regs[reg] = value;
        /* Lab126 rohm-bd71828.c: pmic_power_off() first clears PS_CTRL_2,
         * then rohm_pmic_enter_shpm()/rohm_pmic_enter_hbnt() writes this
         * command. Both remove the SoC supply; the kernel subsequently
         * waits forever for that rail transition. Complete the I2C write
         * before requesting host-side guest shutdown. The charger bank
         * has a different register map and never enters this branch. */
        if (reg == BD72720_REG_PS_CTRL_1 &&
            (value == BD72720_ENTER_SHPM || value == BD72720_ENTER_HBNT)) {
            s->poweroff_pending = true;
        }
        if (reg >= 0xa0 && reg <= 0xa6) {
            s->rtc_dirty = true;
        }
    }
    bd72720_update(s);
    return 0;
}

static void bd72720_reset(Object *obj, ResetType type)
{
    BD72720State *s = BD72720(obj);
    static const uint8_t enables[] = {
        0x20, 0x2a, 0x30, 0x36, 0x3c, 0x3f, 0x42, 0x45, 0x48, 0x4b,
        0x4e, 0x59, 0x5f, 0x65, 0x6b, 0x6e, 0x71, 0x74, 0x77, 0x7a, 0x7d,
    };

    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->addr_byte = false;
    s->rtc_dirty = false;
    s->poweroff_pending = false;
    s->power_button = false;
    s->power_presses = 0;
    s->power_releases = 0;
    s->power_short_acks = 0;
    timer_del(s->power_release_timer);
    s->rtc_offset = 0;
    if (s->charger) {
        /* Static virtual cell at 4.0 V, 25 C, zero load and no VBUS.
         * Driver units: voltage 1 mV/LSB; temperature (2000000-625*x)/1000.
         * These are model inputs, not measured hardware or battery identity. */
        for (unsigned reg = 0x30; reg <= 0x4a; reg += 2) {
            if (reg == 0x38) { /* DVBAT_IMP: voltage delta, zero at no load. */
                continue;
            }
            s->regs[reg] = 4000 >> 8;
            s->regs[reg + 1] = 4000 & 255;
        }
        s->regs[0x58] = 2800 >> 8;
        s->regs[0x59] = 2800 & 255;
    } else {
        /* Model the post-DS1 digital IRQ layout; actual OTP/revision unknown. */
        s->regs[2] = 1;
        for (unsigned i = 0; i < ARRAY_SIZE(enables); i++) {
            s->regs[enables[i]] = 8; /* RUN_B */
        }
        /* eMMC supplies: virtual 1.8 V / 3.3 V, DT-described rails. */
        s->regs[0x41] = 30;
        s->regs[0x44] = 180;
        bd72720_capture_time(s);
    }
    bd72720_update(s);
}

static const Property bd72720_properties[] = {
    DEFINE_PROP_BOOL("charger", BD72720State, charger, false),
};

static int bd72720_post_load(void *opaque, int version_id)
{
    bd72720_update(opaque);
    return 0;
}

static const VMStateDescription vmstate_bd72720 = {
    .name = TYPE_BD72720,
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = bd72720_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, BD72720State),
        VMSTATE_UINT8_ARRAY(regs, BD72720State, 256),
        VMSTATE_UINT8(ptr, BD72720State),
        VMSTATE_BOOL(addr_byte, BD72720State),
        VMSTATE_BOOL(rtc_dirty, BD72720State),
        VMSTATE_BOOL(poweroff_pending, BD72720State),
        VMSTATE_BOOL(power_button, BD72720State),
        VMSTATE_TIMER_PTR(power_release_timer, BD72720State),
        VMSTATE_UINT64(power_presses, BD72720State),
        VMSTATE_UINT64(power_releases, BD72720State),
        VMSTATE_UINT64(power_short_acks, BD72720State),
        VMSTATE_INT64(rtc_offset, BD72720State),
        VMSTATE_END_OF_LIST()
    },
};

static void bd72720_init(Object *obj)
{
    BD72720State *s = BD72720(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_out_named(DEVICE(obj), s->gpio, "gpio", 6);
    qdev_init_gpio_in(DEVICE(obj), bd72720_gpio_input, 2);
    s->power_release_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                         bd72720_power_release, s);
    object_property_add_bool(obj, "power-button", bd72720_get_power_button,
                             bd72720_set_power_button);
    object_property_add_uint64_ptr(obj, "power-presses", &s->power_presses,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "power-releases", &s->power_releases,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "power-short-acks", &s->power_short_acks,
                                   OBJ_PROP_FLAG_READ);
}

static void bd72720_realize(DeviceState *dev, Error **errp)
{
    BD72720State *s = BD72720(dev);

    if (!s->charger) {
        s->powerdown_notifier.notify = bd72720_powerdown;
        qemu_register_powerdown_notifier(&s->powerdown_notifier);
        s->powerdown_registered = true;
    }
}

static void bd72720_unrealize(DeviceState *dev)
{
    BD72720State *s = BD72720(dev);

    timer_del(s->power_release_timer);
    if (s->powerdown_registered) {
        notifier_remove(&s->powerdown_notifier);
        s->powerdown_registered = false;
    }
}

static void bd72720_finalize(Object *obj)
{
    timer_free(BD72720(obj)->power_release_timer);
}

static void bd72720_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(klass);

    ic->event = bd72720_event;
    ic->recv = bd72720_recv;
    ic->send = bd72720_send;
    RESETTABLE_CLASS(klass)->phases.enter = bd72720_reset;
    dc->vmsd = &vmstate_bd72720;
    dc->realize = bd72720_realize;
    dc->unrealize = bd72720_unrealize;
    device_class_set_props(dc, bd72720_properties);
}

static const TypeInfo bd72720_type = {
    .name = TYPE_BD72720,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BD72720State),
    .instance_init = bd72720_init,
    .instance_finalize = bd72720_finalize,
    .class_init = bd72720_class_init,
};

static void bd72720_register(void)
{
    type_register_static(&bd72720_type);
}
type_init(bd72720_register);
