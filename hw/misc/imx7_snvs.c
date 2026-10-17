/*
 * IMX7 Secure Non-Volatile Storage
 *
 * Copyright (c) 2018, Impinj, Inc.
 *
 * Author: Andrey Smirnov <andrew.smirnov@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 * Bare minimum emulation code needed to support being able to shut
 * down linux guest gracefully.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "hw/misc/imx7_snvs.h"
#include "hw/core/irq.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "system/system.h"
#include "system/rtc.h"
#include "system/runstate.h"
#include "trace.h"

#define RTC_FREQ    32768ULL

static void imx7_snvs_update_irq(IMX7SNVSState *s)
{
    qemu_set_irq(s->power_irq, (s->lpsr & BIT(18)) != 0);
    qemu_set_irq(s->alarm_irq, (s->lpsr & BIT(0)) &&
                 (s->lpcr & SNVS_LPCR_LPTA_EN));
}

static int imx7_snvs_post_load(void *opaque, int version)
{
    IMX7SNVSState *s = opaque;
    imx7_snvs_update_irq(s);
    return 0;
}
static const VMStateDescription vmstate_imx7_snvs = {
    .name = TYPE_IMX7_SNVS,
    .post_load = imx7_snvs_post_load,
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(tick_offset, IMX7SNVSState),
        VMSTATE_UINT64(lpcr, IMX7SNVSState),
        VMSTATE_UINT32(lpsr, IMX7SNVSState),
        VMSTATE_UINT32(lptar, IMX7SNVSState),
        VMSTATE_TIMER(alarm_timer, IMX7SNVSState),
        VMSTATE_UINT32_ARRAY(gpr, IMX7SNVSState, 4),
        VMSTATE_BOOL(button_pressed, IMX7SNVSState),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t imx7_snvs_get_count(IMX7SNVSState *s)
{
    uint64_t ticks = muldiv64(qemu_clock_get_ns(rtc_clock), RTC_FREQ,
                              NANOSECONDS_PER_SECOND);
    return s->tick_offset + ticks;
}

static void imx7_snvs_alarm(void *opaque)
{
    IMX7SNVSState *s = opaque;
    s->lpsr |= BIT(0);
    imx7_snvs_update_irq(s);
}

static void imx7_snvs_schedule_alarm(IMX7SNVSState *s)
{
    uint64_t delta;
    uint64_t enable = SNVS_LPCR_SRTC_ENV | SNVS_LPCR_LPTA_EN;

    timer_del(&s->alarm_timer);
    if ((s->lpcr & enable) != enable || (s->lpsr & BIT(0))) {
        return;
    }
    /* The alarm compares the seconds of the 47-bit, 32.768 kHz counter. */
    delta = (((uint64_t)s->lptar << 15) - imx7_snvs_get_count(s)) &
            ((1ULL << 47) - 1);
    timer_mod_ns(&s->alarm_timer, qemu_clock_get_ns(rtc_clock) +
                 muldiv64(delta, NANOSECONDS_PER_SECOND, RTC_FREQ) + 1);
}

static uint64_t imx7_snvs_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX7SNVSState *s = IMX7_SNVS(opaque);
    uint64_t ret = 0;

    if (offset >= 0x90 && offset < 0xa0) {
        return s->gpr[(offset - 0x90) / 4];
    }

    switch (offset) {
    case 0x14: /* HPSR: raw ON/OFF button state. */
        ret = s->button_pressed ? BIT(6) : 0;
        break;
    case 0x4c:
        ret = s->lpsr;
        break;
    case SNVS_LPSRTCMR:
        ret = extract64(imx7_snvs_get_count(s), 32, 15);
        break;
    case SNVS_LPSRTCLR:
        ret = extract64(imx7_snvs_get_count(s), 0, 32);
        break;
    case SNVS_LPCR:
        ret = s->lpcr;
        break;
    case SNVS_LPTAR:
        ret = s->lptar;
        break;
    }

    trace_imx7_snvs_read(offset, ret, size);

    return ret;
}

static void imx7_snvs_reset(DeviceState *dev)
{
    IMX7SNVSState *s = IMX7_SNVS(dev);

    s->lpcr = 0;
    s->lpsr = 0;
    s->lptar = 0;
    timer_del(&s->alarm_timer);
    s->button_pressed = false;
    imx7_snvs_update_irq(s);
}

static void imx7_snvs_write(void *opaque, hwaddr offset,
                            uint64_t v, unsigned size)
{
    trace_imx7_snvs_write(offset, v, size);

    IMX7SNVSState *s = IMX7_SNVS(opaque);

    uint64_t new_value = 0, snvs_count = 0;

    if (offset >= 0x90 && offset < 0xa0) {
        s->gpr[(offset - 0x90) / 4] = v;
        return;
    }

    if (offset == SNVS_LPSRTCMR || offset == SNVS_LPSRTCLR) {
        snvs_count = imx7_snvs_get_count(s);
    }

    switch (offset) {
    case 0x4c:
        s->lpsr &= ~v;
        break;
    case SNVS_LPTAR:
        s->lptar = v;
        break;
    case SNVS_LPSRTCMR:
        new_value = deposit64(snvs_count, 32, 32, v);
        break;
    case SNVS_LPSRTCLR:
        new_value = deposit64(snvs_count, 0, 32, v);
        break;
    case SNVS_LPCR: {
        s->lpcr = v;

        const uint32_t mask  = SNVS_LPCR_TOP | SNVS_LPCR_DP_EN;

        if ((v & mask) == mask) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
        break;
    }
    }

    if (offset == SNVS_LPSRTCMR || offset == SNVS_LPSRTCLR) {
        s->tick_offset += new_value - snvs_count;
    }
    imx7_snvs_update_irq(s);
    imx7_snvs_schedule_alarm(s);
}

static const struct MemoryRegionOps imx7_snvs_ops = {
    .read = imx7_snvs_read,
    .write = imx7_snvs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        /*
         * Our device would not work correctly if the guest was doing
         * unaligned access. This might not be a limitation on the real
         * device but in practice there is no reason for a guest to access
         * this device unaligned.
         */
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void imx7_snvs_power_key(void *opaque, int line, int level)
{
    IMX7SNVSState *s = opaque;
    bool pressed = !level;
    if (pressed && !s->button_pressed) {
        s->lpsr |= BIT(18); /* Short power-off button interrupt, W1C. */
        imx7_snvs_update_irq(s);
    }
    s->button_pressed = pressed;
}

static void imx7_snvs_init(Object *obj)
{
    SysBusDevice *sd = SYS_BUS_DEVICE(obj);
    IMX7SNVSState *s = IMX7_SNVS(obj);
    struct tm tm;

    memory_region_init_io(&s->mmio, obj, &imx7_snvs_ops, s,
                          TYPE_IMX7_SNVS, 0x1000);

    sysbus_init_mmio(sd, &s->mmio);
    sysbus_init_irq(sd, &s->power_irq);
    sysbus_init_irq(sd, &s->alarm_irq);
    timer_init_ns(&s->alarm_timer, rtc_clock, imx7_snvs_alarm, s);
    qdev_init_gpio_in_named(DEVICE(obj), imx7_snvs_power_key, "power-key", 1);

    qemu_get_timedate(&tm, 0);
    s->tick_offset = (uint64_t)mktimegm(&tm) * RTC_FREQ -
        muldiv64(qemu_clock_get_ns(rtc_clock), RTC_FREQ, NANOSECONDS_PER_SECOND);
}

static void imx7_snvs_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, imx7_snvs_reset);
    dc->vmsd = &vmstate_imx7_snvs;
    dc->desc  = "i.MX7 Secure Non-Volatile Storage Module";
}

static const TypeInfo imx7_snvs_info = {
    .name          = TYPE_IMX7_SNVS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX7SNVSState),
    .instance_init = imx7_snvs_init,
    .class_init    = imx7_snvs_class_init,
};

static void imx7_snvs_register_type(void)
{
    type_register_static(&imx7_snvs_info);
}
type_init(imx7_snvs_register_type)
