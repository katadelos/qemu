/*
 * reMarkable LPC804 hardware monitor, UART attribute protocol.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/char/remarkable_hwmon.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "system/reset.h"

#define TYPE_RM_HWMON "chardev-remarkable-hwmon"
OBJECT_DECLARE_SIMPLE_TYPE(RMHwmonState, RM_HWMON)
struct RMHwmonState {
    Chardev parent_obj;
    QEMUTimer timer;
    uint8_t request[4096], reply[4096];
    uint16_t received, queued, sent;
    uint16_t allowed_wakeup, voltage_low;
    uint32_t firmware_id;
    qemu_irq wakeup;
    bool awake;
    bool reset_asserted;
};

static void hwmon_transmit(void *opaque)
{
    RMHwmonState *s = opaque;
    Chardev *chr = CHARDEV(s);
    int count = MIN(1, MIN(s->queued - s->sent, qemu_chr_be_can_write(chr)));
    if (count > 0) {
        qemu_chr_be_write(chr, s->reply + s->sent, count);
        s->sent += count;
    }
    if (s->sent < s->queued) {
        timer_mod_ns(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 87000);
    } else {
        s->sent = s->queued = 0;
    }
}

static void hwmon_reply(RMHwmonState *s, uint8_t command, uint8_t endpoint,
                         const uint8_t *payload, unsigned length)
{
    unsigned start = s->queued;
    uint8_t checksum = 0;
    if (start + length + 5 > sizeof(s->reply)) {
        return;
    }
    s->reply[s->queued++] = '.';
    stw_le_p(s->reply + s->queued, length | ((endpoint & 15) << 12));
    s->queued += 2;
    s->reply[s->queued++] = command;
    memcpy(s->reply + s->queued, payload, length);
    s->queued += length;
    for (unsigned i = start + 1; i < s->queued; i++) {
        checksum -= s->reply[i];
    }
    s->reply[s->queued++] = checksum;
    timer_mod_ns(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000000);
}

static unsigned hwmon_attribute(RMHwmonState *s, uint8_t id, uint8_t *p)
{
    p[0] = id;
    p[1] = 0;
    p[2] = 0x20; /* unsigned byte */
    p[3] = 0;
    switch (id) {
    case 1: p[3] = 1; break; /* protocol version */
    case 2: p[2] = 0x21; stw_le_p(p + 3, 0x0100); return 5;
    case 3:
        p[2] = 0x48; p[3] = 0x0b; stw_le_p(p + 4, 4);
        memset(p + 6, 0, 16);
        stl_le_p(p + 6, 0x804);
        return 22;
    case 4: p[2] = 0x0b; stl_le_p(p + 3, s->firmware_id); return 7;
    case 5: p[3] = 'F'; break; /* Ferrari PCBA revision */
    case 6: p[3] = 1; break;
    case 0x10: p[2] = 0x19; stw_le_p(p + 3, s->allowed_wakeup); return 5;
    case 0x11: p[2] = 0x19; stw_le_p(p + 3, 0); return 5;
    case 0x20: p[2] = 0x29; stw_le_p(p + 3, 2500); return 5; /* 25 C */
    case 0x21: p[2] = 0x21; stw_le_p(p + 3, 4000); return 5;
    case 0x22: p[2] = 0x21; stw_le_p(p + 3, s->voltage_low); return 5;
    case 0xf0: p[2] = 0x18; break;
    case 0xf1:
        p[2] = 0x48; p[3] = 0x1b; stw_le_p(p + 4, 2);
        memset(p + 6, 0, 8);
        return 14;
    default: p[1] = 0x86; return 2; /* unsupported attribute */
    }
    return 4;
}

static void hwmon_command(RMHwmonState *s)
{
    unsigned length = lduw_le_p(s->request + 1) & 0xfff;
    uint8_t endpoint = s->request[2] >> 4;
    uint8_t command = s->request[3];
    uint8_t payload[1024] = {0};
    unsigned response = 0;
    uint8_t checksum = 0;
    for (unsigned i = 1; i < s->received; i++) {
        checksum += s->request[i];
    }
    if (checksum) {
        hwmon_reply(s, 0xf1, endpoint, payload, 0);
        return;
    }
    switch (command) {
    case 0x20:
        for (unsigned i = 0; i < length && response + 32 < sizeof(payload); i++) {
            response += hwmon_attribute(s, s->request[4 + i], payload + response);
        }
        break;
    case 0x21:
        if (length >= 4) {
            uint8_t id = s->request[4];
            if (id == 0x10) {
                s->allowed_wakeup = lduw_le_p(s->request + 6);
            } else if (id == 0x22) {
                s->voltage_low = lduw_le_p(s->request + 6);
            }
            payload[0] = id;
            response = 2; /* attribute ID, success */
        }
        break;
    case 0x00: case 0x02: case 0x03: case 0xd0:
        response = 1; /* command success */
        break;
    case 0x01:
        response = 1;
        break;
    default:
        command = 0xf0;
        break;
    }
    hwmon_reply(s, command, endpoint, payload, response);
}

static int hwmon_write(Chardev *chr, const uint8_t *buf, int length)
{
    RMHwmonState *s = RM_HWMON(chr);
    for (int i = 0; i < length; i++) {
        if (!s->received && buf[i] != ':') {
            continue;
        }
        s->request[s->received++] = buf[i];
        if (s->received >= 3) {
            unsigned expected = (lduw_le_p(s->request + 1) & 0xfff) + 5;
            if (expected > sizeof(s->request)) {
                s->received = 0;
            } else if (s->received == expected) {
                hwmon_command(s);
                s->received = 0;
            }
        }
    }
    return length;
}

static void hwmon_reset_pin(void *opaque, int n, int level)
{
    RMHwmonState *s = opaque;
    if (!level) {
        s->reset_asserted = true;
        s->received = s->queued = s->sent = 0;
        timer_del(&s->timer);
        qemu_set_irq(s->wakeup, 0);
    } else if (s->reset_asserted) {
        s->reset_asserted = false;
        qemu_set_irq(s->wakeup, s->awake);
        hwmon_reply(s, 0xe0, 0, NULL, 0);
        timer_mod_ns(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 3000000);
    }
}
static void hwmon_wakeup_pin(void *opaque, int n, int level)
{
    RMHwmonState *s = opaque;
    s->awake = level;
    qemu_set_irq(s->wakeup, level && !s->reset_asserted);
}

static int hwmon_post_load(void *opaque, int version)
{
    RMHwmonState *s = opaque;
    qemu_set_irq(s->wakeup, s->awake && !s->reset_asserted);
    return 0;
}

static const VMStateDescription hwmon_vmstate = {
    .name = "remarkable-hwmon", .version_id = 1, .minimum_version_id = 1,
    .post_load = hwmon_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(request, RMHwmonState, 4096),
        VMSTATE_UINT8_ARRAY(reply, RMHwmonState, 4096),
        VMSTATE_UINT16(received, RMHwmonState),
        VMSTATE_UINT16(queued, RMHwmonState),
        VMSTATE_UINT16(sent, RMHwmonState),
        VMSTATE_UINT16(allowed_wakeup, RMHwmonState),
        VMSTATE_UINT16(voltage_low, RMHwmonState),
        VMSTATE_BOOL(reset_asserted, RMHwmonState),
        VMSTATE_BOOL(awake, RMHwmonState),
        VMSTATE_TIMER(timer, RMHwmonState),
        VMSTATE_END_OF_LIST()
    },
};

Chardev *remarkable_hwmon_create(uint32_t firmware_id)
{
    Chardev *chr = qemu_chardev_new("ferrari.hwmon", TYPE_RM_HWMON,
                                    NULL, NULL, &error_fatal);
    RMHwmonState *s = RM_HWMON(chr);
    s->firmware_id = firmware_id;
    s->voltage_low = 3300;
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, hwmon_transmit, s);
    vmstate_register(NULL, 0, &hwmon_vmstate, s);
    return chr;
}

qemu_irq remarkable_hwmon_reset_input(Chardev *chr)
{
    return qemu_allocate_irq(hwmon_reset_pin, RM_HWMON(chr), 0);
}
qemu_irq remarkable_hwmon_wakeup_input(Chardev *chr)
{
    return qemu_allocate_irq(hwmon_wakeup_pin, RM_HWMON(chr), 0);
}
void remarkable_hwmon_connect_wakeup(Chardev *chr, qemu_irq output)
{
    RM_HWMON(chr)->wakeup = output;
}

static void hwmon_class_init(ObjectClass *oc, const void *data)
{
    CHARDEV_CLASS(oc)->chr_write = hwmon_write;
}
static const TypeInfo hwmon_info = {
    .name = TYPE_RM_HWMON, .parent = TYPE_CHARDEV,
    .instance_size = sizeof(RMHwmonState), .class_init = hwmon_class_init,
};
static void hwmon_register(void) { type_register_static(&hwmon_info); }
type_init(hwmon_register)
