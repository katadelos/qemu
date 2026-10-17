/* ELAN SPI touch/active-marker controller. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "ui/input.h"

#define TYPE_ELANTS "elants-spi"
OBJECT_DECLARE_SIMPLE_TYPE(ELANTSState, ELANTS)
struct ELANTSState {
    SSIPeripheral parent_obj;
    qemu_irq irq, power_key;
    QemuInputHandlerState *input;
    uint8_t rx[16384], command[8];
    uint16_t queued, sent, position;
    uint16_t firmware_id, firmware_version, idle_rate;
    int32_t x, y;
    bool reading, reset_asserted, left, touch_mode, dirty;
};
static void elants_irq(ELANTSState *s)
{
    qemu_set_irq(s->irq, s->reset_asserted || s->sent == s->queued);
}
static void elants_queue(ELANTSState *s, const uint8_t *data, unsigned length)
{
    if (s->sent) {
        memmove(s->rx, s->rx + s->sent, s->queued - s->sent);
        s->queued -= s->sent;
        s->sent = 0;
    }
    if (s->queued + length <= sizeof(s->rx)) {
        memcpy(s->rx + s->queued, data, length);
        s->queued += length;
    }
    elants_irq(s);
}
static void elants_command(ELANTSState *s)
{
    const uint8_t *c = s->command;
    uint8_t response[6] = {0x52, c[1], 0, 0, 0, 0};
    if (c[0] == 0x53) {
        uint16_t value = c[1] == 0xf0 ? s->firmware_id : s->firmware_version;
        if (c[1] == 0xc3) {
            response[2] = s->idle_rate >> 8;
            response[3] = s->idle_rate;
        } else {
            response[1] = (c[1] & 0xf0) | (value >> 12);
            response[2] = value >> 4;
            response[3] = value << 4;
        }
        elants_queue(s, response, sizeof(response));
    } else if (c[0] == 0x54 && c[1] == 0xc3) {
        s->idle_rate = c[2] << 8 | c[3];
    } else if (c[0] == 0x54 && c[1] == 0x29) {
        memset(response, 0x66, 4);
        elants_queue(s, response, 4);
    }
}
static uint32_t elants_transfer(SSIPeripheral *dev, uint32_t value)
{
    ELANTSState *s = ELANTS(dev);
    if (!s->position) {
        s->reading = value == 0 || value == 0xff;
    }
    unsigned position = s->position++;
    if (s->reading) {
        uint8_t result = s->sent < s->queued ? s->rx[s->sent++] : 0xff;
        elants_irq(s);
        return result;
    }
    if (position < sizeof(s->command)) { s->command[position] = value; }
    return 0xff;
}
static int elants_cs(SSIPeripheral *dev, bool level)
{
    ELANTSState *s = ELANTS(dev);
    if (!level) {
        s->position = 0;
        memset(s->command, 0, sizeof(s->command));
    } else if (!s->reading && s->position == 6) {
        elants_command(s);
    }
    return 0;
}
static void elants_reset_pin(void *opaque, int n, int level)
{
    ELANTSState *s = opaque;
    if (!level) {
        s->reset_asserted = true;
        s->queued = s->sent = s->position = 0;
    } else if (s->reset_asserted) {
        const uint8_t hello[] = {0x55, 0x55, 0x55, 0x55};
        s->reset_asserted = false;
        elants_queue(s, hello, sizeof(hello));
    }
    elants_irq(s);
}

static void elants_input_packet(ELANTSState *s, bool in_range)
{
    uint8_t packet[42] = {0};
    if (s->reset_asserted) { return; }
    uint16_t x = qemu_input_scale_axis(INPUT_EVENT_ABS_MAX - s->y,
        INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX, 0, s->touch_mode ? 2832 : 15340);
    uint16_t y = qemu_input_scale_axis(s->x, INPUT_EVENT_ABS_MIN,
        INPUT_EVENT_ABS_MAX, 0, s->touch_mode ? 2064 : 11180);
    if (s->touch_mode) {
        packet[0] = 0x62;
        packet[1] = s->left;
        packet[3] = (x >> 4 & 0xf0) | (y >> 8 & 15);
        packet[4] = x;
        packet[5] = y;
        packet[35] = 0x30;
        for (unsigned i = 0; i < 34; i++) { packet[34] += packet[i]; }
    } else {
        packet[0] = 7;
        packet[1] = in_range | (s->left ? 2 : 0);
        stw_le_p(packet + 2, x);
        stw_le_p(packet + 4, y);
        stw_le_p(packet + 6, s->left ? 3072 : 0);
        packet[8] = 85;
        stw_le_p(packet + 15, 0x524d);
        for (unsigned i = 0; i < 41; i++) { packet[41] += packet[i]; }
    }
    elants_queue(s, packet, sizeof(packet));
}

static void elants_input_sync(DeviceState *dev)
{
    ELANTSState *s = ELANTS(dev);
    if (s->dirty) {
        s->dirty = false;
        elants_input_packet(s, true);
    }
}
static void elants_input_event(DeviceState *dev, QemuConsole *src, InputEvent *event)
{
    ELANTSState *s = ELANTS(dev);
    if (event->type == INPUT_EVENT_KIND_ABS) {
        if (event->u.abs.data->axis == INPUT_AXIS_X) {
            s->x = event->u.abs.data->value;
        } else {
            s->y = event->u.abs.data->value;
        }
        s->dirty = true;
    } else if (event->type == INPUT_EVENT_KIND_BTN) {
        if (event->u.btn.data->button == INPUT_BUTTON_LEFT) {
            s->left = event->u.btn.data->down;
            s->dirty = true;
        }
    } else if (event->type == INPUT_EVENT_KIND_KEY) {
        InputKeyEvent *key = event->u.key.data;
        QKeyCode code = qemu_input_key_value_to_qcode(key->key);
        if (code == Q_KEY_CODE_POWER) {
            qemu_set_irq(s->power_key, !key->down);
        } else if (key->down && (code == Q_KEY_CODE_T || code == Q_KEY_CODE_P)) {
            bool touch = code == Q_KEY_CODE_T;
            if (touch != s->touch_mode) {
                s->left = false;
                elants_input_packet(s, false);
                s->touch_mode = touch;
                s->dirty = true;
            }
        }
    }
}
static const QemuInputHandler elants_input_handler = {
    .name = "ELAN pen, touch and power button",
    .mask = INPUT_EVENT_MASK_KEY | INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN,
    .event = elants_input_event, .sync = elants_input_sync,
};
static void elants_realize(SSIPeripheral *dev, Error **errp)
{
    ELANTSState *s = ELANTS(dev);
    s->input = qemu_input_handler_register(DEVICE(dev), &elants_input_handler);
    qemu_input_handler_activate(s->input);
}
static void elants_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(ELANTS(dev)->input);
}
static void elants_reset(DeviceState *dev)
{
    ELANTSState *s = ELANTS(dev);
    s->sent = s->queued = s->position = 0;
    s->parent_obj.cs = true; /* GPIO chip-select has a board pull-up. */
    s->left = s->touch_mode = s->dirty = false;
    s->reset_asserted = true;
    s->idle_rate = 30;
    elants_irq(s);
    qemu_set_irq(s->power_key, 1);
}
static int elants_post_load(void *opaque, int version)
{
    elants_irq(opaque);
    return 0;
}
static const VMStateDescription elants_vmstate = {
    .name = TYPE_ELANTS, .version_id = 1, .minimum_version_id = 1,
    .post_load = elants_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, ELANTSState),
        VMSTATE_UINT8_ARRAY(rx, ELANTSState, 16384),
        VMSTATE_UINT8_ARRAY(command, ELANTSState, 8),
        VMSTATE_UINT16(queued, ELANTSState),
        VMSTATE_UINT16(sent, ELANTSState),
        VMSTATE_UINT16(position, ELANTSState),
        VMSTATE_UINT16(idle_rate, ELANTSState),
        VMSTATE_INT32(x, ELANTSState),
        VMSTATE_INT32(y, ELANTSState),
        VMSTATE_BOOL(reading, ELANTSState),
        VMSTATE_BOOL(reset_asserted, ELANTSState),
        VMSTATE_BOOL(left, ELANTSState),
        VMSTATE_BOOL(touch_mode, ELANTSState),
        VMSTATE_BOOL(dirty, ELANTSState),
        VMSTATE_END_OF_LIST()
    },
};
static const Property elants_properties[] = {
    DEFINE_PROP_UINT16("firmware-id", ELANTSState, firmware_id, 0x4100),
    DEFINE_PROP_UINT16("firmware-version", ELANTSState, firmware_version, 0x6712),
};
static void elants_init(Object *obj)
{
    ELANTSState *s = ELANTS(obj);
    qdev_init_gpio_in_named(DEVICE(obj), elants_reset_pin, "reset-n", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->power_key, "power-key", 1);
}
static void elants_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SSIPeripheralClass *sc = SSI_PERIPHERAL_CLASS(oc);
    sc->realize = elants_realize;
    sc->transfer = elants_transfer;
    sc->set_cs = elants_cs;
    sc->cs_polarity = SSI_CS_LOW;
    dc->unrealize = elants_unrealize;
    dc->vmsd = &elants_vmstate;
    device_class_set_props(dc, elants_properties);
    device_class_set_legacy_reset(dc, elants_reset);
}
static const TypeInfo elants_info = {
    .name = TYPE_ELANTS, .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(ELANTSState), .instance_init = elants_init,
    .class_init = elants_class_init,
};
static void elants_register(void) { type_register_static(&elants_info); }
type_init(elants_register)
