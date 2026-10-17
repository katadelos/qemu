/* Host input routing for reMarkable boards. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/i2c/cyttsp5.h"
#include "hw/i2c/wacom_emr.h"
#include "migration/vmstate.h"
#include "ui/input.h"
#define TYPE_REMARKABLE_INPUT "remarkable-input"
OBJECT_DECLARE_SIMPLE_TYPE(RemarkableInputState, REMARKABLE_INPUT)
struct RemarkableInputState {
    DeviceState parent_obj;
    QemuInputHandlerState *input;
    qemu_irq keys[4];
    uint8_t pressed, generation;
    DeviceState *pen, *touch;
    int32_t x, y;
    bool left, touch_mode, dirty;
};
static void remarkable_input_update(RemarkableInputState *s)
{
    for (unsigned i = 0; i < 4; i++) {
        qemu_set_irq(s->keys[i], !(s->pressed & BIT(i)));
    }
}
static void remarkable_input_sync(DeviceState *dev)
{
    RemarkableInputState *s = REMARKABLE_INPUT(dev);
    if (!s->dirty) {
        return;
    }
    s->dirty = false;
    if (s->touch_mode) {
        cyttsp5_set_input(s->touch, s->x, s->y, s->left);
    } else {
        uint16_t x = qemu_input_scale_axis(INPUT_EVENT_ABS_MAX - s->y,
            INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX, 0, 20967);
        uint16_t y = qemu_input_scale_axis(s->x,
            INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX, 0, 15725);
        if (s->generation == 2) {
            x = qemu_input_scale_axis(s->y, INPUT_EVENT_ABS_MIN,
                                       INPUT_EVENT_ABS_MAX, 0, 20966);
            y = qemu_input_scale_axis(INPUT_EVENT_ABS_MAX - s->x,
                                       INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
                                       0, 15725);
        }
        wacom_emr_set_input(s->pen, x, y, s->left ? 3072 : 0, true, false);
    }
}

static void remarkable_input_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    RemarkableInputState *s = REMARKABLE_INPUT(dev);
    InputKeyEvent *key;
    unsigned i;
    if (evt->type == INPUT_EVENT_KIND_ABS) {
        if (evt->u.abs.data->axis == INPUT_AXIS_X) {
            s->x = evt->u.abs.data->value;
        } else {
            s->y = evt->u.abs.data->value;
        }
        s->dirty = true;
        return;
    }
    if (evt->type == INPUT_EVENT_KIND_BTN) {
        if (evt->u.btn.data->button == INPUT_BUTTON_LEFT) {
            s->left = evt->u.btn.data->down;
        } else {
            return;
        }
        s->dirty = true;
        return;
    }
    key = evt->u.key.data;
    switch (qemu_input_key_value_to_qcode(key->key)) {
    case Q_KEY_CODE_LEFT:
    case Q_KEY_CODE_PGUP: i = 0; break;
    case Q_KEY_CODE_HOME: i = 1; break;
    case Q_KEY_CODE_RIGHT:
    case Q_KEY_CODE_PGDN: i = 2; break;
    case Q_KEY_CODE_POWER: i = 3; break;
    case Q_KEY_CODE_T:
    case Q_KEY_CODE_P:
        if (key->down) {
            bool touch = qemu_input_key_value_to_qcode(key->key) == Q_KEY_CODE_T;
            if (touch != s->touch_mode) {
                cyttsp5_set_input(s->touch, s->x, s->y, false);
                wacom_emr_set_input(s->pen, 0, 0, 0, false, false);
                s->left = false;
                s->touch_mode = touch;
            }
        }
        return;
    default: return;
    }
    if (s->generation == 2 && i != 3) {
        return;
    }
    s->pressed = deposit32(s->pressed, i, 1, key->down);
    remarkable_input_update(s);
}
static const QemuInputHandler remarkable_input_handler = {
    .name = "reMarkable pen, touch and buttons",
    .mask = INPUT_EVENT_MASK_KEY | INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN,
    .event = remarkable_input_event,
    .sync = remarkable_input_sync,
};
static void remarkable_input_realize(DeviceState *dev, Error **errp)
{
    RemarkableInputState *s = REMARKABLE_INPUT(dev);
    s->input = qemu_input_handler_register(dev, &remarkable_input_handler);
    qemu_input_handler_activate(s->input);
}
static void remarkable_input_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(REMARKABLE_INPUT(dev)->input);
}
static void remarkable_input_reset(DeviceState *dev)
{
    RemarkableInputState *s = REMARKABLE_INPUT(dev);
    s->pressed = 0;
    s->left = s->touch_mode = s->dirty = false;
    remarkable_input_update(s);
}
static int remarkable_input_post_load(void *opaque, int version)
{
    remarkable_input_update(opaque);
    return 0;
}
static const VMStateDescription remarkable_input_vmstate = {
    .name = TYPE_REMARKABLE_INPUT,
    .version_id = 1, .minimum_version_id = 1,
    .post_load = remarkable_input_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(pressed, RemarkableInputState),
        VMSTATE_INT32(x, RemarkableInputState),
        VMSTATE_INT32(y, RemarkableInputState),
        VMSTATE_BOOL(left, RemarkableInputState),
        VMSTATE_BOOL(touch_mode, RemarkableInputState),
        VMSTATE_BOOL(dirty, RemarkableInputState),
        VMSTATE_END_OF_LIST()
    },
};
static void remarkable_input_init(Object *obj)
{
    qdev_init_gpio_out(DEVICE(obj), REMARKABLE_INPUT(obj)->keys, 4);
}
static const Property remarkable_input_properties[] = {
    DEFINE_PROP_UINT8("generation", RemarkableInputState, generation, 1),
    DEFINE_PROP_LINK("pen", RemarkableInputState, pen, TYPE_WACOM_EMR, DeviceState *),
    DEFINE_PROP_LINK("touch", RemarkableInputState, touch, TYPE_CYTTSP5, DeviceState *),
};
static void remarkable_input_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = remarkable_input_realize;
    dc->unrealize = remarkable_input_unrealize;
    dc->user_creatable = false;
    dc->vmsd = &remarkable_input_vmstate;
    device_class_set_props(dc, remarkable_input_properties);
    device_class_set_legacy_reset(dc, remarkable_input_reset);
}

static const TypeInfo remarkable_input_info = {
    .name = TYPE_REMARKABLE_INPUT, .parent = TYPE_DEVICE,
    .instance_size = sizeof(RemarkableInputState),
    .instance_init = remarkable_input_init, .class_init = remarkable_input_class_init,
};
static void remarkable_input_register(void) { type_register_static(&remarkable_input_info); }
type_init(remarkable_input_register)
