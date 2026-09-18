/* Kobo Forma physical buttons, driven by the host keyboard.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/qdev.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/input.h"

#define TYPE_FORMA_BUTTONS "forma-buttons"
OBJECT_DECLARE_SIMPLE_TYPE(FormaButtonsState, FORMA_BUTTONS)
struct FormaButtonsState {
    DeviceState parent_obj;
    qemu_irq gpio[3];
    QemuInputHandlerState *input;
    uint8_t pressed;
};
static void forma_buttons_update(FormaButtonsState *s)
{
    for (unsigned i = 0; i < 3; i++) {
        qemu_set_irq(s->gpio[i], !(s->pressed & (1 << i)));
    }
}
static void forma_buttons_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    FormaButtonsState *s = FORMA_BUTTONS(dev);
    InputKeyEvent *key = evt->u.key.data;
    unsigned button;
    switch (qemu_input_key_value_to_qcode(key->key)) {
    case Q_KEY_CODE_PGUP: case Q_KEY_CODE_LEFT: button = 0; break;
    case Q_KEY_CODE_PGDN: case Q_KEY_CODE_RIGHT: button = 1; break;
    case Q_KEY_CODE_POWER: case Q_KEY_CODE_P: button = 2; break;
    default: return;
    }
    s->pressed = deposit32(s->pressed, button, 1, key->down);
    forma_buttons_update(s);
}
static const QemuInputHandler forma_buttons_handler = {
    .name = "Kobo Forma buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = forma_buttons_event,
};
static void forma_buttons_reset(DeviceState *dev)
{
    FormaButtonsState *s = FORMA_BUTTONS(dev);
    s->pressed = 0;
    forma_buttons_update(s);
}
static int forma_buttons_post_load(void *opaque, int version)
{
    forma_buttons_update(opaque);
    return 0;
}
static const VMStateDescription forma_buttons_vmstate = {
    .name = TYPE_FORMA_BUTTONS, .version_id = 1, .minimum_version_id = 1,
    .post_load = forma_buttons_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(pressed, FormaButtonsState),
        VMSTATE_END_OF_LIST()
    }
};
static void forma_buttons_realize(DeviceState *dev, Error **errp)
{
    FormaButtonsState *s = FORMA_BUTTONS(dev);
    s->input = qemu_input_handler_register(dev, &forma_buttons_handler);
}
static void forma_buttons_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(FORMA_BUTTONS(dev)->input);
}
static void forma_buttons_init(Object *obj)
{
    qdev_init_gpio_out(DEVICE(obj), FORMA_BUTTONS(obj)->gpio, 3);
}
static void forma_buttons_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = forma_buttons_realize;
    dc->unrealize = forma_buttons_unrealize;
    dc->vmsd = &forma_buttons_vmstate;
    device_class_set_legacy_reset(dc, forma_buttons_reset);
}
static const TypeInfo forma_buttons_type = {
    .name = TYPE_FORMA_BUTTONS, .parent = TYPE_DEVICE,
    .instance_size = sizeof(FormaButtonsState),
    .instance_init = forma_buttons_init,
    .class_init = forma_buttons_class_init,
};
static void forma_buttons_register_types(void)
{
    type_register_static(&forma_buttons_type);
}
type_init(forma_buttons_register_types)
