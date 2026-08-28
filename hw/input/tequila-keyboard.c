/*
 * Host keyboard adapter for the Kindle 4 (Tequila) physical controls.
 *
 * This device drives the same GPIO inputs as the real buttons.  The stock
 * fiveway.ko and tequila_keypad.ko drivers therefore remain responsible for
 * debounce, auto-repeat and Linux input event generation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/input/tequila-keyboard.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/input.h"

#define TEQUILA_FIVEWAY_PULSE_MS 20
#define TEQUILA_KEYPAD_PULSE_MS  100

static int tequila_keyboard_output_for_qcode(int qcode)
{
    switch (qcode) {
    case Q_KEY_CODE_UP:
        return TEQUILA_KEY_FIVEWAY_UP;
    case Q_KEY_CODE_DOWN:
        return TEQUILA_KEY_FIVEWAY_DOWN;
    case Q_KEY_CODE_LEFT:
        return TEQUILA_KEY_FIVEWAY_LEFT;
    case Q_KEY_CODE_RIGHT:
        return TEQUILA_KEY_FIVEWAY_RIGHT;
    case Q_KEY_CODE_RET:
    case Q_KEY_CODE_KP_ENTER:
        return TEQUILA_KEY_FIVEWAY_SELECT;
    case Q_KEY_CODE_BRACKET_LEFT:
        return TEQUILA_KEY_PAGE_PREVIOUS;
    case Q_KEY_CODE_BRACKET_RIGHT:
        return TEQUILA_KEY_PAGE_NEXT;
    case Q_KEY_CODE_K:
        return TEQUILA_KEY_KEYBOARD;
    case Q_KEY_CODE_M:
        return TEQUILA_KEY_MENU;
    case Q_KEY_CODE_H:
        return TEQUILA_KEY_HOME;
    case Q_KEY_CODE_B:
        return TEQUILA_KEY_BACK;
    case Q_KEY_CODE_POWER:
        return TEQUILA_KEY_POWER;
    default:
        return -1;
    }
}

static void tequila_keyboard_event(DeviceState *dev, QemuConsole *src,
                                   InputEvent *evt)
{
    TequilaKeyboardState *s = TEQUILA_KEYBOARD(dev);
    InputKeyEvent *key = evt->u.key.data;
    int qcode = qemu_input_key_value_to_qcode(key->key);
    int output = tequila_keyboard_output_for_qcode(qcode);

    if (output < 0) {
        return;
    }

    if (output == TEQUILA_KEY_POWER) {
        if (extract32(s->host_down, output, 1) == key->down) {
            return;
        }
        s->host_down = deposit32(s->host_down, output, 1, key->down);
        s->pressed = deposit32(s->pressed, output, 1, key->down);
        qemu_set_irq(s->outputs[output], key->down);
        return;
    }

    if (!key->down) {
        s->host_down = deposit32(s->host_down, output, 1, 0);
        return;
    }

    /*
     * Cocoa generates normal desktop key-repeat events while a key remains
     * down.  The Kindle's fiveway module has its own much shorter repeat
     * threshold, so passing through the host hold duration produces several
     * guest actions for a single deliberate tap.  Turn each distinct host
     * press into one debounced electrical pulse and ignore repeated key-downs
     * until Cocoa supplies the matching key-up.
     *
     * Keep the pulse below fiveway.ko's 60 ms debounce interval.  If the
     * release occurs after that interval, the driver re-enables the falling
     * edge IRQ and reports release both from the IRQ handler and again from
     * its debounce worker (RELEASED does not deassert line_state).  Releasing
     * while the IRQ is still masked leaves the worker to report it exactly
     * once, matching the physical switch's bounce-settling path.
     */
    if (extract32(s->host_down, output, 1)) {
        return;
    }
    s->host_down = deposit32(s->host_down, output, 1, 1);

    if (!extract32(s->pressed, output, 1)) {
        unsigned pulse_ms =
            output <= TEQUILA_KEY_FIVEWAY_SELECT ?
            TEQUILA_FIVEWAY_PULSE_MS : TEQUILA_KEYPAD_PULSE_MS;

        s->pressed = deposit32(s->pressed, output, 1, 1);
        qemu_set_irq(s->outputs[output], 1);
        timer_mod(s->release_timers[output],
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                  pulse_ms);
    }
}

static const QemuInputHandler tequila_keyboard_handler = {
    .name = "Tequila physical controls",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = tequila_keyboard_event,
};

static void tequila_keyboard_release(void *opaque)
{
    TequilaKeyboardTimer *context = opaque;
    TequilaKeyboardState *s = context->keyboard;
    unsigned output = context->output;

    s->pressed = deposit32(s->pressed, output, 1, 0);
    qemu_set_irq(s->outputs[output], 0);
}

static void tequila_keyboard_reset(DeviceState *dev)
{
    TequilaKeyboardState *s = TEQUILA_KEYBOARD(dev);
    unsigned i;

    s->host_down = 0;
    s->pressed = 0;
    for (i = 0; i < TEQUILA_KEY_COUNT; i++) {
        timer_del(s->release_timers[i]);
        qemu_set_irq(s->outputs[i], 0);
    }
}

static void tequila_keyboard_realize(DeviceState *dev, Error **errp)
{
    TequilaKeyboardState *s = TEQUILA_KEYBOARD(dev);
    unsigned i;

    for (i = 0; i < TEQUILA_KEY_COUNT; i++) {
        s->timer_contexts[i].keyboard = s;
        s->timer_contexts[i].output = i;
        s->release_timers[i] =
            timer_new_ms(QEMU_CLOCK_VIRTUAL, tequila_keyboard_release,
                         &s->timer_contexts[i]);
    }

    s->input_handler =
        qemu_input_handler_register(dev, &tequila_keyboard_handler);
}

static void tequila_keyboard_unrealize(DeviceState *dev)
{
    TequilaKeyboardState *s = TEQUILA_KEYBOARD(dev);
    unsigned i;

    qemu_input_handler_unregister(s->input_handler);
    for (i = 0; i < TEQUILA_KEY_COUNT; i++) {
        timer_free(s->release_timers[i]);
    }
}

static int tequila_keyboard_post_load(void *opaque, int version_id)
{
    TequilaKeyboardState *s = opaque;
    unsigned i;

    s->host_down = 0;
    s->pressed = 0;
    for (i = 0; i < TEQUILA_KEY_COUNT; i++) {
        qemu_set_irq(s->outputs[i], 0);
    }
    return 0;
}

static const VMStateDescription tequila_keyboard_vmstate = {
    .name = TYPE_TEQUILA_KEYBOARD,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = tequila_keyboard_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(host_down, TequilaKeyboardState),
        VMSTATE_UINT16(pressed, TequilaKeyboardState),
        VMSTATE_END_OF_LIST()
    },
};

static void tequila_keyboard_init(Object *obj)
{
    TequilaKeyboardState *s = TEQUILA_KEYBOARD(obj);

    qdev_init_gpio_out(DEVICE(obj), s->outputs, TEQUILA_KEY_COUNT);
}

static void tequila_keyboard_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = tequila_keyboard_realize;
    dc->unrealize = tequila_keyboard_unrealize;
    device_class_set_legacy_reset(dc, tequila_keyboard_reset);
    dc->vmsd = &tequila_keyboard_vmstate;
}

static const TypeInfo tequila_keyboard_info = {
    .name = TYPE_TEQUILA_KEYBOARD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TequilaKeyboardState),
    .instance_init = tequila_keyboard_init,
    .class_init = tequila_keyboard_class_init,
};

static void tequila_keyboard_register_types(void)
{
    type_register_static(&tequila_keyboard_info);
}

type_init(tequila_keyboard_register_types)
