/*
 * Host keyboard adapter for the Kindle 4 (Tequila) physical controls.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_INPUT_TEQUILA_KEYBOARD_H
#define HW_INPUT_TEQUILA_KEYBOARD_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define TYPE_TEQUILA_KEYBOARD "tequila-keyboard"
OBJECT_DECLARE_SIMPLE_TYPE(TequilaKeyboardState, TEQUILA_KEYBOARD)

enum {
    TEQUILA_KEY_FIVEWAY_UP,
    TEQUILA_KEY_FIVEWAY_DOWN,
    TEQUILA_KEY_FIVEWAY_LEFT,
    TEQUILA_KEY_FIVEWAY_RIGHT,
    TEQUILA_KEY_FIVEWAY_SELECT,
    TEQUILA_KEY_PAGE_PREVIOUS,
    TEQUILA_KEY_PAGE_NEXT,
    TEQUILA_KEY_KEYBOARD,
    TEQUILA_KEY_MENU,
    TEQUILA_KEY_HOME,
    TEQUILA_KEY_BACK,
    TEQUILA_KEY_POWER,
    TEQUILA_KEY_COUNT,
};

typedef struct TequilaKeyboardTimer {
    TequilaKeyboardState *keyboard;
    unsigned output;
} TequilaKeyboardTimer;

struct TequilaKeyboardState {
    SysBusDevice parent_obj;
    QemuInputHandlerState *input_handler;
    qemu_irq outputs[TEQUILA_KEY_COUNT];
    QEMUTimer *release_timers[TEQUILA_KEY_COUNT];
    TequilaKeyboardTimer timer_contexts[TEQUILA_KEY_COUNT];
    uint16_t host_down;
    uint16_t pressed;
};

#endif
