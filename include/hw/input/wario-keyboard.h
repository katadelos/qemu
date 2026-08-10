#ifndef HW_INPUT_WARIO_KEYBOARD_H
#define HW_INPUT_WARIO_KEYBOARD_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "ui/input.h"

#define TYPE_WARIO_KEYBOARD "wario-keyboard"
OBJECT_DECLARE_SIMPLE_TYPE(WarioKeyboardState, WARIO_KEYBOARD)

#define WARIO_KEYBOARD_FIFO_SIZE 64

struct WarioKeyboardState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QemuInputHandlerState *input_handler;
    uint8_t fifo[WARIO_KEYBOARD_FIFO_SIZE * sizeof(uint32_t)];
    uint8_t head;
    uint8_t count;
};

#endif
