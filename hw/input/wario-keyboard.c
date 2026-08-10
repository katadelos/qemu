/* Host-keyboard mailbox for the Lab126 Wario machine. */

#include "qemu/osdep.h"
#include "hw/input/wario-keyboard.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/input.h"

static uint64_t wario_keyboard_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    WarioKeyboardState *s = opaque;
    uint32_t event;

    switch (offset) {
    case 0:
        return s->count;
    case 4:
        if (!s->count) {
            return 0;
        }
        memcpy(&event, &s->fifo[s->head * sizeof(event)], sizeof(event));
        s->head = (s->head + 1) % WARIO_KEYBOARD_FIFO_SIZE;
        s->count--;
        return event;
    default:
        return 0;
    }
}

static void wario_keyboard_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
}

static const MemoryRegionOps wario_keyboard_ops = {
    .read = wario_keyboard_read,
    .write = wario_keyboard_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void wario_keyboard_event(DeviceState *dev, QemuConsole *src,
                                 InputEvent *evt)
{
    WarioKeyboardState *s = WARIO_KEYBOARD(dev);
    InputKeyEvent *key = evt->u.key.data;
    int qcode = qemu_input_key_value_to_qcode(key->key);
    uint16_t linux_code;
    uint8_t tail;

    if (qcode >= qemu_input_map_qcode_to_linux_len) {
        return;
    }
    linux_code = qemu_input_map_qcode_to_linux[qcode];
    if (!linux_code) {
        return;
    }

    if (s->count == WARIO_KEYBOARD_FIFO_SIZE) {
        s->head = (s->head + 1) % WARIO_KEYBOARD_FIFO_SIZE;
        s->count--;
    }
    tail = (s->head + s->count) % WARIO_KEYBOARD_FIFO_SIZE;
    qcode = linux_code | ((uint32_t)key->down << 16);
    memcpy(&s->fifo[tail * sizeof(uint32_t)], &qcode, sizeof(uint32_t));
    s->count++;
}

static const QemuInputHandler wario_keyboard_handler = {
    .name = "Wario host keyboard",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = wario_keyboard_event,
};

static void wario_keyboard_reset(DeviceState *dev)
{
    WarioKeyboardState *s = WARIO_KEYBOARD(dev);

    s->head = 0;
    s->count = 0;
}

static void wario_keyboard_realize(DeviceState *dev, Error **errp)
{
    WarioKeyboardState *s = WARIO_KEYBOARD(dev);

    s->input_handler = qemu_input_handler_register(dev,
                                                   &wario_keyboard_handler);
}

static void wario_keyboard_unrealize(DeviceState *dev)
{
    WarioKeyboardState *s = WARIO_KEYBOARD(dev);

    qemu_input_handler_unregister(s->input_handler);
}

static const VMStateDescription wario_keyboard_vmstate = {
    .name = TYPE_WARIO_KEYBOARD,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BUFFER(fifo, WarioKeyboardState),
        VMSTATE_UINT8(head, WarioKeyboardState),
        VMSTATE_UINT8(count, WarioKeyboardState),
        VMSTATE_END_OF_LIST()
    },
};

static void wario_keyboard_init(Object *obj)
{
    WarioKeyboardState *s = WARIO_KEYBOARD(obj);

    memory_region_init_io(&s->iomem, obj, &wario_keyboard_ops, s,
                          TYPE_WARIO_KEYBOARD, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void wario_keyboard_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = wario_keyboard_realize;
    dc->unrealize = wario_keyboard_unrealize;
    device_class_set_legacy_reset(dc, wario_keyboard_reset);
    dc->vmsd = &wario_keyboard_vmstate;
}

static const TypeInfo wario_keyboard_info = {
    .name = TYPE_WARIO_KEYBOARD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(WarioKeyboardState),
    .instance_init = wario_keyboard_init,
    .class_init = wario_keyboard_class_init,
};

static void wario_keyboard_register_types(void)
{
    type_register_static(&wario_keyboard_info);
}

type_init(wario_keyboard_register_types)
