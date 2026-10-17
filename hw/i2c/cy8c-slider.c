/*
 * Cypress CY8C21434 capacitive slider on the PocketBook Color Lux 801.
 * The vendor cy8c21434_touch driver reads position register 0x12 after
 * a falling interrupt edge.  Positions 1..14 select the slider segments;
 * zero represents no contact.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/cy8c-slider.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/module.h"

#define CY8C_POSITION 0x12
#define CY8C_MAX_POSITION 14

OBJECT_DECLARE_SIMPLE_TYPE(CY8CSliderState, CY8C_SLIDER)

struct CY8CSliderState {
    I2CSlave parent_obj;
    qemu_irq irq;
    uint8_t pointer;
    uint8_t position;
    bool expect_pointer;
    bool powered;
    bool reset_asserted;
    bool pending;
};

static bool cy8c_slider_enabled(CY8CSliderState *s)
{
    return s->powered && !s->reset_asserted;
}

static void cy8c_slider_update_irq(CY8CSliderState *s)
{
    qemu_set_irq(s->irq, !(cy8c_slider_enabled(s) && s->pending));
}

static void cy8c_slider_reset_state(CY8CSliderState *s)
{
    s->pointer = 0;
    s->position = 0;
    s->expect_pointer = true;
    s->pending = false;
    cy8c_slider_update_irq(s);
}

void cy8c_slider_set_position(DeviceState *dev, uint8_t position)
{
    CY8CSliderState *s = CY8C_SLIDER(dev);

    if (position > CY8C_MAX_POSITION || !cy8c_slider_enabled(s) ||
        s->position == position) {
        return;
    }
    s->position = position;
    s->pending = true;
    cy8c_slider_update_irq(s);
}

void cy8c_slider_step(DeviceState *dev, int delta)
{
    CY8CSliderState *s = CY8C_SLIDER(dev);
    int64_t position = (int64_t)s->position + delta;

    cy8c_slider_set_position(dev, CLAMP(position, 1, CY8C_MAX_POSITION));
}

static int cy8c_slider_send(I2CSlave *i2c, uint8_t data)
{
    CY8CSliderState *s = CY8C_SLIDER(i2c);

    if (s->expect_pointer) {
        s->pointer = data;
        s->expect_pointer = false;
    } else {
        /* The position register is read-only. */
        s->pointer++;
    }
    return 0;
}

static uint8_t cy8c_slider_recv(I2CSlave *i2c)
{
    CY8CSliderState *s = CY8C_SLIDER(i2c);
    uint8_t value = 0;

    if (s->pointer == CY8C_POSITION) {
        value = s->position;
        s->pending = false;
        cy8c_slider_update_irq(s);
    }
    s->pointer++;
    return value;
}

static int cy8c_slider_event(I2CSlave *i2c, enum i2c_event event)
{
    CY8CSliderState *s = CY8C_SLIDER(i2c);

    if (event == I2C_START_SEND || event == I2C_START_RECV) {
        if (!cy8c_slider_enabled(s)) {
            return -1;
        }
        if (event == I2C_START_SEND) {
            s->expect_pointer = true;
        }
    }
    return 0;
}

static void cy8c_slider_power(void *opaque, int n, int level)
{
    CY8CSliderState *s = opaque;

    if (s->powered != !!level) {
        s->powered = !!level;
        cy8c_slider_reset_state(s);
    }
}

static void cy8c_slider_reset_pin(void *opaque, int n, int level)
{
    CY8CSliderState *s = opaque;

    if (s->reset_asserted != !!level) {
        s->reset_asserted = !!level;
        cy8c_slider_reset_state(s);
    }
}

static void cy8c_slider_reset(DeviceState *dev)
{
    cy8c_slider_reset_state(CY8C_SLIDER(dev));
}

static void cy8c_slider_get_position(Object *obj, Visitor *v,
                                     const char *name, void *opaque,
                                     Error **errp)
{
    uint8_t position = CY8C_SLIDER(obj)->position;

    visit_type_uint8(v, name, &position, errp);
}

static void cy8c_slider_put_position(Object *obj, Visitor *v,
                                     const char *name, void *opaque,
                                     Error **errp)
{
    uint8_t position;

    if (!visit_type_uint8(v, name, &position, errp)) {
        return;
    }
    if (position > CY8C_MAX_POSITION) {
        error_setg(errp, "Slider position must be between 0 and 14");
        return;
    }
    cy8c_slider_set_position(DEVICE(obj), position);
}

static int cy8c_slider_post_load(void *opaque, int version_id)
{
    CY8CSliderState *s = opaque;

    if (s->position > CY8C_MAX_POSITION) {
        return -EINVAL;
    }
    cy8c_slider_update_irq(s);
    return 0;
}

static const VMStateDescription cy8c_slider_vmstate = {
    .name = TYPE_CY8C_SLIDER,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = cy8c_slider_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, CY8CSliderState),
        VMSTATE_UINT8(pointer, CY8CSliderState),
        VMSTATE_UINT8(position, CY8CSliderState),
        VMSTATE_BOOL(expect_pointer, CY8CSliderState),
        VMSTATE_BOOL(powered, CY8CSliderState),
        VMSTATE_BOOL(reset_asserted, CY8CSliderState),
        VMSTATE_BOOL(pending, CY8CSliderState),
        VMSTATE_END_OF_LIST()
    },
};

static void cy8c_slider_init(Object *obj)
{
    CY8CSliderState *s = CY8C_SLIDER(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
    qdev_init_gpio_in_named(DEVICE(obj), cy8c_slider_power, "power", 1);
    qdev_init_gpio_in_named(DEVICE(obj), cy8c_slider_reset_pin, "reset", 1);
    object_property_add(obj, "position", "uint8", cy8c_slider_get_position,
                        cy8c_slider_put_position, NULL, NULL);
}

static void cy8c_slider_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, cy8c_slider_reset);
    dc->vmsd = &cy8c_slider_vmstate;
    sc->send = cy8c_slider_send;
    sc->recv = cy8c_slider_recv;
    sc->event = cy8c_slider_event;
}

static const TypeInfo cy8c_slider_info = {
    .name = TYPE_CY8C_SLIDER,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(CY8CSliderState),
    .instance_init = cy8c_slider_init,
    .class_init = cy8c_slider_class_init,
};

static void cy8c_slider_register_types(void)
{
    type_register_static(&cy8c_slider_info);
}
type_init(cy8c_slider_register_types)
