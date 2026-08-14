/*
 * Kobo Touch (Trilogy) Netronix board controller.
 *
 * The production kernel talks to an MSP430 at I2C address 0x43 through
 * msp430_read() and msp430_write().  Its error messages show a byte register
 * address and 16-bit values (for example, register 0x30 = 0xff00).  The
 * vendor driver is not included in the available source drops, so this model
 * deliberately represents only the observed transport contract: a persistent
 * 16-bit register file.  Hardware-specific battery, charger and RTC behaviour
 * can be added once the register definitions have been recovered.
 */

#include "qemu/osdep.h"
#include "hw/i2c/zforce.h"
#include "qemu/module.h"

#define NTXEC_REG_VERSION       0x00
#define NTXEC_REG_BATTERY_ADC   0x41
#define NTXEC_REG_SYSTEM_FLAGS  0x60
#define NTXEC_VERSION_KOBO_MINI 0xb83a
#define NTXEC_BATTERY_4000MV    993

struct KoboTouchMSP430State {
    I2CSlave parent_obj;
    uint16_t regs[256];
    uint8_t pointer;
    uint8_t write_data[2];
    uint8_t write_len;
    uint8_t read_pos;
    bool have_pointer;
};

static int kobotouch_msp430_send(I2CSlave *i2c, uint8_t data)
{
    KoboTouchMSP430State *s = KOBOTOUCH_MSP430(i2c);

    if (!s->have_pointer) {
        s->pointer = data;
        s->have_pointer = true;
    } else if (s->write_len < sizeof(s->write_data)) {
        s->write_data[s->write_len++] = data;
    }
    return 0;
}

static uint8_t kobotouch_msp430_recv(I2CSlave *i2c)
{
    KoboTouchMSP430State *s = KOBOTOUCH_MSP430(i2c);
    uint16_t value = s->regs[s->pointer];

    if (s->read_pos++ == 0) {
        return value >> 8;
    }
    return value;
}

static int kobotouch_msp430_event(I2CSlave *i2c, enum i2c_event event)
{
    KoboTouchMSP430State *s = KOBOTOUCH_MSP430(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->have_pointer = false;
        s->write_len = 0;
        break;
    case I2C_START_RECV:
        s->read_pos = 0;
        break;
    case I2C_FINISH:
        if (s->have_pointer && s->write_len == sizeof(s->write_data)) {
            s->regs[s->pointer] = (s->write_data[0] << 8) |
                                s->write_data[1];
        }
        break;
    default:
        break;
    }
    return 0;
}

static void kobotouch_msp430_reset(DeviceState *dev)
{
    KoboTouchMSP430State *s = KOBOTOUCH_MSP430(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[NTXEC_REG_VERSION] = NTXEC_VERSION_KOBO_MINI;
    s->regs[NTXEC_REG_BATTERY_ADC] = NTXEC_BATTERY_4000MV;
    s->regs[NTXEC_REG_SYSTEM_FLAGS] = 0;
    s->pointer = 0;
    s->write_len = 0;
    s->read_pos = 0;
    s->have_pointer = false;
}

static void kobotouch_msp430_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, kobotouch_msp430_reset);
    sc->send = kobotouch_msp430_send;
    sc->recv = kobotouch_msp430_recv;
    sc->event = kobotouch_msp430_event;
}

static const TypeInfo kobotouch_msp430_type = {
    .name = TYPE_KOBOTOUCH_MSP430,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(KoboTouchMSP430State),
    .class_init = kobotouch_msp430_class_init,
};

static void kobotouch_msp430_register_types(void)
{
    type_register_static(&kobotouch_msp430_type);
}

type_init(kobotouch_msp430_register_types)
