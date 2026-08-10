/*
 * Freescale MC13892 power-management IC
 *
 * The register transport is a 32-bit, active-high-CS SPI frame containing a
 * write flag, a six-bit register number, and 24 bits of register data.
 */
#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/ssi/mc13892.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "system/runstate.h"
#include "ui/input.h"
#include "trace.h"

#define MC13892_NUM_REGS       64
#define MC13892_REG_INT_STAT0   0
#define MC13892_REG_INT_MASK0   1
#define MC13892_REG_INT_STAT1   3
#define MC13892_REG_INT_MASK1   4
#define MC13892_REG_INT_SENSE1  5
#define MC13892_REG_IDENT       7
#define MC13892_REG_ADC1       44
#define MC13892_REG_ADC2       45
#define MC13892_REG_MASK        0x00ffffff
#define MC13892_ADC1_ASC        0x00100000
#define MC13892_ADC_BATT_RAW    832
#define MC13892_PWRON1          (1U << 3)

struct MC13892State {
    SSIPeripheral parent_obj;
    qemu_irq irq;
    uint32_t regs[MC13892_NUM_REGS];
    uint32_t tx_frame;
    uint32_t read_value;
    uint8_t byte;
    uint8_t reg;
    bool write;
    bool power_down;
    QemuInputHandlerState *input_handler;
};

static void mc13892_update_irq(MC13892State *s);

static void mc13892_input_event(DeviceState *dev, QemuConsole *src,
                                InputEvent *evt)
{
    MC13892State *s = MC13892(dev);
    InputKeyEvent *key = evt->u.key.data;
    int qcode = qemu_input_key_value_to_qcode(key->key);

    if (qcode != Q_KEY_CODE_POWER || s->power_down == key->down) {
        return;
    }

    s->power_down = key->down;
    if (key->down) {
        s->regs[MC13892_REG_INT_SENSE1] &= ~MC13892_PWRON1;
    } else {
        s->regs[MC13892_REG_INT_SENSE1] |= MC13892_PWRON1;
    }
    s->regs[MC13892_REG_INT_STAT1] |= MC13892_PWRON1;
    trace_whitney_mc13892_power(key->down,
                                s->regs[MC13892_REG_INT_SENSE1],
                                s->regs[MC13892_REG_INT_STAT1],
                                s->regs[MC13892_REG_INT_MASK1]);
    mc13892_update_irq(s);
    if (key->down && !(s->regs[MC13892_REG_INT_MASK1] & MC13892_PWRON1)) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
    }
}

static const QemuInputHandler mc13892_input_handler = {
    .name = "MC13892 PWRON1 button",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = mc13892_input_event,
};

static void mc13892_update_irq(MC13892State *s)
{
    bool pending = (s->regs[MC13892_REG_INT_STAT0] &
                    ~s->regs[MC13892_REG_INT_MASK0]) ||
                   (s->regs[MC13892_REG_INT_STAT1] &
                    ~s->regs[MC13892_REG_INT_MASK1]);

    trace_whitney_mc13892_irq(pending,
                              s->regs[MC13892_REG_INT_STAT1],
                              s->regs[MC13892_REG_INT_MASK1]);
    qemu_set_irq(s->irq, pending);
}

static void mc13892_write_register(MC13892State *s, uint8_t reg,
                                   uint32_t value)
{
    value &= MC13892_REG_MASK;
    switch (reg) {
    case MC13892_REG_INT_STAT0:
    case MC13892_REG_INT_STAT1:
        s->regs[reg] &= ~value;
        break;
    case MC13892_REG_IDENT:
        break;
    default:
        s->regs[reg] = value;
        break;
    }

    if (reg == MC13892_REG_ADC1 && (value & MC13892_ADC1_ASC)) {
        /* Complete the single battery-voltage conversion immediately. */
        s->regs[MC13892_REG_ADC1] &= ~MC13892_ADC1_ASC;
        s->regs[MC13892_REG_ADC2] = MC13892_ADC_BATT_RAW << 2;
    }
    mc13892_update_irq(s);
}

static uint32_t mc13892_transfer(SSIPeripheral *peripheral, uint32_t value)
{
    MC13892State *s = MC13892(peripheral);
    uint8_t input = value;
    uint8_t output = 0;

    if (s->byte == 0) {
        s->write = input & 0x80;
        s->reg = (input >> 1) & 0x3f;
        s->tx_frame = input;
        s->read_value = s->regs[s->reg] & MC13892_REG_MASK;
    } else {
        s->tx_frame = (s->tx_frame << 8) | input;
        output = s->read_value >> (8 * (3 - s->byte));
    }

    if (++s->byte == 4) {
        if (s->write) {
            mc13892_write_register(s, s->reg, s->tx_frame);
        }
        s->byte = 0;
    }

    return output;
}

static int mc13892_set_cs(SSIPeripheral *peripheral, bool select)
{
    MC13892State *s = MC13892(peripheral);

    if (!select) {
        s->byte = 0;
    }
    return 0;
}

static void mc13892_reset(DeviceState *dev)
{
    MC13892State *s = MC13892(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[MC13892_REG_INT_MASK0] = MC13892_REG_MASK;
    s->regs[MC13892_REG_INT_MASK1] = MC13892_REG_MASK;
    /* PWRON1 is active low and the physical button powers up released. */
    s->regs[MC13892_REG_INT_SENSE1] = MC13892_PWRON1;
    /* MC13892 revision 2.0A, as fitted to production Whitney boards. */
    s->regs[MC13892_REG_IDENT] = 0x0045d0;
    s->tx_frame = 0;
    s->read_value = 0;
    s->byte = 0;
    s->reg = 0;
    s->write = false;
    s->power_down = false;
    mc13892_update_irq(s);
}

static void mc13892_realize(SSIPeripheral *peripheral, Error **errp)
{
    MC13892State *s = MC13892(peripheral);

    qdev_init_gpio_out_named(DEVICE(peripheral), &s->irq, "irq", 1);
    s->input_handler = qemu_input_handler_register(
        DEVICE(peripheral), &mc13892_input_handler);
}

static void mc13892_unrealize(DeviceState *dev)
{
    MC13892State *s = MC13892(dev);

    qemu_input_handler_unregister(s->input_handler);
}

static int mc13892_post_load(void *opaque, int version_id)
{
    mc13892_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_mc13892 = {
    .name = TYPE_MC13892,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = mc13892_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, MC13892State),
        VMSTATE_UINT32_ARRAY(regs, MC13892State, MC13892_NUM_REGS),
        VMSTATE_UINT32(tx_frame, MC13892State),
        VMSTATE_UINT32(read_value, MC13892State),
        VMSTATE_UINT8(byte, MC13892State),
        VMSTATE_UINT8(reg, MC13892State),
        VMSTATE_BOOL(write, MC13892State),
        VMSTATE_BOOL(power_down, MC13892State),
        VMSTATE_END_OF_LIST()
    }
};

static void mc13892_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *ssc = SSI_PERIPHERAL_CLASS(klass);

    ssc->realize = mc13892_realize;
    ssc->transfer = mc13892_transfer;
    ssc->set_cs = mc13892_set_cs;
    ssc->cs_polarity = SSI_CS_HIGH;
    dc->unrealize = mc13892_unrealize;
    dc->vmsd = &vmstate_mc13892;
    device_class_set_legacy_reset(dc, mc13892_reset);
}

static const TypeInfo mc13892_info = {
    .name = TYPE_MC13892,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(MC13892State),
    .class_init = mc13892_class_init,
};

static void mc13892_register_types(void)
{
    type_register_static(&mc13892_info);
}

type_init(mc13892_register_types)
