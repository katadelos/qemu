/* FocalTech FT5536G capacitive touchscreen controller model. */

#include "qemu/osdep.h"
#include "hw/i2c/ft5536g.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define FT5536G_CHIP_ID_HIGH_REG  0xa3
#define FT5536G_CHIP_ID_LOW_REG   0x9f
#define FT5536G_FW_VERSION_REG    0xa6
#define FT5536G_VENDOR_ID_REG     0xa8
#define FT5536G_MODULE_ID_REG     0xe3

#define FT5536G_CHIP_ID_HIGH      0x54
#define FT5536G_CHIP_ID_LOW       0x52
#define FT5536G_BOOTLOADER_ID_LOW 0x5c
#define FT5536G_COLOR_VENDOR_ID   0x60
#define FT5536G_COMMAND_MAX       32
#define FT5536G_CONFIG_SIZE       32

typedef enum FT5536GResponse {
    FT5536G_RESPONSE_REGISTERS,
    FT5536G_RESPONSE_HID,
    FT5536G_RESPONSE_BOOT_ID,
    FT5536G_RESPONSE_CONFIG,
} FT5536GResponse;

struct FT5536GState {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t command[FT5536G_COMMAND_MAX];
    uint8_t command_len;
    uint8_t pointer;
    uint8_t response_pos;
    uint8_t response;
    bool sending;
    bool reset_level;
    bool bootloader;
    bool upgrade_aa;
    qemu_irq irq;
};

static void ft5536g_register_reset(FT5536GState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[FT5536G_CHIP_ID_HIGH_REG] = FT5536G_CHIP_ID_HIGH;
    s->regs[FT5536G_CHIP_ID_LOW_REG] = FT5536G_CHIP_ID_LOW;
    /* A maximum version prevents the stock auto-updater rewriting the model. */
    s->regs[FT5536G_FW_VERSION_REG] = 0xff;
    s->regs[FT5536G_VENDOR_ID_REG] = FT5536G_COLOR_VENDOR_ID;
    s->regs[FT5536G_MODULE_ID_REG] = 0;
    s->pointer = 0;
    s->command_len = 0;
    s->response_pos = 0;
    s->response = FT5536G_RESPONSE_REGISTERS;
    s->sending = false;
    s->bootloader = false;
    s->upgrade_aa = false;
}

static void ft5536g_finish_command(FT5536GState *s)
{
    static const uint8_t hid_command[] = { 0xeb, 0xaa, 0x09 };
    static const uint8_t boot_id_command[] = { 0x90, 0x00, 0x00, 0x00 };

    if (!s->command_len) {
        return;
    }

    s->pointer = s->command[0];
    s->response = FT5536G_RESPONSE_REGISTERS;
    if (s->command_len == sizeof(hid_command) &&
        !memcmp(s->command, hid_command, sizeof(hid_command))) {
        s->response = FT5536G_RESPONSE_HID;
    } else if (s->command[0] == boot_id_command[0] &&
               (s->command_len == 1 ||
                (s->command_len == sizeof(boot_id_command) &&
                 !memcmp(s->command, boot_id_command,
                         sizeof(boot_id_command))))) {
        s->response = FT5536G_RESPONSE_BOOT_ID;
    } else if ((s->command_len == 1 && s->command[0] == 0xa8) ||
               (s->command_len == 4 && s->command[0] == 0x03)) {
        s->response = FT5536G_RESPONSE_CONFIG;
    } else if (s->command_len == 1 && s->command[0] == 0x07) {
        s->bootloader = false;
        s->upgrade_aa = false;
    } else {
        for (unsigned i = 1; i < s->command_len; i++) {
            s->regs[s->pointer++] = s->command[i];
        }
        if (s->command_len == 2 && s->command[0] == 0xfc) {
            if (s->command[1] == 0xaa) {
                s->upgrade_aa = true;
            } else if (s->command[1] == 0x55 && s->upgrade_aa) {
                s->bootloader = true;
                s->upgrade_aa = false;
            }
        }
        s->pointer = s->command[0];
    }
    s->response_pos = 0;
    s->command_len = 0;
}

static int ft5536g_send(I2CSlave *i2c, uint8_t data)
{
    FT5536GState *s = FT5536G(i2c);

    if (s->command_len < sizeof(s->command)) {
        s->command[s->command_len++] = data;
    }
    return 0;
}

static uint8_t ft5536g_recv(I2CSlave *i2c)
{
    FT5536GState *s = FT5536G(i2c);
    static const uint8_t hid_response[] = { 0xeb, 0xaa, 0x08 };
    uint8_t config[FT5536G_CONFIG_SIZE] = { 0 };

    switch (s->response) {
    case FT5536G_RESPONSE_HID:
        return s->response_pos < sizeof(hid_response) ?
               hid_response[s->response_pos++] : 0;
    case FT5536G_RESPONSE_BOOT_ID:
        if (s->response_pos++ == 0) {
            return FT5536G_CHIP_ID_HIGH;
        }
        return s->response_pos == 2 ?
               (s->bootloader ? FT5536G_BOOTLOADER_ID_LOW :
                                FT5536G_CHIP_ID_LOW) : 0;
    case FT5536G_RESPONSE_CONFIG:
        /* Color-panel module 0x0060, with each ID followed by its inverse. */
        config[4] = FT5536G_COLOR_VENDOR_ID;
        config[5] = ~FT5536G_COLOR_VENDOR_ID;
        config[30] = 0;
        config[31] = 0xff;
        return s->response_pos < sizeof(config) ?
               config[s->response_pos++] : 0;
    case FT5536G_RESPONSE_REGISTERS:
    default:
        return s->regs[s->pointer++];
    }
}

static int ft5536g_event(I2CSlave *i2c, enum i2c_event event)
{
    FT5536GState *s = FT5536G(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->command_len = 0;
        s->sending = true;
        break;
    case I2C_START_RECV:
        if (s->sending) {
            ft5536g_finish_command(s);
        }
        s->sending = false;
        break;
    case I2C_FINISH:
        if (s->sending) {
            ft5536g_finish_command(s);
        }
        s->sending = false;
        break;
    default:
        break;
    }
    return 0;
}

static void ft5536g_reset_input(void *opaque, int line, int level)
{
    FT5536GState *s = opaque;

    if (!level && s->reset_level) {
        ft5536g_register_reset(s);
    }
    s->reset_level = level;
}

static void ft5536g_reset(DeviceState *dev)
{
    FT5536GState *s = FT5536G(dev);

    ft5536g_register_reset(s);
    s->reset_level = false;
    qemu_set_irq(s->irq, 1);
}

static const VMStateDescription ft5536g_vmstate = {
    .name = TYPE_FT5536G,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FT5536GState),
        VMSTATE_UINT8_ARRAY(regs, FT5536GState, 256),
        VMSTATE_UINT8_ARRAY(command, FT5536GState, FT5536G_COMMAND_MAX),
        VMSTATE_UINT8(command_len, FT5536GState),
        VMSTATE_UINT8(pointer, FT5536GState),
        VMSTATE_UINT8(response_pos, FT5536GState),
        VMSTATE_UINT8(response, FT5536GState),
        VMSTATE_BOOL(sending, FT5536GState),
        VMSTATE_BOOL(reset_level, FT5536GState),
        VMSTATE_BOOL(bootloader, FT5536GState),
        VMSTATE_BOOL(upgrade_aa, FT5536GState),
        VMSTATE_END_OF_LIST()
    },
};

static void ft5536g_init(Object *obj)
{
    FT5536GState *s = FT5536G(obj);

    qdev_init_gpio_in_named(DEVICE(obj), ft5536g_reset_input, "reset", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
}

static void ft5536g_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, ft5536g_reset);
    dc->vmsd = &ft5536g_vmstate;
    sc->send = ft5536g_send;
    sc->recv = ft5536g_recv;
    sc->event = ft5536g_event;
}

static const TypeInfo ft5536g_info = {
    .name = TYPE_FT5536G,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FT5536GState),
    .instance_init = ft5536g_init,
    .class_init = ft5536g_class_init,
};

static void ft5536g_register_types(void)
{
    type_register_static(&ft5536g_info);
}
type_init(ft5536g_register_types)
