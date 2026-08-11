/* ROHM BD71815 PMIC register interface used by Lab126 Heisenberg. */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/i2c/bd71815.h"
#include "migration/vmstate.h"
#include "qemu/bcd.h"
#include "qemu/module.h"
#include "system/rtc.h"
#include "system/runstate.h"

#define BD71815_REG_DEVICE             0x00
#define BD71815_REG_PWRCTRL            0x01
#define BD71815_REG_GPO                0x1c
#define BD71815_REG_SEC                0x1e
#define BD71815_REG_MIN                0x1f
#define BD71815_REG_HOUR               0x20
#define BD71815_REG_WEEK               0x21
#define BD71815_REG_DAY                0x22
#define BD71815_REG_MONTH              0x23
#define BD71815_REG_YEAR               0x24
#define BD71815_REG_CONF               0x37
#define BD71815_REG_CHG_STATE          0x39
#define BD71815_REG_BAT_STAT           0x3b
#define BD71815_REG_DCIN_STAT          0x3c
#define BD71815_REG_VSYS_STAT          0x3d
#define BD71815_REG_BAT_TEMP           0x40
#define BD71815_REG_VM_VBAT_U          0x5d
#define BD71815_REG_VM_BTMP            0x5f
#define BD71815_REG_VM_OCV_PRE_U       0x67
#define BD71815_REG_VM_OCV_PST_U       0x6b
#define BD71815_REG_VM_SA_VBAT_U       0x6d
#define BD71815_REG_CC_CCNTD_3         0x79
#define BD71815_REG_INT_STAT           0x97
#define BD71815_REG_VM_VSYS_U          0xc0
#define BD71815_REG_VM_SA_VSYS_U       0xc2
#define BD71815_REG_VM_SA_VBAT_MIN_U   0xd4
#define BD71815_REG_VM_SA_VBAT_MAX_U   0xd6
#define BD71815_REG_VM_SA_VSYS_MIN_U   0xd8
#define BD71815_REG_VM_SA_VSYS_MAX_U   0xda
#define BD71815_REG_FULL_CCNTD_3       0xe8
#define BD71815_REG_TEST_MODE          0xfe

#define BD71815_RTC_24H                BIT(7)
#define BD71815_BAT_DET                BIT(5)
#define BD71815_BAT_DET_DONE           BIT(4)

struct BD71815State {
    I2CSlave parent_obj;
    qemu_irq irq;
    qemu_irq gpo[4];
    uint8_t regs[256];
    uint8_t pointer;
    uint8_t len;
    uint8_t test_step;
    int64_t rtc_offset;
};

static void bd71815_store_be16(BD71815State *s, unsigned reg,
                               uint16_t value)
{
    s->regs[reg] = value >> 8;
    s->regs[reg + 1] = value;
}

static void bd71815_store_be32(BD71815State *s, unsigned reg,
                               uint32_t value)
{
    s->regs[reg] = value >> 24;
    s->regs[reg + 1] = value >> 16;
    s->regs[reg + 2] = value >> 8;
    s->regs[reg + 3] = value;
}

static void bd71815_update_gpo(BD71815State *s)
{
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(s->gpo); i++) {
        qemu_set_irq(s->gpo[i], (s->regs[BD71815_REG_GPO] >> i) & 1);
    }
}

static void bd71815_rtc_capture(BD71815State *s)
{
    struct tm now;

    qemu_get_timedate(&now, s->rtc_offset);
    s->regs[BD71815_REG_SEC] = to_bcd(now.tm_sec);
    s->regs[BD71815_REG_MIN] = to_bcd(now.tm_min);
    s->regs[BD71815_REG_HOUR] = BD71815_RTC_24H | to_bcd(now.tm_hour);
    s->regs[BD71815_REG_WEEK] = to_bcd(now.tm_wday);
    s->regs[BD71815_REG_DAY] = to_bcd(now.tm_mday);
    s->regs[BD71815_REG_MONTH] = to_bcd(now.tm_mon + 1);
    s->regs[BD71815_REG_YEAR] = to_bcd(now.tm_year % 100);
}

static void bd71815_rtc_commit(BD71815State *s)
{
    struct tm now;

    qemu_get_timedate(&now, s->rtc_offset);
    now.tm_sec = from_bcd(s->regs[BD71815_REG_SEC] & 0x7f);
    now.tm_min = from_bcd(s->regs[BD71815_REG_MIN] & 0x7f);
    now.tm_hour = from_bcd(s->regs[BD71815_REG_HOUR] & 0x3f);
    now.tm_mday = from_bcd(s->regs[BD71815_REG_DAY] & 0x3f);
    now.tm_mon = from_bcd(s->regs[BD71815_REG_MONTH] & 0x1f) - 1;
    now.tm_year = from_bcd(s->regs[BD71815_REG_YEAR]) + 100;
    s->rtc_offset = qemu_timedate_diff(&now);
}

static int bd71815_send(I2CSlave *i2c, uint8_t data)
{
    BD71815State *s = BD71815(i2c);

    if (!s->len++) {
        s->pointer = data;
        return 0;
    }

    if (s->pointer == BD71815_REG_TEST_MODE) {
        static const uint8_t sequence[] = { 0x76, 0x66, 0x56 };

        if (data == 0) {
            s->test_step = 0;
            s->regs[s->pointer] = 0;
        } else if (s->test_step < ARRAY_SIZE(sequence) &&
                   data == sequence[s->test_step]) {
            if (++s->test_step == ARRAY_SIZE(sequence)) {
                s->regs[s->pointer] = 0xff;
            }
        } else {
            s->test_step = 0;
        }
    } else {
        s->regs[s->pointer] = data;
        if (s->pointer >= BD71815_REG_SEC &&
            s->pointer <= BD71815_REG_YEAR) {
            bd71815_rtc_commit(s);
        }
        if (s->pointer == BD71815_REG_GPO) {
            bd71815_update_gpo(s);
        }
        if (s->pointer == BD71815_REG_PWRCTRL && (data & 0x03)) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
    }
    s->pointer++;
    return 0;
}

static uint8_t bd71815_recv(I2CSlave *i2c)
{
    BD71815State *s = BD71815(i2c);

    if (s->pointer >= BD71815_REG_SEC &&
        s->pointer <= BD71815_REG_YEAR) {
        bd71815_rtc_capture(s);
    }
    return s->regs[s->pointer++];
}

static int bd71815_event(I2CSlave *i2c, enum i2c_event event)
{
    BD71815State *s = BD71815(i2c);

    if (event == I2C_START_SEND) {
        s->len = 0;
    }
    return 0;
}

static void bd71815_reset(DeviceState *dev)
{
    BD71815State *s = BD71815(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->len = 0;
    s->test_step = 0;
    s->rtc_offset = 0;

    /* Healthy battery, idle charger, and stable always-on power tree. */
    s->regs[BD71815_REG_DEVICE] = 0x15;
    s->regs[BD71815_REG_CONF] = 0x03;
    s->regs[BD71815_REG_BAT_STAT] =
        BD71815_BAT_DET | BD71815_BAT_DET_DONE;
    s->regs[BD71815_REG_BAT_TEMP] = 0;
    s->regs[BD71815_REG_CHG_STATE] = 0;
    s->regs[BD71815_REG_DCIN_STAT] = 0;
    s->regs[BD71815_REG_VSYS_STAT] = 0;
    s->regs[BD71815_REG_VM_BTMP] = 200 - 25;

    bd71815_store_be16(s, BD71815_REG_VM_VBAT_U, 4000);
    bd71815_store_be16(s, BD71815_REG_VM_SA_VBAT_U, 4000);
    bd71815_store_be16(s, BD71815_REG_VM_VSYS_U, 4000);
    bd71815_store_be16(s, BD71815_REG_VM_SA_VSYS_U, 4000);
    bd71815_store_be16(s, BD71815_REG_VM_SA_VBAT_MIN_U, 3950);
    bd71815_store_be16(s, BD71815_REG_VM_SA_VBAT_MAX_U, 4050);
    bd71815_store_be16(s, BD71815_REG_VM_SA_VSYS_MIN_U, 3950);
    bd71815_store_be16(s, BD71815_REG_VM_SA_VSYS_MAX_U, 4050);
    bd71815_store_be16(s, BD71815_REG_VM_OCV_PRE_U, 4000);
    bd71815_store_be16(s, BD71815_REG_VM_OCV_PST_U, 4000);
    bd71815_store_be32(s, BD71815_REG_CC_CCNTD_3, 728U << 16);
    bd71815_store_be32(s, BD71815_REG_FULL_CCNTD_3, 910U << 16);
    bd71815_rtc_capture(s);
    bd71815_update_gpo(s);
    qemu_set_irq(s->irq, 1);
}

static const VMStateDescription bd71815_vmstate = {
    .name = TYPE_BD71815,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, BD71815State),
        VMSTATE_UINT8_ARRAY(regs, BD71815State, 256),
        VMSTATE_UINT8(pointer, BD71815State),
        VMSTATE_UINT8(len, BD71815State),
        VMSTATE_UINT8(test_step, BD71815State),
        VMSTATE_INT64(rtc_offset, BD71815State),
        VMSTATE_END_OF_LIST()
    },
};

static void bd71815_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, bd71815_reset);
    dc->vmsd = &bd71815_vmstate;
    sc->send = bd71815_send;
    sc->recv = bd71815_recv;
    sc->event = bd71815_event;
}

static void bd71815_init(Object *obj)
{
    BD71815State *s = BD71815(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
    qdev_init_gpio_out_named(DEVICE(obj), s->gpo, "gpo", 4);
}

static const TypeInfo bd71815_info = {
    .name = TYPE_BD71815,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BD71815State),
    .instance_init = bd71815_init,
    .class_init = bd71815_class_init,
};

static void bd71815_register_types(void)
{
    type_register_static(&bd71815_info);
}
type_init(bd71815_register_types)
