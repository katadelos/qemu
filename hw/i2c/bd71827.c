/* ROHM BD71827 PMIC register interface used by Lab126 Rex. */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/i2c/bd71827.h"
#include "migration/vmstate.h"
#include "qemu/bcd.h"
#include "qemu/module.h"
#include "system/rtc.h"
#include "system/runstate.h"

#define BD71827_REG_DEVICE          0x00
#define BD71827_REG_SEC             0x1e
#define BD71827_REG_MIN             0x1f
#define BD71827_REG_HOUR            0x20
#define BD71827_REG_WEEK            0x21
#define BD71827_REG_DAY             0x22
#define BD71827_REG_MONTH           0x23
#define BD71827_REG_YEAR            0x24
#define BD71827_REG_CONF            0x37
#define BD71827_REG_CHG_STATE       0x39
#define BD71827_REG_BAT_STAT        0x3b
#define BD71827_REG_DCIN_STAT       0x3c
#define BD71827_REG_BAT_TEMP        0x40
#define BD71827_REG_VM_VBAT_U       0x5d
#define BD71827_REG_VM_VBAT_L       0x5e
#define BD71827_REG_VM_BTMP         0x5f
#define BD71827_REG_VM_OCV_PRE_U    0x67
#define BD71827_REG_VM_OCV_PST_U    0x6b
#define BD71827_REG_VM_SA_VBAT_U    0x6d
#define BD71827_REG_VM_SA_VBAT_L    0x6e
#define BD71827_REG_CC_CCNTD_3      0x79
#define BD71827_REG_PWRCTRL3        0xa9
#define BD71827_REG_FULL_CCNTD_3    0xe8
#define BD71827_REG_VM_SA_VSYS_U    0xc2
#define BD71827_REG_BATID           0xc6
#define BD71827_REG_VM_SA_VBAT_MIN_U 0xd4
#define BD71827_REG_VM_SA_VBAT_MAX_U 0xd6
#define BD71827_REG_VM_SA_VSYS_MIN_U 0xd8
#define BD71827_REG_VM_SA_VSYS_MAX_U 0xda
#define BD71827_REG_VM_OCV_PWRON_U  0xdd
#define BD71827_REG_I2C_MAGIC       0xfe
#define BD71827_REG_PRODUCT         0xff

#define BD71827_RTC_24H             BIT(7)
#define BD71827_BAT_DET             BIT(5)
#define BD71827_BAT_DET_DONE        BIT(4)

struct BD71827State {
    I2CSlave parent_obj;
    qemu_irq irq;
    uint8_t regs[256];
    uint8_t pointer;
    uint8_t len;
    uint8_t magic_step;
    int64_t rtc_offset;
};

static void bd71827_store_be16(BD71827State *s, unsigned reg,
                               uint16_t value)
{
    s->regs[reg] = value >> 8;
    s->regs[reg + 1] = value;
}

static void bd71827_store_be32(BD71827State *s, unsigned reg,
                               uint32_t value)
{
    s->regs[reg] = value >> 24;
    s->regs[reg + 1] = value >> 16;
    s->regs[reg + 2] = value >> 8;
    s->regs[reg + 3] = value;
}

static void bd71827_rtc_capture(BD71827State *s)
{
    struct tm now;

    qemu_get_timedate(&now, s->rtc_offset);
    s->regs[BD71827_REG_SEC] = to_bcd(now.tm_sec);
    s->regs[BD71827_REG_MIN] = to_bcd(now.tm_min);
    s->regs[BD71827_REG_HOUR] = BD71827_RTC_24H | to_bcd(now.tm_hour);
    s->regs[BD71827_REG_WEEK] = to_bcd(now.tm_wday);
    s->regs[BD71827_REG_DAY] = to_bcd(now.tm_mday);
    s->regs[BD71827_REG_MONTH] = to_bcd(now.tm_mon + 1);
    s->regs[BD71827_REG_YEAR] = to_bcd(now.tm_year % 100);
}

static void bd71827_rtc_commit(BD71827State *s)
{
    struct tm now;

    qemu_get_timedate(&now, s->rtc_offset);
    now.tm_sec = from_bcd(s->regs[BD71827_REG_SEC] & 0x7f);
    now.tm_min = from_bcd(s->regs[BD71827_REG_MIN] & 0x7f);
    now.tm_hour = from_bcd(s->regs[BD71827_REG_HOUR] & 0x3f);
    now.tm_mday = from_bcd(s->regs[BD71827_REG_DAY] & 0x3f);
    now.tm_mon = from_bcd(s->regs[BD71827_REG_MONTH] & 0x1f) - 1;
    now.tm_year = from_bcd(s->regs[BD71827_REG_YEAR]) + 100;
    s->rtc_offset = qemu_timedate_diff(&now);
}

static int bd71827_send(I2CSlave *i2c, uint8_t data)
{
    BD71827State *s = BD71827(i2c);

    if (!s->len++) {
        s->pointer = data;
        return 0;
    }

    if (s->pointer == BD71827_REG_I2C_MAGIC) {
        static const uint8_t sequence[] = { 0x76, 0x66, 0x56 };

        if (data == 0) {
            s->magic_step = 0;
            s->regs[s->pointer] = 0;
        } else if (s->magic_step < ARRAY_SIZE(sequence) &&
                   data == sequence[s->magic_step]) {
            if (++s->magic_step == ARRAY_SIZE(sequence)) {
                s->regs[s->pointer] = 0xff;
            }
        } else {
            s->magic_step = 0;
        }
    } else {
        s->regs[s->pointer] = data;
        if (s->pointer >= BD71827_REG_SEC &&
            s->pointer <= BD71827_REG_YEAR) {
            bd71827_rtc_commit(s);
        }
        if (s->pointer == BD71827_REG_PWRCTRL3 && (data & 0x03)) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
    }
    s->pointer++;
    return 0;
}

static uint8_t bd71827_recv(I2CSlave *i2c)
{
    BD71827State *s = BD71827(i2c);

    if (s->pointer >= BD71827_REG_SEC &&
        s->pointer <= BD71827_REG_YEAR) {
        bd71827_rtc_capture(s);
    }
    return s->regs[s->pointer++];
}

static int bd71827_event(I2CSlave *i2c, enum i2c_event event)
{
    BD71827State *s = BD71827(i2c);

    if (event == I2C_START_SEND) {
        s->len = 0;
    }
    return 0;
}

static void bd71827_reset(DeviceState *dev)
{
    BD71827State *s = BD71827(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->len = 0;
    s->magic_step = 0;
    s->rtc_offset = 0;

    /* A healthy, idle battery and power tree with no external charger. */
    s->regs[BD71827_REG_DEVICE] = 0x27;
    s->regs[BD71827_REG_PRODUCT] = 0x10;
    s->regs[BD71827_REG_CONF] = 0x03;
    s->regs[BD71827_REG_BAT_STAT] =
        BD71827_BAT_DET | BD71827_BAT_DET_DONE;
    s->regs[BD71827_REG_BAT_TEMP] = 0;
    s->regs[BD71827_REG_CHG_STATE] = 0;
    s->regs[BD71827_REG_DCIN_STAT] = 0;

    bd71827_store_be16(s, BD71827_REG_VM_VBAT_U, 4000);
    bd71827_store_be16(s, BD71827_REG_VM_SA_VBAT_U, 4000);
    bd71827_store_be16(s, BD71827_REG_VM_SA_VSYS_U, 4000);
    bd71827_store_be16(s, BD71827_REG_VM_SA_VBAT_MIN_U, 3950);
    bd71827_store_be16(s, BD71827_REG_VM_SA_VBAT_MAX_U, 4050);
    bd71827_store_be16(s, BD71827_REG_VM_SA_VSYS_MIN_U, 3950);
    bd71827_store_be16(s, BD71827_REG_VM_SA_VSYS_MAX_U, 4050);
    bd71827_store_be16(s, BD71827_REG_VM_OCV_PRE_U, 4000);
    bd71827_store_be16(s, BD71827_REG_VM_OCV_PST_U, 4000);
    bd71827_store_be16(s, BD71827_REG_VM_OCV_PWRON_U, 4000);
    s->regs[BD71827_REG_VM_BTMP] = 200 - 25;
    s->regs[BD71827_REG_BATID] = 0x22;

    /*
     * The coulomb counters are 16.16 fixed point.  Lab126 derives SOC from
     * the upper halfword; unshifted A/10-second values therefore look like
     * an empty battery even though all voltage ADCs report 4 V.
     */
    bd71827_store_be32(s, BD71827_REG_CC_CCNTD_3, 1296U << 16);
    bd71827_store_be32(s, BD71827_REG_FULL_CCNTD_3, 1620U << 16);
    bd71827_rtc_capture(s);
    qemu_set_irq(s->irq, 1);
}

static const VMStateDescription bd71827_vmstate = {
    .name = TYPE_BD71827,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, BD71827State),
        VMSTATE_UINT8_ARRAY(regs, BD71827State, 256),
        VMSTATE_UINT8(pointer, BD71827State),
        VMSTATE_UINT8(len, BD71827State),
        VMSTATE_UINT8(magic_step, BD71827State),
        VMSTATE_INT64(rtc_offset, BD71827State),
        VMSTATE_END_OF_LIST()
    },
};

static void bd71827_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, bd71827_reset);
    dc->vmsd = &bd71827_vmstate;
    sc->send = bd71827_send;
    sc->recv = bd71827_recv;
    sc->event = bd71827_event;
}

static void bd71827_init(Object *obj)
{
    BD71827State *s = BD71827(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
}

static const TypeInfo bd71827_info = {
    .name = TYPE_BD71827,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BD71827State),
    .instance_init = bd71827_init,
    .class_init = bd71827_class_init,
};

static void bd71827_register_types(void)
{
    type_register_static(&bd71827_info);
}
type_init(bd71827_register_types)
