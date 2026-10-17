/* i.MX7 system counter. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#define TYPE_IMX7_SYSCNT "imx7-syscnt"
OBJECT_DECLARE_SIMPLE_TYPE(IMX7SyscntState, IMX7_SYSCNT)
struct IMX7SyscntState {
    SysBusDevice parent_obj;
    MemoryRegion control, read, compare;
    qemu_irq irq;
    QEMUTimer timer;
    uint32_t cr, frequency;
    uint32_t cmpcr;
    uint64_t cmpcv;
    uint64_t count;
    int64_t epoch;
};
static void syscnt_schedule(IMX7SyscntState *s);
static uint64_t syscnt_count(IMX7SyscntState *s)
{
    return s->count + ((s->cr & 1) ?
        muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->epoch,
                 s->frequency, NANOSECONDS_PER_SECOND) : 0);
}
static uint64_t syscnt_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX7SyscntState *s = opaque;
    switch (offset) {
    case 0: return s->cr;
    case 4: return s->cr & 0x300; /* Frequency-selection acknowledge. */
    case 8: return syscnt_count(s) & UINT32_MAX;
    case 12: return syscnt_count(s) >> 32;
    case 0x20: return s->frequency;
    default: return 0;
    }
}
static uint64_t syscnt_read_frame(void *opaque, hwaddr offset, unsigned size)
{
    uint64_t count = syscnt_count(opaque);
    return offset == 8 ? count & UINT32_MAX : offset == 12 ? count >> 32 : 0;
}
static void syscnt_expired(void *opaque)
{
    IMX7SyscntState *s = opaque;
    s->cmpcr |= 4;
    qemu_set_irq(s->irq, !(s->cmpcr & 2));
}
static void syscnt_schedule(IMX7SyscntState *s)
{
    timer_del(&s->timer);
    if (!(s->cmpcr & 1)) {
        s->cmpcr &= ~4;
    } else if ((s->cr & 1) && s->frequency) {
        uint64_t count = syscnt_count(s);
        if (count >= s->cmpcv) {
            s->cmpcr |= 4;
        } else {
            timer_mod_ns(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                muldiv64(s->cmpcv - count, NANOSECONDS_PER_SECOND, s->frequency));
        }
    }
    qemu_set_irq(s->irq, (s->cmpcr & 7) == 5);
}
static uint64_t syscnt_compare_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX7SyscntState *s = opaque;
    switch (offset) {
    case 0x20: return s->cmpcv & UINT32_MAX;
    case 0x24: return s->cmpcv >> 32;
    case 0x2c: return s->cmpcr;
    default: return 0;
    }
}
static void syscnt_compare_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    IMX7SyscntState *s = opaque;
    switch (offset) {
    case 0x20: s->cmpcv = deposit64(s->cmpcv, 0, 32, value); break;
    case 0x24: s->cmpcv = deposit64(s->cmpcv, 32, 24, value); break;
    case 0x2c: s->cmpcr = (s->cmpcr & 4) | (value & 3); break;
    }
    syscnt_schedule(s);
}
static void syscnt_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    IMX7SyscntState *s = opaque;
    s->count = syscnt_count(s);
    s->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    switch (offset) {
    case 0: s->cr = value; break;
    case 8: s->count = deposit64(s->count, 0, 32, value); break;
    case 12: s->count = deposit64(s->count, 32, 32, value); break;
    case 0x20: s->frequency = value; break;
    }
    syscnt_schedule(s);
}
static const MemoryRegionOps syscnt_control_ops = {
    .read = syscnt_read, .write = syscnt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static const MemoryRegionOps syscnt_read_ops = {
    .read = syscnt_read_frame,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static const MemoryRegionOps syscnt_compare_ops = {
    .read = syscnt_compare_read, .write = syscnt_compare_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static void syscnt_reset(DeviceState *dev)
{
    IMX7SyscntState *s = IMX7_SYSCNT(dev);
    s->cr = 1;
    s->frequency = 8000000;
    s->count = 0;
    s->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->cmpcr = 0;
    s->cmpcv = 0;
    syscnt_schedule(s);
}
static const VMStateDescription syscnt_vmstate = {
    .name = TYPE_IMX7_SYSCNT, .version_id = 2, .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr, IMX7SyscntState),
        VMSTATE_UINT32(frequency, IMX7SyscntState),
        VMSTATE_UINT64(count, IMX7SyscntState),
        VMSTATE_INT64(epoch, IMX7SyscntState),
        VMSTATE_UINT32(cmpcr, IMX7SyscntState),
        VMSTATE_UINT64(cmpcv, IMX7SyscntState),
        VMSTATE_TIMER(timer, IMX7SyscntState),
        VMSTATE_END_OF_LIST()
    },
};
static void syscnt_init(Object *obj)
{
    IMX7SyscntState *s = IMX7_SYSCNT(obj);
    memory_region_init_io(&s->control, obj, &syscnt_control_ops, s,
                          TYPE_IMX7_SYSCNT ".control", 0x1000);
    memory_region_init_io(&s->read, obj, &syscnt_read_ops, s,
                          TYPE_IMX7_SYSCNT ".read", 0x1000);
    memory_region_init_io(&s->compare, obj, &syscnt_compare_ops, s,
                          TYPE_IMX7_SYSCNT ".compare", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->control);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->read);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->compare);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, syscnt_expired, s);
}
static void syscnt_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_legacy_reset(dc, syscnt_reset);
    dc->vmsd = &syscnt_vmstate;
}
static const TypeInfo syscnt_info = {
    .name = TYPE_IMX7_SYSCNT, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX7SyscntState),
    .instance_init = syscnt_init, .class_init = syscnt_class_init,
};
static void syscnt_register(void) { type_register_static(&syscnt_info); }
type_init(syscnt_register)
