/* NXP eLCDIF v4 scanout and frame/VSYNC interrupts.
 * Register interface: i.MX7D reference manual, LCDIF chapter.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/display/imx_lcdif.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#define CTRL 0
#define CTRL1 (0x10 / 4)
#define TRANSFER (0x30 / 4)
#define CURRENT (0x40 / 4)
#define NEXT (0x50 / 4)

void imx_lcdif_set_output(IMXLCDIFState *s, IMXLCDIFOutput output, void *opaque)
{
    s->output = output;
    s->output_opaque = opaque;
}
static void lcdif_irq(IMXLCDIFState *s)
{
    qemu_set_irq(s->irq, ((s->regs[CTRL1] >> 8) &
                         (s->regs[CTRL1] >> 12) & 15) != 0);
}
static bool lcdif_running(IMXLCDIFState *s)
{
    return (s->regs[CTRL] & (BIT(31) | BIT(30) | BIT(0))) == BIT(0);
}
static void lcdif_tick(void *opaque)
{
    IMXLCDIFState *s = opaque;
    if (!lcdif_running(s)) {
        return;
    }
    s->regs[CURRENT] = s->regs[NEXT];
    if (s->output && s->regs[CURRENT]) {
        s->output(s->output_opaque, s->regs[CURRENT],
                  s->regs[TRANSFER] & 0xffff, s->regs[TRANSFER] >> 16,
                  (s->regs[CTRL] >> 8) & 3);
    }
    /* Frame events are armed through their enables. Latching idle events
     * would complete a newly requested pan before NEXT reaches scanout. */
    s->regs[CTRL1] |= s->latch_masked_frame_events ? BIT(8) | BIT(9) :
                        (s->regs[CTRL1] >> 4) & (BIT(8) | BIT(9));
    lcdif_irq(s);
    /* VDCTRL1/2 carry the vertical/horizontal periods, in scan clocks. */
    uint64_t clocks = (uint64_t)s->regs[0x80 / 4] * (s->regs[0x90 / 4] & 0x3ffff);
    uint64_t period = clocks ? muldiv64(clocks, NANOSECONDS_PER_SECOND, s->pixel_clock) : 11862400;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              MIN(MAX(period, 1000000), NANOSECONDS_PER_SECOND));
}
static uint64_t lcdif_read(void *opaque, hwaddr offset, unsigned size)
{
    IMXLCDIFState *s = opaque;
    if (offset == 0x1a0) {
        return 0x04010000;
    }
    return s->regs[(offset & ~15) / 4];
}
static void lcdif_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    IMXLCDIFState *s = opaque;
    unsigned reg = (offset & ~15) / 4;
    switch (offset & 15) {
    case 0: s->regs[reg] = value; break;
    case 4: s->regs[reg] |= value; break;
    case 8: s->regs[reg] &= ~value; break;
    case 12: s->regs[reg] ^= value; break;
    }
    if (s->regs[CTRL] & BIT(31)) {
        /* SFTRST resets the entire block, including interrupt enables and
         * latched status; CLKGATE acknowledges completion of that reset. */
        memset(s->regs, 0, sizeof(s->regs));
        s->regs[CTRL] = BIT(31) | BIT(30);
    }
    if (lcdif_running(s)) {
        if (!timer_pending(s->timer)) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 11862400);
        }
    } else {
        timer_del(s->timer);
    }
    lcdif_irq(s);
}
static const MemoryRegionOps lcdif_ops = {
    .read = lcdif_read, .write = lcdif_write, .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static void lcdif_reset(DeviceState *dev)
{
    IMXLCDIFState *s = IMX_LCDIF(dev);
    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->timer);
    lcdif_irq(s);
}
static int lcdif_post_load(void *opaque, int version)
{
    lcdif_irq(opaque);
    return 0;
}
static const VMStateDescription lcdif_vmstate = {
    .name = TYPE_IMX_LCDIF, .version_id = 1, .minimum_version_id = 1,
    .post_load = lcdif_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMXLCDIFState, LCDIF_WORDS),
        VMSTATE_TIMER_PTR(timer, IMXLCDIFState),
        VMSTATE_END_OF_LIST()
    },
};
static void lcdif_init(Object *obj)
{
    IMXLCDIFState *s = IMX_LCDIF(obj);
    memory_region_init_io(&s->mmio, obj, &lcdif_ops, s, TYPE_IMX_LCDIF, 0x8000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lcdif_tick, s);
}
static void lcdif_finalize(Object *obj)
{
    timer_free(IMX_LCDIF(obj)->timer);
}
static void lcdif_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    static const Property properties[] = {
        DEFINE_PROP_UINT32("pixel-clock", IMXLCDIFState, pixel_clock, 40000000),
        DEFINE_PROP_BOOL("latch-masked-frame-events", IMXLCDIFState, latch_masked_frame_events, false),
    };
    device_class_set_props(dc, properties);
    dc->vmsd = &lcdif_vmstate;
    device_class_set_legacy_reset(dc, lcdif_reset);
}
static const TypeInfo lcdif_info = {
    .name = TYPE_IMX_LCDIF, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXLCDIFState), .instance_init = lcdif_init,
    .instance_finalize = lcdif_finalize, .class_init = lcdif_class_init,
};
static void lcdif_register(void) { type_register_static(&lcdif_info); }
type_init(lcdif_register)
