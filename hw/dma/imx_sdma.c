/* i.MX SDMA host interface and peripheral-buffer scripts.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/dma/imx_sdma.h"
#include "hw/core/irq.h"
#include "exec/cpu-common.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "system/address-spaces.h"

static uint32_t sdma_load(hwaddr address)
{
    return address_space_ldl_le(&address_space_memory, address, MEMTXATTRS_UNSPECIFIED, NULL);
}
static void sdma_store(hwaddr address, uint32_t value)
{
    address_space_stl_le(&address_space_memory, address, value, MEMTXATTRS_UNSPECIFIED, NULL);
}
static void sdma_irq(IMXSDMAState *s) { qemu_set_irq(s->irq, s->regs[1] != 0); }
static void sdma_channel0(IMXSDMAState *s)
{
    hwaddr bd = sdma_load(s->regs[0]);
    uint32_t mode = sdma_load(bd);
    hwaddr buffer = sdma_load(bd + 4);
    uint32_t address = sdma_load(bd + 8);
    unsigned count = mode & 0xffff;
    unsigned command = mode >> 24;
    uint8_t *memory = command == 4 || command == 8 ? s->program : s->data;
    unsigned unit = command == 4 || command == 8 ? 2 : 4;
    size_t capacity = command == 4 || command == 8 ? sizeof(s->program) : sizeof(s->data);
    size_t offset = address * unit, length = count * unit;
    if (offset <= capacity && length <= capacity - offset) {
        if (command == 2 || command == 3 || command == 8) {
            cpu_physical_memory_write(buffer, memory + offset, length);
        } else {
            cpu_physical_memory_read(buffer, memory + offset, length);
        }
    } else {
        mode |= BIT(20); /* descriptor error */
    }
    sdma_store(bd, mode & ~BIT(16));
    s->active &= ~1;
    s->regs[1] |= 1;
    sdma_irq(s);
}

static bool sdma_transfer_one(IMXSDMAState *s, unsigned channel)
{
    hwaddr ccb = s->regs[0] + channel * 16;
    hwaddr bd = sdma_load(ccb);
    uint32_t mode = sdma_load(bd);
    uint32_t status = (mode >> 16) & 255;
    unsigned count = mode & 0xffff;
    if (!(status & 1) || !count) { return false; }
    hwaddr memory = sdma_load(bd + 4) + s->progress[channel];
    const uint8_t *context = s->data + (2048 + channel * 32) * 4;
    hwaddr port = ldl_le_p(context + (2 + 6) * 4); /* GR6 peripheral address */
    unsigned width = mode >> 24;
    unsigned done = 0;
    if (!width) { width = 4; }
    if (width > 4) { return false; }
    unsigned remaining = count - s->progress[channel];
    unsigned length = MIN(remaining, width);
    uint8_t bytes[4] = {0};
    bool uart = (port & 0xffff0000) == 0x30860000 ||
                (port & 0xffff0000) == 0x30890000 ||
                (port & 0xffff0000) == 0x30880000 ||
                (port & 0xffff0000) == 0x30a60000;
    bool spi = port >= 0x30820000 && port < 0x30850000;
    if (uart && (port & 0xffff) == 0) {
        if (!(sdma_load(port + 0x98) & 1)) { return false; }
        stl_le_p(bytes, sdma_load(port));
        cpu_physical_memory_write(memory, bytes, length);
        done = length;
    } else if (uart && (port & 0xffff) == 0x40) {
        cpu_physical_memory_read(memory, bytes, length);
        sdma_store(port, ldl_le_p(bytes));
        done = length;
    } else if (spi && (port & 0xffff) == 0) {
        if (!(sdma_load(port + 0x18) & BIT(3))) { return false; }
        stl_le_p(bytes, sdma_load(port));
        cpu_physical_memory_write(memory, bytes, length);
        done = length;
    } else if (spi && (port & 0xffff) == 4) {
        if (sdma_load(port - 4 + 0x18) & BIT(2)) { return false; }
        cpu_physical_memory_read(memory, bytes, length);
        sdma_store(port, ldl_le_p(bytes));
        done = length;
    } else if (!port) {
        hwaddr target = sdma_load(bd + 8) + s->progress[channel];
        cpu_physical_memory_read(memory, bytes, length);
        cpu_physical_memory_write(target, bytes, length);
        done = length;
    }
    if (!done) { return false; }
    s->progress[channel] += done;
    bool uart_idle = uart && !(port & 0xffff) && !(sdma_load(port + 0x98) & 1);
    if (s->progress[channel] == count || uart_idle) {
        if (uart_idle) { mode = (mode & ~0xffff) | s->progress[channel]; }
        sdma_store(bd, mode & ~BIT(16));
        sdma_store(ccb, status & 2 ? sdma_load(ccb + 4) : bd + 12);
        s->progress[channel] = 0;
        if (!(status & 4)) { s->active &= ~BIT(channel); }
        if (status & 8) { s->regs[1] |= BIT(channel); }
        sdma_irq(s);
    }
    return true;
}
static void sdma_tick(void *opaque)
{
    IMXSDMAState *s = opaque;
    /* Interleave transmit and receive so a peripheral FIFO cannot overrun
     * while both sides of a full-duplex SPI transfer are active. */
    for (unsigned pass = 0; pass < 64; pass++) {
        bool moved = false;
        for (unsigned channel = 1; channel < 32; channel++) {
            if (s->active & BIT(channel)) { moved |= sdma_transfer_one(s, channel); }
        }
        if (!moved) { break; }
    }
    if (s->active) {
        timer_mod_ns(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
    }
}
static uint64_t sdma_read(void *opaque, hwaddr offset, unsigned size)
{
    IMXSDMAState *s = opaque;
    if (offset == 8) { return s->active; }
    if (offset == 0x24) { return 0; }
    return s->regs[offset / 4];
}
static void sdma_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    IMXSDMAState *s = opaque;
    if (offset == 4) {
        s->regs[1] &= ~value;
        sdma_irq(s);
    } else if (offset == 8) {
        s->active &= ~value;
    } else if (offset == 0xc) {
        s->active |= value;
        if (value & 1) { sdma_channel0(s); }
        if (s->active) {
            timer_mod_ns(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
        }
    } else if (offset == 0x24 && (value & 1)) {
        s->active = 0;
        s->regs[1] = 0;
        memset(s->progress, 0, sizeof(s->progress));
        timer_del(&s->timer);
        sdma_irq(s);
    } else {
        s->regs[offset / 4] = value;
    }
}
static const MemoryRegionOps sdma_ops = {
    .read = sdma_read, .write = sdma_write, .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static void sdma_reset(DeviceState *dev)
{
    IMXSDMAState *s = IMX_SDMA(dev);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->progress, 0, sizeof(s->progress));
    s->active = 0;
    timer_del(&s->timer);
    sdma_irq(s);
}
static const VMStateDescription sdma_vmstate = {
    .name = TYPE_IMX_SDMA, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMXSDMAState, 0x10000 / 4),
        VMSTATE_UINT32(active, IMXSDMAState),
        VMSTATE_UINT8_ARRAY(data, IMXSDMAState, 0x4000),
        VMSTATE_UINT8_ARRAY(program, IMXSDMAState, 0x20000),
        VMSTATE_UINT32_ARRAY(progress, IMXSDMAState, 32),
        VMSTATE_TIMER(timer, IMXSDMAState),
        VMSTATE_END_OF_LIST()
    },
};
static void sdma_init(Object *obj)
{
    IMXSDMAState *s = IMX_SDMA(obj);
    memory_region_init_io(&s->memory, obj, &sdma_ops, s, TYPE_IMX_SDMA, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->memory);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, sdma_tick, s);
}
static void sdma_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->vmsd = &sdma_vmstate;
    device_class_set_legacy_reset(dc, sdma_reset);
}
static const TypeInfo sdma_info = {
    .name = TYPE_IMX_SDMA, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXSDMAState), .instance_init = sdma_init,
    .class_init = sdma_class_init,
};
static void sdma_register(void) { type_register_static(&sdma_info); }
type_init(sdma_register)
