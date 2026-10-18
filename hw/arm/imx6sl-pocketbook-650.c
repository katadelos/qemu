/*
 * PocketBook Ultra 650, Foxconn i.MX6SoloLite board.
 * Wiring: pocketbook/Platform_MX6 Kernel and Uboot_650 vendor sources.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/arm/fsl-imx6.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/split-irq.h"
#include "hw/display/imx_epdc.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/msg2133.h"
#include "hw/i2c/zforce.h"
#include "hw/misc/imx6sl_iomuxc.h"
#include "hw/misc/imx6sl_mmdc.h"
#include "hw/misc/imx6sl_pxp.h"
#include "hw/misc/imx6sll_ocotp.h"
#include "hw/misc/imx_rngc.h"
#include "hw/sd/sd.h"
#include "hw/usb/imx-usb-misc.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "system/block-backend.h"
#include "system/dma.h"
#include "system/qtest.h"
#include "system/reset.h"
#include "ui/input.h"

#define PB650_RAM_BASE 0x80000000
#define PB650_RAM_SIZE (512 * MiB)
#define TYPE_PB650_MACHINE MACHINE_TYPE_NAME("imx6sl-pocketbook-650")
OBJECT_DECLARE_SIMPLE_TYPE(PB650MachineState, PB650_MACHINE)

struct PB650MachineState {
    MachineState parent_obj;
    FslIMX6State *soc;
    uint32_t entry;
};

/* The two board PMICs share the byte-register I2C transport. */
#define TYPE_PB650_REGS "pocketbook-650-i2c-registers"
#define TYPE_PB650_PFUZE "pocketbook-650-pfuze100"
#define TYPE_PB650_BATTERY "pocketbook-650-cw2015"
OBJECT_DECLARE_SIMPLE_TYPE(PB650RegsState, PB650_REGS)

struct PB650RegsState {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t pointer;
    bool have_pointer;
};

static int pb650_regs_send(I2CSlave *i2c, uint8_t value)
{
    PB650RegsState *s = PB650_REGS(i2c);

    if (!s->have_pointer) {
        s->pointer = value;
        s->have_pointer = true;
    } else {
        s->regs[s->pointer++] = value;
    }
    return 0;
}

static uint8_t pb650_regs_recv(I2CSlave *i2c)
{
    PB650RegsState *s = PB650_REGS(i2c);

    return s->regs[s->pointer++];
}

static int pb650_regs_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        PB650_REGS(i2c)->have_pointer = false;
    }
    return 0;
}

static void pb650_regs_reset(DeviceState *dev)
{
    PB650RegsState *s = PB650_REGS(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->have_pointer = false;
}

static void pb650_pfuze_reset(DeviceState *dev)
{
    PB650RegsState *s = PB650_REGS(dev);

    pb650_regs_reset(dev);
    s->regs[0] = 0x10; /* PF0100 device ID */
    s->regs[3] = 0x11;
    s->regs[0x20] = s->regs[0x2e] = 0x2d;
    s->regs[0x35] = 0x2d;
}

static void pb650_battery_reset(DeviceState *dev)
{
    PB650RegsState *s = PB650_REGS(dev);

    pb650_regs_reset(dev);
    s->regs[0] = 0x70;
    s->regs[2] = 0x33; /* About 4 V at 305 uV per count. */
    s->regs[3] = 0x48;
    s->regs[4] = 85;   /* Integer part of the 8.8 percentage. */
    s->regs[6] = 2;
    s->regs[7] = 0xd0; /* 720 minutes remaining, no low-battery alert. */
}

static const VMStateDescription pb650_regs_vmstate = {
    .name = TYPE_PB650_REGS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, PB650RegsState),
        VMSTATE_UINT8_ARRAY(regs, PB650RegsState, 256),
        VMSTATE_UINT8(pointer, PB650RegsState),
        VMSTATE_BOOL(have_pointer, PB650RegsState),
        VMSTATE_END_OF_LIST()
    },
};

static void pb650_regs_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(oc);

    dc->vmsd = &pb650_regs_vmstate;
    ic->send = pb650_regs_send;
    ic->recv = pb650_regs_recv;
    ic->event = pb650_regs_event;
}

static void pb650_pfuze_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), pb650_pfuze_reset);
}

static void pb650_battery_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), pb650_battery_reset);
}

#define TYPE_PB650_BUTTONS "pocketbook-650-buttons"
OBJECT_DECLARE_SIMPLE_TYPE(PB650ButtonsState, PB650_BUTTONS)

struct PB650ButtonsState {
    DeviceState parent_obj;
    QemuInputHandlerState *input;
    qemu_irq outputs[8];
    uint8_t pressed;
};

static void pb650_buttons_update(PB650ButtonsState *s)
{
    for (unsigned i = 0; i < ARRAY_SIZE(s->outputs); i++) {
        /* Power and cover are active low; the six navigation keys high. */
        qemu_set_irq(s->outputs[i], !!(s->pressed & BIT(i)) ^ (i >= 6));
    }
}

static void pb650_buttons_event(DeviceState *dev, QemuConsole *src,
                                InputEvent *evt)
{
    PB650ButtonsState *s = PB650_BUTTONS(dev);
    InputKeyEvent *key = evt->u.key.data;
    unsigned i;

    switch (qemu_input_key_value_to_qcode(key->key)) {
    case Q_KEY_CODE_M:
    case Q_KEY_CODE_ESC:   i = 0; break;
    case Q_KEY_CODE_HOME:  i = 1; break;
    case Q_KEY_CODE_LEFT:  i = 2; break;
    case Q_KEY_CODE_RIGHT: i = 3; break;
    case Q_KEY_CODE_UP:
    case Q_KEY_CODE_PGUP:  i = 4; break;
    case Q_KEY_CODE_DOWN:
    case Q_KEY_CODE_PGDN:  i = 5; break;
    case Q_KEY_CODE_P:
    case Q_KEY_CODE_POWER: i = 6; break;
    case Q_KEY_CODE_H:     i = 7; break;
    default: return;
    }
    s->pressed = deposit32(s->pressed, i, 1, key->down);
    pb650_buttons_update(s);
}

static const QemuInputHandler pb650_buttons_handler = {
    .name = "PocketBook Ultra 650 buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = pb650_buttons_event,
};

static void pb650_buttons_realize(DeviceState *dev, Error **errp)
{
    PB650_BUTTONS(dev)->input = qemu_input_handler_register(
        dev, &pb650_buttons_handler);
}

static void pb650_buttons_unrealize(DeviceState *dev)
{
    qemu_input_handler_unregister(PB650_BUTTONS(dev)->input);
}

static void pb650_buttons_reset(DeviceState *dev)
{
    PB650ButtonsState *s = PB650_BUTTONS(dev);

    s->pressed = 0;
    pb650_buttons_update(s);
}

static int pb650_buttons_post_load(void *opaque, int version_id)
{
    pb650_buttons_update(opaque);
    return 0;
}

static const VMStateDescription pb650_buttons_vmstate = {
    .name = TYPE_PB650_BUTTONS,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = pb650_buttons_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(pressed, PB650ButtonsState),
        VMSTATE_END_OF_LIST()
    },
};

static void pb650_buttons_init(Object *obj)
{
    PB650ButtonsState *s = PB650_BUTTONS(obj);

    qdev_init_gpio_out(DEVICE(obj), s->outputs, ARRAY_SIZE(s->outputs));
}

static void pb650_buttons_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = pb650_buttons_realize;
    dc->unrealize = pb650_buttons_unrealize;
    dc->vmsd = &pb650_buttons_vmstate;
    device_class_set_legacy_reset(dc, pb650_buttons_reset);
}

/* Host-command SDMA, DCP and Vivante control registers used by this BSP. */
#define TYPE_PB650_IO "pocketbook-650-system-io"
OBJECT_DECLARE_SIMPLE_TYPE(PB650IOState, PB650_IO)

struct PB650IOState {
    SysBusDevice parent_obj;
    MemoryRegion sdma_iomem, dcp_iomem, gpu_iomem;
    qemu_irq irq;
    qemu_irq vg_irq;
    QEMUTimer *vg_timer;
    uint32_t vg_address, vg_remaining;
    qemu_irq wifi_power;
    uint8_t wifi_controls;
    uint32_t sdma[0x1000 / 4];
    uint32_t dcp[0x4000 / 4];
    uint32_t gpu[0x8000 / 4];
};

static void pb650_wifi_control(void *opaque, int line, int level)
{
    PB650IOState *s = opaque;

    s->wifi_controls = deposit32(s->wifi_controls, line, 1, !!level);
    qemu_set_irq(s->wifi_power, s->wifi_controls == 3);
}

static uint64_t pb650_sdma_read(void *opaque, hwaddr offset, unsigned size)
{
    return PB650_IO(opaque)->sdma[offset / 4];
}

static void pb650_sdma_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    PB650IOState *s = opaque;
    uint32_t descriptor, mode;

    switch (offset) {
    case 4: /* H_INTR, write one to clear. */
        s->sdma[1] &= ~value;
        qemu_set_irq(s->irq, s->sdma[1] != 0);
        break;
    case 0xc: /* H_START, channel zero loads the host command. */
        if ((value & 1) &&
            dma_memory_read(&address_space_memory, s->sdma[0] + 4,
                            &descriptor, 4, MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
            descriptor = le32_to_cpu(descriptor);
            if (descriptor && dma_memory_read(&address_space_memory, descriptor,
                &mode, 4, MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
                mode = cpu_to_le32(le32_to_cpu(mode) & ~BIT(16));
                dma_memory_write(&address_space_memory, descriptor, &mode, 4,
                                 MEMTXATTRS_UNSPECIFIED);
            }
            s->sdma[1] |= 1;
            qemu_set_irq(s->irq, 1);
        }
        break;
    case 0x24: /* RESET */
        memset(s->sdma, 0, sizeof(s->sdma));
        qemu_set_irq(s->irq, 0);
        break;
    default:
        s->sdma[offset / 4] = value;
        break;
    }
}

static uint64_t pb650_dcp_read(void *opaque, hwaddr offset, unsigned size)
{
    return PB650_IO(opaque)->dcp[offset / 4];
}

static void pb650_dcp_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    PB650IOState *s = opaque;
    unsigned base = offset & ~0xf;
    uint32_t v = s->dcp[base / 4];

    switch (offset & 0xc) {
    case 0: v = value; break;
    case 4: v |= value; break;
    case 8: v &= ~value; break;
    case 12: v ^= value; break;
    }
    if (!base && (v & BIT(31))) {
        memset(s->dcp, 0, sizeof(s->dcp));
        v = BIT(31) | BIT(30);
    }
    s->dcp[base / 4] = v;
}

#define PB650_VG_BASE 0x4000
#define PB650_VG_STEP_NS 1000

static void pb650_vg_update_irq(PB650IOState *s)
{
    qemu_set_irq(s->vg_irq,
        (s->gpu[(PB650_VG_BASE + 0x10) / 4] &
         s->gpu[(PB650_VG_BASE + 0x14) / 4]) != 0);
}

static bool pb650_vg_fetch(uint32_t address, uint32_t *value)
{
    if (address < PB650_RAM_BASE ||
        address > PB650_RAM_BASE + PB650_RAM_SIZE - sizeof(*value) ||
        dma_memory_read(&address_space_memory, address, value, sizeof(*value),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    *value = le32_to_cpu(*value);
    return true;
}

static void pb650_vg_step(void *opaque)
{
    PB650IOState *s = opaque;
    uint32_t command, value;

    /* FE2.0 control commands used by the original driver's power transitions. */
    for (unsigned budget = 0; budget < 256 && s->vg_remaining; budget++) {
        if (!pb650_vg_fetch(s->vg_address, &command)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "PocketBook 650 VG: invalid command address %08x\n",
                          s->vg_address);
            s->vg_remaining = 0;
            break;
        }
        if ((command >> 28) == 0) { /* END also signals its interrupt ID. */
            s->gpu[(PB650_VG_BASE + 0x10) / 4] |= BIT(command & 31);
            s->vg_remaining = 0;
            break;
        }
        if ((command >> 28) == 3 && (command & 0xffff) == 0x0e01 &&
            extract32(command, 16, 12) == 1 &&
            pb650_vg_fetch(s->vg_address + 4, &value)) {
            /* LOAD_STATE to EVENT: completion follows all preceding work. */
            if (value & (BIT(5) | BIT(6))) {
                s->gpu[(PB650_VG_BASE + 0x10) / 4] |= BIT(value & 31);
            }
            s->vg_address += 8;
            s->vg_remaining--;
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "PocketBook 650 VG: unsupported FE2.0 command %08x\n",
                          command);
            s->vg_remaining = 0;
            break;
        }
    }
    if (s->vg_remaining) {
        timer_mod(s->vg_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  PB650_VG_STEP_NS);
    }
    s->gpu[(PB650_VG_BASE + 0x504) / 4] = s->vg_remaining;
    pb650_vg_update_irq(s);
}

static uint64_t pb650_gpu_read(void *opaque, hwaddr offset, unsigned size)
{
    PB650IOState *s = PB650_IO(opaque);
    bool vg = offset >= 0x4000;
    unsigned reg = offset & 0x3fff;

    /* The control interfaces do not implement graphics rendering commands. */
    switch (reg) {
    case 0x00: /* HI_CLOCK_CONTROL: read-only 2D and 3D idle flags. */
        return s->gpu[offset / 4] | BIT(16) | BIT(17);
    case 0x04: /* HI_IDLE_STATE: all engines idle after reset. */
        return vg && s->vg_remaining ? 0x7ffffffe : 0x7fffffff;
    case 0x10: { /* AQIntrAcknowledge clears the pending event bits on read. */
        uint32_t pending = s->gpu[offset / 4];

        s->gpu[offset / 4] = 0;
        pb650_vg_update_irq(s);
        return pending;
    }
    case 0x18: /* Legacy identity; these cores use CHIP_MODEL/CHIP_REV. */
        return 0;
    case 0x1c: /* PIPE_2D, or the VG command front end. */
        return vg ? BIT(28) : BIT(9);
    case 0x20:
        return vg ? 0x355 : 0x320;
    case 0x24:
        return vg ? 0x1215 : 0x5007;
    default:
        return s->gpu[offset / 4];
    }
}

static void pb650_gpu_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    PB650IOState *s = PB650_IO(opaque);
    unsigned base = offset & ~0x3fff;
    unsigned reg = offset & 0x3fff;

    if (reg == 0) {
        if (value & BIT(12)) { /* SOFT_RESET clears the core's registers. */
            memset(&s->gpu[base / 4], 0, 0x4000);
            if (base == PB650_VG_BASE) {
                timer_del(s->vg_timer);
                s->vg_remaining = 0;
                pb650_vg_update_irq(s);
            }
        }
        s->gpu[offset / 4] = value & ~(BIT(16) | BIT(17));
    } else if (reg != 0x04 && reg != 0x10 && (reg < 0x18 || reg > 0x24)) {
        s->gpu[offset / 4] = value;
        if (base == PB650_VG_BASE) {
            if (reg == 0x14) {
                pb650_vg_update_irq(s);
            } else if (reg == 0x504) {
                s->vg_address = s->gpu[(PB650_VG_BASE + 0x500) / 4];
                s->vg_remaining = value;
                timer_del(s->vg_timer);
                if (s->vg_remaining) {
                    timer_mod(s->vg_timer,
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                              PB650_VG_STEP_NS);
                }
            }
        }
    }
}

static const MemoryRegionOps pb650_sdma_ops = {
    .read = pb650_sdma_read,
    .write = pb650_sdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps pb650_dcp_ops = {
    .read = pb650_dcp_read,
    .write = pb650_dcp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps pb650_gpu_ops = {
    .read = pb650_gpu_read,
    .write = pb650_gpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void pb650_io_reset(DeviceState *dev)
{
    PB650IOState *s = PB650_IO(dev);

    memset(s->sdma, 0, sizeof(s->sdma));
    memset(s->dcp, 0, sizeof(s->dcp));
    memset(s->gpu, 0, sizeof(s->gpu));
    timer_del(s->vg_timer);
    s->vg_address = s->vg_remaining = 0;
    pb650_vg_update_irq(s);
    s->dcp[0] = BIT(30);
    s->wifi_controls = 0;
    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->wifi_power, 0);
}

static int pb650_io_post_load(void *opaque, int version_id)
{
    PB650IOState *s = opaque;

    qemu_set_irq(s->irq, s->sdma[1] != 0);
    qemu_set_irq(s->wifi_power, s->wifi_controls == 3);
    pb650_vg_update_irq(s);
    return 0;
}

static const VMStateDescription pb650_io_vmstate = {
    .name = TYPE_PB650_IO,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = pb650_io_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(sdma, PB650IOState, 0x1000 / 4),
        VMSTATE_UINT32_ARRAY(dcp, PB650IOState, 0x4000 / 4),
        VMSTATE_UINT32_ARRAY(gpu, PB650IOState, 0x8000 / 4),
        VMSTATE_UINT32(vg_address, PB650IOState),
        VMSTATE_UINT32(vg_remaining, PB650IOState),
        VMSTATE_TIMER_PTR(vg_timer, PB650IOState),
        VMSTATE_UINT8(wifi_controls, PB650IOState),
        VMSTATE_END_OF_LIST()
    },
};

static void pb650_io_init(Object *obj)
{
    PB650IOState *s = PB650_IO(obj);

    memory_region_init_io(&s->sdma_iomem, obj, &pb650_sdma_ops, s,
                          "pocketbook-650.sdma", sizeof(s->sdma));
    memory_region_init_io(&s->dcp_iomem, obj, &pb650_dcp_ops, s,
                          "pocketbook-650.dcp", sizeof(s->dcp));
    memory_region_init_io(&s->gpu_iomem, obj, &pb650_gpu_ops, s,
                          "pocketbook-650.vivante", sizeof(s->gpu));
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->sdma_iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->dcp_iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->gpu_iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->vg_irq);
    s->vg_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pb650_vg_step, s);
    qdev_init_gpio_in_named(DEVICE(obj), pb650_wifi_control, "wifi-control", 2);
    qdev_init_gpio_out_named(DEVICE(obj), &s->wifi_power, "wifi-power", 1);
}

static void pb650_io_finalize(Object *obj)
{
    timer_free(PB650_IO(obj)->vg_timer);
}

static void pb650_io_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->vmsd = &pb650_io_vmstate;
    device_class_set_legacy_reset(dc, pb650_io_reset);
}

static void pb650_reset(void *opaque)
{
    PB650MachineState *s = opaque;

    cpu_reset(CPU(&s->soc->cpu[0]));
    cpu_set_pc(CPU(&s->soc->cpu[0]), s->entry);
}

static void pb650_load_firmware(PB650MachineState *s)
{
    MachineState *ms = MACHINE(s);
    g_autofree uint8_t *image = NULL;
    gsize size;
    uint32_t self, entry;

    if (!ms->firmware ||
        !g_file_get_contents(ms->firmware, (char **)&image, &size, NULL) ||
        size < 0x2c || ldl_le_p(image) != 0x402000d1) {
        error_report("PocketBook Ultra 650 requires its original U-Boot (-bios)");
        exit(EXIT_FAILURE);
    }
    self = ldl_le_p(image + 0x14);
    entry = ldl_le_p(image + 4);
    if (self < PB650_RAM_BASE || self >= PB650_RAM_BASE + PB650_RAM_SIZE ||
        size > PB650_RAM_BASE + PB650_RAM_SIZE - self ||
        entry < self || entry - self >= size) {
        error_report("PocketBook Ultra 650 U-Boot IVT is outside RAM");
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("pocketbook-650.uboot", image, size, self);
    s->entry = entry;
}

static void pb650_attach_card(FslIMX6State *soc, unsigned unit)
{
    DriveInfo *di = drive_get(IF_SD, 0, unit);
    DeviceState *card;

    if (!di) {
        if (!unit && !qtest_enabled()) {
            error_report("PocketBook Ultra 650 requires internal eMMC index 0");
            exit(EXIT_FAILURE);
        }
        return;
    }
    card = qdev_new(unit ? TYPE_SD_CARD : TYPE_EMMC);
    if (!unit) {
        /* The vendor loader requires byte-addressed capacity from CSD alone. */
        qdev_prop_set_bit(card, "byte-addressed-sector-count", false);
    }
    qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di), &error_fatal);
    qdev_realize_and_unref(card,
        qdev_get_child_bus(DEVICE(&soc->usdhc[unit]), "sd-bus"), &error_fatal);
}

static void pb650_init(MachineState *ms)
{
    PB650MachineState *f = PB650_MACHINE(ms);
    FslIMX6State *s = f->soc = FSL_IMX6(object_new(TYPE_FSL_IMX6));
    DeviceState *dev, *pxp, *buttons, *wifi, *board_io;
    static const unsigned key_gpio[] = { 2, 2, 2, 2, 3, 3, 3, 2 };
    static const unsigned key_pin[] = { 25, 27, 29, 31, 1, 3, 23, 28 };

    if (ms->ram_size != PB650_RAM_SIZE) {
        error_report("PocketBook Ultra 650 requires 512 MiB RAM");
        exit(EXIT_FAILURE);
    }
    object_property_add_child(OBJECT(ms), "soc", OBJECT(s));
    object_property_set_bool(OBJECT(s), "sololite", true, &error_fatal);
    object_property_set_bool(OBJECT(s), "has-el3", false, &error_fatal);
    object_property_set_uint(OBJECT(&s->src), "sbmr1", 0x60, &error_fatal);
    /* Released power/cover, absent removable card, idle active-low alerts. */
    qdev_prop_set_uint32(DEVICE(&s->gpio[2]), "reset-psr",
                         BIT(18) | BIT(21) | BIT(24) | BIT(28));
    qdev_prop_set_uint32(DEVICE(&s->gpio[3]), "reset-psr",
                         BIT(7) | BIT(23));
    for (unsigned i = 0; i < FSL_IMX6_NUM_USDHCS; i++) {
        qdev_prop_set_bit(DEVICE(&s->usdhc[i]), "defer-data-transfer", false);
        qdev_prop_set_bit(DEVICE(&s->usdhc[i]),
                          "preserve-interrupt-enables-on-reset", true);
    }
    qdev_realize(DEVICE(s), NULL, &error_fatal);
    memory_region_add_subregion(get_system_memory(), PB650_RAM_BASE, ms->ram);

    board_io = dev = qdev_new(TYPE_PB650_IO);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, FSL_IMX6_SDMA_ADDR);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, 0x020fc000);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, 0x02200000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
        qdev_get_gpio_in(DEVICE(&s->a9mpcore), FSL_IMX6_SDMA_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
        qdev_get_gpio_in(DEVICE(&s->a9mpcore), 11));

    sysbus_create_simple(TYPE_IMX_RNGC, 0x021b4000,
        qdev_get_gpio_in(DEVICE(&s->a9mpcore), 5));
    dev = qdev_new(TYPE_IMX6SL_MMDC);
    qdev_prop_set_uint64(dev, "ram-size", ms->ram_size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x021b0000);
    sysbus_create_simple(TYPE_IMX6SL_IOMUXC, 0x020e0000, NULL);
    dev = sysbus_create_simple(TYPE_IMX6SLL_OCOTP, FSL_IMX6_OCOTPCTRL_ADDR, NULL);
    imx6sll_ocotp_set_fuse(IMX6SLL_OCOTP(dev), 1, 6, 0x5f055a64);

    pxp = sysbus_create_simple(TYPE_IMX6SL_PXP, 0x020f0000,
        qdev_get_gpio_in(DEVICE(&s->a9mpcore), 98));
    dev = qdev_new(TYPE_IMX_EPDC);
    object_property_add_child(OBJECT(ms), "epdc", OBJECT(dev));
    object_property_set_link(OBJECT(dev), "pxp", OBJECT(pxp), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x020f4000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
        qdev_get_gpio_in(DEVICE(&s->a9mpcore), 97));

    dev = qdev_new(TYPE_IMX_USB_MISC);
    qdev_prop_set_bit(dev, "imx6-layout", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, 0x02184800, 2);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
        qdev_get_gpio_in_named(DEVICE(&s->usb[0]), "wakeup", 0));
    qdev_connect_gpio_out_named(DEVICE(&s->usb[0]), "resume", 0,
                                qdev_get_gpio_in(dev, 0));

    i2c_slave_create_simple(s->i2c[0].bus, TYPE_PB650_PFUZE, 0x08);
    i2c_slave_create_simple(s->i2c[0].bus, TYPE_PB650_BATTERY, 0x62);
    dev = DEVICE(i2c_slave_create_simple(s->i2c[0].bus, TYPE_TPS65185, 0x68));
    qdev_connect_gpio_out_named(dev, "pwrgood", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[1]), 13));
    qdev_connect_gpio_out_named(dev, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[1]), 12));
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 7,
        qdev_get_gpio_in_named(dev, "powerup", 0));
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 14,
        qdev_get_gpio_in_named(dev, "power-enable", 0));

    dev = DEVICE(i2c_slave_create_simple(s->i2c[1].bus, TYPE_MSG2133, 0x26));
    qdev_connect_gpio_out_named(dev, "irq", 0,
        qdev_get_gpio_in(DEVICE(&s->gpio[3]), 2));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 0,
        qdev_get_gpio_in_named(dev, "reset", 0));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 5,
        qdev_get_gpio_in_named(dev, "power", 0));

    pb650_attach_card(s, 0);
    pb650_attach_card(s, 2);
    wifi = qdev_new(TYPE_BCM43362_SDIO);
    qemu_configure_nic_device(wifi, true, "bcm43362");
    qdev_realize_and_unref(wifi,
        qdev_get_child_bus(DEVICE(&s->usdhc[3]), "sd-bus"), &error_fatal);
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 4,
        qdev_get_gpio_in_named(board_io, "wifi-control", 0));
    qdev_connect_gpio_out(DEVICE(&s->gpio[3]), 6,
        qdev_get_gpio_in_named(board_io, "wifi-control", 1));
    qdev_connect_gpio_out_named(board_io, "wifi-power", 0,
        qdev_get_gpio_in_named(wifi, "power", 0));
    qdev_connect_gpio_out_named(wifi, "irq", 0,
        qdev_get_gpio_in_named(DEVICE(&s->usdhc[3]), "sdio-irq", 0));

    buttons = qdev_new(TYPE_PB650_BUTTONS);
    qdev_realize_and_unref(buttons, NULL, &error_fatal);
    for (unsigned i = 0; i < ARRAY_SIZE(key_gpio); i++) {
        qemu_irq pin = qdev_get_gpio_in(DEVICE(&s->gpio[key_gpio[i]]), key_pin[i]);

        if (i == 6) {
            dev = qdev_new(TYPE_SPLIT_IRQ);
            qdev_prop_set_uint16(dev, "num-lines", 2);
            qdev_realize_and_unref(dev, NULL, &error_fatal);
            qdev_connect_gpio_out(dev, 0, pin);
            qdev_connect_gpio_out(dev, 1,
                qdev_get_gpio_in(DEVICE(&s->gpio[2]), 18));
            pin = qdev_get_gpio_in(dev, 0);
        }
        qdev_connect_gpio_out(buttons, i, pin);
    }
    if (!qtest_enabled()) {
        pb650_load_firmware(f);
        qemu_register_reset(pb650_reset, f);
    }
}

static void pb650_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "PocketBook Ultra 650 (i.MX6SoloLite)";
    mc->init = pb650_init;
    mc->max_cpus = mc->default_cpus = 1;
    mc->default_ram_size = PB650_RAM_SIZE;
    mc->default_ram_id = "pocketbook-650.ram";
    mc->default_nic = TYPE_BCM43362_SDIO;
    mc->ignore_memory_transaction_failures = true;
    mc->auto_create_sdcard = false;
}

static const TypeInfo pb650_types[] = {
    {
        .name = TYPE_PB650_REGS,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(PB650RegsState),
        .abstract = true,
        .class_init = pb650_regs_class_init,
    }, {
        .name = TYPE_PB650_PFUZE,
        .parent = TYPE_PB650_REGS,
        .class_init = pb650_pfuze_class_init,
    }, {
        .name = TYPE_PB650_BATTERY,
        .parent = TYPE_PB650_REGS,
        .class_init = pb650_battery_class_init,
    }, {
        .name = TYPE_PB650_BUTTONS,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(PB650ButtonsState),
        .instance_init = pb650_buttons_init,
        .class_init = pb650_buttons_class_init,
    }, {
        .name = TYPE_PB650_IO,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(PB650IOState),
        .instance_init = pb650_io_init,
        .instance_finalize = pb650_io_finalize,
        .class_init = pb650_io_class_init,
    }, {
        .name = TYPE_PB650_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(PB650MachineState),
        .class_init = pb650_class_init,
        .interfaces = arm_machine_interfaces,
    },
};

DEFINE_TYPES(pb650_types)
