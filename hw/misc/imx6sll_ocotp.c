/*
 * Minimal i.MX6SLL OCOTP controller
 *
 * This implements the shadow and direct-read interfaces used by U-Boot.
 * Programming is intentionally volatile: fuse writes survive guest resets,
 * but not a QEMU restart.
 */

#include "qemu/osdep.h"
#include "hw/misc/imx6sll_ocotp.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define OCOTP_MMIO_SIZE       0x4000
#define OCOTP_BANKS           9
#define OCOTP_WORDS           8

#define OCOTP_CTRL            0x000
#define OCOTP_CTRL_SET        0x004
#define OCOTP_CTRL_CLR        0x008
#define OCOTP_CTRL_TOG        0x00c
#define OCOTP_TIMING          0x010
#define OCOTP_DATA            0x020
#define OCOTP_READ_CTRL       0x030
#define OCOTP_READ_FUSE_DATA  0x040
#define OCOTP_VERSION         0x090
#define OCOTP_SHADOW_BASE     0x400
#define OCOTP_BANK_STRIDE     0x080
#define OCOTP_WORD_STRIDE     0x010

#define OCOTP_CTRL_ADDR_MASK  0x7f
#define OCOTP_UNLOCK_KEY      0x3e77
#define OCOTP_UNLOCK_SHIFT    16

struct IMX6SLLOCOTPState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t fuses[OCOTP_BANKS][OCOTP_WORDS];
    uint32_t ctrl;
    uint32_t timing;
    uint32_t data;
    uint32_t read_ctrl;
    uint32_t read_fuse_data;
};

static bool ocotp_direct_address(IMX6SLLOCOTPState *s, unsigned *bank,
                                 unsigned *word)
{
    unsigned address = s->ctrl & OCOTP_CTRL_ADDR_MASK;

    *bank = address >> 3;
    *word = address & 7;
    if (*bank == 7 && *word >= 4) {
        *bank = 8;
        *word -= 4;
    }
    return *bank < OCOTP_BANKS && *word < OCOTP_WORDS;
}

static bool ocotp_shadow_address(hwaddr offset, unsigned *bank,
                                 unsigned *word)
{
    hwaddr shadow;

    if (offset < OCOTP_SHADOW_BASE) {
        return false;
    }
    shadow = offset - OCOTP_SHADOW_BASE;
    *bank = shadow / OCOTP_BANK_STRIDE;
    shadow %= OCOTP_BANK_STRIDE;
    if (shadow % OCOTP_WORD_STRIDE) {
        return false;
    }
    *word = shadow / OCOTP_WORD_STRIDE;
    return *bank < OCOTP_BANKS && *word < OCOTP_WORDS;
}

static uint64_t ocotp_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX6SLLOCOTPState *s = opaque;
    unsigned bank, word;

    if (ocotp_shadow_address(offset, &bank, &word)) {
        return s->fuses[bank][word];
    }

    switch (offset) {
    case OCOTP_CTRL:
        return s->ctrl;
    case OCOTP_TIMING:
        return s->timing;
    case OCOTP_DATA:
        return s->data;
    case OCOTP_READ_CTRL:
        return s->read_ctrl;
    case OCOTP_READ_FUSE_DATA:
        return s->read_fuse_data;
    case OCOTP_VERSION:
        return 0x00010000;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read at 0x%" HWADDR_PRIx "\n",
                      TYPE_IMX6SLL_OCOTP, offset);
        return 0;
    }
}

static void ocotp_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    IMX6SLLOCOTPState *s = opaque;
    unsigned bank, word;

    switch (offset) {
    case OCOTP_CTRL:
        s->ctrl = value;
        break;
    case OCOTP_CTRL_SET:
        s->ctrl |= value;
        break;
    case OCOTP_CTRL_CLR:
        s->ctrl &= ~value;
        break;
    case OCOTP_CTRL_TOG:
        s->ctrl ^= value;
        break;
    case OCOTP_TIMING:
        s->timing = value;
        break;
    case OCOTP_DATA:
        s->data = value;
        if ((s->ctrl >> OCOTP_UNLOCK_SHIFT) == OCOTP_UNLOCK_KEY &&
            ocotp_direct_address(s, &bank, &word)) {
            s->fuses[bank][word] |= value;
        }
        break;
    case OCOTP_READ_CTRL:
        s->read_ctrl = value;
        if ((value & 1) && ocotp_direct_address(s, &bank, &word)) {
            s->read_fuse_data = s->fuses[bank][word];
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write at 0x%" HWADDR_PRIx
                      " value 0x%" PRIx64 "\n",
                      TYPE_IMX6SLL_OCOTP, offset, value);
        break;
    }
}

static const MemoryRegionOps ocotp_ops = {
    .read = ocotp_read,
    .write = ocotp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

void imx6sll_ocotp_set_fuse(IMX6SLLOCOTPState *s, unsigned bank,
                            unsigned word, uint32_t value)
{
    assert(bank < OCOTP_BANKS && word < OCOTP_WORDS);
    s->fuses[bank][word] = value;
}

static const VMStateDescription ocotp_vmstate = {
    .name = TYPE_IMX6SLL_OCOTP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(fuses, IMX6SLLOCOTPState,
                              OCOTP_BANKS, OCOTP_WORDS),
        VMSTATE_UINT32(ctrl, IMX6SLLOCOTPState),
        VMSTATE_UINT32(timing, IMX6SLLOCOTPState),
        VMSTATE_UINT32(data, IMX6SLLOCOTPState),
        VMSTATE_UINT32(read_ctrl, IMX6SLLOCOTPState),
        VMSTATE_UINT32(read_fuse_data, IMX6SLLOCOTPState),
        VMSTATE_END_OF_LIST()
    },
};

static void ocotp_init(Object *obj)
{
    IMX6SLLOCOTPState *s = IMX6SLL_OCOTP(obj);

    memory_region_init_io(&s->iomem, obj, &ocotp_ops, s,
                          TYPE_IMX6SLL_OCOTP, OCOTP_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void ocotp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->vmsd = &ocotp_vmstate;
}

static const TypeInfo ocotp_info = {
    .name = TYPE_IMX6SLL_OCOTP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX6SLLOCOTPState),
    .instance_init = ocotp_init,
    .class_init = ocotp_class_init,
};

static void ocotp_register_types(void)
{
    type_register_static(&ocotp_info);
}
type_init(ocotp_register_types)
