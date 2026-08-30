/*
 * MediaTek MultiMediaCard/Secure Digital Card controller
 *
 * This models the command, FIFO, and descriptor-DMA paths used by the MT8113
 * stock U-Boot, Falcon storage BIOS, and Linux driver.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sd/mtk-msdc.h"
#include "hw/core/irq.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define MSDC_CFG        0x00
#define MSDC_PS         0x08
#define MSDC_INT        0x0c
#define MSDC_INTEN      0x10
#define MSDC_FIFOCS     0x14
#define MSDC_TXDATA     0x18
#define MSDC_RXDATA     0x1c
#define SDC_CMD         0x34
#define SDC_ARG         0x38
#define SDC_STS         0x3c
#define SDC_RESP0       0x40
#define SDC_RESP1       0x44
#define SDC_RESP2       0x48
#define SDC_RESP3       0x4c
#define SDC_BLK_NUM     0x50
#define SDC_ACMD_RESP   0x80
#define MSDC_DMA_SA_H4  0x8c
#define MSDC_DMA_SA     0x90
#define MSDC_DMA_CA     0x94
#define MSDC_DMA_CTRL   0x98
#define MSDC_DMA_CFG    0x9c
#define MSDC_DMA_LENGTH 0xa8

#define MSDC_CFG_CKSTB          BIT(7)
#define MSDC_CFG_RST            BIT(2)
#define MSDC_PS_DAT0            BIT(16)
#define MSDC_INT_ACMDRDY        BIT(3)
#define MSDC_INT_ACMDTMO        BIT(4)
#define MSDC_INT_CMDRDY         BIT(8)
#define MSDC_INT_CMDTMO         BIT(9)
#define MSDC_INT_XFER_COMPL     BIT(12)
#define MSDC_INT_DATTMO         BIT(14)
#define MSDC_FIFOCS_CLR         BIT(31)
#define MSDC_FIFO_SIZE          128

#define MSDC_DMA_CTRL_START     BIT(0)
#define MSDC_DMA_CTRL_STOP      BIT(1)
#define MSDC_DMA_CTRL_MODE      BIT(8)
#define MSDC_DMA_CFG_STS        BIT(0)

#define MSDC_GPD_HWO            BIT(0)
#define MSDC_GPD_BDP            BIT(1)
#define MSDC_GPD_PTR_H4_SHIFT   28
#define MSDC_BD_EOL             BIT(0)
#define MSDC_BD_NEXT_H4_SHIFT   24
#define MSDC_BD_PTR_H4_SHIFT    28
#define MSDC_DMA_ADDR_H4_MASK   0xf
#define MSDC_MAX_BD_NUM         1024
#define MSDC_DMA_DELAY_NS        100

typedef struct MTKMSDCDescriptor {
    uint32_t info;
    uint32_t next;
    uint32_t ptr;
    uint32_t length;
} MTKMSDCDescriptor;

#define SDC_CMD_BLK_LEN_SHIFT   16
#define SDC_CMD_BLK_LEN_MASK    0xfff
#define SDC_CMD_AUTO_SHIFT      28
#define SDC_CMD_AUTO_MASK       0x3
#define SDC_CMD_AUTO_CMD23      0x2
#define SDC_CMD_WRITE           BIT(13)
#define SDC_CMD_DTYPE_SHIFT     11
#define SDC_CMD_DTYPE_MASK      0x3
#define SDC_CMD_RSPTYP_SHIFT    7
#define SDC_CMD_RSPTYP_MASK     0x7
#define SDC_CMD_OPCODE_MASK     0x3f

static void mtk_msdc_update_irq(MTKMSDCState *s)
{
    qemu_set_irq(s->irq,
                 !!(s->regs[MSDC_INT / 4] & s->regs[MSDC_INTEN / 4]));
}

static void mtk_msdc_set_interrupt(MTKMSDCState *s, uint32_t interrupt)
{
    s->regs[MSDC_INT / 4] |= interrupt;
    mtk_msdc_update_irq(s);
}

static void mtk_msdc_transfer_complete(MTKMSDCState *s)
{
    if (!s->transfer_remaining) {
        mtk_msdc_set_interrupt(s, MSDC_INT_XFER_COMPL);
    }
}

static bool mtk_msdc_dma_buffer(MTKMSDCState *s, hwaddr address,
                                uint32_t length)
{
    g_autofree uint8_t *buffer = NULL;
    MemTxResult result;

    length = MIN(length, s->transfer_remaining);
    if (!length) {
        return true;
    }

    buffer = g_malloc(length);
    if (s->transfer_write) {
        result = dma_memory_read(&address_space_memory, address, buffer,
                                 length, MEMTXATTRS_UNSPECIFIED);
        if (result != MEMTX_OK) {
            return false;
        }
        sdbus_write_data(&s->sdbus, buffer, length);
    } else {
        sdbus_read_data(&s->sdbus, buffer, length);
        result = dma_memory_write(&address_space_memory, address, buffer,
                                  length, MEMTXATTRS_UNSPECIFIED);
        if (result != MEMTX_OK) {
            return false;
        }
    }

    s->transfer_remaining -= length;
    s->regs[MSDC_DMA_CA / 4] = address + length;
    return true;
}

static bool mtk_msdc_dma_read_desc(hwaddr address, MTKMSDCDescriptor *desc)
{
    uint32_t raw[4];

    if (dma_memory_read(&address_space_memory, address, raw, sizeof(raw),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }

    desc->info = le32_to_cpu(raw[0]);
    desc->next = le32_to_cpu(raw[1]);
    desc->ptr = le32_to_cpu(raw[2]);
    desc->length = le32_to_cpu(raw[3]);
    return true;
}

static bool mtk_msdc_dma_write_info(hwaddr address, uint32_t info)
{
    uint32_t value = cpu_to_le32(info);

    return dma_memory_write(&address_space_memory, address, &value,
                            sizeof(value), MEMTXATTRS_UNSPECIFIED) == MEMTX_OK;
}

static bool mtk_msdc_dma_descriptors(MTKMSDCState *s)
{
    MTKMSDCDescriptor gpd;
    hwaddr gpd_addr = s->regs[MSDC_DMA_SA / 4] |
        ((hwaddr)(s->regs[MSDC_DMA_SA_H4 / 4] & MSDC_DMA_ADDR_H4_MASK) << 32);

    if (!mtk_msdc_dma_read_desc(gpd_addr, &gpd) ||
        !(gpd.info & MSDC_GPD_HWO)) {
        return false;
    }

    if (gpd.info & MSDC_GPD_BDP) {
        hwaddr bd_addr = gpd.ptr |
            ((hwaddr)((gpd.info >> MSDC_GPD_PTR_H4_SHIFT) &
                      MSDC_DMA_ADDR_H4_MASK) << 32);

        /* The stock driver advertises MAX_BD_NUM (1024) scatter entries. */
        for (unsigned i = 0;
             i < MSDC_MAX_BD_NUM && s->transfer_remaining; i++) {
            MTKMSDCDescriptor bd;
            hwaddr buffer_addr;

            if (!mtk_msdc_dma_read_desc(bd_addr, &bd)) {
                return false;
            }
            buffer_addr = bd.ptr |
                ((hwaddr)((bd.info >> MSDC_BD_PTR_H4_SHIFT) &
                          MSDC_DMA_ADDR_H4_MASK) << 32);
            if (!mtk_msdc_dma_buffer(s, buffer_addr, bd.length & 0xffffff)) {
                return false;
            }
            if (bd.info & MSDC_BD_EOL) {
                break;
            }
            bd_addr = bd.next |
                ((hwaddr)((bd.info >> MSDC_BD_NEXT_H4_SHIFT) &
                          MSDC_DMA_ADDR_H4_MASK) << 32);
        }
    } else {
        hwaddr buffer_addr = gpd.ptr |
            ((hwaddr)((gpd.info >> MSDC_GPD_PTR_H4_SHIFT) &
                      MSDC_DMA_ADDR_H4_MASK) << 32);

        if (!mtk_msdc_dma_buffer(s, buffer_addr,
                                 gpd.length & 0xffffff)) {
            return false;
        }
    }

    return mtk_msdc_dma_write_info(gpd_addr, gpd.info & ~MSDC_GPD_HWO);
}

static void mtk_msdc_dma_run(void *opaque)
{
    MTKMSDCState *s = opaque;
    bool ok;

    if (!s->dma_active || !s->transfer_remaining) {
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  TYPE_MTK_MSDC ": DMA start mode=%u address=%08x "
                  "length=%u remaining=%u\n",
                  !!(s->regs[MSDC_DMA_CTRL / 4] & MSDC_DMA_CTRL_MODE),
                  s->regs[MSDC_DMA_SA / 4],
                  s->regs[MSDC_DMA_LENGTH / 4], s->transfer_remaining);

    if (s->regs[MSDC_DMA_CTRL / 4] & MSDC_DMA_CTRL_MODE) {
        ok = mtk_msdc_dma_descriptors(s);
    } else {
        hwaddr address = s->regs[MSDC_DMA_SA / 4] |
            ((hwaddr)(s->regs[MSDC_DMA_SA_H4 / 4] &
                      MSDC_DMA_ADDR_H4_MASK) << 32);
        uint32_t length = s->regs[MSDC_DMA_LENGTH / 4];

        ok = mtk_msdc_dma_buffer(s, address,
                                 length ? length : s->transfer_remaining);
    }

    s->dma_active = false;
    s->regs[MSDC_DMA_CTRL / 4] &= ~MSDC_DMA_CTRL_START;
    s->regs[MSDC_DMA_CFG / 4] &= ~MSDC_DMA_CFG_STS;
    if (ok && !s->transfer_remaining) {
        qemu_log_mask(LOG_UNIMP, TYPE_MTK_MSDC ": DMA complete\n");
        mtk_msdc_transfer_complete(s);
    } else {
        qemu_log_mask(LOG_UNIMP,
                      TYPE_MTK_MSDC ": DMA failed ok=%u remaining=%u\n",
                      ok, s->transfer_remaining);
        mtk_msdc_set_interrupt(s, MSDC_INT_DATTMO);
    }
}

static void mtk_msdc_schedule_dma(MTKMSDCState *s)
{
    if (s->dma_active && s->transfer_remaining) {
        timer_mod(s->dma_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  MSDC_DMA_DELAY_NS);
    }
}

static void mtk_msdc_start_command(MTKMSDCState *s, uint32_t command)
{
    SDRequest request = {
        .cmd = command & SDC_CMD_OPCODE_MASK,
        .arg = s->regs[SDC_ARG / 4],
    };
    uint8_t response[16] = { 0 };
    unsigned response_type =
        (command >> SDC_CMD_RSPTYP_SHIFT) & SDC_CMD_RSPTYP_MASK;
    unsigned data_type =
        (command >> SDC_CMD_DTYPE_SHIFT) & SDC_CMD_DTYPE_MASK;
    size_t response_length;
    size_t expected_length;
    uint32_t interrupts = MSDC_INT_CMDRDY;

    s->regs[SDC_CMD / 4] = command;
    s->transfer_remaining = 0;

    qemu_log_mask(LOG_UNIMP,
                  TYPE_MTK_MSDC ": CMD%u arg=%08x raw=%08x\n",
                  request.cmd, request.arg, command);

    if (((command >> SDC_CMD_AUTO_SHIFT) & SDC_CMD_AUTO_MASK) ==
        SDC_CMD_AUTO_CMD23) {
        SDRequest auto_request = {
            .cmd = 23,
            .arg = s->regs[SDC_BLK_NUM / 4],
        };
        uint8_t auto_response[4] = { 0 };
        size_t auto_response_length;

        auto_response_length = sdbus_do_command(&s->sdbus, &auto_request,
                                                 auto_response,
                                                 sizeof(auto_response));
        if (auto_response_length != sizeof(auto_response)) {
            mtk_msdc_set_interrupt(s, MSDC_INT_ACMDTMO);
            return;
        }
        s->regs[SDC_ACMD_RESP / 4] = ldl_be_p(auto_response);
        interrupts |= MSDC_INT_ACMDRDY;
    }

    response_length = sdbus_do_command(&s->sdbus, &request, response,
                                       sizeof(response));
    expected_length = response_type == 0 ? 0 : response_type == 2 ? 16 : 4;
    if (response_length != expected_length) {
        mtk_msdc_set_interrupt(s, MSDC_INT_CMDTMO);
        return;
    }

    if (response_length == 4) {
        s->regs[SDC_RESP0 / 4] = ldl_be_p(response);
        s->regs[SDC_RESP1 / 4] = 0;
        s->regs[SDC_RESP2 / 4] = 0;
        s->regs[SDC_RESP3 / 4] = 0;
    } else if (response_length == 16) {
        s->regs[SDC_RESP0 / 4] = ldl_be_p(response + 12);
        s->regs[SDC_RESP1 / 4] = ldl_be_p(response + 8);
        s->regs[SDC_RESP2 / 4] = ldl_be_p(response + 4);
        s->regs[SDC_RESP3 / 4] = ldl_be_p(response);
    }

    if (data_type) {
        uint32_t block_size =
            (command >> SDC_CMD_BLK_LEN_SHIFT) & SDC_CMD_BLK_LEN_MASK;
        uint32_t blocks = data_type == 2 ? s->regs[SDC_BLK_NUM / 4] : 1;

        s->transfer_remaining = block_size * blocks;
        s->transfer_write = command & SDC_CMD_WRITE;
        qemu_log_mask(LOG_UNIMP,
                      TYPE_MTK_MSDC ": data blocks=%u block-size=%u "
                      "write=%u dma-active=%u\n",
                      blocks, block_size, s->transfer_write, s->dma_active);
    }

    /* Command completion precedes the independently-running data engine. */
    mtk_msdc_set_interrupt(s, interrupts);
    mtk_msdc_schedule_dma(s);
}

static uint64_t mtk_msdc_fifo_read(MTKMSDCState *s, unsigned size)
{
    uint8_t bytes[sizeof(uint64_t)] = { 0 };
    uint64_t value = 0;
    unsigned count;

    if (s->transfer_write || !s->transfer_remaining) {
        return 0;
    }

    count = MIN(size, s->transfer_remaining);
    sdbus_read_data(&s->sdbus, bytes, count);
    for (unsigned i = 0; i < count; i++) {
        value |= (uint64_t)bytes[i] << (i * 8);
    }
    s->transfer_remaining -= count;
    mtk_msdc_transfer_complete(s);
    return value;
}

static void mtk_msdc_fifo_write(MTKMSDCState *s, uint64_t value,
                                unsigned size)
{
    uint8_t bytes[sizeof(uint64_t)];
    unsigned count;

    if (!s->transfer_write || !s->transfer_remaining) {
        return;
    }

    count = MIN(size, s->transfer_remaining);
    for (unsigned i = 0; i < count; i++) {
        bytes[i] = value >> (i * 8);
    }
    sdbus_write_data(&s->sdbus, bytes, count);
    s->transfer_remaining -= count;
    mtk_msdc_transfer_complete(s);
}

static uint64_t mtk_msdc_read(void *opaque, hwaddr offset, unsigned size)
{
    MTKMSDCState *s = opaque;

    switch (offset) {
    case MSDC_CFG:
        return s->regs[offset / 4] | MSDC_CFG_CKSTB;
    case MSDC_PS:
        return s->regs[offset / 4] | MSDC_PS_DAT0;
    case MSDC_FIFOCS:
        if (!s->transfer_write) {
            return MIN(s->transfer_remaining, MSDC_FIFO_SIZE);
        }
        return 0;
    case MSDC_RXDATA:
        return mtk_msdc_fifo_read(s, size);
    case SDC_STS:
        return 0;
    default:
        if (offset < MTK_MSDC_MMIO_SIZE && !(offset & 3) && size == 4) {
            return s->regs[offset / 4];
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      TYPE_MTK_MSDC ": invalid read at 0x%" HWADDR_PRIx
                      " size %u\n", offset, size);
        return 0;
    }
}

static void mtk_msdc_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    MTKMSDCState *s = opaque;

    switch (offset) {
    case MSDC_CFG:
        s->regs[offset / 4] = (uint32_t)value & ~MSDC_CFG_RST;
        if (value & MSDC_CFG_RST) {
            timer_del(s->dma_timer);
            s->transfer_remaining = 0;
            s->dma_active = false;
            s->regs[MSDC_DMA_CFG / 4] &= ~MSDC_DMA_CFG_STS;
        }
        break;
    case MSDC_INT:
        s->regs[offset / 4] &= ~(uint32_t)value;
        mtk_msdc_update_irq(s);
        break;
    case MSDC_INTEN:
        s->regs[offset / 4] = value;
        mtk_msdc_update_irq(s);
        break;
    case MSDC_FIFOCS:
        if (value & MSDC_FIFOCS_CLR) {
            timer_del(s->dma_timer);
            s->transfer_remaining = 0;
        }
        break;
    case MSDC_TXDATA:
        mtk_msdc_fifo_write(s, value, size);
        break;
    case SDC_CMD:
        mtk_msdc_start_command(s, value);
        break;
    case MSDC_DMA_CTRL:
        s->regs[offset / 4] = value &
            ~(MSDC_DMA_CTRL_START | MSDC_DMA_CTRL_STOP);
        if (value & MSDC_DMA_CTRL_STOP) {
            timer_del(s->dma_timer);
            s->dma_active = false;
            s->regs[MSDC_DMA_CFG / 4] &= ~MSDC_DMA_CFG_STS;
        }
        if (value & MSDC_DMA_CTRL_START) {
            s->dma_active = true;
            s->regs[MSDC_DMA_CFG / 4] |= MSDC_DMA_CFG_STS;
            mtk_msdc_schedule_dma(s);
        }
        break;
    default:
        if (offset < MTK_MSDC_MMIO_SIZE && !(offset & 3) && size == 4) {
            s->regs[offset / 4] = value;
            break;
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      TYPE_MTK_MSDC ": invalid write at 0x%" HWADDR_PRIx
                      " size %u\n", offset, size);
        break;
    }
}

static const MemoryRegionOps mtk_msdc_ops = {
    .read = mtk_msdc_read,
    .write = mtk_msdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static uint64_t mtk_msdc_top_read(void *opaque, hwaddr offset, unsigned size)
{
    MTKMSDCState *s = opaque;

    return s->top_regs[offset / sizeof(uint32_t)];
}

static void mtk_msdc_top_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    MTKMSDCState *s = opaque;

    s->top_regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps mtk_msdc_top_ops = {
    .read = mtk_msdc_top_read,
    .write = mtk_msdc_top_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mtk_msdc_reset_hold(Object *obj, ResetType type)
{
    MTKMSDCState *s = MTK_MSDC(obj);

    timer_del(s->dma_timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->top_regs, 0, sizeof(s->top_regs));
    s->regs[MSDC_CFG / 4] = MSDC_CFG_CKSTB;
    s->regs[MSDC_PS / 4] = MSDC_PS_DAT0;
    s->transfer_remaining = 0;
    s->transfer_write = false;
    s->dma_active = false;
    qemu_set_irq(s->irq, 0);
}

static void mtk_msdc_init(Object *obj)
{
    MTKMSDCState *s = MTK_MSDC(obj);

    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_SD_BUS, DEVICE(s),
              "sd-bus");
    memory_region_init_io(&s->iomem, obj, &mtk_msdc_ops, s, TYPE_MTK_MSDC,
                          MTK_MSDC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    memory_region_init_io(&s->top_iomem, obj, &mtk_msdc_top_ops, s,
                          TYPE_MTK_MSDC ".top", MTK_MSDC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->top_iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->dma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mtk_msdc_dma_run, s);
}

static void mtk_msdc_finalize(Object *obj)
{
    MTKMSDCState *s = MTK_MSDC(obj);

    timer_free(s->dma_timer);
}

static void mtk_msdc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "MediaTek MSDC controller";
    rc->phases.hold = mtk_msdc_reset_hold;
}

static const TypeInfo mtk_msdc_type_info = {
    .name = TYPE_MTK_MSDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MTKMSDCState),
    .instance_init = mtk_msdc_init,
    .instance_finalize = mtk_msdc_finalize,
    .class_init = mtk_msdc_class_init,
};

static void mtk_msdc_register_types(void)
{
    type_register_static(&mtk_msdc_type_info);
}
type_init(mtk_msdc_register_types)
