/*
 * MediaTek MT65xx/MT8512 I2C controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/i2c/mtk_i2c.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"

#define I2C_REG_DATA_PORT        0x00
#define I2C_REG_SLAVE_ADDR       0x04
#define I2C_REG_INTR_MASK        0x08
#define I2C_REG_INTR_STAT        0x0c
#define I2C_REG_CONTROL          0x10
#define I2C_REG_TRANSFER_LEN     0x14
#define I2C_REG_TRANSAC_LEN      0x18
#define I2C_REG_START            0x24
#define I2C_REG_FIFO_ADDR_CLR    0x38
#define I2C_REG_SOFTRESET        0x50
#define I2C_REG_TRANSFER_LEN_AUX 0x6c

#define I2C_INTR_COMPLETE        BIT(0)
#define I2C_INTR_ACKERR          BIT(1)
#define I2C_CONTROL_DMA_EN       BIT(2)
#define I2C_CONTROL_DIR_CHANGE   BIT(4)

#define DMA_REG_EN               0x08
#define DMA_REG_RST              0x0c
#define DMA_REG_CON              0x18
#define DMA_REG_TX_MEM_ADDR      0x1c
#define DMA_REG_RX_MEM_ADDR      0x20
#define DMA_REG_TX_LEN           0x24
#define DMA_REG_RX_LEN           0x28

static uint64_t mtk_i2c_load(const uint8_t *regs, hwaddr offset,
                             unsigned size)
{
    uint64_t value = 0;

    for (unsigned i = 0; i < size; i++) {
        value |= (uint64_t)regs[offset + i] << (8 * i);
    }
    return value;
}

static void mtk_i2c_store(uint8_t *regs, hwaddr offset, uint64_t value,
                          unsigned size)
{
    for (unsigned i = 0; i < size; i++) {
        regs[offset + i] = value >> (8 * i);
    }
}

static uint16_t mtk_i2c_reg16(MTKI2CState *s, hwaddr offset)
{
    return mtk_i2c_load(s->regs, offset, 2);
}

static uint32_t mtk_i2c_dma_reg32(MTKI2CState *s, hwaddr offset)
{
    return mtk_i2c_load(s->dma_regs, offset, 4);
}

static void mtk_i2c_update_irq(MTKI2CState *s)
{
    qemu_set_irq(s->irq,
                 mtk_i2c_reg16(s, I2C_REG_INTR_STAT) &
                 mtk_i2c_reg16(s, I2C_REG_INTR_MASK));
}

static void mtk_i2c_fifo_clear(MTKI2CState *s)
{
    s->fifo_head = 0;
    s->fifo_count = 0;
}

static void mtk_i2c_fifo_push(MTKI2CState *s, uint8_t value)
{
    if (s->fifo_count < MTK_I2C_FIFO_SIZE) {
        s->fifo[(s->fifo_head + s->fifo_count++) % MTK_I2C_FIFO_SIZE] =
            value;
    }
}

static uint8_t mtk_i2c_fifo_pop(MTKI2CState *s)
{
    uint8_t value = 0xff;

    if (s->fifo_count) {
        value = s->fifo[s->fifo_head++ % MTK_I2C_FIFO_SIZE];
        s->fifo_count--;
    }
    return value;
}

static bool mtk_i2c_dma_read(hwaddr addr, uint8_t *buf, size_t len)
{
    return address_space_read(&address_space_memory, addr,
                              MEMTXATTRS_UNSPECIFIED, buf, len) == MEMTX_OK;
}

static bool mtk_i2c_dma_write(hwaddr addr, const uint8_t *buf, size_t len)
{
    return address_space_write(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, buf, len) == MEMTX_OK;
}

static bool mtk_i2c_send_bytes(MTKI2CState *s, uint8_t address,
                               const uint8_t *buf, size_t len)
{
    if (i2c_start_send(s->bus, address)) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        if (i2c_send(s->bus, buf[i])) {
            return false;
        }
    }
    return true;
}

static bool mtk_i2c_recv_bytes(MTKI2CState *s, uint8_t address,
                               uint8_t *buf, size_t len)
{
    if (i2c_start_recv(s->bus, address)) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = i2c_recv(s->bus);
    }
    return true;
}

static void mtk_i2c_transfer(MTKI2CState *s)
{
    uint16_t slave = mtk_i2c_reg16(s, I2C_REG_SLAVE_ADDR);
    uint16_t control = mtk_i2c_reg16(s, I2C_REG_CONTROL);
    uint16_t transactions = mtk_i2c_reg16(s, I2C_REG_TRANSAC_LEN);
    bool combined = (control & I2C_CONTROL_DIR_CHANGE) && transactions == 2;
    bool is_read = slave & 1;
    bool dma = control & I2C_CONTROL_DMA_EN;
    uint8_t address = (slave >> 1) & 0x7f;
    size_t tx_len = dma ? mtk_i2c_dma_reg32(s, DMA_REG_TX_LEN) :
                          mtk_i2c_reg16(s, I2C_REG_TRANSFER_LEN);
    size_t rx_len = dma ? mtk_i2c_dma_reg32(s, DMA_REG_RX_LEN) :
                          (combined ? mtk_i2c_reg16(s,
                                           I2C_REG_TRANSFER_LEN_AUX) :
                                      mtk_i2c_reg16(s,
                                           I2C_REG_TRANSFER_LEN));
    g_autofree uint8_t *tx = NULL;
    g_autofree uint8_t *rx = NULL;
    bool ok = true;

    if (tx_len > 65536 || rx_len > 65536) {
        ok = false;
        goto done;
    }
    tx = tx_len ? g_malloc(tx_len) : NULL;
    rx = rx_len ? g_malloc0(rx_len) : NULL;

    if (combined || !is_read) {
        if (dma) {
            ok = mtk_i2c_dma_read(mtk_i2c_dma_reg32(s,
                                      DMA_REG_TX_MEM_ADDR), tx, tx_len);
        } else {
            for (size_t i = 0; i < tx_len; i++) {
                tx[i] = mtk_i2c_fifo_pop(s);
            }
        }
        if (ok) {
            ok = mtk_i2c_send_bytes(s, address, tx, tx_len);
        }
    }

    if (ok && (combined || is_read)) {
        ok = mtk_i2c_recv_bytes(s, address, rx, rx_len);
        if (ok && dma) {
            ok = mtk_i2c_dma_write(mtk_i2c_dma_reg32(s,
                                       DMA_REG_RX_MEM_ADDR), rx, rx_len);
        } else if (ok) {
            mtk_i2c_fifo_clear(s);
            for (size_t i = 0; i < rx_len; i++) {
                mtk_i2c_fifo_push(s, rx[i]);
            }
        }
    }

done:
    if (i2c_bus_busy(s->bus)) {
        i2c_end_transfer(s->bus);
    }
    s->regs[I2C_REG_INTR_STAT] |= I2C_INTR_COMPLETE;
    if (!ok) {
        s->regs[I2C_REG_INTR_STAT] |= I2C_INTR_ACKERR;
    }
    mtk_i2c_update_irq(s);
}

static uint64_t mtk_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    MTKI2CState *s = opaque;

    if (offset == I2C_REG_DATA_PORT && size == 1) {
        return mtk_i2c_fifo_pop(s);
    }
    return mtk_i2c_load(s->regs, offset, size);
}

static void mtk_i2c_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    MTKI2CState *s = opaque;

    if (offset == I2C_REG_DATA_PORT && size == 1) {
        mtk_i2c_fifo_push(s, value);
        return;
    }
    if (offset == I2C_REG_INTR_STAT) {
        uint16_t status = mtk_i2c_reg16(s, I2C_REG_INTR_STAT);

        status &= ~(uint16_t)value;
        mtk_i2c_store(s->regs, I2C_REG_INTR_STAT, status, 2);
        mtk_i2c_update_irq(s);
        return;
    }
    if (offset == I2C_REG_FIFO_ADDR_CLR) {
        mtk_i2c_fifo_clear(s);
    }
    if (offset == I2C_REG_SOFTRESET && (value & 1)) {
        mtk_i2c_fifo_clear(s);
        s->regs[I2C_REG_INTR_STAT] = 0;
        s->regs[I2C_REG_INTR_STAT + 1] = 0;
        mtk_i2c_update_irq(s);
    }
    mtk_i2c_store(s->regs, offset, value, size);
    if (offset == I2C_REG_INTR_MASK) {
        mtk_i2c_update_irq(s);
    }
    if (offset == I2C_REG_START && (value & 1)) {
        mtk_i2c_transfer(s);
    }
}

static uint64_t mtk_i2c_dma_read_reg(void *opaque, hwaddr offset,
                                     unsigned size)
{
    MTKI2CState *s = opaque;

    if (offset == DMA_REG_RST) {
        return 0;
    }
    return mtk_i2c_load(s->dma_regs, offset, size);
}

static void mtk_i2c_dma_write_reg(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    MTKI2CState *s = opaque;

    if (offset == DMA_REG_RST) {
        if (value) {
            memset(s->dma_regs, 0, sizeof(s->dma_regs));
        }
        return;
    }
    mtk_i2c_store(s->dma_regs, offset, value, size);
}

static const MemoryRegionOps mtk_i2c_ops = {
    .read = mtk_i2c_read,
    .write = mtk_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps mtk_i2c_dma_ops = {
    .read = mtk_i2c_dma_read_reg,
    .write = mtk_i2c_dma_write_reg,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mtk_i2c_reset(DeviceState *dev)
{
    MTKI2CState *s = MTK_I2C(dev);

    if (s->bus && i2c_bus_busy(s->bus)) {
        i2c_end_transfer(s->bus);
    }
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->dma_regs, 0, sizeof(s->dma_regs));
    mtk_i2c_fifo_clear(s);
    qemu_set_irq(s->irq, 0);
}

static void mtk_i2c_init(Object *obj)
{
    MTKI2CState *s = MTK_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &mtk_i2c_ops, s,
                          TYPE_MTK_I2C, MTK_I2C_MMIO_SIZE);
    memory_region_init_io(&s->dma_iomem, obj, &mtk_i2c_dma_ops, s,
                          TYPE_MTK_I2C ".dma", MTK_I2C_DMA_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->dma_iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static void mtk_i2c_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->desc = "MediaTek MT65xx I2C controller";
    device_class_set_legacy_reset(dc, mtk_i2c_reset);
}

static const TypeInfo mtk_i2c_info = {
    .name = TYPE_MTK_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MTKI2CState),
    .instance_init = mtk_i2c_init,
    .class_init = mtk_i2c_class_init,
};

static void mtk_i2c_register_types(void)
{
    type_register_static(&mtk_i2c_info);
}
type_init(mtk_i2c_register_types)
