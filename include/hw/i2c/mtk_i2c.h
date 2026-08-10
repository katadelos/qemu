/* MediaTek MT65xx/MT8512 I2C controller. */

#ifndef HW_I2C_MTK_I2C_H
#define HW_I2C_MTK_I2C_H

#include "hw/i2c/i2c.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_MTK_I2C "mtk-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(MTKI2CState, MTK_I2C)

#define MTK_I2C_MMIO_SIZE 0x100
#define MTK_I2C_DMA_MMIO_SIZE 0x80
#define MTK_I2C_FIFO_SIZE 256

struct MTKI2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion dma_iomem;
    qemu_irq irq;
    I2CBus *bus;
    uint8_t regs[MTK_I2C_MMIO_SIZE];
    uint8_t dma_regs[MTK_I2C_DMA_MMIO_SIZE];
    uint8_t fifo[MTK_I2C_FIFO_SIZE];
    unsigned fifo_head;
    unsigned fifo_count;
};

#endif
