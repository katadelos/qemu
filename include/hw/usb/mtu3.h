/* MediaTek USB device MAC and QMU. SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_USB_MTU3_H
#define HW_USB_MTU3_H

#include "hw/core/sysbus.h"

#define TYPE_MTU3 "mtu3"
OBJECT_DECLARE_SIMPLE_TYPE(MTU3State, MTU3)
#define MTU3_ENDPOINTS 16
#define MTU3_FIFO_SIZE 1024

typedef struct MTU3Endpoint {
    uint8_t fifo[MTU3_FIFO_SIZE];
    unsigned length, position;
    hwaddr current_gpd;
    uint32_t transferred;
    bool active;
} MTU3Endpoint;

struct MTU3State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[0x2e00 / 4];
    MTU3Endpoint ep[2][MTU3_ENDPOINTS]; /* OUT, IN */
    uint32_t endpoints, fifo_bytes;
    bool vbus, reset, powered;
};

/* USB wire interface. Queues complete only when a host supplies tokens.
 * Return a byte count, -EAGAIN (NAK), -EPIPE (STALL), -EIO (DMA/protocol
 * error), or -ENOSPC (host buffer too small).
 * No host transport is attached by the board at present.
 */
int mtu3_out_packet(MTU3State *s, unsigned ep, const uint8_t *data,
                    size_t length, bool setup);
int mtu3_in_packet(MTU3State *s, unsigned ep, uint8_t *data, size_t capacity);
void mtu3_bus_reset(MTU3State *s);

#endif
