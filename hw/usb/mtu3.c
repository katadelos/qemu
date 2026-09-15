/*
 * MediaTek USB device MAC/BMU/QMU, based on the vendor Linux mtu3 driver.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Implements the register and descriptor interfaces independently of a host
 * transport. An unplugged device never fabricates transfer completion.
 */
#include "qemu/osdep.h"
#include "hw/usb/mtu3.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "system/address-spaces.h"

#define R(s, offset) ((s)->regs[(offset) / 4])
#define EP_CSR(in, n) ((n) ? ((in) ? 0x100 : 0x200) + (n) * 0x10 : 0x100)
#define Q_CSR(in, n) (((in) ? 0x500 : 0x600) + (n) * 0x10)
#define Q_HIGH(in, n) (((in) ? 0x480 : 0x4c0) + (n) * 4)

/* Status, enable, enable-set, enable-clear groups. */
static const unsigned mtu3_irq_groups[] = {
    0x000, 0x080, 0x700, 0x710, 0x780, 0x7c0, 0x7d0,
};

static void mtu3_update_irq(MTU3State *s)
{
    uint32_t pending = 0, errors = 0;

    if (R(s, 0x780) & R(s, 0x784) & 0xffff) {
        errors |= BIT(1);
    }
    if (R(s, 0x780) & R(s, 0x784) & 0xffff0000) {
        errors |= BIT(2);
    }
    if (R(s, 0x7c0) & R(s, 0x7c4) & 0xffff) {
        errors |= BIT(17);
    }
    if (R(s, 0x7c0) & R(s, 0x7c4) & 0xffff0000) {
        errors |= BIT(18);
    }
    if (R(s, 0x7d0) & R(s, 0x7d4)) {
        errors |= BIT(20);
    }
    R(s, 0x710) |= errors;
    if (R(s, 0x080) & R(s, 0x084)) {
        pending |= BIT(0);
    }
    if ((R(s, 0x700) & R(s, 0x704)) || (R(s, 0x710) & R(s, 0x714))) {
        pending |= BIT(1);
    }
    if (R(s, 0x1540) & R(s, 0x153c)) {
        pending |= BIT(2);
    }
    if (R(s, 0x241c) & R(s, 0x2418)) {
        pending |= BIT(4);
    }
    if (R(s, 0x854) & R(s, 0x850)) {
        pending |= BIT(5);
    }
    R(s, 0) = pending;
    qemu_set_irq(s->irq, !s->reset && s->powered && (pending & R(s, 4)));
}

static void mtu3_endpoint_reset(MTU3State *s, bool in, unsigned n)
{
    memset(&s->ep[in][n], 0, sizeof(s->ep[in][n]));
    R(s, EP_CSR(in, n)) = n ? (in ? BIT(24) : 0) : 64;
    if (n) {
        R(s, EP_CSR(in, n) + 4) = 0;
        R(s, EP_CSR(in, n) + 8) = 0;
        R(s, Q_CSR(in, n)) = 0;
        R(s, Q_CSR(in, n) + 4) = 0;
        R(s, Q_CSR(in, n) + 8) = 0;
        R(s, Q_HIGH(in, n)) = 0;
    }
    R(s, 0x080) &= ~BIT(n + (in ? 0 : 16));
    R(s, 0x700) &= ~BIT(n + (in ? 0 : 16));
}

static void mtu3_reset_state(MTU3State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    for (unsigned n = 0; n < MTU3_ENDPOINTS; n++) {
        mtu3_endpoint_reset(s, false, n);
        mtu3_endpoint_reset(s, true, n);
    }
    mtu3_update_irq(s);
    mtu3_ecm_reset(s);
}

static void mtu3_qmu_pointer(MTU3State *s, bool in, unsigned n, hwaddr addr)
{
    s->ep[in][n].current_gpd = addr;
    R(s, Q_CSR(in, n) + 8) = addr;
    R(s, Q_HIGH(in, n)) = (R(s, Q_HIGH(in, n)) & ~0xf00) |
                          (((addr >> 32) & 15) << 8);
}

static void mtu3_qmu_error(MTU3State *s, bool in, unsigned n, bool checksum)
{
    s->ep[in][n].active = false;
    R(s, in ? 0x780 : 0x7c0) |= BIT(n + (checksum ? 0 : 16));
    mtu3_update_irq(s);
}

static bool mtu3_dma(hwaddr address, void *data, size_t size, bool write)
{
    return address_space_rw(&address_space_memory, address,
                            MEMTXATTRS_UNSPECIFIED, data, size, write) == MEMTX_OK;
}

static bool mtu3_qmu_complete(MTU3State *s, bool in, unsigned n,
                               uint8_t gpd[16])
{
    MTU3Endpoint *ep = &s->ep[in][n];
    uint32_t flags = ldl_le_p(gpd), info = ldl_le_p(gpd + 12);
    bool extended = R(s, 0x428) & BIT(n + (in ? 0 : 16));
    uint32_t ext = in ? flags : info;
    hwaddr next = (uint32_t)ldl_le_p(gpd + 4) |
        (uint64_t)((ext >> (extended ? 28 : 20)) & 15) << 32;

    if (!in) {
        info = (info & ~(extended ? 0xfffff : 0xffff)) | ep->transferred;
        stl_le_p(gpd + 12, info);
    }
    stl_le_p(gpd, flags & ~BIT(0));
    if (!mtu3_dma(ep->current_gpd, gpd, 16, true)) {
        mtu3_qmu_error(s, in, n, false);
        return false;
    }
    if (!in) {
        R(s, Q_CSR(in, n) + 12) = ep->current_gpd;
        R(s, Q_HIGH(in, n)) = (R(s, Q_HIGH(in, n)) & ~0xf0000) |
                              (((ep->current_gpd >> 32) & 15) << 16);
    }
    mtu3_qmu_pointer(s, in, n, next);
    ep->transferred = 0;
    if (flags & BIT(7)) {
        R(s, 0x700) |= BIT(n + (in ? 0 : 16));
    }
    mtu3_update_irq(s);
    return true;
}

/* Fetch descriptors, including bypass entries which consume no USB token.
 * Bound a malformed circular bypass list so the guest cannot stall QEMU.
 */
static bool mtu3_qmu_fetch(MTU3State *s, bool in, unsigned n, uint8_t gpd[16])
{
    MTU3Endpoint *ep = &s->ep[in][n];

    for (unsigned count = 0; ep->active && count < 256; count++) {
        uint32_t flags;

        if ((ep->current_gpd & 15) ||
            !mtu3_dma(ep->current_gpd, gpd, 16, false)) {
            break;
        }
        flags = ldl_le_p(gpd);
        if (!(flags & BIT(0))) {
            ep->active = false;
            R(s, 0x710) |= in ? BIT(0) : BIT(16);
            mtu3_update_irq(s);
            return false;
        }
        if (flags & BIT(1)) {
            break; /* Linux's descriptor interface has no buffer chains. */
        }
        if (flags & BIT(2)) {
            if (!mtu3_qmu_complete(s, in, n, gpd)) {
                return false;
            }
            continue;
        }
        /* Checksums are optional; this vendor driver leaves QCR0 disabled. */
        if (R(s, 0x400) & BIT(n + (in ? 0 : 16))) {
            uint8_t sum = 0;
            for (unsigned i = 0; i < 16; i++) {
                sum += gpd[i];
            }
            if (sum != 0xff) {
                break;
            }
        }
        return true;
    }
    if (ep->active) {
        mtu3_qmu_error(s, in, n, true);
    }
    return false;
}

/* One GPD may span multiple packets. Ownership changes only after transfer
 * completion (or an explicit bypass), never merely because a queue starts.
 */
static int mtu3_qmu_packet(MTU3State *s, bool in, unsigned n, uint8_t *data,
                           size_t size)
{
    MTU3Endpoint *ep = &s->ep[in][n];
    uint8_t gpd[16];
    uint32_t flags, info, ext, limit, amount, packet_max;
    bool extended = R(s, 0x428) & BIT(n + (in ? 0 : 16));
    bool append_zlp, complete;
    hwaddr buffer;

    if (!mtu3_qmu_fetch(s, in, n, gpd)) {
        return -EAGAIN;
    }
    flags = ldl_le_p(gpd);
    info = ldl_le_p(gpd + 12);
    ext = in ? flags : info;
    buffer = (uint32_t)ldl_le_p(gpd + 8) |
             (uint64_t)((ext >> (extended ? 24 : 16)) & 15) << 32;
    limit = in ? info & (extended ? 0xfffff : 0xffff) :
                 flags >> (extended ? 12 : 16);
    packet_max = R(s, EP_CSR(in, n)) & 0x7ff;
    if (!packet_max || ep->transferred > limit || (!in && size > packet_max) ||
        (in && !extended && !limit)) {
        mtu3_qmu_error(s, in, n, false);
        return -EIO;
    }
    if (!in && size > limit - ep->transferred) {
        mtu3_qmu_error(s, in, n, false);
        return -EIO;
    }
    amount = in ? MIN(packet_max, limit - ep->transferred) : size;
    if (in && size < amount) {
        return -ENOSPC;
    }
    if (amount && !mtu3_dma(buffer + ep->transferred, data, amount, !in)) {
        mtu3_qmu_error(s, in, n, false);
        return -EIO;
    }
    ep->transferred += amount;
    append_zlp = in && (R(s, 0x404) & BIT(n)) &&
                 (extended ? flags & BIT(6) : info & BIT(29));
    complete = ep->transferred == limit || (!in && amount < packet_max);
    if (in && amount == packet_max && append_zlp) {
        complete = false; /* A following host token consumes the appended ZLP. */
    }
    if (!in && !amount) {
        if (!(R(s, 0x40c) & BIT(n))) {
            R(s, 0x7d0) |= BIT(16 + n);
        }
        complete = R(s, 0x40c) & BIT(16 + n);
    }
    if (complete && !mtu3_qmu_complete(s, in, n, gpd)) {
        return -EIO;
    }
    mtu3_update_irq(s);
    return amount;
}

bool mtu3_connected(MTU3State *s)
{
    bool vbus = (R(s, 0xc84) & 1) ? R(s, 0xc84) & 2 : s->vbus;

    return !s->reset && s->powered && vbus && (R(s, 0x2404) & BIT(6));
}

int mtu3_out_packet(MTU3State *s, unsigned n, const uint8_t *data,
                    size_t length, bool setup)
{
    MTU3Endpoint *ep;
    uint32_t csr;

    if (n > s->endpoints || !mtu3_connected(s)) {
        return -EAGAIN;
    }
    if (setup && (n || length != 8)) {
        return -EIO;
    }
    ep = &s->ep[false][n];
    csr = R(s, EP_CSR(false, n));
    if (!setup && (csr & BIT(n ? 21 : 25))) {
        R(s, EP_CSR(false, n)) |= BIT(22);
        R(s, 0x80) |= n ? BIT(16 + n) : BIT(0);
        mtu3_update_irq(s);
        return -EPIPE;
    }
    if (n && (csr & BIT(29))) {
        return mtu3_qmu_packet(s, false, n, (uint8_t *)data, length);
    }
    if (length > MIN(MTU3_FIFO_SIZE, csr & (n ? 0x7ff : 0x3ff))) {
        return -EIO;
    }
    /* The device MAC consumes the OUT status ZLP after a control read. */
    if (!n && !setup && !length && (csr & BIT(20))) {
        if (!(csr & BIT(19))) {
            return -EAGAIN;
        }
        R(s, 0x100) &= ~(BIT(19) | BIT(20));
        return 0;
    }
    if (!setup && (csr & BIT(16))) {
        return -EAGAIN;
    }
    if (setup) {
        if (csr & (BIT(16) | BIT(18) | BIT(19))) {
            R(s, 0x80) |= BIT(16); /* Previous control transfer aborted. */
        }
        R(s, 0x100) &= ~(BIT(25) | BIT(22) | BIT(20) | BIT(19) | BIT(18) | BIT(16));
        s->ep[true][0].length = s->ep[true][0].position = 0;
    }
    memcpy(ep->fifo, data, length);
    ep->length = length;
    ep->position = 0;
    R(s, EP_CSR(false, n)) |= setup ? BIT(17) : BIT(16);
    if (!n) {
        R(s, 0x108) = length;
    }
    R(s, 0x80) |= n ? BIT(16 + n) : BIT(0);
    mtu3_update_irq(s);
    return length;
}

int mtu3_in_packet(MTU3State *s, unsigned n, uint8_t *data, size_t capacity)
{
    MTU3Endpoint *ep;
    uint32_t csr, ready;
    size_t amount;

    if (n > s->endpoints || !mtu3_connected(s)) {
        return -EAGAIN;
    }
    ep = &s->ep[true][n];
    csr = R(s, EP_CSR(true, n));
    if (csr & BIT(n ? 21 : 25)) {
        R(s, EP_CSR(true, n)) |= BIT(22);
        R(s, 0x80) |= BIT(n);
        mtu3_update_irq(s);
        return -EPIPE;
    }
    ready = BIT(n ? 16 : 18);
    /* An explicitly queued BMU packet takes precedence over QMU (old-IP ZLP). */
    if (n && (csr & BIT(29)) && !(csr & ready)) {
        return mtu3_qmu_packet(s, true, n, data, capacity);
    }
    if (!(csr & ready)) {
        return -EAGAIN;
    }
    amount = ep->length - ep->position;
    if (capacity < amount) {
        return -ENOSPC;
    }
    memcpy(data, ep->fifo + ep->position, amount);
    ep->position += amount;
    if (ep->position == ep->length) {
        ep->length = ep->position = 0;
        R(s, EP_CSR(true, n)) &= ~(ready | (n ? BIT(25) : BIT(23)));
        if (!n && (csr & BIT(19))) {
            R(s, 0x100) &= ~BIT(19);
        }
        if (n) {
            R(s, EP_CSR(true, n)) |= BIT(24);
        }
        R(s, 0x80) |= BIT(n);
        mtu3_update_irq(s);
    }
    return amount;
}

void mtu3_bus_reset(MTU3State *s)
{
    if (!mtu3_connected(s)) {
        return;
    }
    for (unsigned n = 0; n <= s->endpoints; n++) {
        mtu3_endpoint_reset(s, false, n);
        mtu3_endpoint_reset(s, true, n);
    }
    R(s, 0x800) = (R(s, 0x800) & ~0x7f000007) |
                  ((R(s, 0x2404) & BIT(5)) ? 3 : 1);
    R(s, 0x241c) |= BIT(2);
    R(s, 0x854) |= BIT(0);
    mtu3_update_irq(s);
}

static uint64_t mtu3_read(void *opaque, hwaddr offset, unsigned size)
{
    MTU3State *s = opaque;

    if (offset >= 0x300 && offset < 0x340) {
        MTU3Endpoint *ep = &s->ep[false][(offset - 0x300) / 4];
        uint32_t value = 0;

        for (unsigned i = 0; i < size && ep->position < ep->length; i++) {
            value |= (uint32_t)ep->fifo[ep->position++] << (i * 8);
        }
        return value;
    }
    if (offset >= 0x500 && offset < 0x700 && (offset & 15) == 0) {
        unsigned n = (offset & 0xff) / 16;
        return s->ep[offset < 0x600][n].active ? BIT(15) : 0;
    }
    if (offset >= 0x21c && offset < 0x300 && (offset & 15) == 12) {
        return s->ep[false][(offset & 0xff) / 16].length;
    }
    switch (offset) {
    case 0xc04: return 512;
    case 0xc08:
    case 0xc0c: return s->fifo_bytes;
    case 0xc10: return s->endpoints | s->endpoints << 8;
    case 0xc84: return R(s, offset) | BIT(31);
    default: return R(s, offset);
    }
}

static void mtu3_register_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    MTU3State *s = opaque;

    if (offset >= 0x300 && offset < 0x340) {
        unsigned n = (offset - 0x300) / 4;
        MTU3Endpoint *ep = &s->ep[true][n];

        for (unsigned i = 0; i < size && ep->length < MTU3_FIFO_SIZE; i++) {
            ep->fifo[ep->length++] = value >> (i * 8);
        }
        if (n) {
            R(s, EP_CSR(true, n)) &= ~BIT(24);
        }
        if (ep->length >= (R(s, EP_CSR(true, n)) & (n ? 0x7ff : 0x3ff))) {
            R(s, EP_CSR(true, n)) |= BIT(n ? 25 : 23);
        }
        return;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(mtu3_irq_groups); i++) {
        unsigned base = mtu3_irq_groups[i];
        if (offset < base || offset >= base + 16) {
            continue;
        }
        switch (offset - base) {
        case 0:
            if (base) {
                R(s, base) &= ~value;
            }
            break;
        case 4: R(s, base + 4) = value; break;
        case 8: R(s, base + 4) |= value; break;
        case 12: R(s, base + 4) &= ~value; break;
        }
        mtu3_update_irq(s);
        return;
    }
    if (offset >= 0x510 && offset < 0x700 && (offset & 15) == 0 &&
        (offset & 0xff)) {
        bool in = offset < 0x600;
        unsigned n = (offset & 0xff) / 16;
        MTU3Endpoint *ep = &s->ep[in][n];

        if (value & BIT(2)) {
            ep->active = false;
        } else if (value & 3) {
            if (value & 1) {
                mtu3_qmu_pointer(s, in, n, R(s, offset + 4) |
                    (uint64_t)(R(s, Q_HIGH(in, n)) & 15) << 32);
                ep->transferred = 0;
            }
            ep->active = true;
            if (s->powered && !s->reset) {
                uint8_t gpd[16];
                mtu3_qmu_fetch(s, in, n, gpd);
            }
        }
        return;
    }
    if (offset >= 0x110 && offset < 0x300 && (offset & 15) == 0 &&
        (offset & 0xff)) {
        bool in = offset < 0x200;
        unsigned n = (offset & 0xff) / 16;
        uint32_t old = R(s, offset), w1c = BIT(22) | (in ? 0 : BIT(16));

        R(s, offset) = (value & ~w1c) | (old & w1c & ~value);
        if (in && (value & BIT(20))) {
            s->ep[in][n].length = s->ep[in][n].position = 0;
            R(s, offset) &= ~(BIT(20) | BIT(16));
        }
        if (in) {
            R(s, offset) &= ~(BIT(24) | BIT(25));
            R(s, offset) |= s->ep[in][n].length ? 0 : BIT(24);
        }
    } else if (offset == 0x100) {
        uint32_t old = R(s, offset), w1c = BIT(16) | BIT(17) | BIT(22);
        R(s, offset) = (value & ~(w1c | BIT(23))) |
                       (old & w1c & ~value) | (old & BIT(23));
        if ((value & BIT(19)) && !(value & (BIT(18) | BIT(20)))) {
            /* Hardware generates the IN status ZLP for no-data / OUT requests. */
            s->ep[true][0].length = s->ep[true][0].position = 0;
            R(s, offset) |= BIT(18);
        }
    } else if (offset == 0x804) {
        for (unsigned n = 0; n <= s->endpoints; n++) {
            if (value & BIT(n)) {
                mtu3_endpoint_reset(s, false, n);
                if (!n) {
                    mtu3_endpoint_reset(s, true, 0);
                }
            }
            if (n && (value & BIT(16 + n))) {
                mtu3_endpoint_reset(s, true, n);
            }
        }
        R(s, offset) = value;
    } else if (offset == 0x854 || offset == 0x1540 || offset == 0x241c) {
        R(s, offset) &= ~value;
    } else if (offset == 0x800) {
        R(s, offset) = (value & ~7U) | (R(s, offset) & 7);
    } else if (offset >= 0x480 && offset < 0x500) {
        R(s, offset) = (R(s, offset) & ~15U) | (value & 15);
    } else if (offset == 0x108 ||
               (offset >= 0x21c && offset < 0x300 && (offset & 15) == 12) || (offset >= 0xc04 && offset <= 0xc10) ||
               (offset >= 0x510 && offset < 0x700 && (offset & 15) == 8)) {
        return;
    } else {
        R(s, offset) = value;
    }
    mtu3_update_irq(s);
}

static void mtu3_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    mtu3_register_write(opaque, offset, value, size);
    mtu3_ecm_kick(opaque);
}

static void mtu3_reset_input(void *opaque, int n, int level)
{
    MTU3State *s = opaque;

    s->reset = level;
    if (level) {
        mtu3_reset_state(s);
    }
    mtu3_update_irq(s);
}

static void mtu3_vbus_input(void *opaque, int n, int level)
{
    MTU3State *s = opaque;

    if (s->vbus != !!level) {
        s->vbus = level;
        R(s, 0x1540) |= level ? BIT(15) : BIT(16);
        if (!level) {
            R(s, 0x241c) |= BIT(5);
        }
        mtu3_update_irq(s);
        mtu3_ecm_kick(s);
    }
}

static void mtu3_power_input(void *opaque, int n, int level)
{
    MTU3State *s = opaque;
    if (s->powered == !!level) {
        return;
    }
    s->powered = level;
    if (level && !s->reset) {
        for (unsigned ep = 1; ep <= s->endpoints; ep++) {
            for (unsigned in = 0; in < 2; in++) {
                if (s->ep[in][ep].active) {
                    uint8_t gpd[16];
                    mtu3_qmu_fetch(s, in, ep, gpd);
                }
            }
        }
    }
    mtu3_update_irq(s);
    mtu3_ecm_kick(s);
}

static bool mtu3_accepts(void *opaque, hwaddr offset, unsigned size,
                         bool is_write, MemTxAttrs attrs)
{
    /* FIFO is a byte/halfword/word streaming port, other registers are words. */
    return !(offset & 3) && (size == 4 ||
           (offset >= 0x300 && offset < 0x340 && (size == 1 || size == 2)));
}

static const MemoryRegionOps mtu3_ops = {
    .read = mtu3_read,
    .write = mtu3_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4,
               .accepts = mtu3_accepts },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void mtu3_reset(DeviceState *dev)
{
    MTU3State *s = MTU3(dev);
    /* Reset/power pins belong to IPPC; preserve their externally driven state. */
    mtu3_reset_state(s);
}

static void mtu3_realize(DeviceState *dev, Error **errp)
{
    MTU3State *s = MTU3(dev);

    if (!s->endpoints || s->endpoints >= MTU3_ENDPOINTS ||
        !s->fifo_bytes || s->fifo_bytes > 64 * 1024) {
        error_setg(errp, "mtu3: invalid endpoint count or FIFO capacity");
        return;
    }
    mtu3_ecm_realize(s);
}

static void mtu3_unrealize(DeviceState *dev)
{
    mtu3_ecm_unrealize(MTU3(dev));
}

static void mtu3_init(Object *obj)
{
    MTU3State *s = MTU3(obj);

    memory_region_init_io(&s->iomem, obj, &mtu3_ops, s, TYPE_MTU3, 0x2e00);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    qdev_init_gpio_in_named(DEVICE(s), mtu3_reset_input, "reset", 1);
    qdev_init_gpio_in_named(DEVICE(s), mtu3_vbus_input, "vbus", 1);
    qdev_init_gpio_in_named(DEVICE(s), mtu3_power_input, "power", 1);
}

static const Property mtu3_properties[] = {
    DEFINE_NIC_PROPERTIES(MTU3State, nic_conf),
    DEFINE_PROP_UINT32("endpoints", MTU3State, endpoints, 15),
    DEFINE_PROP_UINT32("fifo-bytes", MTU3State, fifo_bytes, 32 * 1024),
};

static void mtu3_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->desc = "MediaTek USB device MAC and QMU";
    dc->realize = mtu3_realize;
    dc->unrealize = mtu3_unrealize;
    device_class_set_legacy_reset(dc, mtu3_reset);
    device_class_set_props(dc, mtu3_properties);
}

static const TypeInfo mtu3_info = {
    .name = TYPE_MTU3,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MTU3State),
    .instance_init = mtu3_init,
    .class_init = mtu3_class_init,
};

static void mtu3_register_types(void)
{
    type_register_static(&mtu3_info);
}
type_init(mtu3_register_types)
