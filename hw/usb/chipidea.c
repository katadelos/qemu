/*
 * Copyright (c) 2018, Impinj, Inc.
 *
 * Chipidea USB block emulation code
 *
 * Author: Andrey Smirnov <andrew.smirnov@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/chipidea.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "system/address-spaces.h"
#include "net/net.h"
#include "qemu/module.h"

enum {
    CHIPIDEA_USBx_DCIVERSION   = 0x000,
    CHIPIDEA_USBx_DCCPARAMS    = 0x004,
    CHIPIDEA_USBx_DCCPARAMS_DEN = 8,
    CHIPIDEA_USBx_DCCPARAMS_DC = BIT(7),
    CHIPIDEA_USBx_DCCPARAMS_HC = BIT(8),
};

static DeviceRealize chipidea_parent_realize;

enum {
    CI_OTGSC = 0x00,
    CI_USBMODE = 0x04,
    CI_ENDPTSETUPSTAT = 0x08,
    CI_ENDPTPRIME = 0x0c,
    CI_ENDPTFLUSH = 0x10,
    CI_ENDPTSTATUS = 0x14,
    CI_ENDPTCOMPLETE = 0x18,
    CI_ENDPTCTRL0 = 0x1c,
};

#define CI_DTD_TERMINATE BIT(0)
#define CI_DTD_ACTIVE BIT(7)
#define CI_DTD_LENGTH_MASK 0x7fff0000
#define CI_USBSTS_INT BIT(0)
#define CI_USBSTS_PORT_CHANGE BIT(2)
#define CI_USBSTS_RESET BIT(6)
#define CI_PORTSC_CONNECTED BIT(0)
#define CI_PORTSC_ENABLED BIT(2)
#define CI_PORTSC_HIGH_SPEED BIT(27)

static void chipidea_raise_irq(ChipideaState *ci)
{
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    ehci->usbsts |= CI_USBSTS_INT;
    if (ehci->usbintr & CI_USBSTS_INT) {
        qemu_set_irq(ehci->irq, 1);
    }
}

static void chipidea_inject_setup(ChipideaState *ci,
                                  const uint8_t setup[8])
{
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;
    hwaddr qh_addr = ehci->asynclistaddr & 0xfffff800;

    if (!qh_addr || (ci->dc_mode & 3) != 2) {
        return;
    }
    dma_memory_write(&address_space_memory, qh_addr + 40, setup, 8,
                     MEMTXATTRS_UNSPECIFIED);
    ci->endptsetupstat |= BIT(0);
    chipidea_raise_irq(ci);
}

static void chipidea_config_timer(void *opaque)
{
    ChipideaState *ci = opaque;
    static const uint8_t set_configuration[8] = {
        0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    static const uint8_t set_interface[8] = {
        0x01, 0x0b, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    };

    if (!ci->gadget || ci->gadget_configured) {
        return;
    }
    if (ci->gadget_config_phase == 0) {
        EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

        ehci->portsc[0] = CI_PORTSC_CONNECTED | CI_PORTSC_ENABLED |
                          CI_PORTSC_HIGH_SPEED;
        ehci->usbsts |= CI_USBSTS_RESET;
        if (ehci->usbintr & CI_USBSTS_RESET) {
            qemu_set_irq(ehci->irq, 1);
        }
        ci->gadget_config_phase = 1;
    } else if (ci->gadget_config_phase == 1) {
        EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

        ehci->usbsts |= CI_USBSTS_PORT_CHANGE;
        if (ehci->usbintr & CI_USBSTS_PORT_CHANGE) {
            qemu_set_irq(ehci->irq, 1);
        }
        ci->gadget_config_phase = 2;
    } else if (ci->gadget_config_phase == 2) {
        chipidea_inject_setup(ci, set_configuration);
        ci->gadget_config_phase = 3;
    } else if (ci->gadget_config_phase == 3) {
        chipidea_inject_setup(ci, set_interface);
        ci->gadget_config_phase = 4;
    } else {
        ci->gadget_configured = true;
        if (ci->gadget_nic) {
            qemu_flush_queued_packets(qemu_get_queue(ci->gadget_nic));
        }
        return;
    }
    timer_mod(ci->gadget_config_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
}

static bool chipidea_dtd(ChipideaState *ci, unsigned ep, bool in,
                         uint8_t *buf, size_t *length)
{
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;
    uint32_t qh_addr = (ehci->asynclistaddr & 0xfffff800) +
                       (ep * 2 + in) * 64;
    uint32_t dtd_addr, next_dtd, token, pages[5];
    size_t capacity, transfer, done = 0;
    unsigned i;

    if (!ehci->asynclistaddr ||
        dma_memory_read(&address_space_memory, qh_addr + 8, &dtd_addr, 4,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    dtd_addr = le32_to_cpu(dtd_addr);
    if (dtd_addr & CI_DTD_TERMINATE) {
        return false;
    }
    dtd_addr &= ~0x1fU;
    if (dma_memory_read(&address_space_memory, dtd_addr, &next_dtd, 4,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK ||
        dma_memory_read(&address_space_memory, dtd_addr + 4, &token, 4,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    token = le32_to_cpu(token);
    if (!(token & CI_DTD_ACTIVE)) {
        return false;
    }
    capacity = (token & CI_DTD_LENGTH_MASK) >> 16;
    transfer = in ? capacity : MIN(capacity, *length);
    if (in) {
        *length = transfer;
    }
    if (dma_memory_read(&address_space_memory, dtd_addr + 8, pages,
                        sizeof(pages), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    for (i = 0; i < ARRAY_SIZE(pages) && done < transfer; i++) {
        hwaddr addr = le32_to_cpu(pages[i]);
        size_t chunk = MIN(transfer - done, 0x1000 - (addr & 0xfff));
        MemTxResult result;

        result = in ? dma_memory_read(&address_space_memory, addr, buf + done,
                                      chunk, MEMTXATTRS_UNSPECIFIED)
                    : dma_memory_write(&address_space_memory, addr, buf + done,
                                       chunk, MEMTXATTRS_UNSPECIFIED);
        if (result != MEMTX_OK) {
            return false;
        }
        done += chunk;
    }
    token &= ~(CI_DTD_ACTIVE | CI_DTD_LENGTH_MASK);
    token |= (capacity - done) << 16;
    token = cpu_to_le32(token);
    dma_memory_write(&address_space_memory, dtd_addr + 4, &token, 4,
                     MEMTXATTRS_UNSPECIFIED);
    dtd_addr = cpu_to_le32(dtd_addr);
    dma_memory_write(&address_space_memory, qh_addr + 4, &dtd_addr, 4,
                     MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, qh_addr + 8, &next_dtd, 4,
                     MEMTXATTRS_UNSPECIFIED);
    *length = done;
    return true;
}

static bool chipidea_dtd_ready(ChipideaState *ci, unsigned ep, bool in)
{
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;
    uint32_t qh_addr = (ehci->asynclistaddr & 0xfffff800) +
                       (ep * 2 + in) * 64;
    uint32_t dtd_addr, token;

    if (!ehci->asynclistaddr ||
        dma_memory_read(&address_space_memory, qh_addr + 8, &dtd_addr, 4,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    dtd_addr = le32_to_cpu(dtd_addr);
    if (dtd_addr & CI_DTD_TERMINATE) {
        return false;
    }
    dtd_addr &= ~0x1fU;
    if (dma_memory_read(&address_space_memory, dtd_addr + 4, &token, 4,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    return le32_to_cpu(token) & CI_DTD_ACTIVE;
}

static void chipidea_complete(ChipideaState *ci, unsigned ep, bool in)
{
    uint32_t bit = BIT(ep + (in ? 16 : 0));

    ci->endpointprime &= ~bit;
    if (chipidea_dtd_ready(ci, ep, in)) {
        ci->endptstatus |= bit;
    } else {
        ci->endptstatus &= ~bit;
    }
    ci->endptcomplete |= bit;
    chipidea_raise_irq(ci);
}

static void chipidea_process_in(ChipideaState *ci, uint32_t bits)
{
    uint8_t packet[0x4000];
    unsigned ep;

    ci->processing_in = true;
    for (ep = 0; ep < 8; ep++) {
        size_t length = sizeof(packet);

        if (!(bits & BIT(ep + 16)) ||
            !chipidea_dtd(ci, ep, true, packet, &length)) {
            continue;
        }
        if (ci->gadget_nic && ep != 0 && length >= 14 &&
            ((ci->endptctrl[ep] >> 18) & 3) == 2) {
            qemu_send_packet(qemu_get_queue(ci->gadget_nic), packet, length);
        }
        chipidea_complete(ci, ep, true);
    }
    ci->processing_in = false;
    if (ci->gadget_nic) {
        qemu_flush_queued_packets(qemu_get_queue(ci->gadget_nic));
    }
}

static bool chipidea_can_receive(NetClientState *nc)
{
    ChipideaState *ci = qemu_get_nic_opaque(nc);
    unsigned ep;

    if (ci->processing_in) {
        return false;
    }
    for (ep = 1; ep < 8; ep++) {
        if ((ci->endptstatus & BIT(ep)) &&
            !(ci->endptcomplete & BIT(ep)) &&
            chipidea_dtd_ready(ci, ep, false) &&
            (ci->endptctrl[ep] & BIT(7))) {
            return true;
        }
    }
    return false;
}

static ssize_t chipidea_receive(NetClientState *nc, const uint8_t *buf,
                                size_t size)
{
    ChipideaState *ci = qemu_get_nic_opaque(nc);
    unsigned ep;

    for (ep = 1; ep < 8; ep++) {
        size_t length = size;

        if (!(ci->endptstatus & BIT(ep)) ||
            !(ci->endptctrl[ep] & BIT(7)) ||
            !chipidea_dtd(ci, ep, false, (uint8_t *)buf, &length)) {
            continue;
        }
        chipidea_complete(ci, ep, false);
        return length;
    }
    return 0;
}

static NetClientInfo chipidea_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = chipidea_can_receive,
    .receive = chipidea_receive,
};

static uint64_t chipidea_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    ChipideaState *ci = opaque;

    switch (offset) {
    case CI_OTGSC: return ci->otgsc | BIT(11) | BIT(8);
    case CI_USBMODE: return ci->dc_mode;
    case CI_ENDPTSETUPSTAT: return ci->endptsetupstat;
    case CI_ENDPTPRIME: return ci->endpointprime;
    case CI_ENDPTFLUSH: return ci->endptflush;
    case CI_ENDPTSTATUS: return ci->endptstatus;
    case CI_ENDPTCOMPLETE: return ci->endptcomplete;
    default:
        if (offset >= CI_ENDPTCTRL0 && offset < CI_ENDPTCTRL0 + 32) {
            return ci->endptctrl[(offset - CI_ENDPTCTRL0) / 4];
        }
        return 0;
    }
}

static void chipidea_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    ChipideaState *ci = opaque;
    unsigned index;

    switch (offset) {
    case CI_OTGSC:
        ci->otgsc = value & ~BIT(19);
        break;
    case CI_USBMODE:
        ci->dc_mode = value;
        break;
    case CI_ENDPTSETUPSTAT:
        ci->endptsetupstat &= ~value;
        break;
    case CI_ENDPTPRIME:
        ci->endpointprime |= value;
        ci->endptstatus |= value;
        chipidea_process_in(ci, value);
        for (index = 0; index < ARRAY_SIZE(ci->endptctrl); index++) {
            uint32_t out_bit = BIT(index);

            if ((value & out_bit) &&
                !chipidea_dtd_ready(ci, index, false)) {
                ci->endptstatus &= ~out_bit;
            }
        }
        ci->endpointprime &= ~value;
        if (ci->gadget_nic) {
            qemu_flush_queued_packets(qemu_get_queue(ci->gadget_nic));
        }
        break;
    case CI_ENDPTFLUSH:
        /*
         * Once STOP has destroyed the operational state, endpoint commands
         * do not complete merely because software selects device mode again.
         * The device command engine has to run at least once first.  Stopping
         * an otherwise live controller does not discard that initialized
         * state, which is why the ordinary disconnect path can still flush.
         */
        if ((ci->dc_mode & 3) != 2 || !ci->endpoint_commands_ready) {
            ci->endptflush |= value;
            break;
        }
        ci->endpointprime &= ~value;
        ci->endptstatus &= ~value;
        ci->endptflush &= ~value;
        break;
    case CI_ENDPTCOMPLETE:
        ci->endptcomplete &= ~value;
        if (!ci->endptcomplete) {
            EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;
            ehci->usbsts &= ~CI_USBSTS_INT;
            qemu_set_irq(ehci->irq, 0);
        }
        chipidea_process_in(ci, ci->endptstatus & 0xffff0000);
        if (ci->gadget_nic) {
            qemu_flush_queued_packets(qemu_get_queue(ci->gadget_nic));
        }
        break;
    default:
        if (offset >= CI_ENDPTCTRL0 && offset < CI_ENDPTCTRL0 + 32) {
            index = (offset - CI_ENDPTCTRL0) / 4;
            ci->endptctrl[index] = value;
            if (ci->gadget_configured && index > 0) {
                unsigned i;
                bool enabled = false;

                for (i = 1; i < ARRAY_SIZE(ci->endptctrl); i++) {
                    enabled |= ci->endptctrl[i] & (BIT(23) | BIT(7));
                }
                if (!enabled) {
                    ci->gadget_configured = false;
                    ci->gadget_config_phase = 0;
                    timer_mod(ci->gadget_config_timer,
                              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 250);
                }
            }
        }
        break;
    }
}

static const struct MemoryRegionOps chipidea_ops = {
    .read = chipidea_read,
    .write = chipidea_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        /*
         * Our device would not work correctly if the guest was doing
         * unaligned access. This might not be a limitation on the
         * real device but in practice there is no reason for a guest
         * to access this device unaligned.
         */
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static uint64_t chipidea_command_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    ChipideaState *ci = opaque;

    return SYS_BUS_EHCI(ci)->ehci.usbcmd;
}

static void chipidea_command_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    ChipideaState *ci = opaque;
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;
    bool was_running = ehci->usbcmd & BIT(0);

    if (value & BIT(1)) {
        ci->dc_mode = 0;
        ci->endptsetupstat = 0;
        ci->endpointprime = 0;
        ci->endptflush = 0;
        ci->endptstatus = 0;
        ci->endptcomplete = 0;
        memset(ci->endptctrl, 0, sizeof(ci->endptctrl));
        ci->endpoint_commands_ready = false;
    }

    ehci->usbcmd = value & ~BIT(1);
    if (value & BIT(0)) {
        ci->endpoint_commands_ready = true;
    }
    if (!(value & BIT(0))) {
        ci->gadget_configured = false;
        ci->gadget_config_phase = 0;
        if (ci->gadget_config_timer) {
            /* OTG gadget binds do not always restart RUN until VBUS work. */
            timer_mod(ci->gadget_config_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 2000);
        }
    } else if (!was_running && ci->gadget_config_timer) {
        ci->gadget_configured = false;
        ci->gadget_config_phase = 0;
        timer_mod(ci->gadget_config_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 250);
    }
}

static const struct MemoryRegionOps chipidea_command_ops = {
    .read = chipidea_command_read,
    .write = chipidea_command_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static uint64_t chipidea_dc_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    switch (offset) {
    case CHIPIDEA_USBx_DCIVERSION:
        return 0x1;
    case CHIPIDEA_USBx_DCCPARAMS:
        /*
         * i.MX6 USB OTG is dual-role and reports eight bidirectional
         * endpoints.  Reporting host-only makes the Freescale UDC driver
         * reject the controller before it can finish probing.
         */
        return CHIPIDEA_USBx_DCCPARAMS_HC |
               CHIPIDEA_USBx_DCCPARAMS_DC |
               CHIPIDEA_USBx_DCCPARAMS_DEN;
    }

    return 0;
}

static void chipidea_dc_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
}

static const struct MemoryRegionOps chipidea_dc_ops = {
    .read = chipidea_dc_read,
    .write = chipidea_dc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        /*
         * Our device would not work correctly if the guest was doing
         * unaligned access. This might not be a limitation on the real
         * device but in practice there is no reason for a guest to access
         * this device unaligned.
         */
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void chipidea_init(Object *obj)
{
    EHCIState *ehci = &SYS_BUS_EHCI(obj)->ehci;
    ChipideaState *ci = CHIPIDEA(obj);
    int i;

    for (i = 0; i < ARRAY_SIZE(ci->iomem); i++) {
        const struct {
            const char *name;
            hwaddr offset;
            uint64_t size;
            const struct MemoryRegionOps *ops;
        } regions[ARRAY_SIZE(ci->iomem)] = {
            /*
             * Registers located between offsets 0x000 and 0xFC
             */
            {
                .name   = TYPE_CHIPIDEA ".misc",
                .offset = 0x000,
                .size   = 0x100,
                .ops    = &chipidea_ops,
            },
            /*
             * Registers located between offsets 0x1A4 and 0x1DC
             */
            {
                .name   = TYPE_CHIPIDEA ".endpoints",
                .offset = 0x1A4,
                .size   = 0x1DC - 0x1A4 + 4,
                .ops    = &chipidea_ops,
            },
            /*
             * USB_x_DCIVERSION and USB_x_DCCPARAMS
             */
            {
                .name   = TYPE_CHIPIDEA ".dc",
                .offset = 0x120,
                .size   = 8,
                .ops    = &chipidea_dc_ops,
            },
        };

        memory_region_init_io(&ci->iomem[i],
                              obj,
                              regions[i].ops,
                              ci,
                              regions[i].name,
                              regions[i].size);

        memory_region_add_subregion(&ehci->mem,
                                    regions[i].offset,
                                    &ci->iomem[i]);
    }
}

static void chipidea_realize(DeviceState *dev, Error **errp)
{
    ChipideaState *ci = CHIPIDEA(dev);

    chipidea_parent_realize(dev, errp);
    if (*errp) {
        return;
    }

    if (ci->gadget) {
        if (ci->gadget_nic_conf.peers.ncs[0]) {
            ci->gadget_nic = qemu_new_nic(&chipidea_net_info,
                                          &ci->gadget_nic_conf,
                                          object_get_typename(OBJECT(dev)),
                                          dev->id,
                                          &dev->mem_reentrancy_guard, ci);
            qemu_format_nic_info_str(qemu_get_queue(ci->gadget_nic),
                                     ci->gadget_nic_conf.macaddr.a);
        }
        ci->gadget_config_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                                chipidea_config_timer, ci);
        memory_region_init_io(&ci->dc_command_iomem, OBJECT(ci),
                              &chipidea_command_ops, ci,
                              TYPE_CHIPIDEA ".dc-command", 4);
        memory_region_add_subregion_overlap(&SYS_BUS_EHCI(ci)->ehci.mem,
                                            0x140,
                                            &ci->dc_command_iomem, 2);
        /* The development rootfs switches from USB storage to g_ether. */
        timer_mod(ci->gadget_config_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 30000);
    }
}

static const Property chipidea_properties[] = {
    DEFINE_PROP_BOOL("gadget", ChipideaState, gadget, false),
    DEFINE_NIC_PROPERTIES(ChipideaState, gadget_nic_conf),
};

static void chipidea_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(klass);

    /*
     * Offsets used were taken from i.MX7Dual Applications Processor
     * Reference Manual, Rev 0.1, p. 3177, Table 11-59
     */
    sec->capsbase   = 0x100;
    sec->opregbase  = 0x140;
    sec->portnr     = 1;

    set_bit(DEVICE_CATEGORY_USB, dc->categories);
    dc->desc = "Chipidea USB Module";
    device_class_set_parent_realize(dc, chipidea_realize,
                                    &chipidea_parent_realize);
    device_class_set_props(dc, chipidea_properties);
}

static const TypeInfo chipidea_info = {
    .name          = TYPE_CHIPIDEA,
    .parent        = TYPE_SYS_BUS_EHCI,
    .instance_size = sizeof(ChipideaState),
    .instance_init = chipidea_init,
    .class_init    = chipidea_class_init,
};

static void chipidea_register_type(void)
{
    type_register_static(&chipidea_info);
}
type_init(chipidea_register_type)
