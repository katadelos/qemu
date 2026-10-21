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
#include "hw/usb/ehci-regs.h"
#include "hw/usb/chipidea.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "system/address-spaces.h"
#include "net/net.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "migration/vmstate.h"

enum {
    CHIPIDEA_USBx_DCIVERSION   = 0x000,
    CHIPIDEA_USBx_DCCPARAMS    = 0x004,
    CHIPIDEA_USBx_DCCPARAMS_DEN = 8,
    CHIPIDEA_USBx_DCCPARAMS_DC = BIT(7),
    CHIPIDEA_USBx_DCCPARAMS_HC = BIT(8),
};

static DeviceRealize chipidea_parent_realize;
static ResettablePhases chipidea_parent_reset;

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
#define CI_USBSTS_SUSPEND BIT(8)
#define CI_PORTSC_CONNECTED BIT(0)
#define CI_PORTSC_ENABLED BIT(2)
#define CI_PORTSC_FORCE_RESUME BIT(6)
#define CI_PORTSC_SUSPEND BIT(7)
#define CI_PORTSC_PHY_LOW_POWER BIT(23)
#define CI_PORTSC_HIGH_SPEED BIT(27)
#define CI_PORTSC_PHY_MASK (0xf0000000U | BIT(25) | CI_PORTSC_PHY_LOW_POWER)
#define CI_PORTSC_RESET_RETAIN (CI_PORTSC_PHY_MASK | \
                                (7U << 20) | (3U << 14) | BIT(12))

static void chipidea_process_in(ChipideaState *ci, uint32_t bits);

static bool chipidea_bus_active(ChipideaState *ci)
{
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    return ci->gadget_host_connected && !ci->gadget_host_suspended &&
           !ci->gadget_resuming && (ehci->usbcmd & BIT(0)) &&
           (ci->dc_mode & 3) == 2 &&
           !(ehci->portsc[0] & CI_PORTSC_PHY_LOW_POWER);
}

static void chipidea_host_activity(ChipideaState *ci)
{
    if (ci->gadget_autosuspend_ms && ci->gadget_configured &&
        chipidea_bus_active(ci)) {
        timer_mod(ci->gadget_idle_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                  ci->gadget_autosuspend_ms);
    }
}

static void chipidea_update_irq(ChipideaState *ci)
{
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    /* OTG change interrupts remain active while the device engine sleeps. */
    qemu_set_irq(ehci->irq, (ehci->usbsts & ehci->usbintr) != 0 ||
                 (ci->otgsc & (ci->otgsc >> 8) & 0x007f0000) != 0 ||
                 ci->wakeup_irq);
}

static void chipidea_wakeup_irq(void *opaque, int n, int level)
{
    ChipideaState *ci = opaque;

    ci->wakeup_irq = level;
    chipidea_update_irq(ci);
}

static void chipidea_raise_irq(ChipideaState *ci)
{
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    ehci->usbsts |= CI_USBSTS_INT;
    chipidea_update_irq(ci);
}

static void chipidea_host_suspend(void *opaque)
{
    ChipideaState *ci = opaque;
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    if (!ci->gadget_configured || !chipidea_bus_active(ci)) {
        return;
    }
    timer_del(ci->gadget_idle_timer);
    ci->gadget_host_suspended = true;
    ehci->portsc[0] |= CI_PORTSC_SUSPEND;
    ehci->usbsts |= CI_USBSTS_SUSPEND;
    chipidea_update_irq(ci);
}

static void chipidea_resume_complete(void *opaque)
{
    ChipideaState *ci = opaque;
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    if (ehci->portsc[0] & CI_PORTSC_PHY_LOW_POWER) {
        /* Keep resume visible until the platform wake handler ungates PHY. */
        return;
    }
    ci->gadget_resuming = false;
    ci->gadget_host_suspended = false;
    ehci->portsc[0] &= ~(CI_PORTSC_SUSPEND | CI_PORTSC_FORCE_RESUME);
    qemu_set_irq(ci->resume, 0);
    chipidea_raise_irq(ci);
    chipidea_host_activity(ci);
    chipidea_process_in(ci, ci->endptstatus & 0xffff0000);
    if (ci->gadget_nic) {
        qemu_flush_queued_packets(qemu_get_queue(ci->gadget_nic));
    }
}

static void chipidea_host_resume(ChipideaState *ci)
{
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    if (!ci->gadget_host_suspended || ci->gadget_resuming) {
        return;
    }
    ci->gadget_resuming = true;
    ehci->portsc[0] |= CI_PORTSC_FORCE_RESUME;
    qemu_set_irq(ci->resume, 1);
    /* The host drives resume for 20 ms before endpoint traffic restarts. */
    timer_mod(ci->gadget_resume_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 20);
}

static bool chipidea_get_host_suspended(Object *obj, Error **errp)
{
    return CHIPIDEA(obj)->gadget_host_suspended;
}

static void chipidea_set_host_suspended(Object *obj, bool value, Error **errp)
{
    ChipideaState *ci = CHIPIDEA(obj);

    if (!ci->gadget || !ci->gadget_configured) {
        error_setg(errp, "USB gadget host is not configured");
        return;
    }
    if (value) {
        chipidea_host_suspend(ci);
    } else {
        chipidea_host_resume(ci);
    }
}

static void chipidea_inject_setup(ChipideaState *ci,
                                  const uint8_t setup[8])
{
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;
    hwaddr qh_addr = ehci->asynclistaddr & 0xfffff800;

    if (!ci->gadget_host_connected || !qh_addr ||
        (ci->dc_mode & 3) != 2) {
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

    if (!ci->gadget || (ci->dc_mode & 3) != 2 ||
        !ci->gadget_host_connected ||
        ci->gadget_configured) {
        return;
    }
    if (!(SYS_BUS_EHCI(ci)->ehci.usbcmd & BIT(0))) {
        timer_mod(ci->gadget_config_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 250);
        return;
    }
    if (ci->gadget_config_phase == 0) {
        EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

        ehci->portsc[0] = CI_PORTSC_CONNECTED | CI_PORTSC_ENABLED |
                          CI_PORTSC_HIGH_SPEED;
        ehci->usbsts |= CI_USBSTS_RESET;
        chipidea_update_irq(ci);
        ci->gadget_config_phase = 1;
    } else if (ci->gadget_config_phase == 1) {
        EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

        ehci->usbsts |= CI_USBSTS_PORT_CHANGE;
        chipidea_update_irq(ci);
        ci->gadget_config_phase = 2;
    } else if (ci->gadget_config_phase == 2) {
        chipidea_inject_setup(ci, set_configuration);
        ci->gadget_config_phase = 3;
    } else if (ci->gadget_config_phase == 3) {
        chipidea_inject_setup(ci, set_interface);
        ci->gadget_config_phase = 4;
    } else {
        ci->gadget_configured = true;
        chipidea_host_activity(ci);
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

    if (!chipidea_bus_active(ci)) {
        return;
    }
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
        chipidea_host_activity(ci);
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

    if (!ci->gadget_host_connected || ci->processing_in) {
        return false;
    }
    /* Let an actual host packet reach receive() and initiate bus resume.
     * Polling this predicate alone must not wake an idle USB device. */
    if (ci->gadget_host_suspended || ci->gadget_resuming) {
        return true;
    }
    if (!chipidea_bus_active(ci)) {
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

    if (!ci->gadget_host_connected) {
        return 0;
    }
    chipidea_host_resume(ci);
    if (!chipidea_bus_active(ci)) {
        return 0; /* Keep the packet queued until resume completes. */
    }
    for (ep = 1; ep < 8; ep++) {
        size_t length = size;

        if (!(ci->endptstatus & BIT(ep)) ||
            !(ci->endptctrl[ep] & BIT(7)) ||
            !chipidea_dtd(ci, ep, false, (uint8_t *)buf, &length)) {
            continue;
        }
        chipidea_complete(ci, ep, false);
        chipidea_host_activity(ci);
        return length;
    }
    return 0;
}

static void chipidea_link_changed(NetClientState *nc)
{
    ChipideaState *ci = qemu_get_nic_opaque(nc);
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    ci->gadget_host_connected = !nc->link_down;
    ci->gadget_configured = false;
    ci->gadget_config_phase = 0;
    timer_del(ci->gadget_config_timer);
    timer_del(ci->gadget_idle_timer);
    timer_del(ci->gadget_resume_timer);
    ci->gadget_host_suspended = false;
    ci->gadget_resuming = false;
    qemu_set_irq(ci->resume, 0);
    qemu_purge_queued_packets(nc);
    qemu_set_irq(ci->vbus, ci->gadget_host_connected);
    ci->otgsc |= BIT(19); /* B-session-valid changed */
    if (nc->link_down) {
        ehci->portsc[0] = BIT(1); /* connection-status change, disconnected */
        ehci->usbsts |= CI_USBSTS_PORT_CHANGE;
    } else {
        timer_mod(ci->gadget_config_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 250);
    }
    chipidea_update_irq(ci);
}

static NetClientInfo chipidea_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = chipidea_can_receive,
    .receive = chipidea_receive,
    .link_status_changed = chipidea_link_changed,
};

static uint64_t chipidea_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    ChipideaState *ci = opaque;

    switch (offset) {
    case CI_OTGSC:
        /* The powered cable exceeds both A- and B-session thresholds. */
        return (ci->otgsc & ~(BIT(9) | BIT(10) | BIT(11) | BIT(12))) |
               BIT(8) | (ci->gadget_host_connected ?
                         BIT(9) | BIT(10) | BIT(11) : BIT(12));
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
        /* Status bits 16..22 are W1C; control and enable bits are RW. */
        ci->otgsc = (value & ~0x007f0000) |
                    (ci->otgsc & ~value & 0x007f0000);
        chipidea_update_irq(ci);
        break;
    case CI_USBMODE: {
        EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

        ci->dc_mode = value;
        if ((value & 3) == 3 && !(ehci->usbcmd & BIT(0))) {
            ehci->usbsts |= USBSTS_HALT;
        } else {
            ehci->usbsts &= ~USBSTS_HALT;
        }
        if (ci->gadget && (value & 3) != 2) {
            timer_del(ci->gadget_config_timer);
            timer_del(ci->gadget_idle_timer);
            timer_del(ci->gadget_resume_timer);
            ci->gadget_configured = false;
            ci->gadget_config_phase = 0;
            ci->gadget_host_suspended = false;
            ci->gadget_resuming = false;
            qemu_set_irq(ci->resume, 0);
        }
        break;
    }
    case CI_ENDPTSETUPSTAT:
        ci->endptsetupstat &= ~value;
        break;
    case CI_ENDPTPRIME:
        ci->endpointprime |= value;
        ci->endptstatus |= value;
        if (value & 0xffff0000) {
            chipidea_host_resume(ci);
        }
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
            chipidea_update_irq(ci);
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
            if (ci->gadget_host_connected && ci->gadget_configured &&
                index > 0) {
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
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    if ((ci->dc_mode & 3) == 3) {
        return ehci_opreg_read(ehci, offset, size);
    }
    switch (offset) {
    case 0: return ehci->usbcmd;
    case 4: return ehci->usbsts;
    case 8: return ehci->usbintr;
    default: return 0;
    }
}

static void chipidea_reset_device(ChipideaState *ci)
{
    timer_del(ci->gadget_config_timer);
    timer_del(ci->gadget_idle_timer);
    timer_del(ci->gadget_resume_timer);
    ci->gadget_configured = false;
    ci->gadget_config_phase = 0;
    ci->gadget_host_suspended = false;
    ci->gadget_resuming = false;
    qemu_set_irq(ci->resume, 0);
    ci->dc_mode = 0;
    ci->endptsetupstat = 0;
    ci->endpointprime = 0;
    ci->endptflush = 0;
    ci->endptstatus = 0;
    ci->endptcomplete = 0;
    memset(ci->endptctrl, 0, sizeof(ci->endptctrl));
    ci->endpoint_commands_ready = false;
}

static void chipidea_command_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    ChipideaState *ci = opaque;
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;
    bool was_running = ehci->usbcmd & BIT(0);

    if ((ci->dc_mode & 3) == 3) {
        uint32_t retained = ehci->portsc[0] & CI_PORTSC_RESET_RETAIN;

        /* The same OTG core probes as an EHCI host before the UDC binds. */
        ehci_opreg_write(ehci, offset, value, size);
        if (offset == 0 && (value & BIT(1))) {
            ehci->portsc[0] = (ehci->portsc[0] & ~CI_PORTSC_RESET_RETAIN) |
                              retained;
            chipidea_reset_device(ci);
            ehci->usbsts &= ~USBSTS_HALT;
        }
        chipidea_update_irq(ci);
        return;
    }
    /* Device reset/suspend status occupies bits that EHCI reserves.  Using
     * the parent's USBSTS/USBINTR handlers silently loses UDC interrupts. */
    if (offset == 4) {
        ehci->usbsts &= ~value;
        chipidea_update_irq(ci);
        return;
    }
    if (offset == 8) {
        ehci->usbintr = value;
        chipidea_update_irq(ci);
        return;
    }
    if (value & BIT(1)) {
        /* Device reset clears pending events and interrupt enables.  Keeping
         * a disconnect's port-change status until the driver enables USBINTR
         * again can interrupt its resume path before the UDC is ready. */
        ehci->usbsts = 0;
        ehci->usbintr = 0;
        chipidea_reset_device(ci);
        ehci->portsc[0] &= ~(CI_PORTSC_SUSPEND | CI_PORTSC_FORCE_RESUME);
        chipidea_update_irq(ci);
    }

    ehci->usbcmd = value & ~BIT(1);
    if (value & BIT(0)) {
        ci->endpoint_commands_ready = true;
    }
    if (!(value & BIT(0))) {
        timer_del(ci->gadget_idle_timer);
        timer_del(ci->gadget_resume_timer);
        ci->gadget_host_suspended = false;
        ci->gadget_resuming = false;
        qemu_set_irq(ci->resume, 0);
        ehci->portsc[0] &= ~(CI_PORTSC_SUSPEND | CI_PORTSC_FORCE_RESUME);
        ci->gadget_configured = false;
        ci->gadget_config_phase = 0;
        if (ci->gadget_host_connected && ci->gadget_config_timer) {
            /* OTG gadget binds do not always restart RUN until VBUS work. */
            timer_mod(ci->gadget_config_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 2000);
        }
    } else if (!was_running && ci->gadget_host_connected &&
               ci->gadget_config_timer) {
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

static uint64_t chipidea_port_read(void *opaque, hwaddr offset, unsigned size)
{
    ChipideaState *ci = opaque;
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;

    if ((ci->dc_mode & 3) == 3) {
        return ehci_port_read(ehci, offset, size);
    }
    return ehci->portsc[0];
}

static void chipidea_port_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    ChipideaState *ci = opaque;
    EHCIState *ehci = &SYS_BUS_EHCI(ci)->ehci;
    uint32_t old = ehci->portsc[0];
    /* In device mode the host controls connection, enable, suspend and
     * speed. The guest controls the PHY and can request remote resume. */
    uint32_t ro = CI_PORTSC_CONNECTED | CI_PORTSC_ENABLED |
                  CI_PORTSC_SUSPEND | (3U << 26);
    uint32_t w1c = BIT(1) | BIT(3) | BIT(5);

    if ((ci->dc_mode & 3) == 3) {
        ehci_port_write(ehci, offset, value, size);
        /* PHY interface and low-power controls are outside standard EHCI. */
        ehci->portsc[0] = (ehci->portsc[0] & ~CI_PORTSC_PHY_MASK) |
                          (value & CI_PORTSC_PHY_MASK);
        chipidea_update_irq(ci);
        return;
    }
    ehci->portsc[0] = (old & ro) | (old & w1c & ~value) |
                      (value & ~(ro | w1c));
    if (value & CI_PORTSC_FORCE_RESUME) {
        chipidea_host_resume(ci);
    }
    if ((old & CI_PORTSC_PHY_LOW_POWER) &&
        !(value & CI_PORTSC_PHY_LOW_POWER)) {
        if (ci->gadget_resuming && !timer_pending(ci->gadget_resume_timer)) {
            chipidea_resume_complete(ci);
        }
        chipidea_process_in(ci, ci->endptstatus & 0xffff0000);
        if (ci->gadget_nic) {
            qemu_flush_queued_packets(qemu_get_queue(ci->gadget_nic));
        }
    }
}

static const MemoryRegionOps chipidea_port_ops = {
    .read = chipidea_port_read,
    .write = chipidea_port_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
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

    qdev_init_gpio_out_named(DEVICE(obj), &ci->vbus, "vbus", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &ci->resume, "resume", 1);
    qdev_init_gpio_in_named(DEVICE(obj), chipidea_wakeup_irq, "wakeup", 1);
    object_property_add_bool(obj, "gadget-host-suspended",
                             chipidea_get_host_suspended,
                             chipidea_set_host_suspended);
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
        ci->gadget_idle_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                            chipidea_host_suspend, ci);
        ci->gadget_resume_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                              chipidea_resume_complete, ci);
        memory_region_init_io(&ci->dc_command_iomem, OBJECT(ci),
                              &chipidea_command_ops, ci,
                              TYPE_CHIPIDEA ".dc-command", 12);
        memory_region_add_subregion_overlap(&SYS_BUS_EHCI(ci)->ehci.mem,
                                            0x140,
                                            &ci->dc_command_iomem, 2);
        memory_region_init_io(&ci->dc_port_iomem, OBJECT(ci),
                              &chipidea_port_ops, ci,
                              TYPE_CHIPIDEA ".dc-port", 4);
        memory_region_add_subregion_overlap(&SYS_BUS_EHCI(ci)->ehci.mem,
                                            0x184, &ci->dc_port_iomem, 2);
        /* A bound UDC is not an attached USB host. */
        if (ci->gadget_host_connected) {
            timer_mod(ci->gadget_config_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 30000);
        }
    }
}

static const Property chipidea_properties[] = {
    DEFINE_PROP_BOOL("gadget", ChipideaState, gadget, false),
    DEFINE_PROP_BOOL("gadget-host-connected", ChipideaState,
                     gadget_host_connected, true),
    DEFINE_PROP_UINT32("gadget-host-autosuspend-ms", ChipideaState,
                       gadget_autosuspend_ms, 0),
    DEFINE_NIC_PROPERTIES(ChipideaState, gadget_nic_conf),
};

static void chipidea_reset_hold(Object *obj, ResetType type)
{
    ChipideaState *ci = CHIPIDEA(obj);

    if (chipidea_parent_reset.hold) {
        chipidea_parent_reset.hold(obj, type);
    }
    if (!ci->gadget) {
        return;
    }
    timer_del(ci->gadget_config_timer);
    timer_del(ci->gadget_idle_timer);
    timer_del(ci->gadget_resume_timer);
    ci->gadget_configured = false;
    ci->gadget_config_phase = 0;
    ci->gadget_host_suspended = false;
    ci->gadget_resuming = false;
    ci->processing_in = false;
    ci->endpoint_commands_ready = false;
    ci->wakeup_irq = false;
    ci->otgsc = 0;
    ci->dc_mode = 0;
    ci->endptsetupstat = 0;
    ci->endpointprime = 0;
    ci->endptflush = 0;
    ci->endptstatus = 0;
    ci->endptcomplete = 0;
    memset(ci->endptctrl, 0, sizeof(ci->endptctrl));
    qemu_set_irq(ci->resume, 0);
    chipidea_update_irq(ci);
    if (ci->gadget_host_connected) {
        timer_mod(ci->gadget_config_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 30000);
    }
}

static bool chipidea_gadget_needed(void *opaque)
{
    ChipideaState *ci = opaque;

    return ci->gadget;
}

static int chipidea_gadget_post_load(void *opaque, int version_id)
{
    ChipideaState *ci = opaque;

    ci->processing_in = false;
    qemu_set_irq(ci->resume, ci->gadget_resuming);
    chipidea_update_irq(ci);
    return 0;
}

static const VMStateDescription vmstate_chipidea_gadget = {
    .name = "usb-chipidea/gadget",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = chipidea_gadget_needed,
    .post_load = chipidea_gadget_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(wakeup_irq, ChipideaState),
        VMSTATE_UINT32(otgsc, ChipideaState),
        VMSTATE_UINT32(dc_mode, ChipideaState),
        VMSTATE_UINT32(endptsetupstat, ChipideaState),
        VMSTATE_UINT32(endpointprime, ChipideaState),
        VMSTATE_UINT32(endptflush, ChipideaState),
        VMSTATE_UINT32(endptstatus, ChipideaState),
        VMSTATE_UINT32(endptcomplete, ChipideaState),
        VMSTATE_UINT32_ARRAY(endptctrl, ChipideaState, 8),
        VMSTATE_TIMER_PTR(gadget_config_timer, ChipideaState),
        VMSTATE_TIMER_PTR(gadget_idle_timer, ChipideaState),
        VMSTATE_TIMER_PTR(gadget_resume_timer, ChipideaState),
        VMSTATE_BOOL(gadget_host_suspended, ChipideaState),
        VMSTATE_BOOL(gadget_resuming, ChipideaState),
        VMSTATE_UINT32(gadget_config_phase, ChipideaState),
        VMSTATE_BOOL(gadget_configured, ChipideaState),
        VMSTATE_BOOL(endpoint_commands_ready, ChipideaState),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_chipidea = {
    .name = "usb-chipidea",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(parent_obj.ehci, ChipideaState, 0,
                       vmstate_ehci, EHCIState),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_chipidea_gadget,
        NULL
    },
};

static void chipidea_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
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
    dc->vmsd = &vmstate_chipidea;
    device_class_set_parent_realize(dc, chipidea_realize,
                                    &chipidea_parent_realize);
    resettable_class_set_parent_phases(rc, NULL, chipidea_reset_hold, NULL,
                                       &chipidea_parent_reset);
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
