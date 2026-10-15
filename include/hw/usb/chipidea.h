#ifndef CHIPIDEA_H
#define CHIPIDEA_H

#include "hw/usb/hcd-ehci.h"
#include "net/net.h"
#include "qemu/timer.h"
#include "qom/object.h"

struct ChipideaState {
    /*< private >*/
    EHCISysBusState parent_obj;

    MemoryRegion iomem[3];
    MemoryRegion dc_command_iomem;
    MemoryRegion dc_port_iomem;
    NICConf gadget_nic_conf;
    NICState *gadget_nic;
    qemu_irq vbus;
    qemu_irq resume;
    bool wakeup_irq;
    bool gadget;
    bool gadget_host_connected;
    uint32_t otgsc;
    uint32_t dc_mode;
    uint32_t endptsetupstat;
    uint32_t endpointprime;
    uint32_t endptflush;
    uint32_t endptstatus;
    uint32_t endptcomplete;
    uint32_t endptctrl[8];
    QEMUTimer *gadget_config_timer;
    QEMUTimer *gadget_idle_timer;
    QEMUTimer *gadget_resume_timer;
    uint32_t gadget_autosuspend_ms;
    bool gadget_host_suspended;
    bool gadget_resuming;
    unsigned gadget_config_phase;
    bool gadget_configured;
    bool processing_in;
    bool endpoint_commands_ready;
};

#define TYPE_CHIPIDEA "usb-chipidea"
OBJECT_DECLARE_SIMPLE_TYPE(ChipideaState, CHIPIDEA)

#endif /* CHIPIDEA_H */
