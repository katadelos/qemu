/* NXP i.MX8M Mini. SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_FSL_IMX8MM_H
#define HW_ARM_FSL_IMX8MM_H

#include "hw/arm/bsa.h"
#include "hw/char/imx_serial.h"
#include "hw/core/sysbus.h"
#include "hw/display/imx_lcdif.h"
#include "hw/dma/imx_sdma.h"
#include "hw/gpio/imx_gpio.h"
#include "hw/i2c/imx_i2c.h"
#include "hw/intc/arm_gicv3_common.h"
#include "hw/misc/imx7_snvs.h"
#include "hw/misc/imx_caam.h"
#include "hw/misc/imx8mp_analog.h"
#include "hw/misc/imx8mp_ccm.h"
#include "hw/sd/sdhci.h"
#include "hw/ssi/imx_spi.h"
#include "hw/watchdog/wdt_imx2.h"
#include "hw/usb/chipidea.h"
#include "target/arm/cpu.h"

#define TYPE_FSL_IMX8MM "fsl-imx8mm"
OBJECT_DECLARE_SIMPLE_TYPE(FslIMX8MMState, FSL_IMX8MM)
#define IMX8MM_NUM_CPUS 4
#define IMX8MM_NUM_IRQS 160
#define IMX8MM_CONTROL_BANKS 17

typedef struct IMX8MMControlRegion {
    MemoryRegion memory;
    FslIMX8MMState *soc;
    hwaddr base;
    unsigned index;
} IMX8MMControlRegion;

struct FslIMX8MMState {
    SysBusDevice parent_obj;
    ARMCPU cpu[IMX8MM_NUM_CPUS];
    GICv3State gic;
    IMXSerialState uart[4];
    IMXI2CState i2c[4];
    IMXGPIOState gpio[5];
    SDHCIState usdhc[3];
    IMXSPIState spi[3];
    IMX2WdtState wdt[3];
    IMX8MPCCMState ccm;
    IMX8MPAnalogState analog;
    IMX7SNVSState snvs;
    IMXLCDIFState lcdif;
    IMXCAAMState caam;
    IMXSDMAState sdma[3];
    ChipideaState usb[2];
    QEMUTimer power_timer;
    uint16_t cpu_irq_levels;
    uint8_t wake_levels;
    bool powerdown[4], sleeping[4];
    bool ddr_initialized;
    MemoryRegion rom, tcm, ocram_s, ocram;
    IMX8MMControlRegion control[IMX8MM_CONTROL_BANKS], phy_region;
    uint32_t registers[IMX8MM_CONTROL_BANKS][0x10000 / 4];
    uint32_t phy[0x1000000 / 4];
};
#endif
