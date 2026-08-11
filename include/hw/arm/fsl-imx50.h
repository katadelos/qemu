/* Freescale i.MX50 / i.MX508 SoC */
#ifndef HW_ARM_FSL_IMX50_H
#define HW_ARM_FSL_IMX50_H

#include "hw/char/imx_serial.h"
#include "hw/display/imx50_epdc.h"
#include "hw/gpio/imx_gpio.h"
#include "hw/i2c/imx_i2c.h"
#include "hw/intc/imx_tzic.h"
#include "hw/misc/imx50_ccm.h"
#include "hw/sd/sdhci.h"
#include "hw/ssi/imx_spi.h"
#include "hw/timer/imx_epit.h"
#include "hw/timer/imx_gpt.h"
#include "hw/usb/chipidea.h"
#include "hw/watchdog/wdt_imx2.h"
#include "cpu.h"

#define TYPE_FSL_IMX50 "fsl-imx50"
OBJECT_DECLARE_SIMPLE_TYPE(FslIMX50State, FSL_IMX50)

#define FSL_IMX50_NUM_UARTS 4
#define FSL_IMX50_NUM_I2CS 3
#define FSL_IMX50_NUM_GPIOS 7
#define FSL_IMX50_NUM_ESDHC 4
#define FSL_IMX50_NUM_SPI 3

struct FslIMX50State {
    DeviceState parent_obj;
    ARMCPU cpu;
    IMXTZICState tzic;
    IMX50CCMState ccm;
    IMXSerialState uart[FSL_IMX50_NUM_UARTS];
    IMXGPTState gpt;
    IMXEPITState epit[2];
    IMXI2CState i2c[FSL_IMX50_NUM_I2CS];
    IMXGPIOState gpio[FSL_IMX50_NUM_GPIOS];
    SDHCIState esdhc[FSL_IMX50_NUM_ESDHC];
    IMXSPIState spi[FSL_IMX50_NUM_SPI];
    ChipideaState usb_otg;
    ChipideaState usb_h1;
    IMX2WdtState wdt;
    IMX50EPDCState epdc;
    MemoryRegion iram;
    MemoryRegion src_iomem;
    MemoryRegion databahn_iomem;
    MemoryRegion sdma_iomem;
    MemoryRegion srtc_iomem;
    MemoryRegion anatop_iomem;
    MemoryRegion pxp_iomem;
    qemu_irq sdma_irq;
    qemu_irq pxp_irq;
    QEMUTimer *pxp_completion_timer;
    uint32_t databahn[0x400 / 4];
    uint32_t sdma[0x1000 / 4];
    uint32_t srtc[0x40 / 4];
    uint32_t anatop[0x100 / 4];
    uint32_t pxp[0x1000 / 4];
    uint8_t pxp_lut[256];
    uint32_t ddr_type;
    uint32_t src_sbmr;
};

#define FSL_IMX50_TZIC_ADDR   0x0fffc000
#define FSL_IMX50_IRAM_ADDR   0xf8000000
#define FSL_IMX50_IRAM_SIZE   0x00020000

#endif
