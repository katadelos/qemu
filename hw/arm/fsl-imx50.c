/* Freescale i.MX50 / i.MX508 SoC emulation */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "chardev/char.h"
#include "hw/arm/fsl-imx50.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/unimp.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "system/dma.h"
#include "system/system.h"
#include "target/arm/cpu-qom.h"
#include "trace.h"

#define IMX50_ESDHC_CAPABILITIES 0x07e20000
#define IMX50_SDMA_C0PTR         0x00
#define IMX50_SDMA_INTR          0x04
#define IMX50_SDMA_START         0x0c
#define IMX50_SDMA_RESET         0x24
#define IMX50_SRTC_LPCR          0x10
#define IMX50_SRTC_LPSR          0x14
#define IMX50_SRTC_LPCR_NVE      (1 << 14)
#define IMX50_SRTC_LPCR_IE       (1 << 15)
#define IMX50_SRTC_LPSR_NVES     (1 << 14)
#define IMX50_SRTC_LPSR_IES      (1 << 15)
#define IMX50_ANATOP_FRAC0        0x10
#define IMX50_ANATOP_FRAC1        0x20
#define IMX50_ANATOP_PLLCTRL      0x70
#define IMX50_DATABAHN_CTL19       0x4c
#define IMX50_DATABAHN_CTL20       0x50
#define IMX50_DATABAHN_CTL42       0xa8
#define IMX50_DATABAHN_CTL63       0xfc
#define IMX50_DATABAHN_CTL79       0x13c
#define IMX50_DATABAHN_PHY25       0x264
#define IMX50_DATABAHN_DDR_TYPE_MASK 0xf00
#define IMX50_DATABAHN_SELF_REFRESH (1U << 0)
#define IMX50_DATABAHN_LPM_MODE    0x1fU
#define IMX50_DATABAHN_DLL_LOCKED  (1U << 8)
#define IMX50_DATABAHN_CKE         (1U << 16)
#define IMX50_DATABAHN_BUSY        (1U << 8)
#define IMX50_DATABAHN_PHY_READY   (1U << 1)
#define IMX50_PXP_CTRL            0x000
#define IMX50_PXP_STAT            0x010
#define IMX50_PXP_OUTBUF          0x020
#define IMX50_PXP_OUTSIZE         0x040
#define IMX50_PXP_S0BUF           0x050
#define IMX50_PXP_S0PARAM         0x080
#define IMX50_PXP_S0CROP          0x0a0
#define IMX50_PXP_LUT_CTRL        0x470
#define IMX50_PXP_LUT             0x480
#define IMX50_PXP_CTRL_IRQ_ENABLE (1U << 1)
#define IMX50_PXP_CTRL_ENABLE     (1U << 0)
#define IMX50_PXP_STAT_IRQ        (1U << 0)
#define IMX50_PXP_LUT_BYPASS      (1U << 31)
#define IMX50_PXP_MAX_DIMENSION   2048
#define IMX50_PXP_ARGB8888        0
#define IMX50_PXP_RGB888          1
#define IMX50_PXP_RGB565          4
#define IMX50_PXP_YUV420          9
#define IMX50_PXP_MONOC8          8

static void imx50_sdma_complete(FslIMX50State *s, unsigned channel)
{
    uint32_t ccb_addr = s->sdma[IMX50_SDMA_C0PTR / 4] + channel * 16;
    uint32_t bd_addr;
    uint32_t mode;

    /*
     * The legacy Freescale I.API waits synchronously for every command.
     * Completing the descriptor is sufficient until a peripheral needs
     * actual SDMA data movement.
     */
    if (dma_memory_read(&address_space_memory, ccb_addr + 4, &bd_addr,
                        sizeof(bd_addr), MEMTXATTRS_UNSPECIFIED) ==
        MEMTX_OK) {
        bd_addr = le32_to_cpu(bd_addr);
        if (bd_addr &&
            dma_memory_read(&address_space_memory, bd_addr, &mode,
                            sizeof(mode), MEMTXATTRS_UNSPECIFIED) ==
            MEMTX_OK) {
            mode = le32_to_cpu(mode) & ~(1 << 16); /* BD_DONE */
            mode = cpu_to_le32(mode);
            dma_memory_write(&address_space_memory, bd_addr, &mode,
                             sizeof(mode), MEMTXATTRS_UNSPECIFIED);
        }
    }

    s->sdma[IMX50_SDMA_INTR / 4] |= 1U << channel;
    qemu_set_irq(s->sdma_irq, 1);
}

static uint64_t imx50_sdma_read(void *opaque, hwaddr offset, unsigned size)
{
    FslIMX50State *s = opaque;

    return s->sdma[offset / 4];
}

static void imx50_sdma_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    FslIMX50State *s = opaque;
    uint32_t channels;

    switch (offset) {
    case IMX50_SDMA_INTR:
        /* H_INTR is write-one-to-clear. */
        s->sdma[offset / 4] &= ~(uint32_t)value;
        qemu_set_irq(s->sdma_irq, s->sdma[offset / 4] != 0);
        break;
    case IMX50_SDMA_START:
        channels = value;
        while (channels) {
            unsigned channel = ctz32(channels);

            channels &= channels - 1;
            /*
             * Channel 0 carries the host-to-SDMA control protocol used by
             * the legacy Freescale I.API during setup.  We can acknowledge
             * those commands without executing SDMA scripts.  Claiming
             * completion for a peripheral channel is unsafe, however: its
             * callback assumes that the requested transfer really happened.
             */
            if (channel == 0) {
                imx50_sdma_complete(s, channel);
            }
        }
        break;
    case IMX50_SDMA_RESET:
        memset(s->sdma, 0, sizeof(s->sdma));
        qemu_set_irq(s->sdma_irq, 0);
        break;
    default:
        s->sdma[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps imx50_sdma_ops = {
    .read = imx50_sdma_read,
    .write = imx50_sdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void imx50_pxp_update_irq(FslIMX50State *s)
{
    qemu_set_irq(s->pxp_irq,
                 (s->pxp[IMX50_PXP_CTRL / 4] &
                  IMX50_PXP_CTRL_IRQ_ENABLE) &&
                 (s->pxp[IMX50_PXP_STAT / 4] & IMX50_PXP_STAT_IRQ));
}

static void imx50_pxp_process(FslIMX50State *s)
{
    uint32_t ctrl = s->pxp[IMX50_PXP_CTRL / 4];
    uint32_t outsize = s->pxp[IMX50_PXP_OUTSIZE / 4];
    uint32_t s0param = s->pxp[IMX50_PXP_S0PARAM / 4];
    uint32_t crop = s->pxp[IMX50_PXP_S0CROP / 4];
    uint32_t source = s->pxp[IMX50_PXP_S0BUF / 4];
    uint32_t output = s->pxp[IMX50_PXP_OUTBUF / 4];
    unsigned out_width = extract32(outsize, 12, 12);
    unsigned out_height = extract32(outsize, 0, 12);
    unsigned src_width = extract32(s0param, 8, 8) * 8;
    unsigned src_height = extract32(s0param, 0, 8) * 8;
    unsigned crop_x = extract32(crop, 24, 8) * 8;
    unsigned crop_y = extract32(crop, 16, 8) * 8;
    unsigned crop_width = extract32(crop, 8, 8) * 8;
    unsigned crop_height = extract32(crop, 0, 8) * 8;
    unsigned rotation = extract32(ctrl, 8, 2);
    unsigned input_format = extract32(ctrl, 12, 4);
    unsigned output_format = extract32(ctrl, 4, 4);
    unsigned input_bpp;
    unsigned result_width;
    unsigned result_height;
    g_autofree uint8_t *input = NULL;
    g_autofree uint8_t *result = NULL;
    int64_t started_ns;
    unsigned x, y;

    /*
     * The EPDC driver uses MONOC8 output.  The i.MX50 PxP accepts ARGB8888,
     * RGB888, RGB565, or the Y plane of a YUV420 buffer on S0; all four use
     * the same crop/rotation machinery.
     */
    if (!source || !output || !out_width || !out_height ||
        !src_width || !src_height ||
        src_width > IMX50_PXP_MAX_DIMENSION ||
        src_height > IMX50_PXP_MAX_DIMENSION ||
        out_width > IMX50_PXP_MAX_DIMENSION ||
        out_height > IMX50_PXP_MAX_DIMENSION ||
        (input_format != IMX50_PXP_ARGB8888 &&
         input_format != IMX50_PXP_RGB888 &&
         input_format != IMX50_PXP_RGB565 &&
         input_format != IMX50_PXP_YUV420) ||
        output_format != IMX50_PXP_MONOC8) {
        return;
    }
    if (input_format == IMX50_PXP_ARGB8888 ||
        input_format == IMX50_PXP_RGB888) {
        input_bpp = 4;
    } else if (input_format == IMX50_PXP_RGB565) {
        input_bpp = 2;
    } else {
        input_bpp = 1;
    }
    if (!crop_width) {
        crop_width = src_width;
    }
    if (!crop_height) {
        crop_height = src_height;
    }
    if (crop_x + crop_width > src_width ||
        crop_y + crop_height > src_height) {
        return;
    }

    /*
     * OUTSIZE describes the source/destination rectangle before rotation.
     * For quarter turns PxP writes a transposed raster to memory.  This is
     * important for EPDC: it consumes that buffer using the post-rotation
     * width as its implicit line stride.
     */
    if (rotation & 1) {
        result_width = out_height;
        result_height = out_width;
    } else {
        result_width = out_width;
        result_height = out_height;
    }

    trace_whitney_pxp_begin(src_width, src_height,
                            result_width, result_height);
    trace_whitney_pxp_config(ctrl, source, output, outsize, s0param, crop);
    started_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    input = g_malloc((size_t)src_width * src_height * input_bpp);
    result = g_malloc0((size_t)result_width * result_height);
    if (dma_memory_read(&address_space_memory, source, input,
                        (size_t)src_width * src_height * input_bpp,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return;
    }

    for (y = 0; y < result_height; y++) {
        for (x = 0; x < result_width; x++) {
            unsigned sx, sy;

            switch (rotation) {
            case 1: /* 90 degrees clockwise */
                sx = crop_x + y;
                sy = crop_y + crop_height - 1 - x;
                break;
            case 2: /* 180 degrees */
                sx = crop_x + crop_width - 1 - x;
                sy = crop_y + crop_height - 1 - y;
                break;
            case 3: /* 270 degrees clockwise */
                sx = crop_x + crop_width - 1 - y;
                sy = crop_y + x;
                break;
            default:
                sx = crop_x + x;
                sy = crop_y + y;
                break;
            }
            if (sx < src_width && sy < src_height) {
                uint8_t gray;

                if (input_format == IMX50_PXP_ARGB8888 ||
                    input_format == IMX50_PXP_RGB888) {
                    const uint8_t *pixel = input +
                        ((size_t)sy * src_width + sx) * input_bpp;

                    gray = (77 * pixel[2] + 150 * pixel[1] +
                            29 * pixel[0]) >> 8;
                } else if (input_format == IMX50_PXP_RGB565) {
                    const uint8_t *pixel = input +
                        ((size_t)sy * src_width + sx) * input_bpp;
                    uint16_t rgb = pixel[0] | ((uint16_t)pixel[1] << 8);
                    unsigned red = extract32(rgb, 11, 5) * 255 / 31;
                    unsigned green = extract32(rgb, 5, 6) * 255 / 63;
                    unsigned blue = extract32(rgb, 0, 5) * 255 / 31;

                    /* Lab126's i.MX50 driver programs CSC2 as 77/150/29. */
                    gray = (77 * red + 150 * green + 29 * blue) >> 8;
                } else {
                    gray = input[(size_t)sy * src_width + sx];
                }

                if (!(s->pxp[IMX50_PXP_LUT_CTRL / 4] &
                      IMX50_PXP_LUT_BYPASS)) {
                    gray = s->pxp_lut[gray];
                }
                result[(size_t)y * result_width + x] = gray;
            }
        }
    }
    dma_memory_write(&address_space_memory, output, result,
                     (size_t)result_width * result_height,
                     MEMTXATTRS_UNSPECIFIED);
    trace_whitney_pxp_process(src_width, src_height,
                              result_width, result_height,
                              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) -
                              started_ns);
}

static uint64_t imx50_pxp_read(void *opaque, hwaddr offset, unsigned size)
{
    FslIMX50State *s = opaque;

    if (offset == IMX50_PXP_LUT) {
        unsigned addr = s->pxp[IMX50_PXP_LUT_CTRL / 4] & 0xff;
        uint8_t value = s->pxp_lut[addr];

        s->pxp[IMX50_PXP_LUT_CTRL / 4] =
            (s->pxp[IMX50_PXP_LUT_CTRL / 4] & ~0xffU) |
            ((addr + 1) & 0xff);
        return value;
    }
    return s->pxp[offset / 4];
}

static void imx50_pxp_complete(void *opaque)
{
    FslIMX50State *s = opaque;
    uint32_t *ctrl = &s->pxp[IMX50_PXP_CTRL / 4];

    if (!(*ctrl & IMX50_PXP_CTRL_ENABLE)) {
        return;
    }
    imx50_pxp_process(s);
    *ctrl &= ~IMX50_PXP_CTRL_ENABLE;
    s->pxp[IMX50_PXP_STAT / 4] |= IMX50_PXP_STAT_IRQ;
    imx50_pxp_update_irq(s);
}

static void imx50_pxp_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    FslIMX50State *s = opaque;
    hwaddr base = offset & ~0xf;
    uint32_t *reg = &s->pxp[base / 4];
    unsigned alias = (offset >> 2) & 3;

    if (offset == IMX50_PXP_LUT) {
        unsigned addr = s->pxp[IMX50_PXP_LUT_CTRL / 4] & 0xff;

        s->pxp_lut[addr] = value;
        s->pxp[IMX50_PXP_LUT_CTRL / 4] =
            (s->pxp[IMX50_PXP_LUT_CTRL / 4] & ~0xffU) |
            ((addr + 1) & 0xff);
        return;
    }

    switch (alias) {
    case 0:
        *reg = value;
        break;
    case 1:
        *reg |= value;
        break;
    case 2:
        *reg &= ~(uint32_t)value;
        break;
    case 3:
        *reg ^= value;
        break;
    }

    if (base == IMX50_PXP_CTRL && (*reg & IMX50_PXP_CTRL_ENABLE)) {
        /*
         * The stock display path uses PxP to crop/rotate its grayscale
         * framebuffer into a transient EPDC update buffer.  Completion must
         * be asynchronous: the DMA driver finishes publishing its active
         * descriptor after writing CTRL.ENABLE.
         */
        timer_mod(s->pxp_completion_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
    }

    if (base == IMX50_PXP_CTRL || base == IMX50_PXP_STAT) {
        imx50_pxp_update_irq(s);
    }
}

static const MemoryRegionOps imx50_pxp_ops = {
    .read = imx50_pxp_read,
    .write = imx50_pxp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t imx50_srtc_read(void *opaque, hwaddr offset, unsigned size)
{
    FslIMX50State *s = opaque;

    if (offset == 0x04) {
        return ++s->srtc[offset / 4];
    }
    return s->srtc[offset / 4];
}

static void imx50_srtc_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    FslIMX50State *s = opaque;

    if (offset == IMX50_SRTC_LPSR) {
        /* Status is W1C; the state-machine indications are read-only. */
        s->srtc[offset / 4] &= ~((uint32_t)value &
            ~(IMX50_SRTC_LPSR_IES | IMX50_SRTC_LPSR_NVES));
        return;
    }

    s->srtc[offset / 4] = value;
    if (offset == IMX50_SRTC_LPCR) {
        if (value & IMX50_SRTC_LPCR_IE) {
            s->srtc[IMX50_SRTC_LPSR / 4] |= IMX50_SRTC_LPSR_IES;
        }
        if (value & IMX50_SRTC_LPCR_NVE) {
            s->srtc[IMX50_SRTC_LPSR / 4] |= IMX50_SRTC_LPSR_NVES;
        }
    }
}

static const MemoryRegionOps imx50_srtc_ops = {
    .read = imx50_srtc_read,
    .write = imx50_srtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t imx50_anatop_read(void *opaque, hwaddr offset, unsigned size)
{
    FslIMX50State *s = opaque;

    return s->anatop[offset / 4];
}

static void imx50_anatop_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    FslIMX50State *s = opaque;
    hwaddr base = offset & ~0xf;

    switch (offset & 0xf) {
    case 0x4:
        s->anatop[base / 4] |= value;
        break;
    case 0x8:
        s->anatop[base / 4] &= ~(uint32_t)value;
        break;
    default:
        s->anatop[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps imx50_anatop_ops = {
    .read = imx50_anatop_read,
    .write = imx50_anatop_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t imx50_databahn_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    FslIMX50State *s = opaque;
    uint32_t value = s->databahn[offset / 4];

    switch (offset) {
    case IMX50_DATABAHN_CTL42:
        /* DRAM initialization and DLL relock complete immediately. */
        return value | 0x10 | IMX50_DATABAHN_DLL_LOCKED;
    case IMX50_DATABAHN_CTL63:
        /*
         * CKE follows the self-refresh request.  The IRAM frequency-change
         * routine polls both transitions with normal memory inaccessible.
         */
        if ((s->databahn[IMX50_DATABAHN_CTL19 / 4] &
             IMX50_DATABAHN_SELF_REFRESH) ||
            (s->databahn[IMX50_DATABAHN_CTL20 / 4] &
             IMX50_DATABAHN_LPM_MODE)) {
            return value & ~IMX50_DATABAHN_CKE;
        }
        return value | IMX50_DATABAHN_CKE;
    case IMX50_DATABAHN_CTL79:
        /* No outstanding DRAM transaction. */
        return value & ~IMX50_DATABAHN_BUSY;
    case IMX50_DATABAHN_PHY25:
        /* DDR PHY initialization/relock has completed. */
        return value | IMX50_DATABAHN_PHY_READY;
    default:
        return value;
    }
}

static void imx50_databahn_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    FslIMX50State *s = opaque;

    s->databahn[offset / 4] = value;
}

static const MemoryRegionOps imx50_databahn_ops = {
    .read = imx50_databahn_read,
    .write = imx50_databahn_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void fsl_imx50_init(Object *obj)
{
    FslIMX50State *s = FSL_IMX50(obj);
    unsigned i;
    char name[12];

    /* PxP's power-on LUT is a pass-through table, not an all-black map. */
    for (i = 0; i < ARRAY_SIZE(s->pxp_lut); i++) {
        s->pxp_lut[i] = i;
    }

    object_initialize_child(obj, "cpu", &s->cpu,
                            ARM_CPU_TYPE_NAME("cortex-a8"));
    object_initialize_child(obj, "tzic", &s->tzic, TYPE_IMX_TZIC);
    object_initialize_child(obj, "ccm", &s->ccm, TYPE_IMX50_CCM);
    for (i = 0; i < FSL_IMX50_NUM_UARTS; i++) {
        snprintf(name, sizeof(name), "uart%u", i + 1);
        object_initialize_child(obj, name, &s->uart[i], TYPE_IMX_SERIAL);
    }
    object_initialize_child(obj, "gpt", &s->gpt, TYPE_IMX50_GPT);
    for (i = 0; i < 2; i++) {
        snprintf(name, sizeof(name), "epit%u", i + 1);
        object_initialize_child(obj, name, &s->epit[i], TYPE_IMX_EPIT);
    }
    for (i = 0; i < FSL_IMX50_NUM_I2CS; i++) {
        snprintf(name, sizeof(name), "i2c%u", i + 1);
        object_initialize_child(obj, name, &s->i2c[i], TYPE_IMX_I2C);
    }
    for (i = 0; i < FSL_IMX50_NUM_GPIOS; i++) {
        snprintf(name, sizeof(name), "gpio%u", i + 1);
        object_initialize_child(obj, name, &s->gpio[i], TYPE_IMX_GPIO);
    }
    for (i = 0; i < FSL_IMX50_NUM_ESDHC; i++) {
        snprintf(name, sizeof(name), "esdhc%u", i + 1);
        object_initialize_child(obj, name, &s->esdhc[i], TYPE_IMX_USDHC);
    }
    for (i = 0; i < FSL_IMX50_NUM_SPI; i++) {
        snprintf(name, sizeof(name), "spi%u", i + 1);
        object_initialize_child(obj, name, &s->spi[i], TYPE_IMX_SPI);
    }
    object_initialize_child(obj, "usb-otg", &s->usb_otg, TYPE_CHIPIDEA);
    object_initialize_child(obj, "usb-h1", &s->usb_h1, TYPE_CHIPIDEA);
    object_initialize_child(obj, "wdt", &s->wdt, TYPE_IMX2_WDT);
    object_initialize_child(obj, "epdc", &s->epdc, TYPE_IMX50_EPDC);
}

static void fsl_imx50_map_unimplemented(void)
{
    create_unimplemented_device("imx50.iomuxc", 0x53fa8000, 0x4000);
    create_unimplemented_device("imx50.src", 0x53fd0000, 0x4000);
    create_unimplemented_device("imx50.ocotp", 0x40102000, 0x2000);
}

static void fsl_imx50_realize(DeviceState *dev, Error **errp)
{
    FslIMX50State *s = FSL_IMX50(dev);
    static const hwaddr uart_addr[] = {
        0x53fbc000, 0x53fc0000, 0x5000c000, 0x53ff0000
    };
    static const unsigned uart_irq[] = { 31, 32, 33, 13 };
    static const hwaddr i2c_addr[] = { 0x63fc8000, 0x63fc4000, 0x53fec000 };
    static const unsigned i2c_irq[] = { 62, 63, 64 };
    static const hwaddr gpio_addr[] = {
        0x53f84000, 0x53f88000, 0x53f8c000, 0x53f90000,
        0x53fdc000, 0x53fe0000, 0x53fe4000
    };
    static const unsigned gpio_irq[] = { 50, 52, 54, 56, 103, 105, 107 };
    static const hwaddr esdhc_addr[] = {
        0x50004000, 0x50008000, 0x50020000, 0x50024000
    };
    static const unsigned esdhc_irq[] = { 1, 2, 3, 4 };
    static const hwaddr spi_addr[] = { 0x50010000, 0x63fac000, 0x63fc0000 };
    static const unsigned spi_irq[] = { 36, 37, 38 };
    unsigned i;

    s->databahn[0] = (s->databahn[0] & ~IMX50_DATABAHN_DDR_TYPE_MASK) |
                     (s->ddr_type & IMX50_DATABAHN_DDR_TYPE_MASK);

    if (!qdev_realize(DEVICE(&s->cpu), NULL, errp)) {
        return;
    }
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->tzic), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->tzic), 0, FSL_IMX50_TZIC_ADDR);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->tzic), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_IRQ));
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ccm), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ccm), 0, 0x53fd4000);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ccm), 1, 0x63f80000);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ccm), 2, 0x63f84000);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ccm), 3, 0x63f88000);
    for (i = 0; i < FSL_IMX50_NUM_UARTS; i++) {
        qdev_prop_set_chr(DEVICE(&s->uart[i]), "chardev", serial_hd(i));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->uart[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->uart[i]), 0, uart_addr[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->tzic), uart_irq[i]));
    }
    s->gpt.ccm = IMX_CCM(&s->ccm);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpt), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpt), 0, 0x53fa0000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpt), 0,
                       qdev_get_gpio_in(DEVICE(&s->tzic), 39));
    for (i = 0; i < 2; i++) {
        s->epit[i].ccm = IMX_CCM(&s->ccm);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->epit[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->epit[i]), 0,
                        0x53fac000 + i * 0x4000);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->epit[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->tzic), 40 + i));
    }
    for (i = 0; i < FSL_IMX50_NUM_I2CS; i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->i2c[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 0, i2c_addr[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->tzic), i2c_irq[i]));
    }
    for (i = 0; i < FSL_IMX50_NUM_GPIOS; i++) {
        object_property_set_bool(OBJECT(&s->gpio[i]), "has-upper-pin-irq",
                                 true, &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio[i]), 0, gpio_addr[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->tzic), gpio_irq[i]));
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio[i]), 1,
                           qdev_get_gpio_in(DEVICE(&s->tzic),
                                            gpio_irq[i] + 1));
    }
    for (i = 0; i < FSL_IMX50_NUM_ESDHC; i++) {
        object_property_set_uint(OBJECT(&s->esdhc[i]), "capareg",
                                 IMX50_ESDHC_CAPABILITIES, &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->esdhc[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->esdhc[i]), 0, esdhc_addr[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->esdhc[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->tzic), esdhc_irq[i]));
    }
    for (i = 0; i < FSL_IMX50_NUM_SPI; i++) {
        if (i == 2) {
            qdev_prop_set_bit(DEVICE(&s->spi[i]), "legacy-cspi", true);
            /* The legacy CSPI block completes after observable bus latency. */
            qdev_prop_set_uint32(DEVICE(&s->spi[i]),
                                 "transfer-completion-reads", 128);
        }
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi[i]), 0, spi_addr[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->tzic), spi_irq[i]));
    }
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->wdt), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->wdt), 0, 0x53f98000);
    memory_region_init_ram(&s->iram, OBJECT(dev), "imx50.iram",
                           FSL_IMX50_IRAM_SIZE, errp);
    memory_region_add_subregion(get_system_memory(), FSL_IMX50_IRAM_ADDR,
                                &s->iram);
    memory_region_init_io(&s->databahn_iomem, OBJECT(dev),
                          &imx50_databahn_ops, s, "imx50.databahn", 0x400);
    memory_region_add_subregion(get_system_memory(), 0x14000000,
                                &s->databahn_iomem);
    s->sdma_irq = qdev_get_gpio_in(DEVICE(&s->tzic), 6);
    memory_region_init_io(&s->sdma_iomem, OBJECT(dev), &imx50_sdma_ops, s,
                          "imx50.sdma", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x63fb0000,
                                &s->sdma_iomem);
    memory_region_init_io(&s->srtc_iomem, OBJECT(dev), &imx50_srtc_ops, s,
                          "imx50.srtc", 0x40);
    memory_region_add_subregion(get_system_memory(), 0x53fa4000,
                                &s->srtc_iomem);
    s->anatop[IMX50_ANATOP_FRAC0 / 4] = 0x58585858;
    s->anatop[IMX50_ANATOP_FRAC1 / 4] = 0x58585858;
    s->anatop[IMX50_ANATOP_PLLCTRL / 4] = 1U << 31;
    memory_region_init_io(&s->anatop_iomem, OBJECT(dev),
                          &imx50_anatop_ops, s, "imx50.anatop", 0x100);
    memory_region_add_subregion(get_system_memory(), 0x41018000,
                                &s->anatop_iomem);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->usb_otg), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->usb_otg), 0, 0x53f80000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->usb_otg), 0,
                       qdev_get_gpio_in(DEVICE(&s->tzic), 18));
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->usb_h1), errp)) {
        return;
    }
    /*
     * Lab126 registers Host 1 at OTG_BASE + 0x200 with its own IRQ.  The
     * ChipIdea sysbus region is larger than the i.MX platform resource, so
     * give this instance priority over the overlapping tail of USB OTG.
     */
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->usb_h1), 0,
                            0x53f80200, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->usb_h1), 0,
                       qdev_get_gpio_in(DEVICE(&s->tzic), 14));
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->epdc), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->epdc), 0, 0x41010000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->epdc), 0,
                       qdev_get_gpio_in(DEVICE(&s->tzic), 27));
    s->pxp_irq = qdev_get_gpio_in(DEVICE(&s->tzic), 21);
    s->pxp_completion_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, imx50_pxp_complete, s);
    memory_region_init_io(&s->pxp_iomem, OBJECT(dev), &imx50_pxp_ops, s,
                          "imx50.pxp", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x4100c000,
                                &s->pxp_iomem);
    fsl_imx50_map_unimplemented();
}

static const Property fsl_imx50_properties[] = {
    DEFINE_PROP_UINT32("ddr-type", FslIMX50State, ddr_type, 0),
};

static void fsl_imx50_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = fsl_imx50_realize;
    dc->desc = "Freescale i.MX50 / i.MX508 SoC";
    dc->user_creatable = false;
    device_class_set_props(dc, fsl_imx50_properties);
}
static const TypeInfo fsl_imx50_info = {
    .name = TYPE_FSL_IMX50, .parent = TYPE_DEVICE,
    .instance_size = sizeof(FslIMX50State), .instance_init = fsl_imx50_init,
    .class_init = fsl_imx50_class_init,
};
static void fsl_imx50_register_types(void)
{
    type_register_static(&fsl_imx50_info);
}

type_init(fsl_imx50_register_types)
