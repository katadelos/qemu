/*
 * MT8115 BL2 handoffs reconstructed from the original 5.19.6 firmware.
 * Firmware containers and signatures remain unchanged on disk.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/mt8115-preload.h"
#include "hw/core/loader.h"
#include "qemu/bswap.h"
#include "qemu/bitops.h"
#include "qemu/error-report.h"
#include "qemu/host-utils.h"

static G_NORETURN void preload_invalid(const char *image)
{
    error_report("MT8115 invalid %s preload layout", image);
    exit(EXIT_FAILURE);
}

static bool span(size_t size, uint64_t offset, uint64_t length)
{
    return offset <= size && length <= size - offset;
}

/* BL2 0x264600: only dpmpm/dpmdm are copied. dpmpt has no normal-boot
 * relocation here; BL2 0x211a50 exports it through the crash-dump path.
 */
uint32_t mt8115_preload_dpm(const uint8_t *data, size_t size)
{
    static const char *names[] = { "dpmpm", "dpmdm" };
    static const unsigned capacities[] = { 0xc000, 0x4000 };
    size_t offset = 0;
    uint32_t pm_size = 0;

    for (unsigned section = 0; section < ARRAY_SIZE(names); section++) {
        const uint8_t *header;
        uint32_t len, align;

        if (!span(size, offset, 0x200)) {
            preload_invalid("DPM");
        }
        header = data + offset;
        len = ldl_le_p(header + 4);
        align = ldl_le_p(header + 0x44);
        if (ldl_le_p(header) != 0x58881688 ||
            strncmp((const char *)header + 8, names[section], 32) ||
            !len || len > capacities[section] || (len & 3) ||
            !is_power_of_2(align) ||
            !span(size, offset + 0x200, ROUND_UP((uint64_t)len, align))) {
            preload_invalid("DPM");
        }
        for (unsigned i = 0; i < 2; i++) {
            g_autofree char *name = g_strdup_printf("mt8115.dpm%u.%s", i,
                                                    names[section]);
            rom_add_blob_fixed(name, header + 0x200, len,
                               0x10900000 + i * 0x100000 + section * 0x20000);
        }
        if (!section) {
            pm_size = len;
        }
        offset += 0x200 + ROUND_UP((uint64_t)len, align);
    }
    return pm_size;
}

/* BL2 0x21ca80. The xfile descriptor field stores a BL2 virtual pointer,
 * not a physical pointer. Do not substitute an assumed direct-map address.
 */
void mt8115_preload_sspm(const uint8_t *data, size_t size, uint64_t load,
                         uint64_t bl2_va, uint32_t *cfg_init)
{
    size_t offset = 0x200;
    size_t pm = 0, dm = 0, xfile = 0;
    uint32_t pm_size = 0, dm_size = 0, xfile_size = 0;
    g_autofree uint8_t *staging = g_memdup2(data, size);

    if (size < 0x218 || ldl_le_p(data) != 0x58881688 ||
        load + size < load || load + size > UINT32_MAX) {
        preload_invalid("SSPM");
    }
    while (span(size, offset, 24) &&
           ldl_le_p(data + offset) == 0x58901690) {
        const uint8_t *hdr = data + offset;
        uint32_t header = ldl_le_p(hdr + 4), len = ldl_le_p(hdr + 8);
        uint32_t align = ldl_le_p(hdr + 12), type = ldl_le_p(hdr + 16);
        size_t payload;

        if (header < 24 || !span(size, offset, header) ||
            !is_power_of_2(align)) {
            preload_invalid("SSPM");
        }
        payload = offset + header;
        if (!len || !span(size, payload, len)) {
            preload_invalid("SSPM");
        }
        if (type == 1) {
            if (pm || len > 0x2c000 || (len & 3)) {
                preload_invalid("SSPM");
            }
            pm = payload;
            pm_size = len;
        } else if (type == 0) {
            dm = payload;
            dm_size = len;
        } else if (type == 99) {
            xfile = payload;
            xfile_size = len;
        }
        offset = payload + ROUND_UP((uint64_t)len, align);
    }
    if (!pm || !dm || !xfile) {
        preload_invalid("SSPM");
    }
    if (bl2_va) {
        uint8_t *descriptor = staging + 0x1e0;

        if (bl2_va + xfile < bl2_va || xfile_size > UINT32_MAX - 0x1c) {
            preload_invalid("SSPM");
        }
        memset(descriptor, 0, 32);
        stl_le_p(descriptor, dm - 0x1e0);
        stl_le_p(descriptor + 4, dm_size);
        stl_le_p(descriptor + 8, 0xa0000);
        stl_le_p(descriptor + 12, 0xc000);
        stq_le_p(descriptor + 16, bl2_va + xfile - 0x18);
        stl_le_p(descriptor + 24, xfile_size + 0x1c);
        cfg_init[0x20 / 4] = load + 0x1e0;
        cfg_init[0] = 1;
    }
    /* One staging ROM avoids overlapping a descriptor ROM with its image. */
    rom_add_blob_fixed("mt8115.sspm-staging", staging, size, load);
    rom_add_blob_fixed("mt8115.sspm-pm", data + pm, pm_size, 0x10400000);
}

/* Kraken condition evaluator, BL2 0x2182d0 and 0x217e30. This implements
 * the operations present in the shipped parameter image. Unknown register
 * operands and opcodes are rejected rather than assigned fabricated values.
 */
static bool pi_condition(const uint8_t *data, size_t size, size_t offset,
                          uint32_t efuse_word)
{
    uint32_t stack[100];
    unsigned depth = 0;

    while (span(size, offset, 1)) {
        uint8_t op = data[offset++];
        uint32_t left, right;

        if (op == 0 || op == 0x78) {
            if (!span(size, offset, 4) || depth == ARRAY_SIZE(stack)) {
                preload_invalid("PI condition");
            }
            stack[depth++] = ldl_le_p(data + offset);
            offset += 4;
            continue;
        }
        if (op == 0x28 || op == 0x38 || op == 0xc8) {
            /* These three operand types explicitly push zero in this BL2. */
            if (depth == ARRAY_SIZE(stack)) {
                preload_invalid("PI condition");
            }
            stack[depth++] = 0;
            continue;
        }
        if (op == 0xf0) {
            if (depth != 1) {
                preload_invalid("PI condition");
            }
            return stack[0] != 0;
        }
        if (op == 0x08) {
            uint32_t count, high, low, address;

            if (depth < 4) {
                preload_invalid("PI condition");
            }
            count = stack[--depth];
            high = stack[--depth];
            low = stack[--depth];
            address = stack[--depth];
            if (count != 3 || high >= 32 || low > high ||
                address != 0x11fb0558) {
                preload_invalid("PI register condition");
            }
            stack[depth++] = extract32(efuse_word, low, high - low + 1);
            continue;
        }
        if (depth < 2) {
            preload_invalid("PI condition");
        }
        right = stack[--depth];
        left = stack[--depth];
        switch (op) {
        case 0x40:
            left -= right;
            break;
        case 0x48:
            left = left == right;
            break;
        case 0x50:
        case 0x60:
            left |= right;
            break;
        case 0x80:
            left += right;
            break;
        case 0x90:
        case 0xa0:
            left &= right;
            break;
        default:
            preload_invalid("PI opcode");
        }
        stack[depth++] = left;
    }
    preload_invalid("PI unterminated condition");
}

/* BL2 0x2187d0 unwraps PI; 0x2183f0 selects a section, 0x2184b0 filters
 * its optional entries, and 0x2188b0 publishes it in SRAM at 0x112c00.
 * PI is parameter data. This does not execute a PI coprocessor.
 */
void mt8115_preload_pi(const uint8_t *data, size_t size, uint32_t efuse_word)
{
    const uint8_t *section = NULL;
    size_t section_size = 0;
    uint32_t count, body_offset, body_size, kept = 0;
    uint64_t conditions_size = 0;
    uint32_t expression_pos, extra_pos, header_size;
    uint64_t extra_size = 0;
    g_autofree uint8_t *out = NULL;
    g_autofree bool *selected = NULL;

    if (size < 20 || ldl_le_p(data) != 0x17c3a6b4 ||
        ldl_le_p(data + size - 4) != 0x17c3a6b4) {
        preload_invalid("PI wrapper");
    }
    data += 16;
    size -= 20;
    for (unsigned i = 0; i < lduw_le_p(data); i++) {
        uint32_t expr, start, len;

        if (!span(size, 2 + 12 * i, 12)) {
            preload_invalid("PI table");
        }
        expr = ldl_le_p(data + 2 + 12 * i);
        start = ldl_le_p(data + 6 + 12 * i);
        len = ldl_le_p(data + 10 + 12 * i);
        if (pi_condition(data, size, expr, efuse_word)) {
            if (!span(size, start, len)) {
                preload_invalid("PI section");
            }
            section = data + start;
            section_size = len;
            break;
        }
    }
    if (!section || section_size < 16) {
        preload_invalid("PI selection");
    }
    count = ldl_le_p(section);
    body_offset = ldl_le_p(section + 4);
    body_size = ldl_le_p(section + 12);
    if (!span(section_size, 16, (uint64_t)count * 20) ||
        !span(section_size, body_offset, body_size)) {
        preload_invalid("PI entries");
    }
    selected = g_new0(bool, count);
    for (unsigned i = 0; i < count; i++) {
        const uint8_t *entry = section + 16 + i * 20;
        uint32_t expr = ldl_le_p(entry), len = ldl_le_p(entry + 4);
        uint32_t extra = ldl_le_p(entry + 12), extra_len = ldl_le_p(entry + 16);

        if (!span(section_size, expr, len) ||
            !span(section_size, extra, extra_len)) {
            preload_invalid("PI entry");
        }
        selected[i] = pi_condition(section, section_size, expr, efuse_word);
        if (selected[i]) {
            kept++;
            conditions_size += len;
            extra_size += extra_len;
        }
    }
    if (16 + (uint64_t)kept * 20 + conditions_size > 0xc00) {
        preload_invalid("PI SRAM header");
    }
    header_size = 16 + kept * 20 + conditions_size;
    /* BL2 flushes the fixed 3-KiB SRAM block; reject data outside it. */
    if ((uint64_t)12 + header_size + body_size + extra_size > 0xc00) {
        preload_invalid("PI SRAM size");
    }
    out = g_malloc0(12 + header_size + body_size + extra_size);
    stl_le_p(out, 0xdeadbeef);
    stl_le_p(out + 4, header_size + body_size + extra_size);
    stl_le_p(out + 12, kept);
    stl_le_p(out + 16, header_size);
    stl_le_p(out + 20, body_size + extra_size);
    stl_le_p(out + 24, body_size);
    expression_pos = 16 + kept * 20;
    extra_pos = header_size + body_size;
    kept = 0;
    for (unsigned i = 0; i < count; i++) {
        const uint8_t *entry = section + 16 + i * 20;
        uint32_t expr = ldl_le_p(entry), len = ldl_le_p(entry + 4);
        uint32_t extra = ldl_le_p(entry + 12), extra_len = ldl_le_p(entry + 16);
        uint8_t *target;

        if (!selected[i]) {
            continue;
        }
        target = out + 12 + 16 + kept++ * 20;
        memcpy(target, entry, 20);
        stl_le_p(target, expression_pos);
        stl_le_p(target + 12, extra_pos - header_size - 4);
        memcpy(out + 12 + expression_pos, section + expr, len);
        memcpy(out + 12 + extra_pos, section + extra, extra_len);
        expression_pos += len;
        extra_pos += extra_len;
    }
    memcpy(out + 12 + header_size, section + body_offset, body_size);
    rom_add_blob_fixed("mt8115.pi-parameters", out,
                       12 + header_size + body_size + extra_size, 0x112c00);
}
