/* Synthetic identities for Lab126 Kindle boards.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/kindle-idme.h"
#include "qemu/error-report.h"

/* PSN v2 requires P at byte zero; 000 selects the legacy Yoshi decoder.
 * Serial device codes and PCB tattoos describe different identifiers.
 */
static const KindleIdmeDefaults kindle_identities[] = {
    {
        .board = "tequila",
        .serial = "B00E000000000001",
        .board_id = "0031500000000001",
        .mac = "020000000001",
        .manufacturing = "00000000000000000000",
        .bootmode = "main",
        .postmode = "normal",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "whitney",
        .serial = "B011000000000001",
        .board_id = "0060600000000001",
        .mac = "020000000003",
        .manufacturing = "00000000000000000000",
        .bootmode = "main",
        .postmode = "normal",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "celeste",
        .serial = "B024000000000001",
        .board_id = "00A1000000000001",
        .mac = "020000000004",
        .manufacturing = "00000000000000000000",
        .bootmode = "main",
        .postmode = "normal",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "icewine",
        .serial = "B013000000000001",
        .board_id = "0470000000000001",
        .mac = "020000000002",
        .manufacturing = "00000000000000000000",
        .bootmode = "main",
        .postmode = "normal",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "bourbon",
        .serial = "90DD000000000001",
        .board_id = "0510300000000001",
        .mac = "020000000002",
        .manufacturing = "00000000000000000000",
        .bootmode = "main",
        .postmode = "normal",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "pinot",
        .serial = "B0D4000000000001",
        .board_id = "0271600000000001",
        .mac = "020000000002",
        .manufacturing = "00000000000000000000",
        .bootmode = "main",
        .postmode = "normal",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "muscat",
        .serial = "G090G10500000000",
        .board_id = "0670400000000001",
        .mac = "020000000002",
        .manufacturing = "00000000000000000000",
        .bootmode = "main",
        .postmode = "normal",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "eanab",
        .serial = "G000K90000000001",
        .board_id = "P001180400000001",
        .mac = "020000000006",
        .manufacturing = "00000000000000000000",
        .bootmode = "main",
        .postmode = "normal",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "whisky",
        .serial = "G0B0GC0000000001",
        .board_id = "0790700000000001",
        .mac = "020000000001",
        .manufacturing = "00000000000000000000",
        .bootmode = "main",
        .postmode = "normal",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "cognac",
        .serial = "G000SA0000000000",
        .board_id = "P0013H0000000000",
        .mac = "020000000004",
        .manufacturing = "ZELDAQEMU0000000000000",
        .bootmode = "0",
        .postmode = "0",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "moonshine",
        .serial = "G000PP0400000000",
        .board_id = "P0015R0000000000",
        .mac = "020000000004",
        .manufacturing = "REXQEMU0000000000000",
        .bootmode = "1",
        .postmode = "0",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "jaeger",
        .serial = "G000WF0400000000",
        .board_id = "P001M70000000000",
        .mac = "020000000010",
        .manufacturing = "JAEGERQEMU0000000",
        .bootmode = "0",
        .postmode = "0",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "stinger",
        .serial = "G000WL0000000000",
        .board_id = "P001C70000000000",
        .mac = "020000000006",
        .manufacturing = "STINGERQEMU0000000000000",
        .bootmode = "0",
        .postmode = "0",
        .product_name = "",
        .device_type = "",
        .fos_flags = "0",
    },
    {
        .board = "malbec",
        .serial = "G002DK0000000000",
        .board_id = "P002R90000000000",
        .mac = "020000000006",
        .manufacturing = "PAPERWHITE5QEMU00",
        .bootmode = "1",
        .postmode = "0",
        .product_name = "",
        .device_type = "2DK",
        .fos_flags = "0",
    },
    {
        .board = "cava",
        .serial = "G0022D0000000000",
        .board_id = "P002R50000000000",
        .mac = "020000000006",
        .manufacturing = "KINDLE11QEMU000000",
        .bootmode = "1",
        .postmode = "0",
        .product_name = "",
        .device_type = "22D",
        .fos_flags = "0",
    },
    {
        .board = "rossini",
        .serial = "G003KM0000000000",
        .board_id = "P003M50000000000",
        .mac = "020000000006",
        .manufacturing = "KINDLE11QEMU000000",
        .bootmode = "1",
        .postmode = "0",
        .product_name = "",
        .device_type = "3KM",
        .fos_flags = "0",
    },
    {
        .board = "barolo",
        .serial = "G002WS0000000000",
        .board_id = "P002WS0000000000",
        .mac = "020000000006",
        .manufacturing = "SCRIBE1QEMU000000",
        .bootmode = "1",
        .postmode = "0",
        .product_name = "",
        .device_type = "2WS",
        .fos_flags = "0",
    },
    {
        .board = "pisco",
        .serial = "G003XE0000000000",
        .board_id = "P003XE0000000000",
        .mac = "020000000006",
        .manufacturing = "SCRIBE2QEMU000000",
        .bootmode = "1",
        .postmode = "0",
        .product_name = "",
        .device_type = "3XE",
        .fos_flags = "0",
    },
    {
        .board = "prosecco",
        .serial = "GN456U0000000000",
        .board_id = "PN44PB0200000000",
        .mac = "020000000006",
        .manufacturing = "KT7QEMU0000000000 bsn_shipped=SWCS2610100000000056",
        .bootmode = "0",
        .postmode = "0",
        .product_name = "po8",
        .device_type = "A1BCDFP38HIEWE",
        .fos_flags = "80",
    },
    {
        .board = "sangria",
        .serial = "G003J50000000000",
        .board_id = "P003RV0000000000",
        .mac = "020000000005",
        .manufacturing = "PAPERWHITE6QEMU0000",
        .bootmode = "1",
        .postmode = "0",
        /* Let devcap select the PW12 capability group from the tattoo. */
        .product_name = "",
        .device_type = "3J5",
        .fos_flags = "0",
    },
    {
        .board = "sangria-color",
        .serial = "G003H90000000000",
        .board_id = "P003RW0000000000",
        .mac = "020000000005",
        .manufacturing = "COLOURSOFTQEMU00000",
        .bootmode = "1",
        .postmode = "0",
        .product_name = "se9_p",
        .device_type = "3H9",
        .fos_flags = "0",
    },
    {
        .board = "calvados",
        .serial = "G004VX0000000000",
        /* PA6/CS8 use supported device codes as synthetic PCB tattoos. */
        .board_id = "P004VX0000000000",
        .mac = "020000000001",
        .manufacturing = "ScribeColorsoftQEMU",
        .bootmode = "main",
        .postmode = "0",
        .product_name = "cs8",
        .device_type = "4VX",
        .fos_flags = "0",
    },
    {
        .board = "paloma",
        .serial = "G004PG0000000000",
        .board_id = "P004PG0000000000",
        .mac = "020000000001",
        .manufacturing = "Scribe3QEMU",
        .bootmode = "main",
        .postmode = "0",
        .product_name = "pa6",
        .device_type = "4PG",
        .fos_flags = "0",
    },
};

const KindleIdmeDefaults *kindle_idme_defaults(const char *board)
{
    for (size_t i = 0; i < ARRAY_SIZE(kindle_identities); i++) {
        const KindleIdmeDefaults *idme = &kindle_identities[i];

        if (!strcmp(board, idme->board)) {
            g_assert(strlen(idme->serial) == 16);
            g_assert(strlen(idme->board_id) == 16);
            g_assert(strlen(idme->mac) == 12);
            return idme;
        }
    }
    error_report("unknown Kindle IDME board '%s'", board);
    exit(EXIT_FAILURE);
}

char *kindle_idme_psn(const char *tattoo)
{
    g_assert(strlen(tattoo) == 3);
    return g_strdup_printf("P00%s0000000000", tattoo);
}

void kindle_idme_set_default(char **field, const char *value)
{
    /* Resolve shared-board defaults after CLI options. Explicit empty strings
     * are overrides too, so only an unset pointer receives the default.
     */
    if (!*field) {
        *field = g_strdup(value);
    }
}
