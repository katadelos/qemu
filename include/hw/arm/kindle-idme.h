/* Synthetic identities for Lab126 Kindle boards.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_KINDLE_IDME_H
#define HW_ARM_KINDLE_IDME_H

typedef struct KindleIdmeDefaults {
    const char *board;
    const char *serial;
    const char *board_id;
    const char *mac;
    const char *manufacturing;
    const char *bootmode;
    const char *postmode;
    const char *product_name;
    const char *device_type;
    const char *fos_flags;
} KindleIdmeDefaults;

const KindleIdmeDefaults *kindle_idme_defaults(const char *board);
char *kindle_idme_psn(const char *tattoo);
void kindle_idme_set_default(char **field, const char *value);

#endif /* HW_ARM_KINDLE_IDME_H */
