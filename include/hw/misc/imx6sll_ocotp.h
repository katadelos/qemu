/* Minimal i.MX6SLL OCOTP controller. */

#ifndef HW_MISC_IMX6SLL_OCOTP_H
#define HW_MISC_IMX6SLL_OCOTP_H

#include "hw/core/sysbus.h"

#define TYPE_IMX6SLL_OCOTP "imx6sll-ocotp"
OBJECT_DECLARE_SIMPLE_TYPE(IMX6SLLOCOTPState, IMX6SLL_OCOTP)

void imx6sll_ocotp_set_fuse(IMX6SLLOCOTPState *s, unsigned bank,
                            unsigned word, uint32_t value);
void imx6sll_ocotp_override_fuse(IMX6SLLOCOTPState *s, unsigned bank,
                                 unsigned word, uint32_t value);
void imx6sll_ocotp_apply_shadow_overrides(IMX6SLLOCOTPState *s);

#endif
