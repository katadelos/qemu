/* MT8115 shipped BL2 firmware handoff reconstruction.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_MT8115_PRELOAD_H
#define HW_ARM_MT8115_PRELOAD_H

/* DPM PM and DM SRAM copies; returns PM length for reset register state.
 * The caller retains the complete original FIT staging image separately.
 */
uint32_t mt8115_preload_dpm(const uint8_t *data, size_t size);

/* Installs SSPM staging and PM SRAM together. bl2_va is the independently
 * established BL2 virtual mapping of staging, or zero when not established.
 * Only a known mapping permits descriptor publication and reset release.
 * cfg_init points to the board's 64-KiB SSPM reset-register image.
 */
void mt8115_preload_sspm(const uint8_t *data, size_t size, uint64_t load,
                         uint64_t bl2_va, uint32_t *cfg_init);

/* PI selection uses the explicit virtual-device 0x11fb0558 fuse value.
 * This must not be described as a captured physical-device fuse value.
 */
void mt8115_preload_pi(const uint8_t *data, size_t size, uint32_t efuse_word);
#endif
