/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TENSTORRENT_JTAG_BOOTROM_H_
#define TENSTORRENT_JTAG_BOOTROM_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zephyr/spinlock.h>

#include <tenstorrent/bh_chip.h>

#ifdef __cplusplus
extern "C" {
#endif

const uint8_t *get_bootcode(void);
const size_t get_bootcode_len(void);

int jtag_bootrom_init(struct bh_chip *chip);

int jtag_bootrom_reset_asic(struct bh_chip *chip);

int jtag_bootrom_patch_offset(struct bh_chip *chip, const uint32_t *patch, size_t patch_len,
			      const uint32_t start_addr);
int jtag_bootrom_verify(const struct device *dev, const uint32_t *patch, size_t patch_len);
int jtag_bootrom_verify_offset(const struct device *dev, const uint32_t *patch, size_t patch_len,
			       uint32_t start_addr);
int jtag_bootrom_soft_reset_arc(struct bh_chip *chip);
int jtag_bootrom_set_cable_power_limit(struct bh_chip *chip, uint16_t power_limit);
int jtag_bootrom_teardown(const struct bh_chip *chip);

uint32_t jtag_bootrom_get_perst_start_time(void);

#ifdef __cplusplus
}
#endif

#endif
