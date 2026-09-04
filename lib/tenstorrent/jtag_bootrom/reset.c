/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <tenstorrent/jtag_bootrom.h>
#include <tenstorrent/bh_chip.h>

#include <zephyr/drivers/jtag.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(jtag_bootrom, CONFIG_TT_JTAG_BOOTROM_LOG_LEVEL);

#define BOOTCODE_LOAD_OFFSET 0x80U

__aligned(sizeof(uint32_t)) static const uint8_t bootcode[] = {
#include "bootcode.h"
};

/* discarded if no zephyr,gpio-emul exists or if CONFIG_JTAG_VERIFY_WRITE=n */
__maybe_unused
	__aligned(sizeof(uint32_t)) static uint8_t sram[sizeof(bootcode) + BOOTCODE_LOAD_OFFSET];

const uint8_t *get_bootcode(void)
{
	return bootcode;
}

const size_t get_bootcode_len(void)
{
	return sizeof(bootcode) / sizeof(uint32_t);
}

int jtag_bootrom_reset_sequence(struct bh_chip *chip, bool force_reset, uint16_t cable_power_limit)
{
	const uint32_t *const patch = (const uint32_t *)bootcode;
	const size_t patch_len = get_bootcode_len();
	bool jtag_ready = false;
	int cleanup_ret;
	int ret;

#ifdef CONFIG_JTAG_LOAD_ON_PRESET
	bool soft_reset_requested = force_reset;

	soft_reset_requested |= atomic_set(&chip->data.trigger_reset, false);
#else
	ARG_UNUSED(force_reset);
#endif

	LOG_DBG("start reset sequence at %lld us", k_cyc_to_us_floor64(k_cycle_get_64()));

	/* Need to be able to send an i2c transaction to set the straps on the p300 */
	bh_chip_cancel_bus_transfer_clear(chip);
	ret = jtag_bootrom_reset_asic(chip);

	if (ret != 0) {
		goto cleanup_resets;
	}
	jtag_ready = true;

#if DT_HAS_COMPAT_STATUS_OKAY(zephyr_gpio_emul) && defined(CONFIG_JTAG_VERIFY_WRITE)
	jtag_emul_setup(chip->config.jtag, (uint32_t *)sram, ARRAY_SIZE(sram) / sizeof(uint32_t));
#endif

	ret = jtag_bootrom_patch_offset(chip, patch, patch_len, BOOTCODE_LOAD_OFFSET);
	if (ret != 0) {
		goto cleanup;
	}

	LOG_DBG("load sequence finished at %lld us", k_cyc_to_us_floor64(k_cycle_get_64()));

	ret = jtag_bootrom_verify_offset(chip->config.jtag, patch, patch_len, BOOTCODE_LOAD_OFFSET);
	if (ret != 0) {
		printk("Bootrom verification failed\n");
		goto cleanup;
	}

	ret = jtag_bootrom_set_cable_power_limit(chip, cable_power_limit);
	if (ret != 0) {
		goto cleanup;
	}

#ifdef CONFIG_JTAG_LOAD_ON_PRESET
	if (soft_reset_requested) {
		ret = jtag_bootrom_soft_reset_arc(chip);
		if (ret != 0) {
			goto cleanup;
		}
	}
#else
	ret = jtag_bootrom_soft_reset_arc(chip);
	if (ret != 0) {
		goto cleanup;
	}
#endif

cleanup:
	if (jtag_ready) {
		cleanup_ret = jtag_bootrom_teardown(chip);
		if (ret == 0) {
			ret = cleanup_ret;
		}
	}

cleanup_resets:
	if (ret != 0) {
		/* The lower-level reset helper already unwinds these lines, but repeat the
		 * deassertions here so every reset-sequence error exit is independently safe.
		 */
		cleanup_ret = bh_chip_deassert_spi_reset(chip);
		if (cleanup_ret != 0) {
			LOG_ERR("Failed to deassert SPI reset during cleanup: %d", cleanup_ret);
		}
		cleanup_ret = bh_chip_deassert_asic_reset_if_pgood(chip);
		if (cleanup_ret != 0 && cleanup_ret != -EAGAIN) {
			LOG_ERR("Failed to deassert ASIC reset during cleanup: %d", cleanup_ret);
		}
	}

	if (ret != 0) {
		return ret;
	}
	LOG_DBG("reset sequence finished at %lld us", k_cyc_to_us_floor64(k_cycle_get_64()));
	return 0;
}
