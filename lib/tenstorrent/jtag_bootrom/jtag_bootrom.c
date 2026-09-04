/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bh_reg_def.h"
#include "status_reg.h"

#include <stdint.h>

#include <tenstorrent/bh_chip.h>
#include <tenstorrent/event.h>
#include <tenstorrent/jtag_bootrom.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/jtag.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(jtag_bootrom, CONFIG_TT_JTAG_BOOTROM_LOG_LEVEL);

#define PGOOD_WAIT_TIMEOUT_MS       1000U
#define JTAG_ID_WAIT_TIMEOUT_MS     250U
#define JTAG_AXI_WAIT_TIMEOUT_MS    1000U
#define MEM_REPAIR_WAIT_TIMEOUT_US  20000U
#define RESET_WAIT_POLL_INTERVAL_MS 1U

static uint32_t perst_start_time;

static int jtag_axiwait(const struct device *dev, uint32_t addr)
{
	/* If we are using the emulated driver then always return true */
	if (DT_HAS_COMPAT_STATUS_OKAY(zephyr_gpio_emul)) {
		return 0;
	}

	int ret = jtag_reset(dev);

	if (ret != 0) {
		return ret;
	}

	uint32_t value;

	return jtag_axi_read32(dev, addr, &value);
}

#if !DT_HAS_COMPAT_STATUS_OKAY(zephyr_gpio_emul)
static int jtag_bitbang_wait_for_id(const struct device *dev)
{
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(JTAG_ID_WAIT_TIMEOUT_MS));

	do {
		uint32_t reset_id = 0;
		int ret = jtag_reset(dev);

		if (ret != 0) {
			return ret;
		}
		ret = jtag_read_id(dev, &reset_id);
		if (ret != 0) {
			return ret;
		}
		if (reset_id == 0x138A5) {
			return 0;
		}
		k_msleep(RESET_WAIT_POLL_INTERVAL_MS);
	} while (!sys_timepoint_expired(timeout));

	return -ETIMEDOUT;
}
#endif

static const __maybe_unused struct gpio_dt_spec arc_rambus_jtag_mux_sel =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(arc_rambus_jtag_mux_sel), gpios, {0});
static const __maybe_unused struct gpio_dt_spec arc_l2_jtag_mux_sel =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(arc_l2_jtag_mux_sel), gpios, {0});

#ifdef CONFIG_JTAG_LOAD_ON_PRESET
static const struct gpio_dt_spec preset_trigger = GPIO_DT_SPEC_GET(DT_PATH(preset_trigger), gpios);

void gpio_asic_reset_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	perst_start_time = k_cycle_get_32();

	ARRAY_FOR_EACH_BH_CHIP(chip) {
		chip->data.perst_seen = true;
		atomic_set(&chip->data.trigger_reset, true);
		/* Set the bus cancel following the logic of (reset_triggered && !performing_reset)
		 */
		if (!chip->data.performing_reset) {
			bh_chip_cancel_bus_transfer_set(chip);
		}
	}
	tt_event_post(TT_EVENT_PERST);
}

static struct gpio_callback preset_cb_data;
#endif /* IS_ENABLED(CONFIG_JTAG_LOAD_ON_PRESET) */

int jtag_trigger_mem_repair(struct bh_chip *chip)
{
#ifdef CONFIG_JTAG_LOAD_BOOTROM
	/* GPIO emulation has no memory-repair state machine to acknowledge this request. */
	if (DT_HAS_COMPAT_STATUS_OKAY(zephyr_gpio_emul)) {
		return 0;
	}

	const struct device *dev = chip->config.jtag;

	int64_t start_cycles = k_cycle_get_64();
	k_timepoint_t timeout = sys_timepoint_calc(K_USEC(MEM_REPAIR_WAIT_TIMEOUT_US));
	int ret;

	LOG_DBG("start mem_repair at %lld us since boot", k_cyc_to_us_floor64(start_cycles));

	/* Start mem repair */
	ret = jtag_axi_write32(dev, RESET_UNIT_MEM_REPAIR_CNTL_REG_ADDR, 1);
	if (ret != 0) {
		return ret;
	}

	uint32_t mem_repair_status = 0;

	do {
		ret = jtag_axi_read32(dev, RESET_UNIT_MEM_REPAIR_STATUS_REG_ADDR,
				      &mem_repair_status);
		if (ret != 0) {
			return ret;
		}
		if (mem_repair_status != 0) {
			break;
		}
		k_busy_wait(10);
	} while (!sys_timepoint_expired(timeout));

	if (mem_repair_status == 0) {
		LOG_ERR("mem repair timed out at %lld us", k_cyc_to_us_floor64(k_cycle_get_64()));
		return -ETIMEDOUT;
	}

	LOG_DBG("mem repair finished at %lld us, status=0x%08x",
		k_cyc_to_us_floor64(k_cycle_get_64()), mem_repair_status);
#else
	ARG_UNUSED(chip);
#endif
	return 0;
}

int jtag_bootrom_reset_asic(struct bh_chip *chip)
{
	const struct device *dev = chip->config.jtag;
	bool resets_asserted = false;
	bool straps_set = false;
	bool jtag_setup_attempted = false;
	int cleanup_ret;
	int ret;

	/* Only check for pgood if we aren't emulating */
#if !DT_HAS_COMPAT_STATUS_OKAY(zephyr_gpio_emul)
	k_timepoint_t pgood_timeout = sys_timepoint_calc(K_MSEC(PGOOD_WAIT_TIMEOUT_MS));
	int pgood = gpio_pin_get_dt(&chip->config.pgood);

	if (pgood < 0) {
		bh_chip_hold_asic_reset_for_recovery(chip);
		return pgood;
	}
	if (pgood == 0) {
		ret = bh_chip_hold_asic_reset_for_recovery(chip);
		if (ret != 0) {
			return ret;
		}
		printk("Waiting for pgood to rise...\n");
	}
	while (pgood == 0 && !sys_timepoint_expired(pgood_timeout)) {
		k_msleep(RESET_WAIT_POLL_INTERVAL_MS);
		pgood = gpio_pin_get_dt(&chip->config.pgood);
		if (pgood < 0) {
			bh_chip_hold_asic_reset_for_recovery(chip);
			return pgood;
		}
	}
	if (pgood == 0) {
		return -ETIMEDOUT;
	}
#endif

	/* From this point, cleanup always attempts to release both reset outputs even
	 * if a GPIO driver reports an error after partially changing a line.
	 */
	resets_asserted = true;
	ret = bh_chip_assert_asic_reset(chip);
	if (ret != 0) {
		goto cleanup;
	}
	ret = bh_chip_assert_spi_reset(chip);
	if (ret != 0) {
		goto cleanup;
	}

	jtag_setup_attempted = true;
	ret = jtag_setup(dev);
	if (ret != 0) {
		goto cleanup;
	}

	/* k_sleep(K_MSEC(1)); */
	k_busy_wait(1000);

	bh_chip_set_straps(chip);
	straps_set = true;

	ret = bh_chip_deassert_asic_reset_if_pgood(chip);
	if (ret != 0) {
		goto cleanup;
	}
	ret = bh_chip_deassert_spi_reset(chip);
	if (ret != 0) {
		goto cleanup;
	}
	resets_asserted = false;

	/* k_sleep(K_MSEC(2)); */
	k_busy_wait(2000);

	ret = jtag_reset(dev);
	if (ret != 0) {
		goto cleanup;
	}

#if !DT_HAS_COMPAT_STATUS_OKAY(zephyr_gpio_emul)
	ret = jtag_bitbang_wait_for_id(dev);
	if (ret != 0) {
		goto cleanup;
	}
#endif

	ret = jtag_reset(dev);
	if (ret != 0) {
		goto cleanup;
	}

	k_timepoint_t axi_timeout = sys_timepoint_calc(K_MSEC(JTAG_AXI_WAIT_TIMEOUT_MS));

	do {
		ret = jtag_axiwait(dev, STATUS_POST_CODE_REG_ADDR);
		if (ret == 0) {
			break;
		}
		k_msleep(RESET_WAIT_POLL_INTERVAL_MS);
	} while (!sys_timepoint_expired(axi_timeout));
	if (ret != 0) {
		ret = -ETIMEDOUT;
		goto cleanup;
	}

	ret = jtag_trigger_mem_repair(chip);
	if (ret != 0) {
		goto cleanup;
	}

	ret = jtag_reset(dev);
	if (ret != 0) {
		goto cleanup;
	}

	bh_chip_unset_straps(chip);

	return 0;

cleanup:
	if (straps_set) {
		bh_chip_unset_straps(chip);
	}
	if (jtag_setup_attempted) {
		cleanup_ret = jtag_teardown(dev);
		if (cleanup_ret != 0) {
			LOG_ERR("Failed to tear down JTAG after reset error: %d", cleanup_ret);
		}
	}
	/* Always release reset lines asserted by this attempt. The caller retains the
	 * failed request and will retry after checking PGOOD again.
	 */
	if (resets_asserted) {
		cleanup_ret = bh_chip_deassert_spi_reset(chip);
		if (cleanup_ret != 0) {
			LOG_ERR("Failed to deassert SPI reset during cleanup: %d", cleanup_ret);
		}
		cleanup_ret = bh_chip_deassert_asic_reset_if_pgood(chip);
		if (cleanup_ret != 0 && cleanup_ret != -EAGAIN) {
			LOG_ERR("Failed to deassert ASIC reset during cleanup: %d", cleanup_ret);
		}
	}

	return ret;
}

int jtag_bootrom_init(struct bh_chip *chip)
{
	bool reset_outputs_touched = false;
	int cleanup_ret;
	int ret;

	if (DT_NODE_EXISTS(DT_NODELABEL(arc_rambus_jtag_mux_sel))) {
		ret = gpio_pin_configure_dt(&arc_rambus_jtag_mux_sel, GPIO_OUTPUT_ACTIVE);
		if (ret != 0) {
			return ret;
		}
	}

	if (DT_NODE_EXISTS(DT_NODELABEL(arc_l2_jtag_mux_sel))) {
		ret = gpio_pin_configure_dt(&arc_l2_jtag_mux_sel, GPIO_OUTPUT_ACTIVE);
		if (ret != 0) {
			return ret;
		}
	}

	ret = gpio_pin_configure_dt(&chip->config.pgood, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}
	reset_outputs_touched = true;
	ret = gpio_pin_configure_dt(&chip->config.asic_reset, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		goto cleanup;
	}
	ret = gpio_pin_configure_dt(&chip->config.spi_reset, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		goto cleanup;
	}

#ifdef CONFIG_JTAG_LOAD_ON_PRESET
	if (chip == &BH_CHIPS[BH_CHIP_PRIMARY_INDEX]) {
		ret = gpio_pin_configure_dt(&preset_trigger, GPIO_INPUT);
		if (ret != 0) {
			goto cleanup;
		}

		ret = gpio_pin_interrupt_configure_dt(&preset_trigger, GPIO_INT_EDGE_TO_INACTIVE);
		if (ret != 0) {
			goto cleanup;
		}

		gpio_init_callback(&preset_cb_data, gpio_asic_reset_callback,
				   BIT(preset_trigger.pin));
		ret = gpio_add_callback(preset_trigger.port, &preset_cb_data);
		if (ret != 0) {
			goto cleanup;
		}
	}

	/* Active LOW, so will be false if high */
	if (!gpio_pin_get_dt(&preset_trigger)) {
		/* If the preset trigger started high, then we came out of reset with the
		 * system
		 */
		/* thinking that pcie is ready to go. We need to forcibly apply the
		 * workaround to
		 * ensure this remains true.
		 */
		chip->data.trigger_reset = true;
	}
#endif /* IS_ENABLED(CONFIG_JTAG_LOAD_ON_PRESET) */

	return 0;

cleanup:
	if (reset_outputs_touched) {
		cleanup_ret = bh_chip_deassert_spi_reset(chip);
		if (cleanup_ret != 0) {
			LOG_ERR("Failed to deassert SPI reset during init cleanup: %d",
				cleanup_ret);
		}
		cleanup_ret = bh_chip_deassert_asic_reset_if_pgood(chip);
		if (cleanup_ret != 0 && cleanup_ret != -EAGAIN) {
			LOG_ERR("Failed to deassert ASIC reset during init cleanup: %d",
				cleanup_ret);
		}
	}

	return ret;
}

int jtag_bootrom_patch_offset(struct bh_chip *chip, const uint32_t *patch, size_t patch_len,
			      const uint32_t start_addr)
{
#ifdef CONFIG_JTAG_LOAD_BOOTROM
	const struct device *dev = chip->config.jtag;
	int ret;

	ret = jtag_reset(dev);
	if (ret != 0) {
		return ret;
	}

	/* HALT THE ARC CORE!!!!! */
	uint32_t arc_misc_cntl = 0;

	ret = jtag_axi_read32(dev, RESET_UNIT_ARC_MISC_CNTL_REG_ADDR, &arc_misc_cntl);
	if (ret != 0) {
		return ret;
	}

	arc_misc_cntl |= (0b1111 << 4);
	ret = jtag_axi_write32(dev, RESET_UNIT_ARC_MISC_CNTL_REG_ADDR, arc_misc_cntl);
	if (ret != 0) {
		return ret;
	}
	/* Reset it back to zero */
	ret = jtag_axi_read32(dev, RESET_UNIT_ARC_MISC_CNTL_REG_ADDR, &arc_misc_cntl);
	if (ret != 0) {
		return ret;
	}
	arc_misc_cntl &= ~(0b1111 << 4);
	ret = jtag_axi_write32(dev, RESET_UNIT_ARC_MISC_CNTL_REG_ADDR, arc_misc_cntl);
	if (ret != 0) {
		return ret;
	}

	/* Enable gpio trien */
	ret = jtag_axi_write32(dev, RESET_UNIT_GPIO_PAD_TRIEN_CNTL_REG_ADDR, 0xff00);
	if (ret != 0) {
		return ret;
	}

	/* Write to postcode */
	ret = jtag_axi_write32(dev, STATUS_POST_CODE_REG_ADDR, 0xF2);
	if (ret != 0) {
		return ret;
	}

	ret = jtag_axi_block_write(dev, start_addr, patch, patch_len);
	if (ret != 0) {
		return ret;
	}

	ret = jtag_axi_write32(dev, STATUS_POST_CODE_REG_ADDR, 0xF3);
	if (ret != 0) {
		return ret;
	}

	chip->data.workaround_applied = true;
#else
	ARG_UNUSED(chip);
	ARG_UNUSED(patch);
	ARG_UNUSED(patch_len);
	ARG_UNUSED(start_addr);
#endif

	return 0;
}

int jtag_bootrom_verify(const struct device *dev, const uint32_t *patch, size_t patch_len)
{
	return jtag_bootrom_verify_offset(dev, patch, patch_len, 0U);
}

int jtag_bootrom_verify_offset(const struct device *dev, const uint32_t *patch, size_t patch_len,
			       uint32_t start_addr)
{
	if (!IS_ENABLED(CONFIG_JTAG_VERIFY_WRITE)) {
		return 0;
	}

	/* Confirmed matching */
	for (int i = 0; i < patch_len; ++i) {
		/* ICCM start addr is 0 */
		uint32_t readback = 0;
		int ret;
#ifdef CONFIG_JTAG_EMUL
		ret = jtag_emul_axi_read32(dev, start_addr + i * 4, &readback);
#else
		ret = jtag_axi_read32(dev, start_addr + i * 4, &readback);
#endif
		if (ret != 0) {
			return ret;
		}

		if (patch[i] != readback) {
			printk("Bootcode mismatch at %03x. expected: %08x actual: %08x "
			       "¯\\_(ツ)_/¯\n",
			       start_addr + i * 4, patch[i], readback);

			ret = jtag_axi_write32(dev, STATUS_POST_CODE_REG_ADDR, 0x6);
			return ret != 0 ? ret : -EIO;
		}
	}

	printk("Bootcode write verified! \\o/\n");

	return 0;
}

uint32_t jtag_bootrom_get_perst_start_time(void)
{
	return perst_start_time;
}

int jtag_bootrom_soft_reset_arc(struct bh_chip *chip)
{
#ifdef CONFIG_JTAG_LOAD_BOOTROM
	const struct device *dev = chip->config.jtag;
	uint32_t dm_init_done = k_cycle_get_32();
	int ret;

	ret = jtag_reset(dev);
	if (ret != 0) {
		return ret;
	}

	/* HALT THE ARC CORE!!!!! */

	/* NOTE(drosen): Assuming that it is okay to set the register to 0b1111 << 4, this saves
	 * some cycles but may lead to errors in the future.
	 */
	ret = jtag_axi_write32(dev, RESET_UNIT_ARC_MISC_CNTL_REG_ADDR, GENMASK(7, 4));
	if (ret != 0) {
		return ret;
	}
	/* Reset it back to zero */
	/* NOTE(drosen): Assuming that it is okay to set the register back to zero, this saves some
	 * cycles but may lead to errors in the future.
	 */
	ret = jtag_axi_write32(dev, RESET_UNIT_ARC_MISC_CNTL_REG_ADDR, 0);
	if (ret != 0) {
		return ret;
	}

	/* Write reset_vector (rom_memory[0]) */
	ret = jtag_axi_write32(dev, ROM_MEMORY_MEM_BASE_ADDR, 0x84);
	if (ret != 0) {
		return ret;
	}

	/* store ASIC refclk timestamp of DMC starts bootcode execution as a reference for cmfw. */
	ret = jtag_axi_read32(dev, RESET_UNIT_REFCLK_CNT_LO_REG_ADDR, &chip->data.arc_start_time);
	if (ret != 0) {
		return ret;
	}

	/* Toggle soft-reset */
	/* ARC_MISC_CNTL.soft_reset (12th bit) */
	/* NOTE(drosen): Assuming that it is okay to set the register to 1 << 12, this saves some
	 * cycles but may lead to errors in the future.
	 */
	ret = jtag_axi_write32(dev, RESET_UNIT_ARC_MISC_CNTL_REG_ADDR, BIT(12));
	if (ret != 0) {
		return ret;
	}

	/* Set to 0 */
	/* NOTE(drosen): Assuming that it is okay to set the register back to zero, this saves some
	 * cycles but may lead to errors in the future.
	 */
	ret = jtag_axi_write32(dev, RESET_UNIT_ARC_MISC_CNTL_REG_ADDR, 0);
	if (ret != 0) {
		return ret;
	}

	/* Only consume the PERST timing marker after the soft reset completed. A
	 * failed attempt remains eligible to record timing on its retry.
	 */
	if (chip->data.perst_seen) {
		chip->data.dm_init_done = dm_init_done;
		chip->data.perst_seen = false;
	}
#endif
	return 0;
}

int jtag_bootrom_set_cable_power_limit(struct bh_chip *chip, uint16_t power_limit)
{
#ifdef CONFIG_JTAG_LOAD_BOOTROM
	const struct device *dev = chip->config.jtag;

	/* Write cable power limit with magic marker for SMC to detect feature support.
	 * Format: [31:16] = CABLE_POWER_LIMIT_MAGIC, [15:0] = power_limit
	 * Legacy SMC will read this as a large positive value (not 0), so safe.
	 * New SMC checks for magic marker to enable cable fault detection.
	 * A power_limit of 0 indicates cable fault (no cable or improper installation).
	 */
	uint32_t value = CABLE_POWER_LIMIT_MAGIC | (uint32_t)power_limit;

	return jtag_axi_write32(dev, DMC_CABLE_POWER_LIMIT_REG_ADDR, value);
#else
	ARG_UNUSED(chip);
	ARG_UNUSED(power_limit);
	return 0;
#endif
}

int jtag_bootrom_teardown(const struct bh_chip *chip)
{
	int reset_ret;
	int teardown_ret;

	/* Just one more for good luck */
	reset_ret = jtag_reset(chip->config.jtag);
	teardown_ret = jtag_teardown(chip->config.jtag);

	return reset_ret != 0 ? reset_ret : teardown_ret;
}
