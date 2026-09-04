/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>

#include <tenstorrent/jtag_bootrom.h>
#include <zephyr/drivers/jtag.h>

enum failure_mode {
	FAIL_NONE,
	FAIL_PATCH_WITH_PGOOD_LOW,
	FAIL_CABLE_LIMIT,
	FAIL_SOFT_RESET,
	FAIL_TEARDOWN,
};

static const struct device *const real_jtag = DEVICE_DT_GET(DT_PATH(jtag));
static const struct jtag_api *real_jtag_api;
static struct device test_jtag;
static enum failure_mode failure_mode;
static unsigned int teardown_count;

static int test_smbus_cancel(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static const struct smbus_driver_api test_smbus_api = {
	.smbus_cancel = test_smbus_cancel,
	.smbus_uncancel = test_smbus_cancel,
};
static const struct device test_smbus = {
	.api = &test_smbus_api,
};
static struct bh_chip test_chip = {.config = {
					   .jtag = &test_jtag,
					   .asic_reset = GPIO_DT_SPEC_GET(DT_PATH(mcureset), gpios),
					   .spi_reset = GPIO_DT_SPEC_GET(DT_PATH(spireset), gpios),
					   .pgood = GPIO_DT_SPEC_GET(DT_PATH(pgood), gpios),
					   .arc = {.smbus = {.bus = &test_smbus}},
				   }};

static int test_jtag_setup(const struct device *dev)
{
	return real_jtag_api->setup(dev);
}

static int test_jtag_teardown(const struct device *dev)
{
	int ret = real_jtag_api->teardown(dev);

	teardown_count++;
	return failure_mode == FAIL_TEARDOWN ? -EIO : ret;
}

static int test_jtag_reset(const struct device *dev)
{
	return real_jtag_api->reset(dev);
}

static int test_jtag_axi_read32(const struct device *dev, uint32_t addr, uint32_t *value)
{
	/* The GPIO JTAG emulator models SRAM, not Blackhole's sparse control
	 * registers. Model those as readable zero-valued registers in this test.
	 */
	if (addr >= 0x80000000U) {
		*value = 0U;
		return 0;
	}

	return real_jtag_api->axi_read32(dev, addr, value);
}

static int test_jtag_axi_write32(const struct device *dev, uint32_t addr, uint32_t value)
{
	if (failure_mode == FAIL_CABLE_LIMIT || failure_mode == FAIL_SOFT_RESET) {
		return -EIO;
	}
	if (addr >= 0x80000000U) {
		return 0;
	}

	return real_jtag_api->axi_write32(dev, addr, value);
}

static int test_jtag_axi_block_write(const struct device *dev, uint32_t addr, const uint32_t *value,
				     uint32_t len)
{
	if (failure_mode == FAIL_PATCH_WITH_PGOOD_LOW) {
		zassert_ok(gpio_emul_input_set(test_chip.config.pgood.port,
					       test_chip.config.pgood.pin, 0));
		return -EIO;
	}

	return real_jtag_api->axi_block_write(dev, addr, value, len);
}

static const struct jtag_api test_jtag_api = {
	.setup = test_jtag_setup,
	.teardown = test_jtag_teardown,
	.reset = test_jtag_reset,
	.axi_read32 = test_jtag_axi_read32,
	.axi_write32 = test_jtag_axi_write32,
	.axi_block_write = test_jtag_axi_block_write,
};

ZTEST(jtag_bootrom, test_jtag_bootrom)
{
	const uint32_t *const patch = (const uint32_t *)get_bootcode();

	/* A patch failure concurrent with PGOOD falling must propagate, tear JTAG
	 * down, release SPI reset, retain ASIC reset, and queue full recovery.
	 */
	failure_mode = FAIL_PATCH_WITH_PGOOD_LOW;
	zassert_equal(jtag_bootrom_reset_sequence(&test_chip, false, 300), -EIO);
	zassert_equal(teardown_count, 1U);
	zassert_equal(gpio_emul_output_get(test_chip.config.asic_reset.port,
					   test_chip.config.asic_reset.pin),
		      1);
	zassert_equal(gpio_emul_output_get(test_chip.config.spi_reset.port,
					   test_chip.config.spi_reset.pin),
		      0);
	zassert_true(test_chip.data.pgood_recovery_pending);
	zassert_equal(test_chip.data.pending_reset_type, BH_CHIP_RESET_FULL);

	zassert_ok(gpio_emul_input_set(test_chip.config.pgood.port, test_chip.config.pgood.pin, 1));
	zassert_ok(bh_chip_deassert_asic_reset_if_pgood(&test_chip));
	bh_chip_reset_complete(&test_chip);
	uint32_t bad_word = ~patch[0];

	zassert_not_equal(jtag_bootrom_verify(test_chip.config.jtag, &bad_word, 1), 0);

	failure_mode = FAIL_CABLE_LIMIT;
	zassert_equal(jtag_bootrom_set_cable_power_limit(&test_chip, 300), -EIO);
	failure_mode = FAIL_SOFT_RESET;
	zassert_equal(jtag_bootrom_soft_reset_arc(&test_chip), -EIO);
	failure_mode = FAIL_TEARDOWN;
	zassert_equal(jtag_bootrom_teardown(&test_chip), -EIO);
	zassert_equal(teardown_count, 2U);
	failure_mode = FAIL_NONE;
	zassert_ok(jtag_setup(test_chip.config.jtag));
	zassert_equal(gpio_emul_output_get(test_chip.config.asic_reset.port,
					   test_chip.config.asic_reset.pin),
		      0);
	zassert_equal(gpio_emul_output_get(test_chip.config.spi_reset.port,
					   test_chip.config.spi_reset.pin),
		      0);
}

static void before(void *arg)
{
	ARG_UNUSED(arg);

	real_jtag_api = real_jtag->api;
	test_jtag = *real_jtag;
	test_jtag.api = &test_jtag_api;
	test_chip.data = (struct bh_chip_data){0};
	failure_mode = FAIL_NONE;
	teardown_count = 0U;
	zassert_ok(jtag_bootrom_init(&test_chip));
	zassert_ok(gpio_emul_input_set(test_chip.config.pgood.port, test_chip.config.pgood.pin, 1));
	zassert_ok(jtag_bootrom_reset_asic(&test_chip));
}

static void after(void *arg)
{
	ARG_UNUSED(arg);

	zassert_ok(jtag_bootrom_teardown(&test_chip));
}

ZTEST_SUITE(jtag_bootrom, NULL, NULL, before, after, NULL);
