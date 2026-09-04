/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/ztest.h>

#include <tenstorrent/bh_chip.h>

static struct bh_chip test_chip = {
	.config = {
		.asic_reset = GPIO_DT_SPEC_GET(DT_PATH(asic_reset), gpios),
		.pgood = GPIO_DT_SPEC_GET(DT_PATH(pgood), gpios),
	}};

const struct device *gpio_emul = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct gpio_dt_spec board_fault_led =
	GPIO_DT_SPEC_GET(DT_PATH(board_fault_led), gpios);

ZTEST(pgood, test_pgood)
{
	/* Start with PGOOD high */
	gpio_emul_input_set(gpio_emul, 1, 1);
	/* Manually clear pgood_rise_triggered */
	test_chip.data.pgood_rise_triggered = 0;

	/* Wait 1ms so pgood_last_trip_ms will not be set to 0 */
	k_msleep(1);

	/* Set PGOOD low */
	gpio_emul_input_set(gpio_emul, 1, 0);
	/* Check that PGOOD fall was triggered */
	zassert_true(test_chip.data.pgood_fall_triggered);

	handle_pgood_event(&test_chip, board_fault_led);
	/* Check that PGOOD fall was handled */
	zassert_true(test_chip.data.pgood_last_trip_ms > 0);
	zassert_false(test_chip.data.pgood_fall_triggered);
	zassert_false(test_chip.data.pgood_severe_fault);
	zassert_equal(gpio_pin_get_dt(&test_chip.config.asic_reset), 1);

	/* Set PGOOD high */
	gpio_emul_input_set(gpio_emul, 1, 1);
	/* Check that PGOOD rise was triggered */
	zassert_true(test_chip.data.pgood_rise_triggered);
	handle_pgood_event(&test_chip, board_fault_led);
	zassert_false(test_chip.data.pgood_rise_triggered);
	zassert_true(test_chip.data.pgood_recovery_pending);
	zassert_equal(test_chip.data.pending_reset_type, BH_CHIP_RESET_FULL);
	zassert_true(bh_chip_reset_due(&test_chip, k_uptime_get_32()));

	/* A failed attempt retains the request and uses capped exponential backoff. */
	uint32_t retry_time = k_uptime_get_32();

	for (int i = 0; i < 10; i++) {
		bh_chip_reset_retry(&test_chip, retry_time);
		zassert_true(test_chip.data.reset_retry_at_ms - retry_time <=
			     BH_CHIP_RESET_RETRY_MAX_MS);
		zassert_true(test_chip.data.reset_retry_delay_ms <= BH_CHIP_RESET_RETRY_MAX_MS);
		zassert_false(bh_chip_reset_due(&test_chip, retry_time));
		retry_time = test_chip.data.reset_retry_at_ms;
		zassert_true(bh_chip_reset_due(&test_chip, retry_time));
	}

	bh_chip_reset_complete(&test_chip);
	zassert_equal(test_chip.data.pending_reset_type, BH_CHIP_RESET_NONE);
	zassert_equal(test_chip.data.reset_retry_delay_ms, BH_CHIP_RESET_RETRY_INITIAL_MS);
	zassert_ok(bh_chip_deassert_asic_reset(&test_chip));

	/* Set PGOOD low */
	gpio_emul_input_set(gpio_emul, 1, 0);
	zassert_true(test_chip.data.pgood_fall_triggered);

	handle_pgood_event(&test_chip, board_fault_led);
	/* Check that PGOOD fall was handled and severe state was entered */
	zassert_true(test_chip.data.pgood_last_trip_ms > 0);
	zassert_false(test_chip.data.pgood_fall_triggered);
	zassert_true(test_chip.data.pgood_severe_fault);
	/* Cleanup must not release ASIC reset into a low rail. It also makes the
	 * required full recovery durable even before the rising edge arrives.
	 */
	zassert_equal(bh_chip_deassert_asic_reset_if_pgood(&test_chip), -EAGAIN);
	zassert_equal(gpio_pin_get_dt(&test_chip.config.asic_reset), 1);
	zassert_true(test_chip.data.pgood_recovery_pending);
	zassert_equal(test_chip.data.pending_reset_type, BH_CHIP_RESET_FULL);

	/* Severe remains latched for diagnostics, but the next stable rise still queues
	 * a full recovery instead of permanently suppressing it.
	 */
	gpio_emul_input_set(gpio_emul, 1, 1);
	zassert_true(test_chip.data.pgood_rise_triggered);
	handle_pgood_event(&test_chip, board_fault_led);
	zassert_true(test_chip.data.pgood_severe_fault);
	zassert_false(test_chip.data.pgood_rise_triggered);
	zassert_true(test_chip.data.pgood_recovery_pending);
	zassert_equal(test_chip.data.pending_reset_type, BH_CHIP_RESET_FULL);
	zassert_true(bh_chip_reset_due(&test_chip, k_uptime_get_32()));
}

static void before(void *arg)
{
	ARG_UNUSED(arg);

	test_chip.data = (struct bh_chip_data){0};
	zassert_ok(gpio_pin_configure_dt(&test_chip.config.asic_reset, GPIO_OUTPUT_INACTIVE));
	zassert_ok(pgood_gpio_setup(&test_chip));
}

ZTEST_SUITE(pgood, NULL, NULL, before, NULL, NULL);
