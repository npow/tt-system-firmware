/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include "aiclk_ppm.h"
#include "telemetry.h"
#include "throttler.h"
#include <tenstorrent/msgqueue.h>
#include <tenstorrent/smc_msg.h>

static uint8_t set_board_power_limit(uint32_t watts, bool restore_default)
{
	union request req = {0};
	struct response rsp = {0};

	req.set_board_power_limit.command_code = TT_SMC_MSG_SET_BOARD_POWER_LIMIT;
	req.set_board_power_limit.board_power_limit = watts;
	req.set_board_power_limit.restore_default = restore_default;

	zassert_ok(msgqueue_request_push(0, &req));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));

	return rsp.data[0];
}

static void set_dmc_board_power_limit(uint16_t watts)
{
	uint8_t data[2];

	sys_put_le16(watts, data);
	zassert_ok(Dm2CmSetBoardPowerLimit(data, sizeof(data)));
}

ZTEST(throttler, test_set_board_power_limit)
{
	set_dmc_board_power_limit(300);

	zassert_equal(set_board_power_limit(225, false), 0);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 225);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 247);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 270);
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
}

ZTEST(throttler, test_reject_board_power_limit_above_board_maximum)
{
	set_dmc_board_power_limit(300);

	zassert_equal(set_board_power_limit(301, false), 1);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300);
}

ZTEST(throttler, test_reject_board_power_limit_below_controller_minimum)
{
	set_dmc_board_power_limit(300);

	zassert_equal(set_board_power_limit(49, false), 1);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300);
}

ZTEST(throttler, test_restore_board_power_limit)
{
	set_dmc_board_power_limit(300);
	zassert_equal(set_board_power_limit(225, false), 0);

	zassert_equal(set_board_power_limit(0, true), 0);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 600);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 750);
}

ZTEST(throttler, test_runtime_board_power_limit_tightens_transient_thresholds)
{
	set_dmc_board_power_limit(300);

	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 600);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 750);

	zassert_equal(set_board_power_limit(150, false), 0);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 165);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 180);
}

ZTEST(throttler, test_runtime_power_guard_requires_sustained_overage)
{
	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	zassert_equal(set_board_power_limit(100, false), 0);
	zassert_equal(ThrottlerGetRuntimePowerFailSafeLimit(), 110);

	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1000));
	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1099));
	zassert_false(ThrottlerRuntimePowerFaultLatched());
	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1100));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT), (160U << 16U) | 1U);
	zassert_equal(set_board_power_limit(100, false), 2);

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_runtime_power_guard_resets_dwell_after_recovery)
{
	ThrottlerTestResetRuntimePowerGuard();

	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1000));
	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(false, 100, 1099));
	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1100));
	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1199));
	zassert_false(ThrottlerRuntimePowerFaultLatched());

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST_SUITE(throttler, NULL, NULL, NULL, NULL, NULL);
