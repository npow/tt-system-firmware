/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include "telemetry.h"
#include "throttler.h"
#include "voltage.h"
#include "cm2dm_msg.h"
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

static void throttler_before(void *fixture)
{
	ARG_UNUSED(fixture);
	ThrottlerTestResetRuntimePowerState();
}

static void throttler_after(void *fixture)
{
	ARG_UNUSED(fixture);
	ThrottlerTestResetRuntimePowerState();
}

ZTEST(throttler, test_set_board_power_limit)
{
	set_dmc_board_power_limit(300);

	zassert_equal(set_board_power_limit(225, false), 0);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 225);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 225);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 247);
	zassert_equal(ThrottlerGetDopplerSlowLimit(), 213);
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
	zassert_true(ThrottlerStrictRuntimePowerLimitActive());
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 300);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 330);
}

ZTEST(throttler, test_dmc_default_enables_bounded_power_policy)
{
	set_dmc_board_power_limit(300);

	zassert_true(ThrottlerStrictRuntimePowerLimitActive());
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 300);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 330);
	zassert_equal(ThrottlerGetDopplerSlowLimit(), 285);

	zassert_equal(set_board_power_limit(150, false), 0);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 150);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 165);
	zassert_equal(ThrottlerGetDopplerSlowLimit(), 142);
}

ZTEST(throttler, test_fast_clamp_retains_peak_and_releases_with_hysteresis)
{
	set_dmc_board_power_limit(300);

	ThrottlerTestRecordInputPowerSampleAtPower(1000, 299);
	zassert_false(ThrottlerRuntimePowerClampActive());
	ThrottlerTestRecordInputPowerSampleAtPower(1001, 300);
	zassert_true(ThrottlerRuntimePowerClampActive());

	/* A newer low sample cannot erase the high sample before DVFS consumes it. */
	ThrottlerTestRecordInputPowerSampleAtPower(1002, 200);
	zassert_true(ThrottlerRuntimePowerClampActive());
	zassert_equal(ThrottlerTestConsumeRuntimePowerPeak(), 300);
	ThrottlerTestRecordInputPowerSampleAtPower(1003, 285);
	zassert_false(ThrottlerRuntimePowerClampActive());
}

ZTEST(throttler, test_missing_sample_clamp_is_reversible)
{
	set_dmc_board_power_limit(300);
	ThrottlerTestStartRuntimePowerSampleWatchdog(100);

	zassert_false(ThrottlerTestRuntimePowerSampleExpired(199));
	zassert_true(ThrottlerTestRuntimePowerSampleExpired(200));
	zassert_true(ThrottlerTestUpdateRuntimePowerFreshnessGuard(200));
	zassert_true(ThrottlerRuntimePowerClampActive());

	ThrottlerTestRecordInputPowerSampleAtPower(201, 200);
	zassert_false(ThrottlerTestRuntimePowerSampleExpired(201));
	zassert_false(ThrottlerTestUpdateRuntimePowerFreshnessGuard(201));
	zassert_false(ThrottlerRuntimePowerClampActive());
}

ZTEST(throttler, test_invalid_power_sample_does_not_refresh_watchdog)
{
	uint8_t malformed = 0U;

	set_dmc_board_power_limit(300);
	ThrottlerTestStartRuntimePowerSampleWatchdog(100);
	zassert_equal(Dm2CmSendPowerHandler(&malformed, sizeof(malformed)), -1);
	zassert_true(ThrottlerTestRuntimePowerSampleExpired(200));
}

ZTEST(throttler, test_saturated_power_sample_stays_unsafe)
{
	uint8_t data[2];

	set_dmc_board_power_limit(300);
	sys_put_le16(UINT16_MAX, data);
	zassert_ok(Dm2CmSendPowerHandler(data, sizeof(data)));
	zassert_equal(ThrottlerGetInputPower(), UINT16_MAX);
	zassert_true(ThrottlerRuntimePowerClampActive());
}

ZTEST(throttler, test_runtime_controller_uses_instantaneous_and_short_windows)
{
	set_dmc_board_power_limit(300);
	ThrottlerTestResetBoardPowerHistory(100);

	zassert_equal(ThrottlerTestUpdateBoardPowerHistory(400), 400);
	zassert_equal(ThrottlerTestUpdateBoardPowerHistory(100), 118);
}

ZTEST(throttler, test_strict_policy_rejects_forced_voltage)
{
	voltage_arbiter.vdd_min = 650U;
	voltage_arbiter.vdd_max = 900U;
	set_dmc_board_power_limit(300);
	zassert_equal(ForceVdd(800), 2);
}

ZTEST_SUITE(throttler, NULL, NULL, throttler_before, throttler_after, NULL);
