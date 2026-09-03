/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include "aiclk_ppm.h"
#include "cm2dm_msg.h"
#include "init.h"
#include "pcie.h"
#include "reg_mock.h"
#include "telemetry.h"
#include "throttler.h"
#include <tenstorrent/bh_power.h>
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

static void *throttler_setup(void)
{
	InitThrottlers();
	PcieTestResetErrorInterrupts();
	return NULL;
}

ZTEST(throttler, test_reject_zero_dmc_power_limit)
{
	uint8_t data[2];

	ThrottlerTestResetRuntimePowerGuard();
	sys_put_le16(0, data);
	zassert_equal(Dm2CmSetBoardPowerLimit(data, sizeof(data)), -1);
	zassert_false(ThrottlerRuntimePowerFaultLatched());
}

static uint8_t request_full_power_state(void)
{
	union request req = {0};
	struct response rsp = {0};

	req.power_setting.command_code = TT_SMC_MSG_POWER_SETTING;
	req.power_setting.power_flags_valid = BH_POWER_DOMAIN_COUNT;
	req.power_setting.power_flags_bitfield.max_ai_clk = 1;
	req.power_setting.power_flags_bitfield.mrisc_phy_power = 1;
	req.power_setting.power_flags_bitfield.tensix_enable = 1;
	req.power_setting.power_flags_bitfield.l2cpu_enable = 1;

	zassert_ok(msgqueue_request_push(0, &req));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));

	return rsp.data[0];
}

static uint8_t request_aiclk_control(const union request *req)
{
	struct response rsp = {0};

	zassert_ok(msgqueue_request_push(0, req));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));
	return rsp.data[0];
}

ZTEST(throttler, test_set_board_power_limit)
{
	set_dmc_board_power_limit(300);

	zassert_equal(set_board_power_limit(225, false), 0);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 225);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 202);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 225);
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
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 270);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 300);
}

ZTEST(throttler, test_runtime_board_power_limit_tightens_transient_thresholds)
{
	set_dmc_board_power_limit(300);

	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 270);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 300);

	zassert_equal(set_board_power_limit(150, false), 0);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 135);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 150);
}

ZTEST(throttler, test_runtime_power_guard_trips_on_first_emergency_sample)
{
	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	zassert_equal(set_board_power_limit(100, false), 0);
	zassert_equal(ThrottlerGetRuntimePowerFailSafeLimit(), 100);
	zassert_false(ThrottlerTestRuntimePowerFailSafeEligible(99, GetAiclkFmin()));
	zassert_true(ThrottlerTestRuntimePowerFailSafeEligible(100, GetAiclkFmin()));

	/* The measured 306 W P150A escape must trip a 300 W policy; the normal
	 * controller targets 95% below this hard containment boundary.
	 */
	set_dmc_board_power_limit(300);
	zassert_true(ThrottlerTestRuntimePowerFailSafeEligible(306, GetAiclkFmin()));

	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1000));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(set_board_power_limit(100, false), 2);
	zassert_equal(request_full_power_state(), 1);

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_runtime_power_latch_immediately_asserts_asic_risc_resets)
{
	ThrottlerTestResetRuntimePowerGuard();
	RESET_FAKE(WriteReg);

	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 300, 1000));
	zassert_equal(WriteReg_fake.call_count, 8U);
	for (uint32_t i = 0; i < 8U; i++) {
		zassert_equal(WriteReg_fake.arg0_history[i],
			      RESET_UNIT_TENSIX_RISC_RESET_0_REG_ADDR + i * sizeof(uint32_t));
		zassert_equal(WriteReg_fake.arg1_history[i], 0U);
	}

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_input_power_addition_saturates_instead_of_wrapping)
{
	uint8_t power_data[2];

	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	sys_put_le16(UINT16_MAX, power_data);
	zassert_ok(Dm2CmSendPowerHandler(power_data, sizeof(power_data)));
	zassert_equal(GetInputPower(), UINT16_MAX);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT) >> 16U, UINT16_MAX);

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_latched_containment_cannot_release_kernel_nops)
{
	ThrottlerTestResetRuntimePowerGuard();
	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 300, 1000));
	zassert_equal(ThrottlerSetKernelThrottlerEnabled(0), 2);
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_lowering_cap_checks_already_published_peak)
{
	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	ThrottlerTestRecordInputPowerSampleAtPower(1000, 250);
	zassert_false(ThrottlerRuntimePowerFaultLatched());

	/* Tightening the cap must inspect the coherent current/peak snapshot; it
	 * cannot wait for another DMC sample that may already be below the peak.
	 */
	zassert_equal(set_board_power_limit(225, false), 0);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT) >> 16U, 250U);
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_runtime_power_freshness_latches_when_first_sample_is_missing)
{
	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	ThrottlerTestStartRuntimePowerSampleWatchdog(100);
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT) & 0xffU,
		      RUNTIME_POWER_FAULT_STRICT_BIT);

	/* DMC's 1 ms producer starts after its post-reset SMBus initialization,
	 * which may consume an I2C timeout. The startup deadline must not use the
	 * tighter cadence watchdog before any valid sample is accepted.
	 */
	zassert_false(ThrottlerTestRuntimePowerSampleExpired(199));
	zassert_true(ThrottlerTestUpdateRuntimePowerFreshnessGuard(200));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT) & 0xffU,
		      RUNTIME_POWER_FAULT_LATCHED_BIT | RUNTIME_POWER_FAULT_STRICT_BIT);
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_runtime_power_freshness_latches_when_sample_is_stale)
{
	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	ThrottlerTestStartRuntimePowerSampleWatchdog(100);
	/* A valid sample arriving near the end of the post-reset allowance arms
	 * the normal 10 ms stale-sample watchdog immediately.
	 */
	ThrottlerTestRecordInputPowerSampleAtPower(199, 100);
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT) & 0xffU,
		      RUNTIME_POWER_FAULT_STRICT_BIT | RUNTIME_POWER_FAULT_SAMPLE_FRESH_BIT |
			      RUNTIME_POWER_FAULT_POLICY_READY_BIT);

	zassert_false(ThrottlerTestRuntimePowerSampleExpired(208));
	zassert_true(ThrottlerTestUpdateRuntimePowerFreshnessGuard(209));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_runtime_power_freshness_accepts_valid_samples_and_wraps)
{
	uint8_t power_data[2];
	uint32_t now_ms;

	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	now_ms = k_uptime_get_32();
	ThrottlerTestStartRuntimePowerSampleWatchdog(now_ms);
	sys_put_le16(100, power_data);
	PcieTestResetErrorInterrupts();
	zassert_false(PcieTestErrorInterruptsArmed());
	zassert_equal(Dm2CmSendPowerHandler(power_data, 1), -1);
	zassert_false(PcieTestErrorInterruptsArmed());
	zassert_ok(Dm2CmSendPowerHandler(power_data, sizeof(power_data)));
	zassert_true(PcieTestErrorInterruptsArmed());
	zassert_equal(PcieTestErrorInterruptArmCount(), 1);
	zassert_ok(Dm2CmSendPowerHandler(power_data, sizeof(power_data)));
	zassert_equal(PcieTestErrorInterruptArmCount(), 1);
	zassert_false(ThrottlerTestRuntimePowerSampleExpired(k_uptime_get_32()));

	ThrottlerTestStartRuntimePowerSampleWatchdog(100);
	zassert_equal(Dm2CmSendPowerHandler(power_data, 1), -1);
	zassert_false(ThrottlerTestRuntimePowerSampleExpired(199));
	zassert_true(ThrottlerTestRuntimePowerSampleExpired(200));

	ThrottlerTestStartRuntimePowerSampleWatchdog(UINT32_MAX - 5U);
	zassert_false(ThrottlerTestRuntimePowerSampleExpired(93));
	zassert_true(ThrottlerTestUpdateRuntimePowerFreshnessGuard(94));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_pcie_error_interrupts_wait_for_initialized_power_monitor)
{
	uint8_t power_data[2];

	sys_put_le16(100, power_data);
	PcieTestResetErrorInterrupts();
	ThrottlerTestSetRuntimePowerMonitorInitialized(false);
	zassert_ok(Dm2CmSendPowerHandler(power_data, sizeof(power_data)));
	zassert_false(PcieTestErrorInterruptsArmed());

	ThrottlerTestSetRuntimePowerMonitorInitialized(true);
	zassert_ok(Dm2CmSendPowerHandler(power_data, sizeof(power_data)));
	zassert_true(PcieTestErrorInterruptsArmed());
}

ZTEST(throttler, test_runtime_power_guard_stays_latched)
{
	ThrottlerTestResetRuntimePowerGuard();

	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1000));
	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(false, 100, 1001));
	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(true, 170, 1002));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	/* Later sampling must retain the sample that originally latched the fault. */
	ThrottlerTestRecordInputPowerSampleAtPower(1003, 100);
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT) >> 16U, 160U);

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_over_limit_sample_latches_before_dvfs_and_cannot_be_hidden)
{
	uint8_t power_data[2];
	uint16_t trip_power;
	uint16_t later_power;
	union request req = {0};
	struct response rsp = {0};
	bool state;

	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	zassert_equal(request_full_power_state(), 0);

	/* Queue a normal last-close/all-off request ahead of the DVFS worker. The
	 * accepted DMC sample must latch synchronously, so processing this request
	 * cannot remove a target after the limit was crossed.
	 */
	req.power_setting.command_code = TT_SMC_MSG_POWER_SETTING;
	req.power_setting.power_flags_valid = BH_POWER_DOMAIN_COUNT;
	zassert_ok(msgqueue_request_push(0, &req));

	/* The DMC callback can publish multiple samples before the cooperative
	 * DVFS work item runs. The first over-limit sample must latch immediately,
	 * and a later low value must not replace its fault provenance.
	 */
	sys_put_le16(301, power_data);
	zassert_ok(Dm2CmSendPowerHandler(power_data, sizeof(power_data)));
	trip_power = GetInputPower();
	sys_put_le16(100, power_data);
	zassert_ok(Dm2CmSendPowerHandler(power_data, sizeof(power_data)));
	later_power = GetInputPower();

	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_true(later_power < trip_power);
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT) >> 16U, trip_power);
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));
	zassert_equal(rsp.data[0], 1);
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_MRISC, &state));
	zassert_true(state);
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_TENSIX, &state));
	zassert_true(state);
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_L2CPU, &state));
	zassert_true(state);
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_runtime_power_peak_survives_newer_low_sample)
{
	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);

	ThrottlerTestRecordInputPowerSampleAtPower(1000, 280);
	ThrottlerTestRecordInputPowerSampleAtPower(1001, 100);
	zassert_equal(GetInputPower(), 100);
	zassert_equal(ThrottlerTestConsumeRuntimePowerPeak(), 280);
	zassert_equal(ThrottlerTestConsumeRuntimePowerPeak(), 100);
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_exact_sample_publishes_fast_clamp_before_hard_limit)
{
	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);

	ThrottlerTestRecordInputPowerSampleAtPower(1000, 269);
	zassert_false(ThrottlerRuntimePowerClampActive());
	ThrottlerTestRecordInputPowerSampleAtPower(1001, 270);
	zassert_true(ThrottlerRuntimePowerClampActive());
	zassert_false(ThrottlerRuntimePowerFaultLatched());
	CalculateTargAiclk();
	zassert_equal(GetAiclkTarg(), GetAiclkFmin());
	/* A newer low sample cannot erase the clamp until the retained peak was
	 * consumed by DVFS. One later sample below the hysteretic release point
	 * then permits normal arbitration to resume.
	 */
	ThrottlerTestRecordInputPowerSampleAtPower(1002, 100);
	zassert_true(ThrottlerRuntimePowerClampActive());
	zassert_equal(ThrottlerTestConsumeRuntimePowerPeak(), 270);
	zassert_true(ThrottlerRuntimePowerClampActive());
	ThrottlerTestRecordInputPowerSampleAtPower(1003, 100);
	zassert_false(ThrottlerRuntimePowerClampActive());

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_strict_power_idle_keeps_pcie_targets_clocked)
{
	union request req = {0};
	struct response rsp = {0};
	bool state;

	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	zassert_equal(request_full_power_state(), 0);

	req.power_setting.command_code = TT_SMC_MSG_POWER_SETTING;
	req.power_setting.power_flags_valid = BH_POWER_DOMAIN_COUNT;
	/* Every power bit is zero: AICLK may idle, but strict policy must not
	 * remove any PCIe-addressable Tensix, L2CPU, or GDDR/MRISC target.
	 */
	zassert_ok(msgqueue_request_push(0, &req));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));
	zassert_equal(rsp.data[0], 0);

	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_AICLK, &state));
	zassert_false(state);
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_MRISC, &state));
	zassert_true(state);
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_TENSIX, &state));
	zassert_true(state);
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_L2CPU, &state));
	zassert_true(state);

	zassert_equal(request_full_power_state(), 0);
}

ZTEST(throttler, test_runtime_power_controller_uses_instantaneous_and_short_windows)
{
	set_dmc_board_power_limit(300);
	zassert_equal(set_board_power_limit(300, false), 0);
	ThrottlerTestResetBoardPowerHistory(100);

	/* The newest sample wins immediately. The 16-sample window then retains an
	 * excursion after instantaneous power falls instead of releasing on the
	 * legacy one-second average alone.
	 */
	zassert_equal(ThrottlerTestUpdateBoardPowerHistory(400), 400);
	zassert_equal(ThrottlerTestUpdateBoardPowerHistory(100), 118);

	uint16_t control_power = 0;

	for (int i = 0; i < 16; i++) {
		control_power = ThrottlerTestUpdateBoardPowerHistory(400);
	}
	zassert_equal(control_power, 400);
}

ZTEST(throttler, test_strict_policy_rejects_characterization_clock_raises)
{
	union request req = {0};

	set_dmc_board_power_limit(300);
	ThrottlerTestRecordInputPowerSampleAtPower(k_uptime_get_32(), 100);

	req.force_aiclk.command_code = TT_SMC_MSG_FORCE_AICLK;
	req.force_aiclk.forced_freq = GetAiclkFmax();
	zassert_equal(request_aiclk_control(&req), 2);
	/* Production reset/cleanup may use the reset-safe frequency, then release
	 * that force without reopening the normal characterization controls.
	 */
	req.force_aiclk.forced_freq = (uint32_t)AICLK_RESET_SAFE_FREQ;
	zassert_ok(request_aiclk_control(&req));
	zassert_true(GetAiclkTarg() <= (uint32_t)AICLK_RESET_SAFE_FREQ);
	req.force_aiclk.forced_freq = (uint32_t)AICLK_RESET_SAFE_FREQ + 1U;
	zassert_equal(request_aiclk_control(&req), 2);
	req.force_aiclk.forced_freq = 0U;
	zassert_ok(request_aiclk_control(&req));

	req = (union request){0};
	req.force_vdd.command_code = TT_SMC_MSG_FORCE_VDD;
	req.force_vdd.forced_voltage = 800U;
	zassert_equal(request_aiclk_control(&req), 2);

	req = (union request){0};
	req.aisweep.command_code = TT_SMC_MSG_AISWEEP_START;
	req.aisweep.sweep_low = GetAiclkFmin();
	req.aisweep.sweep_high = GetAiclkFmax();
	zassert_equal(request_aiclk_control(&req), 2);

	req = (union request){0};
	req.characterisation_msg.command_code = TT_SMC_MSG_CHARACTERISATION;
	req.characterisation_msg.submsg_ID = TT_SUB_MSG_SET_HOST_REQUESTED_FMIN;
	req.characterisation_msg.submsg_data.fmin_value.value = GetAiclkFmax();
	zassert_equal(request_aiclk_control(&req), 2);

	/* Releasing an old characterization setting remains harmless and allowed. */
	req.characterisation_msg.submsg_data.fmin_value.value = 1;
	zassert_ok(request_aiclk_control(&req));
}

ZTEST_SUITE(throttler, NULL, throttler_setup, NULL, NULL, NULL);
