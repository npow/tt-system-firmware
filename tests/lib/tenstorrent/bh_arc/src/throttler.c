/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <errno.h>

#include "aiclk_ppm.h"
#include "bh_reset.h"
#include "chip_info.h"
#include "cm2dm_msg.h"
#include "dvfs.h"
#include "init.h"
#include "telemetry.h"
#include "telemetry_internal.h"
#include "throttler.h"
#include <tenstorrent/bh_power.h>
#include <tenstorrent/msgqueue.h>
#include <tenstorrent/smc_msg.h>

static float post_enable_idle_vcore_power = 20.0F;
static uint16_t post_enable_idle_board_power = 100U;

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

static int32_t send_dmc_board_power_limit(uint16_t watts)
{
	uint8_t data[2];

	sys_put_le16(watts, data);
	return Dm2CmSetBoardPowerLimit(data, sizeof(data));
}

static void set_dmc_board_power_limit(uint16_t watts)
{
	zassert_ok(send_dmc_board_power_limit(watts));
	ThrottlerTestFlushDmcBoardPowerLimitWork();
}

static void set_input_power(uint16_t watts)
{
	uint8_t data[2];

	sys_put_le16(watts, data);
	zassert_ok(Dm2CmSendPowerHandler(data, sizeof(data)));
}

static void publish_lower_dmc_limit_during_host_request(void)
{
	zassert_ok(send_dmc_board_power_limit(150U));
}

static void publish_post_enable_idle_sample(void)
{
	k_msleep(1);
	TelemetryInternalTestSetVcorePower(post_enable_idle_vcore_power, true);
	set_input_power(post_enable_idle_board_power);
}

static void throttler_before(void *fixture)
{
	ARG_UNUSED(fixture);
	post_enable_idle_vcore_power = 20.0F;
	post_enable_idle_board_power = 100U;

	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestPrepareForInit();
	bh_test_set_boot_cable_power_limit(true, 300U);
	set_input_power(100U);
	TelemetryInternalTestSetVcorePower(20.0F, true);
	InitThrottlers();
	zassert_true(ThrottlerPrepareComputeRelease());
	zassert_ok(bh_reset_safe_aiclk_release());
	ThrottlerTestSetPowerTransitionSampleHook(publish_post_enable_idle_sample);
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

static uint8_t request_aiclk_busy(void)
{
	union request req = {0};
	struct response rsp = {0};

	req.aiclk_set_speed.command_code = TT_SMC_MSG_AICLK_GO_BUSY;
	zassert_ok(msgqueue_request_push(0, &req));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));

	return rsp.data[0];
}

static void set_activity_samples(float vcore_power, bool vcore_valid,
				 uint16_t raw_board_power)
{
	TelemetryInternalTestSetVcorePower(vcore_power, vcore_valid);
	set_input_power(raw_board_power);
}

static void publish_delayed_release_sample(struct k_work *work)
{
	uint8_t data[2];

	ARG_UNUSED(work);
	TelemetryInternalTestSetVcorePower(20.0F, true);
	sys_put_le16(100U, data);
	(void)Dm2CmSendPowerHandler(data, sizeof(data));
}

static K_WORK_DELAYABLE_DEFINE(delayed_release_sample_work,
				       publish_delayed_release_sample);

ZTEST(throttler, test_boot_policy_is_installed_before_dmc_runtime_message)
{
	set_input_power(100U);
	zassert_true(ThrottlerBoardPowerPolicyStrict());
	zassert_true(ThrottlerBoardPowerSampleFresh());
	zassert_true(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300U);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 300U);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 315U);
	/* Releasing the reset-safe clamp runs one DVFS pass, so the closed-loop
	 * controller may already have taken its first bounded step above Fmin.
	 */
	zassert_true(ThrottlerGetDopplerSlowAiclkLimit() >= GetAiclkFmin());
	zassert_true(ThrottlerGetDopplerSlowAiclkLimit() < GetAiclkFmax());
}

ZTEST(throttler, test_early_dmc_policy_survives_throttler_initialization)
{
	ThrottlerTestPrepareForInit();
	bh_test_set_boot_cable_power_limit(true, 300U);
	zassert_ok(send_dmc_board_power_limit(250U));

	InitThrottlers();
	set_input_power(100U);

	zassert_true(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 250U);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 250U);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 262U);
}

ZTEST(throttler, test_runtime_dmc_policy_is_deferred_out_of_smbus_handler)
{
	ThrottlerTestPauseDmcBoardPowerLimitWorker(true);
	zassert_ok(send_dmc_board_power_limit(250U));

	/* The ISR-facing handler only publishes atomics. It must not touch the
	 * installed limit, histories, or arbiters while a DVFS pass may be active.
	 */
	zassert_true(ThrottlerTestDmcBoardPowerLimitPending());
	zassert_false(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300U);
	zassert_equal(ThrottlerTestRuntimePowerGuardLimit(), 250U);

	ThrottlerTestPauseDmcBoardPowerLimitWorker(false);
	ThrottlerTestFlushDmcBoardPowerLimitWork();
	set_input_power(100U);
	zassert_false(ThrottlerTestDmcBoardPowerLimitPending());
	zassert_true(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 250U);
}

ZTEST(throttler, test_lower_dmc_limit_wins_host_request_validation_race)
{
	set_input_power(100U);
	ThrottlerTestPauseDmcBoardPowerLimitWorker(true);
	ThrottlerTestSetHostBoardPowerPreApplyHook(
		publish_lower_dmc_limit_during_host_request);

	/* The synthetic IRQ lands after host validation but before controller
	 * mutation. It must tighten the IRQ guard and invalidate the host request;
	 * the stale 300 W maximum must never be acknowledged as current policy.
	 */
	zassert_equal(set_board_power_limit(300U, false), 2U);
	zassert_true(ThrottlerTestDmcBoardPowerLimitPending());
	zassert_false(ThrottlerBoardPowerPolicyReady());
	zassert_false(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(ThrottlerTestRuntimePowerGuardLimit(), 150U);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300U);

	ThrottlerTestPauseDmcBoardPowerLimitWorker(false);
	ThrottlerTestFlushDmcBoardPowerLimitWork();
	set_input_power(100U);
	zassert_true(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 150U);
}

ZTEST(throttler, test_lower_dmc_limit_latches_against_last_observed_power)
{
	ThrottlerTestPauseDmcBoardPowerLimitWorker(true);
	ThrottlerTestPauseRuntimeContainmentWorker(true);
	set_input_power(200U);
	zassert_false(ThrottlerRuntimePowerFaultLatched());

	/* 200 W was legal under the installed 300 W limit. Publishing a new 150 W
	 * electrical maximum must latch synchronously instead of waiting for the
	 * next power sample or DMC policy worker.
	 */
	zassert_ok(send_dmc_board_power_limit(150U));
	zassert_equal(ThrottlerTestRuntimePowerGuardLimit(), 150U);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_true(ThrottlerTestRuntimeContainmentPending());

	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestPauseRuntimeContainmentWorker(false);
	ThrottlerTestPauseDmcBoardPowerLimitWorker(false);
	ThrottlerTestFlushDmcBoardPowerLimitWork();
}

ZTEST(throttler, test_invalid_dmc_limit_latches_before_deferred_policy_work)
{
	ThrottlerTestPauseDmcBoardPowerLimitWorker(true);
	ThrottlerTestPauseRuntimeContainmentWorker(true);

	zassert_true(send_dmc_board_power_limit(49U) < 0);
	zassert_true(ThrottlerTestDmcBoardPowerLimitPending());
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_true(ThrottlerTestRuntimeContainmentPending());
	zassert_false(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300U);

	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestPauseRuntimeContainmentWorker(false);
	ThrottlerTestPauseDmcBoardPowerLimitWorker(false);
	ThrottlerTestFlushDmcBoardPowerLimitWork();
}

ZTEST(throttler, test_invalid_dmc_limit_before_init_holds_without_false_runtime_latch)
{
	ThrottlerTestPrepareForInit();
	bh_test_set_boot_cable_power_limit(false, 0U);

	/* No workload can exist before throttler initialization. Retain the invalid
	 * publication for the boot policy barrier, but do not fabricate a runtime
	 * trip that would obscure the actual boot-policy failure.
	 */
	zassert_true(send_dmc_board_power_limit(0U) < 0);
	zassert_false(ThrottlerRuntimePowerFaultLatched());
	InitThrottlers();
	zassert_false(ThrottlerRuntimePowerFaultLatched());
	zassert_false(ThrottlerBoardPowerPolicyReady());
	zassert_false(ThrottlerPrepareComputeRelease());
}

ZTEST(throttler, test_throttler_initialization_restores_dmc_policy_idempotently)
{
	set_dmc_board_power_limit(300U);
	zassert_equal(set_board_power_limit(225U, false), 0U);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 225U);

	InitThrottlers();
	set_input_power(100U);

	zassert_true(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300U);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 300U);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 315U);
}

ZTEST(throttler, test_zero_dmc_policy_retains_limit_and_enters_containment)
{
	enum aiclk_arb_max limiting_arbiter;

	zassert_true(send_dmc_board_power_limit(0U) < 0);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	ThrottlerTestFlushDmcBoardPowerLimitWork();
	zassert_true(ThrottlerBoardPowerPolicyStrict());
	zassert_false(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300U);
	zassert_equal(get_aiclk_effective_arb_max(&limiting_arbiter), GetAiclkFmin());
	zassert_equal(limiting_arbiter, aiclk_arb_max_doppler_critical);
}

ZTEST(throttler, test_missing_boot_policy_enters_containment)
{
	enum aiclk_arb_max limiting_arbiter;

	ThrottlerTestPrepareForInit();
	bh_test_set_boot_cable_power_limit(false, 0U);
	InitThrottlers();

	zassert_false(ThrottlerBoardPowerPolicyStrict());
	zassert_false(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 0U);
	zassert_equal(get_aiclk_effective_arb_max(&limiting_arbiter), GetAiclkFmin());
	zassert_equal(limiting_arbiter, aiclk_arb_max_doppler_critical);
}

ZTEST(throttler, test_zero_boot_policy_enters_containment)
{
	ThrottlerTestPrepareForInit();
	bh_test_set_boot_cable_power_limit(true, 0U);
	InitThrottlers();

	zassert_false(ThrottlerBoardPowerPolicyStrict());
	zassert_false(ThrottlerBoardPowerPolicyReady());
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 0U);
}

ZTEST(throttler, test_policy_readiness_requires_a_fresh_power_sample)
{
	set_input_power(100U);
	zassert_true(ThrottlerBoardPowerPolicyReady());
	k_msleep(10);
	zassert_false(ThrottlerBoardPowerSampleFresh());
	zassert_false(ThrottlerBoardPowerPolicyReady());
}

ZTEST(throttler, test_policy_readiness_rejects_any_hardware_init_failure)
{
	uint32_t saved_error_status = error_status0;

	set_input_power(100U);
	zassert_true(ThrottlerBoardPowerPolicyInstalled());
	zassert_true(ThrottlerBoardPowerPolicyReady());

	error_status0 = BIT(0);
	zassert_false(ThrottlerBoardPowerPolicyInstalled());
	zassert_false(ThrottlerBoardPowerPolicyReady());

	error_status0 = saved_error_status;
	zassert_true(ThrottlerBoardPowerPolicyInstalled());
	zassert_true(ThrottlerBoardPowerPolicyReady());
}

ZTEST(throttler, test_reset_safe_aiclk_releases_with_ready_policy)
{
	set_input_power(100U);
	zassert_ok(bh_reset_safe_aiclk_acquire());
	zassert_true(AiclkTestResetSafeEnabled());
	zassert_true(GetAiclkCurrent() <= (uint32_t)AICLK_RESET_SAFE_FREQ);

	set_input_power(100U);
	zassert_ok(bh_reset_safe_aiclk_release());
	zassert_false(AiclkTestResetSafeEnabled());
}

ZTEST(throttler, test_reset_safe_aiclk_is_retained_without_fresh_sample)
{
	zassert_ok(bh_reset_safe_aiclk_acquire());
	k_msleep(10);
	zassert_false(ThrottlerBoardPowerPolicyReady());

	zassert_equal(bh_reset_safe_aiclk_release(), -EPERM);
	zassert_true(AiclkTestResetSafeEnabled());
}

ZTEST(throttler, test_idle_go_busy_remains_at_configured_fmin)
{
	set_activity_samples(20.0F, true, 100U);
	zassert_equal(request_aiclk_busy(), 0U);
	zassert_false(ThrottlerTestRuntimeActivityGateOpen());
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
	CalculateTargAiclk();
	zassert_equal(GetAiclkTarg(), GetAiclkFmin());
}

ZTEST(throttler, test_power_setting_rebaselines_before_releasing_reset_safe_clock)
{
	union request req = {0};
	struct response rsp = {0};

	/* Model clock/domain idle overhead large enough to cross both old +5 W
	 * thresholds. A rising POWER_SETTING must capture it while reset-safe and
	 * must not mistake that overhead for an actual kernel launch.
	 */
	set_activity_samples(26.0F, true, 106U);
	CalculateThrottlers();
	zassert_true(ThrottlerTestRuntimeActivityGateOpen());
	SetAiclkArbMax(aiclk_arb_max_doppler_slow, GetAiclkFmax());
	post_enable_idle_vcore_power = 26.0F;
	post_enable_idle_board_power = 106U;

	req.power_setting.command_code = TT_SMC_MSG_POWER_SETTING;
	req.power_setting.power_flags_valid = BH_POWER_DOMAIN_AICLK + 1U;
	req.power_setting.power_flags_bitfield.max_ai_clk = 1U;
	zassert_ok(msgqueue_request_push(0, &req));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));
	zassert_equal(rsp.data[0], 0U);
	zassert_false(ThrottlerTestRuntimeActivityGateOpen());
	CalculateTargAiclk();
	zassert_equal(GetAiclkTarg(), GetAiclkFmin());

	/* A later, independently sampled rise is the launch signal. It opens the
	 * gate, but normal slew still prevents a jump directly to Fmax.
	 */
	k_msleep(1);
	set_activity_samples(32.0F, true, 112U);
	CalculateThrottlers();
	zassert_true(ThrottlerTestRuntimeActivityGateOpen());
	zassert_ok(DVFSChange());
	zassert_true(GetAiclkTarg() <=
		     GetAiclkCurrent() + AICLK_POWER_SLEW_UP_MHZ_PER_MS);
}

ZTEST(throttler, test_confirmed_activity_permits_only_bounded_clock_rise)
{
	uint32_t current_aiclk;

	set_activity_samples(20.0F, true, 100U);
	zassert_equal(request_aiclk_busy(), 0U);
	set_activity_samples(26.0F, true, 106U);
	CalculateThrottlers();
	zassert_true(ThrottlerTestRuntimeActivityGateOpen());

	/* The derivative term can conservatively hold the first active sample at
	 * Fmin. Sustained confirmed activity must then release, without skipping
	 * either the preload point or the per-millisecond rise bound.
	 */
	for (uint32_t i = 0; i < 2U; i++) {
		k_msleep(2);
		set_activity_samples(26.0F, true, 106U);
		current_aiclk = GetAiclkCurrent();
		zassert_ok(DVFSChange());
		zassert_true(GetAiclkTarg() <=
			     MAX(GetAiclkFmin(),
				 current_aiclk + AICLK_POWER_SLEW_UP_MHZ_PER_MS));
	}
	zassert_true(ThrottlerGetDopplerSlowAiclkLimit() > GetAiclkFmin());
	zassert_true(GetAiclkTarg() > GetAiclkFmin());
}

ZTEST(throttler, test_short_idle_gap_discards_previously_banked_clock)
{
	set_activity_samples(26.0F, true, 106U);
	CalculateThrottlers();
	zassert_true(ThrottlerTestRuntimeActivityGateOpen());

	/* Model a medium workload that had enough time to reach Fmax. The first
	 * observed idle sample must discard that headroom; the 20 ms dwell is for
	 * re-baselining, not a grace period at high clock.
	 */
	SetAiclkArbMax(aiclk_arb_max_doppler_slow, GetAiclkFmax());
	set_activity_samples(20.0F, true, 100U);
	CalculateThrottlers();
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
	CalculateTargAiclk();
	zassert_equal(GetAiclkTarg(), GetAiclkFmin());

	k_msleep(5);
	set_activity_samples(26.0F, true, 106U);
	CalculateThrottlers();
	CalculateTargAiclk();
	zassert_true(ThrottlerTestRuntimeActivityGateOpen());
	zassert_true(ThrottlerGetDopplerSlowAiclkLimit() < GetAiclkFmax());
	zassert_true(GetAiclkTarg() <=
		     GetAiclkCurrent() + AICLK_POWER_SLEW_UP_MHZ_PER_MS);
}

ZTEST(throttler, test_low_power_dwell_rearms_fmin_gate)
{
	set_activity_samples(26.0F, true, 106U);
	CalculateThrottlers();
	zassert_true(ThrottlerTestRuntimeActivityGateOpen());

	set_activity_samples(20.0F, true, 100U);
	CalculateThrottlers();
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
	k_msleep(21);
	set_activity_samples(20.0F, true, 100U);
	CalculateThrottlers();
	zassert_false(ThrottlerTestRuntimeActivityGateOpen());
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
}

ZTEST(throttler, test_invalid_avs_sample_never_releases_preload_gate)
{
	set_activity_samples(1000.0F, false, 150U);
	CalculateThrottlers();
	zassert_false(ThrottlerTestRuntimeActivityGateOpen());
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
}

ZTEST(throttler, test_both_activity_sensors_must_cross_enter_threshold)
{
	/* Board power crosses +5 W, but Vcore remains just below it. */
	set_activity_samples(24.9F, true, 106U);
	CalculateThrottlers();
	zassert_false(ThrottlerTestRuntimeActivityGateOpen());

	/* Vcore crosses +5 W, but board input remains just below it. */
	set_activity_samples(26.0F, true, 104U);
	CalculateThrottlers();
	zassert_false(ThrottlerTestRuntimeActivityGateOpen());
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
}

ZTEST(throttler, test_later_peripheral_power_does_not_replace_boot_idle_baseline)
{
	/* A board-only rise can occur as peripherals finish initialization. It must
	 * neither release the gate nor raise the minimum retained baseline.
	 */
	set_activity_samples(20.0F, true, 150U);
	CalculateThrottlers();
	zassert_false(ThrottlerTestRuntimeActivityGateOpen());
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());

	set_activity_samples(26.0F, true, 106U);
	CalculateThrottlers();
	zassert_true(ThrottlerTestRuntimeActivityGateOpen());
}

ZTEST(throttler, test_missing_release_baseline_is_never_learned_from_later_workload)
{
	ThrottlerTestPrepareForInit();
	bh_test_set_boot_cable_power_limit(true, 300U);
	set_input_power(100U);
	TelemetryInternalTestSetVcorePower(20.0F, false);
	InitThrottlers();
	zassert_false(ThrottlerPrepareComputeRelease());
	zassert_false(ThrottlerTestRuntimeActivityBaselineValid());
	zassert_false(ThrottlerComputePowerPolicyReady());

	/* Once the release barrier has failed, apparent activity is never accepted
	 * as a new idle baseline and cannot open the clock gate.
	 */
	set_activity_samples(40.0F, true, 150U);
	CalculateThrottlers();
	zassert_false(ThrottlerTestRuntimeActivityGateOpen());
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
}

ZTEST(throttler, test_release_barrier_accepts_delayed_coherent_idle_sample)
{
	ThrottlerTestPrepareForInit();
	bh_test_set_boot_cable_power_limit(true, 300U);
	set_input_power(100U);
	TelemetryInternalTestSetVcorePower(20.0F, false);
	InitThrottlers();

	zassert_true(k_work_schedule(&delayed_release_sample_work, K_MSEC(1)) >= 0);
	zassert_true(ThrottlerPrepareComputeRelease());
	zassert_true(ThrottlerTestRuntimeActivityBaselineValid());
	zassert_true(ThrottlerComputePowerPolicyReady());
}

ZTEST(throttler, test_policy_not_required_board_does_not_wait_for_dmc_sample)
{
	uint32_t enabled_arbiters;

	ThrottlerTestSetBoardPowerPolicyRequired(false);
	enabled_arbiters = get_enabled_arb_max_bitmask();
	zassert_true((enabled_arbiters & BIT(aiclk_arb_max_tdp)) != 0U);
	zassert_true((enabled_arbiters & BIT(aiclk_arb_max_fast_tdc)) != 0U);
	zassert_true((enabled_arbiters & BIT(aiclk_arb_max_tdc)) != 0U);
	zassert_false((enabled_arbiters & BIT(aiclk_arb_max_doppler_slow)) != 0U);
	k_msleep(10);
	zassert_false(ThrottlerBoardPowerSampleFresh());
	zassert_true(ThrottlerPrepareComputeRelease());
	zassert_true(ThrottlerComputePowerPolicyReady());
}

ZTEST(throttler, test_release_initialization_activity_is_clamped_before_later_launch)
{
	/* RISC release or UMD open/reset traffic may look active. If it later
	 * quiesces, the first observed low sample must discard all accumulated
	 * clock before a subsequent workload can start.
	 */
	set_activity_samples(30.0F, true, 120U);
	CalculateThrottlers();
	zassert_true(ThrottlerTestRuntimeActivityGateOpen());
	SetAiclkArbMax(aiclk_arb_max_doppler_slow, GetAiclkFmax());

	set_activity_samples(20.0F, true, 100U);
	CalculateThrottlers();
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
	k_msleep(21);
	set_activity_samples(20.0F, true, 100U);
	CalculateThrottlers();
	zassert_false(ThrottlerTestRuntimeActivityGateOpen());

	set_activity_samples(30.0F, true, 120U);
	CalculateThrottlers();
	CalculateTargAiclk();
	zassert_true(GetAiclkTarg() <=
		     GetAiclkCurrent() + AICLK_POWER_SLEW_UP_MHZ_PER_MS);
}

ZTEST(throttler, test_medium_to_heavy_step_latches_on_first_observable_sample)
{
	uint16_t limit = ThrottlerTestRuntimePowerGuardLimit();
	uint16_t additional_power = bh_chip_info_additional_board_power();

	set_activity_samples(40.0F, true, 150U);
	CalculateThrottlers();
	zassert_true(ThrottlerTestRuntimeActivityGateOpen());
	SetAiclkArbMax(aiclk_arb_max_doppler_slow, GetAiclkFmax());

	/* Firmware cannot see the interval before this 1 ms-class sample, but the
	 * first observable at-limit sample must synchronously hold every Tensix RISC.
	 */
	zassert_true(limit > additional_power);
	set_input_power(limit - additional_power);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_set_board_power_limit)
{
	set_dmc_board_power_limit(300);

	zassert_equal(set_board_power_limit(225, false), 0);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 225);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 225);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 236);
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
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 300);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 315);
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());
}

ZTEST(throttler, test_repeated_dmc_policy_does_not_erase_host_override)
{
	set_dmc_board_power_limit(300U);
	zassert_equal(set_board_power_limit(225U, false), 0U);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 225U);

	/* Re-announcing or temporarily tightening the cable maximum must preserve
	 * the independent host safety constraint until an explicit restore/reset.
	 */
	set_dmc_board_power_limit(300U);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 225U);
	set_dmc_board_power_limit(200U);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 200U);
	set_dmc_board_power_limit(300U);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 225U);

	zassert_equal(set_board_power_limit(0U, true), 0U);
	zassert_equal(GetTelemetryTag(TAG_BOARD_POWER_LIMIT), 300U);
}

ZTEST(throttler, test_dmc_board_power_limit_is_strict_after_reset)
{
	set_dmc_board_power_limit(300);

	/* This is the post-reset path. It must enforce the same electrical policy
	 * as an explicit host request, even before systemd or tt-smi runs.
	 */
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 300);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 315);
	zassert_equal(ThrottlerGetDopplerSlowAiclkLimit(), GetAiclkFmin());

	zassert_equal(set_board_power_limit(150, false), 0);
	zassert_equal(ThrottlerGetDopplerT2PowerLimit(), 150);
	zassert_equal(ThrottlerGetDopplerT3PowerLimit(), 157);
}

ZTEST(throttler, test_runtime_power_guard_trips_on_first_emergency_sample)
{
	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	set_input_power(1U);
	zassert_equal(set_board_power_limit(100, false), 0);
	zassert_equal(ThrottlerGetRuntimePowerFailSafeLimit(), 100);

	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(false, 100, 1000));
	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1001));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT), (160U << 16U) | 0xFU);
	zassert_equal(set_board_power_limit(100, false), 0xFFU);
	zassert_equal(request_full_power_state(), 0xFFU);

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_input_power_irq_latches_at_effective_limit)
{
	uint16_t effective_limit = ThrottlerTestRuntimePowerGuardLimit();
	uint16_t additional_power = bh_chip_info_additional_board_power();

	zassert_true(effective_limit > additional_power);
	ThrottlerTestResetRuntimePowerGuard();
	set_input_power(effective_limit - additional_power - 1U);
	zassert_false(ThrottlerRuntimePowerFaultLatched());

	/* No DVFS pass or system-workqueue flush occurs between this write and the
	 * assertion: the SMBus handler itself must latch the threshold crossing.
	 */
	set_input_power(effective_limit - additional_power);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT) >> 16U, effective_limit);

	/* Repeated over-limit IRQs cannot clear or replace the first trip. */
	set_input_power(effective_limit - additional_power + 1U);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT) >> 16U, effective_limit);
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_stale_input_power_watchdog_latches_and_rearms_after_reset)
{
	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestConfigureRuntimePowerWatchdog(300U, 1000U);
	zassert_false(ThrottlerTestCheckRuntimePowerWatchdog(1005U));
	zassert_false(ThrottlerTestCheckRuntimePowerWatchdog(1025U));
	zassert_true(ThrottlerTestCheckRuntimePowerWatchdog(1026U));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_false(ThrottlerTestCheckRuntimePowerWatchdog(1027U));

	/* The test reset models an ASIC reset. Re-arming starts a new grace window;
	 * a below-limit sample refreshes it, then loss of samples latches again.
	 */
	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestConfigureRuntimePowerWatchdog(300U, 2000U);
	ThrottlerObserveInputPowerFromIsr(299U, 2001U);
	zassert_false(ThrottlerTestCheckRuntimePowerWatchdog(2006U));
	zassert_false(ThrottlerTestCheckRuntimePowerWatchdog(2026U));
	zassert_true(ThrottlerTestCheckRuntimePowerWatchdog(2027U));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_stale_sample_soft_clamps_before_irreversible_latch)
{
	enum aiclk_arb_max limiting_arbiter;
	uint32_t nop_starts = GetStartNOPCount();

	/* Remove the normal startup floor so this test identifies the independent
	 * stale-sample clamp, then age the sample past the 5 ms operational window.
	 */
	SetAiclkArbMax(aiclk_arb_max_doppler_slow, GetAiclkFmax());
	k_msleep(10);
	zassert_false(ThrottlerBoardPowerSampleFresh());
	zassert_false(ThrottlerRuntimePowerFaultLatched());

	CalculateThrottlers();
	zassert_equal(get_aiclk_effective_arb_max(&limiting_arbiter), GetAiclkFmin());
	zassert_equal(limiting_arbiter, aiclk_arb_max_doppler_critical);
	zassert_true(GetStartNOPCount() > nop_starts);
	zassert_false(ThrottlerRuntimePowerFaultLatched());
}

ZTEST(throttler, test_runtime_power_guard_includes_exact_emergency_limit)
{
	ThrottlerTestResetRuntimePowerGuard();
	set_dmc_board_power_limit(300);
	set_input_power(1U);
	zassert_equal(set_board_power_limit(100, false), 0);
	zassert_equal(ThrottlerGetRuntimePowerFailSafeLimit(), 100);

	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 100, 1000));
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT), (100U << 16U) | 0xFU);

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_runtime_power_guard_stays_latched)
{
	ThrottlerTestResetRuntimePowerGuard();

	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 160, 1000));
	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(false, 100, 1001));
	zassert_false(ThrottlerTestUpdateRuntimePowerGuard(true, 170, 1002));
	zassert_true(ThrottlerRuntimePowerFaultLatched());

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(throttler, test_runtime_power_controller_uses_instantaneous_and_short_windows)
{
	set_dmc_board_power_limit(300);
	zassert_equal(set_board_power_limit(300, false), 0);
	ThrottlerTestResetBoardPowerHistory(100);

	/* The newest sample wins immediately on a rising edge. The 16-sample window
	 * then retains the excursion after instantaneous power falls again, instead
	 * of letting the legacy 1000-sample average release the clock too early.
	 */
	zassert_equal(ThrottlerTestUpdateBoardPowerHistory(400), 400);
	zassert_equal(ThrottlerTestUpdateBoardPowerHistory(100), 118);

	uint16_t control_power = 0;

	for (int i = 0; i < 16; i++) {
		control_power = ThrottlerTestUpdateBoardPowerHistory(400);
	}
	zassert_equal(control_power, 400);
}

ZTEST(throttler, test_runtime_power_controller_fails_safe_on_stale_or_limit_sample)
{
	set_dmc_board_power_limit(300);

	zassert_true(ThrottlerTestRuntimeBoardPowerCritical(false, 100));
	zassert_false(ThrottlerTestRuntimeBoardPowerCritical(true, 299));
	zassert_true(ThrottlerTestRuntimeBoardPowerCritical(true, 300));
}

ZTEST_SUITE(throttler, NULL, NULL, throttler_before, NULL, NULL);
