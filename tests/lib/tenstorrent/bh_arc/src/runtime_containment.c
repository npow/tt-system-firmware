/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <tenstorrent/bh_power.h>
#include <tenstorrent/msgqueue.h>
#include <tenstorrent/smc_msg.h>

#include "aiclk_ppm.h"
#include "cm2dm_msg.h"
#include "dvfs.h"
#include "init.h"
#include "reg_mock.h"
#include "telemetry.h"
#include "throttler.h"
#include "voltage.h"

static uint8_t send_request(const union request *request)
{
	struct response response = {0};

	zassert_ok(msgqueue_request_push(0, request));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &response));

	return response.data[0];
}

static void refresh_input_power_sample(void)
{
	uint8_t data[2];

	sys_put_le16(100U, data);
	zassert_ok(Dm2CmSendPowerHandler(data, sizeof(data)));
}

static void latch_during_reset_safe_clear(void)
{
	ThrottlerRequestRuntimeContainment();
}

static void latch_during_aiclk_raise_commit(void)
{
	ThrottlerRequestRuntimeContainment();
}

static int latch_during_successful_voltage_raise(uint32_t voltage)
{
	ARG_UNUSED(voltage);
	ThrottlerRequestRuntimeContainment();
	return 0;
}

ZTEST(runtime_containment, test_worker_completes_without_periodic_dvfs)
{
	bool saved_dvfs_enabled = dvfs_enabled;

	ThrottlerTestPrepareForInit();
	dvfs_enabled = false;
	ThrottlerEnableRuntimeContainmentWorker();
	ThrottlerRequestRuntimeContainment();
	ThrottlerTestFlushRuntimeContainmentWork();

	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_false(ThrottlerTestRuntimeContainmentPending());

	dvfs_enabled = saved_dvfs_enabled;
	ThrottlerTestPrepareForInit();
}

ZTEST(runtime_containment, test_first_request_holds_every_tensix_risc_in_reset)
{
	ThrottlerTestResetRuntimePowerGuard();
	RESET_FAKE(WriteReg);

	ThrottlerRequestRuntimeContainment();

	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_true(ThrottlerTestRuntimeContainmentPending());
	zassert_equal(WriteReg_fake.call_count, 8U);
	for (uint32_t i = 0; i < 8U; i++) {
		zexpect_equal(WriteReg_fake.arg0_history[i],
			      RESET_UNIT_TENSIX_RISC_RESET_0_REG_ADDR + i * sizeof(uint32_t));
		zexpect_equal(WriteReg_fake.arg1_history[i], 0U);
	}

	/* Repeated interrupts are idempotent and cannot write a deassert value. */
	ThrottlerRequestRuntimeContainment();
	zassert_equal(WriteReg_fake.call_count, 8U);
	zassert_true(ThrottlerTestRuntimeContainmentPending());
	zassert_true(ThrottlerTestRuntimeBoardPowerCritical(true, 0U));

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(runtime_containment, test_fault_latch_blocks_unsafe_messages_but_allows_power_down)
{
	static const uint8_t unsafe_commands[] = {
		TT_SMC_MSG_SET_VOLTAGE,
		TT_SMC_MSG_SWITCH_CLK_SCHEME,
		TT_SMC_MSG_SWITCH_VOUT_CONTROL,
		TT_SMC_MSG_I2C_MESSAGE,
		TT_SMC_MSG_REINIT_TENSIX,
		TT_SMC_MSG_AISWEEP_START,
		TT_SMC_MSG_FORCE_AICLK,
		TT_SMC_MSG_FORCE_VDD,
		TT_SMC_MSG_AICLK_GO_BUSY,
		TT_SMC_MSG_TOGGLE_SINGLE_TENSIX_RESET,
		TT_SMC_MSG_TOGGLE_TENSIX_RESET,
		TT_SMC_MSG_TOGGLE_ETH_RESET,
		TT_SMC_MSG_TOGGLE_GDDR_RESET,
	};
	union request request = {0};

	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerRequestRuntimeContainment();

	for (size_t i = 0; i < ARRAY_SIZE(unsafe_commands); i++) {
		request = (union request){0};
		request.command_code = unsafe_commands[i];
		zexpect_equal(send_request(&request), 0xFFU, "command %#x was not blocked",
			      unsafe_commands[i]);
	}

	request = (union request){0};
	request.power_setting.command_code = TT_SMC_MSG_POWER_SETTING;
	request.power_setting.power_flags_valid = BH_POWER_DOMAIN_COUNT;
	request.power_setting.power_flags_bitfield.max_ai_clk = 1;
	request.power_setting.power_flags_bitfield.mrisc_phy_power = 1;
	request.power_setting.power_flags_bitfield.tensix_enable = 1;
	request.power_setting.power_flags_bitfield.l2cpu_enable = 1;
	zexpect_equal(send_request(&request), 0xFFU);

	/* An AICLK-idle request is a pure power-down and remains available. */
	request = (union request){0};
	request.power_setting.command_code = TT_SMC_MSG_POWER_SETTING;
	request.power_setting.power_flags_valid = 1;
	zexpect_equal(send_request(&request), 0U);

	request = (union request){0};
	request.get_aiclk.command_code = TT_SMC_MSG_GET_AICLK;
	zexpect_equal(send_request(&request), 0U);

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(runtime_containment, test_reset_and_voltage_overrides_cannot_escape_latch)
{
	VoltageArbiter saved_voltage_arbiter = voltage_arbiter;

	ThrottlerTestResetRuntimePowerGuard();
	LatchAiclkPowerFault();
	voltage_arbiter.vdd_min = 650U;
	voltage_arbiter.forced_voltage = 900U;
	for (VoltageRequestor i = 0; i < VoltageReqCount; i++) {
		voltage_arbiter.req_voltage[i] = 900U;
	}

	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 300U, 1U));
	LatchVoltagePowerFault();
	zassert_true(AiclkTestResetSafeEnabled());
	zassert_not_equal(SetAiclkResetSafe(false), 0);
	zassert_true(AiclkTestResetSafeEnabled());
	zassert_equal(voltage_arbiter.forced_voltage, 0U);
	for (VoltageRequestor i = 0; i < VoltageReqCount; i++) {
		zexpect_equal(voltage_arbiter.req_voltage[i], voltage_arbiter.vdd_min);
	}
	zassert_not_equal(ForceVdd(800U), 0);
	zassert_equal(voltage_arbiter.forced_voltage, 0U);

	ThrottlerTestResetRuntimePowerGuard();
	refresh_input_power_sample();
	zassert_ok(SetAiclkResetSafe(false));
	voltage_arbiter = saved_voltage_arbiter;
}

ZTEST(runtime_containment, test_reset_safe_clear_is_atomic_against_fault_irq)
{
	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestPauseRuntimeContainmentWorker(true);
	LatchAiclkPowerFault();
	zassert_true(AiclkTestResetSafeEnabled());

	AiclkTestSetResetSafePreClearHook(latch_during_reset_safe_clear);
	zassert_equal(SetAiclkResetSafe(false), -EPERM);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_true(AiclkTestResetSafeEnabled());

	ThrottlerTestResetRuntimePowerGuard();
	refresh_input_power_sample();
	zassert_ok(SetAiclkResetSafe(false));
	ThrottlerTestPauseRuntimeContainmentWorker(false);
}

ZTEST(runtime_containment, test_fault_at_aiclk_raise_commit_cannot_program_pll)
{
	uint32_t current_aiclk;

	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestPauseRuntimeContainmentWorker(true);
	AiclkTestSetTarget((uint32_t)AICLK_RESET_SAFE_FREQ);
	zassert_ok(DecreaseAiclk());
	current_aiclk = GetAiclkCurrent();
	zassert_true(current_aiclk <= (uint32_t)AICLK_RESET_SAFE_FREQ);

	zassert_true(current_aiclk < GetAiclkFmax());
	AiclkTestSetTarget(MIN(current_aiclk + 100U, GetAiclkFmax()));
	AiclkTestSetRaisePreCommitHook(latch_during_aiclk_raise_commit);
	zassert_equal(IncreaseAiclk(), -EPERM);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(GetAiclkCurrent(), current_aiclk);

	AiclkTestSetRaisePreCommitHook(NULL);
	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestPauseRuntimeContainmentWorker(false);
}

ZTEST(runtime_containment, test_fault_during_voltage_raise_skips_clock_commit)
{
	VoltageArbiter saved_voltage_arbiter = voltage_arbiter;
	uint32_t current_aiclk = GetAiclkCurrent();

	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestPauseRuntimeContainmentWorker(true);
	voltage_arbiter.vdd_min = 650U;
	voltage_arbiter.vdd_max = 900U;
	voltage_arbiter.curr_voltage = 700U;
	voltage_arbiter.targ_voltage = 800U;
	voltage_arbiter.forced_voltage = 800U;
	for (VoltageRequestor i = 0; i < VoltageReqCount; i++) {
		voltage_arbiter.req_voltage[i] = 800U;
	}

	/* Model an IRQ landing while a successful, preemptible regulator write is
	 * in flight. VoltageChange must report the latch so DVFS returns without
	 * entering IncreaseAiclk(), while retaining the raised-voltage observation
	 * for safe clock-before-voltage containment.
	 */
	VoltageTestSetVcoreHook(latch_during_successful_voltage_raise);
	zassert_equal(VoltageChange(), -EPERM);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_equal(voltage_arbiter.curr_voltage, 800U);
	zassert_equal(voltage_arbiter.forced_voltage, 0U);
	for (VoltageRequestor i = 0; i < VoltageReqCount; i++) {
		zexpect_equal(voltage_arbiter.req_voltage[i], voltage_arbiter.vdd_min);
	}
	zassert_equal(GetAiclkCurrent(), current_aiclk);

	VoltageTestSetVcoreHook(NULL);
	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestPauseRuntimeContainmentWorker(false);
	voltage_arbiter = saved_voltage_arbiter;
}

ZTEST(runtime_containment, test_failed_init_cannot_use_uncontrolled_force_paths)
{
	VoltageArbiter saved_voltage_arbiter = voltage_arbiter;
	bool saved_dvfs_enabled = dvfs_enabled;
	uint32_t saved_aiclk = GetAiclkCurrent();
	uint32_t saved_read_reg_return = ReadReg_fake.return_val;
	STATUS_BOOT_STATUS0_reg_u boot_status0 = {0};
	uint32_t valid_voltage = 800U;

	ThrottlerTestResetRuntimePowerGuard();
	boot_status0.f.hw_init_status = kHwInitError;
	ReadReg_fake.return_val = boot_status0.val;
	dvfs_enabled = false;
	voltage_arbiter.vdd_min = 650U;
	voltage_arbiter.vdd_max = 900U;
	voltage_arbiter.forced_voltage = valid_voltage;

	zassert_not_equal(ForceAiclk(GetAiclkFmax()), 0U);
	zassert_equal(GetAiclkCurrent(), saved_aiclk);
	/* Release sentinels clear software overrides but perform no raw hardware
	 * write when initialization failed.
	 */
	zassert_ok(ForceAiclk(0U));
	zassert_equal(GetAiclkCurrent(), saved_aiclk);
	zassert_not_equal(ForceVdd(valid_voltage), 0U);
	zassert_equal(voltage_arbiter.forced_voltage, 0U);
	zassert_ok(ForceVdd(0U));

	dvfs_enabled = saved_dvfs_enabled;
	voltage_arbiter = saved_voltage_arbiter;
	ReadReg_fake.return_val = saved_read_reg_return;
}

ZTEST(runtime_containment, test_failed_safe_state_remains_pending_and_retries)
{
	ThrottlerTestResetRuntimePowerGuard();
	ThrottlerTestPauseRuntimeContainmentWorker(true);
	bh_test_fail_next_force_safe_power_state();
	ThrottlerRequestRuntimeContainment();
	zassert_true(ThrottlerTestRuntimeContainmentPending());

	ThrottlerTestApplyPendingRuntimeContainment(250U);
	zassert_true(ThrottlerRuntimePowerFaultLatched());
	zassert_true(ThrottlerTestRuntimeContainmentPending());

	/* The one-shot injected failure is gone; the next bounded pass completes
	 * containment and consumes the durable pending bit.
	 */
	ThrottlerTestApplyPendingRuntimeContainment(250U);
	zassert_false(ThrottlerTestRuntimeContainmentPending());
	zassert_true(ThrottlerRuntimePowerFaultLatched());

	ThrottlerTestPauseRuntimeContainmentWorker(false);
	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST(runtime_containment, test_power_down_classifier_rejects_any_rising_flag)
{
	union request request = {0};

	request.power_setting.command_code = TT_SMC_MSG_POWER_SETTING;
	request.power_setting.power_flags_valid = BH_POWER_DOMAIN_COUNT;
	zassert_true(bh_power_setting_is_safe_after_fault(&request.power_setting));

	request.power_setting.power_flags_bitfield.max_ai_clk = 1;
	zassert_false(bh_power_setting_is_safe_after_fault(&request.power_setting));
	request.power_setting.power_flags_bitfield.max_ai_clk = 0;
	request.power_setting.power_flags_bitfield.mrisc_phy_power = 1;
	zassert_false(bh_power_setting_is_safe_after_fault(&request.power_setting));
	request.power_setting.power_flags_bitfield.mrisc_phy_power = 0;
	request.power_setting.power_flags_bitfield.tensix_enable = 1;
	zassert_false(bh_power_setting_is_safe_after_fault(&request.power_setting));
	request.power_setting.power_flags_bitfield.tensix_enable = 0;
	request.power_setting.power_flags_bitfield.l2cpu_enable = 1;
	zassert_false(bh_power_setting_is_safe_after_fault(&request.power_setting));
}

ZTEST(runtime_containment, test_runtime_status_and_fault_fields_do_not_clobber_each_other)
{
	UpdateTelemetryRuntimePowerFault(false, 0U);
	UpdateTelemetryRuntimePowerStatus(true, false, true);
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT), 0xAU);

	UpdateTelemetryRuntimePowerFault(true, 321U);
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT), (321U << 16U) | 0xBU);

	UpdateTelemetryRuntimePowerStatus(false, true, false);
	zassert_equal(GetTelemetryTag(TAG_RUNTIME_POWER_FAULT), (321U << 16U) | 0x5U);

	UpdateTelemetryRuntimePowerFault(false, 0U);
}

ZTEST_SUITE(runtime_containment, NULL, NULL, NULL, NULL, NULL);
