/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/sys/byteorder.h>

#include "aiclk_ppm.h"
#include "dvfs.h"
#include "telemetry.h"
#include "telemetry_internal.h"
#include "throttler.h"
#include <tenstorrent/msgqueue.h>
#include <tenstorrent/smc_msg.h>
static uint32_t fmax;
static uint32_t fmin;

static void *aiclk_ppm_setup(void)
{
	fmax = GetAiclkFmax();
	fmin = GetAiclkFmin();

	zassert_not_equal(fmin, fmax, "Fmin and Fmax values should not be equal");

	return NULL;
}

static void reset_arb(void *fixture)
{
	(void)fixture;
	dvfs_enabled = false;
	TelemetryInternalData telemetry = {};

	/* Keep every test independent of prior safety and telemetry state. */
	zassert_true(SetAiclkResetSafe(false), "Expected reset-safe state to clear");
	AiclkTestClearCharacterizationOverrides();
	TelemetryInternalTestSetCached(&telemetry);

	/* Reset all arbiter values and disable */
	for (int i = 0; i < aiclk_arb_max_count; i++) {
		SetAiclkArbMax(i, fmax);
		EnableArbMax(i, false);
	}
	for (int i = 0; i < aiclk_arb_min_count; i++) {
		SetAiclkArbMin(i, fmin);
		EnableArbMin(i, false);
	}
}

static void set_busy(bool busy)
{
	union request req = {0};
	struct response rsp = {0};

	req.aiclk_set_speed.command_code =
		busy ? TT_SMC_MSG_AICLK_GO_BUSY : TT_SMC_MSG_AICLK_GO_LONG_IDLE;
	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);
	zexpect_equal(rsp.data[0], 0);
}

static void reinit_arb(void *fixture)
{
	TelemetryInternalData telemetry = {};

	(void)fixture;
	dvfs_enabled = false;
	for (int i = 0; i < aiclk_arb_max_count; i++) {
		SetAiclkArbMax(i, fmax);
		EnableArbMax(i, true);
	}
	for (int i = 0; i < aiclk_arb_min_count; i++) {
		SetAiclkArbMin(i, fmin);
		EnableArbMin(i, true);
	}

	set_busy(false);
	zassert_true(SetAiclkResetSafe(false), "Expected reset-safe state to clear");
	AiclkTestClearCharacterizationOverrides();
	ThrottlerTestResetRuntimePowerState();
	TelemetryInternalTestSetCached(&telemetry);
}

ZTEST(aiclk_ppm, test_power_slew_reaches_max_without_same_tick_bypass)
{
	SetAiclkPowerSlew(true);

	zassert_equal(AiclkTestApplyPowerSlew(800, 1350, 100), 801);
	zassert_equal(AiclkTestApplyPowerSlew(801, 1350, 100), 801);
	zassert_equal(AiclkTestApplyPowerSlew(801, 1350, 101), 802);
	/* Falling clocks are never delayed. */
	zassert_equal(AiclkTestApplyPowerSlew(1000, 800, 101), 800);

	SetAiclkPowerSlew(false);
	zassert_equal(AiclkTestApplyPowerSlew(800, 1350, 102), 1350);
}

ZTEST(aiclk_ppm, test_strict_power_max_wins_over_force_and_host_floor)
{
	uint8_t data[2];
	union request req = {0};
	struct response rsp = {0};
	uint32_t strict_max = fmin + 100U;

	/* This test alone exercises the runtime board-power controller. */
	ThrottlerTestResetRuntimePowerState();
	sys_put_le16(300U, data);
	zassert_ok(Dm2CmSetBoardPowerLimit(data, sizeof(data)));
	ThrottlerTestApplyPendingBoardPowerLimit();
	ThrottlerTestRecordInputPowerSampleAtPower(k_uptime_get_32(), 200U);

	SetAiclkArbMax(aiclk_arb_max_host_fmax, strict_max);
	EnableArbMax(aiclk_arb_max_host_fmax, true);
	req.characterisation_msg.command_code = TT_SMC_MSG_CHARACTERISATION;
	req.characterisation_msg.submsg_ID = TT_SUB_MSG_SET_HOST_REQUESTED_FMIN;
	req.characterisation_msg.submsg_data.fmin_value.value = fmax;
	zassert_ok(msgqueue_request_push(0, &req));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));
	zassert_not_equal(rsp.data[0], 0, "normal runtime must reject characterization overrides");

	dvfs_enabled = true;
	zassert_ok(ForceAiclk(fmax));
	zassert_true(GetAiclkTarg() <= strict_max,
		     "forced/host-min target %u bypassed strict max %u", GetAiclkTarg(),
		     strict_max);
}

ZTEST(aiclk_ppm, test_nonzero_force_is_rejected_without_dvfs)
{
	zassert_equal(ForceAiclk(fmax), 2);
	zassert_ok(ForceAiclk(0));
}

ZTEST(aiclk_ppm, test_no_arb_enabled)
{
	uint32_t targ_freq;

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zassert_equal(
		targ_freq, fmin,
		"Target frequency (%d) should be equal to Fmin (%d) when no arbiters are enabled",
		targ_freq, fmin);
}

ZTEST(aiclk_ppm, test_arb_min_disable_enable)
{
	uint32_t mod_fmin = fmin + 100;
	uint32_t targ_freq;

	/* Increase fmin arbiter */
	/* This should limit target aiclk to modified fmin when arbiter is enabled */
	SetAiclkArbMin(aiclk_arb_min_fmin, mod_fmin);

	EnableArbMin(aiclk_arb_min_fmin, false);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zexpect_equal(targ_freq, fmin,
		      "Target frequency (%d) should be equal to Fmin (%d) when "
		      "Fmin arbiter is disabled",
		      targ_freq, fmin);

	EnableArbMin(aiclk_arb_min_fmin, true);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zassert_equal(targ_freq, mod_fmin,
		      "Target frequency (%d) should be equal to modified Fmin (%d) when "
		      "arbiter enabled",
		      targ_freq, mod_fmin);
}

ZTEST(aiclk_ppm, test_arb_max_disable_enable)
{
	uint32_t mod_fmax = (fmin + fmax) / 2;
	uint32_t targ_freq;

	/* Set busy arbiter (aiclk_arb_min_busy = fmax [1400]) */
	/* Set fmax arbiter to value in between fmin and fmax [800] */
	/* This should limit target aiclk to modified fmax when both arbiters are enabled */
	set_busy(true);
	SetAiclkArbMax(aiclk_arb_max_fmax, mod_fmax);

	EnableArbMin(aiclk_arb_min_busy, false);
	EnableArbMax(aiclk_arb_max_fmax, false);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zexpect_equal(targ_freq, fmin,
		      "Target frequency (%d) should be equal to Fmin (%d) when "
		      "Fmax arbiter and Busy arbiter is disabled",
		      targ_freq, fmin);

	EnableArbMin(aiclk_arb_min_busy, true);
	EnableArbMax(aiclk_arb_max_fmax, true);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zassert_equal(targ_freq, mod_fmax,
		      "Target frequency (%d) should be equal to modified Fmax (%d) when "
		      "arbiter enabled",
		      targ_freq, mod_fmax);
}

ZTEST(aiclk_ppm, test_arb_freq_clamping)
{
	uint32_t targ_freq;

	/* Try setting min arbiter above fmax */
	SetAiclkArbMin(aiclk_arb_min_fmin, fmax + 100);
	EnableArbMin(aiclk_arb_min_fmin, true);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zexpect_true(targ_freq >= fmin && targ_freq <= fmax,
		     "Target frequency (%d) should be clamped within [%d, %d]", targ_freq, fmin,
		     fmax);

	EnableArbMin(aiclk_arb_min_fmin, false);

	/* Try setting max arbiter below fmin */
	SetAiclkArbMax(aiclk_arb_max_fmax, fmin - 100);
	EnableArbMax(aiclk_arb_max_fmax, true);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zassert_true(targ_freq >= fmin && targ_freq <= fmax,
		     "Target frequency (%d) should be clamped within [%d, %d]", targ_freq, fmin,
		     fmax);
}

ZTEST(aiclk_ppm, test_arb_lowest_max)
{
	uint32_t targ_freq;
	uint32_t expected_max;
	enum aiclk_arb_max effective_max_arb;

	/* Set a high min arbiter */

	set_busy(true);
	EnableArbMin(aiclk_arb_min_busy, true);

	/* Set multiple max arbiters to different values */
	SetAiclkArbMax(aiclk_arb_max_fmax, fmax - 100);
	EnableArbMax(aiclk_arb_max_fmax, true);

	SetAiclkArbMax(aiclk_arb_max_tdp, fmax - 200);
	EnableArbMax(aiclk_arb_max_tdp, true);

	SetAiclkArbMax(aiclk_arb_max_thm, fmax - 150);
	EnableArbMax(aiclk_arb_max_thm, true);

	expected_max = fmax - 200;

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zexpect_equal(expected_max, get_aiclk_effective_arb_max(&effective_max_arb));
	zexpect_equal(aiclk_arb_max_tdp, effective_max_arb,
		      "Expected TDP arbiter (200 MHz reduction) to be effective max");
	zassert_equal(targ_freq, expected_max,
		      "Target frequency (%d) should be equal to lowest max arbiter (%d)", targ_freq,
		      expected_max);
}

ZTEST(aiclk_ppm, test_arb_lowest_max_fractional)
{
	uint32_t targ_freq;
	uint32_t expected_max;
	enum aiclk_arb_max effective_max_arb;

	/* Set a high min arbiter */

	set_busy(true);
	EnableArbMin(aiclk_arb_min_busy, true);

	/* Set multiple max arbiters to different values */
	SetAiclkArbMax(aiclk_arb_max_fmax, fmax - 100);
	EnableArbMax(aiclk_arb_max_fmax, true);

	SetAiclkArbMax(aiclk_arb_max_tdp, fmax - 200.1F);
	EnableArbMax(aiclk_arb_max_tdp, true);

	SetAiclkArbMax(aiclk_arb_max_thm, fmax - 150);
	EnableArbMax(aiclk_arb_max_thm, true);

	expected_max = (uint32_t)((float)fmax - 200.1F);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zexpect_equal(expected_max, get_aiclk_effective_arb_max(&effective_max_arb));
	zexpect_equal(aiclk_arb_max_tdp, effective_max_arb,
		      "Expected TDP arbiter (200 MHz reduction) to be effective max");
	zassert_equal(targ_freq, expected_max,
		      "Target frequency (%d) should be equal to lowest max arbiter (%d)", targ_freq,
		      expected_max);
}

ZTEST(aiclk_ppm, test_arb_highest_min)
{
	uint32_t targ_freq;
	uint32_t expected_min;
	enum aiclk_arb_min effective_min_arb;

	/* Set multiple min arbiters to different values */
	SetAiclkArbMin(aiclk_arb_min_fmin, fmin + 100);
	EnableArbMin(aiclk_arb_min_fmin, true);

	SetAiclkArbMin(aiclk_arb_min_busy, fmin + 200);
	EnableArbMin(aiclk_arb_min_busy, true);

	expected_min = fmin + 200;

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zexpect_equal(expected_min, get_aiclk_effective_arb_min(&effective_min_arb));
	zexpect_equal(aiclk_arb_min_busy, effective_min_arb,
		      "Expected busy arbiter (200 MHz increase) to be effective min");
	zassert_equal(targ_freq, expected_min,
		      "Target frequency (%d) should be equal to highest min arbiter (%d)",
		      targ_freq, expected_min);
}

ZTEST(aiclk_ppm, test_arb_highest_min_fractional)
{
	uint32_t targ_freq;
	uint32_t expected_min;
	enum aiclk_arb_min effective_min_arb;

	/* Set multiple min arbiters to different values */
	SetAiclkArbMin(aiclk_arb_min_fmin, fmin + 100);
	EnableArbMin(aiclk_arb_min_fmin, true);

	SetAiclkArbMin(aiclk_arb_min_busy, fmin + 200.1F);
	EnableArbMin(aiclk_arb_min_busy, true);

	expected_min = (uint32_t)((float)fmin + 200.1F);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zexpect_equal(expected_min, get_aiclk_effective_arb_min(&effective_min_arb));
	zexpect_equal(aiclk_arb_min_busy, effective_min_arb,
		      "Expected busy arbiter (200 MHz increase) to be effective min");
	zassert_equal(targ_freq, expected_min,
		      "Target frequency (%d) should be equal to highest min arbiter (%d)",
		      targ_freq, expected_min);
}
ZTEST(aiclk_ppm, test_max_arb_less_than_fmin)
{
	uint32_t targ_freq;

	/* Set fmax arbiter below fmin */
	SetAiclkArbMax(aiclk_arb_max_fmax, fmin - 100);
	EnableArbMax(aiclk_arb_max_fmax, true);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zassert_equal(
		targ_freq, fmin,
		"Target frequency (%d) should be equal to Fmin (%d) when max arbiter is below Fmin",
		targ_freq, fmin);
}

ZTEST(aiclk_ppm, test_min_arb_greater_than_max_arb)
{
	uint32_t targ_freq;
	uint32_t min_arb_value = fmax - 100;
	uint32_t max_arb_value = fmin + 100;

	/* Set fmin arbiter above fmax arbiter */
	SetAiclkArbMin(aiclk_arb_min_fmin, min_arb_value);
	EnableArbMin(aiclk_arb_min_fmin, true);

	SetAiclkArbMax(aiclk_arb_max_fmax, max_arb_value);
	EnableArbMax(aiclk_arb_max_fmax, true);

	CalculateTargAiclk();
	targ_freq = GetAiclkTarg();

	zassert_equal(targ_freq, max_arb_value,
		      "Target frequency (%d) should be equal to max arbiter value (%d) when min "
		      "arbiter is above max arbiter",
		      targ_freq, fmin);
}

ZTEST(aiclk_ppm, test_enabled_arb_min_bitmask)
{
	uint32_t bitmask;

	/* Initially all arbiters should be disabled (reset_arb) */
	bitmask = get_enabled_arb_min_bitmask();
	zassert_equal(bitmask, 0, "Bitmask should be 0 when all min arbiters are disabled");

	/* Enable aiclk_arb_min_fmin (bit 0) */
	EnableArbMin(aiclk_arb_min_fmin, true);
	bitmask = get_enabled_arb_min_bitmask();
	zassert_equal(bitmask, (1 << aiclk_arb_min_fmin),
		      "Bitmask should have bit %d set when aiclk_arb_min_fmin is enabled",
		      aiclk_arb_min_fmin);

	/* Enable aiclk_arb_min_busy (bit 1) as well */
	EnableArbMin(aiclk_arb_min_busy, true);
	bitmask = get_enabled_arb_min_bitmask();
	zassert_equal(bitmask, (1 << aiclk_arb_min_fmin) | (1 << aiclk_arb_min_busy),
		      "Bitmask should have bits %d and %d set when both arbiters are enabled",
		      aiclk_arb_min_fmin, aiclk_arb_min_busy);

	/* Disable aiclk_arb_min_fmin, only aiclk_arb_min_busy should be set */
	EnableArbMin(aiclk_arb_min_fmin, false);
	bitmask = get_enabled_arb_min_bitmask();
	zassert_equal(bitmask, (1 << aiclk_arb_min_busy),
		      "Bitmask should have only bit %d set when only aiclk_arb_min_busy is enabled",
		      aiclk_arb_min_busy);

	/* Enable all min arbiters */
	for (int i = 0; i < aiclk_arb_min_count; i++) {
		EnableArbMin(i, true);
	}
	bitmask = get_enabled_arb_min_bitmask();
	uint32_t expected_all = (1 << aiclk_arb_min_count) - 1;

	zassert_equal(
		bitmask, expected_all,
		"Bitmask (0x%x) should have all %d bits set (0x%x) when all arbiters are enabled",
		bitmask, aiclk_arb_min_count, expected_all);
}

ZTEST(aiclk_ppm, test_enabled_arb_max_bitmask)
{
	uint32_t bitmask;

	/* Initially all arbiters should be disabled (reset_arb) */
	bitmask = get_enabled_arb_max_bitmask();
	zassert_equal(bitmask, 0, "Bitmask should be 0 when all max arbiters are disabled");

	/* Enable aiclk_arb_max_fmax (bit 0) */
	EnableArbMax(aiclk_arb_max_fmax, true);
	bitmask = get_enabled_arb_max_bitmask();
	zassert_equal(bitmask, (1 << aiclk_arb_max_fmax),
		      "Bitmask should have bit %d set when aiclk_arb_max_fmax is enabled",
		      aiclk_arb_max_fmax);

	/* Enable aiclk_arb_max_tdp and aiclk_arb_max_thm as well */
	EnableArbMax(aiclk_arb_max_tdp, true);
	EnableArbMax(aiclk_arb_max_thm, true);
	bitmask = get_enabled_arb_max_bitmask();
	uint32_t expected =
		(1 << aiclk_arb_max_fmax) | (1 << aiclk_arb_max_tdp) | (1 << aiclk_arb_max_thm);
	zassert_equal(bitmask, expected,
		      "Bitmask (0x%x) should have bits %d, %d, and %d set (0x%x)", bitmask,
		      aiclk_arb_max_fmax, aiclk_arb_max_tdp, aiclk_arb_max_thm, expected);

	/* Disable aiclk_arb_max_tdp */
	EnableArbMax(aiclk_arb_max_tdp, false);
	bitmask = get_enabled_arb_max_bitmask();
	expected = (1 << aiclk_arb_max_fmax) | (1 << aiclk_arb_max_thm);
	zassert_equal(
		bitmask, expected,
		"Bitmask (0x%x) should have only bits %d and %d set (0x%x) after disabling TDP",
		bitmask, aiclk_arb_max_fmax, aiclk_arb_max_thm, expected);

	/* Enable all max arbiters */
	for (int i = 0; i < aiclk_arb_max_count; i++) {
		EnableArbMax(i, true);
	}
	bitmask = get_enabled_arb_max_bitmask();
	uint32_t expected_all = (1 << aiclk_arb_max_count) - 1;

	zassert_equal(
		bitmask, expected_all,
		"Bitmask (0x%x) should have all %d bits set (0x%x) when all arbiters are enabled",
		bitmask, aiclk_arb_max_count, expected_all);
}

ZTEST(aiclk_ppm, test_arb_bitmask_independent)
{
	uint32_t min_bitmask, max_bitmask;

	/* Enable some min and max arbiters independently and verify they don't interfere */
	EnableArbMin(aiclk_arb_min_fmin, true);
	EnableArbMax(aiclk_arb_max_tdp, true);
	EnableArbMax(aiclk_arb_max_thm, true);

	min_bitmask = get_enabled_arb_min_bitmask();
	max_bitmask = get_enabled_arb_max_bitmask();

	zassert_equal(min_bitmask, (1 << aiclk_arb_min_fmin),
		      "Min bitmask should only reflect min arbiters");
	zassert_equal(max_bitmask, (1 << aiclk_arb_max_tdp) | (1 << aiclk_arb_max_thm),
		      "Max bitmask should only reflect max arbiters");
}

static void send_set_host_fmax(uint32_t freq, uint8_t restore_default, uint8_t *status_out)
{
	union request req = {0};
	struct response rsp = {0};

	req.set_asic_host_fmax.command_code = TT_SMC_MSG_SET_ASIC_HOST_FMAX;
	req.set_asic_host_fmax.asic_fmax = freq;
	req.set_asic_host_fmax.restore_default = restore_default;
	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);
	if (status_out != NULL) {
		*status_out = rsp.data[0];
	}
}

static void send_set_host_fmin(uint32_t freq, uint8_t *status_out)
{
	union request req = {0};
	struct response rsp = {0};

	req.characterisation_msg.command_code = TT_SMC_MSG_CHARACTERISATION;
	req.characterisation_msg.submsg_ID = TT_SUB_MSG_SET_HOST_REQUESTED_FMIN;
	req.characterisation_msg.submsg_data.fmin_value.value = freq;
	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);
	if (status_out != NULL) {
		*status_out = rsp.data[0];
	}
}

ZTEST(aiclk_ppm, test_set_host_fmax_valid)
{
	uint8_t status;
	uint32_t new_fmax = 1000; /* MHz, within [AICLK_FMAX_MIN, AICLK_FMAX_MAX] */
	enum aiclk_arb_max effective_arb;
	uint32_t effective_max;

	send_set_host_fmax(new_fmax, 0, &status);
	zassert_equal(status, 0, "Expected success for valid fmax");

	effective_max = get_aiclk_effective_arb_max(&effective_arb);
	zassert_equal(effective_max, new_fmax,
		      "Effective arb max (%d) should reflect new host fmax (%d)", effective_max,
		      new_fmax);
	zassert_equal(effective_arb, aiclk_arb_max_host_fmax,
		      "Effective arbiter should be aiclk_arb_max_host_fmax");
}

ZTEST(aiclk_ppm, test_set_host_fmax_restore_default)
{
	uint8_t status;
	uint32_t bitmask;

	/* First set a host fmax limit */
	send_set_host_fmax(1000, 0, &status);
	zassert_equal(status, 0);

	/* Then restore to default */
	send_set_host_fmax(0, 1, &status);
	zassert_equal(status, 0, "Expected success for restore_default");

	/* Verify the host fmax arbiter is disabled */
	bitmask = get_enabled_arb_max_bitmask();
	zassert_equal(bitmask & (1 << aiclk_arb_max_host_fmax), 0,
		      "host fmax arbiter should be disabled after restore_default");
}

ZTEST(aiclk_ppm, test_failed_host_fmax_raise_preserves_tighter_ceiling)
{
	uint8_t status;
	enum aiclk_arb_max effective_arb;
	uint32_t effective_max;

	send_set_host_fmax(900U, 0, &status);
	zassert_ok(status);
	dvfs_enabled = true;
	AiclkTestFailDvfsControlChange(true);
	send_set_host_fmax(1200U, 0, &status);
	zassert_not_equal(status, 0, "injected DVFS failure was not returned");

	effective_max = get_aiclk_effective_arb_max(&effective_arb);
	zassert_equal(effective_arb, aiclk_arb_max_host_fmax);
	zassert_equal(effective_max, 900U, "failed raise escaped prior tighter ceiling");
	zassert_equal(GetTelemetryTag(TAG_HOST_AICLK_LIMIT), 900U,
		      "telemetry did not roll back with the ceiling");

	AiclkTestFailDvfsControlChange(false);
	zassert_true(DVFSChange(), "later DVFS pass failed");
	zassert_true(GetAiclkTarg() <= 900U, "later DVFS pass applied rejected raise");
}

ZTEST(aiclk_ppm, test_failed_go_busy_does_not_retry_raise)
{
	union request req = {0};
	struct response rsp = {0};

	set_busy(false);
	dvfs_enabled = true;
	AiclkTestFailDvfsControlChange(true);
	req.aiclk_set_speed.command_code = TT_SMC_MSG_AICLK_GO_BUSY;
	zassert_ok(msgqueue_request_push(0, &req));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));
	zassert_not_equal(rsp.data[0], 0, "injected GO_BUSY failure was not returned");

	AiclkTestFailDvfsControlChange(false);
	zassert_true(DVFSChange(), "later DVFS pass failed");
	zassert_equal(GetAiclkTarg(), fmin, "later DVFS pass retried rejected GO_BUSY raise");
}

ZTEST(aiclk_ppm, test_set_host_fmax_out_of_range_high)
{
	uint8_t status;

	send_set_host_fmax(9999, 0, &status);
	zassert_not_equal(status, 0, "Expected error for out-of-range fmax (too high)");
}

ZTEST(aiclk_ppm, test_set_host_fmax_out_of_range_low)
{
	uint8_t status;

	send_set_host_fmax(100, 0, &status);
	zassert_not_equal(status, 0, "Expected error for out-of-range fmax (too low)");
}

ZTEST(aiclk_ppm, test_set_host_fmax_updates_target_immediately_with_dvfs)
{
	uint8_t status;

	dvfs_enabled = true;
	set_busy(true);
	EnableArbMin(aiclk_arb_min_busy, true);
	send_set_host_fmax(1000, 0, &status);
	zassert_equal(status, 0, "Expected success for valid fmax");
	zassert_equal(GetAiclkTarg(), 1000U,
		      "Expected target AICLK to reflect host fmax immediately");

	send_set_host_fmax(0, 1, &status);
	zassert_equal(status, 0, "Expected success for restore_default");
	zassert_equal(GetAiclkTarg(), fmax,
		      "Expected target AICLK to return to Fmax after restore_default");
}

ZTEST(aiclk_ppm, test_set_host_fmin_updates_target_immediately_with_dvfs)
{
	uint8_t status;

	dvfs_enabled = true;
	set_busy(false);
	send_set_host_fmin(fmax, &status);
	zassert_not_equal(status, 0, "normal runtime must reject host fmin characterization");
	zassert_equal(GetAiclkTarg(), fmin, "rejected host fmin changed target AICLK");

	send_set_host_fmin(1U, &status);
	zassert_not_equal(status, 0, "normal runtime must reject characterization messages");
	zassert_equal(GetAiclkTarg(), fmin,
		      "Expected target AICLK to return to Fmin after restore");
}

ZTEST(aiclk_ppm, test_reset_safe_updates_target_immediately_with_dvfs)
{
	dvfs_enabled = true;
	set_busy(true);
	EnableArbMin(aiclk_arb_min_busy, true);

	zassert_true(SetAiclkResetSafe(true), "Expected reset-safe clamp to succeed");
	zassert_equal(GetAiclkTarg(), (uint32_t)AICLK_RESET_SAFE_FREQ,
		      "Expected reset-safe clamp to apply immediately");

	zassert_true(SetAiclkResetSafe(false), "Expected reset-safe release to succeed");
	zassert_equal(GetAiclkTarg(), fmax, "Expected target AICLK to return to Fmax");
}

ZTEST(aiclk_ppm, test_msg_type_force_aiclk)
{
	union request req = {0};
	struct response rsp = {0};

	dvfs_enabled = true;
	req.force_aiclk.command_code = TT_SMC_MSG_FORCE_AICLK;
	req.force_aiclk.forced_freq = 1000;

	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	zassert_not_equal(rsp.data[0], 0, "raw FORCE_AICLK must be denied");

	/* Disable forcing */
	req = (union request){0};
	rsp = (struct response){0};

	req.force_aiclk.command_code = TT_SMC_MSG_FORCE_AICLK;
	req.force_aiclk.forced_freq = 0;

	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	zassert_not_equal(rsp.data[0], 0, "raw FORCE_AICLK release must be denied");
}

ZTEST(aiclk_ppm, test_msg_type_get_aiclk)
{
	union request req = {0};
	struct response rsp = {0};

	req.get_aiclk.command_code = TT_SMC_MSG_GET_AICLK;

	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	zassert_equal(rsp.data[0], 0);
	/* data[1] = current AICLK, data[2] = clock mode; just verify non-failure */
	zassert_true(rsp.data[2] >= 1 && rsp.data[2] <= 3, "Clock mode (%d) should be 1, 2, or 3",
		     rsp.data[2]);
}

ZTEST(aiclk_ppm, test_msg_type_aisweep_start_stop)
{
	union request req = {0};
	struct response rsp = {0};

	/* Start sweep with valid range */
	req.aisweep.command_code = TT_SMC_MSG_AISWEEP_START;
	req.aisweep.sweep_low = fmin;
	req.aisweep.sweep_high = fmax;

	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	zassert_not_equal(rsp.data[0], 0, "AICLK sweep must be denied in normal runtime");

	/* Stop sweep */
	req = (union request){0};
	rsp = (struct response){0};

	req.aisweep.command_code = TT_SMC_MSG_AISWEEP_STOP;

	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	zassert_not_equal(rsp.data[0], 0, "AICLK sweep must be denied in normal runtime");
}

ZTEST(aiclk_ppm, test_msg_type_aisweep_rejects_empty_intersection)
{
	const struct {
		uint32_t low;
		uint32_t high;
	} invalid_ranges[] = {
		{.low = fmax, .high = fmin},
		{.low = fmax + 1U, .high = UINT32_MAX},
		{.low = 1U, .high = fmin - 1U},
	};

	for (size_t i = 0; i < ARRAY_SIZE(invalid_ranges); i++) {
		union request req = {0};
		struct response rsp = {0};

		req.aisweep.command_code = TT_SMC_MSG_AISWEEP_START;
		req.aisweep.sweep_low = invalid_ranges[i].low;
		req.aisweep.sweep_high = invalid_ranges[i].high;

		zassert_ok(msgqueue_request_push(0, &req));
		process_message_queues();
		zassert_ok(msgqueue_response_pop(0, &rsp));
		zassert_not_equal(rsp.data[0], 0, "Invalid sweep [%u, %u] was accepted",
				  invalid_ranges[i].low, invalid_ranges[i].high);
	}
}

ZTEST_SUITE(aiclk_ppm, NULL, aiclk_ppm_setup, reset_arb, NULL, reinit_arb);
