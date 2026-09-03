/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>
#include "throttler.h"
#include "aiclk_ppm.h"
#include <tenstorrent/smc_msg.h>
#include <tenstorrent/msgqueue.h>
#include "cm2dm_msg.h"
#include <zephyr/drivers/misc/bh_fwtable.h>
#include <zephyr/tracing/tracing.h>
#include "telemetry_internal.h"
#include "telemetry.h"
#include "noc2axi.h"
#include "tensix_state_msg.h"
#include "bh_reset.h"
#include <tenstorrent/bh_power.h>

static uint32_t power_limit;
static uint32_t max_board_power_limit;
static bool strict_runtime_power_limit;
static atomic_t runtime_power_fault_latched;
static atomic_t runtime_containment_pending;
static atomic_t runtime_power_fault_trip_power;
static atomic_t runtime_power_sample_seen;
static atomic_t runtime_power_sample_timestamp_ms;
static atomic_t runtime_power_sample_watchdog_started_ms;
static struct k_spinlock runtime_power_sample_lock;

static bool doppler;
static bool doppler_slow;
static bool doppler_t2;
static bool doppler_t3;
static const bool thermal_throttling = true;

/*
 * Kernel-throttler-at-AICLK-floor configuration. The defaults are sourced from
 * the firmware table at init (feature_enable.kernel_throttler_at_floor_en and
 * chip_limits.kernel_throttler_stop_nops_freq), so they can be persisted in SPI
 * flash and overridden via bh-mod.
 */
static uint32_t kernel_throttler_stop_nops_freq;
static uint32_t kernel_throttler_stop_nops_freq_default;

#define kThrottlerAiclkScaleFactor             500.0F
#define DEFAULT_BOARD_POWER_LIMIT              150
#define DOPPLER_SHORT_WINDOW_SAMPLES           16U
#define RUNTIME_POWER_CONTROL_HEADROOM_PERCENT 5U
#define RUNTIME_POWER_T2_PERCENT               100U
#define RUNTIME_POWER_T3_PERCENT               105U
/* DMC normally posts INA228 input power every 1 ms. Ten periods leaves room
 * for SMBus/DVFS scheduling jitter while containing a stopped stream quickly.
 */
#define RUNTIME_POWER_SAMPLE_FRESHNESS_MS      10U
/* A fresh SMC must wait for DMC's post-reset SMBus initialization before it
 * can receive the first input-power sample. Allow one 64 ms DMC I2C timeout
 * plus reset/queue jitter; every subsequent sample keeps the 10 ms bound.
 */
#define RUNTIME_POWER_FIRST_SAMPLE_TIMEOUT_MS  100U

LOG_MODULE_REGISTER(throttler);

static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));

typedef enum {
	kThrottlerTDP,
	kThrottlerFastTDC,
	kThrottlerTDC,
	kThrottlerThm,
	kThrottlerBoardPower,
	kThrottlerGDDRThm,
	kThrottlerDopplerSlow,
	kThrottlerCount,
} ThrottlerId;

typedef struct {
	float min;
	float max;
} ThrottlerLimitRange;

/* This table is used to restrict the throttler limits to reasonable ranges. */
/* They are passed in from the FW table in SPI */
/* clang-format off */
static const ThrottlerLimitRange throttler_limit_ranges[kThrottlerCount] = {
	[kThrottlerTDP]		= { .min = 50, .max = 500, },
	[kThrottlerFastTDC]	= { .min = 50, .max = 500, },
	[kThrottlerTDC]		= { .min = 50, .max = 400, },
	[kThrottlerThm]		= { .min = 50, .max = 100, },
	[kThrottlerBoardPower]	= { .min = 50, .max = 600, },
	[kThrottlerGDDRThm]	= { .min = 50, .max = 100, },
	[kThrottlerDopplerSlow]	= { .min = 50, .max = 1200, },
};
/* clang-format on */

typedef struct {
	float alpha_filter;
	float p_gain;
	float d_gain;
} ThrottlerParams;

typedef struct {
	const enum aiclk_arb_max arb_max; /* The arbiter associated with this throttler */

	const ThrottlerParams params;
	float limit;
	float value;
	float error;
	float prev_error;
	float output;
} Throttler;

/* clang-format off */
static Throttler throttler[kThrottlerCount] = {
	[kThrottlerTDP] = {
			.arb_max = aiclk_arb_max_tdp,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.015,
					.d_gain = 0.1,
				},
		},
	[kThrottlerFastTDC] = {
			.arb_max = aiclk_arb_max_fast_tdc,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.5,
					.d_gain = 0,
				},
		},
	[kThrottlerTDC] = {
			.arb_max = aiclk_arb_max_tdc,
			.params = {
					.alpha_filter = 0.1,
					.p_gain = 0.2,
					.d_gain = 0,
				},
		},
	[kThrottlerThm] = {
			.arb_max = aiclk_arb_max_thm,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.2,
					.d_gain = 0,
				},
		},
	[kThrottlerBoardPower] = {
			.arb_max = aiclk_arb_max_board_power,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.1,
					.d_gain = 0.1,
				},
		},
	[kThrottlerGDDRThm] = {
			.arb_max = aiclk_arb_max_gddr_thm,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.2,
					.d_gain = 0,
				},
		},
	[kThrottlerDopplerSlow] = {
			.arb_max = aiclk_arb_max_doppler_slow,
			.params = {
					.alpha_filter = 1.0,
					.p_gain = 0.0025,
					.d_gain = 0.3,
				},
		},
};
/* clang-format on */

static float get_throttler_clamped_limit(ThrottlerId id, float limit)
{
	return CLAMP(limit, throttler_limit_ranges[id].min, throttler_limit_ranges[id].max);
}

static void SetThrottlerLimit(ThrottlerId id, float limit)
{
	float clamped_limit = get_throttler_clamped_limit(id, limit);

	LOG_INF("Throttler %d limit set to %d", id, (uint32_t)clamped_limit);
	throttler[id].limit = clamped_limit;
}

static void ResetBoardPowerHistory(uint16_t current_power);
static void ResetDopplerTransientState(void);
static int InstallDmcBoardPowerLimit(uint16_t cable_power_limit);
static bool RuntimePowerSampleExpired(uint32_t now_ms);
static void UpdateRuntimePowerFaultTelemetry(uint32_t now_ms, uint16_t input_power);

static void StartRuntimePowerSampleWatchdog(uint32_t now_ms)
{
	k_spinlock_key_t key;

	/* Require a sample received after strict policy becomes active. A stale
	 * value from before the DMC limit exchange is not a valid power monitor.
	 */
	key = k_spin_lock(&runtime_power_sample_lock);
	atomic_set(&runtime_power_sample_watchdog_started_ms, now_ms);
	atomic_clear(&runtime_power_sample_seen);
	k_spin_unlock(&runtime_power_sample_lock, key);
	UpdateRuntimePowerFaultTelemetry(now_ms, GetInputPower());
}

void ThrottlerRecordInputPowerSample(uint32_t now_ms)
{
	k_spinlock_key_t key = k_spin_lock(&runtime_power_sample_lock);

	atomic_set(&runtime_power_sample_timestamp_ms, now_ms);
	atomic_set(&runtime_power_sample_seen, 1);
	k_spin_unlock(&runtime_power_sample_lock, key);
	UpdateRuntimePowerFaultTelemetry(now_ms, GetInputPower());
}

static bool RuntimePowerSampleExpired(uint32_t now_ms)
{
	uint32_t reference_ms;
	uint32_t timeout_ms;
	k_spinlock_key_t key;

	if (!strict_runtime_power_limit) {
		return false;
	}

	/* CM2DM message processing and DVFS normally share one workqueue, but use
	 * a lock so a valid sample cannot race the first-sample or steady-state
	 * decision on another context. The snapshot covers the seen flag and its
	 * paired timestamp.
	 */
	key = k_spin_lock(&runtime_power_sample_lock);
	if (atomic_get(&runtime_power_sample_seen)) {
		reference_ms = (uint32_t)atomic_get(&runtime_power_sample_timestamp_ms);
		timeout_ms = RUNTIME_POWER_SAMPLE_FRESHNESS_MS;
	} else {
		reference_ms = (uint32_t)atomic_get(&runtime_power_sample_watchdog_started_ms);
		timeout_ms = RUNTIME_POWER_FIRST_SAMPLE_TIMEOUT_MS;
	}
	k_spin_unlock(&runtime_power_sample_lock, key);

	/* Unsigned subtraction deliberately remains valid over the 32-bit uptime
	 * wrap. A missing first sample expires at its startup timeout; after a
	 * sample arrives, one exactly 10 ms old is no longer fresh.
	 */
	return now_ms - reference_ms >= timeout_ms;
}

static void UpdateRuntimePowerFaultTelemetry(uint32_t now_ms, uint16_t input_power)
{
	uint8_t status = 0U;
	bool latched = atomic_get(&runtime_power_fault_latched);
	bool sample_fresh = strict_runtime_power_limit && atomic_get(&runtime_power_sample_seen) &&
			    !RuntimePowerSampleExpired(now_ms);

	if (latched) {
		status |= RUNTIME_POWER_FAULT_LATCHED_BIT;
		/* The telemetry payload is the original fault sample, not a later
		 * input-power update after domains have been contained.
		 */
		input_power = (uint16_t)atomic_get(&runtime_power_fault_trip_power);
	}
	if (strict_runtime_power_limit) {
		status |= RUNTIME_POWER_FAULT_STRICT_BIT;
	}
	if (sample_fresh) {
		status |= RUNTIME_POWER_FAULT_SAMPLE_FRESH_BIT;
		status |= RUNTIME_POWER_FAULT_POLICY_READY_BIT;
	}

	UpdateTelemetryRuntimePowerFault(status, input_power);
}

static void ConfigureRuntimePowerArbiters(void)
{
	/* Doppler normally replaces the core rail controllers. A strict whole-board
	 * cap keeps the independent, on-chip TDP/TDC paths active as the fast
	 * backstop while the external DMC measurement propagates.
	 */
	EnableArbMax(throttler[kThrottlerTDP].arb_max, !doppler || strict_runtime_power_limit);
	EnableArbMax(throttler[kThrottlerFastTDC].arb_max, !doppler || strict_runtime_power_limit);
	EnableArbMax(throttler[kThrottlerTDC].arb_max, !doppler || strict_runtime_power_limit);
}

static void apply_board_power_limit(uint32_t new_power_limit)
{
	uint32_t controller_limit = new_power_limit;

	if (strict_runtime_power_limit) {
		controller_limit =
			new_power_limit * (100U - RUNTIME_POWER_CONTROL_HEADROOM_PERCENT) / 100U;
	}

	power_limit = new_power_limit;
	SetThrottlerLimit(kThrottlerBoardPower, power_limit);
	SetThrottlerLimit(kThrottlerDopplerSlow, controller_limit);
	ResetBoardPowerHistory(GetInputPower());
	ResetDopplerTransientState();
	ConfigureRuntimePowerArbiters();
	SetAiclkPowerSlew(strict_runtime_power_limit);
	if (strict_runtime_power_limit) {
		/* Host characterization controls can bypass normal min/max
		 * arbitration. The runtime controller still owns the normal 1350 MHz
		 * ceiling, but it must be the path which gets there.
		 */
		ClearAiclkCharacterizationOverrides();
	}
	UpdateTelemetryBoardPowerLimit(power_limit);
}

static uint32_t throttle_counter;
static const uint32_t kKernelThrottleAddress = 0x10;
static bool tensixes_enabled = true;

static uint32_t nop_on_since_ms;      /* uptime when NOP last turned on */
static uint32_t nop_on_accum_ms;      /* total ms NOP's been on until now */
static uint32_t prev_nop_on_accum_ms; /* total ms NOP's been on until prev telemetry update */

static void BroadcastKernelThrottleState(void)
{
	const uint8_t kNocRing = 0;
	const uint8_t kNocTlb = 1;

	if (tensixes_enabled) {
		sys_trace_named_event("kernel_throttle", throttle_counter & 1, 0);
		NOC2AXITensixBroadcastTlbSetup(kNocRing, kNocTlb, kKernelThrottleAddress,
					       kNoc2AxiOrderingStrict);
		NOC2AXIWrite32(kNocRing, kNocTlb, kKernelThrottleAddress, throttle_counter);
	}
}

static void InitKernelThrottling(void)
{
	throttle_counter = 0;
	nop_on_since_ms = 0;
	nop_on_accum_ms = 0;
	prev_nop_on_accum_ms = 0;

	BroadcastKernelThrottleState();
}

/* must only be called when throttle state changes */
static void SendKernelThrottlingMessage(bool throttle)
{
	/* The LLK uses fast = even, slow = odd, but for debug purposes, they'd like to
	 * know how many times throttling has happened. Just in case CMFW somehow gets
	 * out of sync internally, double-check the parity.
	 */
	throttle_counter++;
	if ((throttle_counter & 1) != throttle) {
		throttle_counter++;
	}

	/* Accumulate NOP-on time: stamp the start on the rising edge, bank the
	 * elapsed interval on the falling edge. Centralised here so every edge
	 * (kernel throttler, Doppler, and feature-disable paths) is accounted for.
	 */
	if (throttle) {
		nop_on_since_ms = k_uptime_get_32();
	} else {
		nop_on_accum_ms += k_uptime_get_32() - nop_on_since_ms;
	}

	BroadcastKernelThrottleState();
}

static void doppler_tensix_state_callback(const struct zbus_channel *chan)
{
	const struct tensix_state_msg *msg = zbus_chan_const_msg(chan);

	tensixes_enabled = msg->enable;

	BroadcastKernelThrottleState();
}

ZBUS_LISTENER_DEFINE(doppler_tensix_state_listener, doppler_tensix_state_callback);
ZBUS_CHAN_ADD_OBS(tensix_state_chan, doppler_tensix_state_listener, 0);

void InitThrottlers(void)
{
	uint16_t boot_cable_power_limit;

	doppler = tt_bh_fwtable_get_fw_table(fwtable_dev)->feature_enable.doppler_en;
	doppler_slow = doppler;
	doppler_t2 = doppler;
	doppler_t3 = doppler;
	if (!atomic_get(&runtime_power_fault_latched)) {
		atomic_clear(&runtime_containment_pending);
		atomic_clear(&runtime_power_fault_trip_power);
	}
	{
		k_spinlock_key_t key = k_spin_lock(&runtime_power_sample_lock);

		atomic_clear(&runtime_power_sample_seen);
		atomic_clear(&runtime_power_sample_timestamp_ms);
		atomic_clear(&runtime_power_sample_watchdog_started_ms);
		k_spin_unlock(&runtime_power_sample_lock, key);
	}
	UpdateRuntimePowerFaultTelemetry(k_uptime_get_32(), 0);

	kernel_throttler_stop_nops_freq_default =
		tt_bh_fwtable_get_fw_table(fwtable_dev)
			->chip_limits.kernel_throttler_stop_nops_freq;
	/* A non-zero stop frequency must be within the valid AICLK floor range.
	 * An out-of-range value (e.g. from a board table or ccfgovr override)
	 * could otherwise leave kernel NOPs permanently engaged, so treat it as
	 * 0 (fall back to the effective minimum arbiter frequency at runtime).
	 */
	if (kernel_throttler_stop_nops_freq_default != 0U &&
	    (kernel_throttler_stop_nops_freq_default < (uint32_t)AICLK_FMIN_MIN ||
	     kernel_throttler_stop_nops_freq_default > (uint32_t)AICLK_FMIN_MAX)) {
		LOG_WRN("Invalid fwtable kernel_throttler_stop_nops_freq=%u MHz; using 0 (auto)",
			kernel_throttler_stop_nops_freq_default);
		kernel_throttler_stop_nops_freq_default = 0U;
	}
	kernel_throttler_stop_nops_freq = kernel_throttler_stop_nops_freq_default;
	UpdateTelemetryKernelThrottler(tt_bh_fwtable_get_fw_table(fwtable_dev)
					       ->feature_enable.kernel_throttler_at_floor_en,
				       kernel_throttler_stop_nops_freq);

	SetThrottlerLimit(kThrottlerTDP,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.tdp_limit);
	SetThrottlerLimit(kThrottlerFastTDC,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.tdc_fast_limit);
	SetThrottlerLimit(kThrottlerTDC,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.tdc_limit);
	SetThrottlerLimit(kThrottlerThm,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.thm_limit);
	SetThrottlerLimit(kThrottlerBoardPower, DEFAULT_BOARD_POWER_LIMIT);
	SetThrottlerLimit(kThrottlerGDDRThm,
			  tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.gddr_thm_limit);

	SetThrottlerLimit(kThrottlerDopplerSlow, DEFAULT_BOARD_POWER_LIMIT);

	InitKernelThrottling();

	ConfigureRuntimePowerArbiters();
	EnableArbMax(throttler[kThrottlerBoardPower].arb_max, !doppler);

	EnableArbMax(throttler[kThrottlerThm].arb_max, thermal_throttling);
	EnableArbMax(throttler[kThrottlerGDDRThm].arb_max, thermal_throttling);

	EnableArbMax(throttler[kThrottlerDopplerSlow].arb_max, doppler_slow);

	SetAiclkArbMax(aiclk_arb_max_doppler_critical, GetAiclkFmin());
	EnableArbMax(aiclk_arb_max_doppler_critical, false); /* enabled when limit triggered */

	/* P150A's DMC writes this magic-qualified value before releasing ARC. It
	 * closes the interval between CMFW bring-up and the later DMC SMBus init
	 * message, so policy does not depend on a host-side reapply.
	 */
	if (bh_get_boot_cable_power_limit(&boot_cable_power_limit)) {
		(void)InstallDmcBoardPowerLimit(boot_cable_power_limit);
	}
}

static void UpdateThrottler(ThrottlerId id, float value)
{
	Throttler *t = &throttler[id];

	t->value = t->params.alpha_filter * value + (1 - t->params.alpha_filter) * t->value;
	t->error = (t->limit - t->value) / t->limit;
	t->output = t->params.p_gain * t->error + t->params.d_gain * (t->error - t->prev_error);
	t->prev_error = t->error;
}

static void UpdateThrottlerArb(ThrottlerId id)
{
	Throttler *t = &throttler[id];

	float arb_val = GetThrottlerArbMax(t->arb_max);

	arb_val += t->output * kThrottlerAiclkScaleFactor;

	SetAiclkArbMax(t->arb_max, arb_val);
}

static uint16_t board_power_history[1000];
static uint16_t *board_power_history_cursor = board_power_history;
static uint32_t board_power_sum;
static uint16_t board_power_short_history[DOPPLER_SHORT_WINDOW_SAMPLES];
static uint16_t *board_power_short_history_cursor = board_power_short_history;
static uint32_t board_power_short_sum;
static bool kernel_nops_enabled;

static uint8_t t2_count;
static uint8_t t3_count;

static void ResetDopplerTransientState(void)
{
	t2_count = 0;
	t3_count = 0;
}

#define ADVANCE_CIRCULAR_POINTER(pointer, array)                                                   \
	do {                                                                                       \
		if (++(pointer) == (array) + ARRAY_SIZE(array))                                    \
			(pointer) = (array);                                                       \
	} while (false)

static void ResetBoardPowerHistory(uint16_t current_power)
{
	for (size_t i = 0; i < ARRAY_SIZE(board_power_history); i++) {
		board_power_history[i] = current_power;
	}
	for (size_t i = 0; i < ARRAY_SIZE(board_power_short_history); i++) {
		board_power_short_history[i] = current_power;
	}

	board_power_history_cursor = board_power_history;
	board_power_sum = current_power * ARRAY_SIZE(board_power_history);
	board_power_short_history_cursor = board_power_short_history;
	board_power_short_sum = current_power * ARRAY_SIZE(board_power_short_history);
}

static uint16_t UpdateMovingAveragePower(uint16_t current_power)
{
	board_power_sum += current_power - *board_power_history_cursor;
	*board_power_history_cursor = current_power;

	ADVANCE_CIRCULAR_POINTER(board_power_history_cursor, board_power_history);

	return board_power_sum / ARRAY_SIZE(board_power_history);
}

static uint16_t UpdateShortAveragePower(uint16_t current_power)
{
	board_power_short_sum += current_power - *board_power_short_history_cursor;
	*board_power_short_history_cursor = current_power;

	ADVANCE_CIRCULAR_POINTER(board_power_short_history_cursor, board_power_short_history);

	return board_power_short_sum / ARRAY_SIZE(board_power_short_history);
}

static bool DopplerActive(void)
{
	return doppler && power_limit > 0;
}

static uint32_t GetDopplerT2PowerLimit(void)
{
	return strict_runtime_power_limit ? power_limit * RUNTIME_POWER_T2_PERCENT / 100U
					  : power_limit * 2U;
}

static uint32_t GetDopplerT3PowerLimit(void)
{
	return strict_runtime_power_limit ? power_limit * RUNTIME_POWER_T3_PERCENT / 100U
					  : power_limit * 5U / 2U;
}

static uint32_t GetRuntimePowerFailSafeLimit(void)
{
	/* The closed-loop controller already targets 5% below the configured
	 * electrical cap. A sampled input power at the cap is therefore a failed
	 * containment response, not transient headroom. Latch immediately so a
	 * 300 W P150A policy also contains the observed 306 W escape.
	 */
	return power_limit;
}

static bool LatchRuntimePowerFault(uint16_t current_power)
{
	if (!atomic_cas(&runtime_power_fault_latched, 0, 1)) {
		return false;
	}

	atomic_set(&runtime_power_fault_trip_power, current_power);
	atomic_set(&runtime_containment_pending, 1);
	UpdateRuntimePowerFaultTelemetry(k_uptime_get_32(), current_power);
	return true;
}

static bool ApplyRuntimePowerFault(void)
{
	uint16_t current_power;

	if (!atomic_cas(&runtime_containment_pending, 1, 0)) {
		return false;
	}
	current_power = (uint16_t)atomic_get(&runtime_power_fault_trip_power);
	UpdateRuntimePowerFaultTelemetry(k_uptime_get_32(), current_power);

	LOG_ERR("Runtime board-power fail-safe tripped at %u W (limit %u W)", current_power,
		power_limit);
	if (bh_force_safe_power_state() != 0) {
		LOG_ERR("Failed to enter runtime board-power containment");
	}

	return true;
}

static void UpdateRuntimePowerFailSafe(uint16_t current_power)
{
	if (strict_runtime_power_limit && current_power >= GetRuntimePowerFailSafeLimit()) {
		(void)LatchRuntimePowerFault(current_power);
	}
}

#if defined(CONFIG_ZTEST)
uint32_t ThrottlerGetDopplerT2PowerLimit(void)
{
	return GetDopplerT2PowerLimit();
}

uint32_t ThrottlerGetDopplerT3PowerLimit(void)
{
	return GetDopplerT3PowerLimit();
}

uint32_t ThrottlerGetRuntimePowerFailSafeLimit(void)
{
	return GetRuntimePowerFailSafeLimit();
}

uint32_t ThrottlerGetDopplerSlowAiclkLimit(void)
{
	return GetThrottlerArbMax(throttler[kThrottlerDopplerSlow].arb_max);
}

bool ThrottlerTestRuntimePowerFailSafeEligible(uint16_t current_power, uint32_t aiclk_targ)
{
	ARG_UNUSED(aiclk_targ);
	return strict_runtime_power_limit && current_power >= GetRuntimePowerFailSafeLimit();
}

bool ThrottlerTestUpdateRuntimePowerGuard(bool eligible, uint16_t current_power, uint32_t now_ms)
{
	ARG_UNUSED(now_ms);
	return eligible ? LatchRuntimePowerFault(current_power) : false;
}

void ThrottlerTestResetRuntimePowerGuard(void)
{
	atomic_clear(&runtime_power_fault_latched);
	atomic_clear(&runtime_containment_pending);
	atomic_clear(&runtime_power_fault_trip_power);
	{
		k_spinlock_key_t key = k_spin_lock(&runtime_power_sample_lock);

		atomic_clear(&runtime_power_sample_seen);
		atomic_clear(&runtime_power_sample_timestamp_ms);
		atomic_clear(&runtime_power_sample_watchdog_started_ms);
		k_spin_unlock(&runtime_power_sample_lock, key);
	}
	UpdateRuntimePowerFaultTelemetry(0, 0);
}

void ThrottlerTestStartRuntimePowerSampleWatchdog(uint32_t now_ms)
{
	StartRuntimePowerSampleWatchdog(now_ms);
}

void ThrottlerTestRecordInputPowerSample(uint32_t now_ms)
{
	ThrottlerRecordInputPowerSample(now_ms);
}

bool ThrottlerTestRuntimePowerSampleExpired(uint32_t now_ms)
{
	return RuntimePowerSampleExpired(now_ms);
}

bool ThrottlerTestUpdateRuntimePowerFreshnessGuard(uint32_t now_ms)
{
	if (!RuntimePowerSampleExpired(now_ms)) {
		return false;
	}

	return LatchRuntimePowerFault(GetInputPower());
}

uint16_t ThrottlerTestUpdateBoardPowerHistory(uint16_t current_power)
{
	uint16_t average_power = UpdateMovingAveragePower(current_power);
	uint16_t short_average_power = UpdateShortAveragePower(current_power);

	return strict_runtime_power_limit
		       ? MAX(current_power, MAX(average_power, short_average_power))
		       : average_power;
}

void ThrottlerTestResetBoardPowerHistory(uint16_t current_power)
{
	ResetBoardPowerHistory(current_power);
}
#endif

static void UpdateDoppler(const TelemetryInternalData *telemetry)
{
	uint16_t current_power = GetInputPower();
	uint16_t average_power = UpdateMovingAveragePower(current_power);
	uint16_t short_average_power = UpdateShortAveragePower(current_power);
	uint16_t control_power =
		strict_runtime_power_limit
			? MAX(current_power, MAX(average_power, short_average_power))
			: average_power;

	UpdateThrottler(kThrottlerDopplerSlow, control_power);

	/* Doppler T2 throttler: 10 consecutive samples over its transient limit. */
	uint32_t t2_power_limit = GetDopplerT2PowerLimit();

	if (current_power >= t2_power_limit) {
		if (t2_count < UINT8_MAX) {
			t2_count++;
		}
	} else {
		t2_count = 0;
	}

	uint8_t t2_required_samples = strict_runtime_power_limit ? 1U : 10U;
	bool t2_triggered = t2_count >= t2_required_samples && doppler_t2;

	/* Doppler T3 throttler: 2 consecutive samples over its critical limit. */
	uint32_t t3_power_limit = GetDopplerT3PowerLimit();

	if (current_power >= t3_power_limit) {
		if (t3_count < UINT8_MAX) {
			t3_count++;
		}
	} else {
		t3_count = 0;
	}

	uint8_t t3_required_samples = strict_runtime_power_limit ? 1U : 2U;
	bool t3_triggered = t3_count >= t3_required_samples && doppler_t3;

	/* AICLK=Fmin isn't always enough to get below the board power limit. */
	bool start_nops = GetAiclkTarg() == GetAiclkFmin() && current_power > power_limit;
	uint32_t runtime_release_power =
		power_limit * (100U - RUNTIME_POWER_CONTROL_HEADROOM_PERCENT) / 100U;
	bool stop_nops = strict_runtime_power_limit
				 ? current_power <= runtime_release_power
				 : GetAiclkTarg() == GetAiclkFmax() && current_power < power_limit;

	bool critical_throttling =
		atomic_get(&runtime_power_fault_latched) || t2_triggered || t3_triggered;

	bool new_kernel_nops_enabled =
		((kernel_nops_enabled || start_nops) && !stop_nops) || critical_throttling;

	if (new_kernel_nops_enabled != kernel_nops_enabled) {
		kernel_nops_enabled = new_kernel_nops_enabled;
		SendKernelThrottlingMessage(kernel_nops_enabled);
	}

	EnableArbMax(aiclk_arb_max_doppler_critical, critical_throttling);
	UpdateRuntimePowerFailSafe(current_power);
}

/* Update kernel throttler NOPs state when running at the AICLK floor.
 *
 * This path is enabled when TAG_FW_ACTIVE_CONFIG_0 bit 0
 * (kernel_nops_at_aiclk_fmin) is set. The bit is seeded from the fwtable at
 * telemetry init and may later be changed at runtime via the characterization
 * message path.
 *
 * The stop frequency comes from kernel_throttler_stop_nops_freq. When that
 * value is 0, FW falls back to the effective minimum arbiter frequency.
 */
static void UpdateKernelThrottler(float current_power, float tdp_limit)
{
	telemetry_feature_flags_bits_0_t active_config = GetActiveFeatures();
	bool start_nops = false;
	bool stop_nops = false;
	enum aiclk_arb_min arb;

	if (active_config.kernel_nops_at_aiclk_fmin) {
		start_nops = GetAiclkTarg() == GetAiclkFmin() && current_power > tdp_limit;

		uint32_t stop_freq = kernel_throttler_stop_nops_freq;

		if (stop_freq == 0U) {
			stop_freq = get_aiclk_effective_arb_min(&arb);
		}

		stop_nops = GetAiclkTarg() >= stop_freq && current_power < tdp_limit;
	}

	bool new_kernel_nops_enabled = ((kernel_nops_enabled || start_nops) && !stop_nops);

	if (new_kernel_nops_enabled != kernel_nops_enabled) {
		kernel_nops_enabled = new_kernel_nops_enabled;
		SendKernelThrottlingMessage(kernel_nops_enabled);
	}
}

void CalculateThrottlers(void)
{
	TelemetryInternalData telemetry_internal_data;
	uint32_t now_ms = k_uptime_get_32();

	/* PCIe error IRQs only latch containment. Complete the domain transition in
	 * normal DVFS context, never from the interrupt path.
	 */
	UpdateRuntimePowerFaultTelemetry(now_ms, GetInputPower());
	if (RuntimePowerSampleExpired(now_ms) && LatchRuntimePowerFault(GetInputPower())) {
		LOG_ERR("Runtime board-power sample stale or missing; entering containment");
	}
	if (ApplyRuntimePowerFault()) {
		return;
	}
	ReadTelemetryInternal(1, &telemetry_internal_data);

	if (DopplerActive()) {
		UpdateDoppler(&telemetry_internal_data);
	} else {
		UpdateThrottler(kThrottlerTDP, telemetry_internal_data.vcore_power);
		UpdateThrottler(kThrottlerFastTDC, telemetry_internal_data.vcore_current);
		UpdateThrottler(kThrottlerTDC, telemetry_internal_data.vcore_current);
		UpdateThrottler(kThrottlerBoardPower, GetInputPower());

		float current_power = telemetry_internal_data.vcore_power;
		float tdp_limit = throttler[kThrottlerTDP].limit;

		UpdateKernelThrottler(current_power, tdp_limit);
	}

	UpdateThrottler(kThrottlerThm, telemetry_internal_data.asic_temperature);
	UpdateThrottler(kThrottlerGDDRThm, telemetry_internal_data.gddr_temps.max_temp);

	for (ThrottlerId i = 0; i < kThrottlerCount; i++) {
		UpdateThrottlerArb(i);
	}
	if (ApplyRuntimePowerFault()) {
		return;
	}
}

bool ThrottlerRuntimePowerFaultLatched(void)
{
	return atomic_get(&runtime_power_fault_latched) != 0;
}

bool ThrottlerStrictRuntimePowerLimitActive(void)
{
	return strict_runtime_power_limit;
}

void ThrottlerRequestRuntimeContainment(void)
{
	(void)LatchRuntimePowerFault(0);
}

uint8_t ThrottlerSetKernelThrottlerEnabled(uint32_t enabled)
{
	if (enabled > 1) {
		return 1;
	}
	if (!enabled && ThrottlerRuntimePowerFaultLatched()) {
		/* A latched containment state owns the kernel NOP signal until ASIC
		 * reset. Do not let a characterization request release it early.
		 */
		return 2;
	}

	LOG_INF("kernel throttler at aiclk floor %s", enabled ? "enabled" : "disabled");

	/* Release NOPs immediately if the feature is being disabled while active. */
	if (!enabled && kernel_nops_enabled) {
		kernel_nops_enabled = false;
		SendKernelThrottlingMessage(false);
	}

	UpdateTelemetryKernelThrottler((bool)enabled, kernel_throttler_stop_nops_freq);
	return 0;
}

uint8_t ThrottlerSetKernelThrottlerStopFreq(uint32_t frequency)
{
	/* 0 restores the fwtable-provided default (which may itself be 0, meaning
	 * fall back to the effective minimum arbiter frequency at runtime).
	 */
	if (frequency == 0) {
		telemetry_feature_flags_bits_0_t active_config = GetActiveFeatures();

		kernel_throttler_stop_nops_freq = kernel_throttler_stop_nops_freq_default;
		LOG_INF("kernel throttler stop nops frequency restored to fwtable default %u MHz",
			kernel_throttler_stop_nops_freq);
		UpdateTelemetryKernelThrottler(active_config.kernel_nops_at_aiclk_fmin,
					       kernel_throttler_stop_nops_freq);
		return 0;
	}

	/* Reject if outside valid range [AICLK_FMIN_MIN, AICLK_FMIN_MAX] */
	if (frequency > (uint32_t)AICLK_FMIN_MAX || frequency < (uint32_t)AICLK_FMIN_MIN) {
		return 1;
	}

	telemetry_feature_flags_bits_0_t active_config = GetActiveFeatures();

	kernel_throttler_stop_nops_freq = frequency;
	LOG_INF("kernel throttler stop nops frequency set to %u MHz", frequency);
	UpdateTelemetryKernelThrottler(active_config.kernel_nops_at_aiclk_fmin,
				       kernel_throttler_stop_nops_freq);
	return 0;
}

static int InstallDmcBoardPowerLimit(uint16_t cable_power_limit)
{
	uint32_t firmware_power_limit =
		tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.board_power_limit;
	uint32_t resolved_power_limit;

	/* Sensorless/non-Doppler variants retain their existing board-power
	 * controller behavior. A zero cable value is cable fault information, not
	 * a usable electrical maximum.
	 */
	if (cable_power_limit == 0U || firmware_power_limit == 0U) {
		return -1;
	}

	resolved_power_limit = MIN(cable_power_limit, firmware_power_limit);
	if (get_throttler_clamped_limit(kThrottlerBoardPower, resolved_power_limit) !=
	    resolved_power_limit) {
		return -1;
	}

	bool was_strict = strict_runtime_power_limit;

	max_board_power_limit = resolved_power_limit;
	strict_runtime_power_limit = doppler;
	if (strict_runtime_power_limit && !was_strict) {
		StartRuntimePowerSampleWatchdog(k_uptime_get_32());
	}
	apply_board_power_limit(max_board_power_limit);
	if (strict_runtime_power_limit) {
		SetAiclkArbMax(throttler[kThrottlerDopplerSlow].arb_max, GetAiclkFmin());
	}

	return 0;
}

int32_t Dm2CmSetBoardPowerLimit(const uint8_t *data, uint8_t size)
{
	if (size != 2) {
		return -1;
	}

	uint16_t cable_power_limit = sys_get_le16(data);

	LOG_INF("Cable Power Limit: %u", cable_power_limit);
	return InstallDmcBoardPowerLimit(cable_power_limit);
}

static uint8_t set_board_power_limit_handler(const union request *request,
					     struct response *response)
{
	ARG_UNUSED(response);

	uint32_t new_power_limit = request->set_board_power_limit.restore_default
					   ? max_board_power_limit
					   : request->set_board_power_limit.board_power_limit;

	/* The fail-safe deliberately requires an ASIC reset. Accepting another
	 * limit here would suggest the card is ready to run even though Tensix is
	 * still held reset and clock-gated.
	 */
	if (ThrottlerRuntimePowerFaultLatched()) {
		return 2;
	}

	/* Do not allow the host to exceed the cable/board limit or the controller's
	 * supported range. The DMC initializes max_board_power_limit before the host
	 * can issue runtime requests; a zero maximum therefore remains invalid.
	 */
	if (new_power_limit > max_board_power_limit ||
	    get_throttler_clamped_limit(kThrottlerBoardPower, new_power_limit) != new_power_limit) {
		return 1;
	}

	LOG_INF("Runtime board power limit: %u", new_power_limit);
	/* restore_default removes a host override; it must not weaken the DMC
	 * cable/board electrical limit.
	 */
	bool was_strict = strict_runtime_power_limit;

	strict_runtime_power_limit = doppler && new_power_limit > 0U;
	if (strict_runtime_power_limit && !was_strict) {
		StartRuntimePowerSampleWatchdog(k_uptime_get_32());
	}
	apply_board_power_limit(new_power_limit);

	/* A host-requested limit is a safety constraint. Begin at the AICLK floor
	 * and let the closed-loop controller ramp upward instead of waiting for an
	 * over-limit sample before it reacts.
	 */
	if (strict_runtime_power_limit) {
		SetAiclkArbMax(throttler[kThrottlerDopplerSlow].arb_max, GetAiclkFmin());
	}

	return 0;
}

static uint8_t set_tdp_limit_handler(const union request *request, struct response *response)
{
	float default_tdp_limit = get_throttler_clamped_limit(
		kThrottlerTDP, tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.tdp_limit);
	float max_tdp_limit =
		CLAMP(tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.max_tdp_limit,
		      default_tdp_limit, throttler_limit_ranges[kThrottlerTDP].max);
	float new_tdp_limit;

	if (request->set_tdp_limit.restore_default) {
		new_tdp_limit = default_tdp_limit;
	} else {
		new_tdp_limit = request->set_tdp_limit.tdp_limit;
	}

	/* Return an error if the new TDP limit is outside of the valid range */
	if (new_tdp_limit > max_tdp_limit) {
		return 1;
	} else if (get_throttler_clamped_limit(kThrottlerTDP, new_tdp_limit) != new_tdp_limit) {
		return 1;
	}

	SetThrottlerLimit(kThrottlerTDP, new_tdp_limit);
	UpdateTelemetryTdpLimit(throttler[kThrottlerTDP].limit);

	return 0;
}

uint32_t GetStartNOPCount(void)
{
	/* throttle_counter increments on every throttle-state change.
	 * Need to convert transition count to NOP start count
	 */
	return (throttle_counter + 1) >> 1;
}

uint32_t GetNOPOnAccumulatedTime(void)
{
	/* If NOPs are currently enabled, add time since they were last turned on to
	 * accumulated time. Wraps at ~49.7 days of cumulative NOP-on time; consumers
	 * must difference samples with unsigned (modular) arithmetic.
	 */
	if (kernel_nops_enabled) {
		return nop_on_accum_ms + (k_uptime_get_32() - nop_on_since_ms);
	} else {
		return nop_on_accum_ms;
	}
}

uint32_t GetNOPOnDuration(uint32_t window_ms)
{
	/* NOP-on time accrued since the previous call. Unsigned subtraction stays
	 * correct across accumulator wrap, since one window's delta is tiny relative
	 * to the 32-bit millisecond range.
	 */
	uint32_t accumulated_time = GetNOPOnAccumulatedTime();
	uint32_t duration = accumulated_time - prev_nop_on_accum_ms;

	prev_nop_on_accum_ms = accumulated_time;

	/* On the first call prev_nop_on_accum_ms is still 0 from init, so the delta is
	 * the entire NOP-on time banked since boot rather than a single window. Clamp
	 * that bootstrap sample to the window length. `seeded` makes this one-shot:
	 * later samples are returned unclamped so their running sum stays faithful to
	 * the true cumulative NOP-on time.
	 */
	static bool seeded;

	if (!seeded) {
		seeded = true;
		duration = MIN(duration, window_ms);
	}

	return duration;
}

REGISTER_MESSAGE(TT_SMC_MSG_SET_TDP_LIMIT, set_tdp_limit_handler);
REGISTER_MESSAGE(TT_SMC_MSG_SET_BOARD_POWER_LIMIT, set_board_power_limit_handler);
