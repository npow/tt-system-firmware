/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>
#include "throttler.h"
#include "aiclk_ppm.h"
#include "dvfs.h"
#include <tenstorrent/smc_msg.h>
#include <tenstorrent/msgqueue.h>
#include "cm2dm_msg.h"
#include "chip_info.h"
#include <zephyr/drivers/misc/bh_fwtable.h>
#include <zephyr/tracing/tracing.h>
#include "telemetry_internal.h"
#include "telemetry.h"
#include "noc2axi.h"
#include "tensix_state_msg.h"
#include "bh_reset.h"
#include "init.h"
#include <tenstorrent/bh_power.h>

static uint32_t power_limit;
static uint32_t max_board_power_limit;
static uint32_t host_board_power_limit_override;
static bool strict_board_power_limit;
static bool board_power_policy_required;
static bool board_power_policy_containment;
static atomic_t board_power_policy_strict;
static atomic_t board_power_policy_installed;
static atomic_t throttlers_initialized;
static atomic_t dmc_board_power_limit_received;
static atomic_t dmc_board_power_limit;
static atomic_t dmc_board_power_limit_generation;
static atomic_t dmc_board_power_limit_pending;
static atomic_t dmc_board_power_limit_applying;
static atomic_t runtime_power_fault_latched;
static atomic_t runtime_containment_pending;
static atomic_t runtime_power_fault_trip_power;
static atomic_t runtime_containment_worker_enabled;
static atomic_t runtime_power_guard_armed;
static atomic_t runtime_power_guard_limit;
static atomic_t runtime_power_guard_arm_ms;
static atomic_t runtime_power_guard_last_sample_ms;
static atomic_t runtime_power_guard_last_power;
static atomic_t runtime_power_guard_sample_seen;
static atomic_t runtime_activity_baseline_valid;
static float runtime_activity_boot_idle_vcore_power;
static uint16_t runtime_activity_boot_idle_board_power;
static float runtime_activity_idle_vcore_power;
static uint16_t runtime_activity_idle_board_power;
static bool runtime_activity_gate_open;
static bool runtime_activity_idle_candidate;
static uint32_t runtime_activity_idle_since_ms;
static bool runtime_activity_transition_sample_pending;
static bool runtime_activity_transition_input_seen;
static bool runtime_activity_transition_vcore_seen;
static uint32_t runtime_activity_transition_input_ms;
static uint32_t runtime_activity_transition_vcore_ms;

static void RuntimeContainmentWorkHandler(struct k_work *work);
static K_WORK_DEFINE(runtime_containment_work, RuntimeContainmentWorkHandler);
static K_WORK_DELAYABLE_DEFINE(runtime_containment_retry_work,
			       RuntimeContainmentWorkHandler);
static void DmcBoardPowerLimitWorkHandler(struct k_work *work);
static K_WORK_DEFINE(dmc_board_power_limit_work, DmcBoardPowerLimitWorkHandler);
static void RuntimePowerSampleWatchdogHandler(struct k_timer *timer);
static K_TIMER_DEFINE(runtime_power_sample_watchdog, RuntimePowerSampleWatchdogHandler, NULL);
#if defined(CONFIG_ZTEST)
static bool dmc_board_power_limit_worker_paused;
static bool runtime_containment_worker_paused;
static void (*host_board_power_pre_apply_hook)(void);
static void (*runtime_activity_transition_sample_hook)(void);
#endif

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
static bool kernel_nops_enabled;
static uint8_t t2_count;
static uint8_t t3_count;

#define kThrottlerAiclkScaleFactor             500.0F
#define DEFAULT_BOARD_POWER_LIMIT              150
#define DOPPLER_SHORT_WINDOW_SAMPLES           16U
#define RUNTIME_POWER_CONTROL_HEADROOM_PERCENT 5U
#define RUNTIME_POWER_T2_PERCENT               100U
#define RUNTIME_POWER_T3_PERCENT               105U
#define RUNTIME_POWER_FAILSAFE_MARGIN_PERCENT  0U
#define INPUT_POWER_FRESHNESS_MAX_AGE_MS       5U
#define RUNTIME_POWER_STALE_LATCH_MS           25U
#define VCORE_ACTIVITY_SAMPLE_MAX_AGE_MS        2U
#define RUNTIME_ACTIVITY_ENTER_DELTA_W          5.0F
#define RUNTIME_ACTIVITY_EXIT_DELTA_W           2.0F
#define RUNTIME_ACTIVITY_IDLE_DWELL_MS          20U
#define RUNTIME_ACTIVITY_BOOT_SAMPLE_WAIT_MS    20U
#define RUNTIME_ACTIVITY_TRANSITION_WAIT_MS     20U
#define BOARD_POWER_CONTROLLER_MIN_W           50U

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
	[kThrottlerBoardPower]	= { .min = BOARD_POWER_CONTROLLER_MIN_W, .max = 600, },
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
static bool BoardPowerPolicyInstalled(void);
static void ArmRuntimePowerGuard(uint16_t effective_limit, uint32_t now_ms);
static void DisarmRuntimePowerGuard(void);

static void CloseRuntimeActivityGate(void)
{
	runtime_activity_gate_open = false;
	runtime_activity_idle_candidate = false;
	runtime_activity_idle_since_ms = 0U;
}

static void RearmRuntimeActivityGate(void)
{
	CloseRuntimeActivityGate();
	runtime_activity_transition_sample_pending = false;
	runtime_activity_transition_input_seen = false;
	runtime_activity_transition_vcore_seen = false;
}

static bool VcoreActivitySampleFresh(const TelemetryInternalData *telemetry,
				     uint32_t now_ms)
{
	return telemetry->vcore_power_valid &&
	       (uint32_t)(now_ms - telemetry->vcore_power_updated_ms) <=
		       VCORE_ACTIVITY_SAMPLE_MAX_AGE_MS;
}

static void ResetRuntimeActivityGateBaseline(void)
{
	RearmRuntimeActivityGate();
	atomic_clear(&runtime_activity_baseline_valid);
	runtime_activity_boot_idle_vcore_power = 0.0F;
	runtime_activity_boot_idle_board_power = 0U;
	runtime_activity_idle_vcore_power = 0.0F;
	runtime_activity_idle_board_power = 0U;
}

static void UpdateRuntimeActivityGate(const TelemetryInternalData *telemetry,
				      bool input_fresh,
				      const InputPowerSample *input_power_sample,
				      uint32_t now_ms)
{
	bool vcore_fresh = VcoreActivitySampleFresh(telemetry, now_ms);
	uint16_t input_power = input_power_sample->power;

	if (!strict_board_power_limit) {
		runtime_activity_gate_open = true;
		return;
	}

	/* Either independent sensor becoming invalid closes the gate immediately.
	 * A failed AVS read returns a 0xffff data sentinel, but validity—not that
	 * numerically huge value—is authoritative here.
	 */
	if (atomic_get(&runtime_activity_baseline_valid) == 0 || !vcore_fresh ||
	    !input_fresh) {
		CloseRuntimeActivityGate();
		return;
	}

	/* A POWER_SETTING transition owns this pending state while reset-safe AICLK
	 * is asserted. It captures the new domain-idle baseline before releasing the
	 * clock; a concurrent DVFS pass must keep the gate closed in the meantime.
	 */
	if (runtime_activity_transition_sample_pending) {
		return;
	}

	if (!runtime_activity_gate_open) {
		/* Retain a minimum, never an average: an early workload cannot pull the
		 * current idle reference upward and hide its own activity.
		 */
		runtime_activity_idle_vcore_power =
			MIN(runtime_activity_idle_vcore_power, telemetry->vcore_power);
		runtime_activity_idle_board_power =
			MIN(runtime_activity_idle_board_power, input_power);

		if (telemetry->vcore_power >= runtime_activity_idle_vcore_power +
						      RUNTIME_ACTIVITY_ENTER_DELTA_W &&
		    (float)input_power >= (float)runtime_activity_idle_board_power +
						     RUNTIME_ACTIVITY_ENTER_DELTA_W) {
			runtime_activity_gate_open = true;
			runtime_activity_idle_candidate = false;
		}
		return;
	}

	if (runtime_activity_idle_candidate) {
		bool activity_confirmed =
			telemetry->vcore_power >= runtime_activity_idle_vcore_power +
						  RUNTIME_ACTIVITY_ENTER_DELTA_W &&
			(float)input_power >= (float)runtime_activity_idle_board_power +
						 RUNTIME_ACTIVITY_ENTER_DELTA_W;

		if (activity_confirmed) {
			runtime_activity_idle_candidate = false;
		} else if ((uint32_t)(now_ms - runtime_activity_idle_since_ms) >=
			   RUNTIME_ACTIVITY_IDLE_DWELL_MS) {
			runtime_activity_idle_vcore_power =
				MIN(runtime_activity_idle_vcore_power, telemetry->vcore_power);
			runtime_activity_idle_board_power =
				MIN(runtime_activity_idle_board_power, input_power);
			RearmRuntimeActivityGate();
		}
	} else if (telemetry->vcore_power <= runtime_activity_idle_vcore_power +
						 RUNTIME_ACTIVITY_EXIT_DELTA_W ||
		   (float)input_power <= (float)runtime_activity_idle_board_power +
						RUNTIME_ACTIVITY_EXIT_DELTA_W) {
		/* Clamp on the first low-activity sample. The dwell confirms idle for
		 * baseline maintenance; it is never a grace period at high clock.
		 */
		runtime_activity_idle_candidate = true;
		runtime_activity_idle_since_ms = now_ms;
	}
}

void ThrottlerBeginHighPowerTransition(void)
{
	TelemetryInternalData telemetry;
	InputPowerSample input_power_sample;

	if (!strict_board_power_limit) {
		return;
	}

	(void)GetInputPowerSample(UINT32_MAX, &input_power_sample);
	ReadTelemetryInternal(INT64_MAX, &telemetry);
	CloseRuntimeActivityGate();
	runtime_activity_transition_sample_pending = true;
	runtime_activity_transition_input_seen = input_power_sample.valid;
	runtime_activity_transition_vcore_seen = telemetry.vcore_power_valid;
	runtime_activity_transition_input_ms = input_power_sample.updated_ms;
	runtime_activity_transition_vcore_ms = telemetry.vcore_power_updated_ms;
	SetAiclkArbMax(throttler[kThrottlerDopplerSlow].arb_max, GetAiclkFmin());
}

bool ThrottlerFinishHighPowerTransition(void)
{
	uint32_t start_ms = k_uptime_get_32();

	if (!strict_board_power_limit) {
		return ThrottlerComputePowerPolicyReady();
	}

	for (;;) {
		TelemetryInternalData telemetry;
		InputPowerSample input_power_sample;
		uint32_t now_ms;
		bool input_fresh;

#if defined(CONFIG_ZTEST)
		if (runtime_activity_transition_sample_hook != NULL) {
			runtime_activity_transition_sample_hook();
		}
#endif
		ReadTelemetryInternal(0, &telemetry);
		now_ms = k_uptime_get_32();
		input_fresh = GetInputPowerSample(INPUT_POWER_FRESHNESS_MAX_AGE_MS,
						 &input_power_sample);
		bool vcore_fresh = VcoreActivitySampleFresh(&telemetry, now_ms);
		bool new_input_sample =
			!runtime_activity_transition_input_seen ||
			input_power_sample.updated_ms != runtime_activity_transition_input_ms;
		bool new_vcore_sample =
			!runtime_activity_transition_vcore_seen ||
			telemetry.vcore_power_updated_ms != runtime_activity_transition_vcore_ms;

		if (input_fresh && vcore_fresh && new_input_sample && new_vcore_sample &&
		    ThrottlerComputePowerPolicyReady()) {
			/* This sample is taken after every requested domain setter has
			 * returned while AICLK is still reset-safe. It is the correct idle
			 * reference for the new clock/domain configuration. The immutable
			 * boot reference remains available for abort/recovery.
			 */
			runtime_activity_idle_vcore_power = telemetry.vcore_power;
			runtime_activity_idle_board_power = input_power_sample.power;
			runtime_activity_transition_sample_pending = false;
			CloseRuntimeActivityGate();
			return true;
		}

		if (ThrottlerRuntimePowerFaultLatched() ||
		    (uint32_t)(k_uptime_get_32() - start_ms) >=
			    RUNTIME_ACTIVITY_TRANSITION_WAIT_MS) {
			return false;
		}
		k_msleep(1);
	}
}

void ThrottlerAbortHighPowerTransition(void)
{
	if (!strict_board_power_limit) {
		return;
	}

	RearmRuntimeActivityGate();
	runtime_activity_idle_vcore_power = runtime_activity_boot_idle_vcore_power;
	runtime_activity_idle_board_power = runtime_activity_boot_idle_board_power;
	SetAiclkArbMax(throttler[kThrottlerDopplerSlow].arb_max, GetAiclkFmin());
}

static void ConfigureRuntimePowerArbiters(void)
{
	/* Doppler normally replaces the core rail controllers. A strict whole-board
	 * policy keeps the independent, on-chip TDP/TDC paths active as a faster
	 * backstop for transients that have not yet propagated through the DMC input
	 * power measurement.
	 */
	bool enable = !doppler || strict_board_power_limit || board_power_policy_containment;

	EnableArbMax(throttler[kThrottlerTDP].arb_max, enable);
	EnableArbMax(throttler[kThrottlerFastTDC].arb_max, enable);
	EnableArbMax(throttler[kThrottlerTDC].arb_max, enable);
}

static void apply_board_power_limit(uint32_t new_power_limit)
{
	uint32_t controller_limit = new_power_limit;

	board_power_policy_containment = false;

	if (strict_board_power_limit) {
		controller_limit =
			new_power_limit * (100U - RUNTIME_POWER_CONTROL_HEADROOM_PERCENT) / 100U;
	}

	power_limit = new_power_limit;
	SetThrottlerLimit(kThrottlerBoardPower, power_limit);
	SetThrottlerLimit(kThrottlerDopplerSlow, controller_limit);
	ResetBoardPowerHistory(GetInputPower());
	ResetDopplerTransientState();
	ConfigureRuntimePowerArbiters();
	EnableArbMax(throttler[kThrottlerDopplerSlow].arb_max,
		     doppler_slow || strict_board_power_limit);
	SetAiclkPowerSlew(strict_board_power_limit);
	RearmRuntimeActivityGate();
	UpdateTelemetryBoardPowerLimit(power_limit);
	atomic_set(&board_power_policy_strict, strict_board_power_limit);
	ArmRuntimePowerGuard((uint16_t)power_limit, k_uptime_get_32());
}

static bool ResolveBoardPowerLimit(uint16_t cable_power_limit, uint32_t *resolved_power_limit)
{
	uint32_t firmware_power_limit =
		tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.board_power_limit;
	uint32_t candidate;

	if (cable_power_limit == 0U || firmware_power_limit == 0U ||
	    resolved_power_limit == NULL) {
		return false;
	}

	candidate = MIN(cable_power_limit, firmware_power_limit);
	if (get_throttler_clamped_limit(kThrottlerBoardPower, candidate) != candidate) {
		return false;
	}

	*resolved_power_limit = candidate;
	return true;
}

static void EnterBoardPowerPolicyContainment(void)
{
	DisarmRuntimePowerGuard();
	atomic_clear(&board_power_policy_installed);
	board_power_policy_containment = true;
	ConfigureRuntimePowerArbiters();
	SetAiclkArbMax(throttler[kThrottlerDopplerSlow].arb_max, GetAiclkFmin());
	SetAiclkArbMax(aiclk_arb_max_doppler_critical, GetAiclkFmin());
	EnableArbMax(aiclk_arb_max_doppler_critical, true);
	SetAiclkPowerSlew(true);
}

static int InstallDmcBoardPowerLimit(uint16_t cable_power_limit)
{
	uint32_t resolved_power_limit;

	LOG_INF("Installing cable power limit: %u W", cable_power_limit);

	if (error_status0 != 0U) {
		LOG_ERR("Refusing board-power policy after hardware initialization failure");
		EnterBoardPowerPolicyContainment();
		return -1;
	}

	if (cable_power_limit == 0U) {
		LOG_ERR("DMC reported a zero-watt cable power limit; retaining containment");
		EnterBoardPowerPolicyContainment();
		return -1;
	}

	if (is_cable_fault_mode()) {
		LOG_ERR("Refusing to leave boot cable-fault containment without a reset");
		EnterBoardPowerPolicyContainment();
		return -1;
	}

	if (!board_power_policy_required) {
		return 0;
	}

	if (!ResolveBoardPowerLimit(cable_power_limit, &resolved_power_limit)) {
		LOG_ERR("No valid firmware/DMC board-power policy; retaining containment");
		EnterBoardPowerPolicyContainment();
		return -1;
	}

	max_board_power_limit = resolved_power_limit;
	strict_board_power_limit = true;
	apply_board_power_limit(host_board_power_limit_override != 0U
					? MIN(resolved_power_limit,
					      host_board_power_limit_override)
					: resolved_power_limit);
	SetAiclkArbMax(throttler[kThrottlerDopplerSlow].arb_max, GetAiclkFmin());
	atomic_set(&board_power_policy_installed, 1);

	return 0;
}

static void ApplyPendingDmcBoardPowerLimit(void)
{
	int install_result;

	/* Consume at most one publication per work invocation. A hostile DMC must
	 * not be able to starve every other user of the system workqueue by keeping
	 * this handler in an unbounded loop.
	 */
	atomic_set(&dmc_board_power_limit_applying, 1);
	if (!atomic_cas(&dmc_board_power_limit_pending, 1, 0)) {
		atomic_clear(&dmc_board_power_limit_applying);
		return;
	}

	uint16_t cable_power_limit = (uint16_t)atomic_get(&dmc_board_power_limit);

	install_result = InstallDmcBoardPowerLimit(cable_power_limit);
	atomic_clear(&dmc_board_power_limit_applying);

	/* ArmRuntimePowerGuard() never raises the IRQ-visible limit while a DMC
	 * publication is being applied. Publish a legitimate increase only after
	 * that publication is complete and only if no newer one is pending.
	 */
	if (install_result == 0 && atomic_get(&board_power_policy_installed) != 0 &&
	    atomic_get(&dmc_board_power_limit_pending) == 0) {
		ArmRuntimePowerGuard((uint16_t)power_limit, k_uptime_get_32());
	}
}

static void SubmitDmcBoardPowerLimitWork(void)
{
#if defined(CONFIG_ZTEST)
	if (dmc_board_power_limit_worker_paused) {
		return;
	}
#endif
	(void)k_work_submit(&dmc_board_power_limit_work);
}

static void DmcBoardPowerLimitWorkHandler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* The system workqueue also serializes DVFS and host message handling, so
	 * controller history and arbiters cannot be changed from the SMBus ISR or
	 * halfway through a DVFS pass.
	 */
	ApplyPendingDmcBoardPowerLimit();
	if (atomic_get(&dmc_board_power_limit_pending) != 0) {
		SubmitDmcBoardPowerLimitWork();
	}
}

static void ResetThrottlerControllerState(void)
{
	for (ThrottlerId id = 0; id < kThrottlerCount; id++) {
		throttler[id].value = 0.0F;
		throttler[id].error = 0.0F;
		throttler[id].prev_error = 0.0F;
		throttler[id].output = 0.0F;
		SetAiclkArbMax(throttler[id].arb_max, GetAiclkFmax());
	}

	ResetBoardPowerHistory(GetInputPower());
	ResetDopplerTransientState();
	SetAiclkPowerSlew(false);
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
	const FwTable *fw_table = tt_bh_fwtable_get_fw_table(fwtable_dev);
	uint16_t boot_cable_power_limit = 0U;
	bool boot_policy_received;
	bool dmc_policy_used;
	uint16_t selected_cable_power_limit = 0U;

	atomic_clear(&throttlers_initialized);
	DisarmRuntimePowerGuard();
	power_limit = 0U;
	max_board_power_limit = 0U;
	host_board_power_limit_override = 0U;
	strict_board_power_limit = false;
	board_power_policy_containment = false;
	atomic_clear(&board_power_policy_strict);
	atomic_clear(&board_power_policy_installed);

	board_power_policy_required = bh_chip_info_board_power_policy_required();
	/* Doppler consumes the external whole-board sample. Sensorless lab/UBB
	 * variants retain the independent on-chip TDP/TDC controllers even if an
	 * old firmware table left doppler_en set.
	 */
	doppler = fw_table->feature_enable.doppler_en && board_power_policy_required;
	doppler_slow = doppler;
	doppler_t2 = doppler;
	doppler_t3 = doppler;
	/* PCIe error interrupts are enabled earlier in boot. Do not erase a fault
	 * that arrived between PCIe initialization and DVFS initialization.
	 */
	if (!atomic_get(&runtime_power_fault_latched)) {
		atomic_clear(&runtime_containment_pending);
		atomic_clear(&runtime_power_fault_trip_power);
		UpdateTelemetryRuntimePowerFault(false, 0);
	}

	kernel_throttler_stop_nops_freq_default =
		fw_table->chip_limits.kernel_throttler_stop_nops_freq;
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
	UpdateTelemetryKernelThrottler(fw_table->feature_enable.kernel_throttler_at_floor_en,
				       kernel_throttler_stop_nops_freq);

	SetThrottlerLimit(kThrottlerTDP, fw_table->chip_limits.tdp_limit);
	SetThrottlerLimit(kThrottlerFastTDC, fw_table->chip_limits.tdc_fast_limit);
	SetThrottlerLimit(kThrottlerTDC, fw_table->chip_limits.tdc_limit);
	SetThrottlerLimit(kThrottlerThm, fw_table->chip_limits.thm_limit);
	SetThrottlerLimit(kThrottlerBoardPower, DEFAULT_BOARD_POWER_LIMIT);
	SetThrottlerLimit(kThrottlerGDDRThm, fw_table->chip_limits.gddr_thm_limit);

	SetThrottlerLimit(kThrottlerDopplerSlow, DEFAULT_BOARD_POWER_LIMIT);

	ResetThrottlerControllerState();
	ResetRuntimeActivityGateBaseline();
	if (!ThrottlerRuntimePowerFaultLatched()) {
		kernel_nops_enabled = false;
		InitKernelThrottling();
	}

	EnableArbMax(throttler[kThrottlerTDP].arb_max, !doppler);
	EnableArbMax(throttler[kThrottlerFastTDC].arb_max, !doppler);
	EnableArbMax(throttler[kThrottlerTDC].arb_max, !doppler);
	EnableArbMax(throttler[kThrottlerBoardPower].arb_max, !doppler);

	EnableArbMax(throttler[kThrottlerThm].arb_max, thermal_throttling);
	EnableArbMax(throttler[kThrottlerGDDRThm].arb_max, thermal_throttling);

	EnableArbMax(throttler[kThrottlerDopplerSlow].arb_max, doppler_slow);

	SetAiclkArbMax(aiclk_arb_max_doppler_critical, GetAiclkFmin());
	EnableArbMax(aiclk_arb_max_doppler_critical, false); /* enabled when limit triggered */
	UpdateTelemetryBoardPowerLimit(0U);

	boot_policy_received = bh_get_boot_cable_power_limit(&boot_cable_power_limit);
	dmc_policy_used = atomic_get(&dmc_board_power_limit_received) != 0;
	if (dmc_policy_used) {
		selected_cable_power_limit = (uint16_t)atomic_get(&dmc_board_power_limit);
	} else if (boot_policy_received) {
		selected_cable_power_limit = boot_cable_power_limit;
	}

	if (is_cable_fault_mode()) {
		EnterBoardPowerPolicyContainment();
	} else if (board_power_policy_required &&
		   (!dmc_policy_used && !boot_policy_received)) {
		LOG_ERR("No DMC board-power policy available at throttler initialization");
		EnterBoardPowerPolicyContainment();
	} else if (dmc_policy_used || boot_policy_received) {
		(void)InstallDmcBoardPowerLimit(selected_cable_power_limit);
	}

	if (ThrottlerRuntimePowerFaultLatched()) {
		EnterBoardPowerPolicyContainment();
	}

	/* Consume any DMC update that raced initialization while the handler was
	 * deliberately forbidden from touching controller state. Keep initialized
	 * false until this synchronous, non-ISR replay is complete.
	 */
	ApplyPendingDmcBoardPowerLimit();

	/* Publish initialized only after every limiter and arbiter is deterministic.
	 * An update between the final drain above and this store observes false and
	 * leaves pending set; the check below then schedules it. An update after the
	 * store schedules itself.
	 */
	atomic_set(&throttlers_initialized, 1);
	if (atomic_get(&dmc_board_power_limit_pending) != 0) {
		SubmitDmcBoardPowerLimitWork();
	}
	UpdateTelemetryRuntimePowerStatus(ThrottlerBoardPowerPolicyStrict(),
					 ThrottlerBoardPowerSampleFresh(),
					 BoardPowerPolicyInstalled());
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
	/* The DMC-provided cable/board maximum and host overrides are both electrical
	 * safety limits. Once either is known, do not retain the legacy 2x transient
	 * allowance before forcing kernel NOPs.
	 */
	return strict_board_power_limit ? power_limit * RUNTIME_POWER_T2_PERCENT / 100U
					: power_limit * 2U;
}

static uint32_t GetDopplerT3PowerLimit(void)
{
	return strict_board_power_limit ? power_limit * RUNTIME_POWER_T3_PERCENT / 100U
					: power_limit * 5U / 2U;
}

static uint32_t GetRuntimePowerFailSafeLimit(void)
{
	return power_limit * (100U + RUNTIME_POWER_FAILSAFE_MARGIN_PERCENT) / 100U;
}

static bool RuntimeBoardPowerCritical(bool sample_fresh, uint16_t current_power)
{
	return ThrottlerRuntimePowerFaultLatched() ||
	       (strict_board_power_limit && (!sample_fresh || current_power >= power_limit));
}

static bool TryLatchRuntimeContainment(uint16_t current_power)
{
	if (!atomic_cas(&runtime_power_fault_latched, 0, 1)) {
		return false;
	}

	atomic_set(&runtime_power_fault_trip_power, current_power);
	atomic_set(&runtime_containment_pending, 1);
	return true;
}

static void SubmitRuntimeContainmentWork(void)
{
#if defined(CONFIG_ZTEST)
	if (runtime_containment_worker_paused) {
		return;
	}
#endif
	if (atomic_get(&runtime_containment_worker_enabled) != 0) {
		(void)k_work_submit(&runtime_containment_work);
	}
}

static void ScheduleRuntimeContainmentRetry(void)
{
#if defined(CONFIG_ZTEST)
	if (runtime_containment_worker_paused) {
		return;
	}
#endif
	if (atomic_get(&runtime_containment_worker_enabled) != 0) {
		(void)k_work_reschedule(&runtime_containment_retry_work, K_MSEC(1));
	}
}

static void CompleteRuntimeContainmentRequest(uint16_t current_power)
{
	InputPowerSample input_power_sample;
	bool sample_fresh;

	/* Stop compute immediately using only reset-unit APB writes. The deferred
	 * worker performs clock/voltage changes outside interrupt context.
	 */
	bh_hold_tensix_riscs_in_reset();
	sample_fresh =
		GetInputPowerSample(INPUT_POWER_FRESHNESS_MAX_AGE_MS, &input_power_sample);
	UpdateTelemetryRuntimePowerStatus(ThrottlerBoardPowerPolicyStrict(),
					 sample_fresh, BoardPowerPolicyInstalled());
	UpdateTelemetryRuntimePowerFault(true, current_power);
	SubmitRuntimeContainmentWork();
}

static bool RequestRuntimeContainment(uint16_t current_power)
{
	if (!TryLatchRuntimeContainment(current_power)) {
		return false;
	}

	CompleteRuntimeContainmentRequest(current_power);
	return true;
}

void ThrottlerObserveInputPowerFromIsr(uint16_t current_power, uint32_t sample_time_ms)
{
	unsigned int key = irq_lock();

	atomic_set(&runtime_power_guard_last_power, current_power);
	atomic_set(&runtime_power_guard_last_sample_ms, sample_time_ms);
	atomic_set(&runtime_power_guard_sample_seen, 1);
	bool threshold_crossed =
		atomic_get(&runtime_power_guard_armed) != 0 &&
		current_power >= (uint16_t)atomic_get(&runtime_power_guard_limit);
	bool newly_latched = threshold_crossed && TryLatchRuntimeContainment(current_power);

	irq_unlock(key);

	if (newly_latched) {
		CompleteRuntimeContainmentRequest(current_power);
	}
}

static void ArmRuntimePowerGuard(uint16_t effective_limit, uint32_t now_ms)
{
	InputPowerSample sample;
	bool sample_fresh;
	bool newly_armed;
	unsigned int key;

	if (effective_limit == 0U) {
		DisarmRuntimePowerGuard();
		return;
	}

	key = irq_lock();
	newly_armed = atomic_cas(&runtime_power_guard_armed, 0, 1);
	if (newly_armed) {
		atomic_set(&runtime_power_guard_limit, effective_limit);
		atomic_set(&runtime_power_guard_arm_ms, now_ms);
		atomic_clear(&runtime_power_guard_last_sample_ms);
		atomic_clear(&runtime_power_guard_last_power);
		atomic_clear(&runtime_power_guard_sample_seen);
	} else {
		uint16_t current_limit =
			(uint16_t)atomic_get(&runtime_power_guard_limit);
		bool dmc_apply = atomic_get(&dmc_board_power_limit_applying) != 0;
		bool dmc_update_pending =
			atomic_get(&dmc_board_power_limit_pending) != 0;

		/* A host update may raise an existing limit only when no DMC
		 * publication is pending or being applied. The DMC worker republishes a
		 * legitimate increase after completing its serialized transaction.
		 */
		if (effective_limit < current_limit || (!dmc_apply && !dmc_update_pending)) {
			atomic_set(&runtime_power_guard_limit, effective_limit);
		}
	}
	irq_unlock(key);

	if (newly_armed) {
#if !defined(CONFIG_ZTEST)
		k_timer_start(&runtime_power_sample_watchdog, K_MSEC(1), K_MSEC(1));
#endif
	}

	/* A sample can arrive before or during policy installation while the guard
	 * is disarmed. Re-evaluate one coherent, still-fresh sample after publishing
	 * the effective limit so that boundary cannot bypass the threshold.
	 */
	sample_fresh = GetInputPowerSample(INPUT_POWER_FRESHNESS_MAX_AGE_MS, &sample);
	if (sample_fresh) {
		ThrottlerObserveInputPowerFromIsr(sample.power, sample.updated_ms);
	}
}

static void DisarmRuntimePowerGuard(void)
{
	atomic_clear(&runtime_power_guard_armed);
	k_timer_stop(&runtime_power_sample_watchdog);
}

static bool CheckRuntimePowerSampleWatchdog(uint32_t now_ms)
{
	uint32_t reference_ms;
	uint16_t last_power;
	bool newly_latched = false;
	unsigned int key = irq_lock();

	if (atomic_get(&runtime_power_guard_armed) == 0 ||
	    ThrottlerRuntimePowerFaultLatched()) {
		irq_unlock(key);
		return false;
	}

	reference_ms = atomic_get(&runtime_power_guard_sample_seen) != 0
			       ? (uint32_t)atomic_get(&runtime_power_guard_last_sample_ms)
			       : (uint32_t)atomic_get(&runtime_power_guard_arm_ms);
	last_power = (uint16_t)atomic_get(&runtime_power_guard_last_power);
	if ((uint32_t)(now_ms - reference_ms) > RUNTIME_POWER_STALE_LATCH_MS) {
		newly_latched = TryLatchRuntimeContainment(last_power);
	}
	irq_unlock(key);

	if (newly_latched) {
		CompleteRuntimeContainmentRequest(last_power);
	}
	return newly_latched;
}

static void RuntimePowerSampleWatchdogHandler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	(void)CheckRuntimePowerSampleWatchdog(k_uptime_get_32());
}

void ThrottlerRequestRuntimeContainment(void)
{
	(void)RequestRuntimeContainment(0);
}

void ThrottlerRetryRuntimeContainment(void)
{
	if (!ThrottlerRuntimePowerFaultLatched()) {
		ThrottlerRequestRuntimeContainment();
		return;
	}

	/* Retry is distinct from an idempotent repeated fault IRQ: a failed
	 * clock/voltage transaction must reassert hardware reset and leave durable
	 * pending work for a later, fair system-workqueue pass.
	 */
	bh_hold_tensix_riscs_in_reset();
	atomic_set(&runtime_containment_pending, 1);
	ScheduleRuntimeContainmentRetry();
}

void ThrottlerEnableRuntimeContainmentWorker(void)
{
	atomic_set(&runtime_containment_worker_enabled, 1);
	if (atomic_get(&runtime_containment_pending)) {
		SubmitRuntimeContainmentWork();
	}
}

static bool UpdateRuntimePowerGuard(bool eligible, uint16_t current_power, uint32_t now_ms)
{
	ARG_UNUSED(now_ms);

	return eligible && RequestRuntimeContainment(current_power);
}

static void ApplyPendingRuntimeContainment(uint16_t current_power)
{
	if (!atomic_cas(&runtime_containment_pending, 1, 0)) {
		return;
	}

	uint16_t trip_power = (uint16_t)atomic_get(&runtime_power_fault_trip_power);

	if (trip_power == 0U) {
		trip_power = current_power;
		atomic_set(&runtime_power_fault_trip_power, trip_power);
		UpdateTelemetryRuntimePowerFault(true, trip_power);
	}

	LOG_ERR("Applying runtime power containment at %u W (limit %u W)", trip_power,
		power_limit);
	if (bh_force_safe_power_state() != 0) {
		LOG_ERR("Failed to enter runtime power containment state");
		ThrottlerRetryRuntimeContainment();
	}
}

static void RuntimeContainmentWorkHandler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* This work item is independent of the periodic DVFS timer, so a PCIe IRQ
	 * cannot leave containment pending when that timer is stopped or disabled.
	 * DVFSChange() consumes the pending state first, then applies clock-before-
	 * voltage ordering through the normal serialized system workqueue path.
	 */
	if (dvfs_enabled) {
		(void)DVFSChange();
	} else {
		InputPowerSample sample;

		/* Initialization failures can leave the periodic DVFS controller
		 * unavailable. The APB RISC hold is already active; still commit the
		 * durable logical safe state without entering an uninitialized VF path.
		 */
		(void)GetInputPowerSample(UINT32_MAX, &sample);
		ApplyPendingRuntimeContainment(sample.power);
	}
}

static void UpdateRuntimePowerFailSafe(bool sample_fresh, uint16_t current_power)
{
	bool eligible = sample_fresh && strict_board_power_limit &&
			current_power >= GetRuntimePowerFailSafeLimit();

	if (!UpdateRuntimePowerGuard(eligible, current_power, k_uptime_get_32())) {
		return;
	}

	LOG_ERR("Runtime board-power fail-safe tripped at %u W (limit %u W)", current_power,
		power_limit);
	ApplyPendingRuntimeContainment(current_power);
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

bool ThrottlerTestUpdateRuntimePowerGuard(bool eligible, uint16_t current_power, uint32_t now_ms)
{
	return UpdateRuntimePowerGuard(eligible, current_power, now_ms);
}

void ThrottlerTestResetRuntimePowerGuard(void)
{
	InputPowerSample input_power_sample;
	bool sample_fresh;

	atomic_clear(&runtime_power_fault_latched);
	atomic_clear(&runtime_containment_pending);
	atomic_clear(&runtime_power_fault_trip_power);
	atomic_clear(&runtime_containment_worker_enabled);
	if (atomic_get(&runtime_power_guard_armed) != 0) {
		atomic_set(&runtime_power_guard_arm_ms, k_uptime_get_32());
		atomic_clear(&runtime_power_guard_sample_seen);
		atomic_clear(&runtime_power_guard_last_power);
	}
	sample_fresh =
		GetInputPowerSample(INPUT_POWER_FRESHNESS_MAX_AGE_MS, &input_power_sample);
	UpdateTelemetryRuntimePowerStatus(ThrottlerBoardPowerPolicyStrict(),
					 sample_fresh, BoardPowerPolicyInstalled());
	UpdateTelemetryRuntimePowerFault(false, 0);
}

uint16_t ThrottlerTestUpdateBoardPowerHistory(uint16_t current_power)
{
	uint16_t average_power = UpdateMovingAveragePower(current_power);
	uint16_t short_average_power = UpdateShortAveragePower(current_power);

	return strict_board_power_limit
		       ? MAX(current_power, MAX(average_power, short_average_power))
		       : average_power;
}

void ThrottlerTestResetBoardPowerHistory(uint16_t current_power)
{
	ResetBoardPowerHistory(current_power);
}

bool ThrottlerTestRuntimeBoardPowerCritical(bool sample_fresh, uint16_t current_power)
{
	return RuntimeBoardPowerCritical(sample_fresh, current_power);
}

bool ThrottlerTestRuntimeContainmentPending(void)
{
	return atomic_get(&runtime_containment_pending) != 0;
}

void ThrottlerTestPauseRuntimeContainmentWorker(bool pause)
{
	runtime_containment_worker_paused = pause;
	if (!pause && atomic_get(&runtime_containment_pending) != 0) {
		SubmitRuntimeContainmentWork();
	}
}

void ThrottlerTestApplyPendingRuntimeContainment(uint16_t current_power)
{
	ApplyPendingRuntimeContainment(current_power);
}

void ThrottlerTestPrepareForInit(void)
{
	struct k_work_sync dmc_sync;
	struct k_work_sync containment_sync;
	struct k_work_sync retry_sync;

	/* Model an ASIC restart, including quiescing work left by the previous
	 * test. Otherwise an armed low limit can relatch on the boot sample before
	 * InitThrottlers() has installed the next test's policy.
	 */
	dmc_board_power_limit_worker_paused = true;
	runtime_containment_worker_paused = true;
	(void)k_work_cancel_sync(&dmc_board_power_limit_work, &dmc_sync);
	(void)k_work_cancel_sync(&runtime_containment_work, &containment_sync);
	(void)k_work_cancel_delayable_sync(&runtime_containment_retry_work,
					   &retry_sync);
	DisarmRuntimePowerGuard();
	atomic_clear(&runtime_power_guard_limit);
	atomic_clear(&runtime_power_guard_arm_ms);
	atomic_clear(&runtime_power_guard_last_sample_ms);
	atomic_clear(&runtime_power_guard_last_power);
	atomic_clear(&runtime_power_guard_sample_seen);
	atomic_clear(&runtime_power_fault_latched);
	atomic_clear(&runtime_containment_pending);
	atomic_clear(&runtime_power_fault_trip_power);
	atomic_clear(&throttlers_initialized);
	atomic_clear(&dmc_board_power_limit_received);
	atomic_clear(&dmc_board_power_limit);
	atomic_clear(&dmc_board_power_limit_generation);
	atomic_clear(&dmc_board_power_limit_pending);
	atomic_clear(&dmc_board_power_limit_applying);
	dmc_board_power_limit_worker_paused = false;
	runtime_containment_worker_paused = false;
	host_board_power_pre_apply_hook = NULL;
	runtime_activity_transition_sample_hook = NULL;
}

void ThrottlerTestConfigureRuntimePowerWatchdog(uint16_t effective_limit,
					       uint32_t arm_ms)
{
	DisarmRuntimePowerGuard();
	atomic_set(&runtime_power_guard_limit, effective_limit);
	atomic_set(&runtime_power_guard_arm_ms, arm_ms);
	atomic_clear(&runtime_power_guard_last_sample_ms);
	atomic_clear(&runtime_power_guard_last_power);
	atomic_clear(&runtime_power_guard_sample_seen);
	atomic_set(&runtime_power_guard_armed, effective_limit != 0U);
}

bool ThrottlerTestCheckRuntimePowerWatchdog(uint32_t now_ms)
{
	return CheckRuntimePowerSampleWatchdog(now_ms);
}

uint16_t ThrottlerTestRuntimePowerGuardLimit(void)
{
	return (uint16_t)atomic_get(&runtime_power_guard_limit);
}

void ThrottlerTestSetHostBoardPowerPreApplyHook(void (*hook)(void))
{
	host_board_power_pre_apply_hook = hook;
}

void ThrottlerTestPauseDmcBoardPowerLimitWorker(bool pause)
{
	dmc_board_power_limit_worker_paused = pause;
	if (!pause && atomic_get(&dmc_board_power_limit_pending) != 0 &&
	    atomic_get(&throttlers_initialized) != 0) {
		SubmitDmcBoardPowerLimitWork();
	}
}

bool ThrottlerTestDmcBoardPowerLimitPending(void)
{
	return atomic_get(&dmc_board_power_limit_pending) != 0;
}

void ThrottlerTestFlushDmcBoardPowerLimitWork(void)
{
	struct k_work_sync sync;

	(void)k_work_flush(&dmc_board_power_limit_work, &sync);
}

void ThrottlerTestFlushRuntimeContainmentWork(void)
{
	struct k_work_sync sync;

	(void)k_work_flush(&runtime_containment_work, &sync);
}

bool ThrottlerTestRuntimeActivityGateOpen(void)
{
	return runtime_activity_gate_open;
}

bool ThrottlerTestRuntimeActivityBaselineValid(void)
{
	return atomic_get(&runtime_activity_baseline_valid) != 0;
}

void ThrottlerTestSetBoardPowerPolicyRequired(bool required)
{
	board_power_policy_required = required;
	if (!required) {
		doppler = false;
		doppler_slow = false;
		doppler_t2 = false;
		doppler_t3 = false;
		ConfigureRuntimePowerArbiters();
		EnableArbMax(throttler[kThrottlerDopplerSlow].arb_max, false);
	}
}

void ThrottlerTestSetPowerTransitionSampleHook(void (*hook)(void))
{
	runtime_activity_transition_sample_hook = hook;
}
#endif

static void UpdateDoppler(const TelemetryInternalData *telemetry, uint16_t current_power,
			  bool sample_fresh)
{
	if (sample_fresh) {
		uint16_t average_power = UpdateMovingAveragePower(current_power);
		uint16_t short_average_power = UpdateShortAveragePower(current_power);
		uint16_t control_power =
			strict_board_power_limit
				? MAX(current_power, MAX(average_power, short_average_power))
				: average_power;

		UpdateThrottler(kThrottlerDopplerSlow, control_power);
	}

	/* Doppler T2 throttler: 10 consecutive samples over its transient limit. */
	uint32_t t2_power_limit = GetDopplerT2PowerLimit();

	if (current_power >= t2_power_limit) {
		if (t2_count < UINT8_MAX) {
			t2_count++;
		}
	} else {
		t2_count = 0;
	}

	uint8_t t2_required_samples = strict_board_power_limit ? 1U : 10U;
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

	uint8_t t3_required_samples = strict_board_power_limit ? 1U : 2U;
	bool t3_triggered = t3_count >= t3_required_samples && doppler_t3;

	/* AICLK=Fmin isn't always enough to get below the board power limit. */
	bool start_nops = GetAiclkTarg() == GetAiclkFmin() && current_power > power_limit;
	uint32_t runtime_release_power =
		power_limit * (100U - RUNTIME_POWER_CONTROL_HEADROOM_PERCENT) / 100U;
	bool stop_nops = strict_board_power_limit
				 ? sample_fresh && current_power <= runtime_release_power
				 : GetAiclkTarg() == GetAiclkFmax() && current_power < power_limit;

	/* Never raise clocks using a missing or stale board-power sample. Holding
	 * AICLK at Fmin and sending kernel NOPs keeps the card and PCIe management
	 * path alive while DMC/INA228 communication recovers.
	 */
	bool critical_throttling = RuntimeBoardPowerCritical(sample_fresh, current_power) ||
				   t2_triggered || t3_triggered;

	bool new_kernel_nops_enabled =
		((kernel_nops_enabled || start_nops) && !stop_nops) || critical_throttling;

	if (new_kernel_nops_enabled != kernel_nops_enabled) {
		kernel_nops_enabled = new_kernel_nops_enabled;
		SendKernelThrottlingMessage(kernel_nops_enabled);
	}

	EnableArbMax(aiclk_arb_max_doppler_critical, critical_throttling);
	UpdateRuntimePowerFailSafe(sample_fresh, current_power);
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
	InputPowerSample input_power_sample;
	bool sample_fresh =
		GetInputPowerSample(INPUT_POWER_FRESHNESS_MAX_AGE_MS, &input_power_sample);
	uint16_t board_power = input_power_sample.power;

	ApplyPendingRuntimeContainment(board_power);
	ReadTelemetryInternal(1, &telemetry_internal_data);
	UpdateRuntimeActivityGate(&telemetry_internal_data, sample_fresh,
				  &input_power_sample, k_uptime_get_32());

	if (DopplerActive()) {
		UpdateDoppler(&telemetry_internal_data, board_power, sample_fresh);
		if (strict_board_power_limit) {
			UpdateThrottler(kThrottlerTDP, telemetry_internal_data.vcore_power);
			UpdateThrottler(kThrottlerFastTDC, telemetry_internal_data.vcore_current);
			UpdateThrottler(kThrottlerTDC, telemetry_internal_data.vcore_current);
		}
	} else {
		UpdateThrottler(kThrottlerTDP, telemetry_internal_data.vcore_power);
		UpdateThrottler(kThrottlerFastTDC, telemetry_internal_data.vcore_current);
		UpdateThrottler(kThrottlerTDC, telemetry_internal_data.vcore_current);

		if (sample_fresh) {
			UpdateThrottler(kThrottlerBoardPower, board_power);
		}

		if (strict_board_power_limit) {
			bool critical = RuntimeBoardPowerCritical(sample_fresh, board_power);
			bool release =
				sample_fresh &&
				board_power <=
					power_limit *
						(100U - RUNTIME_POWER_CONTROL_HEADROOM_PERCENT) /
						100U;
			bool new_kernel_nops_enabled =
				critical || (kernel_nops_enabled && !release);

			if (new_kernel_nops_enabled != kernel_nops_enabled) {
				kernel_nops_enabled = new_kernel_nops_enabled;
				SendKernelThrottlingMessage(kernel_nops_enabled);
			}

			EnableArbMax(aiclk_arb_max_doppler_critical, critical);
			UpdateRuntimePowerFailSafe(sample_fresh, board_power);
		} else {
			UpdateKernelThrottler(telemetry_internal_data.vcore_power,
					      throttler[kThrottlerTDP].limit);
		}
	}

	UpdateThrottler(kThrottlerThm, telemetry_internal_data.asic_temperature);
	UpdateThrottler(kThrottlerGDDRThm, telemetry_internal_data.gddr_temps.max_temp);

	for (ThrottlerId i = 0; i < kThrottlerCount; i++) {
		UpdateThrottlerArb(i);
	}
	if (strict_board_power_limit &&
	    (!runtime_activity_gate_open || runtime_activity_idle_candidate)) {
		/* BUSY is asserted at device-open time, potentially long before a
		 * workload. Keep the physical load step bounded by holding the board's
		 * configured Fmin until independent activity sensors agree.
		 */
		SetAiclkArbMax(throttler[kThrottlerDopplerSlow].arb_max, GetAiclkFmin());
	}

	UpdateTelemetryRuntimePowerStatus(ThrottlerBoardPowerPolicyStrict(),
					 sample_fresh, BoardPowerPolicyInstalled());
}

bool ThrottlerRuntimePowerFaultLatched(void)
{
	return atomic_get(&runtime_power_fault_latched) != 0;
}

bool ThrottlerBoardPowerPolicyStrict(void)
{
	return atomic_get(&board_power_policy_strict) != 0;
}

bool ThrottlerBoardPowerSampleFresh(void)
{
	InputPowerSample input_power_sample;

	return GetInputPowerSample(INPUT_POWER_FRESHNESS_MAX_AGE_MS, &input_power_sample);
}

static bool BoardPowerPolicyInstalled(void)
{
	if (atomic_get(&throttlers_initialized) == 0 ||
	    atomic_get(&dmc_board_power_limit_pending) != 0 ||
	    atomic_get(&dmc_board_power_limit_applying) != 0 ||
	    atomic_get(&board_power_policy_installed) == 0 || error_status0 != 0U) {
		return false;
	}
	if (board_power_policy_required &&
	    atomic_get(&board_power_policy_strict) == 0) {
		return false;
	}

	return true;
}

bool ThrottlerBoardPowerPolicyInstalled(void)
{
	return BoardPowerPolicyInstalled();
}

bool ThrottlerBoardPowerPolicyRequired(void)
{
	return atomic_get(&throttlers_initialized) != 0 && board_power_policy_required;
}

bool ThrottlerBoardPowerPolicyReady(void)
{
	InputPowerSample input_power_sample;
	bool sample_fresh =
		GetInputPowerSample(INPUT_POWER_FRESHNESS_MAX_AGE_MS, &input_power_sample);

	return BoardPowerPolicyInstalled() && sample_fresh;
}

bool ThrottlerPrepareComputeRelease(void)
{
	uint32_t start_ms = k_uptime_get_32();

	if (atomic_get(&throttlers_initialized) == 0 || error_status0 != 0U ||
	    ThrottlerRuntimePowerFaultLatched()) {
		return false;
	}

	if (!board_power_policy_required) {
		/* Galaxy/UBB has no DMC or whole-board input-power sensor. It retains the
		 * pre-existing on-chip TDP/TDC policy and does not use the activity gate.
		 */
		runtime_activity_gate_open = true;
		UpdateTelemetryRuntimePowerStatus(false, false, false);
		return true;
	}

	for (;;) {
		TelemetryInternalData telemetry;
		InputPowerSample input_power;
		uint32_t now_ms;

		ReadTelemetryInternal(0, &telemetry);
		now_ms = k_uptime_get_32();
		if (BoardPowerPolicyInstalled() &&
		    VcoreActivitySampleFresh(&telemetry, now_ms) &&
		    GetInputPowerSample(INPUT_POWER_FRESHNESS_MAX_AGE_MS, &input_power)) {
			/* Capture the one permitted idle reference at the final release
			 * barrier, while every Tensix RISC is still held in hardware reset.
			 * Never learn a missing reference after compute may have started.
			 */
			runtime_activity_boot_idle_vcore_power = telemetry.vcore_power;
			runtime_activity_boot_idle_board_power = input_power.power;
			runtime_activity_idle_vcore_power = telemetry.vcore_power;
			runtime_activity_idle_board_power = input_power.power;
			RearmRuntimeActivityGate();
			atomic_set(&runtime_activity_baseline_valid, 1);
			UpdateTelemetryRuntimePowerStatus(
				ThrottlerBoardPowerPolicyStrict(), true,
				BoardPowerPolicyInstalled());
			return true;
		}

		if ((uint32_t)(k_uptime_get_32() - start_ms) >=
		    RUNTIME_ACTIVITY_BOOT_SAMPLE_WAIT_MS) {
			break;
		}
		k_msleep(1);
	}

	LOG_ERR("No coherent DMC/AVS idle sample before compute-release deadline");
	return false;
}

bool ThrottlerComputePowerPolicyReady(void)
{
	if (atomic_get(&throttlers_initialized) == 0 || error_status0 != 0U ||
	    ThrottlerRuntimePowerFaultLatched()) {
		return false;
	}

	if (!board_power_policy_required) {
		return true;
	}

	return ThrottlerBoardPowerPolicyStrict() && ThrottlerBoardPowerPolicyReady() &&
	       atomic_get(&runtime_activity_baseline_valid) != 0;
}

uint8_t ThrottlerSetKernelThrottlerEnabled(uint32_t enabled)
{
	if (enabled > 1) {
		return 1;
	}
	if (ThrottlerBoardPowerPolicyStrict() && enabled == 0U) {
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
	if (ThrottlerBoardPowerPolicyStrict() && frequency != 0U) {
		return 2;
	}

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

static void PublishDmcBoardPowerLimitFromIsr(uint16_t cable_power_limit)
{
	bool newly_latched = false;
	uint16_t trip_power = 0U;
	unsigned int key = irq_lock();

	/* Publish the value, pending state, and generation as one IRQ-visible
	 * transaction. A host request that was validated against the previous DMC
	 * maximum can then detect that its premise changed before acknowledging it.
	 */
	atomic_set(&dmc_board_power_limit, cable_power_limit);
	atomic_set(&dmc_board_power_limit_received, 1);
	atomic_set(&dmc_board_power_limit_pending, 1);
	atomic_inc(&dmc_board_power_limit_generation);

	if (cable_power_limit < BOARD_POWER_CONTROLLER_MIN_W &&
	    atomic_get(&throttlers_initialized) != 0) {
		/* Zero and below-controller-minimum limits cannot be represented safely.
		 * Once runtime control exists, stop compute now and let the serialized
		 * worker retain policy containment. Before initialization there can be no
		 * running workload; InitThrottlers() consumes the invalid retained value
		 * and keeps the normal compute-release barrier closed without inventing an
		 * irreversible runtime fault during clean boot.
		 */
		if (atomic_get(&runtime_power_guard_sample_seen) != 0) {
			trip_power = (uint16_t)atomic_get(&runtime_power_guard_last_power);
		}
		newly_latched = TryLatchRuntimeContainment(trip_power);
	} else if (atomic_get(&runtime_power_guard_armed) != 0) {
		uint16_t current_limit =
			(uint16_t)atomic_get(&runtime_power_guard_limit);
		uint16_t tightened_limit = MIN(current_limit, cable_power_limit);

		/* A lower cable maximum becomes the IRQ threshold immediately, before
		 * deferred controller/history work. Never raise the guard here.
		 */
		atomic_set(&runtime_power_guard_limit, tightened_limit);
		if (atomic_get(&runtime_power_guard_sample_seen) != 0) {
			trip_power =
				(uint16_t)atomic_get(&runtime_power_guard_last_power);
			newly_latched = trip_power >= tightened_limit &&
					TryLatchRuntimeContainment(trip_power);
		}
	}
	irq_unlock(key);

	if (newly_latched) {
		CompleteRuntimeContainmentRequest(trip_power);
	}
}

int32_t Dm2CmSetBoardPowerLimit(const uint8_t *data, uint8_t size)
{
	if (size != 2) {
		return -1;
	}

	uint16_t cable_power_limit = sys_get_le16(data);

	PublishDmcBoardPowerLimitFromIsr(cable_power_limit);

	/* The SMBus target becomes reachable before InitThrottlers() runs. Retain
	 * an early DMC policy and apply it only after every arbiter has been reset.
	 */
	if (atomic_get(&throttlers_initialized) == 0) {
		return cable_power_limit < BOARD_POWER_CONTROLLER_MIN_W ? -1 : 0;
	}

	SubmitDmcBoardPowerLimitWork();
	return cable_power_limit < BOARD_POWER_CONTROLLER_MIN_W ? -1 : 0;
}

static uint8_t set_board_power_limit_handler(const union request *request,
					     struct response *response)
{
	ARG_UNUSED(response);
	atomic_val_t dmc_generation;
	uint32_t new_power_limit;
	uint32_t previous_power_limit;
	uint32_t previous_host_override;
	bool previous_strict;

	/* A DMC policy transaction owns the electrical maximum. Never validate a
	 * host request while that maximum is pending or halfway through installation.
	 */
	if (atomic_get(&dmc_board_power_limit_pending) != 0 ||
	    atomic_get(&dmc_board_power_limit_applying) != 0) {
		return 2;
	}
	dmc_generation = atomic_get(&dmc_board_power_limit_generation);

	new_power_limit = request->set_board_power_limit.restore_default
				  ? max_board_power_limit
				  : request->set_board_power_limit.board_power_limit;

	/* The fail-safe deliberately requires an ASIC reset. Accepting another
	 * limit here would suggest the card is ready to run even though Tensix is
	 * still held in hardware RISC reset.
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

#if defined(CONFIG_ZTEST)
	if (host_board_power_pre_apply_hook != NULL) {
		void (*hook)(void) = host_board_power_pre_apply_hook;

		host_board_power_pre_apply_hook = NULL;
		hook();
	}
#endif

	/* This is the host/DMC race linearization point. An SMBus IRQ may have
	 * published a new cable maximum after the checks above; reject instead of
	 * applying or acknowledging against stale policy state.
	 */
	if (dmc_generation != atomic_get(&dmc_board_power_limit_generation) ||
	    atomic_get(&dmc_board_power_limit_pending) != 0 ||
	    atomic_get(&dmc_board_power_limit_applying) != 0) {
		return 2;
	}

	previous_power_limit = power_limit;
	previous_host_override = host_board_power_limit_override;
	previous_strict = strict_board_power_limit;
	/* Restoring the board default removes the host override, not the electrical
	 * safety policy. The DMC-provided cable/board maximum is always enforced.
	 */
	host_board_power_limit_override =
		request->set_board_power_limit.restore_default ? 0U : new_power_limit;
	strict_board_power_limit = new_power_limit > 0U;
	apply_board_power_limit(new_power_limit);

	/* Close the second IRQ window across the controller update itself. The
	 * IRQ-visible guard can only tighten while a DMC update is pending, and this
	 * rollback restores the old controller/override without raising that guard.
	 */
	if (dmc_generation != atomic_get(&dmc_board_power_limit_generation) ||
	    atomic_get(&dmc_board_power_limit_pending) != 0 ||
	    atomic_get(&dmc_board_power_limit_applying) != 0) {
		host_board_power_limit_override = previous_host_override;
		strict_board_power_limit = previous_strict;
		apply_board_power_limit(previous_power_limit);
		return 2;
	}
	if (ThrottlerRuntimePowerFaultLatched() || !ThrottlerBoardPowerPolicyReady()) {
		/* An unrelated fault or sample-expiry IRQ won the race. The hardware
		 * latch is authoritative; do not report the host update as accepted.
		 */
		host_board_power_limit_override = previous_host_override;
		strict_board_power_limit = previous_strict;
		apply_board_power_limit(previous_power_limit);
		return 2;
	}

	LOG_INF("Runtime board power limit: %u", new_power_limit);

	/* A host-requested limit is a safety constraint. Begin at the AICLK floor
	 * and let the closed-loop controller ramp upward instead of waiting for an
	 * over-limit sample before it reacts.
	 */
	if (strict_board_power_limit) {
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
	if (ThrottlerBoardPowerPolicyStrict() &&
	    new_tdp_limit > throttler[kThrottlerTDP].limit) {
		return 2;
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
