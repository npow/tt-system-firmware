/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <tenstorrent/smc_msg.h>
#include <tenstorrent/msgqueue.h>
#include <zephyr/drivers/misc/bh_fwtable.h>

#include "voltage.h"
#include "regulator.h"
#include "dvfs.h"
#include "init.h"
#include "throttler.h"

#define VDD_BOOT            750
/* Bound checks for VDD_MAX and VDD_MIN (in mV) */
#define VDD_MAX_UPPER_LIMIT 900U
#define VDD_MAX_LOWER_LIMIT 700U
#define VDD_MIN_UPPER_LIMIT 900U
#define VDD_MIN_LOWER_LIMIT 650U

static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));

VoltageArbiter voltage_arbiter;

#if defined(CONFIG_ZTEST)
static int (*set_vcore_test_hook)(uint32_t voltage);
#endif

static int SetVcoreForDvfs(uint32_t voltage)
{
#if defined(CONFIG_ZTEST)
	if (set_vcore_test_hook != NULL) {
		return set_vcore_test_hook(voltage);
	}
#endif
	return set_vcore(voltage);
}

int VoltageChange(void)
{
	if (voltage_arbiter.targ_voltage != voltage_arbiter.curr_voltage) {
		bool increasing = voltage_arbiter.targ_voltage > voltage_arbiter.curr_voltage;
		int ret;

		/* A stale high target calculated before the power-fault IRQ must never
		 * start a regulator transaction. A fault that arrives during a
		 * preemptible regulator transaction is checked again below; Tensix is
		 * already held in reset and the caller will skip the AICLK increase.
		 */
		if (increasing && ThrottlerRuntimePowerFaultLatched()) {
			return -EPERM;
		}
		ret = SetVcoreForDvfs(voltage_arbiter.targ_voltage);

		if (ret != 0) {
			return ret;
		}
		voltage_arbiter.curr_voltage = voltage_arbiter.targ_voltage;
		if (increasing && ThrottlerRuntimePowerFaultLatched()) {
			/* Keep the observed voltage in curr_voltage. Lowering it before the
			 * containment worker downclocks AICLK would invert the safe VF order.
			 */
			LatchVoltagePowerFault();
			return -EPERM;
		}
	}

	return 0;
}

void VoltageArbRequest(VoltageRequestor req, uint32_t voltage)
{
	voltage_arbiter.req_voltage[req] =
		CLAMP(voltage, voltage_arbiter.vdd_min, voltage_arbiter.vdd_max);
}

void CalculateTargVoltage(void)
{
	/* The target voltage is the maximum of all requested voltages */
	uint32_t targ_voltage = voltage_arbiter.vdd_min;

	for (VoltageRequestor i = 0; i < VoltageReqCount; i++) {
		if (voltage_arbiter.req_voltage[i] > targ_voltage) {
			targ_voltage = voltage_arbiter.req_voltage[i];
		}
	}

	/* Limit to vdd_max */
	voltage_arbiter.targ_voltage = MIN(targ_voltage, voltage_arbiter.vdd_max);

	/* Apply forced voltage at the end, regardless of any limits */
	if (voltage_arbiter.forced_voltage != 0 && !ThrottlerRuntimePowerFaultLatched()) {
		voltage_arbiter.targ_voltage = voltage_arbiter.forced_voltage;
	}
}

void LatchVoltagePowerFault(void)
{
	/* CalculateThrottlers() invokes this from an active DVFS transaction. Clear
	 * the override and pending requests here; the outer transaction preserves
	 * the required clock-before-voltage ordering when it applies the safe point.
	 */
	voltage_arbiter.forced_voltage = 0;
	for (VoltageRequestor i = 0; i < VoltageReqCount; i++) {
		voltage_arbiter.req_voltage[i] = voltage_arbiter.vdd_min;
	}
}

#if defined(CONFIG_ZTEST)
void VoltageTestSetVcoreHook(int (*hook)(uint32_t voltage))
{
	set_vcore_test_hook = hook;
}
#endif

int InitVoltagePPM(void)
{
	voltage_arbiter.vdd_min =
		CLAMP(tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.vdd_min,
		      VDD_MIN_LOWER_LIMIT, VDD_MIN_UPPER_LIMIT);
	voltage_arbiter.vdd_max =
		CLAMP(tt_bh_fwtable_get_fw_table(fwtable_dev)->chip_limits.vdd_max,
		      VDD_MAX_LOWER_LIMIT, VDD_MAX_UPPER_LIMIT);

	/* disable forcing of VDD */
	voltage_arbiter.forced_voltage = 0;

	for (VoltageRequestor i = 0; i < VoltageReqCount; i++) {
		voltage_arbiter.req_voltage[i] = voltage_arbiter.vdd_min;
	}
	if (set_vcore(VDD_BOOT) != 0) {
		return -EIO;
	}
	voltage_arbiter.curr_voltage = VDD_BOOT;
	voltage_arbiter.targ_voltage = voltage_arbiter.curr_voltage;

	/* Change VCOREM to 0.85 V to enforce the rule VCOREM - 300 mV <= VCORE <= VCOREM + 100mV */
	/* Thus allowing VCORE in the range of 0.55 V to 0.95 V */
	if (set_vcorem(850) != 0) {
		return -EIO;
	}

	return 0;
}

uint8_t ForceVdd(uint32_t voltage)
{
	if (ThrottlerRuntimePowerFaultLatched()) {
		/* A release request is harmless and keeps this API idempotent, but no
		 * caller may establish a new voltage override before ASIC reset.
		 */
		voltage_arbiter.forced_voltage = 0;
		return voltage == 0 ? 0 : 1;
	}

	if ((voltage > voltage_arbiter.vdd_max || voltage < voltage_arbiter.vdd_min) &&
	    (voltage != 0)) {
		return 1;
	}

	if (dvfs_enabled) {
		voltage_arbiter.forced_voltage = voltage;
		return DVFSChange() == 0 ? 0 : 1;
	} else {
		/* Direct regulator programming is only valid before application init.
		 * If DVFS is still disabled after init begins, regulator/DVFS setup did
		 * not complete and every nonzero override must fail closed.
		 */
		if (BhArcInitializationStarted()) {
			voltage_arbiter.forced_voltage = 0;
			return voltage == 0U ? 0U : 1U;
		}

		/* restore to boot voltage */
		if (voltage == 0) {
			voltage = VDD_BOOT;
		}

		if (set_vcore(voltage) != 0) {
			return 1;
		}
	}
	return 0;
}

/**
 * @brief Handler for @ref TT_SMC_MSG_FORCE_VDD
 * @see force_vdd_rqst
 */
static uint8_t ForceVddHandler(const union request *request, struct response *response)
{
	uint32_t forced_voltage = request->force_vdd.forced_voltage;

	return ForceVdd(forced_voltage);
}

REGISTER_MESSAGE(TT_SMC_MSG_FORCE_VDD, ForceVddHandler);
