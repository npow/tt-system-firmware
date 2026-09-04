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
#include "throttler.h"

#define VDD_BOOT            750
/* Bound checks for VDD_MAX and VDD_MIN (in mV) */
#define VDD_MAX_UPPER_LIMIT 900U
#define VDD_MAX_LOWER_LIMIT 700U
#define VDD_MIN_UPPER_LIMIT 900U
#define VDD_MIN_LOWER_LIMIT 650U

static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));

VoltageArbiter voltage_arbiter;

bool VoltageChange(void)
{
	if (voltage_arbiter.targ_voltage != voltage_arbiter.curr_voltage) {
		if (set_vcore(voltage_arbiter.targ_voltage) != 0) {
			return false;
		}
		voltage_arbiter.curr_voltage = voltage_arbiter.targ_voltage;
	}

	return true;
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
	if (voltage_arbiter.forced_voltage != 0 && !ThrottlerStrictRuntimePowerLimitActive()) {
		voltage_arbiter.targ_voltage = voltage_arbiter.forced_voltage;
	}
}

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
		return -1;
	}
	voltage_arbiter.curr_voltage = VDD_BOOT;
	voltage_arbiter.targ_voltage = voltage_arbiter.curr_voltage;

	/* Change VCOREM to 0.85 V to enforce the rule VCOREM - 300 mV <= VCORE <= VCOREM + 100mV */
	/* Thus allowing VCORE in the range of 0.55 V to 0.95 V */
	if (set_vcorem(850) != 0) {
		return -1;
	}

	return 0;
}

uint8_t ForceVdd(uint32_t voltage)
{
	if ((voltage > voltage_arbiter.vdd_max || voltage < voltage_arbiter.vdd_min) &&
	    (voltage != 0)) {
		return 1;
	}
	/* A forced voltage bypasses the normal AICLK/VF request. Once DMC has
	 * installed the board-power policy, only releasing an old override is safe.
	 */
	if (ThrottlerStrictRuntimePowerLimitActive() && voltage != 0U) {
		return 2;
	}

	if (!DVFSControlLock()) {
		return 3;
	}

	uint8_t status = 0;

	if (dvfs_enabled) {
		voltage_arbiter.forced_voltage = voltage;
		if (!DVFSChangeLocked()) {
			status = 3;
		}
	} else {
		/* restore to boot voltage */
		if (voltage == 0) {
			voltage = VDD_BOOT;
			voltage_arbiter.forced_voltage = 0U;
		}

		if (set_vcore(voltage) != 0) {
			status = 3;
		}
	}

	DVFSControlUnlock();
	return status;
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

REGISTER_MESSAGE(TT_SMC_MSG_FORCE_VDD, ForceVddHandler, MSGQUEUE_COMMAND_DENIED);
