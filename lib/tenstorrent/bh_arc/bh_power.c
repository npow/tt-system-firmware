/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <tenstorrent/bh_power.h>
#include <tenstorrent/smc_msg.h>
#include <tenstorrent/msgqueue.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_tt_bh.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

#include "noc_init.h"
#include "aiclk_ppm.h"
#include "dvfs.h"
#include "gddr.h"
#include "bh_reset.h"

LOG_MODULE_REGISTER(power, CONFIG_TT_APP_LOG_LEVEL);
static const struct device *pll4 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(pll4));

static bool power_state[BH_POWER_DOMAIN_COUNT] = {
	[BH_POWER_DOMAIN_AICLK] = false,
	[BH_POWER_DOMAIN_MRISC] = true,
	[BH_POWER_DOMAIN_TENSIX] = true,
	[BH_POWER_DOMAIN_L2CPU] = true,
};

static int32_t apply_aiclk_busy_setting(bool enable)
{
	if (!DVFSControlLock()) {
		return -1;
	}

	bool previous = power_state[BH_POWER_DOMAIN_AICLK];

	power_state[BH_POWER_DOMAIN_AICLK] = enable;
	aiclk_update_busy_locked();

	int32_t ret = 0;

	if (dvfs_enabled && !DVFSChangeLocked()) {
		/* Roll back a failed enable, but retain a failed disable as an idle
		 * request. Even if the immediate safe-state DVFS attempt also fails, a
		 * later 1 ms tick cannot observe a new busy request and raise AICLK.
		 */
		power_state[BH_POWER_DOMAIN_AICLK] = previous && enable;
		aiclk_update_busy_locked();
		(void)DVFSChangeLocked();
		ret = -1;
	}

	DVFSControlUnlock();
	return ret;
}

int32_t bh_set_l2cpu_enable(bool enable)
{
	/* L2CPU hosts PCIe/management services, and there is no protocol to
	 * quiesce those users before gating its clocks. Keep the runtime API
	 * fail-safe until such a protocol exists.
	 */
	if (!enable) {
		LOG_WRN("Rejecting unsafe runtime L2CPU clock disable");
		return -EPERM;
	}

	int32_t ret = 0;
	int32_t first_error = 0;

	ret = clock_control_on(pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_0);
	if (ret != 0 && first_error == 0) {
		first_error = ret;
	}
	ret = clock_control_on(pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_1);
	if (ret != 0 && first_error == 0) {
		first_error = ret;
	}
	ret = clock_control_on(pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_2);
	if (ret != 0 && first_error == 0) {
		first_error = ret;
	}
	ret = clock_control_on(pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_3);
	if (ret != 0 && first_error == 0) {
		first_error = ret;
	}

	return first_error;
}

int bh_power_state_get(enum bh_power_domain domain, bool *state)
{
	if (domain >= BH_POWER_DOMAIN_COUNT) {
		return -EINVAL;
	}

	*state = power_state[domain];
	return 0;
}

static int32_t apply_power_settings(const struct power_setting_rqst *power_setting)
{
	int32_t op_ret;

	/* Validate the entire mixed request before any register or NoC access. */
	/* KMD deliberately sends validity 15 and defaults unknown future flags to
	 * one for forward compatibility. Apply only the four fields understood by
	 * this firmware and reject numeric settings, which remain unsupported.
	 */
	if (power_setting->power_settings_valid != 0U) {
		return -EINVAL;
	}
	/* A host process closing is not proof that autonomous work or NoC/GDDR
	 * traffic has drained. Until a quiesce handshake exists, runtime power
	 * requests may idle AICLK but must retain every dependent power domain.
	 */
	if ((power_setting->power_flags_valid > BH_POWER_DOMAIN_MRISC &&
	     !power_setting->power_flags_bitfield.mrisc_phy_power) ||
	    (power_setting->power_flags_valid > BH_POWER_DOMAIN_TENSIX &&
	     !power_setting->power_flags_bitfield.tensix_enable)) {
		return -EPERM;
	}
	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_L2CPU &&
	    !power_setting->power_flags_bitfield.l2cpu_enable) {
		return -EPERM;
	}

	/* Wake management dependencies first. If an enable later fails, the
	 * retained partial result is deliberately reset-safe: dependencies and
	 * Tensix are only left more enabled, and AICLK is committed last.
	 */
	bool mrisc_change = power_setting->power_flags_valid > BH_POWER_DOMAIN_MRISC &&
			    power_setting->power_flags_bitfield.mrisc_phy_power !=
				    power_state[BH_POWER_DOMAIN_MRISC];

	if (mrisc_change && power_setting->power_flags_bitfield.mrisc_phy_power) {
		op_ret = set_mrisc_power_setting(
			power_setting->power_flags_bitfield.mrisc_phy_power);
		if (op_ret != 0) {
			return op_ret;
		}
		power_state[BH_POWER_DOMAIN_MRISC] =
			power_setting->power_flags_bitfield.mrisc_phy_power;
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_L2CPU) {
		op_ret = bh_set_l2cpu_enable(true);
		if (op_ret != 0) {
			return op_ret;
		}
		power_state[BH_POWER_DOMAIN_L2CPU] = true;
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_TENSIX) {
		if (power_setting->power_flags_bitfield.tensix_enable !=
		    power_state[BH_POWER_DOMAIN_TENSIX]) {
			op_ret = set_tensix_enable(
				power_setting->power_flags_bitfield.tensix_enable);
			if (op_ret == 0) {
				power_state[BH_POWER_DOMAIN_TENSIX] =
					power_setting->power_flags_bitfield.tensix_enable;
			} else {
				return op_ret;
			}
		}

		/*Note, if we're turning on the tensixes, we don't take them out of reset,
		 *we just lift the clock gating.
		 */
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_AICLK) {
		op_ret = apply_aiclk_busy_setting(power_setting->power_flags_bitfield.max_ai_clk);
		if (op_ret != 0) {
			return op_ret;
		}
	}

	return 0;
}

/** @brief Handles the request to set the power settings
 * @param[in] request The request, of type @ref power_setting_rqst, with command code
 *	@ref TT_SMC_MSG_POWER_SETTING
 * @param[out] response The response to the host
 * @return 0 for success. 1 for Failure.
 */
static uint8_t power_setting_msg_handler(const union request *request, struct response *response)
{
	const struct power_setting_rqst *power_setting = &request->power_setting;

	uint8_t status = apply_power_settings(power_setting) != 0;

	LOG_DBG("Power State: GDDR-%u Tensix-%u AICLK-%u, L2CPU-%u",
		power_state[BH_POWER_DOMAIN_MRISC], power_state[BH_POWER_DOMAIN_TENSIX],
		power_state[BH_POWER_DOMAIN_AICLK], power_state[BH_POWER_DOMAIN_L2CPU]);

	return status;
}

REGISTER_MESSAGE(TT_SMC_MSG_POWER_SETTING, power_setting_msg_handler, MSGQUEUE_COMMAND_MUTATING);
