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
#include "gddr.h"
#include "bh_reset.h"
#include "throttler.h"

LOG_MODULE_REGISTER(power, CONFIG_TT_APP_LOG_LEVEL);
static const struct device *pll4 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(pll4));

static bool power_state[BH_POWER_DOMAIN_COUNT] = {
	[BH_POWER_DOMAIN_AICLK] = false,
	[BH_POWER_DOMAIN_MRISC] = true,
	[BH_POWER_DOMAIN_TENSIX] = true,
	[BH_POWER_DOMAIN_L2CPU] = true,
};

int32_t bh_set_l2cpu_enable(bool enable)
{
	int32_t ret = 0;

	if (enable) {
		ret = clock_control_on(
			pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_0);
		ret = clock_control_on(
			pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_1);
		ret = clock_control_on(
			pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_2);
		ret = clock_control_on(
			pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_3);
	} else {
		ret = clock_control_off(
			pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_0);
		ret = clock_control_off(
			pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_1);
		ret = clock_control_off(
			pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_2);
		ret = clock_control_off(
			pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_3);
	}

	return ret;
}

int bh_power_state_get(enum bh_power_domain domain, bool *state)
{
	if (domain >= BH_POWER_DOMAIN_COUNT) {
		return -EINVAL;
	}

	*state = power_state[domain];
	return 0;
}

int32_t bh_force_tensix_off(void)
{
	int32_t ret;

	if (power_state[BH_POWER_DOMAIN_TENSIX]) {
		bh_soft_reset_all_tensix();
		k_usleep(100);
	}

	ret = set_tensix_enable(false);
	if (ret == 0) {
		power_state[BH_POWER_DOMAIN_TENSIX] = false;
	}

	return ret;
}

int32_t bh_force_safe_power_state(void)
{
	int32_t first_error = bh_force_tensix_off();
	int32_t ret;

	/* Stop asking PPM for the busy AICLK floor before removing the memory
	 * domain. The strict power arbiter remains at Fmin independently.
	 */
	power_state[BH_POWER_DOMAIN_AICLK] = false;
	aiclk_update_busy();

	/* Let MRISC place each GDDR PHY into its supported low-power state. Keep
	 * the NoC fabric active: ARC telemetry, fan control, and PCIe recovery all
	 * depend on that fabric remaining responsive after the fault is latched.
	 */
	ret = set_mrisc_power_setting(false);
	if (ret == 0) {
		power_state[BH_POWER_DOMAIN_MRISC] = false;
	} else if (first_error == 0) {
		first_error = ret;
	}

	return first_error;
}

static int32_t apply_power_settings(const struct power_setting_rqst *power_setting)
{
	int32_t ret = 0;

	/* The hardware safety state is reset-latched. Reject every power-setting
	 * request so no partial or legacy request can raise a gated domain.
	 */
	if (ThrottlerRuntimePowerFaultLatched()) {
		LOG_ERR("Refusing to leave low-power safety state before reset");
		return -EPERM;
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_AICLK) {
		power_state[BH_POWER_DOMAIN_AICLK] = power_setting->power_flags_bitfield.max_ai_clk;
		aiclk_update_busy();
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_TENSIX) {
		bool enable = power_setting->power_flags_bitfield.tensix_enable;

		if (enable) {
			ret = set_tensix_enable(true);
			if (ret == 0) {
				power_state[BH_POWER_DOMAIN_TENSIX] = true;
			}
		} else {
			ret = bh_force_tensix_off();
		}

		/*Note, if we're turning on the tensixes, we don't take them out of reset,
		 *we just lift the clock gating.
		 */
	}
	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_L2CPU) {
		ret = bh_set_l2cpu_enable(power_setting->power_flags_bitfield.l2cpu_enable);
		power_state[BH_POWER_DOMAIN_L2CPU] =
			power_setting->power_flags_bitfield.l2cpu_enable;
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_MRISC) {
		ret = set_mrisc_power_setting(power_setting->power_flags_bitfield.mrisc_phy_power);
		power_state[BH_POWER_DOMAIN_MRISC] =
			power_setting->power_flags_bitfield.mrisc_phy_power;
	}

	return ret;
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
	int32_t ret;

	ret = apply_power_settings(power_setting);
	LOG_DBG("Power State: GDDR-%u Tensix-%u AICLK-%u, L2CPU-%u",
		power_state[BH_POWER_DOMAIN_MRISC], power_state[BH_POWER_DOMAIN_TENSIX],
		power_state[BH_POWER_DOMAIN_AICLK], power_state[BH_POWER_DOMAIN_L2CPU]);

	return ret == 0 ? 0 : 1;
}

REGISTER_MESSAGE(TT_SMC_MSG_POWER_SETTING, power_setting_msg_handler);
