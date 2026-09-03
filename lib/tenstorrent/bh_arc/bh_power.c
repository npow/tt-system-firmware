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
#include "bh_reset.h"
#include "gddr.h"
#include "init.h"
#include "reg.h"
#include "throttler.h"
#include "voltage.h"

LOG_MODULE_REGISTER(power, CONFIG_TT_APP_LOG_LEVEL);
static const struct device *pll4 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(pll4));

static bool power_state[BH_POWER_DOMAIN_COUNT] = {
	[BH_POWER_DOMAIN_AICLK] = false,
	[BH_POWER_DOMAIN_MRISC] = true,
	[BH_POWER_DOMAIN_TENSIX] = true,
	[BH_POWER_DOMAIN_L2CPU] = true,
};
#if defined(CONFIG_ZTEST)
static bool fail_next_force_safe_power_state;
static void (*power_enable_hook)(enum bh_power_domain domain);
#endif

static void run_power_enable_hook(enum bh_power_domain domain)
{
#if defined(CONFIG_ZTEST)
	if (power_enable_hook != NULL) {
		power_enable_hook(domain);
	}
#else
	ARG_UNUSED(domain);
#endif
}

int32_t bh_set_l2cpu_enable(bool enable)
{
	int32_t first_error = 0;

	if (enable) {
		for (uint32_t clock = CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_0;
		     clock <= CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_3; clock++) {
			int32_t ret = clock_control_on(pll4, (clock_control_subsys_t)clock);

			if (first_error == 0 && ret != 0) {
				first_error = ret;
			}
		}
	} else {
		for (uint32_t clock = CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_0;
		     clock <= CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_3; clock++) {
			int32_t ret = clock_control_off(pll4, (clock_control_subsys_t)clock);

			if (first_error == 0 && ret != 0) {
				first_error = ret;
			}
		}
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

int32_t bh_force_tensix_off(void)
{
	int32_t ret;

	if (power_state[BH_POWER_DOMAIN_TENSIX]) {
		bh_soft_reset_all_tensix();
		k_busy_wait(100);
	}

	ret = set_tensix_enable(false);
	if (ret == 0) {
		power_state[BH_POWER_DOMAIN_TENSIX] = false;
	}

	return ret;
}

int32_t bh_force_safe_power_state(void)
{
#if defined(CONFIG_ZTEST)
	if (fail_next_force_safe_power_state) {
		fail_next_force_safe_power_state = false;
		return -EIO;
	}
#endif
	/* This function is called from CalculateThrottlers(), inside DVFSChange().
	 * Latch the clock and voltage policy without recursively entering DVFS. The
	 * outer transaction lowers AICLK before it lowers VCORE.
	 */
	LatchAiclkPowerFault();
	LatchVoltagePowerFault();
	power_state[BH_POWER_DOMAIN_AICLK] = false;
	aiclk_update_busy();

	/* Stop a non-cooperative workload immediately. The in-tile soft reset is
	 * useful while tile clocks are running, but software reachable over PCIe can
	 * rewrite it. Holding reset_n low in the reset unit makes containment persist
	 * until a full ASIC reset. Tile, NoC, GDDR, Ethernet, ARC, and PCIe clocks are
	 * deliberately left running so outstanding host transactions can complete.
	 */
	if (power_state[BH_POWER_DOMAIN_TENSIX]) {
		bh_soft_reset_all_tensix();
		k_busy_wait(100);
	}

	bh_hold_tensix_riscs_in_reset();

	return 0;
}

#if defined(CONFIG_ZTEST)
void bh_test_fail_next_force_safe_power_state(void)
{
	fail_next_force_safe_power_state = true;
}

void bh_test_set_power_enable_hook(void (*hook)(enum bh_power_domain domain))
{
	power_enable_hook = hook;
}
#endif

bool bh_power_setting_is_safe_after_fault(const struct power_setting_rqst *power_setting)
{
	if (power_setting == NULL || power_setting->power_flags_valid > BH_POWER_DOMAIN_COUNT ||
	    power_setting->power_settings_valid != 0) {
		return false;
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_AICLK &&
	    power_setting->power_flags_bitfield.max_ai_clk) {
		return false;
	}
	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_MRISC &&
	    power_setting->power_flags_bitfield.mrisc_phy_power) {
		return false;
	}
	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_TENSIX &&
	    power_setting->power_flags_bitfield.tensix_enable) {
		return false;
	}
	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_L2CPU &&
	    power_setting->power_flags_bitfield.l2cpu_enable) {
		return false;
	}

	return true;
}

static bool power_setting_requests_high_power(const struct power_setting_rqst *power_setting)
{
	return (power_setting->power_flags_valid > BH_POWER_DOMAIN_AICLK &&
		power_setting->power_flags_bitfield.max_ai_clk) ||
	       (power_setting->power_flags_valid > BH_POWER_DOMAIN_MRISC &&
		power_setting->power_flags_bitfield.mrisc_phy_power) ||
	       (power_setting->power_flags_valid > BH_POWER_DOMAIN_TENSIX &&
		power_setting->power_flags_bitfield.tensix_enable) ||
	       (power_setting->power_flags_valid > BH_POWER_DOMAIN_L2CPU &&
		power_setting->power_flags_bitfield.l2cpu_enable);
}

static int32_t rollback_rising_power_domains(uint32_t rising_domains)
{
	int32_t first_error = 0;
	int32_t ret;

	/* Never undo a completed power-down. Only restore domains which this
	 * transaction attempted to raise from their prior low-power state.
	 */
	if ((rising_domains & BIT(BH_POWER_DOMAIN_AICLK)) != 0U) {
		power_state[BH_POWER_DOMAIN_AICLK] = false;
		aiclk_update_busy();
	}

	if ((rising_domains & BIT(BH_POWER_DOMAIN_TENSIX)) != 0U) {
		ret = bh_force_tensix_off();
		if (first_error == 0 && ret != 0) {
			first_error = ret;
		}
	}

	if ((rising_domains & BIT(BH_POWER_DOMAIN_L2CPU)) != 0U) {
		ret = bh_set_l2cpu_enable(false);
		if (ret == 0) {
			power_state[BH_POWER_DOMAIN_L2CPU] = false;
		} else if (first_error == 0) {
			first_error = ret;
		}
	}

	if ((rising_domains & BIT(BH_POWER_DOMAIN_MRISC)) != 0U) {
		ret = set_mrisc_power_setting(false);
		if (ret == 0) {
			power_state[BH_POWER_DOMAIN_MRISC] = false;
		} else if (first_error == 0) {
			first_error = ret;
		}
	}

	return first_error;
}

static void record_first_error(int32_t *first_error, int32_t ret)
{
	if (*first_error == 0 && ret != 0) {
		*first_error = ret;
	}
}

static int32_t apply_power_settings(const struct power_setting_rqst *power_setting)
{
	int32_t first_error = 0;
	uint32_t rising_domains = 0U;
	bool staged_high_power = false;

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_COUNT ||
	    power_setting->power_settings_valid != 0) {
		return -EINVAL;
	}

	/* The hardware safety state is reset-latched. Continue accepting pure
	 * power-down requests so the host can reduce the residual board load, but
	 * reject the complete request if any supplied field could raise power.
	 */
	if (ThrottlerRuntimePowerFaultLatched() &&
	    !bh_power_setting_is_safe_after_fault(power_setting)) {
		LOG_ERR("Refusing to leave whole-board safety state before reset");
		return -EPERM;
	}

	/* A legacy client can request every retained domain in one message. Force
	 * the physical AICLK to the reset-safe point before honoring any supplied
	 * high-power flag, even if the tracked state says that domain was already on.
	 */
	if (power_setting_requests_high_power(power_setting)) {
		if (SetAiclkResetSafe(true) != 0 ||
		    GetAiclkCurrent() > (uint32_t)AICLK_RESET_SAFE_FREQ) {
			LOG_ERR("Failed to stage high-power transition at reset-safe AICLK");
			return -EIO;
		}
		if (!ThrottlerComputePowerPolicyReady()) {
			return -EPERM;
		}
		ThrottlerBeginHighPowerTransition();
		staged_high_power = true;
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_AICLK) {
		bool enable = power_setting->power_flags_bitfield.max_ai_clk;

		if (!enable) {
			power_state[BH_POWER_DOMAIN_AICLK] = false;
			aiclk_update_busy();
		} else if (first_error != 0) {
			/* Do not start another rising transition after one has failed. */
		} else if (ThrottlerComputePowerPolicyReady()) {
			if (!power_state[BH_POWER_DOMAIN_AICLK]) {
				rising_domains |= BIT(BH_POWER_DOMAIN_AICLK);
			}
			power_state[BH_POWER_DOMAIN_AICLK] = enable;
			aiclk_update_busy();
			run_power_enable_hook(BH_POWER_DOMAIN_AICLK);
			if (!ThrottlerComputePowerPolicyReady()) {
				first_error = -EPERM;
			}
		} else {
			first_error = -EPERM;
		}
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_TENSIX) {
		bool enable = power_setting->power_flags_bitfield.tensix_enable;
		int32_t ret;

		if (enable && first_error != 0) {
			ret = first_error;
		} else if (enable && !ThrottlerComputePowerPolicyReady()) {
			ret = -EPERM;
		} else if (enable) {
			if (!power_state[BH_POWER_DOMAIN_TENSIX]) {
				rising_domains |= BIT(BH_POWER_DOMAIN_TENSIX);
			}
			ret = set_tensix_enable(true);
			if (ret == 0) {
				power_state[BH_POWER_DOMAIN_TENSIX] = true;
			}
			run_power_enable_hook(BH_POWER_DOMAIN_TENSIX);
			if (ret == 0 && !ThrottlerComputePowerPolicyReady()) {
				ret = -EPERM;
			}
		} else {
			ret = bh_force_tensix_off();
		}
		record_first_error(&first_error, ret);

		/*Note, if we're turning on the tensixes, we don't take them out of reset,
		 *we just lift the clock gating.
		 */
	}
	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_L2CPU) {
		bool enable = power_setting->power_flags_bitfield.l2cpu_enable;
		int32_t ret = -EPERM;

		if (!enable) {
			ret = bh_set_l2cpu_enable(enable);
		} else if (first_error != 0) {
			ret = first_error;
		} else if (ThrottlerComputePowerPolicyReady()) {
			if (!power_state[BH_POWER_DOMAIN_L2CPU]) {
				rising_domains |= BIT(BH_POWER_DOMAIN_L2CPU);
			}
			ret = bh_set_l2cpu_enable(true);
			run_power_enable_hook(BH_POWER_DOMAIN_L2CPU);
			if (ret == 0 && !ThrottlerComputePowerPolicyReady()) {
				ret = -EPERM;
			}
		}

		if (ret == 0) {
			power_state[BH_POWER_DOMAIN_L2CPU] = enable;
		}
		record_first_error(&first_error, ret);
	}

	if (power_setting->power_flags_valid > BH_POWER_DOMAIN_MRISC) {
		bool enable = power_setting->power_flags_bitfield.mrisc_phy_power;
		int32_t ret = -EPERM;

		if (!enable) {
			ret = set_mrisc_power_setting(enable);
		} else if (first_error != 0) {
			ret = first_error;
		} else if (ThrottlerComputePowerPolicyReady()) {
			if (!power_state[BH_POWER_DOMAIN_MRISC]) {
				rising_domains |= BIT(BH_POWER_DOMAIN_MRISC);
			}
			ret = set_mrisc_power_setting(true);
			run_power_enable_hook(BH_POWER_DOMAIN_MRISC);
			if (ret == 0 && !ThrottlerComputePowerPolicyReady()) {
				ret = -EPERM;
			}
		}

		if (ret == 0) {
			power_state[BH_POWER_DOMAIN_MRISC] = enable;
		}
		record_first_error(&first_error, ret);
	}

	/* A failed/aborted transition stays at reset-safe AICLK. Successful domain
	 * enables may resume normal arbitration only while the safety latch is clear.
	 */
	if (staged_high_power && first_error == 0 &&
	    !ThrottlerComputePowerPolicyReady()) {
		first_error = -EPERM;
	}
	if (staged_high_power && first_error == 0 &&
	    !ThrottlerFinishHighPowerTransition()) {
		first_error = -EPERM;
	}
	if (staged_high_power && first_error == 0) {
		first_error = SetAiclkResetSafe(false);
	}
	if (staged_high_power && first_error == 0 &&
	    !ThrottlerComputePowerPolicyReady()) {
		first_error = -EPERM;
	}

	if (staged_high_power && first_error != 0) {
		int32_t safe_error = SetAiclkResetSafe(true);
		int32_t rollback_error = rollback_rising_power_domains(rising_domains);

		ThrottlerAbortHighPowerTransition();
		if (safe_error != 0 || rollback_error != 0) {
			LOG_ERR("Failed to roll back high-power transition (%d, %d)",
				safe_error, rollback_error);
			ThrottlerRetryRuntimeContainment();
		}
	}

	return first_error;
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
