/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BH_POWER_H
#define BH_POWER_H

#include <stdbool.h>
#include <stdint.h>

enum bh_power_domain {
	BH_POWER_DOMAIN_AICLK,
	BH_POWER_DOMAIN_MRISC,
	BH_POWER_DOMAIN_TENSIX,
	BH_POWER_DOMAIN_L2CPU,
	BH_POWER_DOMAIN_COUNT,
};

struct power_setting_rqst;

int32_t bh_set_l2cpu_enable(bool enable);

/**
 * @brief Assert reset and clock-gate every Tensix core.
 *
 * Used for normal low-power requests. Runtime fault containment uses a
 * separate hardware RISC-reset hold while leaving tile clocks accessible.
 *
 * @retval 0 On success
 * @retval negative errno On failure
 */
int32_t bh_force_tensix_off(void);

/**
 * @brief Hold all Tensix RISC-V cores in reset without resetting their tiles.
 *
 * This performs only reset-unit APB writes and is safe to call from an ISR.
 * Runtime fault containment never releases this hold; the boot path may release
 * it only for the bounded destination-register wipe and after policy readiness.
 */
void bh_hold_tensix_riscs_in_reset(void);

/**
 * @brief Release all Tensix RISC-V hardware reset holds during safe boot.
 *
 * This low-level helper has no policy checks. It is reserved for the bounded
 * boot wipe and the final, interrupt-serialized init readiness transition.
 */
void bh_release_tensix_riscs_from_reset(void);

/**
 * @brief Enter a reset-latched whole-board low-power safety state.
 *
 * Clamps AICLK and resets the Tensix RISC-V cores. All host-addressable tile,
 * NoC, GDDR, Ethernet, ARC, and PCIe clocks remain active so in-flight PCIe
 * transactions can complete and the host can inspect and reset the card.
 *
 * @retval 0 On success
 */
int32_t bh_force_safe_power_state(void);
#if defined(CONFIG_ZTEST)
void bh_test_fail_next_force_safe_power_state(void);
void bh_test_set_power_enable_hook(void (*hook)(enum bh_power_domain domain));
#endif

/**
 * @brief Check whether a power request can only lower power after a fault.
 *
 * A request is safe only when every supplied flag is zero and no unsupported
 * power-setting fields are present.
 */
bool bh_power_setting_is_safe_after_fault(const struct power_setting_rqst *power_setting);

/**
 * @brief Returns the power state of the specified domain
 *
 * @param[out] state True if power state is high (busy for AICLK),
 *                   false if power state is low (idle for AICLK)
 *
 * @retval 0 On success
 * @retval -EINVAL If domain is invalid
 */
int bh_power_state_get(enum bh_power_domain domain, bool *state);

#endif
