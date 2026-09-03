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

int32_t bh_set_l2cpu_enable(bool enable);

/**
 * @brief Assert reset and clock-gate every Tensix core.
 *
 * This is reserved for host-coordinated legacy power-down. Runtime board-power
 * containment keeps every PCIe-addressable target clocked.
 *
 * @retval 0 On success
 * @retval negative errno On failure
 */
int32_t bh_force_tensix_off(void);

/**
 * @brief Enter a reset-latched low-power safety state.
 *
 * Holds the Tensix compute engines and RISCs in soft reset and clamps AICLK
 * without clock- or power-gating any PCIe-addressable target. GDDR, L2CPU, ARC,
 * PCIe, telemetry, and fan control remain active so in-flight host transactions
 * can complete and the host can inspect and reset the card safely.
 *
 * @retval 0 On success
 */
int32_t bh_force_safe_power_state(void);

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
