/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BH_RESET
#define BH_RESET

#include <stdbool.h>
#include <stdint.h>

void bh_soft_reset_all_tensix(void);

/**
 * @brief Check if the system is in cable fault mode
 *
 * Cable fault mode is entered when DMC reports 0W power limit (no cable or
 * improper installation). In this mode, all tiles except column 15 are clock-gated via
 * NIU_CFG_0 TILE_CLK_OFF to minimize power draw while the full NOC mesh
 * and ARC-PCIe path remain available for host communication.
 *
 * @return true if cable fault mode is active, false otherwise
 */
bool is_cable_fault_mode(void);

/**
 * @brief Get the cable power limit captured before SMC hardware bring-up
 *
 * The DMC writes this value, with a magic marker, before releasing the ARC.
 * Keeping the decoded value allows the throttler to install the electrical
 * policy without waiting for the later DMC ready-message handshake.
 *
 * @param[out] power_limit Cable input power limit in watts
 * @return true when a magic-qualified value was captured, false otherwise
 */
bool bh_get_boot_cable_power_limit(uint16_t *power_limit);

/** Stage and verify the physical AICLK ceiling required by runtime reset flows. */
int bh_reset_safe_aiclk_acquire(void);

/**
 * Release the reset ceiling only after a successful flow with a ready policy.
 *
 * @return -EPERM when containment requires the ceiling to remain asserted.
 */
int bh_reset_safe_aiclk_release(void);

#if defined(CONFIG_ZTEST)
void bh_test_set_boot_cable_power_limit(bool valid, uint16_t power_limit);
#endif

#endif /*BH_RESET*/
