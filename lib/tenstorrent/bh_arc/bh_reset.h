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

/* Returns the magic-qualified, non-zero DMC cable limit captured before ARC
 * boot. This lets board-power policy be installed before the later SMBus
 * initialization handshake.
 */
bool bh_get_boot_cable_power_limit(uint16_t *power_limit);

#endif /*BH_RESET*/
