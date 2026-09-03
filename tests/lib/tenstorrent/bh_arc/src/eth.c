/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/fff.h>
#include <zephyr/ztest.h>

#include "eth.h"
#include "init.h"
#include "reg_mock.h"

ZTEST(eth, test_failed_reset_holds_requested_tiles_and_riscs_in_hardware_reset)
{
	RESET_UNIT_ETH_RESET_reg_u initial = {.val = UINT32_MAX};
	RESET_UNIT_ETH_RESET_reg_u expected = initial;
	uint32_t mask = BIT(3) | BIT(9);

	ReadReg_fake.return_val = initial.val;
	EthTestHoldMaskInReset(mask);

	expected.f.eth_reset_n &= ~mask;
	expected.f.eth_risc_reset_n &= ~mask;
	zassert_equal(WriteReg_fake.call_count, 1U);
	zassert_equal(WriteReg_fake.arg0_val, RESET_UNIT_ETH_RESET_REG_ADDR);
	zassert_equal(WriteReg_fake.arg1_val, expected.val);
}

ZTEST_SUITE(eth, NULL, NULL, NULL, NULL, NULL);
