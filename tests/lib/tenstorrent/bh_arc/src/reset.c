/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/fff.h>
#include <zephyr/ztest.h>

#include "init.h"
#include "reg_mock.h"

ZTEST(reset, test_init_failure_holds_all_programmable_riscs)
{
	uint32_t saved_error_status = error_status0;

	error_status0 = 0U;
	record_init_failure(INIT_STAGE_ETH);

	zassert_equal(error_status0, BIT(INIT_STAGE_ETH));
	zassert_equal(WriteReg_fake.call_count, 11U);
	zexpect_equal(WriteReg_fake.arg0_history[0], STATUS_ERROR_STATUS0_REG_ADDR);
	zexpect_equal(WriteReg_fake.arg1_history[0], BIT(INIT_STAGE_ETH));
	for (uint32_t i = 0; i < 8U; i++) {
		zexpect_equal(WriteReg_fake.arg0_history[i + 1U],
			      RESET_UNIT_TENSIX_RISC_RESET_0_REG_ADDR + i * sizeof(uint32_t));
		zexpect_equal(WriteReg_fake.arg1_history[i + 1U], 0U);
	}
	zexpect_equal(WriteReg_fake.arg0_history[9], RESET_UNIT_ETH_RESET_REG_ADDR);
	zexpect_equal(WriteReg_fake.arg1_history[9], 0U);
	zexpect_equal(WriteReg_fake.arg0_history[10], RESET_UNIT_DDR_RESET_REG_ADDR);
	zexpect_equal(WriteReg_fake.arg1_history[10], 0U);

	error_status0 = saved_error_status;
}

ZTEST_SUITE(reset, NULL, NULL, NULL, NULL, NULL);
