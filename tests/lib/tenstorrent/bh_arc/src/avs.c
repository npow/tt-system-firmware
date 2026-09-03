/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "avs.h"
#include "reg_mock.h"

ZTEST(avs, test_wedged_fifo_returns_fail_high)
{
	float current = 0.0F;

	/* A zero FIFO-status register models a controller that never accepts a
	 * command. The call must return instead of wedging the DVFS workqueue, and
	 * current telemetry must fail high rather than appearing safe.
	 */
	ReadReg_fake.return_val = 0U;

	zassert_equal(AVSReadCurrent(AVS_VCORE_RAIL, &current), AVSTimeout);
	zassert_true(current > 600.0F, "failed AVS current read did not fail high");
}

ZTEST_SUITE(avs, NULL, NULL, NULL, NULL, NULL);
