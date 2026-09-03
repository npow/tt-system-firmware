/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "throttler.h"
#include <tenstorrent/msgqueue.h>
#include <tenstorrent/smc_msg.h>

static uint8_t request_tensix_recovery(uint8_t command_code)
{
	union request req = {0};
	struct response rsp = {0};

	req.command_code = command_code;
	zassert_ok(msgqueue_request_push(0, &req));
	process_message_queues();
	zassert_ok(msgqueue_response_pop(0, &rsp));
	return rsp.data[0];
}

ZTEST(reset, test_latched_containment_blocks_all_tensix_recovery_commands)
{
	ThrottlerTestResetRuntimePowerGuard();
	zassert_true(ThrottlerTestUpdateRuntimePowerGuard(true, 300, 1000));

	zassert_equal(request_tensix_recovery(TT_SMC_MSG_TOGGLE_TENSIX_RESET), 2);
	zassert_equal(request_tensix_recovery(TT_SMC_MSG_TOGGLE_SINGLE_TENSIX_RESET), 2);
	zassert_equal(request_tensix_recovery(TT_SMC_MSG_REINIT_TENSIX), 2);

	ThrottlerTestResetRuntimePowerGuard();
}

ZTEST_SUITE(reset, NULL, NULL, NULL, NULL, NULL);
