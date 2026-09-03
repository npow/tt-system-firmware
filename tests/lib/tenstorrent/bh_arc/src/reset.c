/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "aiclk_ppm.h"
#include "bh_reset.h"
#include "init.h"
#include "reg_mock.h"
#include "status_reg.h"
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

static uint32_t request_value;
static uint32_t ack_value;
static uint32_t first_request;
static bool return_matching_ack;
static bool return_stale_ack;

static void write_reg_handshake(uint32_t addr, uint32_t value)
{
	if (addr == TENSIX_RESET_REQUEST_REG_ADDR) {
		request_value = value;
		if (first_request == 0U) {
			first_request = value;
		}
	} else if (addr == TENSIX_RESET_ACK_REG_ADDR) {
		ack_value = value;
	}
}

static uint32_t read_reg_handshake(uint32_t addr)
{
	if (addr == TENSIX_RESET_REQUEST_REG_ADDR) {
		return request_value;
	}
	if (addr != TENSIX_RESET_ACK_REG_ADDR) {
		return 0U;
	}
	if (return_matching_ack) {
		ack_value = request_value;
	}
	if (return_stale_ack) {
		return request_value - 1U;
	}
	return ack_value;
}

static void reset_handshake_fakes(void)
{
	request_value = 0U;
	ack_value = 0U;
	first_request = 0U;
	return_matching_ack = false;
	return_stale_ack = false;
	ReadReg_fake.custom_fake = read_reg_handshake;
	WriteReg_fake.custom_fake = write_reg_handshake;
	ResetTestResetHostQuiesceState();
}

ZTEST(reset, test_host_quiesce_requires_exact_request_token)
{
	reset_handshake_fakes();
	return_matching_ack = true;

	zassert_true(ResetTestRequestHostTensixQuiesce());
	zassert_equal(first_request, 1U);
	zassert_equal(request_value, 1U);
	zassert_equal(ack_value, 1U);
}

ZTEST(reset, test_host_quiesce_recovers_reserved_transaction_values)
{
	reset_handshake_fakes();
	request_value = UINT32_MAX;
	ack_value = UINT32_MAX;
	return_matching_ack = true;

	zassert_true(ResetTestRequestHostTensixQuiesce());
	zassert_equal(first_request, 1U);
	zassert_equal(request_value, 1U);
	zassert_equal(ack_value, 1U);
}

ZTEST(reset, test_stale_host_quiesce_ack_is_rejected)
{
	reset_handshake_fakes();
	return_stale_ack = true;

	zassert_false(ResetTestRequestHostTensixQuiesce());
	zassert_equal(first_request, 1U);
	zassert_equal(request_value, 1U);
	zassert_equal(ack_value, 0U);
}

ZTEST(reset, test_host_quiesce_timeout_latches_safe_containment)
{
	reset_handshake_fakes();
	/* Avoid coupling this reset protocol test to persistent DVFS state left by
	 * the msgqueue voltage tests. The 0xAF handler must preserve an already
	 * asserted reset-safe clock cap.
	 */
	AiclkTestSetResetSafeState(true);

	zassert_equal(request_tensix_recovery(TT_SMC_MSG_TOGGLE_TENSIX_RESET), 3U);
	zassert_true(ResetTestUnsafeTensixResetLatched());
	zassert_true(AiclkTestResetSafeEnabled());
	zassert_equal(request_tensix_recovery(TT_SMC_MSG_TOGGLE_SINGLE_TENSIX_RESET), 3U);
	zassert_equal(request_tensix_recovery(TT_SMC_MSG_REINIT_TENSIX), 3U);

	for (uint32_t i = 0U; i < WriteReg_fake.call_count; ++i) {
		uint32_t addr = WriteReg_fake.arg0_history[i];

		zassert_false(addr >= RESET_UNIT_TENSIX_RESET_0_REG_ADDR &&
			      addr < RESET_UNIT_TENSIX_RESET_0_REG_ADDR + 8U * sizeof(uint32_t),
			      "destructive tile reset occurred without host acknowledgment");
	}
}

ZTEST(reset, test_successful_whole_reset_clears_timeout_containment)
{
	reset_handshake_fakes();
	AiclkTestSetResetSafeState(true);
	zassert_equal(request_tensix_recovery(TT_SMC_MSG_TOGGLE_TENSIX_RESET), 3U);
	zassert_true(ResetTestUnsafeTensixResetLatched());

	return_matching_ack = true;
	zassert_equal(request_tensix_recovery(TT_SMC_MSG_TOGGLE_TENSIX_RESET), 0U);
	zassert_false(ResetTestUnsafeTensixResetLatched());
	zassert_equal(request_value, 2U);
	zassert_equal(ack_value, 2U);
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
