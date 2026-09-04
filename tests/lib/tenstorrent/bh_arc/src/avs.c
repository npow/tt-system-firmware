/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "avs.h"
#include "reg_mock.h"

#define AVS_CMD_REG          0x80100000U
#define AVS_READBACK_REG     0x80100004U
#define AVS_FIFOS_STATUS_REG 0x80100028U
#define AVS_CFG_0_REG        0x80100050U
#define AVS_CMD_VACANT       0x00000100U
#define AVS_READBACK_READY   0x00010000U
#define AVS_ERROR_DATA       0xffffU

static uint32_t successful_read(uint32_t addr)
{
	switch (addr) {
	case AVS_FIFOS_STATUS_REG:
		return AVS_CMD_VACANT | AVS_READBACK_READY;
	case AVS_CFG_0_REG:
		return 0;
	case AVS_READBACK_REG:
		return 0x1234U << 8;
	default:
		return 0;
	}
}

static void avs_before(void *fixture)
{
	ARG_UNUSED(fixture);
	AVSTestResetState();
}

ZTEST(avs, test_successful_read)
{
	uint16_t voltage = 0;

	ReadReg_fake.custom_fake = successful_read;
	zassert_equal(AVSReadVoltage(AVS_VCORE_RAIL, &voltage), AVSOk);
	zassert_equal(voltage, 0x1234);
	zassert_equal(WriteReg_fake.call_count, 1);
	zassert_equal(WriteReg_fake.arg0_history[0], AVS_CMD_REG);
}

ZTEST(avs, test_command_fifo_timeout_does_not_issue_command)
{
	uint16_t voltage = 0;

	ReadReg_fake.return_val = 0;
	zassert_equal(AVSReadVoltage(AVS_VCORE_RAIL, &voltage), AVSTimeout);
	zassert_equal(voltage, AVS_ERROR_DATA);
	zassert_equal(WriteReg_fake.call_count, 0);

	/* No command entered the FIFO, so this timeout did not desynchronize it. */
	ReadReg_fake.custom_fake = successful_read;
	zassert_equal(AVSReadVoltage(AVS_VCORE_RAIL, &voltage), AVSOk);
	zassert_equal(voltage, 0x1234);
}

static uint32_t status_reads;

static uint32_t missing_readback(uint32_t addr)
{
	if (addr == AVS_FIFOS_STATUS_REG) {
		return status_reads++ == 0 ? AVS_CMD_VACANT : 0;
	}

	return 0;
}

ZTEST(avs, test_readback_fifo_timeout)
{
	uint16_t voltage = 0;

	status_reads = 0;
	ReadReg_fake.custom_fake = missing_readback;
	zassert_equal(AVSReadVoltage(AVS_VCORE_RAIL, &voltage), AVSTimeout);
	zassert_equal(voltage, AVS_ERROR_DATA);
	zassert_equal(WriteReg_fake.call_count, 1);

	/* A late response from the timed-out command must never acknowledge the
	 * next voltage transaction.
	 */
	ReadReg_fake.custom_fake = successful_read;
	zassert_equal(AVSReadVoltage(AVS_VCORE_RAIL, &voltage), AVSTimeout);
	zassert_equal(WriteReg_fake.call_count, 1);
}

ZTEST_SUITE(avs, NULL, NULL, avs_before, NULL, NULL);
