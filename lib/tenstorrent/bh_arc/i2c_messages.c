/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <tenstorrent/smc_msg.h>
#include <tenstorrent/msgqueue.h>
#include "dw_apb_i2c.h"

/*
 * Response Buffer
 * |   | 0            | 1           | 2        | 3             |
 * |---|--------------|-------------|----------|---------------|
 * | 0 | status       | unused      | unused   | unused        |
 * | 1 | Read Data (28B)                                       |
 * | 2 |                                                       |
 * | 3 |                                                       |
 * | 4 |                                                       |
 * | 5 |                                                       |
 * | 6 |                                                       |
 * | 7 |                                                       |
 */

/**
 * @brief Handler for TT_SMC_MSG_I2C_MESSAGE messages
 *
 * @details Performs I2C read/write transactions. The message can contain both write
 *          and read operations in a single transaction.
 *
 * @param request Pointer to the host request message, use request->i2c_message for structured
 *                access
 * @param response Pointer to the response message to be sent back to host, will contain:
 *                 - Read data if read operation was requested
 *
 * @return 0 on success
 * @return non-zero on error
 *
 * @see i2c_message_rqst_t
 */
static uint8_t i2c_message_handler(const union request *request, struct response *response)
{
	uint8_t I2C_mst_id = request->i2c_message.i2c_mst_id;
	bool valid_id = IsValidI2CMasterId(I2C_mst_id);

	ARG_UNUSED(response);

	if (!valid_id) {
		return !valid_id;
	}

	/* Every Blackhole SMC I2C controller reaches a safety-critical function:
	 * I2C0 is the live DMC telemetry target, I2C1 controls the PMBus regulators,
	 * and I2C2 controls board power switches. A generic raw transaction can
	 * therefore disable the power monitor or rails and bypass the DVFS/power
	 * policy. Keep the message ABI visible, but reject it before any controller
	 * or pad register is touched. Dedicated bounded messages provide the safe
	 * runtime controls.
	 */
	return 1;
}

REGISTER_MESSAGE(TT_SMC_MSG_I2C_MESSAGE, i2c_message_handler, MSGQUEUE_COMMAND_DENIED);
