/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "avs.h"
#include "dvfs.h"
#include "dw_apb_i2c.h"
#include "init.h"
#include "regulator.h"
#include "regulator_config.h"
#include "status_reg.h"
#include "timer.h"

#include <math.h>  /* for ldexp */
#include <float.h> /* for FLT_MAX */
#include <stdint.h>

#include <tenstorrent/smc_msg.h>
#include <tenstorrent/msgqueue.h>
#include <tenstorrent/post_code.h>
#include <tenstorrent/sys_init_defines.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/bh_fwtable.h>

#define LINEAR_FORMAT_CONSTANT (1 << 9)
#define SCALE_LOOP             0.335f

/* I2C constants */
#define PMBUS_MST_ID 1

/* PMBus Spec constants */
#define MFR_CTRL_OPS                   0xD2
#define MFR_CTRL_OPS_DATA_BYTE_SIZE    1
#define VOUT_COMMAND                   0x21
#define VOUT_COMMAND_DATA_BYTE_SIZE    2
#define VOUT_SCALE_LOOP                0x29
#define VOUT_SCALE_LOOP_DATA_BYTE_SIZE 2
#define READ_VOUT                      0x8B
#define READ_VOUT_DATA_BYTE_SIZE       2
#define READ_IOUT                      0x8C
#define READ_IOUT_DATA_BYTE_SIZE       2
#define READ_IOUT_DIRECT_MASK          0x3FFF  /* 14-bit raw IOUT field */
#define GDDRIO_IOUT_LSB_A              0.0625f /* 62.5 mA per LSB */
#define GDDR_IO_RAIL_VOLTAGE           1.35f   /* V - fixed IO rail supply */
#define READ_POUT                      0x96
#define READ_POUT_DATA_BYTE_SIZE       2
#define OPERATION                      0x1
#define OPERATION_DATA_BYTE_SIZE       1
#define PMBUS_CMD_BYTE_SIZE            1
#define PMBUS_FLIP_BYTES               0

/* VR feedback resistors */
#define GDDR_VDDR_FB1         0.422
#define GDDR_VDDR_FB2         1.0
#define CB_GDDR_VDDR_FB1      1.37
#define CB_GDDR_VDDR_FB2      4.32
#define SCRAPPY_GDDR_VDDR_FB1 1.07
#define SCRAPPY_GDDR_VDDR_FB2 3.48

struct OperationBits {
	uint8_t reserved: 1;
	uint8_t transition_control: 1;
	uint8_t margin_fault_response: 2;

	enum VoltageCmdSource voltage_command_source: 2;
	uint8_t turn_off_behaviour: 1;
	uint8_t on_off_state: 1;
};
LOG_MODULE_REGISTER(regulator);

/* The default value is the regulator default */
static uint8_t vout_cmd_source = VoutCommand;
static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));

static uint32_t PmbusRead(uint32_t slave_addr, uint16_t command, uint8_t *data, uint32_t data_size)
{
	uint32_t status;

	if (I2CLock(PMBUS_MST_ID) != 0) {
		return -EINVAL;
	}
	status = I2CInit(I2CMst, slave_addr, I2CFastMode, PMBUS_MST_ID);
	if (status != 0) {
		I2CUnlock(PMBUS_MST_ID);
		return status;
	}
	status = I2CReadBytes(PMBUS_MST_ID, command, PMBUS_CMD_BYTE_SIZE, data, data_size,
			      PMBUS_FLIP_BYTES);
	I2CUnlock(PMBUS_MST_ID);
	return status;
}

static uint32_t PmbusWrite(uint32_t slave_addr, uint16_t command, const uint8_t *data,
			   uint32_t data_size)
{
	uint32_t status;

	if (I2CLock(PMBUS_MST_ID) != 0) {
		return -EINVAL;
	}
	status = I2CInit(I2CMst, slave_addr, I2CFastMode, PMBUS_MST_ID);
	if (status != 0) {
		I2CUnlock(PMBUS_MST_ID);
		return status;
	}
	status = I2CWriteBytes(PMBUS_MST_ID, command, PMBUS_CMD_BYTE_SIZE, data, data_size);
	I2CUnlock(PMBUS_MST_ID);
	return status;
}

static float ConvertLinear11ToFloat(uint16_t value)
{
	int16_t exponent = (value >> 11) & 0x1f;
	uint16_t mantissa = value & 0x7ff;

	if (exponent >> 4 == 1) { /* sign extension if negative */
		exponent |= ~0x1F;
	}

	return ldexp(mantissa, exponent);
}

/* GDDR IO regulators report READ_IOUT as a direct 14-bit value, 62.5 mA/LSB. */
static float ConvertGddrIoCurrentToFloat(uint16_t value)
{
	return (value & READ_IOUT_DIRECT_MASK) * GDDRIO_IOUT_LSB_A;
}

/* The function returns the core current in A. */
float GetVcoreCurrent(void)
{
	uint16_t iout = 0;

	if (PmbusRead(P0V8_VCORE_ADDR, READ_IOUT, (uint8_t *)&iout, READ_IOUT_DATA_BYTE_SIZE) !=
	    0) {
		return FLT_MAX;
	}
	return ConvertLinear11ToFloat(iout);
}

/* The function returns the GDDR west IO rail current in A. */
float GetGddrWestIoCurrent(void)
{
	uint16_t iout = 0;

	if (PmbusRead(GDDRIO_WEST_ADDR, READ_IOUT, (uint8_t *)&iout, READ_IOUT_DATA_BYTE_SIZE) !=
	    0) {
		return FLT_MAX;
	}
	return ConvertGddrIoCurrentToFloat(iout);
}

/* The function returns the GDDR east IO rail current in A. */
float GetGddrEastIoCurrent(void)
{
	uint16_t iout = 0;

	if (PmbusRead(GDDRIO_EAST_ADDR, READ_IOUT, (uint8_t *)&iout, READ_IOUT_DATA_BYTE_SIZE) !=
	    0) {
		return FLT_MAX;
	}
	return ConvertGddrIoCurrentToFloat(iout);
}

/* The function returns the GDDR west IO rail power in W. */
float GetGddrWestIoPower(void)
{
	return GetGddrWestIoCurrent() * GDDR_IO_RAIL_VOLTAGE;
}

/* The function returns the GDDR east IO rail power in W. */
float GetGddrEastIoPower(void)
{
	return GetGddrEastIoCurrent() * GDDR_IO_RAIL_VOLTAGE;
}

/* The function returns the core power in W. */
float GetVcorePower(void)
{
	uint16_t pout = 0;

	if (PmbusRead(P0V8_VCORE_ADDR, READ_POUT, (uint8_t *)&pout, READ_POUT_DATA_BYTE_SIZE) !=
	    0) {
		return FLT_MAX;
	}
	return ConvertLinear11ToFloat(pout);
}

static uint32_t set_max20730(uint32_t slave_addr, uint32_t voltage_in_mv, float rfb1, float rfb2)
{
	float vref = voltage_in_mv / (1 + rfb1 / rfb2);
	uint16_t vout_cmd = vref * LINEAR_FORMAT_CONSTANT * 0.001f;

	uint32_t status = PmbusWrite(slave_addr, VOUT_COMMAND, (uint8_t *)&vout_cmd,
				     VOUT_COMMAND_DATA_BYTE_SIZE);

	/* delay to flush i2c transaction and voltage change */
	WaitUs(250);
	return status;
}

static uint32_t set_mpm3695(uint32_t slave_addr, uint32_t voltage_in_mv, float rfb1, float rfb2)
{
	uint16_t vout_cmd = voltage_in_mv * 0.5f / SCALE_LOOP / (1 + rfb1 / rfb2);

	uint32_t status = PmbusWrite(slave_addr, VOUT_COMMAND, (uint8_t *)&vout_cmd,
				     VOUT_COMMAND_DATA_BYTE_SIZE);

	/* delay to flush i2c transaction and voltage change */
	WaitUs(250);
	return status;
}

/* Set MAX20816 voltage using I2C, MAX20816 is used for Vcore and Vcorem */
static uint32_t i2c_set_max20816(uint32_t slave_addr, uint32_t voltage_in_mv)
{
	uint16_t vout_cmd = 2 * voltage_in_mv;

	uint32_t status = PmbusWrite(slave_addr, VOUT_COMMAND, (uint8_t *)&vout_cmd,
				     VOUT_COMMAND_DATA_BYTE_SIZE);

	/* 100us to flush the tx of i2c + 150us to cover voltage switch from 0.65V to 0.95V with
	 * 50us of margin
	 */
	WaitUs(250);
	return status;
}

/* Returns MAX20816 output volage in mV. */
static uint32_t i2c_get_max20816(uint32_t slave_addr)
{
	uint16_t vout_cmd = 0;

	if (PmbusRead(slave_addr, READ_VOUT, (uint8_t *)&vout_cmd, READ_VOUT_DATA_BYTE_SIZE) != 0) {
		return UINT32_MAX;
	}

	return vout_cmd / 2U;
}

uint32_t set_vcore(uint32_t voltage_in_mv)
{
	if (vout_cmd_source == AVSVoutCommand) {
		return AVSWriteVoltage(voltage_in_mv, AVS_VCORE_RAIL);
	}

	return i2c_set_max20816(P0V8_VCORE_ADDR, voltage_in_mv);
}

uint32_t get_vcore(void)
{
	return i2c_get_max20816(P0V8_VCORE_ADDR);
}

uint32_t set_vcorem(uint32_t voltage_in_mv)
{
	return i2c_set_max20816(P0V8_VCOREM_ADDR, voltage_in_mv);
}

uint32_t get_vcorem(void)
{
	return i2c_get_max20816(P0V8_VCOREM_ADDR);
}

/* Set GDDR VDDR voltage for corner parts before DRAM training */
uint32_t set_gddr_vddr(PcbType board_type, uint32_t voltage_in_mv)
{
	if (board_type == PcbTypeOrionSLT) {
		uint32_t west_status = set_max20730(CB_GDDR_VDDR_WEST_ADDR, voltage_in_mv,
						    CB_GDDR_VDDR_FB1, CB_GDDR_VDDR_FB2);
		uint32_t east_status = set_max20730(CB_GDDR_VDDR_EAST_ADDR, voltage_in_mv,
						    CB_GDDR_VDDR_FB1, CB_GDDR_VDDR_FB2);

		return west_status != 0 ? west_status : east_status;
	}

	return set_mpm3695(GDDR_VDDR_ADDR, voltage_in_mv, GDDR_VDDR_FB1, GDDR_VDDR_FB2);
}

uint32_t SwitchVoutControl(enum VoltageCmdSource source)
{
	uint32_t status;
	struct OperationBits operation = {0};

	if (source > AVSVoutCommand) {
		return -EINVAL;
	}
	if (I2CLock(PMBUS_MST_ID) != 0) {
		return -EINVAL;
	}
	status = I2CInit(I2CMst, P0V8_VCORE_ADDR, I2CFastMode, PMBUS_MST_ID);
	if (status != 0) {
		I2CUnlock(PMBUS_MST_ID);
		return status;
	}
	status = I2CReadBytes(PMBUS_MST_ID, OPERATION, PMBUS_CMD_BYTE_SIZE, (uint8_t *)&operation,
			      OPERATION_DATA_BYTE_SIZE, PMBUS_FLIP_BYTES);
	if (status != 0) {
		I2CUnlock(PMBUS_MST_ID);
		return status;
	}
	operation.transition_control =
		1; /* copy vout command when control is passed from AVSBus to PMBus */
	operation.voltage_command_source = source;
	status = I2CWriteBytes(PMBUS_MST_ID, OPERATION, PMBUS_CMD_BYTE_SIZE, (uint8_t *)&operation,
			       OPERATION_DATA_BYTE_SIZE);
	I2CUnlock(PMBUS_MST_ID);
	if (status != 0) {
		return status;
	}

	/* 100us to flush the tx of i2c */
	WaitUs(100);
	vout_cmd_source = source;
	return 0;
}

uint32_t RegulatorInit(PcbType board_type)
{
	uint32_t aggregate_i2c_errors = 0;
	uint32_t i2c_error = 0;

	const struct board_regulators_config *regulators_config = NULL;

	if (board_type == PcbTypeP150) {
		regulators_config = &p150_regulators_config;
	} else if (board_type == PcbTypeP300) {
		if (tt_bh_fwtable_is_p300_left_chip()) {
			regulators_config = &p300_left_regulators_config;
		} else {
			regulators_config = &p300_right_regulators_config;
		}
	} else if (board_type == PcbTypeUBB) {
		regulators_config = &ubb_regulators_config;
	} else if (board_type == PcbTypeOrionSLT) {
		/* Do nothing, Orion SLT regulators are manually programmed */
	} else {
		LOG_ERR("Unsupported board type %d", board_type);
		return -ENOTSUP;
	}
	if (regulators_config) {
		for (uint32_t i = 0; i < regulators_config->count; i++) {
			const struct regulator_config *regulator_config =
				regulators_config->regulator_config + i;

			if (I2CLock(PMBUS_MST_ID) != 0) {
				aggregate_i2c_errors |= EINVAL;
				continue;
			}
			i2c_error = I2CInit(I2CMst, regulator_config->address, I2CFastMode,
					    PMBUS_MST_ID);
			if (i2c_error != 0) {
				aggregate_i2c_errors |= i2c_error;
				I2CUnlock(PMBUS_MST_ID);
				continue;
			}

			for (uint32_t j = 0; j < regulator_config->count; j++) {
				const struct regulator_data *regulator_data =
					&regulator_config->regulator_data[j];

				LOG_DBG("Regulator %#x init on cmd %#x", regulator_config->address,
					regulator_data->cmd);

				i2c_error = I2CRMWV(PMBUS_MST_ID, regulator_data->cmd,
						    PMBUS_CMD_BYTE_SIZE, regulator_data->data,
						    regulator_data->mask, regulator_data->size);

				if (i2c_error) {
					LOG_WRN("Regulator %#x init retried on cmd %#x "
						"with error %#x",
						regulator_config->address, regulator_data->cmd,
						i2c_error);

					/* First, try a bus recovery */
					I2CRecoverBus(PMBUS_MST_ID);
					/* Retry once */
					i2c_error =
						I2CRMWV(PMBUS_MST_ID, regulator_data->cmd,
							PMBUS_CMD_BYTE_SIZE, regulator_data->data,
							regulator_data->mask, regulator_data->size);
					if (i2c_error) {
						LOG_ERR("Regulator init failed on cmd %#x "
							"with error %#x",
							regulator_data->cmd, i2c_error);
						aggregate_i2c_errors |= i2c_error;
					} else {
						LOG_INF("Regulator init succeeded on cmd %#x",
							regulator_data->cmd);
					}
				}
			}
			I2CUnlock(PMBUS_MST_ID);
		}
	}
	return aggregate_i2c_errors;
}

/**
 * @brief Handler for @ref TT_SMC_MSG_SET_VOLTAGE messages
 *
 * @details Sets the voltage on the specified regulator via I2C. The request should contain
 *          the I2C slave address and the voltage value in millivolts.
 *
 * @param request Pointer to the host request message, use request->set_voltage for structured
 *                access
 * @param response Pointer to the response message to be sent back to host
 *
 * @return 0 on success
 * @return non-zero on error
 *
 * @see set_voltage_rqst
 */
static uint8_t set_voltage_handler(const union request *request, struct response *response)
{
	uint32_t slave_addr = request->set_voltage.slave_addr;
	uint32_t voltage_in_mv = request->set_voltage.voltage_in_mv;

	/* Direct regulator writes bypass the VF arbiter and can undervolt a live
	 * high-frequency clock. Runtime callers must use the ordered DVFS controls.
	 */
	if (dvfs_enabled) {
		return 2;
	}

	switch (slave_addr) {
	case P0V8_VCORE_ADDR:
		return set_vcore(voltage_in_mv) != 0;
	case P0V8_VCOREM_ADDR:
		return set_vcorem(voltage_in_mv) != 0;
	default:
		return 1;
	}
}

/**
 * @brief Handler for @ref TT_SMC_MSG_GET_VOLTAGE messages
 *
 * @details Reads the current voltage from the specified regulator via I2C and returns
 *          it in the response message.
 *
 * @param request Pointer to the host request message, use request->get_voltage for structured
 *                access
 * @param response Pointer to the response message to be sent back to host, will contain:
 *                 - data[1]: Current voltage reading in millivolts
 *
 * @return 0 on success
 * @return non-zero on error
 *
 * @see get_voltage_rqst
 */
static uint8_t get_voltage_handler(const union request *request, struct response *response)
{
	uint32_t slave_addr = request->get_voltage.slave_addr;

	switch (slave_addr) {
	case P0V8_VCORE_ADDR:
		response->data[1] = get_vcore();
		return response->data[1] == UINT32_MAX;
	case P0V8_VCOREM_ADDR:
		response->data[1] = get_vcorem();
		return response->data[1] == UINT32_MAX;
	default:
		return 1;
	}
}

/**
 * @brief Handler for @ref TT_SMC_MSG_SWITCH_VOUT_CONTROL messages
 *
 * @details Switches the VOUT control source for voltage regulators. This allows
 *          switching between different control methods.
 *
 * @param request Pointer to the host request message, use request->switch_vout_control for
 *                structured access
 * @param response Pointer to the response message to be sent back to host
 *
 * @return 0 on success
 * @return non-zero on error
 *
 * @see switch_vout_control_rqst
 */
static uint8_t switch_vout_control_handler(const union request *request, struct response *response)
{
	uint32_t source = request->switch_vout_control.source;

	/* Keep runtime voltage changes on the source validated during DVFS init. */
	if (dvfs_enabled) {
		return 2;
	}
	return SwitchVoutControl(source) != 0;
}

REGISTER_MESSAGE(TT_SMC_MSG_SET_VOLTAGE, set_voltage_handler, MSGQUEUE_COMMAND_DENIED);
REGISTER_MESSAGE(TT_SMC_MSG_GET_VOLTAGE, get_voltage_handler, MSGQUEUE_COMMAND_DIAGNOSTIC);
REGISTER_MESSAGE(TT_SMC_MSG_SWITCH_VOUT_CONTROL, switch_vout_control_handler,
		 MSGQUEUE_COMMAND_DENIED);

static int regulator_init(void)
{
	int ret;

	SetPostCode(POST_CODE_SRC_CMFW, POST_CODE_ARC_INIT_STEPC);

	if (IS_ENABLED(CONFIG_TT_SMC_RECOVERY) || !IS_ENABLED(CONFIG_ARC)) {
		return 0;
	}

	ret = (int)RegulatorInit(tt_bh_fwtable_get_pcb_type(fwtable_dev));
	if (ret != 0) {
		record_init_failure(INIT_STAGE_REGULATOR);
		return -EIO;
	}

	return 0;
}
SYS_INIT_APP(regulator_init);
