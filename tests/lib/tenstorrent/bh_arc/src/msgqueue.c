/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_tt_bh.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>

#include <zephyr/device.h>
#include <zephyr/drivers/misc/bh_fwtable.h>

#include <tenstorrent/smc_msg.h>
#include <tenstorrent/msgqueue.h>
#include <tenstorrent/tt_smbus_regs.h>
#include <tenstorrent/bh_arc.h>
#include <tenstorrent/bh_power.h>
#include "asic_state.h"
#include "clock_wave.h"
#include "cm2dm_msg.h"
#include "dvfs.h"
#include "noc_init.h"
#include "aiclk_ppm.h"
#include "throttler.h"

#include "reg_mock.h"
#include "voltage.h"

/* Custom fake for ReadReg to simulate timer progression */
#define RESET_UNIT_REFCLK_CNT_LO_REG_ADDR         0x800300E0
#define PLL_CNTL_WRAPPER_CLOCK_WAVE_CNTL_REG_ADDR 0x80020038
#define ARC_NOC0_TLB_BASE_ADDR                    0xC0000000U
#define ARC_NOC1_TLB_BASE_ADDR                    0xE0000000U
#define ARC_NOC_TLB_WINDOW_SPAN                   0x10000000U
static uint32_t timer_counter;
static uint8_t i2c_read_buf_emul[256] = {0};
static uint8_t i2c_read_buf_idx;

static uint8_t i2c_write_buf_emul[256] = {0};
static uint8_t i2c_write_buf_idx;

static uint32_t clock_wave_value;
static uint32_t noc_2_axi_last_write;
static uint32_t msgqueue_handler_73_calls;

union request req = {0};
struct response rsp = {0};

static const struct device *const i2c0_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(i2c0));
static const uint8_t tt_i2c_addr = 0xA;

/* Build-time-resolved handle for the bh_fwtable device. */
#define FWTABLE_DEV DEVICE_DT_GET(DT_NODELABEL(fwtable))

/* Runs FIRST in the msgqueue suite. ztest orders tests by SORT_BY_NAME on the
 * symbol z_ztest_unit_test__<suite>__<fn>, so the "0_" prefix sorts this ahead
 * of every other test. The build guarantees flash.bin exists (CMake FATAL_ERRORs
 * when protoc is missing), so a not-ready fwtable device is a real bh_fwtable
 * init/load regression and must FAIL — never skip.
 */
ZTEST(msgqueue, test_0_fwtable_device_ready)
{
	const struct device *dev = FWTABLE_DEV;

	zassert_not_null(dev, "fwtable device pointer is NULL");
	zassert_true(device_is_ready(dev),
		     "fwtable device not ready — bh_fwtable init/load failed. "
		     "flash.bin is built by CMake; check the driver's load paths.");
}

/* Helper function to simulate DMC reading posted SMBUS messages */
static cm2dmMessage read_posted_smbus_message(void)
{
	cm2dmMessage msg = {0};
	uint8_t write_data[] = {CMFW_SMBUS_REQ};
	uint8_t read_data[7]; /* 48 bits = 6 bytes for cm2dmMessage */

	/* Read the posted message */
	int ret = i2c_write_read(i2c0_dev, tt_i2c_addr, write_data, sizeof(write_data), read_data,
				 sizeof(read_data));

	if (ret == 0) {
		/* Parse the cm2dmMessage struct */
		msg.msg_id = read_data[1];
		msg.seq_num = read_data[2];
		/* data is uint32_t, so combine bytes 2-5 */
		msg.data = (uint32_t)read_data[3] | ((uint32_t)read_data[4] << 8) |
			   ((uint32_t)read_data[5] << 16) | ((uint32_t)read_data[6] << 24);
	}

	return msg;
}

static inline uint8_t pec_crc_8(uint8_t crc, uint8_t data)
{
	return crc8(&data, 1, 0x7, crc, false);
}

/* Helper function to send ACK for received message */
static void ack_smbus_message(const cm2dmMessage *msg)
{
	cm2dmAck ack;
	uint8_t pec = 0;

	ack.msg_id = msg->msg_id;
	ack.seq_num = msg->seq_num;

	pec = pec_crc_8(pec, (tt_i2c_addr << 1) | I2C_MSG_WRITE);
	pec = pec_crc_8(pec, CMFW_SMBUS_ACK);
	pec = pec_crc_8(pec, ack.msg_id);
	pec = pec_crc_8(pec, ack.seq_num);

	uint8_t write_data[] = {CMFW_SMBUS_ACK, ack.msg_id, ack.seq_num, pec};
	int x = i2c_write(i2c0_dev, write_data, sizeof(write_data), tt_i2c_addr);

	printf("%d", x);
}

/* Helper function to clear all pending SMBUS messages */
static void clear_pending_smbus_messages(void)
{
	cm2dmMessage msg;
	int attempts = 10; /* Prevent infinite loop */

	do {
		msg = read_posted_smbus_message();
		if (msg.msg_id != 0) {
			ack_smbus_message(&msg);
			attempts--;
		}
	} while (msg.msg_id != 0 && attempts > 0);
}

/*
 * These variadic macros can be used to attach a printf-style context
 * message describing a specific sub-case inside a ZTEST, instead of
 * 1 message per ZTEST
 *
 * Uses zexpect_* (soft assertions) so a single sub-case failure doesn't
 * short-circuit the rest of the test
 */
#define push_msg_success(...)                                                                      \
	do {                                                                                       \
		msgqueue_request_push(0, &req);                                                    \
		process_message_queues();                                                          \
		msgqueue_response_pop(0, &rsp);                                                    \
		zexpect_equal(rsp.data[0], 0, ##__VA_ARGS__);                                      \
	} while (0)

#define push_msg_failure(...)                                                                      \
	do {                                                                                       \
		msgqueue_request_push(0, &req);                                                    \
		process_message_queues();                                                          \
		msgqueue_response_pop(0, &rsp);                                                    \
		zexpect_not_equal(rsp.data[0], 0, ##__VA_ARGS__);                                  \
	} while (0)

static uint32_t ReadReg_msgqueue_fake(uint32_t addr)
{
	/* IC_STATUS; Fake out TX_FIFO to say empty and not full Fake out RX_FIFO to say not empty.
	 * This should be replaced by a emulated i2c driver once we use
	 * a real zephyr i2c controller in our app.
	 */
	if (addr == 0x80090070) {
		return 0b1110;
	}

	/* IC_DATA_CMD; Fake out RX data to provide emulated data*/
	if (addr == 0x80090010) {
		return i2c_read_buf_emul[i2c_read_buf_idx++];
	}

	if (addr == RESET_UNIT_REFCLK_CNT_LO_REG_ADDR) {
		return timer_counter++;
	}

	/*BH_PCIE_DWC_PCIE_USP_PF0_MSI_CAP_PCI_MSI_CAP_ID_NEXT_CTRL_REG_REG_ADDR*/
	if (addr == 0xCE000050) {
		return BIT(16) | BIT(20); /*pci_msi_enable | pci_msi_multiple_msg_en == 1*/
	}

	return 0;
}

#define I2C1_TX_ABRT_SOURCE_REG 0x80090080U

static uint32_t ReadReg_pmbus_read_abort_fake(uint32_t addr)
{
	if (addr == I2C1_TX_ABRT_SOURCE_REG) {
		return 1;
	}

	return ReadReg_msgqueue_fake(addr);
}

static void WriteReg_msgqueue_fake(uint32_t addr, uint32_t value)
{
	/* IC_DATA_CMD; Fake out TX data to get test visibility on sent data
	 * Note the truncation; data to I2C is in the LSB
	 * More significant bytes of the value word contain i2c transaction flags
	 */
	if (addr == 0x80090010) {
		i2c_write_buf_emul[i2c_write_buf_idx++] = value;
	}

	if (addr == PLL_CNTL_WRAPPER_CLOCK_WAVE_CNTL_REG_ADDR) {
		clock_wave_value = value;
	}

	if ((addr >= ARC_NOC0_TLB_BASE_ADDR &&
	     addr < ARC_NOC0_TLB_BASE_ADDR + ARC_NOC_TLB_WINDOW_SPAN) ||
	    (addr >= ARC_NOC1_TLB_BASE_ADDR &&
	     addr < ARC_NOC1_TLB_BASE_ADDR + ARC_NOC_TLB_WINDOW_SPAN)) {
		noc_2_axi_last_write = value;
	}
}

static uint8_t msgqueue_handler_73(const union request *req, struct response *rsp)
{
	BUILD_ASSERT(MSG_TYPE_SHIFT % 8 == 0);
	msgqueue_handler_73_calls++;
	rsp->data[1] = req->data[0];
	return 0;
}

ZTEST(msgqueue, test_msgqueue_register_handler)
{
	msgqueue_register_handler(0x73, msgqueue_handler_73, MSGQUEUE_COMMAND_UNSPECIFIED);

	req.data[0] = 0x73737373;
	push_msg_failure("a handler without an explicit policy must fail closed");
	zassert_equal(msgqueue_handler_73_calls, 0U, "denied handler was called");

	msgqueue_register_handler(0x73, msgqueue_handler_73, MSGQUEUE_COMMAND_REQUEST_DEPENDENT);
	req.data[0] = 0x73737373;
	push_msg_failure("unhandled request-dependent command must fail closed");
	zassert_equal(msgqueue_handler_73_calls, 0U, "request-dependent handler was called");

	req = (union request){0};
	rsp = (struct response){0};
	req.test.command_code = TT_SMC_MSG_TEST;
	req.test.test_value = 0x73;
	push_msg_success("queue did not respond after default-denied command");
	zassert_equal(rsp.data[1], 0x74U);
}

ZTEST(msgqueue, test_msgqueue_registered_surface_has_explicit_policy)
{
	static const struct {
		uint8_t command;
		enum msgqueue_command_policy policy;
	} expected[] = {
		{TT_SMC_MSG_SET_VOLTAGE, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_GET_VOLTAGE, MSGQUEUE_COMMAND_DIAGNOSTIC},
		{TT_SMC_MSG_SWITCH_CLK_SCHEME, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_DEBUG_NOC_TRANSLATION, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_SEND_PCIE_MSI, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_SWITCH_VOUT_CONTROL, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_READ_EEPROM, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_WRITE_EEPROM, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_READ_TS, MSGQUEUE_COMMAND_DIAGNOSTIC},
		{TT_SMC_MSG_READ_PD, MSGQUEUE_COMMAND_DIAGNOSTIC},
		{TT_SMC_MSG_READ_VM, MSGQUEUE_COMMAND_DIAGNOSTIC},
		{TT_SMC_MSG_I2C_MESSAGE, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_REINIT_TENSIX, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_POWER_SETTING, MSGQUEUE_COMMAND_MUTATING},
		{TT_SMC_MSG_SET_TDP_LIMIT, MSGQUEUE_COMMAND_MUTATING},
		{TT_SMC_MSG_SET_ASIC_HOST_FMAX, MSGQUEUE_COMMAND_MUTATING},
		{TT_SMC_MSG_SET_BOARD_POWER_LIMIT, MSGQUEUE_COMMAND_MUTATING},
		{TT_SMC_MSG_GET_FREQ_CURVE_FROM_VOLTAGE, MSGQUEUE_COMMAND_DIAGNOSTIC},
		{TT_SMC_MSG_AISWEEP_START, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_AISWEEP_STOP, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_FORCE_AICLK, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_GET_AICLK, MSGQUEUE_COMMAND_DIAGNOSTIC},
		{TT_SMC_MSG_COUNTER, MSGQUEUE_COMMAND_REQUEST_DEPENDENT},
		{TT_SMC_MSG_FORCE_VDD, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_AICLK_GO_BUSY, MSGQUEUE_COMMAND_MUTATING},
		{TT_SMC_MSG_AICLK_GO_LONG_IDLE, MSGQUEUE_COMMAND_MUTATING},
		{TT_SMC_MSG_TRIGGER_RESET, MSGQUEUE_COMMAND_MUTATING},
		{TT_SMC_MSG_PCIE_DMA_CHIP_TO_HOST_TRANSFER, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_PCIE_DMA_HOST_TO_CHIP_TRANSFER, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_ASIC_STATE0, MSGQUEUE_COMMAND_MUTATING},
		{TT_SMC_MSG_ASIC_STATE3, MSGQUEUE_COMMAND_MUTATING},
		{TT_SMC_MSG_GET_VOLTAGE_CURVE_FROM_FREQ, MSGQUEUE_COMMAND_DIAGNOSTIC},
		{TT_SMC_MSG_FORCE_FAN_SPEED, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_TOGGLE_SINGLE_TENSIX_RESET, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_TOGGLE_TENSIX_RESET, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_TOGGLE_ETH_RESET, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_TOGGLE_GDDR_RESET, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_PING_DM, MSGQUEUE_COMMAND_DIAGNOSTIC},
		{TT_SMC_MSG_SET_WDT_TIMEOUT, MSGQUEUE_COMMAND_REQUEST_DEPENDENT},
		{TT_SMC_MSG_FLASH_UNLOCK, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_FLASH_LOCK, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_CONFIRM_FLASHED_SPI, MSGQUEUE_COMMAND_DIAGNOSTIC},
		{TT_SMC_MSG_BLINKY, MSGQUEUE_COMMAND_DENIED},
		{TT_SMC_MSG_CHARACTERISATION, MSGQUEUE_COMMAND_DENIED},
#ifdef CONFIG_TT_PCIE_LOG_BACKEND
		{TT_SMC_MSG_TT_PCIE_LOG, MSGQUEUE_COMMAND_DENIED},
#endif
	};
	bool seen[UINT8_MAX + 1] = {0};
	size_t count = 0;

	STRUCT_SECTION_FOREACH(msgqueue_handler, item) {
		zexpect_true(item->msg_type <= UINT8_MAX, "registered code out of range");
		zexpect_not_equal(item->policy, MSGQUEUE_COMMAND_UNSPECIFIED,
				  "command 0x%02x has no admission policy", item->msg_type);
		zexpect_false(seen[item->msg_type], "duplicate command 0x%02x", item->msg_type);
		bool matched = false;

		for (size_t i = 0; i < ARRAY_SIZE(expected); i++) {
			if (expected[i].command == item->msg_type) {
				zexpect_equal(item->policy, expected[i].policy,
					      "policy changed for command 0x%02x", item->msg_type);
				matched = true;
				break;
			}
		}
		zexpect_true(matched, "unexpected registered command 0x%02x", item->msg_type);
		seen[item->msg_type] = true;
		count++;
	}

	zassert_equal(count, ARRAY_SIZE(expected), "registered surface changed");
	for (size_t i = 0; i < ARRAY_SIZE(expected); i++) {
		zexpect_true(seen[expected[i].command], "command 0x%02x is not registered",
			     expected[i].command);
	}

	req = (union request){0};
	req.command_code = TT_SMC_MSG_TEST;
	zexpect_equal(msgqueue_get_command_policy(&req), MSGQUEUE_COMMAND_DIAGNOSTIC);
	req.command_code = TT_SMC_MSG_REPORT_SCRATCH_ONLY;
	zexpect_equal(msgqueue_get_command_policy(&req), MSGQUEUE_COMMAND_DIAGNOSTIC);
	req.command_code = TT_SMC_MSG_SET_LAST_SERIAL;
	zexpect_equal(msgqueue_get_command_policy(&req), MSGQUEUE_COMMAND_DIAGNOSTIC);
}

ZTEST(msgqueue, test_msgqueue_content_dependent_policy_fails_closed)
{
	req = (union request){0};
	req.counter.command_code = TT_SMC_MSG_COUNTER;
	req.counter.command = COUNTER_CMD_GET;
	zexpect_equal(msgqueue_get_command_policy(&req), MSGQUEUE_COMMAND_DIAGNOSTIC);
	req.counter.command = COUNTER_CMD_CLEAR;
	zexpect_equal(msgqueue_get_command_policy(&req), MSGQUEUE_COMMAND_DENIED);
	req.counter.command = UINT8_MAX;
	zexpect_equal(msgqueue_get_command_policy(&req), MSGQUEUE_COMMAND_DENIED);

	req = (union request){0};
	req.set_wdt_timeout.command_code = TT_SMC_MSG_SET_WDT_TIMEOUT;
	req.set_wdt_timeout.timeout_ms = 0;
	zexpect_equal(msgqueue_get_command_policy(&req), MSGQUEUE_COMMAND_MUTATING);
	req.set_wdt_timeout.timeout_ms = 1;
	zexpect_equal(msgqueue_get_command_policy(&req), MSGQUEUE_COMMAND_DENIED);

	req = (union request){0};
	req.command_code = 0x72;
	zexpect_equal(msgqueue_get_command_policy(&req), MSGQUEUE_COMMAND_UNSPECIFIED);
}

ZTEST(msgqueue, test_msgqueue_power_settings_cmd)
{
	const struct device *pll4 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(pll4));

	/* LSB to MSB:
	 * 0x21: TT_SMC_MSG_POWER_SETTING
	 * 0x04: 4 power flags valid, 0 power settings valid
	 * 0x000f: max_ai_clk, mrisc, tensix and l2cpu all on
	 */
	req.data[0] = 0x000F0421;
	push_msg_success();

	CalculateTargAiclk();
	zexpect_equal(GetAiclkTarg(), GetAiclkFmax());

	zexpect_equal(rsp.data[0], 0x0);

	/* Validate that all emulated L2CPU clocks are enabled. */
	zexpect_true(device_is_ready(pll4));
	zexpect_equal(clock_control_get_status(
			      pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_0),
		      CLOCK_CONTROL_STATUS_ON);
	zexpect_equal(clock_control_get_status(
			      pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_1),
		      CLOCK_CONTROL_STATUS_ON);
	zexpect_equal(clock_control_get_status(
			      pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_2),
		      CLOCK_CONTROL_STATUS_ON);
	zexpect_equal(clock_control_get_status(
			      pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_3),
		      CLOCK_CONTROL_STATUS_ON);

	/* LSB to MSB:
	 * 0x21: TT_SMC_MSG_POWER_SETTING
	 * 0x04: 4 power flags valid, 0 power settings valid
	 * 0x0007: max_ai_clk, mrisc and tensix on; l2cpu off
	 */
	uint32_t read_count = ReadReg_fake.call_count;
	uint32_t write_count = WriteReg_fake.call_count;

	req.data[0] = 0x00070421;
	push_msg_failure("runtime L2CPU clock disable must be rejected");
	zexpect_equal(ReadReg_fake.call_count, read_count,
		      "mixed request performed reads before rejecting L2CPU-off");
	zexpect_equal(WriteReg_fake.call_count, write_count,
		      "mixed request performed writes before rejecting L2CPU-off");

	CalculateTargAiclk();
	zexpect_equal(GetAiclkTarg(), GetAiclkFmax());

	/* A rejected request must leave every L2CPU clock enabled. */
	zexpect_true(device_is_ready(pll4));
	zexpect_equal(clock_control_get_status(
			      pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_0),
		      CLOCK_CONTROL_STATUS_ON);
	zexpect_equal(clock_control_get_status(
			      pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_1),
		      CLOCK_CONTROL_STATUS_ON);
	zexpect_equal(clock_control_get_status(
			      pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_2),
		      CLOCK_CONTROL_STATUS_ON);
	zexpect_equal(clock_control_get_status(
			      pll4, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_L2CPUCLK_3),
		      CLOCK_CONTROL_STATUS_ON);
}

ZTEST(msgqueue, test_msgqueue_powerdown_requires_quiescence)
{
	static const uint8_t valid_counts[] = {4, 15};
	bool before[BH_POWER_DOMAIN_COUNT];

	/* Exercise every combination of the defined power flags, including a
	 * last-close all-off request and mixed requests which raise AICLK while
	 * shutting down a dependency. No runtime message proves quiescence.
	 */
	for (size_t i = 0; i < ARRAY_SIZE(valid_counts); i++) {
		for (uint32_t flags = 0; flags < 16; flags++) {
			req = (union request){0};
			req.data[0] = 0x000F0421U;
			push_msg_success("could not establish powered baseline");
			for (int domain = 0; domain < BH_POWER_DOMAIN_COUNT; domain++) {
				zassert_ok(bh_power_state_get(domain, &before[domain]));
			}
			uint32_t reads = ReadReg_fake.call_count;
			uint32_t writes = WriteReg_fake.call_count;

			req = (union request){0};
			req.data[0] =
				(flags << 16) | (valid_counts[i] << 8) | TT_SMC_MSG_POWER_SETTING;
			if (flags == 0xEU || flags == 0xFU) {
				push_msg_success("safe idle/busy request was rejected");
				continue;
			}
			push_msg_failure("unquiesced powerdown accepted: flags=%x validity=%u",
					 flags, valid_counts[i]);
			zexpect_equal(ReadReg_fake.call_count, reads,
				      "rejected powerdown performed hardware reads");
			zexpect_equal(WriteReg_fake.call_count, writes,
				      "rejected powerdown performed hardware writes");
			for (int domain = 0; domain < BH_POWER_DOMAIN_COUNT; domain++) {
				bool after;

				zassert_ok(bh_power_state_get(domain, &after));
				zexpect_equal(after, before[domain],
					      "rejected powerdown mutated domain %d", domain);
			}
			req = (union request){0};
			req.command_code = TT_SMC_MSG_TEST;
			push_msg_success("ARC stopped responding after rejected powerdown");
		}
	}
}

ZTEST(msgqueue, test_msgqueue_kmd_legacy_power_state_abi)
{
	bool state;

	/* KMD sends all 15 protocol flags valid and defaults unknown flags to one.
	 * Firmware must ignore unknown bits, keep L2CPU on, and apply the four
	 * defined flags. 0x7ffe is KMD's legacy-open state (AICLK idle).
	 */
	req = (union request){0};
	req.data[0] = 0x7FFE0F21U;
	push_msg_success("KMD legacy POWER_SETTING ABI was rejected");
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_AICLK, &state));
	zexpect_false(state);
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_MRISC, &state));
	zexpect_true(state);
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_TENSIX, &state));
	zexpect_true(state);
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_L2CPU, &state));
	zexpect_true(state);

	/* Bit 15 is reserved, but the protocol validity count still permits it.
	 * Treat unknown flags as forward-compatible no-ops, not a rejected open.
	 */
	req = (union request){0};
	req.data[0] = 0xFFFF0F21U;
	push_msg_success("reserved/future KMD power flags were not ignored");
}

ZTEST(msgqueue, test_msgqueue_failed_aiclk_enable_does_not_latch_busy)
{
	VoltageArbiter previous_voltage = voltage_arbiter;
	bool aiclk_busy;

	/* Establish an unambiguous idle baseline independent of earlier messages. */
	dvfs_enabled = false;
	req = (union request){0};
	req.aiclk_set_speed.command_code = TT_SMC_MSG_AICLK_GO_LONG_IDLE;
	push_msg_success();
	req = (union request){0};
	req.data[0] = 0x00000121; /* one valid flag: AICLK idle */
	push_msg_success();

	/* Force the immediate DVFS transaction to fail at its PMBus voltage write. */
	voltage_arbiter.vdd_min = 650U;
	voltage_arbiter.vdd_max = 900U;
	voltage_arbiter.curr_voltage = 0U;
	voltage_arbiter.targ_voltage = 0U;
	ReadReg_fake.custom_fake = ReadReg_pmbus_read_abort_fake;
	dvfs_enabled = true;
	req.data[0] = 0x00010121; /* one valid flag: AICLK busy */
	push_msg_failure("failed AICLK enable must not retain the busy arbiter");

	dvfs_enabled = false;
	ReadReg_fake.custom_fake = ReadReg_msgqueue_fake;
	zassert_ok(bh_power_state_get(BH_POWER_DOMAIN_AICLK, &aiclk_busy));
	zexpect_false(aiclk_busy);
	CalculateTargAiclk();
	zexpect_equal(GetAiclkTarg(), GetAiclkFmin(),
		      "a later DVFS tick could raise after the failed enable");
	voltage_arbiter = previous_voltage;
	ThrottlerTestResetRuntimePowerState();
}

ZTEST(msgqueue, test_msgqueue_power_settings_with_go_busy)
{
	/* LSB to MSB:
	 * 0x21: TT_SMC_MSG_POWER_SETTING
	 * 0x01: 1 power flags valid,  power settings valid
	 * 0x0000: max_ai_clk off or 0x0001: max_ai_clk on
	 */
	static const uint32_t on_power_cmd = 0x00010121;
	static const uint32_t off_power_cmd = 0x00000121;

	req.data[0] = off_power_cmd;
	push_msg_success();

	CalculateTargAiclk();
	zexpect_equal(GetAiclkTarg(), GetAiclkFmin());

	/* Go busy should set targ to max */
	req.data[0] = TT_SMC_MSG_AICLK_GO_BUSY;
	push_msg_success();

	CalculateTargAiclk();
	zexpect_equal(GetAiclkTarg(), GetAiclkFmax());

	/*
	 * Because we got GO_BUSY, AICLK should remain at FMax after aiclk off POWER_SETTING
	 */
	req.data[0] = off_power_cmd;
	push_msg_success();

	CalculateTargAiclk();
	zexpect_equal(GetAiclkTarg(), GetAiclkFmax());

	/*
	 * Send POWER_SETTING with AICLK high
	 */

	req.data[0] = on_power_cmd;
	push_msg_success();

	CalculateTargAiclk();
	zexpect_equal(GetAiclkTarg(), GetAiclkFmax());

	/*
	 * Send GO_LONG_IDLE. We should remain at FMax because POWER_SETTING was set high
	 */
	req.data[0] = TT_SMC_MSG_AICLK_GO_LONG_IDLE;
	push_msg_success();

	CalculateTargAiclk();
	zexpect_equal(GetAiclkTarg(), GetAiclkFmax());

	/*
	 * Send POWER_SETTING with AICLK low. Now we should go to Fmin
	 */

	req.data[0] = off_power_cmd;
	push_msg_success();

	CalculateTargAiclk();
	zexpect_equal(GetAiclkTarg(), GetAiclkFmin());
}

ZTEST(msgqueue, test_msg_type_unsafe_tensix_resets_rejected)
{
	static const uint8_t unsafe_commands[] = {
		TT_SMC_MSG_REINIT_TENSIX,
		TT_SMC_MSG_TOGGLE_SINGLE_TENSIX_RESET,
		TT_SMC_MSG_TOGGLE_TENSIX_RESET,
	};
	uint32_t read_count = ReadReg_fake.call_count;
	uint32_t write_count = WriteReg_fake.call_count;

	for (size_t i = 0; i < ARRAY_SIZE(unsafe_commands); i++) {
		req = (union request){0};
		rsp = (struct response){0};
		req.command_code = unsafe_commands[i];
		if (unsafe_commands[i] == TT_SMC_MSG_TOGGLE_SINGLE_TENSIX_RESET) {
			req.toggle_single_tensix_reset.noc_x = UINT8_MAX;
			req.toggle_single_tensix_reset.noc_y = UINT8_MAX;
		}
		push_msg_failure("runtime reset command 0x%02x must be rejected",
				 unsafe_commands[i]);
		zexpect_equal(ReadReg_fake.call_count, read_count,
			      "rejected command 0x%02x touched reset/NOC registers",
			      unsafe_commands[i]);
		zexpect_equal(WriteReg_fake.call_count, write_count,
			      "rejected command 0x%02x touched reset/NOC registers",
			      unsafe_commands[i]);

		/* Rejection must leave the ARC request queue responsive. */
		req = (union request){0};
		rsp = (struct response){0};
		req.test.command_code = TT_SMC_MSG_TEST;
		req.test.test_value = unsafe_commands[i];
		push_msg_success("ARC stopped responding after rejected command 0x%02x",
				 unsafe_commands[i]);
		zexpect_equal(rsp.data[1], unsafe_commands[i] + 1U);
	}
}

ZTEST(msgqueue, test_msg_type_set_voltage)
{
	req.data[0] = TT_SMC_MSG_SET_VOLTAGE;
	req.data[1] = 0x64; /* regulator id */
	req.data[2] = 800;  /* voltage in mV */
	push_msg_failure("raw voltage writes must be denied");
	zexpect_equal(i2c_write_buf_idx, 0U);
}

ZTEST(msgqueue, test_msg_type_set_voltage_rejected_while_dvfs_active)
{
	dvfs_enabled = true;
	req.data[0] = TT_SMC_MSG_SET_VOLTAGE;
	req.data[1] = 0x64; /* Vcore regulator */
	req.data[2] = 800;
	push_msg_failure("direct voltage writes must not bypass active DVFS");
	zexpect_equal(i2c_write_buf_idx, 0U);
}

ZTEST(msgqueue, test_msg_type_get_voltage)
{
	/*Setup the simulated voltage for the i2c read*/
	uint32_t simulated_voltage_mv = 950;

	memcpy(i2c_read_buf_emul, &simulated_voltage_mv, sizeof(simulated_voltage_mv));

	req.data[0] = TT_SMC_MSG_GET_VOLTAGE;
	req.data[1] = 0x64; /* regulator id */
	push_msg_success();

	zexpect_equal(rsp.data[1], simulated_voltage_mv / 2);
}

ZTEST(msgqueue, test_msg_type_switch_vout_control)
{
	req.data[0] = TT_SMC_MSG_SWITCH_VOUT_CONTROL;
	req.data[1] = 0x01; /* regulator id */
	req.data[2] = 1;    /* enable */
	push_msg_failure("raw VOUT source changes must be denied");
	zexpect_equal(i2c_write_buf_idx, 0U);
}

ZTEST(msgqueue, test_msg_type_switch_vout_control_rejected_while_dvfs_active)
{
	dvfs_enabled = true;
	req.data[0] = TT_SMC_MSG_SWITCH_VOUT_CONTROL;
	req.data[2] = 3; /* AVSVoutCommand */
	push_msg_failure("runtime callers must not change the DVFS voltage source");
	zexpect_equal(i2c_write_buf_idx, 0U);
}

ZTEST(msgqueue, test_msg_type_switch_vout_control_failure_keeps_previous_source)
{
	req.data[0] = TT_SMC_MSG_SWITCH_VOUT_CONTROL;
	req.data[2] = 3; /* AVSVoutCommand */

	push_msg_failure("raw VOUT source changes must not reach PMBus");
	zexpect_equal(i2c_read_buf_idx, 0U);
	zexpect_equal(i2c_write_buf_idx, 0U);
}

ZTEST(msgqueue, test_msg_type_switch_clk_scheme)
{
	/* Reset timer counter and set up the fake */
	timer_counter = 0;

	req.data[0] = TT_SMC_MSG_SWITCH_CLK_SCHEME;
	req.data[1] = TT_CLK_SCHEME_CLOCK_WAVE;
	push_msg_failure("raw clock-scheme changes must be denied");
	zassert_equal(clock_wave_value, 0U);

	req.data[1] = TT_CLK_SCHEME_ZERO_SKEW;
	push_msg_failure("raw clock-scheme changes must be denied");
	zassert_equal(clock_wave_value, 0U);
}

ZTEST(msgqueue, test_msg_type_debug_noc_translation)
{
	uint32_t read_count = ReadReg_fake.call_count;
	uint32_t write_count = WriteReg_fake.call_count;

	req.data[0] = TT_SMC_MSG_DEBUG_NOC_TRANSLATION | (BIT(0) << 8U) /* Enable translation*/
		      | (BIT(1) << 8U)                                  /* PCIE Instance  = 1*/
		      | (BIT(2) << 8U)                                  /*PCIE instance override*/
		      | ((BIT(0) | BIT(3)) << 16U) /*Bad tensix columns 0 and 3*/
		;
	req.data[1] = 8U /* Bad GDDR 8 */ | ((BIT(1) | BIT(3)) << 8U) /*skip eth 1 and 3*/;
	push_msg_failure("live NOC translation reprogramming must be rejected");
	zexpect_equal(ReadReg_fake.call_count, read_count);
	zexpect_equal(WriteReg_fake.call_count, write_count);

	req = (union request){0};
	rsp = (struct response){0};
	req.test.command_code = TT_SMC_MSG_TEST;
	req.test.test_value = TT_SMC_MSG_DEBUG_NOC_TRANSLATION;
	push_msg_success("ARC stopped responding after rejected NOC translation request");
	zexpect_equal(rsp.data[1], TT_SMC_MSG_DEBUG_NOC_TRANSLATION + 1U);
}

static void test_setup(void *ctx)
{
	(void)ctx;
	Cm2DmResetTestClear();
	dvfs_enabled = false;
	ReadReg_fake.custom_fake = ReadReg_msgqueue_fake;
	WriteReg_fake.custom_fake = WriteReg_msgqueue_fake;
	timer_counter = 0U;
	i2c_read_buf_idx = 0U;
	i2c_write_buf_idx = 0U;
	clock_wave_value = 0U;
	msgqueue_handler_73_calls = 0U;
	req = (union request){0};
	rsp = (struct response){0};
	memset(i2c_read_buf_emul, 0, sizeof(i2c_read_buf_emul));
	memset(i2c_write_buf_emul, 0, sizeof(i2c_write_buf_emul));
}

ZTEST(msgqueue, test_msg_type_send_pcie_msi)
{
	noc_2_axi_last_write = 0xffffffffU;
	req.data[0] = TT_SMC_MSG_SEND_PCIE_MSI | (BIT(0) << 8U) /*PCIe instance 1*/;
	req.data[1] = 0x00; /* MSI number */
	push_msg_failure("raw MSI injection must be denied");
	zexpect_equal(noc_2_axi_last_write, 0xffffffffU);

	req.data[1] = 0x01; /* MSI number */

	push_msg_failure("raw MSI injection must be denied");
	zexpect_equal(noc_2_axi_last_write, 0xffffffffU);
}

ZTEST(msgqueue, test_msg_type_i2c_message_bad_line_id)
{
	union request req = {0};
	struct response rsp = {0};

	/* Reset timer counter and set up the fake */
	timer_counter = 0;
	ReadReg_fake.custom_fake = ReadReg_msgqueue_fake;

	req.data[0] = BIT(24U)                     /*Write Operation*/
		      | FIELD_PREP(0xFF0000, 0x50) /* target address */
		      | FIELD_PREP(0xFF00, 0x5U)   /*Invalid Line Id*/
		      | TT_SMC_MSG_I2C_MESSAGE;
	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	zassert_not_equal(rsp.data[0], 0);
}

ZTEST(msgqueue, test_msg_type_i2c_message_rejects_dmc_target_controller)
{
	req.data[0] = FIELD_PREP(0xFF0000, 0x50) | TT_SMC_MSG_I2C_MESSAGE;
	push_msg_failure("I2C0 must remain reserved for DMC power telemetry");
	zexpect_equal(i2c_write_buf_idx, 0U);
}

ZTEST(msgqueue, test_msg_type_i2c_message_rejects_board_control_controllers)
{
	static const struct {
		uint8_t controller;
		uint8_t target;
		uint8_t command;
	} hostile_requests[] = {
		{0U, 0x0AU, 0U}, /* DMC telemetry target */
		{1U, 0x64U, 1U}, /* VCORE PMBus OPERATION */
		{2U, 0x72U, 0U}, /* Board power switch */
	};

	for (size_t i = 0; i < ARRAY_SIZE(hostile_requests); i++) {
		uint32_t read_count = ReadReg_fake.call_count;
		uint32_t write_count = WriteReg_fake.call_count;

		req = (union request){0};
		rsp = (struct response){0};
		req.i2c_message.command_code = TT_SMC_MSG_I2C_MESSAGE;
		req.i2c_message.i2c_mst_id = hostile_requests[i].controller;
		req.i2c_message.i2c_slave_address = hostile_requests[i].target;
		req.i2c_message.num_write_bytes = 2U;
		req.i2c_message.write_data[0] = hostile_requests[i].command;
		req.i2c_message.write_data[1] = 0U;
		push_msg_failure("raw I2C controller %u must be rejected",
				 hostile_requests[i].controller);

		zexpect_equal(ReadReg_fake.call_count, read_count,
			      "rejected controller %u performed a register read",
			      hostile_requests[i].controller);
		zexpect_equal(WriteReg_fake.call_count, write_count,
			      "rejected controller %u performed a register write",
			      hostile_requests[i].controller);

		/* Rejection must leave the ARC request queue responsive. */
		req = (union request){0};
		rsp = (struct response){0};
		req.test.command_code = TT_SMC_MSG_TEST;
		req.test.test_value = hostile_requests[i].controller;
		push_msg_success("ARC stopped responding after rejected raw I2C controller %u",
				 hostile_requests[i].controller);
		zexpect_equal(rsp.data[1], hostile_requests[i].controller + 1U);
	}
}

ZTEST(msgqueue, test_msg_type_blink_led)
{
	clear_pending_smbus_messages();

	req.data[0] = TT_SMC_MSG_BLINKY;
	req.data[1] = 0x1;

	push_msg_failure("DMC mutation outside the runtime allowlist must be denied");

	/* Now act as DMC and read the posted SMBUS message */
	cm2dmMessage posted_msg = read_posted_smbus_message();

	/* Verify the posted message contains the correct LED blink data */
	zassert_equal(posted_msg.msg_id, 0, "denied message reached DMC");
}

ZTEST(msgqueue, test_msg_type_test)
{
	req.data[0] = TT_SMC_MSG_TEST;
	req.data[1] = 42; /* test_value to be incremented */

	push_msg_success();
	zexpect_equal(rsp.data[1], 43); /* test_value + 1 */
}

ZTEST(msgqueue, test_msg_type_asic_state)
{
	req.data[0] = TT_SMC_MSG_ASIC_STATE3;
	push_msg_success();
	zexpect_equal(get_asic_state(), A3State);

	/* Test ASIC_STATE0 to return to state 0 */
	req.data[0] = TT_SMC_MSG_ASIC_STATE0;
	push_msg_success();

	zexpect_equal(rsp.data[0], 0);
	zexpect_equal(get_asic_state(), A0State);
}

ZTEST(msgqueue, test_msg_type_read_eeprom_no_flash)
{
	req = (union request){0};
	rsp = (struct response){0};

	req.eeprom.command_code = TT_SMC_MSG_READ_EEPROM;
	req.eeprom.buffer_mem_type = 0;
	req.eeprom.spi_address = 0x1000;
	req.eeprom.num_bytes = 64;

	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	zassert_not_equal(rsp.data[0], 0, "runtime EEPROM reads must be denied");
}

ZTEST(msgqueue, test_msg_type_force_vdd)
{
	(void)InitVoltagePPM();
	req = (union request){0};
	rsp = (struct response){0};
	req.force_vdd.command_code = TT_SMC_MSG_FORCE_VDD;
	req.force_vdd.forced_voltage = (voltage_arbiter.vdd_min + voltage_arbiter.vdd_max) / 2U;
	push_msg_failure("raw FORCE_VDD must be denied even for an in-range voltage");
}

ZTEST(msgqueue, test_msg_type_pcie_dma_chip_to_host)
{
	uint32_t read_count = ReadReg_fake.call_count;
	uint32_t write_count = WriteReg_fake.call_count;

	noc_2_axi_last_write = 0xA5A5A5A5U;
	req.pcie_dma_transfer.command_code = TT_SMC_MSG_PCIE_DMA_CHIP_TO_HOST_TRANSFER;
	req.pcie_dma_transfer.completion_data = 0xAB;
	req.pcie_dma_transfer.transfer_size_bytes = UINT32_MAX;
	req.pcie_dma_transfer.chip_addr = UINT64_MAX;
	req.pcie_dma_transfer.host_addr = UINT64_MAX - 1U;
	req.pcie_dma_transfer.msi_completion_addr = UINT64_MAX - 2U;

	push_msg_failure("raw chip-to-host HDMA must be rejected");
	zexpect_equal(ReadReg_fake.call_count, read_count);
	zexpect_equal(WriteReg_fake.call_count, write_count);
	zexpect_equal(noc_2_axi_last_write, 0xA5A5A5A5U,
		      "rejected HDMA request programmed PCIe/NOC registers");

	req = (union request){0};
	rsp = (struct response){0};
	req.test.command_code = TT_SMC_MSG_TEST;
	req.test.test_value = TT_SMC_MSG_PCIE_DMA_CHIP_TO_HOST_TRANSFER;
	push_msg_success("ARC stopped responding after rejected chip-to-host HDMA");
	zexpect_equal(rsp.data[1], TT_SMC_MSG_PCIE_DMA_CHIP_TO_HOST_TRANSFER + 1U);
}

ZTEST(msgqueue, test_msg_type_pcie_dma_host_to_chip)
{
	uint32_t read_count = ReadReg_fake.call_count;
	uint32_t write_count = WriteReg_fake.call_count;

	noc_2_axi_last_write = 0x5A5A5A5AU;
	req.pcie_dma_transfer.command_code = TT_SMC_MSG_PCIE_DMA_HOST_TO_CHIP_TRANSFER;
	req.pcie_dma_transfer.completion_data = 0xCD;
	req.pcie_dma_transfer.transfer_size_bytes = UINT32_MAX;
	req.pcie_dma_transfer.chip_addr = UINT64_MAX;
	req.pcie_dma_transfer.host_addr = UINT64_MAX - 1U;
	req.pcie_dma_transfer.msi_completion_addr = UINT64_MAX - 2U;

	push_msg_failure("raw host-to-chip HDMA must be rejected");
	zexpect_equal(ReadReg_fake.call_count, read_count);
	zexpect_equal(WriteReg_fake.call_count, write_count);
	zexpect_equal(noc_2_axi_last_write, 0x5A5A5A5AU,
		      "rejected HDMA request programmed PCIe/NOC registers");

	req = (union request){0};
	rsp = (struct response){0};
	req.test.command_code = TT_SMC_MSG_TEST;
	req.test.test_value = TT_SMC_MSG_PCIE_DMA_HOST_TO_CHIP_TRANSFER;
	push_msg_success("ARC stopped responding after rejected host-to-chip HDMA");
	zexpect_equal(rsp.data[1], TT_SMC_MSG_PCIE_DMA_HOST_TO_CHIP_TRANSFER + 1U);
}

ZTEST(msgqueue, test_msg_type_trigger_reset_invalid)
{
	/* Invalid reset level should be rejected */
	req.trigger_reset.command_code = TT_SMC_MSG_TRIGGER_RESET;
	req.trigger_reset.reset_level = 5;

	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	zassert_equal(rsp.data[0], 5, "Invalid level should return the level as error");
}

ZTEST(msgqueue, test_msg_type_reset_pending_latches_type_and_quiesces_handlers)
{
	uint32_t read_count;
	uint32_t write_count;

	req = (union request){0};
	req.trigger_reset.command_code = TT_SMC_MSG_TRIGGER_RESET;
	req.trigger_reset.reset_level = kCm2DmResetLevelAsic;
	push_msg_success("first reset request was rejected");
	zassert_true(Cm2DmResetPending());
	zassert_equal(Cm2DmResetTestPendingLevel(), kCm2DmResetLevelAsic);

	/* A later reset cannot overwrite the first request's type. */
	req = (union request){0};
	req.trigger_reset.command_code = TT_SMC_MSG_TRIGGER_RESET;
	req.trigger_reset.reset_level = kCm2DmResetLevelDmc;
	push_msg_failure("second reset request must be rejected");
	zassert_equal(Cm2DmResetTestPendingLevel(), kCm2DmResetLevelAsic);

	read_count = ReadReg_fake.call_count;
	write_count = WriteReg_fake.call_count;
	req = (union request){0};
	req.power_setting.command_code = TT_SMC_MSG_POWER_SETTING;
	req.power_setting.power_flags_valid = 4U;
	req.power_setting.power_flags_bitfield.max_ai_clk = 1U;
	req.power_setting.power_flags_bitfield.mrisc_phy_power = 1U;
	req.power_setting.power_flags_bitfield.tensix_enable = 1U;
	req.power_setting.power_flags_bitfield.l2cpu_enable = 1U;
	push_msg_failure("mutation entered a handler while reset was pending");
	zexpect_equal(ReadReg_fake.call_count, read_count);
	zexpect_equal(WriteReg_fake.call_count, write_count);

	/* Even registered diagnostic handlers can touch buses, so only queue-local
	 * builtins remain live while teardown is pending.
	 */
	req = (union request){0};
	req.get_voltage.command_code = TT_SMC_MSG_GET_VOLTAGE;
	push_msg_failure("hardware diagnostic entered handler while reset was pending");
	zexpect_equal(ReadReg_fake.call_count, read_count);
	zexpect_equal(WriteReg_fake.call_count, write_count);

	req = (union request){0};
	req.test.command_code = TT_SMC_MSG_TEST;
	req.test.test_value = 41U;
	push_msg_success("queue-local heartbeat failed while reset was pending");
	zexpect_equal(rsp.data[1], 42U);
}

ZTEST(msgqueue, test_msg_type_flash_unlock)
{
	req.flash_unlock.command_code = TT_SMC_MSG_FLASH_UNLOCK;
	push_msg_failure("flash unlock requires a future explicit maintenance state");
}

ZTEST(msgqueue, test_msg_type_flash_lock)
{
	req.flash_lock.command_code = TT_SMC_MSG_FLASH_LOCK;
	push_msg_failure("flash mutation requires a future explicit maintenance state");
}

ZTEST(msgqueue, test_msg_type_confirm_flashed_spi)
{
	uint32_t challenge_data = 0xDEADBEEF;

	/* Test SPI flash confirmation with challenge data */
	req.confirm_flashed_spi.command_code = TT_SMC_MSG_CONFIRM_FLASHED_SPI;
	req.data[1] = challenge_data;

	push_msg_success();

	/* Should succeed */
	zassert_equal(rsp.data[0], 0, "Confirm flash should succeed");
	/* Response should echo the challenge data */
	zassert_equal(rsp.data[1], challenge_data, "Challenge data should be echoed back");
}

ZTEST(msgqueue, test_msg_type_set_last_serial)
{
	uint32_t test_serial_number = 0x12345678;

	/* Test setting a specific serial number */
	req.set_last_serial.command_code = TT_SMC_MSG_SET_LAST_SERIAL;
	req.set_last_serial.serial_number = test_serial_number;

	push_msg_success();

	/* Verify the operation succeeded */
	zassert_equal(rsp.data[0], 0, "Set last serial should succeed");

	/* Verify the serial number was set by using a test message */
	memset(&req, 0, sizeof(req));
	memset(&rsp, 0, sizeof(rsp));
	req.test.command_code = TT_SMC_MSG_TEST;
	req.test.test_value = 42; /* arbitrary test value */

	push_msg_success();

	/* Response data[2] contains last_serial + 1 */
	zassert_equal(rsp.data[2], test_serial_number + 1,
		      "Serial number should be set to the specified value");
	/* Verify test value was also processed correctly */
	zassert_equal(rsp.data[1], 43, "Test value should be incremented");

	/* Test setting a different serial number */
	memset(&req, 0, sizeof(req));
	memset(&rsp, 0, sizeof(rsp));
	uint32_t new_serial_number = 0xABCDEF00;

	req.set_last_serial.command_code = TT_SMC_MSG_SET_LAST_SERIAL;
	req.set_last_serial.serial_number = new_serial_number;

	push_msg_success();

	/* Verify the new serial number was set */
	memset(&req, 0, sizeof(req));
	memset(&rsp, 0, sizeof(rsp));
	req.test.command_code = TT_SMC_MSG_TEST;
	req.test.test_value = 99; /* arbitrary test value */

	push_msg_success();

	/* Verify new serial number is active */
	zassert_equal(rsp.data[2], new_serial_number + 1,
		      "Serial number should be updated to the new value");
}

ZTEST(msgqueue, test_msg_type_set_wdt_timeout)
{
	/* Clear any pending messages from previous tests */
	clear_pending_smbus_messages();

	/* Arming the watchdog from a raw host command can reset the card later. */
	req.set_wdt_timeout.command_code = TT_SMC_MSG_SET_WDT_TIMEOUT;
	req.set_wdt_timeout.timeout_ms = 5000;
	push_msg_failure("nonzero watchdog timeout must be denied");
	cm2dmMessage posted_msg = read_posted_smbus_message();

	zassert_equal(posted_msg.msg_id, 0, "denied watchdog request reached DMC");

	/* Test disabling watchdog (timeout = 0) */
	memset(&req, 0, sizeof(req));
	memset(&rsp, 0, sizeof(rsp));
	req.set_wdt_timeout.command_code = TT_SMC_MSG_SET_WDT_TIMEOUT;
	req.set_wdt_timeout.timeout_ms = 0; /* Disable watchdog */

	push_msg_success();

	/* Read the posted disable message */
	cm2dmMessage disable_msg = read_posted_smbus_message();

	/* Verify the disable message */
	zassert_equal(disable_msg.msg_id, kCm2DmMsgIdAutoResetTimeoutUpdate,
		      "Posted message should be AutoResetTimeoutUpdate");
	zassert_equal(disable_msg.data, 0,
		      "Posted message data should contain timeout value 0 (disabled)");

	/* Send ACK for the disable message */
	ack_smbus_message(&disable_msg);

	/* Every nonzero value is denied before the handler. */
	memset(&req, 0, sizeof(req));
	memset(&rsp, 0, sizeof(rsp));
	req.set_wdt_timeout.command_code = TT_SMC_MSG_SET_WDT_TIMEOUT;
	req.set_wdt_timeout.timeout_ms = 1; /* Very small timeout - should be rejected */

	msgqueue_request_push(0, &req);
	process_message_queues();
	msgqueue_response_pop(0, &rsp);

	zassert_not_equal(rsp.data[0], 0, "Small watchdog timeout must be denied");

	/* Verify no message was posted for invalid timeout */
	cm2dmMessage invalid_msg = read_posted_smbus_message();

	zassert_equal(invalid_msg.msg_id, 0, "No message should be posted for invalid timeout");
}

ZTEST(msgqueue, test_msg_type_ping_dm)
{
	/* Clear any pending messages from previous tests */
	clear_pending_smbus_messages();

	/* Test ping DMC with legacy_ping = false (read transaction) */
	req.dmc_ping.command_code = TT_SMC_MSG_PING_DM;
	req.dmc_ping.legacy_ping = false;

	push_msg_success();

	/* Now act as DMC and read the posted SMBUS message */
	cm2dmMessage posted_msg = read_posted_smbus_message();

	/* Verify the posted message contains the correct ping data */
	zassert_equal(posted_msg.msg_id, 2, "Posted message should be Ping (kCm2DmMsgIdPing = 2)");
	zassert_equal(posted_msg.data, 0, "Posted message data should contain legacy_ping = false");

	/* Send ACK for the message */
	ack_smbus_message(&posted_msg);
	/* Note the DM PING SMBUS message is not simulated in this test */

	/* Test ping DMC with legacy_ping = true (write transaction) */
	clear_pending_smbus_messages();
	memset(&req, 0, sizeof(req));
	memset(&rsp, 0, sizeof(rsp));

	req.dmc_ping.command_code = TT_SMC_MSG_PING_DM;
	req.dmc_ping.legacy_ping = true;

	push_msg_success();

	/* Verify the command was processed */
	zassert_equal(rsp.data[0], 0, "Ping DM command should be processed successfully");

	/* Read the posted SMBUS message for legacy ping */
	cm2dmMessage legacy_msg = read_posted_smbus_message();

	/* Verify the legacy ping message */
	zassert_equal(legacy_msg.msg_id, 2, "Posted message should be Ping (kCm2DmMsgIdPing = 2)");
	zassert_equal(legacy_msg.data, 1, "Posted message data should contain legacy_ping = true");

	/* Send ACK for the legacy message */
	ack_smbus_message(&legacy_msg);
	/* Note the DM PING SMBUS message is not simulated in this test */
}

ZTEST_SUITE(msgqueue, NULL, NULL, test_setup, NULL, NULL);
