/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bh_reset.h"
#include "gddr.h"
#include "harvesting.h"
#include "init.h"
#include "noc.h"
#include "noc_init.h"
#include "noc2axi.h"
#include "reg.h"
#include "status_reg.h"

#include <stddef.h>
#include <string.h>

#include <tenstorrent/bh_power.h>
#include <tenstorrent/msgqueue.h>
#include <tenstorrent/post_code.h>
#include <tenstorrent/smc_msg.h>
#include <tenstorrent/spi_flash_buf.h>
#include <tenstorrent/sys_init_defines.h>
#include <tenstorrent/tt_boot_fs.h>
#include <zephyr/drivers/misc/bh_fwtable.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/clock_control_tt_bh.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_tt_bh_noc.h>
#include <zephyr/drivers/dma/dma_arc_hs.h>

static const struct device *const pll_dev_3 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(pll3));
static const struct device *flash = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(spi_flash));
#ifdef CONFIG_DMA_ARC_HS
static const struct device *const arc_dma_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(dma0));
/*
 * The ARC DMA engine cannot abort a timed-out remote access. Keep its local
 * source/destination alive for the lifetime of the firmware so a late
 * completion cannot touch a returned stack frame. The channel is quarantined
 * after an incomplete transfer, so this staging area is never reused afterward.
 */
static union {
	gddr_telemetry_table_t telemetry;
	uint32_t word;
} gddr_remote_read_staging __aligned(4);
static uint32_t gddr_remote_write_staging __aligned(4);
K_MUTEX_DEFINE(gddr_remote_dma_lock);
static bool gddr_remote_dma_available = true;
#endif
static const struct device *dma_noc = DEVICE_DT_GET(DT_NODELABEL(dma1));

/* This is the default noc2axi instance we want to run the MRISC FW on */
#define MRISC_FW_NOC2AXI_PORT 0

/*
 * gddr0 runs its MRISC FW on noc2axi port 2 (NoC 0-11) instead of the default
 * port 0 (NoC 0-0); all other GDDR instances stay on the default port.
 */
uint8_t get_gddr_mrisc_noc2axi_port(uint8_t gddr_inst)
{
	return (gddr_inst == 0) ? 2 : MRISC_FW_NOC2AXI_PORT;
}

#define MRISC_SETUP_TLB            13
#define MRISC_DMA_TLB              12
#define MRISC_L1_ADDR              (1ULL << 37)
#define MRISC_REG_ADDR             (1ULL << 40)
#define MRISC_FW_CFG_OFFSET        0x3C00
#define ARC_NOC0_X                 8
#define ARC_NOC0_Y                 0
#define MRISC_L1_SIZE              (128 * 1024)
#define GDDR_WIPE_DMA_CHANNEL      1U
#define GDDR_WIPE_TIMEOUT_MS       50U
#define GDDR_REMOTE_DMA_TIMEOUT_MS 5U

#define MRISC_FW_TAG     "memfw"
#define MRISC_FW_CFG_TAG "memfwcfg"

LOG_MODULE_REGISTER(gddr, CONFIG_TT_APP_LOG_LEVEL);

static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));

static struct gddr_bist_info gddr_bist;
static uint8_t gddr_telemetry_version_ok;
static struct gddr_temps cached_gddr_temps;
static uint8_t cached_gddr_temp_valid;
static struct k_spinlock cached_gddr_temp_lock;

struct gddr_bist_info get_gddr_bist_info(void)
{
	return gddr_bist;
}

static uint32_t GetGddrSpeedFromCfg(uint8_t *fw_cfg_image)
{
	/* GDDR speed is the second DWORD of the MRISC FW Config table */
	uint32_t *fw_cfg_dw = (uint32_t *)fw_cfg_image;
	return fw_cfg_dw[1];
}

static void GetGddrMriscNocCoords(uint8_t gddr_inst, uint8_t noc_id, uint8_t *x, uint8_t *y)
{
	GetGddrNocCoords(gddr_inst, get_gddr_mrisc_noc2axi_port(gddr_inst), noc_id, x, y);
}

static volatile void *SetupMriscL1Tlb(uint8_t gddr_inst)
{
	uint8_t x, y;

	GetGddrMriscNocCoords(gddr_inst, 0, &x, &y);
	NOC2AXITlbSetup(0, MRISC_SETUP_TLB, x, y, MRISC_L1_ADDR);
	return GetTlbWindowAddr(0, MRISC_SETUP_TLB, MRISC_L1_ADDR);
}

static int gddr_remote_read_xy(uint8_t x, uint8_t y, uint64_t base_addr, uint32_t offset,
			       void *dest, size_t len)
{
#if defined(CONFIG_TT_BH_ARC_EMUL)
	if ((dest == NULL) || ((len % sizeof(uint32_t)) != 0U)) {
		return -EINVAL;
	}

	NOC2AXITlbSetup(0, MRISC_DMA_TLB, x, y, base_addr);
	for (size_t i = 0; i < len; i += sizeof(uint32_t)) {
		uint32_t word = NOC2AXIRead32(0, MRISC_DMA_TLB, base_addr + offset + i);

		memcpy((uint8_t *)dest + i, &word, sizeof(word));
	}
	return 0;
#elif defined(CONFIG_DMA_ARC_HS)
	int rc;

	if ((dest == NULL) || (len > sizeof(gddr_remote_read_staging))) {
		return -EINVAL;
	}

	k_mutex_lock(&gddr_remote_dma_lock, K_FOREVER);
	if (!gddr_remote_dma_available) {
		k_mutex_unlock(&gddr_remote_dma_lock);
		return -ENODEV;
	}
	NOC2AXITlbSetup(0, MRISC_DMA_TLB, x, y, base_addr);
	rc = dma_arc_hs_transfer(
		arc_dma_dev, 0,
		(const void *)((volatile uint8_t *)GetTlbWindowAddr(0, MRISC_DMA_TLB, base_addr) +
			       offset),
		&gddr_remote_read_staging, len, K_MSEC(GDDR_REMOTE_DMA_TIMEOUT_MS));
	if (rc == 0) {
		memcpy(dest, &gddr_remote_read_staging, len);
	} else if ((rc == -ETIMEDOUT) || (rc == -EIO)) {
		gddr_remote_dma_available = false;
		LOG_WRN("Disabling GDDR remote DMA after incomplete read: %d", rc);
	}
	k_mutex_unlock(&gddr_remote_dma_lock);
	return rc;
#else
	ARG_UNUSED(x);
	ARG_UNUSED(y);
	ARG_UNUSED(base_addr);
	ARG_UNUSED(offset);
	ARG_UNUSED(dest);
	ARG_UNUSED(len);
	return -ENOTSUP;
#endif
}

static int gddr_remote_read(uint8_t gddr_inst, uint64_t base_addr, uint32_t offset, void *dest,
			    size_t len)
{
	uint8_t x, y;

	if (gddr_inst >= NUM_GDDR) {
		return -EINVAL;
	}
	GetGddrMriscNocCoords(gddr_inst, 0, &x, &y);
	return gddr_remote_read_xy(x, y, base_addr, offset, dest, len);
}

static int gddr_remote_write32_xy(uint8_t x, uint8_t y, uint64_t base_addr, uint32_t offset,
				  uint32_t value)
{
#if defined(CONFIG_TT_BH_ARC_EMUL)
	NOC2AXITlbSetup(0, MRISC_DMA_TLB, x, y, base_addr);
	NOC2AXIWrite32(0, MRISC_DMA_TLB, base_addr + offset, value);
	return 0;
#elif defined(CONFIG_DMA_ARC_HS)
	int rc;

	k_mutex_lock(&gddr_remote_dma_lock, K_FOREVER);
	if (!gddr_remote_dma_available) {
		k_mutex_unlock(&gddr_remote_dma_lock);
		return -ENODEV;
	}
	NOC2AXITlbSetup(0, MRISC_DMA_TLB, x, y, base_addr);
	gddr_remote_write_staging = value;
	rc = dma_arc_hs_transfer(
		arc_dma_dev, 0, &gddr_remote_write_staging,
		(void *)((volatile uint8_t *)GetTlbWindowAddr(0, MRISC_DMA_TLB, base_addr) +
			 offset),
		sizeof(value), K_MSEC(GDDR_REMOTE_DMA_TIMEOUT_MS));
	if ((rc == -ETIMEDOUT) || (rc == -EIO)) {
		gddr_remote_dma_available = false;
		LOG_WRN("Disabling GDDR remote DMA after incomplete write: %d", rc);
	}
	k_mutex_unlock(&gddr_remote_dma_lock);
	return rc;
#else
	ARG_UNUSED(x);
	ARG_UNUSED(y);
	ARG_UNUSED(base_addr);
	ARG_UNUSED(offset);
	ARG_UNUSED(value);
	return -ENOTSUP;
#endif
}

static int gddr_remote_write32(uint8_t gddr_inst, uint64_t base_addr, uint32_t offset,
			       uint32_t value)
{
	uint8_t x, y;

	if (gddr_inst >= NUM_GDDR) {
		return -EINVAL;
	}
	GetGddrMriscNocCoords(gddr_inst, 0, &x, &y);
	return gddr_remote_write32_xy(x, y, base_addr, offset, value);
}

static int MriscL1Read32(uint8_t gddr_inst, uint32_t addr, uint32_t *value)
{
	return gddr_remote_read(gddr_inst, MRISC_L1_ADDR, addr, value, sizeof(*value));
}

static int MriscL1Write32(uint8_t gddr_inst, uint32_t addr, uint32_t value)
{
	return gddr_remote_write32(gddr_inst, MRISC_L1_ADDR, addr, value);
}

static int MriscRegRead32(uint8_t gddr_inst, uint32_t addr, uint32_t *value)
{
	return gddr_remote_read(gddr_inst, MRISC_REG_ADDR, addr, value, sizeof(*value));
}

static int MriscRegWrite32(uint8_t gddr_inst, uint32_t addr, uint32_t value)
{
	return gddr_remote_write32(gddr_inst, MRISC_REG_ADDR, addr, value);
}

int read_gddr_telemetry_table(uint8_t gddr_inst, gddr_telemetry_table_t *gddr_telemetry)
{
#ifdef CONFIG_DMA_ARC_HS
	if ((gddr_inst >= NUM_GDDR) || (gddr_telemetry == NULL)) {
		return -EINVAL;
	}
	int rc = gddr_remote_read(gddr_inst, MRISC_L1_ADDR, GDDR_TELEMETRY_TABLE_ADDR,
				  gddr_telemetry, sizeof(*gddr_telemetry));

	if (rc < 0) {
		/* Never fall back to CPU loads from a remote MRISC. If the tile is
		 * unresponsive, such a load can block ARC forever and take PCIe
		 * management down with it. The DMA path has a finite timeout and
		 * quarantines the channel because the hardware request cannot be
		 * aborted.
		 */
		return rc;
	}
#else
	ARG_UNUSED(gddr_inst);
	ARG_UNUSED(gddr_telemetry);
	return -ENOTSUP;
#endif
	/* Check that version matches expectation. */
	if (gddr_telemetry->telemetry_table_version != GDDR_TELEMETRY_TABLE_T_VERSION) {
		LOG_WRN_ONCE("GDDR telemetry table version mismatch: %d (expected %d)",
			     gddr_telemetry->telemetry_table_version,
			     GDDR_TELEMETRY_TABLE_T_VERSION);
		return -ENOTSUP;
	}

	{
		k_spinlock_key_t key = k_spin_lock(&cached_gddr_temp_lock);

		cached_gddr_temps.inst[gddr_inst].top = gddr_telemetry->dram_temperature_top;
		cached_gddr_temps.inst[gddr_inst].bottom = gddr_telemetry->dram_temperature_bottom;
		cached_gddr_temp_valid |= BIT(gddr_inst);
		cached_gddr_temps.max_temp = 0U;
		for (uint8_t i = 0; i < NUM_GDDR; i++) {
			if (IS_BIT_SET(cached_gddr_temp_valid, i)) {
				cached_gddr_temps.max_temp =
					MAX(cached_gddr_temps.max_temp,
					    MAX(cached_gddr_temps.inst[i].top,
						cached_gddr_temps.inst[i].bottom));
			}
		}
		k_spin_unlock(&cached_gddr_temp_lock, key);
	}
	return 0;
}

static int ReleaseMriscReset(uint8_t gddr_inst)
{
	const uint32_t kSoftReset0Addr = 0xFFB121B0;
	uint32_t soft_reset_0;
	int rc;

	rc = gddr_remote_read(gddr_inst, kSoftReset0Addr, 0, &soft_reset_0, sizeof(soft_reset_0));
	if (rc != 0) {
		return rc;
	}
	soft_reset_0 &= ~BIT(11); /* Clear bit corresponding to MRISC reset */
	return gddr_remote_write32(gddr_inst, kSoftReset0Addr, 0, soft_reset_0);
}

static void SetAxiEnable(uint8_t gddr_inst, uint8_t noc2axi_port, bool axi_enable)
{
	const uint32_t kNiuCfg0Addr[NUM_NOCS] = {0xFFB20100, 0xFFB30100};
	uint8_t x, y;
	volatile uint32_t *niu_cfg_0[NUM_NOCS];

	for (uint8_t i = 0; i < NUM_NOCS; i++) {
		GetGddrNocCoords(gddr_inst, noc2axi_port, i, &x, &y);
		/* Note this actually sets up two TLBs (one for each NOC) */
		NOC2AXITlbSetup(i, MRISC_SETUP_TLB, x, y, kNiuCfg0Addr[i]);
		niu_cfg_0[i] = GetTlbWindowAddr(i, MRISC_SETUP_TLB, kNiuCfg0Addr[i]);
	}

	if (axi_enable) {
		for (uint8_t i = 0; i < NUM_NOCS; i++) {
			*niu_cfg_0[i] |= (1 << NIU_CFG_0_AXI_SLAVE_ENABLE);
		}
	} else {
		for (uint8_t i = 0; i < NUM_NOCS; i++) {
			*niu_cfg_0[i] &= ~(1 << NIU_CFG_0_AXI_SLAVE_ENABLE);
		}
	}
}

static int LoadMriscFw(uint8_t gddr_inst, uint8_t *buf, size_t buf_size, size_t spi_address,
		       size_t image_size)
{
	volatile uint32_t *mrisc_l1 = SetupMriscL1Tlb(gddr_inst);
	int rc = spi_arc_dma_transfer_to_tile(flash, spi_address, image_size, buf, buf_size,
					      (uint8_t *)mrisc_l1);

	return rc;
}
static int LoadMriscFwCfg(uint8_t gddr_inst, uint8_t *buf, size_t buf_size, size_t spi_address,
			  size_t image_size)
{
	volatile uint32_t *mrisc_l1 = SetupMriscL1Tlb(gddr_inst);
	int rc = spi_arc_dma_transfer_to_tile(flash, spi_address, image_size, buf, buf_size,
					      (uint8_t *)mrisc_l1 + MRISC_FW_CFG_OFFSET);

	if (rc == 0) {
		/* Set the controller_id field to the GDDR instance */
		volatile gddr_params_table_t *params_table =
			(volatile gddr_params_table_t *)((uint8_t *)mrisc_l1 + MRISC_FW_CFG_OFFSET);
		if (params_table->params_table_version >= 6) {
			params_table->controller_id = gddr_inst;
		} else {
			LOG_WRN_ONCE("MRISC params table version %d does not support controller_id "
				     "field (>= 6 required)",
				     params_table->params_table_version);
		}
	}

	return rc;
}

static uint32_t GetDramMask(void)
{
	uint32_t dram_mask = tile_enable.gddr_enabled; /* bit mask */

	if (tt_bh_fwtable_get_fw_table(fwtable_dev)->has_dram_table &&
	    tt_bh_fwtable_get_fw_table(fwtable_dev)->dram_table.dram_mask_en) {
		dram_mask &= tt_bh_fwtable_get_fw_table(fwtable_dev)->dram_table.dram_mask;
	}
	return dram_mask;
}

int get_gddr_temps(struct gddr_temps *temps)
{
	uint8_t valid;
	k_spinlock_key_t key;

	if (temps == NULL) {
		return -EINVAL;
	}

	/* DVFS calls this every millisecond. Never perform a synchronous remote
	 * tile access on that critical path; the 100 ms telemetry worker refreshes
	 * this cache through dma_arc_hs_transfer(), which has a finite timeout.
	 */
	key = k_spin_lock(&cached_gddr_temp_lock);
	*temps = cached_gddr_temps;
	valid = cached_gddr_temp_valid;
	k_spin_unlock(&cached_gddr_temp_lock, key);

	return (valid & GetDramMask()) == GetDramMask() ? 0 : -EAGAIN;
}

static int check_mrisc_busy(uint8_t gddr_inst)
{
	uint32_t status;
	int rc = MriscRegRead32(gddr_inst, MRISC_MSG_REGISTER, &status);

	if (rc != 0) {
		return rc;
	}

	if (status != MRISC_MSG_TYPE_NONE) {
		LOG_WRN("GDDR %d message buffer is not free. Current value: 0x%x", gddr_inst,
			status);
		return -EBUSY;
	}
	return 0;
}

static int wait_mrisc_not_busy(uint8_t gddr_inst, k_timepoint_t timeout, const char *op_desc)
{
	uint32_t status;
	int rc;

	do {
		rc = MriscRegRead32(gddr_inst, MRISC_MSG_REGISTER, &status);
		if (rc != 0) {
			return rc;
		}
		if (status == MRISC_MSG_TYPE_NONE) {
			return 0;
		}
		/* Wait for the message to be processed */
		if (sys_timepoint_expired(timeout)) {
			LOG_ERR("Timeout after %d ms waiting for GDDR instance %d to run %s",
				MRISC_MEMTEST_TIMEOUT, gddr_inst, op_desc);
			return -ETIMEDOUT;
		}
		k_busy_wait(100);
	} while (true);
}

static int StartHwMemtest(uint8_t gddr_inst, uint32_t addr_bits, uint32_t start_addr, uint32_t mask)
{
	uint32_t msg_args[3] = {addr_bits, start_addr, mask};

	/* Only run if MRISC FW support it. Must be > 2.6 */
	gddr_telemetry_table_t gddr_telemetry;

	if (read_gddr_telemetry_table(gddr_inst, &gddr_telemetry) < 0) {
		LOG_WRN("Failed to read GDDR telemetry table while starting memtest");
		return -ENOTSUP;
	}
	if (gddr_telemetry.mrisc_fw_version_major < 2 ||
	    (gddr_telemetry.mrisc_fw_version_major == 2 &&
	     gddr_telemetry.mrisc_fw_version_minor < 7)) {
		LOG_WRN("GDDR %d MRISC FW version %d.%d does not support memtest", gddr_inst,
			gddr_telemetry.mrisc_fw_version_major,
			gddr_telemetry.mrisc_fw_version_minor);
		return -ENOTSUP;
	}

	/*
	 * Messaging should not be done concurrently to the same GDDR instance, but still do sanity
	 * check if the message buffer is free.
	 */
	int32_t ret = check_mrisc_busy(gddr_inst);

	if (ret != 0) {
		return ret;
	}

	if (addr_bits > 26) {
		LOG_WRN("Invalid number of address bits for memory test. Expected <= 26, got %d",
			addr_bits);
		return -EINVAL;
	}
	for (int i = 0; i < 3; i++) {
		ret = MriscL1Write32(gddr_inst, GDDR_MSG_STRUCT_ADDR + i * 4, msg_args[i]);
		if (ret != 0) {
			return ret;
		}
	}
	return MriscRegWrite32(gddr_inst, MRISC_MSG_REGISTER, MRISC_MSG_TYPE_RUN_MEMTEST);
}

static int CheckHwMemtestResult(uint8_t gddr_inst, k_timepoint_t timeout)
{
	/* This should only be called after StartHwMemtest() has already been called. */
	int32_t ret = wait_mrisc_not_busy(gddr_inst, timeout, "memtest");

	if (ret != 0) {
		gddr_bist.failed |= BIT(gddr_inst);
		return ret;
	}

	uint32_t pass;

	ret = MriscL1Read32(gddr_inst, GDDR_MSG_STRUCT_ADDR + 8 * 4, &pass);
	if (ret != 0) {
		gddr_bist.failed |= BIT(gddr_inst);
		return ret;
	}

	gddr_bist.complete |= BIT(gddr_inst);
	if (pass != 0) {
		gddr_bist.failed |= BIT(gddr_inst);
		LOG_ERR("GDDR %d memory test failed", gddr_inst);
		return -EIO;
	}
	gddr_bist.failed &= ~BIT(gddr_inst);
	LOG_DBG("GDDR %d memory test passed", gddr_inst);
	return 0;
}

static int wait_for_gddr_wipe_dma(void)
{
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(GDDR_WIPE_TIMEOUT_MS));
	struct dma_status status;
	int rc;

	do {
		rc = dma_get_status(dma_noc, GDDR_WIPE_DMA_CHANNEL, &status);
		if (rc != 0) {
			return rc;
		}
		if (!status.busy) {
			return 0;
		}
		k_busy_wait(10);
	} while (!sys_timepoint_expired(timeout));

	return -ETIMEDOUT;
}

/* This function assumes that tensix L1s have already been cleared */
static int wipe_l1(void)
{
	uint8_t noc_id = 0;
	uint64_t addr = 0;
	uint32_t dram_mask = GetDramMask();
	uint8_t tensix_x, tensix_y;

	GetEnabledTensix(&tensix_x, &tensix_y);

	struct dma_block_config block = {
		.source_address = addr,
		.dest_address = addr,
		.block_size = MRISC_L1_SIZE,
	};

	struct dma_config config = {.channel_direction = PERIPHERAL_TO_MEMORY,
				    .source_data_size = 1,
				    .dest_data_size = 1,
				    .source_burst_length = 1,
				    .dest_burst_length = 1,
				    .block_count = 1,
				    .head_block = &block};

	struct tt_bh_dma_noc_coords coords = {.source_x = tensix_x, .source_y = tensix_y};

	for (uint32_t gddr_inst = 0; gddr_inst < NUM_GDDR; gddr_inst++) {
		if (IS_BIT_SET(dram_mask, gddr_inst)) {
			for (uint32_t noc2axi_port = 0; noc2axi_port < NUM_MRISC_NOC2AXI_PORT;
			     noc2axi_port++) {
				GetGddrNocCoords(gddr_inst, noc2axi_port, noc_id, &coords.dest_x,
						 &coords.dest_y);

				/* AXI enable must not be set, using MRISC address 0 */
				int rc = tt_dma_config(dma_noc, GDDR_WIPE_DMA_CHANNEL, &config,
						       &coords);

				if (rc == 0) {
					rc = dma_start(dma_noc, GDDR_WIPE_DMA_CHANNEL);
					if (rc == -ETIMEDOUT) {
						dma_stop(dma_noc, GDDR_WIPE_DMA_CHANNEL);
					}
				}
				if (rc == 0) {
					rc = wait_for_gddr_wipe_dma();
					if (rc != 0) {
						/*
						 * NoC DMA has no abort; stop quarantines the
						 * engine.
						 */
						dma_stop(dma_noc, GDDR_WIPE_DMA_CHANNEL);
					}
				}
				if (rc != 0) {
					return rc;
				}
			}
		}
	}

	return 0;
}

static int InitMrisc(void)
{
	SetPostCode(POST_CODE_SRC_CMFW, POST_CODE_ARC_INIT_STEP9);

	if (IS_ENABLED(CONFIG_TT_SMC_RECOVERY) || !IS_ENABLED(CONFIG_ARC)) {
		return 0;
	}

	/* In cable fault mode, GDDR tiles are clock-gated - skip MRISC init
	 * since tiles are not active.
	 */
	if (is_cable_fault_mode()) {
		return 0;
	}

	int rc = wipe_l1();

	if (rc != 0) {
		LOG_ERR("%s() failed: %d", "wipe_l1", rc);
		record_init_failure(INIT_STAGE_MRISC_LOAD);
		return rc;
	}

	/* Load MRISC (DRAM RISC) FW to all DRAMs in the middle NOC node */

	for (uint8_t gddr_inst = 0; gddr_inst < NUM_GDDR; gddr_inst++) {
		for (uint8_t noc2axi_port = 0; noc2axi_port < NUM_MRISC_NOC2AXI_PORT;
		     noc2axi_port++) {
			SetAxiEnable(gddr_inst, noc2axi_port, true);
		}
	}

	uint32_t dram_mask = GetDramMask();

	tt_boot_fs_fd tag_fd;
	size_t image_size;
	size_t spi_address;

	uint8_t buf[SCRATCHPAD_SIZE] __aligned(4);

	rc = tt_boot_fs_find_fd_by_tag(flash, MRISC_FW_TAG, &tag_fd);
	if (rc < 0) {
		LOG_ERR("%s (%s) failed: %d", "tt_boot_fs_find_fd_by_tag", MRISC_FW_TAG, rc);
		record_init_failure(INIT_STAGE_MRISC_LOAD);
		return rc;
	}
	image_size = tag_fd.flags.f.image_size;
	spi_address = tag_fd.spi_addr;

	for (uint8_t gddr_inst = 0; gddr_inst < NUM_GDDR; gddr_inst++) {
		if (IS_BIT_SET(dram_mask, gddr_inst)) {
			if (LoadMriscFw(gddr_inst, buf, SCRATCHPAD_SIZE, spi_address, image_size)) {
				LOG_ERR("%s(%d) failed: %d", "LoadMriscFw", gddr_inst, -EIO);
				record_init_failure(INIT_STAGE_MRISC_LOAD);
				return -EIO;
			}
		}
	}

	rc = tt_boot_fs_find_fd_by_tag(flash, MRISC_FW_CFG_TAG, &tag_fd);
	if (rc < 0) {
		LOG_ERR("%s (%s) failed: %d", "tt_boot_fs_find_fd_by_tag", MRISC_FW_CFG_TAG, rc);
		record_init_failure(INIT_STAGE_MRISC_LOAD);
		return rc;
	}
	image_size = tag_fd.flags.f.image_size;
	spi_address = tag_fd.spi_addr;

	/* Loading ETH FW configuration data requires the whole data to be loaded into buffer */
	__ASSERT(SCRATCHPAD_SIZE >= image_size,
		 "spi buffer size %zu must be larger than image size %zu", SCRATCHPAD_SIZE,
		 image_size);

	rc = flash_read(flash, spi_address, buf, image_size);
	if (rc < 0) {
		LOG_ERR("%s() failed: %d", "flash_read", rc);
		record_init_failure(INIT_STAGE_MRISC_LOAD);
		return rc;
	}

	uint32_t gddr_speed = GetGddrSpeedFromCfg(buf);

	if (!IN_RANGE(gddr_speed, MIN_GDDR_SPEED, MAX_GDDR_SPEED)) {
		LOG_WRN("%s() failed: %d", "GetGddrSpeedFromCfg", gddr_speed);
		gddr_speed = MIN_GDDR_SPEED;
	}

	if (clock_control_set_rate(
		    pll_dev_3, (clock_control_subsys_t)CLOCK_CONTROL_TT_BH_CLOCK_GDDRMEMCLK,
		    (clock_control_subsys_rate_t)(gddr_speed / GDDR_SPEED_TO_MEMCLK_RATIO))) {
		LOG_ERR("%s(%d) failed: %d", "SetGddrMemClk", gddr_speed, -EIO);
		record_init_failure(INIT_STAGE_MRISC_LOAD);
		return -EIO;
	}

	for (uint8_t gddr_inst = 0; gddr_inst < NUM_GDDR; gddr_inst++) {
		if (IS_BIT_SET(dram_mask, gddr_inst)) {
			if (LoadMriscFwCfg(gddr_inst, buf, SCRATCHPAD_SIZE, spi_address,
					   image_size)) {
				LOG_ERR("%s(%d) failed: %d", "LoadMriscFwCfg", gddr_inst, -EIO);
				record_init_failure(INIT_STAGE_MRISC_LOAD);
				return -EIO;
			}
			rc = MriscRegWrite32(gddr_inst, MRISC_INIT_STATUS, MRISC_INIT_BEFORE);
			if (rc != 0) {
				LOG_ERR("Failed to initialize MRISC %u status: %d", gddr_inst, rc);
				record_init_failure(INIT_STAGE_MRISC_LOAD);
				return rc;
			}
			rc = ReleaseMriscReset(gddr_inst);
			if (rc != 0) {
				LOG_ERR("Failed to release MRISC %u reset: %d", gddr_inst, rc);
				record_init_failure(INIT_STAGE_MRISC_LOAD);
				return rc;
			}
		}
	}

	return 0;
}
SYS_INIT_APP(InitMrisc);

static int CheckGddrTraining(uint8_t gddr_inst, k_timepoint_t timeout)
{
	gddr_telemetry_version_ok &= ~BIT(gddr_inst);

	do {
		uint32_t poll_val;
		int rc = MriscRegRead32(gddr_inst, MRISC_INIT_STATUS, &poll_val);

		if (rc != 0) {
			return rc;
		}

		if (poll_val == MRISC_INIT_FINISHED) {
			uint32_t version;

			rc = MriscL1Read32(
				gddr_inst,
				GDDR_TELEMETRY_TABLE_ADDR +
					offsetof(gddr_telemetry_table_t, telemetry_table_version),
				&version);
			if (rc != 0) {
				return rc;
			}

			if (version != GDDR_TELEMETRY_TABLE_T_VERSION) {
				LOG_ERR("%s[%d]: version mismatch: %d (expected %d)",
					"GDDR telemetry table", gddr_inst, version,
					GDDR_TELEMETRY_TABLE_T_VERSION);
				return -ENOTSUP;
			}
			gddr_telemetry_version_ok |= BIT(gddr_inst);
			return 0;
		}
		if (poll_val == MRISC_INIT_FAILED) {
			LOG_ERR("%s[%d]: 0x%x", "MRISC_INIT_STATUS", gddr_inst, poll_val);
			return -EIO;
		}
		k_msleep(1);
	} while (!sys_timepoint_expired(timeout));

	uint32_t post_code;
	int rc = MriscRegRead32(gddr_inst, MRISC_POST_CODE, &post_code);

	if (rc == 0) {
		LOG_ERR("%s[%d]: 0x%x", "MRISC_POST_CODE", gddr_inst, post_code);
	} else {
		LOG_ERR("Failed to read MRISC_POST_CODE[%d]: %d", gddr_inst, rc);
	}

	return -ETIMEDOUT;
}

static int CheckGddrHwTest(void)
{
	/* First kick off all tests in parallel, then check their results. Test will take
	 * approximately 300-400 ms.
	 */
	uint8_t test_started = 0; /* Bitmask of tests started */
	int any_error = 0;

	for (uint8_t gddr_inst = 0; gddr_inst < NUM_GDDR; gddr_inst++) {
		if (IS_BIT_SET(tile_enable.gddr_enabled, gddr_inst)) {
			int error = StartHwMemtest(gddr_inst, 26, 0, 0);

			if (error == -ENOTSUP) {
				/* Shouldn't be considered a test failure if MRISC FW is too old. */
				LOG_DBG("%s(%d) %s: %d", "StartHwMemtest", gddr_inst, "skipped",
					error);
			} else if (error < 0) {
				LOG_ERR("%s(%d) %s: %d", "StartHwMemtest", gddr_inst, "failed",
					error);
				any_error = -EIO;
			} else {
				test_started |= BIT(gddr_inst);
			}
		}
	}
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(MRISC_MEMTEST_TIMEOUT));

	for (uint8_t gddr_inst = 0; gddr_inst < NUM_GDDR; gddr_inst++) {
		if (IS_BIT_SET(test_started, gddr_inst)) {
			int error = CheckHwMemtestResult(gddr_inst, timeout);

			if (error < 0) {
				any_error = -EIO;
				LOG_ERR("%s(%d) %s: %d", "CheckHwMemtestResult", gddr_inst,
					"failed", error);
			} else {
				LOG_DBG("%s(%d) %s: %d", "CheckHwMemtestResult", gddr_inst,
					"succeeded", error);
			}
		}
	}
	return any_error;
}

static int gddr_training(void)
{
	SetPostCode(POST_CODE_SRC_CMFW, POST_CODE_ARC_INIT_STEPE);

	/* Check GDDR training status. */
	if (IS_ENABLED(CONFIG_TT_SMC_RECOVERY) || !IS_ENABLED(CONFIG_ARC)) {
		return 0;
	}

	/* In cable fault mode, GDDR tiles are clock-gated - skip training
	 * since tiles are not active.
	 */
	if (is_cable_fault_mode()) {
		return 0;
	}

	bool init_errors = false;
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(MRISC_INIT_TIMEOUT));

	for (uint8_t gddr_inst = 0; gddr_inst < NUM_GDDR; gddr_inst++) {
		if (IS_BIT_SET(GetDramMask(), gddr_inst)) {
			int error = CheckGddrTraining(gddr_inst, timeout);

			if (error == -ETIMEDOUT) {
				LOG_ERR("GDDR instance %d timed out during training", gddr_inst);
				init_errors = true;
			} else if (error) {
				LOG_ERR("GDDR instance %d failed training", gddr_inst);
				init_errors = true;
			}
		}
	}

	if (!init_errors) {
		/* this is needed to securely wipe DRAM */
		if (CheckGddrHwTest() < 0) {
			LOG_ERR("GDDR HW test failed");
			record_init_failure(INIT_STAGE_GDDR_TRAIN);
			return -EIO;
		}
	} else {
		record_init_failure(INIT_STAGE_GDDR_TRAIN);
	}

	return 0;
}

static int32_t mrisc_message(uint32_t op_code, uint32_t instance_mask, uint32_t timeout_ms,
			     const char *op_desc)
{
	for (uint8_t gddr_inst = 0U; gddr_inst < NUM_GDDR; gddr_inst++) {
		if (IS_BIT_SET(instance_mask, gddr_inst)) {

			int ret = check_mrisc_busy(gddr_inst);

			if (ret != 0) {
				return ret;
			}
			ret = MriscRegWrite32(gddr_inst, MRISC_MSG_REGISTER, op_code);
			if (ret != 0) {
				return ret;
			}
		}
	}
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(timeout_ms));

	for (uint8_t gddr_inst = 0U; gddr_inst < NUM_GDDR; gddr_inst++) {
		if (IS_BIT_SET(instance_mask, gddr_inst)) {

			int ret = wait_mrisc_not_busy(gddr_inst, timeout, op_desc);

			if (ret != 0) {
				return ret;
			}
		}
	}

	return 0;
}

int32_t set_mrisc_power_setting(bool on)
{
	uint32_t op_code = on ? MRISC_MSG_TYPE_PHY_WAKEUP : MRISC_MSG_TYPE_PHY_POWERDOWN;

	return mrisc_message(op_code, GetDramMask(), MRISC_POWER_SETTING_TIMEOUT_MS,
			     "power_setting");
}

SYS_INIT_APP(gddr_training);

static int assert_mrisc_soft_reset(uint8_t gddr_inst)
{
	const uint32_t kSoftReset0Addr = 0xFFB121B0;
	const uint32_t kAllRiscSoftReset = 0x47800;
	int rc;

	for (uint8_t noc_node = 0; noc_node < NUM_MRISC_NOC2AXI_PORT; noc_node++) {
		uint8_t x, y;

		GetGddrNocCoords(gddr_inst, noc_node, 0, &x, &y);
		rc = gddr_remote_write32_xy(x, y, kSoftReset0Addr, 0, kAllRiscSoftReset);
		if (rc != 0) {
			return rc;
		}
	}
	return 0;
}

/**
 * @brief Toggle GDDR MRISC reset, re-train, and re-run BIST
 *
 * Sequence:
 *   1) Assert MRISC soft reset
 *   2) Release MRISC reset
 *   3) Wait for GDDR training completion
 *   4) Re-run BIST
 *   5) If PHY was powered down, issue PHY wakeup
 */
static uint8_t toggle_gddr_reset(const union request *req, struct response *rsp)
{
	uint32_t gddr_inst = req->gddr_reset.gddr_inst;
	bool original_mrisc_state;
	int rc;

	bh_power_state_get(BH_POWER_DOMAIN_MRISC, &original_mrisc_state);

	if (gddr_inst >= NUM_GDDR) {
		rsp->data[1] = GDDR_RESET_ERR_INVALID_INST;
		return 1;
	}

	if (!IS_BIT_SET(tile_enable.gddr_enabled, gddr_inst)) {
		rsp->data[1] = GDDR_RESET_ERR_HARVESTED;
		return 1;
	}

	if (!IS_BIT_SET(GetDramMask(), gddr_inst)) {
		rsp->data[1] = GDDR_RESET_ERR_MASKED;
		return 1;
	}

	gddr_bist.complete &= ~BIT(gddr_inst);
	gddr_bist.failed &= ~BIT(gddr_inst);

	if (!original_mrisc_state) {
		rc = set_mrisc_power_setting(true);
		if (rc < 0) {
			rsp->data[1] = GDDR_RESET_ERR_POWERDOWN;
			return 1;
		}
	}

	rc = assert_mrisc_soft_reset(gddr_inst);
	if (rc != 0) {
		rsp->data[1] = GDDR_RESET_ERR_TRAINING;
		return 1;
	}

	rc = MriscRegWrite32(gddr_inst, MRISC_INIT_STATUS, MRISC_INIT_BEFORE);
	if (rc != 0) {
		rsp->data[1] = GDDR_RESET_ERR_TRAINING;
		return 1;
	}
	rc = ReleaseMriscReset(gddr_inst);
	if (rc != 0) {
		rsp->data[1] = GDDR_RESET_ERR_TRAINING;
		return 1;
	}

	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(MRISC_INIT_TIMEOUT));

	rc = CheckGddrTraining(gddr_inst, timeout);
	if (rc < 0) {
		rsp->data[1] = GDDR_RESET_ERR_TRAINING;
		return 1;
	}

	rc = StartHwMemtest(gddr_inst, 26, 0, 0);
	if (rc < 0) {
		rsp->data[1] = GDDR_RESET_ERR_BIST;
		return 1;
	}

	timeout = sys_timepoint_calc(K_MSEC(MRISC_MEMTEST_TIMEOUT));
	rc = CheckHwMemtestResult(gddr_inst, timeout);
	if (rc < 0) {
		rsp->data[1] = GDDR_RESET_ERR_BIST;
		return 1;
	}

	if (!original_mrisc_state) {
		rc = set_mrisc_power_setting(false);
		if (rc < 0) {
			rsp->data[1] = GDDR_RESET_ERR_POWERDOWN;
			return 1;
		}
	}

	rsp->data[1] = 0;
	return 0;
}

#ifndef CONFIG_TT_SMC_RECOVERY
REGISTER_MESSAGE(TT_SMC_MSG_TOGGLE_GDDR_RESET, toggle_gddr_reset, MSGQUEUE_COMMAND_DENIED);
#endif
