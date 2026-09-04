/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bh_reset.h"
#include "functional_efuse.h"
#include "eth.h"
#include "harvesting.h"
#include "init.h"
#include "noc.h"
#include "noc_init.h"
#include "noc2axi.h"
#include "reg.h"
#include "serdes_eth.h"
#include "aiclk_ppm.h"

#include <tenstorrent/msgqueue.h>
#include <tenstorrent/post_code.h>
#include <tenstorrent/smc_msg.h>
#include <tenstorrent/spi_flash_buf.h>
#include <tenstorrent/sys_init_defines.h>
#include <tenstorrent/tt_boot_fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <zephyr/drivers/misc/bh_fwtable.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_tt_bh_noc.h>
#include <zephyr/drivers/dma/dma_arc_hs.h>
#include <zephyr/device.h>

LOG_MODULE_REGISTER(eth, CONFIG_TT_APP_LOG_LEVEL);

#define ETH_SETUP_TLB              0
#define ETH_FW_BASE_ADDR           0x70000
#define ETH_PARAM_ADDR             0x7c000
#define ETH_FW_VERSION_ADDR_OFFSET 0x188
#define ETH_HEARTBEAT_ADDR         0x7cc70
#define ETH_SERDES_CFG_ADDR        0x7d600

#define ERISC_L1_SIZE (512 * 1024)

#define ETH_RESET_PC_0              0xFFB14000
#define ETH_END_PC_0                0xFFB14004
#define ETH_RESET_PC_1              0xFFB14008
#define ETH_END_PC_1                0xFFB1400C
#define ETH_RISC_DEBUG_SOFT_RESET_0 0xFFB121B0
#define ETH_PCS_STATUS              0xFFB9800C
#define ETH_ALL_RISC_SOFT_RESET     0x47800

#define ARC_NOC0_X 8U
#define ARC_NOC0_Y 0U

#define ETH_MAC_ADDR_ORG 0x208C47 /* 20:8C:47 */

#define ETH_FW_CFG_TAG     "ethfwcfg"
#define ETH_FW_TAG         "ethfw"
#define ETH_SD_REG_TAG     "ethsdreg"
#define ETH_SD_FW_TAG      "ethsdfw"
#define ETH_ALT_SD_REG_TAG "altsdreg"
#define ETH_ALT_SD_FW_TAG  "altsdfw"

#define ETH_WIPE_DMA_CHANNEL              1U
#define ETH_RUNTIME_TELEMETRY_DMA_CHANNEL 2U
#define ETH_WIPE_DMA_TIMEOUT_MS           50U
#define ETH_RUNTIME_TELEMETRY_TIMEOUT_MS  5U

static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));
static const struct device *flash = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(spi_flash));
static const struct device *const arc_dma_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(dma0));
static const struct device *dma_noc = DEVICE_DT_GET(DT_NODELABEL(dma1));

static uint32_t saved_heartbeat[MAX_ETH_INSTANCES];
static uint32_t cached_eth_fw_version;
#if !defined(CONFIG_TT_BH_ARC_EMUL)
static uint32_t eth_telemetry_readback __aligned(4);
#endif
static bool eth_runtime_telemetry_available;

typedef struct {
	uint32_t sd_mode_sel_0: 1;
	uint32_t sd_mode_sel_1: 1;
	uint32_t reserved_2: 1;
	uint32_t mux_sel: 2;
	uint32_t master_sel_0: 2;
	uint32_t master_sel_1: 2;
	uint32_t master_sel_2: 2;
	uint32_t reserved_31_11: 21;
} RESET_UNIT_PCIE_MISC_CNTL3_reg_t;

typedef union {
	uint32_t val;
	RESET_UNIT_PCIE_MISC_CNTL3_reg_t f;
} RESET_UNIT_PCIE_MISC_CNTL3_reg_u;

#define RESET_UNIT_PCIE_MISC_CNTL3_REG_DEFAULT 0x00000000

#define RESET_UNIT_PCIE1_MISC_CNTL_3_REG_ADDR 0x8003050C
#define RESET_UNIT_PCIE_MISC_CNTL_3_REG_ADDR  0x8003009C

static inline void SetupEthTlb(uint32_t eth_inst, uint32_t ring, uint64_t addr)
{
	/* Logical X,Y coordinates */
	uint8_t x, y;

	GetEthNocCoords(eth_inst, ring, &x, &y);

	NOC2AXITlbSetup(ring, ETH_SETUP_TLB, x, y, addr);
}

void SetupEthSerdesMux(uint32_t eth_enabled)
{
	RESET_UNIT_PCIE_MISC_CNTL3_reg_u pcie_misc_cntl3_reg, pcie1_misc_cntl3_reg;

	pcie_misc_cntl3_reg.val = ReadReg(RESET_UNIT_PCIE_MISC_CNTL_3_REG_ADDR);
	pcie1_misc_cntl3_reg.val = ReadReg(RESET_UNIT_PCIE1_MISC_CNTL_3_REG_ADDR);

	/* 4,5,6 */
	if (!IS_BIT_SET(eth_enabled, 4)) {
		pcie_misc_cntl3_reg.f.mux_sel = 0b11;
	} else if (!IS_BIT_SET(eth_enabled, 5)) {
		pcie_misc_cntl3_reg.f.mux_sel = 0b10;
	} else if (!IS_BIT_SET(eth_enabled, 6)) {
		pcie_misc_cntl3_reg.f.mux_sel = 0b00;
	}

	/* 7,8,9 */
	if (!IS_BIT_SET(eth_enabled, 7)) {
		pcie1_misc_cntl3_reg.f.mux_sel = 0b00;
	} else if (!IS_BIT_SET(eth_enabled, 8)) {
		pcie1_misc_cntl3_reg.f.mux_sel = 0b10;
	} else if (!IS_BIT_SET(eth_enabled, 9)) {
		pcie1_misc_cntl3_reg.f.mux_sel = 0b11;
	}

	WriteReg(RESET_UNIT_PCIE_MISC_CNTL_3_REG_ADDR, pcie_misc_cntl3_reg.val);
	WriteReg(RESET_UNIT_PCIE1_MISC_CNTL_3_REG_ADDR, pcie1_misc_cntl3_reg.val);
}

uint32_t GetEthSel(uint32_t eth_enabled)
{
	uint32_t eth_sel = 0;

	/* Turn on the correct ETH instances based on the mux selects */
	/* Mux selects should be set earlier in the init sequence, when reading */
	/* efuses and setting up harvesting information */
	RESET_UNIT_PCIE_MISC_CNTL3_reg_u pcie_misc_cntl3_reg, pcie1_misc_cntl3_reg;

	pcie_misc_cntl3_reg.val = ReadReg(RESET_UNIT_PCIE_MISC_CNTL_3_REG_ADDR);
	pcie1_misc_cntl3_reg.val = ReadReg(RESET_UNIT_PCIE1_MISC_CNTL_3_REG_ADDR);

	/* 0b01 is invalid/not used */
	if (pcie_misc_cntl3_reg.f.mux_sel == 0b00) {
		eth_sel |= BIT(4) | BIT(5); /* ETH 4, 5 */
	} else if (pcie_misc_cntl3_reg.f.mux_sel == 0b10) {
		eth_sel |= BIT(4) | BIT(6); /* ETH 4, 6 */
	} else if (pcie_misc_cntl3_reg.f.mux_sel == 0b11) {
		eth_sel |= BIT(5) | BIT(6); /* ETH 5, 6 */
	}

	/* 0b01 is invalid/not used */
	if (pcie1_misc_cntl3_reg.f.mux_sel == 0b00) {
		eth_sel |= BIT(9) | BIT(8); /* ETH 9, 8 */
	} else if (pcie1_misc_cntl3_reg.f.mux_sel == 0b10) {
		eth_sel |= BIT(9) | BIT(7); /* ETH 9, 7 */
	} else if (pcie1_misc_cntl3_reg.f.mux_sel == 0b11) {
		eth_sel |= BIT(8) | BIT(7); /* ETH 8, 7 */
	}

	/* Turn on the correct ETH instances based on pcie serdes properties */
	if (tt_bh_fwtable_get_fw_table(fwtable_dev)->pci0_property_table.pcie_mode ==
	    FwTable_PciPropertyTable_PcieMode_DISABLED) {
		/* Enable ETH 0-3 */
		eth_sel |= BIT(0) | BIT(1) | BIT(2) | BIT(3);
	} else if (tt_bh_fwtable_get_fw_table(fwtable_dev)->pci0_property_table.num_serdes == 1) {
		/* Only enable ETH 2,3 */
		eth_sel |= BIT(2) | BIT(3);
	}
	if (tt_bh_fwtable_get_fw_table(fwtable_dev)->pci1_property_table.pcie_mode ==
	    FwTable_PciPropertyTable_PcieMode_DISABLED) {
		/* Enable ETH 10-13 */
		eth_sel |= BIT(10) | BIT(11) | BIT(12) | BIT(13);
	} else if (tt_bh_fwtable_get_fw_table(fwtable_dev)->pci1_property_table.num_serdes == 1) {
		/* Only enable ETH 10,11 */
		eth_sel |= BIT(10) | BIT(11);
	}

	eth_sel &= eth_enabled;

	/* If eth_disable_mask_en is set then make sure the disabled eths are not enabled */
	if (tt_bh_fwtable_get_fw_table(fwtable_dev)->eth_property_table.eth_disable_mask_en) {
		eth_sel &= ~tt_bh_fwtable_get_fw_table(fwtable_dev)
				    ->eth_property_table.eth_disable_mask;
	}

	/* Make sure to send the mux_sel information as well so the ETH can configure itself
	 * correctly to SerDes lanes
	 * This is mainly for edge cases where a mux_sel enabled ETH is forcefilly disabled by the
	 * eth_disable_mask
	 * e.g. if pcie0 mux_sel is 0b00, ETH4 goes to SerDes 3 Lane 3:0, ETH5 goes to SerDes 3 Lane
	 * 7:4 but eth_disable_mask is 0b10000, then ETH4 is disabled and only ETH5 is enabled via
	 * eth_sel, at which point it becomes ambiguous which SerDes lane ETH5 should be connected
	 * to (3:0 or 7:4?)
	 * having the mux_sel information will allow ETH5 to disambiguate this
	 */
	return (pcie1_misc_cntl3_reg.f.mux_sel << 24) | (pcie_misc_cntl3_reg.f.mux_sel << 16) |
	       eth_sel;
}

uint64_t GetMacAddressBase(void)
{
	uint32_t asic_id = READ_FUNCTIONAL_EFUSE(ASIC_ID_LOW) & 0xFFFF;

	/* TODO: This will later be updated with the final code to create unique base MAC addresses
	 */
	uint32_t mac_addr_base_id = asic_id * 12;

	/* Base MAC address is 48 bits, concatenation of 2 24-bit values */
	uint64_t mac_addr_base = ((uint64_t)ETH_MAC_ADDR_ORG << 24) | (uint64_t)mac_addr_base_id;

	return mac_addr_base;
}

uint32_t GetEthFwVersion(uint32_t ring)
{
	ARG_UNUSED(ring);

	/* The version is immutable and was cached from the same SPI image loaded into
	 * every enabled ETH tile. Reading it from a tile here can block ARC forever
	 * when that tile is absent or stalled, preventing PCIe management recovery.
	 */
	return cached_eth_fw_version;
}

static int wait_for_eth_dma(uint32_t channel, uint32_t timeout_ms)
{
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(timeout_ms));
	struct dma_status status;
	int rc;

	do {
		rc = dma_get_status(dma_noc, channel, &status);
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

static int read_eth_remote32(uint8_t eth_inst, uint32_t ring, uint64_t address, uint32_t *value)
{
#if defined(CONFIG_TT_BH_ARC_EMUL)
	ARG_UNUSED(eth_inst);
	ARG_UNUSED(ring);
	ARG_UNUSED(address);
	*value = 0U;
	return 0;
#else
	uint8_t x, y;
	int rc;

	if (!eth_runtime_telemetry_available) {
		return -ENODEV;
	}
	if (ring != 0U) {
		return -ENOTSUP;
	}

	GetEthNocCoords(eth_inst, ring, &x, &y);
	eth_telemetry_readback = 0U;

	struct dma_block_config block = {
		.source_address = (uintptr_t)&eth_telemetry_readback,
		.dest_address = address,
		.block_size = sizeof(eth_telemetry_readback),
	};
	struct dma_config config = {
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_data_size = 1,
		.dest_data_size = 1,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.block_count = 1,
		.head_block = &block,
	};
	struct tt_bh_dma_noc_coords coords = {
		.source_x = ARC_NOC0_X,
		.source_y = ARC_NOC0_Y,
		.dest_x = x,
		.dest_y = y,
	};

	rc = tt_dma_config(dma_noc, ETH_RUNTIME_TELEMETRY_DMA_CHANNEL, &config, &coords);
	if (rc == 0) {
		rc = dma_start(dma_noc, ETH_RUNTIME_TELEMETRY_DMA_CHANNEL);
		if (rc == -ETIMEDOUT) {
			dma_stop(dma_noc, ETH_RUNTIME_TELEMETRY_DMA_CHANNEL);
		}
	}
	if (rc == 0) {
		rc = wait_for_eth_dma(ETH_RUNTIME_TELEMETRY_DMA_CHANNEL,
				      ETH_RUNTIME_TELEMETRY_TIMEOUT_MS);
		if (rc != 0) {
			/* NoC DMA has no abort; stop quarantines the whole engine. */
			dma_stop(dma_noc, ETH_RUNTIME_TELEMETRY_DMA_CHANNEL);
		}
	}
	if (rc != 0) {
		/* dma_stop() cannot cancel an in-flight NOC request. Permanently stop
		 * using this dedicated channel for the current boot so a late response
		 * cannot be mistaken for completion of a later telemetry request.
		 */
		eth_runtime_telemetry_available = false;
		LOG_WRN("Disabling ETH runtime telemetry after bounded read failure: %d", rc);
		return rc;
	}

	*value = eth_telemetry_readback;
	return 0;
#endif
}

uint32_t GetEthHeartbeatStatus(uint32_t ring)
{
	/* Look through all the enabled ETH tiles, and read the heartbeat from each tile's L1
	 * Compare the heartbeat status versus the saved heartbeat to see if it's still alive.
	 * Accumulate the heartbeat status of all the tiles into a bitmask
	 */
	uint32_t heartbeat_status = 0;

	for (uint8_t eth_inst = 0; eth_inst < MAX_ETH_INSTANCES; eth_inst++) {
		if (IS_BIT_SET(tile_enable.eth_enabled, eth_inst)) {
			uint32_t heartbeat;

			if (read_eth_remote32(eth_inst, ring, ETH_HEARTBEAT_ADDR, &heartbeat) !=
			    0) {
				break;
			}

			if (saved_heartbeat[eth_inst] != heartbeat) {
				heartbeat_status |= BIT(eth_inst);
			}
			saved_heartbeat[eth_inst] = heartbeat;
		}
	}

	return heartbeat_status;
}

uint32_t GetEthLinkStatus(uint32_t ring)
{
	/* Look through all the enabled ETH tiles, and read the PCS_STATUS from each tile's
	 * eth_ctrl_a register space to get the link status.
	 * Accumulate the link status of all the tiles into a bitmask
	 */
	uint32_t link_status = 0;

	for (uint8_t eth_inst = 0; eth_inst < MAX_ETH_INSTANCES; eth_inst++) {
		if (IS_BIT_SET(tile_enable.eth_enabled, eth_inst)) {
			uint32_t pcs_status;

			if (read_eth_remote32(eth_inst, ring, ETH_PCS_STATUS, &pcs_status) != 0) {
				break;
			}

			if (pcs_status) {
				link_status |= BIT(eth_inst);
			}
		}
	}

	return link_status;
}

void ReleaseEthReset(uint32_t eth_inst, uint32_t ring)
{
	SetupEthTlb(eth_inst, ring, ETH_RESET_PC_0);
	/* AssertSoftResets and the reset message both write this known value first.
	 * Avoid a remote read-modify-write: a missing ETH response would otherwise
	 * block ARC and all PCIe management indefinitely.
	 */
	NOC2AXIWrite32(ring, ETH_SETUP_TLB, ETH_RISC_DEBUG_SOFT_RESET_0,
		       ETH_ALL_RISC_SOFT_RESET & ~BIT(11));
}

int LoadEthFw(uint32_t eth_inst, uint32_t ring, uint8_t *buf, size_t buf_size, size_t spi_address,
	      size_t image_size)
{
	/* The shifting is to align the address to the lowest 16 bytes */
	/* uint32_t fw_load_addr = ((ETH_PARAM_ADDR - fw_size) >> 2) << 2; */
	uint32_t fw_load_addr = ETH_FW_BASE_ADDR;

	SetupEthTlb(eth_inst, ring, fw_load_addr);
	volatile uint32_t *eth_tlb = GetTlbWindowAddr(ring, ETH_SETUP_TLB, fw_load_addr);

	if (spi_arc_dma_transfer_to_tile(flash, spi_address, image_size, buf, buf_size,
					 (uint8_t *)eth_tlb)) {
		return -1;
	}

	SetupEthTlb(eth_inst, ring, ETH_RESET_PC_0);
	NOC2AXIWrite32(ring, ETH_SETUP_TLB, ETH_RESET_PC_0, fw_load_addr);
	NOC2AXIWrite32(ring, ETH_SETUP_TLB, ETH_END_PC_0, ETH_PARAM_ADDR - 0x4);

	return 0;
}

/**
 * @brief Load the ETH FW configuration data into ETH L1 memory
 * @param eth_inst ETH instance to load the FW config for
 * @param ring Load over NOC 0 or NOC 1
 * @param buf Scratch buffer for reading flash data
 * @param eth_enabled Bitmask of enabled ETH instances
 * @param spi_address SPI flash address of the FW config image
 * @param image_size Size of the FW config image in bytes
 * @return 0 on success, -1 on failure
 */
int LoadEthFwCfg(uint32_t eth_inst, uint32_t ring, uint8_t *buf, uint32_t eth_enabled,
		 size_t spi_address, size_t image_size)
{
	int rc;

	rc = flash_read(flash, spi_address, buf, image_size);
	if (rc < 0) {
		LOG_ERR("%s() failed: %d", "flash_read", rc);
		return rc;
	}

	uint32_t *fw_cfg_32b = (uint32_t *)buf;

	/* Pass in eth_sel based on harvesting info and PCIe configuration */
	fw_cfg_32b[0] = GetEthSel(eth_enabled);

	/* Check if speed overrides exist, */
	/* apply them if they are a valid speed setting (40G, 100G, 200G, 400G) */
	uint32_t speed_override =
		tt_bh_fwtable_get_fw_table(fwtable_dev)->eth_property_table.eth_speed_override;

	if (speed_override == 40 || speed_override == 100 || speed_override == 200 ||
	    speed_override == 400) {
		fw_cfg_32b[1] = speed_override;
	}

	/* Pass in some board/chip specific data for ETH to use */
	/* InitHW -> InitEth -> LoadEthFwCfg comes before init_telemtry, so cannot simply call for
	 * telemetry data here
	 */
	fw_cfg_32b[32] = tt_bh_fwtable_get_pcb_type(fwtable_dev);
	fw_cfg_32b[33] = tt_bh_fwtable_get_asic_location(fwtable_dev);
	fw_cfg_32b[34] = tt_bh_fwtable_get_read_only_table(fwtable_dev)->board_id >> 32;
	fw_cfg_32b[35] = tt_bh_fwtable_get_read_only_table(fwtable_dev)->board_id & 0xFFFFFFFF;
	/* Split the 48-bit MAC address into 2 24-bit values, separated by organisation ID and
	 * device ID
	 */
	uint64_t mac_addr_base = GetMacAddressBase();

	fw_cfg_32b[36] = (mac_addr_base >> 24) & 0xFFFFFF;
	fw_cfg_32b[37] = mac_addr_base & 0xFFFFFF;

	fw_cfg_32b[38] = READ_FUNCTIONAL_EFUSE(ASIC_ID_HIGH);
	fw_cfg_32b[39] = READ_FUNCTIONAL_EFUSE(ASIC_ID_LOW);
	fw_cfg_32b[40] = tile_enable.eth_enabled;

	/* Write the ETH Param table */
	SetupEthTlb(eth_inst, ring, ETH_PARAM_ADDR);
	volatile uint32_t *eth_tlb = GetTlbWindowAddr(ring, ETH_SETUP_TLB, ETH_PARAM_ADDR);

	if (dma_arc_hs_transfer(arc_dma_dev, 0, buf, (void *)eth_tlb, image_size, K_MSEC(500)) <
	    0) {
		LOG_ERR("DMA transfer failed");
		return -1;
	}

	return 0;
}

/* Load the SerDes cfg from the SPI into given ETH's L1 at ETH_SERDES_CFG_ADDR */
static int load_eth_serdes_cfg(uint32_t eth_inst, uint32_t ring, uint8_t *buf, size_t buf_size,
			       size_t spi_address, size_t image_size)
{
	SetupEthTlb(eth_inst, ring, ETH_SERDES_CFG_ADDR);
	volatile uint32_t *eth_tlb = GetTlbWindowAddr(ring, ETH_SETUP_TLB, ETH_SERDES_CFG_ADDR);

	if (spi_arc_dma_transfer_to_tile(flash, spi_address, image_size, buf, buf_size,
					 (uint8_t *)eth_tlb) < 0) {
		return -1;
	}

	return 0;
}

static bool load_alt_eth_serdes_cfg(uint8_t eth_inst)
{
	PcbType pcb_type;
	uint32_t asic_location;
	uint32_t speed_override;

	pcb_type = tt_bh_fwtable_get_pcb_type(fwtable_dev);
	asic_location = tt_bh_fwtable_get_asic_location(fwtable_dev);
	speed_override =
		tt_bh_fwtable_get_fw_table(fwtable_dev)->eth_property_table.eth_speed_override;

	if (pcb_type == PcbTypeUBB && (asic_location >= 5 && asic_location <= 8) &&
	    (eth_inst == 7 || eth_inst == 8 || eth_inst == 9) &&
	    (speed_override == 0 || speed_override == 400)) {
		return true;
	}

	return false;
}

static bool LoadAltSerdes(uint8_t serdes_inst)
{
	PcbType pcb_type;
	uint32_t asic_location;
	uint32_t speed_override;

	pcb_type = tt_bh_fwtable_get_pcb_type(fwtable_dev);
	asic_location = tt_bh_fwtable_get_asic_location(fwtable_dev);
	speed_override =
		tt_bh_fwtable_get_fw_table(fwtable_dev)->eth_property_table.eth_speed_override;

	if (pcb_type == PcbTypeUBB && (asic_location >= 5 && asic_location <= 8) &&
	    serdes_inst == 5 && (speed_override == 0 || speed_override == 400)) {
		return true;
	}

	return false;
}

static int SerdesEthInit(void)
{
	uint32_t ring = 0;
	int rc;
	tt_boot_fs_fd serdes_fw_fd;
	tt_boot_fs_fd alt_serdes_fw_fd;

	SetupEthSerdesMux(tile_enable.eth_enabled);

	uint32_t load_serdes = BIT(2) | BIT(5); /* Serdes 2, 5 are always for ETH */
	/* Select the other ETH Serdes instances based on pcie serdes properties */
	if (tt_bh_fwtable_get_fw_table(fwtable_dev)->pci0_property_table.pcie_mode ==
	    FwTable_PciPropertyTable_PcieMode_DISABLED) { /* Enable Serdes 0, 1 */
		load_serdes |= BIT(0) | BIT(1);
	} else if (tt_bh_fwtable_get_fw_table(fwtable_dev)->pci0_property_table.num_serdes ==
		   1) { /* Just enable Serdes 1 */
		load_serdes |= BIT(1);
	}
	if (tt_bh_fwtable_get_fw_table(fwtable_dev)->pci1_property_table.pcie_mode ==
	    FwTable_PciPropertyTable_PcieMode_DISABLED) { /* Enable Serdes 3, 4 */
		load_serdes |= BIT(3) | BIT(4);
	} else if (tt_bh_fwtable_get_fw_table(fwtable_dev)->pci1_property_table.num_serdes ==
		   1) { /* Just enable Serdes 4 */
		load_serdes |= BIT(4);
	}

	uint8_t buf[SCRATCHPAD_SIZE] __aligned(4);

	rc = tt_boot_fs_find_fd_by_tag(flash, ETH_SD_FW_TAG, &serdes_fw_fd);
	if (rc < 0) {
		LOG_ERR("%s(%s) failed: %d", "tt_boot_fs_find_fd_by_tag", ETH_SD_FW_TAG, rc);
		return rc;
	}

	rc = tt_boot_fs_find_fd_by_tag(flash, ETH_ALT_SD_FW_TAG, &alt_serdes_fw_fd);
	if (rc < 0) {
		LOG_ERR("%s(%s) failed: %d", "tt_boot_fs_find_fd_by_tag", ETH_ALT_SD_FW_TAG, rc);
		return rc;
	}

	/* Load fw */
	for (uint8_t serdes_inst = 0; serdes_inst < 6; serdes_inst++) {
		if (IS_BIT_SET(load_serdes, serdes_inst)) {
			if (LoadAltSerdes(serdes_inst)) {
				rc = LoadSerdesEthFw(serdes_inst, ring, buf, SCRATCHPAD_SIZE,
						     alt_serdes_fw_fd.spi_addr,
						     alt_serdes_fw_fd.flags.f.image_size);
			} else {
				rc = LoadSerdesEthFw(serdes_inst, ring, buf, SCRATCHPAD_SIZE,
						     serdes_fw_fd.spi_addr,
						     serdes_fw_fd.flags.f.image_size);
			}
			if (rc != 0) {
				LOG_ERR("%s(%u) failed: %d", "LoadSerdesEthFw", serdes_inst, rc);
				return rc;
			}
		}
	}

	return 0;
}

/* This function assumes that tensix L1s have already been cleared */
static int wipe_l1(void)
{
	uint8_t noc_id = 0;
	uint64_t addr = 0;
	uint8_t tensix_x, tensix_y;

	GetEnabledTensix(&tensix_x, &tensix_y);

	struct dma_block_config block = {
		.source_address = addr,
		.dest_address = addr,
		.block_size = ERISC_L1_SIZE,
	};

	struct dma_config config = {
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.source_data_size = 1,
		.dest_data_size = 1,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.block_count = 1,
		.head_block = &block,
	};

	struct tt_bh_dma_noc_coords coords = {.source_x = tensix_x, .source_y = tensix_y};

	for (uint8_t eth_inst = 0; eth_inst < MAX_ETH_INSTANCES; eth_inst++) {
		if (IS_BIT_SET(tile_enable.eth_enabled, eth_inst)) {
			GetEthNocCoords(eth_inst, noc_id, &coords.dest_x, &coords.dest_y);

			int rc = tt_dma_config(dma_noc, ETH_WIPE_DMA_CHANNEL, &config, &coords);

			if (rc == 0) {
				rc = dma_start(dma_noc, ETH_WIPE_DMA_CHANNEL);
				if (rc == -ETIMEDOUT) {
					dma_stop(dma_noc, ETH_WIPE_DMA_CHANNEL);
				}
			}
			if (rc == 0) {
				rc = wait_for_eth_dma(ETH_WIPE_DMA_CHANNEL,
						      ETH_WIPE_DMA_TIMEOUT_MS);
				if (rc != 0) {
					/* NoC DMA has no abort; stop quarantines the engine. */
					dma_stop(dma_noc, ETH_WIPE_DMA_CHANNEL);
				}
			}
			if (rc != 0) {
				return rc;
			}
		}
	}

	return 0;
}

static int EthInit(void)
{
	uint32_t ring = 0;
	int rc;
	tt_boot_fs_fd eth_fd;
	tt_boot_fs_fd eth_cfg_fd;
	tt_boot_fs_fd serdes_reg_fd;
	tt_boot_fs_fd alt_serdes_reg_fd;

	/* Early exit if no ETH tiles enabled */
	if (tile_enable.eth_enabled == 0) {
		return 0;
	}

	rc = wipe_l1();
	if (rc != 0) {
		LOG_ERR("%s() failed: %d", "wipe_l1", rc);
		return rc;
	}

	uint8_t buf[SCRATCHPAD_SIZE] __aligned(4);

	rc = tt_boot_fs_find_fd_by_tag(flash, ETH_FW_TAG, &eth_fd);
	if (rc < 0) {
		LOG_ERR("%s(%s) failed: %d", "tt_boot_fs_find_fd_by_tag", ETH_FW_TAG, rc);
		return rc;
	}
	if (eth_fd.flags.f.image_size < ETH_FW_VERSION_ADDR_OFFSET + sizeof(uint32_t)) {
		LOG_ERR("ETH firmware image is too small to contain its version");
		return -EINVAL;
	}
	rc = flash_read(flash, eth_fd.spi_addr + ETH_FW_VERSION_ADDR_OFFSET, &cached_eth_fw_version,
			sizeof(cached_eth_fw_version));
	if (rc != 0) {
		LOG_ERR("Failed to cache ETH firmware version: %d", rc);
		return rc;
	}

	rc = tt_boot_fs_find_fd_by_tag(flash, ETH_FW_CFG_TAG, &eth_cfg_fd);
	if (rc < 0) {
		LOG_ERR("%s(%s) failed: %d", "tt_boot_fs_find_fd_by_tag", ETH_FW_CFG_TAG, rc);
		return rc;
	}

	/* Loading ETH FW configuration data requires the whole data to be loaded into buffer */
	if (eth_cfg_fd.flags.f.image_size > SCRATCHPAD_SIZE) {
		LOG_ERR("ETH configuration image too large: %zu > %u",
			(size_t)eth_cfg_fd.flags.f.image_size, SCRATCHPAD_SIZE);
		return -E2BIG;
	}

	/* Load the SerDes cfg from SPI into each enabled ETH tile's L1 at ETH_SERDES_CFG_ADDR */
	rc = tt_boot_fs_find_fd_by_tag(flash, ETH_SD_REG_TAG, &serdes_reg_fd);
	if (rc < 0) {
		LOG_ERR("%s(%s) failed: %d", "tt_boot_fs_find_fd_by_tag", ETH_SD_REG_TAG, rc);
		return rc;
	}

	rc = tt_boot_fs_find_fd_by_tag(flash, ETH_ALT_SD_REG_TAG, &alt_serdes_reg_fd);
	if (rc < 0) {
		LOG_ERR("%s(%s) failed: %d", "tt_boot_fs_find_fd_by_tag", ETH_ALT_SD_REG_TAG, rc);
		return rc;
	}

	/* Load fw, params, and serdes cfg */
	for (uint8_t eth_inst = 0; eth_inst < MAX_ETH_INSTANCES; eth_inst++) {
		if (!IS_BIT_SET(tile_enable.eth_enabled, eth_inst)) {
			continue;
		}

		rc = LoadEthFw(eth_inst, ring, buf, SCRATCHPAD_SIZE, eth_fd.spi_addr,
			       eth_fd.flags.f.image_size);
		if (rc != 0) {
			LOG_ERR("%s(%u) failed: %d", "LoadEthFw", eth_inst, rc);
			return rc;
		}

		if (load_alt_eth_serdes_cfg(eth_inst)) {
			rc = load_eth_serdes_cfg(eth_inst, ring, buf, SCRATCHPAD_SIZE,
						 alt_serdes_reg_fd.spi_addr,
						 alt_serdes_reg_fd.flags.f.image_size);
		} else {
			rc = load_eth_serdes_cfg(eth_inst, ring, buf, SCRATCHPAD_SIZE,
						 serdes_reg_fd.spi_addr,
						 serdes_reg_fd.flags.f.image_size);
		}
		if (rc < 0) {
			LOG_ERR("%s(%u) failed: %d", "load_eth_serdes_cfg", eth_inst, rc);
			return rc;
		}

		rc = LoadEthFwCfg(eth_inst, ring, buf, tile_enable.eth_enabled, eth_cfg_fd.spi_addr,
				  eth_cfg_fd.flags.f.image_size);
		if (rc != 0) {
			LOG_ERR("%s(%u) failed: %d", "LoadEthFwCfg", eth_inst, rc);
			return rc;
		}
	}

	/* Deassert tile reset */
	for (uint8_t eth_inst = 0; eth_inst < MAX_ETH_INSTANCES; eth_inst++) {
		/* Clear saved heartbeat since we are releasing reset, so heartbeat starts from 0 */
		saved_heartbeat[eth_inst] = 0;

		if (!IS_BIT_SET(tile_enable.eth_enabled, eth_inst)) {
			continue;
		}

		ReleaseEthReset(eth_inst, ring);
	}

	eth_runtime_telemetry_available = true;
	return 0;
}

static void assert_eth_risc_soft_reset(uint32_t eth_inst, uint32_t ring)
{
	const uint32_t kAllRiscSoftReset = 0x47800;
	uint8_t x, y;

	GetEthNocCoords(eth_inst, ring, &x, &y);
	NOC2AXITlbSetup(ring, ETH_SETUP_TLB, x, y, ETH_RISC_DEBUG_SOFT_RESET_0);
	NOC2AXIWrite32(ring, ETH_SETUP_TLB, ETH_RISC_DEBUG_SOFT_RESET_0, kAllRiscSoftReset);
}

static uint8_t toggle_eth_reset_handler(const union request *req, struct response *rsp)
{
	const uint32_t ring = 0;
	const uint32_t valid_bits = (1U << MAX_ETH_INSTANCES) - 1U;
	uint32_t requested = req->eth_tile_reset.eth_inst_mask;
	const bool skip_fw = req->eth_tile_reset.no_fw_reload != 0;
	int rc;

	if (!IS_ENABLED(CONFIG_ARC)) {
		return 0;
	}

	if (requested & ~valid_bits) {
		rsp->data[1] = ETH_RESET_ERR_INVALID_MASK;
		return 1;
	}

	uint32_t mask = requested & tile_enable.eth_enabled;

	if (mask == 0) {
		rsp->data[1] = 0;
		return 0;
	}

	if (is_cable_fault_mode()) {
		rsp->data[1] = ETH_RESET_ERR_CABLE_FAULT;
		return 1;
	}

	tt_boot_fs_fd fw_fd;
	tt_boot_fs_fd cfg_fd;

	if (!skip_fw) {
		if (flash == NULL || !device_is_ready(flash)) {
			rsp->data[1] = ETH_RESET_ERR_NO_FLASH;
			return 1;
		}

		rc = tt_boot_fs_find_fd_by_tag(flash, ETH_FW_TAG, &fw_fd);
		if (rc < 0) {
			rsp->data[1] = ETH_RESET_ERR_FW_LOOKUP;
			return 1;
		}

		rc = tt_boot_fs_find_fd_by_tag(flash, ETH_FW_CFG_TAG, &cfg_fd);
		if (rc < 0) {
			rsp->data[1] = ETH_RESET_ERR_CFG_LOOKUP;
			return 1;
		}

		if (cfg_fd.flags.f.image_size > SCRATCHPAD_SIZE) {
			rsp->data[1] = ETH_RESET_ERR_CFG_SIZE;
			return 1;
		}
	}

	if (!SetAiclkResetSafe(true)) {
		rsp->data[1] = ETH_RESET_ERR_CLOCK;
		return 1;
	}

	RESET_UNIT_ETH_RESET_reg_u eth_reset = {.val = ReadReg(RESET_UNIT_ETH_RESET_REG_ADDR)};

	/* Assert tile and risc reset */
	eth_reset.f.eth_reset_n &= ~mask;
	eth_reset.f.eth_risc_reset_n &= ~mask;
	WriteReg(RESET_UNIT_ETH_RESET_REG_ADDR, eth_reset.val);

	/* Deassert tile reset */
	eth_reset.f.eth_reset_n |= mask;
	WriteReg(RESET_UNIT_ETH_RESET_REG_ADDR, eth_reset.val);

	/* Assert RISC soft reset via NOC for each tile. */
	for (uint8_t eth_inst = 0; eth_inst < MAX_ETH_INSTANCES; eth_inst++) {
		if (IS_BIT_SET(mask, eth_inst)) {
			assert_eth_risc_soft_reset(eth_inst, ring);
		}
	}

	/* Deassert risc reset */
	eth_reset.f.eth_risc_reset_n |= mask;
	WriteReg(RESET_UNIT_ETH_RESET_REG_ADDR, eth_reset.val);

	if (!SetAiclkResetSafe(false)) {
		rsp->data[1] = ETH_RESET_ERR_CLOCK;
		return 1;
	}

	if (!skip_fw) {
		uint8_t buf[SCRATCHPAD_SIZE] __aligned(4);

		for (uint8_t eth_inst = 0; eth_inst < MAX_ETH_INSTANCES; eth_inst++) {
			if (!IS_BIT_SET(mask, eth_inst)) {
				continue;
			}

			rc = LoadEthFw(eth_inst, ring, buf, sizeof(buf), fw_fd.spi_addr,
				       fw_fd.flags.f.image_size);
			if (rc < 0) {
				rsp->data[1] = ETH_RESET_ERR_FW_LOAD;
				return 1;
			}

			rc = LoadEthFwCfg(eth_inst, ring, buf, tile_enable.eth_enabled,
					  cfg_fd.spi_addr, cfg_fd.flags.f.image_size);
			if (rc < 0) {
				rsp->data[1] = ETH_RESET_ERR_CFG_LOAD;
				return 1;
			}
		}

		/* Start ERISC FW */
		for (uint8_t eth_inst = 0; eth_inst < MAX_ETH_INSTANCES; eth_inst++) {
			/* Ensure that saved heartbeat is cleared before releasing reset */
			saved_heartbeat[eth_inst] = 0;

			if (!IS_BIT_SET(mask, eth_inst)) {
				continue;
			}

			ReleaseEthReset(eth_inst, ring);
		}
	}

	rsp->data[1] = mask;
	return 0;
}

REGISTER_MESSAGE(TT_SMC_MSG_TOGGLE_ETH_RESET, toggle_eth_reset_handler, MSGQUEUE_COMMAND_DENIED);

static int eth_init(void)
{
	int rc;

	/* TODO: Load ERISC (Ethernet RISC) FW to all ethernets (8 of them) */
	SetPostCode(POST_CODE_SRC_CMFW, POST_CODE_ARC_INIT_STEPA);
	cached_eth_fw_version = 0U;
	eth_runtime_telemetry_available = false;
	if (IS_ENABLED(CONFIG_TT_SMC_RECOVERY) || !IS_ENABLED(CONFIG_ARC)) {
		return 0;
	}

	/* In cable fault mode, ETH tiles are clock-gated - skip init
	 * since tiles are not active.
	 */
	if (is_cable_fault_mode()) {
		return 0;
	}

	if (!device_is_ready(dma_noc) || flash == NULL || !device_is_ready(flash)) {
		return -ENODEV;
	}

	rc = SerdesEthInit();
	if (rc != 0) {
		return rc;
	}

	rc = EthInit();
	if (rc != 0) {
		return rc;
	}

	return 0;
}
SYS_INIT_APP(eth_init);
