/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cat.h"
#include "dvfs.h"
#include "fan_ctrl.h"
#include "init.h"
#include "reg.h"
#include "status_reg.h"
#include "telemetry.h"
#include "tensix.h"
#include "throttler.h"
#include "timer.h"

#include <stdint.h>

#if defined(HAS_APP_VERSION)
#include <zephyr/app_version.h>
#else
#define APPVERSION         0x00000000
#define APP_VERSION_STRING "unknown"
#endif

#include <tenstorrent/msgqueue.h>
#include <tenstorrent/bh_power.h>
#include <tenstorrent/post_code.h>
#include <tenstorrent/sys_init_defines.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#ifdef CONFIG_BH_FWTABLE
#include <zephyr/drivers/misc/bh_fwtable.h>
#endif

LOG_MODULE_REGISTER(bh_arc_init, CONFIG_TT_APP_LOG_LEVEL);

#ifdef CONFIG_BH_FWTABLE
static const struct device *const fwtable_dev = DEVICE_DT_GET(DT_NODELABEL(fwtable));
#endif

#define FW_VERSION_SEMANTIC APPVERSION
#define FW_VERSION_DATE     0x00000000
#define FW_VERSION_LOW      0x00000000
#define FW_VERSION_HIGH     0x00000000

uint32_t FW_VERSION[4] __attribute__((section(".fw_version"))) = {
	FW_VERSION_SEMANTIC, FW_VERSION_DATE, FW_VERSION_LOW, FW_VERSION_HIGH};

static int tt_appversion_init(void)
{
	WriteReg(STATUS_FW_VERSION_REG_ADDR, APPVERSION);
	return 0;
}
SYS_INIT(tt_appversion_init, EARLY, 0);

static int record_cmfw_start_time(void)
{
	WriteReg(CMFW_START_TIME_REG_ADDR, TimerTimestamp());
	return 0;
}
SYS_INIT(record_cmfw_start_time, EARLY, 0);

static int bh_arc_init_start(void)
{
	/* Write a status register indicating HW init progress */
	STATUS_BOOT_STATUS0_reg_u boot_status0 = {0};

	boot_status0.val = ReadReg(STATUS_BOOT_STATUS0_REG_ADDR);
	boot_status0.f.hw_init_status = kHwInitStarted;
	WriteReg(STATUS_BOOT_STATUS0_REG_ADDR, boot_status0.val);

	SetPostCode(POST_CODE_SRC_CMFW, POST_CODE_ARC_INIT_STEP1);
	SetPostCode(POST_CODE_SRC_CMFW, POST_CODE_ARC_INIT_STEP2);

	return 0;
}
SYS_INIT_APP(bh_arc_init_start);

bool BhArcInitializationStarted(void)
{
	STATUS_BOOT_STATUS0_reg_u boot_status0 = {
		.val = ReadReg(STATUS_BOOT_STATUS0_REG_ADDR),
	};

	return boot_status0.f.hw_init_status != kHwInitNotStarted;
}

static int bh_arc_init_end(void)
{
#if defined(CONFIG_BH_FWTABLE) && !defined(CONFIG_TT_BH_ARC_EMUL)
	if (tt_bh_fwtable_get_fw_table(fwtable_dev)->feature_enable.aiclk_ppm_en) {
		uint32_t err0 = ReadReg(STATUS_ERROR_STATUS0_REG_ADDR);

		if (err0 & BIT(INIT_STAGE_REGULATOR)) {
			LOG_ERR("Not enabling AICLK PPM due to regulator init error");
		} else {
			/* DVFS should get enabled if AICLK PPM or L2CPUCLK PPM is enabled
			 * We currently don't have plans to implement L2CPUCLK PPM,
			 * so currently, dvfs_enable == aiclk_ppm_enable
			 */
			if (InitDVFS() != 0) {
				LOG_ERR("AICLK PPM initialization failed");
				record_init_failure(INIT_STAGE_REGULATOR);
			}
		}
	}
#endif

#if !defined(CONFIG_TT_SMC_RECOVERY)
	/* IRQ-side power faults may arrive before, during, or instead of DVFS
	 * initialization. Enable the independent containment worker on every
	 * mission-firmware path so pending work cannot be stranded by an earlier
	 * initialization error or a disabled periodic DVFS loop.
	 */
	ThrottlerEnableRuntimeContainmentWorker();
#endif

#ifdef CONFIG_TT_BH_ARC_MSGQUEUE_ENABLED
	init_msgqueue();
#endif

#ifdef CONFIG_BH_FWTABLE
	init_telemetry(APPVERSION);
	if (tt_bh_fwtable_get_fw_table(fwtable_dev)->feature_enable.fan_ctrl_en) {
		init_fan_ctrl();
	}

	/* These timers are split out from their init functions since their work tasks have
	 * i2c conflicts with other init functions.
	 *
	 * Note: The above issue would be solved by using Zephyr's driver model.
	 */
	StartTelemetryTimer();

#if !defined(CONFIG_TT_BH_ARC_EMUL)
	if (dvfs_enabled) {
		StartDVFSTimer();
	}
#endif
	StartGddrThermTripMonitor();
#endif

#if !defined(CONFIG_TT_SMC_RECOVERY) && defined(CONFIG_ARC)
	/* Close the retained-workload boot window: the wipe path left every
	 * Tensix RISC in reset. Release only as one interrupt-serialized transition
	 * immediately before publishing readiness, after the strict board-power
	 * policy and all host-facing runtime services are initialized.
	 */
	bool compute_prepared = ThrottlerPrepareComputeRelease();
	unsigned int key = irq_lock();
	bool compute_ready = compute_prepared && ThrottlerComputePowerPolicyReady();

	if (compute_ready) {
		bh_release_tensix_riscs_from_reset();
		/* The boot wipe ran while hardware RISC reset was held. Re-issue the
		 * first-instruction workaround after the safe release transition.
		 */
		tensix_inject_instruction(TENSIX_INSTRUCTION_UNPACR, 0, true, 0, 0);
	} else {
		bh_hold_tensix_riscs_in_reset();
	}
	irq_unlock(key);

	if (!compute_ready && error_status0 == 0U) {
		LOG_ERR("Board-power policy not ready; keeping Tensix hardware reset asserted");
		record_init_failure(INIT_STAGE_CABLE_FAULT);
	}
	if (!compute_ready) {
		/* record_init_failure() synchronously stops every programmable RISC;
		 * this request additionally latches the clock/voltage safe state.
		 */
		ThrottlerRequestRuntimeContainment();
	}
#endif

	/* Publish successful initialization only after DVFS, the board-power
	 * policy, telemetry, and the host message queue have all been initialized.
	 * Host software is allowed to power domains and raise AICLK after observing
	 * kHwInitDone, so publishing it earlier creates an unprotected boot window.
	 */
	STATUS_BOOT_STATUS0_reg_u boot_status0 = {
		.val = ReadReg(STATUS_BOOT_STATUS0_REG_ADDR),
	};

	boot_status0.f.fw_id = IS_ENABLED(CONFIG_TT_SMC_RECOVERY) ? FW_ID_SMC_RECOVERY
							       : FW_ID_SMC_NORMAL;
	boot_status0.f.hw_init_status = (error_status0 != 0) ? kHwInitError : kHwInitDone;
	WriteReg(STATUS_ERROR_STATUS0_REG_ADDR, error_status0);
	WriteReg(STATUS_BOOT_STATUS0_REG_ADDR, boot_status0.val);

	SetPostCode(POST_CODE_SRC_CMFW, POST_CODE_ZEPHYR_INIT_DONE);
	printk("Tenstorrent Blackhole CMFW %s\n", APP_VERSION_STRING);

	return 0;
}
SYS_INIT_APP(bh_arc_init_end);
