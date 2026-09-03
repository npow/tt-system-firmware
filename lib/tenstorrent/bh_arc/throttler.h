/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef THROTTLER_H
#define THROTTLER_H

#include <stdint.h>
#include <stdbool.h>

void InitThrottlers(void);
void CalculateThrottlers(void);
int32_t Dm2CmSetBoardPowerLimit(const uint8_t *data, uint8_t size);
uint8_t ThrottlerSetKernelThrottlerEnabled(uint32_t enabled);
uint8_t ThrottlerSetKernelThrottlerStopFreq(uint32_t frequency);
uint32_t GetStartNOPCount(void);
uint32_t GetNOPOnAccumulatedTime(void);
/* ms NOP was on during the last telemetry update window, clamped to window_ms */
uint32_t GetNOPOnDuration(uint32_t window_ms);
bool ThrottlerRuntimePowerFaultLatched(void);
bool ThrottlerBoardPowerPolicyStrict(void);
bool ThrottlerBoardPowerSampleFresh(void);
bool ThrottlerBoardPowerPolicyInstalled(void);
bool ThrottlerBoardPowerPolicyReady(void);
bool ThrottlerBoardPowerPolicyRequired(void);
/** Capture a coherent idle sample while compute is still held in reset. */
bool ThrottlerPrepareComputeRelease(void);
/** True when this board's applicable power policy permits compute release. */
bool ThrottlerComputePowerPolicyReady(void);
/** Clamp/rearm activity detection before a host-requested rising transition. */
void ThrottlerBeginHighPowerTransition(void);
/** Capture a fresh post-enable idle reference while AICLK remains reset-safe. */
bool ThrottlerFinishHighPowerTransition(void);
/** Cancel a failed rising transition and restore the boot-idle reference. */
void ThrottlerAbortHighPowerTransition(void);
/** Publish one DMC input-power sample and enforce the IRQ-side threshold. */
void ThrottlerObserveInputPowerFromIsr(uint16_t current_power, uint32_t sample_time_ms);
/**
 * @brief Request irreversible runtime containment from any context.
 *
 * This function is IRQ-safe. The next DVFS pass performs the hardware work.
 */
void ThrottlerRequestRuntimeContainment(void);
/** Reassert and retry an already latched containment transaction. */
void ThrottlerRetryRuntimeContainment(void);
/** Enable asynchronous containment after DVFS state has been initialized. */
void ThrottlerEnableRuntimeContainmentWorker(void);

#if defined(CONFIG_ZTEST)
uint32_t ThrottlerGetDopplerT2PowerLimit(void);
uint32_t ThrottlerGetDopplerT3PowerLimit(void);
uint32_t ThrottlerGetRuntimePowerFailSafeLimit(void);
uint32_t ThrottlerGetDopplerSlowAiclkLimit(void);
bool ThrottlerTestUpdateRuntimePowerGuard(bool eligible, uint16_t current_power, uint32_t now_ms);
void ThrottlerTestResetRuntimePowerGuard(void);
uint16_t ThrottlerTestUpdateBoardPowerHistory(uint16_t current_power);
void ThrottlerTestResetBoardPowerHistory(uint16_t current_power);
bool ThrottlerTestRuntimeBoardPowerCritical(bool sample_fresh, uint16_t current_power);
bool ThrottlerTestRuntimeContainmentPending(void);
void ThrottlerTestPauseRuntimeContainmentWorker(bool pause);
void ThrottlerTestApplyPendingRuntimeContainment(uint16_t current_power);
void ThrottlerTestPrepareForInit(void);
void ThrottlerTestConfigureRuntimePowerWatchdog(uint16_t effective_limit,
					       uint32_t arm_ms);
bool ThrottlerTestCheckRuntimePowerWatchdog(uint32_t now_ms);
uint16_t ThrottlerTestRuntimePowerGuardLimit(void);
void ThrottlerTestSetHostBoardPowerPreApplyHook(void (*hook)(void));
void ThrottlerTestPauseDmcBoardPowerLimitWorker(bool pause);
bool ThrottlerTestDmcBoardPowerLimitPending(void);
void ThrottlerTestFlushDmcBoardPowerLimitWork(void);
void ThrottlerTestFlushRuntimeContainmentWork(void);
bool ThrottlerTestRuntimeActivityGateOpen(void);
bool ThrottlerTestRuntimeActivityBaselineValid(void);
void ThrottlerTestSetBoardPowerPolicyRequired(bool required);
void ThrottlerTestSetPowerTransitionSampleHook(void (*hook)(void));
#endif

#endif
