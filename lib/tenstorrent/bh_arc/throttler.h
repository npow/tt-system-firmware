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
/* Publish one complete DMC board-power sample to the fast guard. */
void ThrottlerRecordInputPowerSample(uint32_t now_ms, uint16_t input_power);
uint16_t ThrottlerGetInputPower(void);
uint8_t ThrottlerSetKernelThrottlerEnabled(uint32_t enabled);
uint8_t ThrottlerSetKernelThrottlerStopFreq(uint32_t frequency);
uint32_t GetStartNOPCount(void);
uint32_t GetNOPOnAccumulatedTime(void);
/* ms NOP was on during the last telemetry update window, clamped to window_ms */
uint32_t GetNOPOnDuration(uint32_t window_ms);
/* The runtime board-power policy is always non-destructive. These predicates
 * let AICLK/VDD control honor its reversible clamp and prevent host
 * characterization controls from bypassing the board limit.
 */
bool ThrottlerRuntimePowerClampActive(void);
bool ThrottlerStrictRuntimePowerLimitActive(void);

#if defined(CONFIG_ZTEST)
uint32_t ThrottlerGetDopplerT2PowerLimit(void);
uint32_t ThrottlerGetDopplerT3PowerLimit(void);
uint32_t ThrottlerGetDopplerSlowLimit(void);
void ThrottlerTestResetRuntimePowerState(void);
void ThrottlerTestSetRuntimePowerControllerInitialized(bool initialized);
void ThrottlerTestCompleteRuntimePowerControllerInit(void);
void ThrottlerTestStartRuntimePowerSampleWatchdog(uint32_t now_ms);
void ThrottlerTestRecordInputPowerSampleAtPower(uint32_t now_ms, uint16_t input_power);
void ThrottlerTestApplyPendingBoardPowerLimit(void);
uint16_t ThrottlerTestConsumeRuntimePowerPeak(void);
bool ThrottlerTestRuntimePowerSampleExpired(uint32_t now_ms);
bool ThrottlerTestUpdateRuntimePowerFreshnessGuard(uint32_t now_ms);
uint16_t ThrottlerTestUpdateBoardPowerHistory(uint16_t current_power);
void ThrottlerTestResetBoardPowerHistory(uint16_t current_power);
#endif

#endif
