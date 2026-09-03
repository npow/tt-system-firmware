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
/* Called only after CMFW has accepted a complete DMC input-power sample. */
void ThrottlerRecordInputPowerSample(uint32_t now_ms);
uint8_t ThrottlerSetKernelThrottlerEnabled(uint32_t enabled);
uint8_t ThrottlerSetKernelThrottlerStopFreq(uint32_t frequency);
uint32_t GetStartNOPCount(void);
uint32_t GetNOPOnAccumulatedTime(void);
/* ms NOP was on during the last telemetry update window, clamped to window_ms */
uint32_t GetNOPOnDuration(uint32_t window_ms);
bool ThrottlerRuntimePowerFaultLatched(void);
bool ThrottlerStrictRuntimePowerLimitActive(void);
/* True only after throttler initialization and a later complete DMC sample. */
bool ThrottlerRuntimePowerMonitorReady(void);
/* IRQ-safe request used by PCIe error handling; containment runs on the next
 * DVFS pass so ARC/PCIe management is never reset from the interrupt path.
 */
void ThrottlerRequestRuntimeContainment(void);

#if defined(CONFIG_ZTEST)
uint32_t ThrottlerGetDopplerT2PowerLimit(void);
uint32_t ThrottlerGetDopplerT3PowerLimit(void);
uint32_t ThrottlerGetRuntimePowerFailSafeLimit(void);
uint32_t ThrottlerGetDopplerSlowAiclkLimit(void);
bool ThrottlerTestRuntimePowerFailSafeEligible(uint16_t current_power, uint32_t aiclk_targ);
bool ThrottlerTestUpdateRuntimePowerGuard(bool eligible, uint16_t current_power, uint32_t now_ms);
void ThrottlerTestResetRuntimePowerGuard(void);
void ThrottlerTestStartRuntimePowerSampleWatchdog(uint32_t now_ms);
void ThrottlerTestRecordInputPowerSample(uint32_t now_ms);
void ThrottlerTestSetRuntimePowerMonitorInitialized(bool initialized);
bool ThrottlerTestRuntimePowerSampleExpired(uint32_t now_ms);
bool ThrottlerTestUpdateRuntimePowerFreshnessGuard(uint32_t now_ms);
uint16_t ThrottlerTestUpdateBoardPowerHistory(uint16_t current_power);
void ThrottlerTestResetBoardPowerHistory(uint16_t current_power);
#endif

#endif
