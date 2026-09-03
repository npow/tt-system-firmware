/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CM2DM_MSG_H
#define CM2DM_MSG_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/toolchain.h>
#include <tenstorrent/bh_arc.h>

void PostCm2DmMsg(Cm2DmMsgId msg_id, uint32_t data);
int32_t Cm2DmMsgReqSmbusHandler(uint8_t *data, uint8_t *size);
int32_t Cm2DmMsgAckSmbusHandler(const uint8_t *data, uint8_t size);

void ChipResetRequest(void *arg);
void UpdateFanSpeedRequest(uint32_t fan_speed);
void UpdateForcedFanSpeedRequest(uint32_t fan_speed);
void Dm2CmReadyRequest(void);
void UpdateAutoResetTimeoutRequest(uint32_t timeout);
void UpdateTelemHeartbeatRequest(uint32_t heartbeat);
void RequestLedBlink(uint32_t blink_mode);
void ReportGddrThermTrip(GddrThermTripReason reason);

int32_t Dm2CmSendDataHandler(const uint8_t *data, uint8_t size);
int32_t Dm2CmPingHandler(const uint8_t *data, uint8_t size);
int32_t Dm2CmSendCurrentHandler(const uint8_t *data, uint8_t size);
int32_t Dm2CmSendPowerHandler(const uint8_t *data, uint8_t size);
int32_t GetInputCurrent(void);

typedef struct {
	uint16_t power;
	uint32_t updated_ms;
	bool valid;
} InputPowerSample;

/* Copy power, timestamp, and validity as one interrupt-safe observation. The
 * return value reports whether that copied sample is no older than max_age_ms.
 */
bool GetInputPowerSample(uint32_t max_age_ms, InputPowerSample *sample);
uint16_t GetInputPower(void);
bool InputPowerSampleIsFresh(uint32_t max_age_ms);
#if defined(CONFIG_ZTEST)
void InputPowerTestSetPostSnapshotHook(void (*hook)(void));
#endif
int32_t Dm2CmSendFanRPMHandler(const uint8_t *data, uint8_t size);
int32_t SMBusTelemRegHandler(const uint8_t *data, uint8_t size);
int32_t SMBusTelemDataHandler(uint8_t *data, uint8_t *size);
int32_t Dm2CmSendThermTripCountHandler(const uint8_t *data, uint8_t size);
int32_t Dm2CmWriteTelemetry(const uint8_t *data, uint8_t size);
int32_t Dm2CmReadControlData(uint8_t *data, uint8_t *size);
int32_t Dm2CmDMCLogHandler(const uint8_t *data, uint8_t size);
int32_t Dm2CmPingV2(uint8_t *data, uint8_t *size);

#endif
