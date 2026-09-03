/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TELEMETRY_INTERNAL_H
#define TELEMETRY_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "gddr.h"

typedef struct {
	float vcore_voltage;          /* mV */
	float vcore_power;            /* W */
	float vcore_current;          /* A */
	uint32_t vcore_power_updated_ms;
	bool vcore_power_valid;
	float asic_temperature;       /* degC */
	struct gddr_temps gddr_temps; /* per-instance GDDR die temps + max across all dies, degC */
	float gddr_io_power_west;     /* W */
	float gddr_io_power_east;     /* W */
} TelemetryInternalData;

void ReadTelemetryInternal(int64_t max_staleness, TelemetryInternalData *data);
#if defined(CONFIG_ZTEST)
void TelemetryInternalTestSetVcorePower(float power, bool valid);
#endif

#endif
