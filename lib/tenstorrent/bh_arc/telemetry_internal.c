/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "avs.h"
#include "gddr.h"
#include "telemetry_internal.h"
#include "regulator.h"

#include <math.h>
#include <tenstorrent/bh_arc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/tenstorrent/pvt_tt_bh.h>
#include <zephyr/sys/util.h>

#define TELEMETRY_WORK_STACK_SIZE     2048
#define TELEMETRY_WORK_PRIORITY       K_PRIO_PREEMPT(3)
#define TELEMETRY_FAILURE_BACKOFF_MS  100U
#define TELEMETRY_ASIC_TEMP_MIN_C     -60.0F
#define TELEMETRY_ASIC_TEMP_MAX_C     200.0F
#define TELEMETRY_VCORE_MIN_MV        500U
#define TELEMETRY_VCORE_MAX_MV        1200U
#define TELEMETRY_VCORE_CURRENT_MAX_A 1000.0F
#define TELEMETRY_GDDR_IO_POWER_MAX_W 1000.0F
#define TELEMETRY_GDDR_TEMP_MAX_C     200U
#define TELEMETRY_FAILSAFE_TEMP_C     (GDDR_THERM_TRIP_TEMP - 1U)

/* Slow sensor and PMBus reads never run on the system workqueue. The telemetry
 * queue is preemptible and lower priority than the watchdog feeder, dedicated
 * DVFS queue, and host/system work, so no safety path waits for telemetry.
 */
static struct k_work_q telemetry_work_q;
static K_THREAD_STACK_DEFINE(telemetry_work_stack, TELEMETRY_WORK_STACK_SIZE);
static struct k_work telemetry_refresh_work;

static struct k_spinlock telemetry_lock;
static TelemetryInternalData internal_data;
static uint32_t last_update_time_ms;
static uint32_t last_attempt_time_ms;
static bool refresh_attempted;
static bool refresh_failed;
static bool refresh_pending;
static bool internal_data_valid;

static const struct device *const pvt = DEVICE_DT_GET(DT_NODELABEL(pvt));

SENSOR_DT_READ_IODEV(ts_avg_iodev, DT_NODELABEL(pvt), {SENSOR_CHAN_PVT_TT_BH_TS_AVG, 0});

RTIO_DEFINE(ts_avg_ctx, 1, 1);

static uint8_t ts_avg_buf[sizeof(struct pvt_tt_bh_rtio_data)];

static bool sample_in_range(float value, float min, float max)
{
	return isfinite(value) && value >= min && value <= max;
}

static void telemetry_refresh_handler(struct k_work *work)
{
	TelemetryInternalData new_data;
	bool sample_valid = true;
	bool refresh_ok = true;
	uint32_t now_ms;
	k_spinlock_key_t key;

	ARG_UNUSED(work);

	key = k_spin_lock(&telemetry_lock);
	new_data = internal_data;
	k_spin_unlock(&telemetry_lock, key);

	float avg_tmp;
	const struct sensor_decoder_api *decoder = NULL;
	int ret = -ENODEV;

	if (device_is_ready(pvt) && sensor_get_decoder(pvt, &decoder) == 0 && decoder != NULL &&
	    sensor_read(&ts_avg_iodev, &ts_avg_ctx, ts_avg_buf, sizeof(ts_avg_buf)) == 0) {
		ret = decoder->decode(ts_avg_buf,
				      (struct sensor_chan_spec){SENSOR_CHAN_PVT_TT_BH_TS_AVG, 0},
				      NULL, 1, &avg_tmp);
	}
	if (ret == 0 &&
	    sample_in_range(avg_tmp, TELEMETRY_ASIC_TEMP_MIN_C, TELEMETRY_ASIC_TEMP_MAX_C)) {
		new_data.asic_temperature = avg_tmp;
	} else {
		sample_valid = false;
		refresh_ok = false;
	}

	/* Get all dynamically updated values. */
#ifndef CONFIG_TT_BH_ARC_EMUL
	uint32_t vcore_voltage = get_vcore();
	float vcore_current;
	AVSStatus current_status = AVSReadCurrent(AVS_VCORE_RAIL, &vcore_current);

	if (current_status == AVSOk &&
	    IN_RANGE(vcore_voltage, TELEMETRY_VCORE_MIN_MV, TELEMETRY_VCORE_MAX_MV) &&
	    sample_in_range(vcore_current, 0.0F, TELEMETRY_VCORE_CURRENT_MAX_A)) {
		new_data.vcore_voltage = vcore_voltage;
		new_data.vcore_current = vcore_current;
		new_data.vcore_power = vcore_current * vcore_voltage * 0.001F;
	} else {
		sample_valid = false;
		refresh_ok = false;
	}

	/* These rails are telemetry-only. Preserve their last plausible values if
	 * a PMBus read produces an invalid result.
	 */
	float gddr_io_power = GetGddrWestIoPower();

	if (sample_in_range(gddr_io_power, 0.0F, TELEMETRY_GDDR_IO_POWER_MAX_W)) {
		new_data.gddr_io_power_west = gddr_io_power;
	} else {
		refresh_ok = false;
	}
	gddr_io_power = GetGddrEastIoPower();
	if (sample_in_range(gddr_io_power, 0.0F, TELEMETRY_GDDR_IO_POWER_MAX_W)) {
		new_data.gddr_io_power_east = gddr_io_power;
	} else {
		refresh_ok = false;
	}

	struct gddr_temps gddr_temps;

	if (get_gddr_temps(&gddr_temps) == 0 && gddr_temps.max_temp <= TELEMETRY_GDDR_TEMP_MAX_C) {
		new_data.gddr_temps = gddr_temps;
	} else {
		sample_valid = false;
		refresh_ok = false;
	}
#endif

	now_ms = k_uptime_get_32();
	key = k_spin_lock(&telemetry_lock);
	last_attempt_time_ms = now_ms;
	refresh_attempted = true;
	refresh_failed = !refresh_ok;
	refresh_pending = false;
	if (sample_valid) {
		internal_data = new_data;
		last_update_time_ms = now_ms;
		internal_data_valid = true;
	} else {
		/* Retain the last complete sample in the cache, but make control users
		 * fail safe until a complete refresh succeeds.
		 */
		internal_data_valid = false;
	}
	k_spin_unlock(&telemetry_lock, key);
}

static int telemetry_internal_init(void)
{
	k_work_init(&telemetry_refresh_work, telemetry_refresh_handler);
	k_work_queue_start(&telemetry_work_q, telemetry_work_stack,
			   K_THREAD_STACK_SIZEOF(telemetry_work_stack), TELEMETRY_WORK_PRIORITY,
			   NULL);
	k_thread_name_set(&telemetry_work_q.thread, "bh_telemetry");
	return 0;
}

SYS_INIT(telemetry_internal_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

/**
 * @brief Read telemetry values that are shared by multiple components
 *
 * This function will update the cached TelemetryInternalData values if necessary.
 * Then return a copy of the values through the *data pointer.
 *
 * @param max_staleness Maximum time interval in milliseconds since the last update
 * @param data Pointer to the TelemetryInternalData struct to fill with the values
 */
void ReadTelemetryInternal(int64_t max_staleness, TelemetryInternalData *data)
{
	uint32_t now_ms = k_uptime_get_32();
	uint32_t requested_staleness =
		max_staleness <= 0 ? 0U : MIN((uint64_t)max_staleness, UINT32_MAX);
	bool submit_refresh = false;
	k_spinlock_key_t key = k_spin_lock(&telemetry_lock);

	*data = internal_data;
	if (!internal_data_valid) {
		/* Keep cooling conservative while the clock clamp is active, but stay
		 * below the GDDR thermal-trip threshold: missing telemetry must not be
		 * reported as a real over-temperature event and reset the board.
		 */
		data->asic_temperature = TELEMETRY_FAILSAFE_TEMP_C;
		data->gddr_temps.max_temp = TELEMETRY_FAILSAFE_TEMP_C;
	}
	if (!refresh_pending) {
		if (!refresh_attempted) {
			submit_refresh = true;
		} else if (refresh_failed) {
			submit_refresh =
				now_ms - last_attempt_time_ms >= TELEMETRY_FAILURE_BACKOFF_MS;
		} else {
			submit_refresh = now_ms - last_update_time_ms >= requested_staleness;
		}
		refresh_pending = submit_refresh;
	}
	k_spin_unlock(&telemetry_lock, key);

	if (submit_refresh &&
	    k_work_submit_to_queue(&telemetry_work_q, &telemetry_refresh_work) < 0) {
		key = k_spin_lock(&telemetry_lock);
		refresh_pending = false;
		refresh_failed = true;
		refresh_attempted = true;
		last_attempt_time_ms = now_ms;
		internal_data_valid = false;
		k_spin_unlock(&telemetry_lock, key);
	}
}

bool ReadTelemetryInternalCached(uint32_t max_staleness, TelemetryInternalData *data)
{
	uint32_t now_ms = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&telemetry_lock);

	*data = internal_data;
	bool valid = internal_data_valid && now_ms - last_update_time_ms <= max_staleness;

	k_spin_unlock(&telemetry_lock, key);
	return valid;
}

#if defined(CONFIG_ZTEST)
void TelemetryInternalTestSetCached(const TelemetryInternalData *data)
{
	k_spinlock_key_t key = k_spin_lock(&telemetry_lock);

	internal_data = *data;
	last_update_time_ms = k_uptime_get_32();
	last_attempt_time_ms = last_update_time_ms;
	refresh_attempted = true;
	refresh_failed = false;
	refresh_pending = false;
	internal_data_valid = true;
	k_spin_unlock(&telemetry_lock, key);
}
#endif
