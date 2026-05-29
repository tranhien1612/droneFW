#include "imu.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu_filter.h"
#include "mpu6050.h"

#define IMU_TASK_PERIOD_MS 4
#define IMU_TASK_STACK_WORDS 4096
#define IMU_TASK_PRIORITY 4
#define IMU_TASK_CORE 1

static ImuConfig imu_config;
static ImuFilterState imu_filter;
static ImuData imu_snapshot;
static ImuStats imu_stats;
static TaskHandle_t imu_task_handle = nullptr;
static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool imu_reset_requested = false;
static bool imu_started = false;
static bool imu_filter_config_pending = false;
static float pending_trim_roll_deg = 0.0f;
static float pending_trim_pitch_deg = 0.0f;
static float pending_yaw_deadband_dps = 0.0f;
static bool pending_invert_yaw = false;

static void apply_pending_filter_config()
{
    float trim_roll;
    float trim_pitch;
    float yaw_deadband;
    bool invert_yaw;
    bool pending;

    portENTER_CRITICAL(&imu_mux);
    pending = imu_filter_config_pending;
    trim_roll = pending_trim_roll_deg;
    trim_pitch = pending_trim_pitch_deg;
    yaw_deadband = pending_yaw_deadband_dps;
    invert_yaw = pending_invert_yaw;
    imu_filter_config_pending = false;
    portEXIT_CRITICAL(&imu_mux);

    if (pending) {
        IMU_FilterSetConfig(&imu_filter, trim_roll, trim_pitch, invert_yaw, yaw_deadband);
    }
}

static bool update_once_with_dt(float dt_s)
{
    ImuRaw raw = {0};
    ImuData next = {0};

    apply_pending_filter_config();

    const uint32_t start_us = micros();
    const bool ok = MPU6050_ReadRaw(&raw);

    if (ok) {
        IMU_FilterUpdate(&imu_filter, &raw, dt_s, &next);
        next.healthy = true;
    } else {
        portENTER_CRITICAL(&imu_mux);
        next = imu_snapshot;
        portEXIT_CRITICAL(&imu_mux);
        next.healthy = false;
    }

    next.update_us = micros() - start_us;

    portENTER_CRITICAL(&imu_mux);
    imu_snapshot = next;
    imu_stats.read_count++;
    imu_stats.last_update_ms = millis();
    imu_stats.read_time_us = next.update_us;
    if (next.update_us > imu_stats.max_read_time_us) {
        imu_stats.max_read_time_us = next.update_us;
    }
    if (ok) {
        imu_stats.consecutive_fail_count = 0;
        imu_stats.last_ok_ms = millis();
    } else {
        imu_stats.fail_count++;
        imu_stats.consecutive_fail_count++;
    }
    imu_stats.task_running = imu_task_handle != nullptr;
    imu_stats.healthy = ok;
    portEXIT_CRITICAL(&imu_mux);

    return ok;
}

static void imu_task(void *parameter)
{
    (void)parameter;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_us = micros();

    for (;;) {
        if (imu_reset_requested) {
            IMU_FilterReset(&imu_filter);
            imu_reset_requested = false;
        }

        const uint32_t now_us = micros();
        float dt_s = (now_us - last_us) * 1e-6f;
        last_us = now_us;

        update_once_with_dt(dt_s);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

bool IMU_Begin(const ImuConfig *config)
{
    if (config == nullptr) {
        return false;
    }

    imu_config = *config;

    portENTER_CRITICAL(&imu_mux);
    imu_snapshot = {};
    imu_stats = {};
    pending_trim_roll_deg = imu_config.trim_roll_deg;
    pending_trim_pitch_deg = imu_config.trim_pitch_deg;
    pending_yaw_deadband_dps = imu_config.yaw_deadband_dps;
    pending_invert_yaw = imu_config.invert_yaw;
    imu_filter_config_pending = false;
    portEXIT_CRITICAL(&imu_mux);

    IMU_FilterInit(&imu_filter, &imu_config);

    if (!MPU6050_Init(imu_config.sda_pin, imu_config.scl_pin, imu_config.i2c_clock)) {
        return false;
    }

    imu_started = true;
    return true;
}

bool IMU_StartTask(void)
{
    if (!imu_started) {
        return false;
    }

    if (imu_task_handle != nullptr) {
        return true;
    }

    const bool ok = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        IMU_TASK_STACK_WORDS,
        nullptr,
        IMU_TASK_PRIORITY,
        &imu_task_handle,
        IMU_TASK_CORE
    ) == pdPASS;

    if (ok) {
        portENTER_CRITICAL(&imu_mux);
        imu_stats.task_running = true;
        portEXIT_CRITICAL(&imu_mux);
    }

    return ok;
}

bool IMU_UpdateOnce(void)
{
    static uint32_t last_us = 0;
    const uint32_t now_us = micros();
    float dt_s = 0.004f;

    if (last_us != 0) {
        dt_s = (now_us - last_us) * 1e-6f;
    }
    last_us = now_us;

    if (imu_reset_requested) {
        IMU_FilterReset(&imu_filter);
        imu_reset_requested = false;
    }

    return update_once_with_dt(dt_s);
}

bool IMU_GetData(ImuData *out)
{
    if (out == nullptr) {
        return false;
    }

    portENTER_CRITICAL(&imu_mux);
    *out = imu_snapshot;
    portEXIT_CRITICAL(&imu_mux);

    return out->healthy;
}

void IMU_GetStats(ImuStats *out)
{
    if (out == nullptr) {
        return;
    }

    portENTER_CRITICAL(&imu_mux);
    *out = imu_stats;
    portEXIT_CRITICAL(&imu_mux);
}

void IMU_RequestReset(void)
{
    imu_reset_requested = true;
}

bool IMU_IsHealthy()
{
    ImuStats stats = {};
    ImuData data = {};

    portENTER_CRITICAL(&imu_mux);
    stats = imu_stats;
    data = imu_snapshot;
    portEXIT_CRITICAL(&imu_mux);

    if (!imu_started || !data.healthy) {
        return false;
    }

    return (millis() - stats.last_ok_ms) < 100;
}

void IMU_SetTrim(float roll_deg, float pitch_deg)
{
    portENTER_CRITICAL(&imu_mux);
    imu_config.trim_roll_deg = roll_deg;
    imu_config.trim_pitch_deg = pitch_deg;
    pending_trim_roll_deg = roll_deg;
    pending_trim_pitch_deg = pitch_deg;
    pending_invert_yaw = imu_config.invert_yaw;
    pending_yaw_deadband_dps = imu_config.yaw_deadband_dps;
    imu_filter_config_pending = true;
    portEXIT_CRITICAL(&imu_mux);
}

void IMU_SetYawInvert(bool invert)
{
    portENTER_CRITICAL(&imu_mux);
    imu_config.invert_yaw = invert;
    pending_trim_roll_deg = imu_config.trim_roll_deg;
    pending_trim_pitch_deg = imu_config.trim_pitch_deg;
    pending_invert_yaw = invert;
    pending_yaw_deadband_dps = imu_config.yaw_deadband_dps;
    imu_filter_config_pending = true;
    portEXIT_CRITICAL(&imu_mux);
}

void IMU_SetYawDeadband(float deadband_dps)
{
    if (deadband_dps < 0.0f) {
        deadband_dps = 0.0f;
    }

    portENTER_CRITICAL(&imu_mux);
    imu_config.yaw_deadband_dps = deadband_dps;
    pending_trim_roll_deg = imu_config.trim_roll_deg;
    pending_trim_pitch_deg = imu_config.trim_pitch_deg;
    pending_invert_yaw = imu_config.invert_yaw;
    pending_yaw_deadband_dps = deadband_dps;
    imu_filter_config_pending = true;
    portEXIT_CRITICAL(&imu_mux);
}

bool IMU_Recalibrate()
{
    if (!imu_started) {
        return false;
    }

    if (imu_task_handle != nullptr) {
        return false;
    }

    if (!MPU6050_Calibrate(800, 2)) {
        return false;
    }

    IMU_RequestReset();

    portENTER_CRITICAL(&imu_mux);
    imu_stats.fail_count = 0;
    imu_stats.consecutive_fail_count = 0;
    imu_stats.max_read_time_us = 0;
    portEXIT_CRITICAL(&imu_mux);

    return true;
}

const char *IMU_GetLastError(void) {
    return MPU6050_GetLastError();
}

uint8_t IMU_GetDeviceAddress(void) {
    return MPU6050_GetAddress();
}
