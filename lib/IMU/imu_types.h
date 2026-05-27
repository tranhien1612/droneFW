#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

typedef struct {
    int sda_pin;
    int scl_pin;
    uint32_t i2c_clock;
    float trim_roll_deg;
    float trim_pitch_deg;
    float yaw_deadband_dps;
    bool invert_yaw;
} ImuConfig;

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} ImuRaw;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float gyro_roll_dps;
    float gyro_pitch_dps;
    float gyro_yaw_dps;

    float acc_x_g;
    float acc_y_g;
    float acc_z_g;

    uint32_t update_us;
    bool healthy;
} ImuData;

typedef struct {
    uint32_t read_count;
    uint32_t fail_count;
    uint32_t consecutive_fail_count;
    uint32_t last_ok_ms;
    uint32_t last_update_ms;
    uint32_t read_time_us;
    uint32_t max_read_time_us;
    bool task_running;
    bool healthy;
} ImuStats;

#endif
