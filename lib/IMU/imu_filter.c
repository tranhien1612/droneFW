#include "imu_filter.h"

#include <math.h>
#include <string.h>

#define IMU_RAD_TO_DEG 57.2957795f
#define IMU_GYRO_SCALE_500DPS 65.5f
#define IMU_ACCEL_SCALE_8G 4096.0f
#define IMU_GYRO_PT1_COEFF 0.85f

static void kalman_init(ImuKalman1D *kf)
{
    memset(kf, 0, sizeof(*kf));
    kf->q_angle = 0.001f;
    kf->q_bias = 0.003f;
    kf->r_measure = 0.03f;
}

static void kalman_set_angle(ImuKalman1D *kf, float angle)
{
    kf->angle = angle;
    kf->bias = 0.0f;
    kf->rate = 0.0f;
    kf->p00 = 0.0f;
    kf->p01 = 0.0f;
    kf->p10 = 0.0f;
    kf->p11 = 0.0f;
}

static float kalman_update(ImuKalman1D *kf, float measured_angle, float measured_rate, float dt_s)
{
    kf->rate = measured_rate - kf->bias;
    kf->angle += dt_s * kf->rate;

    kf->p00 += dt_s * (dt_s * kf->p11 - kf->p01 - kf->p10 + kf->q_angle);
    kf->p01 -= dt_s * kf->p11;
    kf->p10 -= dt_s * kf->p11;
    kf->p11 += kf->q_bias * dt_s;

    const float s = kf->p00 + kf->r_measure;
    const float k0 = kf->p00 / s;
    const float k1 = kf->p10 / s;
    const float y = measured_angle - kf->angle;

    kf->angle += k0 * y;
    kf->bias += k1 * y;

    const float p00_temp = kf->p00;
    const float p01_temp = kf->p01;

    kf->p00 -= k0 * p00_temp;
    kf->p01 -= k0 * p01_temp;
    kf->p10 -= k1 * p00_temp;
    kf->p11 -= k1 * p01_temp;

    return kf->angle;
}

void IMU_FilterInit(ImuFilterState *state, const ImuConfig *config)
{
    memset(state, 0, sizeof(*state));
    kalman_init(&state->roll);
    kalman_init(&state->pitch);

    if (config != 0) {
        state->trim_roll_deg = config->trim_roll_deg;
        state->trim_pitch_deg = config->trim_pitch_deg;
        state->yaw_deadband_dps = config->yaw_deadband_dps;
        state->invert_yaw = config->invert_yaw;
    }
}

void IMU_FilterReset(ImuFilterState *state)
{
    const float trim_roll = state->trim_roll_deg;
    const float trim_pitch = state->trim_pitch_deg;
    const float yaw_deadband = state->yaw_deadband_dps;
    const bool invert_yaw = state->invert_yaw;

    IMU_FilterInit(state, 0);
    state->trim_roll_deg = trim_roll;
    state->trim_pitch_deg = trim_pitch;
    state->yaw_deadband_dps = yaw_deadband;
    state->invert_yaw = invert_yaw;
}

void IMU_FilterSetConfig(ImuFilterState *state, float trim_roll_deg, float trim_pitch_deg, bool invert_yaw, float yaw_deadband_dps)
{
    state->trim_roll_deg = trim_roll_deg;
    state->trim_pitch_deg = trim_pitch_deg;
    state->invert_yaw = invert_yaw;
    state->yaw_deadband_dps = yaw_deadband_dps < 0.0f ? 0.0f : yaw_deadband_dps;
}

void IMU_FilterUpdate(ImuFilterState *state, const ImuRaw *raw, float dt_s, ImuData *out)
{
    if (dt_s < 0.002f) {
        dt_s = 0.002f;
    } else if (dt_s > 0.010f) {
        dt_s = 0.010f;
    }

    const float gyro_roll_dps = -(float)raw->gyro_y / IMU_GYRO_SCALE_500DPS;
    const float gyro_pitch_dps = (float)raw->gyro_x / IMU_GYRO_SCALE_500DPS;
    const float gyro_yaw_dps = (state->invert_yaw ? -(float)raw->gyro_z : (float)raw->gyro_z) / IMU_GYRO_SCALE_500DPS;

    state->gyro_roll_filt += IMU_GYRO_PT1_COEFF * (gyro_roll_dps - state->gyro_roll_filt);
    state->gyro_pitch_filt += IMU_GYRO_PT1_COEFF * (gyro_pitch_dps - state->gyro_pitch_filt);
    state->gyro_yaw_filt += IMU_GYRO_PT1_COEFF * (gyro_yaw_dps - state->gyro_yaw_filt);

    if (fabsf(state->gyro_yaw_filt) < state->yaw_deadband_dps) {
        state->gyro_yaw_filt = 0.0f;
    }

    const float acc_roll = (float)raw->acc_x;
    const float acc_pitch = (float)raw->acc_y;
    const float acc_z = (float)raw->acc_z;
    const float acc_total = sqrtf(acc_roll * acc_roll + acc_pitch * acc_pitch + acc_z * acc_z);

    float angle_roll_acc = 0.0f;
    float angle_pitch_acc = 0.0f;

    if (acc_total > 1.0f) {
        const float roll_ratio = acc_roll / acc_total;
        const float pitch_ratio = acc_pitch / acc_total;

        if (roll_ratio > -1.0f && roll_ratio < 1.0f) {
            angle_roll_acc = asinf(roll_ratio) * IMU_RAD_TO_DEG;
        }
        if (pitch_ratio > -1.0f && pitch_ratio < 1.0f) {
            angle_pitch_acc = asinf(pitch_ratio) * IMU_RAD_TO_DEG;
        }
    }

    angle_roll_acc += state->trim_roll_deg;
    angle_pitch_acc += state->trim_pitch_deg;

    if (!state->initialized) {
        kalman_set_angle(&state->roll, angle_roll_acc);
        kalman_set_angle(&state->pitch, angle_pitch_acc);
        state->initialized = true;
    }

    state->yaw_deg += state->gyro_yaw_filt * dt_s;
    while (state->yaw_deg > 180.0f) {
        state->yaw_deg -= 360.0f;
    }
    while (state->yaw_deg < -180.0f) {
        state->yaw_deg += 360.0f;
    }

    out->roll_deg = kalman_update(&state->roll, angle_roll_acc, state->gyro_roll_filt, dt_s);
    out->pitch_deg = kalman_update(&state->pitch, angle_pitch_acc, state->gyro_pitch_filt, dt_s);
    out->yaw_deg = state->yaw_deg;

    out->gyro_roll_dps = state->gyro_roll_filt;
    out->gyro_pitch_dps = state->gyro_pitch_filt;
    out->gyro_yaw_dps = state->gyro_yaw_filt;

    out->acc_x_g = (float)raw->acc_x / IMU_ACCEL_SCALE_8G;
    out->acc_y_g = (float)raw->acc_y / IMU_ACCEL_SCALE_8G;
    out->acc_z_g = acc_z / IMU_ACCEL_SCALE_8G;
}
