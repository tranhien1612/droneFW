#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "imu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q_angle;
    float q_bias;
    float r_measure;
    float angle;
    float bias;
    float rate;
    float p00;
    float p01;
    float p10;
    float p11;
} ImuKalman1D;

typedef struct {
    ImuKalman1D roll;
    ImuKalman1D pitch;

    float gyro_roll_filt;
    float gyro_pitch_filt;
    float gyro_yaw_filt;
    float yaw_deg;

    float trim_roll_deg;
    float trim_pitch_deg;
    float yaw_deadband_dps;
    bool invert_yaw;
    bool initialized;
} ImuFilterState;

void IMU_FilterInit(ImuFilterState *state, const ImuConfig *config);
void IMU_FilterReset(ImuFilterState *state);
void IMU_FilterSetConfig(ImuFilterState *state, float trim_roll_deg, float trim_pitch_deg, bool invert_yaw, float yaw_deadband_dps);
void IMU_FilterUpdate(ImuFilterState *state, const ImuRaw *raw, float dt_s, ImuData *out);

#ifdef __cplusplus
}
#endif

#endif
