#ifndef IMU_H
#define IMU_H

#include "imu_types.h"

bool IMU_Begin(const ImuConfig *config);
bool IMU_StartTask(void);
bool IMU_UpdateOnce(void);
bool IMU_GetData(ImuData *out);
void IMU_GetStats(ImuStats *out);
void IMU_RequestReset(void);

bool IMU_IsHealthy();
void IMU_SetTrim(float roll_deg, float pitch_deg);
void IMU_SetYawInvert(bool invert);
void IMU_SetYawDeadband(float deadband_dps);
bool IMU_Recalibrate();
const char *IMU_GetLastError(void);
uint8_t IMU_GetDeviceAddress(void);
#endif
