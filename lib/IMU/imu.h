#ifndef ___IMU_H
#define ___IMU_H

#include <stdint.h>
#include <math.h>

typedef struct{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} Gyro_struct;

typedef struct{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} Accel_struct;

typedef struct{
    Gyro_struct gyro;
    Accel_struct accel;
} Gyro_Accel_Struct;

typedef struct{
    float yaw;
    float pitch;
    float roll;
} Euler_struct;

typedef struct{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;

/* Calculate the square */
#define squa(Sq) (((float)Sq) * ((float)Sq))

/**
* @brief Reads 6-axis data from the MPU6050 and calculates the current Euler angles.
* @param gyroAccel 6-axis data from the MPU6050.
* @param eulerAngle Calculated Euler angles.
* @param dt Sampling period (in seconds).
*/
void IMU_GetEulerAngle(Gyro_Accel_Struct *gyroAccel,
                              Euler_struct *eulerAngle,
                              float dt);

/**
* @brief Get the acceleration along the Z-axis (already normalized, will consider the composition of Z-axis acceleration)
* @return Z-axis acceleration
*/
float IMU_GetNormAccZ(void);

#endif