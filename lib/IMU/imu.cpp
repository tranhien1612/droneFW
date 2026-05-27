#include "imu.h"

//Mahony AHRS

/* Three constants used for Euler angle conversion */
float RtA = 57.2957795f; // Radians -> Degrees (180/π)
float Gyro_G = 4000.0f / 65536.0f; // Initial gyroscope range ±2000 degrees/s 1/(65536 / 4000) = 0.03051756*2
float Gyro_Gr = 4000.0f / 65536.0f / 180.0f * 3.1415926f; // Degrees/s to Radians/s
static float normAccz;

/**
* @brief Quick inverse square root 1/sqrt(num)
* @param number Input value
* @return 1/sqrt(number)
*/
static float Q_rsqrt(float number){
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y)); // 1st iteration: Newton's iteration method
    return y;
}


void IMU_GetEulerAngle(Gyro_Accel_Struct *gyroAccel,
                              Euler_struct *eulerAngle,
                              float dt){
    volatile struct V{
        float x;
        float y;
        float z;
    } Gravity, Acc, Gyro, AccGravity;

    static struct V GyroIntegError = {0};
    static float KpDef = 0.8f;
    static float KiDef = 0.0003f;
    static Quaternion_Struct NumQ = {1, 0, 0, 0};
    float q0_t, q1_t, q2_t, q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // Get the gravity vector in the current attitude solution
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // Acceleration normalization
    NormQuat = Q_rsqrt(squa(gyroAccel->accel.accel_x) +
                       squa(gyroAccel->accel.accel_y) +
                       squa(gyroAccel->accel.accel_z));

    Acc.x = gyroAccel->accel.accel_x * NormQuat;
    Acc.y = gyroAccel->accel.accel_y * NormQuat;
    Acc.z = gyroAccel->accel.accel_z * NormQuat;

    // Cross product yields the error
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    // Integrate to obtain gyroscope deviation
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;

    // Gyroscope fusion accelerometer deviation value
    Gyro.x = gyroAccel->gyro.gyro_x * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;
    Gyro.y = gyroAccel->gyro.gyro_y * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = gyroAccel->gyro.gyro_z * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

    // First-order Longokuta method, updating quaternions
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // Quaternion normalization
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /* Calculate the Z-axis components in the geographic system */
    float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;
    float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;
    float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;

    float yaw_G = gyroAccel->gyro.gyro_z * Gyro_G; //Convert the original Z-axis angular velocity value to Z angle/s
    if ((yaw_G > 0.5f) || (yaw_G < -0.5f))        // If the value is too small, it is considered noise and the yaw angle is not updated.
    {
        eulerAngle->yaw += yaw_G * dt; // Integral angular velocity to yaw angle
    }

    eulerAngle->pitch = asin(vecxZ) * RtA; // Pitch angle
    eulerAngle->roll = atan2f(vecyZ, veczZ) * RtA; // Roll angle

    normAccz = gyroAccel->accel.accel_x * vecxZ + gyroAccel->accel.accel_y * vecyZ + gyroAccel->accel.accel_z * veczZ;
}

float IMU_GetNormAccZ(void){
    return normAccz;
}