#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "imu_types.h"

#define MPU6050_ADDR 0x68
#define MPU6050_ADDR_ALT 0x69

#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_WHO_AM_I 0x75

bool MPU6050_Init(int sda_pin, int scl_pin, uint32_t i2c_clock);
const char *MPU6050_GetLastError(void);
uint8_t MPU6050_GetAddress(void);
bool MPU6050_ReadWhoAmI(uint8_t *who_am_i);
bool MPU6050_ReadRaw(ImuRaw *raw);
bool MPU6050_Calibrate(uint16_t samples, uint16_t sample_delay_ms);
void MPU6050_ResetOffsets(void);

#endif
