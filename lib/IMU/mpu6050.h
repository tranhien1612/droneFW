#ifndef __MPU6050__
#define __MPU6050__

#include "imu.h"
#include "../Filter/filter.h"

// Device address
#define MPU6050_ADDR            0x68
// Read/Write Address
#define MPU6050_ADDR_WRITE      0xD0
#define MPU6050_ADDR_READ       0xD1

#define MPU_SELF_TESTX_REG      0X0D // Self-test register X
#define MPU_SELF_TESTY_REG      0X0E // Self-test register Y
#define MPU_SELF_TESTZ_REG      0X0F // Self-test register Z
#define MPU_SELF_TESTA_REG      0X10 // Self-test register A
#define MPU_SAMPLE_RATE_REG     0X19 // Sampling frequency divider

#define MPU_CFG_REG             0X1A // Configuration register
#define MPU_GYRO_CFG_REG        0X1B // Gyroscope configuration register
#define MPU_ACCEL_CFG_REG       0X1C // Accelerometer configuration register
#define MPU_MOTION_DET_REG      0X1F // Motion detection threshold register
#define MPU_FIFO_EN_REG         0X23 // FIFO enable register
#define MPU_I2CMST_CTRL_REG     0X24 // I2C master controller register

#define MPU_I2CSLV0_ADDR_REG    0X25 // IIC Slave 0 Device Address Register
#define MPU_I2CSLV0_REG         0X26 // IIC Slave 0 Data Address Register
#define MPU_I2CSLV0_CTRL_REG    0X27 // IIC Slave 0 Control Register
#define MPU_I2CSLV1_ADDR_REG    0X28 // IIC Slave 1 Device Address Register
#define MPU_I2CSLV1_REG         0X29 // IIC Slave 1 Data Address Register
#define MPU_I2CSLV1_CTRL_REG    0X2A // IIC Slave 1 Control Register
#define MPU_I2CSLV2_ADDR_REG    0X2B // IIC Slave 2 Device Address Register
#define MPU_I2CSLV2_REG         0X2C // IIC Slave 2 Data Address Register
#define MPU_I2CSLV2_CTRL_REG    0X2D // IIC Slave 2 Control Register
#define MPU_I2CSLV3_ADDR_REG    0X2E // IIC Slave 3 Device Address Register
#define MPU_I2CSLV3_REG         0X2F // IIC Slave 3 Data Address Register
#define MPU_I2CSLV3_CTRL_REG    0X30 // IIC Slave 3 Control Register
#define MPU_I2CSLV4_ADDR_REG    0X31 // IIC Slave 4 Device Address Register
#define MPU_I2CSLV4_REG         0X32 // IIC Slave 4 Data Address Register
#define MPU_I2CSLV4_DO_REG      0X33 // IIC Slave 4 Write Data Register
#define MPU_I2CSLV4_CTRL_REG    0X34 // IIC Slave 4 Control Register
#define MPU_I2CSLV4_DI_REG      0X35 // IIC Slave 4 Read Data Register

#define MPU_I2CMST_STA_REG      0X36 // IIC Master Status Register
#define MPU_INTBP_CFG_REG       0X37 // Interrupt/Bypass Configuration Register
#define MPU_INT_EN_REG          0X38 // Interrupt Enable Register
#define MPU_INT_STA_REG         0X3A // Interrupt Status Register

#define MPU_ACCEL_XOUTH_REG     0X3B // Acceleration value, high 8 bits of the X-axis register
#define MPU_ACCEL_XOUTL_REG     0X3C // Acceleration value, low 8 bits of the X-axis register
#define MPU_ACCEL_YOUTH_REG     0X3D // Acceleration value, high 8 bits of the Y-axis register
#define MPU_ACCEL_YOUTL_REG     0X3E // Acceleration value, low 8 bits of the Y-axis register
#define MPU_ACCEL_ZOUTH_REG     0X3F // Acceleration value, high 8 bits of the Z-axis register
#define MPU_ACCEL_ZOUTL_REG     0X40 // Acceleration value, low 8 bits of the Z-axis register

#define MPU_TEMP_OUTH_REG       0X41 // High temperature value 8-bit register
#define MPU_TEMP_OUTL_REG       0X42 // Low temperature value 8-bit register

#define MPU_GYRO_XOUTH_REG      0X43 // Gyroscope value, high 8 bits of the X-axis register
#define MPU_GYRO_XOUTL_REG      0X44 // Gyroscope value, low 8 bits of the X-axis register
#define MPU_GYRO_YOUTH_REG      0X45 // Gyroscope value, high 8 bits of the Y-axis register
#define MPU_GYRO_YOUTL_REG      0X46 // Gyroscope value, low 8 bits of the Y-axis register
#define MPU_GYRO_ZOUTH_REG      0X47 // Gyroscope value, high 8 bits of the Z-axis register
#define MPU_GYRO_ZOUTL_REG      0X48 // Gyroscope value, low 8 bits of the Z-axis register

#define MPU_I2CSLV0_DO_REG      0X63 // IIC Slave 0 Data Register
#define MPU_I2CSLV1_DO_REG      0X64 // IIC Slave 1 Data Register
#define MPU_I2CSLV2_DO_REG      0X65 // IIC Slave 2 Data Register
#define MPU_I2CSLV3_DO_REG      0X66 // IIC Slave 3 Data Register

#define MPU_I2CMST_DELAY_REG    0X67 // IIC Master Delay Control Register
#define MPU_SIGPATH_RST_REG     0X68 // Signal Channel Reset Register
#define MPU_MDETECT_CTRL_REG    0X69 // Motion Detection Control Register
#define MPU_USER_CTRL_REG       0X6A // User Control Register
#define MPU_PWR_MGMT1_REG       0X6B // Power Management Register 1
#define MPU_PWR_MGMT2_REG       0X6C // Power Management Register 2
#define MPU_FIFO_CNTH_REG       0X72 // FIFO Count Register High 8 Bits
#define MPU_FIFO_CNTL_REG       0X73 // FIFO Count Register Low 8 Bits
#define MPU_FIFO_RW_REG         0X74 // FIFO read/write registers
#define MPU_DEVICE_ID_REG       0X75 // Device ID register

/**
 * @brief Initialize the MPU6050 chip
 *
 */
void MPU6050_Init(int sdaPin, int sclPin, uint32_t frequency);
// void MPU6050_Init(void);

/**
 * @brief Reads MPU6050 registers (for debugging)
 * @param reg Register address
 * @param data Data to be read
 */
void MPU6050_Read_Reg(uint8_t reg, uint8_t *data);

/**
 * @brief Acquiring gyroscope data
 *
 * @param gyro
 */
void MPU6050_Get_Gyro(Gyro_struct *gyro);

/**
 * @brief Acquiring accelerometer data
 *
 * @param acc
 */
void MPU6050_Get_Acc(Accel_struct *acc);

/**
 * @brief Get all data
 *
 * @param imu
 */
void MPU6050_Get_Data(Gyro_Accel_Struct *imu);

void imu_start(void);

#endif