#include "mpu6050.h"

// Accelerometer bias value
int32_t acc_x_offset = 0;
int32_t acc_y_offset = 0;
int32_t acc_z_offset = 0;
int32_t gyro_x_offset = 0;
int32_t gyro_y_offset = 0;
int32_t gyro_z_offset = 0;

/*
static void MPU6050_Write_Reg(uint8_t reg, uint8_t data) {
//   HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT,
//                     &data, 1, 1000);
}

void MPU6050_Read_Reg(uint8_t reg, uint8_t *data) {
//   HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, data,
//                    1, 1000); 
}

static void MPU6050_calculate_offset(void) {
    // 1. Wait for the aircraft to come to a stable stop
    // Criteria for determining whether the aircraft has come to a stable stop: The difference between two consecutive accelerometer readings is less than 200, for 100 consecutive times.
    Accel_struct current_accel = {0};
    Accel_struct last_accel = {0};
    uint8_t count = 0;
    Int_MPU6050_Get_Acc(&last_accel);

    while (count < 100) {
        Int_MPU6050_Get_Acc(&current_accel);
        // To determine if an aircraft is stable, the selected parameters should be low; otherwise, it will be impossible to determine whether it is stable.
        if (abs(current_accel.accel_x - last_accel.accel_x) < 400 &&
            abs(current_accel.accel_y - last_accel.accel_y) < 400 &&
            abs(current_accel.accel_z - last_accel.accel_z) < 400) {
            count++;
        } else {
            count = 0;
        }
        last_accel = current_accel;
        // vTaskDelay(6);
    }

    // 2. The aircraft has stabilized and is beginning offset calibration.
    Gyro_Accel_Struct gyro_accel_data = {0};
    int32_t acc_x_sum = 0;
    int32_t acc_y_sum = 0;
    int32_t acc_z_sum = 0;

    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;
    for (uint8_t i = 0; i < 100; i++) {
        // Reread acceleration and angular velocity
        Int_MPU6050_Get_Data(&gyro_accel_data);
        acc_x_sum += (gyro_accel_data.accel.accel_x - 0);
        acc_y_sum += (gyro_accel_data.accel.accel_y - 0);
        // The initial value of the Z-axis acceleration should be 1g => approximately 16384.
        acc_z_sum += (gyro_accel_data.accel.accel_z - 16384);

        gyro_x_sum += (gyro_accel_data.gyro.gyro_x - 0);
        gyro_y_sum += (gyro_accel_data.gyro.gyro_y - 0);
        gyro_z_sum += (gyro_accel_data.gyro.gyro_z - 0);

        // Each data collection requires a time delay; the average of the first and last data collections is used as the bias.
        // vTaskDelay(6);
    }

    acc_x_offset = acc_x_sum / 100;
    acc_y_offset = acc_y_sum / 100;
    acc_z_offset = acc_z_sum / 100;

    gyro_x_offset = gyro_x_sum / 100;
    gyro_y_offset = gyro_y_sum / 100;
    gyro_z_offset = gyro_z_sum / 100;
}

void Int_MPU6050_Init(void) {
    // 1. Reset the chip Clear the current register value => Write to power management register 1 => DEVICE_RESET
    MPU6050_Write_Reg(0x6B, 0x80);
    uint8_t data = 0;
    // After waiting for a reset, the value of register 0x6B is 0x40, indicating that the current mode is low power.
    while (data != 0x40) {
        MPU6050_Read_Reg(0x6B, &data);
    }
    // Wake up the MPU6050 and bring it into normal working state.
    MPU6050_Write_Reg(0x6B, 0x00);

    // 2. Choose an appropriate measurement range. Within the sufficient range, choose the smaller the better => higher accuracy.
    // 2.1 The gyroscope's range is ±2000 degrees/s.
    MPU6050_Write_Reg(0x1B, 3 << 3);

    // 2.2 Write the accelerometer range as ±2g
    MPU6050_Write_Reg(0x1C, 0x00);

    // 3. Interrupts are disabled because interrupts are not configured.
    MPU6050_Write_Reg(0x38, 0x00);

    // 4. User control register; disable FIFO module; disable extended I2C.
    MPU6050_Write_Reg(0x6A, 0x00);

    // 5. Set sampling frequency => Acquires accelerometer and gyroscope data => Default frequency 1000Hz => 1ms
    // Read once. The sampling frequency should be higher than the subsequent data usage frequency; otherwise, data will be lost. => Nyquist
    // Sampling frequency >= 2 times the usage frequency. Set the sampling divider to 2 => That is, the written value is 2-1
    MPU6050_Write_Reg(0x19, 0x01);

    // 6. Set the low-pass filter cutoff value to 184Hz 188Hz => 1
    MPU6050_Write_Reg(0x1A, 1);

    // 7. Select the PLL clock source as the system clock to use.
    MPU6050_Write_Reg(0x6B, 0x01);

    // 8. Enable accelerometer and gyroscope
    MPU6050_Write_Reg(0x6C, 0x00);

    // 9. Perform bias calibration
    MPU6050_calculate_offset();
}

void MPU6050_Get_Gyro(Gyro_struct *gyro) {
    // The register address storing the gyroscope starts from 0x43 and consists of 8 consecutive bits in the order of XYZ.
    uint8_t high = 0;
    uint8_t low = 0;
    // X axis
    MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &high);
    MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &low);
    gyro->gyro_x = (int16_t)((high << 8) | low); // Do not reduce the bias for now
    // Y axis
    MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &high);
    MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &low);
    gyro->gyro_y = (int16_t)((high << 8) | low); // Do not reduce the bias for now
    // Z axis
    MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &high);
    MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &low);
    gyro->gyro_z = (int16_t)((high << 8) | low); // Do not reduce the bias for now
}

void MPU6050_Get_Acc(Accel_struct *acc) {
    uint8_t high = 0;
    uint8_t low = 0;
    // X axis
    MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &high);
    MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &low);
    acc->accel_x = (int16_t)((high << 8) | low) - acc_x_offset;
    // Y axis
    MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &high);
    MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &low);
    acc->accel_y = (int16_t)((high << 8) | low) - acc_y_offset;
    // Z axis
    MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &high);
    MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &low);
    acc->accel_z = (int16_t)((high << 8) | low) - acc_z_offset;
}

void MPU6050_Get_Data(Gyro_Accel_Struct *data) {
    MPU6050_Get_Gyro(&data->gyro);
    MPU6050_Get_Acc(&data->accel);
}
*/

#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void MPU6050_Read_Buffer(uint8_t reg, uint8_t *buffer, uint8_t len){
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    uint8_t count = Wire.requestFrom(static_cast<uint8_t>(MPU6050_ADDR), static_cast<uint8_t>(len));
    if(count != len){
        return;
    }
    for (uint8_t i = 0; i < len; i++){
        if (Wire.available()){
            buffer[i] = Wire.read();
        }
    }
}

void MPU6050_Get_Gyro(Gyro_struct *gyro){
    uint8_t buf[6];
    // gyro register starts at 0x43
    MPU6050_Read_Buffer(0x43, buf, 6);
    gyro->gyro_x = (int16_t)((buf[0] << 8) | buf[1]);
    gyro->gyro_y = (int16_t)((buf[2] << 8) | buf[3]);
    gyro->gyro_z = (int16_t)((buf[4] << 8) | buf[5]);
}

void MPU6050_Get_Acc(Accel_struct *acc){
    uint8_t buf[6];
    // accel register starts at 0x3B
    MPU6050_Read_Buffer(0x3B, buf, 6);
    acc->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    acc->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    acc->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
}

static void MPU6050_calculate_offset(void) {
    // 1. Wait for the aircraft to come to a stable stop
    // Criteria for determining whether the aircraft has come to a stable stop: The difference between two consecutive accelerometer readings is less than 200, for 100 consecutive times.
    Accel_struct current_accel = {0};
    Accel_struct last_accel = {0};
    MPU6050_Get_Acc(&last_accel);
    uint8_t count = 0;
    uint32_t start = millis();
    while (count < 100) {
        if (millis() - start > 10000){
            // Serial.println("Calibration timeout");
            break;
        }

        MPU6050_Get_Acc(&current_accel);
        // To determine if an aircraft is stable, the selected parameters should be low; 
        // otherwise, it will be impossible to determine whether it is stable.
        if (abs(current_accel.accel_x - last_accel.accel_x) < 400 &&
            abs(current_accel.accel_y - last_accel.accel_y) < 400 &&
            abs(current_accel.accel_z - last_accel.accel_z) < 400) {
            count++;
        } else {
            count = 0;
        }
        last_accel = current_accel;
        vTaskDelay(pdMS_TO_TICKS(4));
    }

    // 2. The aircraft has stabilized and is beginning offset calibration.
    Gyro_Accel_Struct gyro_accel_data = {0};
    int32_t acc_x_sum = 0;
    int32_t acc_y_sum = 0;
    int32_t acc_z_sum = 0;

    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;

    for (uint8_t i = 0; i < 100; i++) {
        // Reread acceleration and angular velocity
        MPU6050_Get_Data(&gyro_accel_data);
        acc_x_sum += (gyro_accel_data.accel.accel_x - 0);
        acc_y_sum += (gyro_accel_data.accel.accel_y - 0);
        // The initial value of the Z-axis acceleration should be 1g => approximately 16384.
        acc_z_sum += (gyro_accel_data.accel.accel_z - 16384);

        gyro_x_sum += (gyro_accel_data.gyro.gyro_x - 0);
        gyro_y_sum += (gyro_accel_data.gyro.gyro_y - 0);
        gyro_z_sum += (gyro_accel_data.gyro.gyro_z - 0);

        // Each data collection requires a time delay; the average of the first and last data collections is used as the bias.
        vTaskDelay(pdMS_TO_TICKS(4));
    }

    acc_x_offset = acc_x_sum / 100;
    acc_y_offset = acc_y_sum / 100;
    acc_z_offset = acc_z_sum / 100;

    gyro_x_offset = gyro_x_sum / 100;
    gyro_y_offset = gyro_y_sum / 100;
    gyro_z_offset = gyro_z_sum / 100;
}

void MPU6050_Get_Data(Gyro_Accel_Struct *imu){
    uint8_t buf[14];
    MPU6050_Read_Buffer(0x3B, buf, 14);

    // ACC
    imu->accel.accel_x =
        ((int16_t)(buf[0] << 8 | buf[1])) - acc_x_offset;
    imu->accel.accel_y =
        ((int16_t)(buf[2] << 8 | buf[3])) - acc_y_offset;
    imu->accel.accel_z =
        ((int16_t)(buf[4] << 8 | buf[5])) - acc_z_offset;

    // GYRO
    imu->gyro.gyro_x =
        ((int16_t)(buf[8] << 8 | buf[9])) - gyro_x_offset;
    imu->gyro.gyro_y =
        ((int16_t)(buf[10] << 8 | buf[11])) - gyro_y_offset;
    imu->gyro.gyro_z =
        ((int16_t)(buf[12] << 8 | buf[13])) - gyro_z_offset;
}

void MPU6050_Init(int sdaPin, int sclPin, uint32_t frequency){
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(frequency); //400kHz I2C

    delay(100);

    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    delay(100);

    // Gyro ±2000 dps
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);
    Wire.write(0x18);
    Wire.endTransmission();

    // Accel ±2g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission();

    delay(100);

    MPU6050_calculate_offset();
}

#define IMU_PERIOD_MS 4
TaskHandle_t imuTaskHandle = NULL;
void imu_task(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_us = micros();

    Gyro_Accel_Struct imu;
    Euler_struct euler_angle;
    Gyro_struct last_gyro = {0};

    uint32_t last_log_ms = 0;

    for (;;){
        uint32_t now = micros();
        float dt = (now - last_us) * 1e-6f;
        last_us = now;

        // clamp an toàn
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.02f)  dt = 0.02f;

        // ================= READ IMU =================
        MPU6050_Get_Data(&imu);

        // ================= LOWPASS GYRO =================
        imu.gyro.gyro_x = Filter_LowPass(imu.gyro.gyro_x, last_gyro.gyro_x);
        imu.gyro.gyro_y = Filter_LowPass(imu.gyro.gyro_y, last_gyro.gyro_y);
        imu.gyro.gyro_z = Filter_LowPass(imu.gyro.gyro_z, last_gyro.gyro_z);
        last_gyro = imu.gyro;

        // ================= KALMAN ACC =================
        imu.accel.accel_x = Filter_KalmanFilter(&kfs[0], imu.accel.accel_x);
        imu.accel.accel_y = Filter_KalmanFilter(&kfs[1], imu.accel.accel_y);
        imu.accel.accel_z = Filter_KalmanFilter(&kfs[2], imu.accel.accel_z);

        // ================= MAHONY AHRS =================
        IMU_GetEulerAngle(&imu, &euler_angle, dt);

        if (millis() - last_log_ms >= 1000) {
            Serial.print("Roll: ");
            Serial.print(euler_angle.roll, 2);

            Serial.print("  Pitch: ");
            Serial.print(euler_angle.pitch, 2);

            Serial.print("  Yaw: ");
            Serial.println(euler_angle.yaw, 2);

            last_log_ms = millis();
        }

        // ================= LOOP 250Hz =================
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IMU_PERIOD_MS));//250Hz
    }
}

void imu_start(void){
    xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        4096,
        NULL,
        3,              // priority vừa phải
        &imuTaskHandle,
        1               // core 1 (tránh conflict WiFi core 0)
    );
}
