#include "mpu6050.h"

#include <Arduino.h>
#include <Wire.h>

static int32_t acc_x_offset = 0;
static int32_t acc_y_offset = 0;
static int32_t acc_z_offset = 0;
static int32_t gyro_x_offset = 0;
static int32_t gyro_y_offset = 0;
static int32_t gyro_z_offset = 0;
static uint8_t mpu6050_addr = MPU6050_ADDR;
static const char *last_error = "not initialized";

static int16_t clamp_i16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static bool write_reg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(mpu6050_addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool read_buffer(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    Wire.beginTransmission(mpu6050_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const uint8_t count = Wire.requestFrom(mpu6050_addr, len);
    if (count != len) {
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) {
            return false;
        }
        buffer[i] = Wire.read();
    }

    return true;
}

static bool read_raw_no_offset(ImuRaw *raw)
{
    uint8_t buf[14] = {0};
    if (!read_buffer(MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
        return false;
    }

    raw->acc_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw->acc_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw->acc_z = (int16_t)((buf[4] << 8) | buf[5]);
    raw->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    raw->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}

bool MPU6050_ReadWhoAmI(uint8_t *who_am_i)
{
    return who_am_i != nullptr && read_buffer(MPU6050_REG_WHO_AM_I, who_am_i, 1);
}

const char *MPU6050_GetLastError(void)
{
    return last_error;
}

uint8_t MPU6050_GetAddress(void)
{
    return mpu6050_addr;
}

void MPU6050_ResetOffsets(void)
{
    acc_x_offset = 0;
    acc_y_offset = 0;
    acc_z_offset = 0;
    gyro_x_offset = 0;
    gyro_y_offset = 0;
    gyro_z_offset = 0;
}

bool MPU6050_Calibrate(uint16_t samples, uint16_t sample_delay_ms)
{
    if (samples == 0) {
        return false;
    }

    MPU6050_ResetOffsets();

    int64_t acc_x_sum = 0;
    int64_t acc_y_sum = 0;
    int64_t acc_z_sum = 0;
    int64_t gyro_x_sum = 0;
    int64_t gyro_y_sum = 0;
    int64_t gyro_z_sum = 0;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < samples; i++) {
        ImuRaw raw = {0};
        if (read_raw_no_offset(&raw)) {
            acc_x_sum += raw.acc_x;
            acc_y_sum += raw.acc_y;
            acc_z_sum += raw.acc_z;
            gyro_x_sum += raw.gyro_x;
            gyro_y_sum += raw.gyro_y;
            gyro_z_sum += raw.gyro_z;
            valid_samples++;
        }

        delay(sample_delay_ms);
    }

    if (valid_samples == 0) {
        return false;
    }

    acc_x_offset = (int32_t)(acc_x_sum / valid_samples);
    acc_y_offset = (int32_t)(acc_y_sum / valid_samples);
    acc_z_offset = (int32_t)(acc_z_sum / valid_samples) - 4096;
    gyro_x_offset = (int32_t)(gyro_x_sum / valid_samples);
    gyro_y_offset = (int32_t)(gyro_y_sum / valid_samples);
    gyro_z_offset = (int32_t)(gyro_z_sum / valid_samples);

    return true;
}

bool MPU6050_ReadRaw(ImuRaw *raw)
{
    if (raw == nullptr) {
        return false;
    }

    ImuRaw uncompensated = {0};
    if (!read_raw_no_offset(&uncompensated)) {
        return false;
    }

    raw->acc_x = clamp_i16((int32_t)uncompensated.acc_x - acc_x_offset);
    raw->acc_y = clamp_i16((int32_t)uncompensated.acc_y - acc_y_offset);
    raw->acc_z = clamp_i16((int32_t)uncompensated.acc_z - acc_z_offset);
    raw->gyro_x = clamp_i16((int32_t)uncompensated.gyro_x - gyro_x_offset);
    raw->gyro_y = clamp_i16((int32_t)uncompensated.gyro_y - gyro_y_offset);
    raw->gyro_z = clamp_i16((int32_t)uncompensated.gyro_z - gyro_z_offset);

    return true;
}

bool MPU6050_Init(int sda_pin, int scl_pin, uint32_t i2c_clock)
{
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(i2c_clock);
    delay(100);

    uint8_t who_am_i = 0;
    mpu6050_addr = MPU6050_ADDR;
    bool found = MPU6050_ReadWhoAmI(&who_am_i);

    if (!found) {
        mpu6050_addr = MPU6050_ADDR_ALT;
        found = MPU6050_ReadWhoAmI(&who_am_i);
    }

    if (!found) {
        last_error = "no device replied at I2C address 0x68 or 0x69";
        return false;
    }

    if (who_am_i != MPU6050_ADDR) {
        last_error = "device replied, but WHO_AM_I is not 0x68";
        return false;
    }

    if (!write_reg(MPU6050_REG_PWR_MGMT_1, 0x00)) {
        last_error = "failed to wake MPU6050";
        return false;
    }
    delay(100);

    bool ok = true;
    ok = ok && write_reg(MPU6050_REG_SMPLRT_DIV, 0x03);
    ok = ok && write_reg(MPU6050_REG_CONFIG, 0x03);
    ok = ok && write_reg(MPU6050_REG_GYRO_CONFIG, 0x08);
    ok = ok && write_reg(MPU6050_REG_ACCEL_CONFIG, 0x10);
    ok = ok && write_reg(MPU6050_REG_PWR_MGMT_1, 0x01);

    if (!ok) {
        last_error = "failed to configure MPU6050 registers";
        return false;
    }

    delay(100);
    if (!MPU6050_Calibrate(800, 2)) {
        last_error = "calibration failed; raw reads returned no valid samples";
        return false;
    }

    last_error = "ok";
    return true;
}
