#include <Arduino.h>

#include "config.h"
#include "imu.h"

static bool imu_ready = false;

void setup()
{
    Serial.begin(115200);
    delay(500);

    const ImuConfig imu_config = {
        PIN_SDA,
        PIN_SCL,
        I2C_CLOCK,
        0.0f,
        0.0f,
        0.05f,
        false
    };

    if (!IMU_Begin(&imu_config)) {
        Serial.printf("IMU init failed: %s\n", IMU_GetLastError());
        return;
    }

    if (!IMU_StartTask()) {
        Serial.println("IMU task start failed");
        return;
    }

    imu_ready = true;
    Serial.printf("IMU ready at I2C address 0x%02X\n", IMU_GetDeviceAddress());
}

void loop()
{
    static uint32_t last_log_ms = 0;

    if (!imu_ready) {
        delay(1000);
        return;
    }

    if (millis() - last_log_ms >= 500) {
        ImuData imu = {};
        ImuStats stats = {};
        IMU_GetData(&imu);
        IMU_GetStats(&stats);
        const bool ok = IMU_IsHealthy();

        Serial.printf(
            "IMU %s | Roll %.2f Pitch %.2f Yaw %.2f | Gyro %.2f %.2f %.2f | Acc %.2f %.2f %.2f | %lu us | fail %lu/%lu max %lu us\n",
            ok ? "OK" : "ERR",
            imu.roll_deg,
            imu.pitch_deg,
            imu.yaw_deg,
            imu.gyro_roll_dps,
            imu.gyro_pitch_dps,
            imu.gyro_yaw_dps,
            imu.acc_x_g,
            imu.acc_y_g,
            imu.acc_z_g,
            (unsigned long)imu.update_us,
            (unsigned long)stats.fail_count,
            (unsigned long)stats.read_count,
            (unsigned long)stats.max_read_time_us
        );

        last_log_ms = millis();
    }

    delay(10);
}
