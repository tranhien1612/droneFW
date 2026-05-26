#include <Arduino.h>
#include "config.h"
#include "../IMU/mpu6050.h"

void setup() {
    Serial.begin(115200);
    MPU6050_Init(PIN_SDA, PIN_SCL, I2C_CLOCK);
    imu_start();
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
