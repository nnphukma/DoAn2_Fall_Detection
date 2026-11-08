#pragma once

#include <Arduino.h>

namespace Config
{
    // MPU6050 - I2C
    constexpr uint8_t MPU_SDA_PIN = 6;
    constexpr uint8_t MPU_SCL_PIN = 7;
    constexpr uint8_t MPU_INT_PIN = 10;

    constexpr uint32_t I2C_CLOCK_HZ = 400000;
    constexpr uint8_t MPU6050_ADDRESS = 0x68;

    // Sensor sampling
    constexpr uint32_t SENSOR_PERIOD_MS = 10; // 100 Hz

    // FreeRTOS
    constexpr uint8_t SENSOR_QUEUE_LENGTH = 10;

    // Serial
    constexpr uint32_t SERIAL_BAUDRATE = 115200;
}