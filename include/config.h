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

    // Battery monitor: BAT+ -> 100k -> GPIO0 -> 100k -> GND
    constexpr uint8_t BATTERY_ADC_PIN = 0; // ESP32-C3 GPIO0 = ADC1_CH0
    constexpr uint8_t BATTERY_DIVIDER_RATIO = 2; // R1 = R2 = 100k
    constexpr uint16_t BATTERY_EMPTY_MV = 3600;
    constexpr uint16_t BATTERY_FULL_MV = 4200;

    // FreeRTOS
    constexpr uint8_t SENSOR_QUEUE_LENGTH = 20;
    constexpr uint8_t FALL_EVENT_QUEUE_LENGTH = 8;
    // Bounded telemetry queue. SensorTask must never wait for BLE.
    constexpr uint8_t BLE_QUEUE_LENGTH = 32;

    // User interface GPIO
    constexpr uint8_t BTN_SOS_PIN = 4;       // Active LOW, INPUT_PULLUP
    constexpr uint8_t BTN_CANCEL_PIN = 5;    // Active LOW, INPUT_PULLUP
    constexpr uint8_t BUZZER_PIN = 1;        // HIGH = buzzer ON (transistor driver)
    constexpr uint8_t FALL_LED_PIN = 20;     // Active HIGH
    constexpr uint8_t BLE_STATUS_LED_PIN = 21;

    // Serial
    constexpr uint32_t SERIAL_BAUDRATE = 115200;
}
