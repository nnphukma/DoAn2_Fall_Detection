#include <Arduino.h>

#include "config.h"
#include "sensor_manager.h"
#include "task_manager.h"

SensorManager sensorManager;

void setup()
{
    Serial.begin(Config::SERIAL_BAUDRATE);
    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("       FALL DETECTION SYSTEM");
    Serial.println("       ESP32-C3 + MPU6050");
    Serial.println("========================================");

    Serial.println("[INIT] Initializing MPU6050...");

    if (!sensorManager.begin())
    {
        Serial.println("[ERROR] MPU6050 initialization failed!");
        Serial.println("[ERROR] Check I2C or gyro calibration.");
        Serial.println("[INFO] Keep device stationary during startup.");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("[OK] MPU6050 initialized.");
    Serial.println("[OK] Gyroscope calibration completed.");

    Serial.println("[INIT] Starting FreeRTOS tasks...");

    if (!startRtosTasks())
    {
        Serial.println("[ERROR] Failed to start FreeRTOS tasks!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("[OK] FreeRTOS tasks started.");
    Serial.println("[OK] MPU6050 Data Ready INT enabled.");
    Serial.println("[INFO] INT GPIO: 10");
    Serial.println("[INFO] Sampling frequency: 100 Hz");
    Serial.println("[INFO] Waiting for sensor data...");
    Serial.println("----------------------------------------");
}

void loop()
{
    delay(1000);
}