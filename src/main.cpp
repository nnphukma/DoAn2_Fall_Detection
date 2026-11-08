
#include <Arduino.h>

#include "config.h"
#include "sensor_manager.h"
#include "task_manager.h"

// Global SensorManager instance.
// Used by sensorTask().
SensorManager sensorManager;

void setup()
{
    // Initialize USB Serial
    Serial.begin(Config::SERIAL_BAUDRATE);
    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("       FALL DETECTION SYSTEM");
    Serial.println("       ESP32-C3 + MPU6050");
    Serial.println("========================================");

    // Initialize MPU6050
    Serial.println("[INIT] Initializing MPU6050...");

    if (!sensorManager.begin())
    {
        Serial.println("[ERROR] MPU6050 initialization failed!");
        Serial.println("[ERROR] Check I2C connection and sensor address.");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("[OK] MPU6050 initialized successfully!");
    Serial.println("[OK] I2C Address: 0x68");

    // Start FreeRTOS tasks
    Serial.println("[INIT] Starting FreeRTOS tasks...");

    if (!startRtosTasks())
    {
        Serial.println("[ERROR] Failed to create FreeRTOS tasks!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("[OK] FreeRTOS tasks started!");
    Serial.println("[INFO] Sampling frequency: 100 Hz");
    Serial.println("[INFO] Waiting for sensor data...");
    Serial.println("----------------------------------------");
}

void loop()
{
    // Sensor acquisition and processing
    // are handled by FreeRTOS tasks.
    delay(1000);
}
