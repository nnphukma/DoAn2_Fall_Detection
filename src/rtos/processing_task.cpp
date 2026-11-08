#include "task_manager.h"

void processingTask(void *parameter)
{
    Mpu6050Data sample{};

    uint32_t sampleCount = 0;

    for (;;)
    {
        if (xQueueReceive(
                getSensorQueue(),
                &sample,
                portMAX_DELAY
            ) == pdTRUE)
        {
            // Print approximately 10 lines per second.
            if (++sampleCount >= 10)
            {
                sampleCount = 0;

                Serial.printf(
                    "ACC[g] X:%7.3f Y:%7.3f Z:%7.3f | "
                    "GYRO[dps] X:%7.2f Y:%7.2f Z:%7.2f | "
                    "TIME:%lu ms\n",

                    sample.accelX_g,
                    sample.accelY_g,
                    sample.accelZ_g,

                    sample.gyroX_dps,
                    sample.gyroY_dps,
                    sample.gyroZ_dps,

                    static_cast<unsigned long>(sample.timestamp_ms)
                );
            }
        }
    }
}