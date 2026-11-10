#include "task_manager.h"

void processingTask(void *parameter)
{
    Mpu6050Data sample{};

    uint32_t sampleCount = 0;

    for (;;)
    {
        if (xQueueReceive(getSensorQueue(), &sample, portMAX_DELAY) == pdTRUE)
        {
            // Print approximately 10 lines per second.
            if (++sampleCount >= 10)
            {
                sampleCount = 0;

                Serial.println("\n========== MPU6050 DATA ==========");

                Serial.printf(
                    "RAW ACC [g]  X:%7.3f Y:%7.3f Z:%7.3f | MAG:%6.3f\n",
                    sample.accelX_raw_g,
                    sample.accelY_raw_g,
                    sample.accelZ_raw_g,
                    sample.accelMagnitude_raw_g
                );

                Serial.printf(
                    "RAW GYRO[dps] X:%7.2f Y:%7.2f Z:%7.2f | MAG:%6.2f\n",
                    sample.gyroX_raw_dps,
                    sample.gyroY_raw_dps,
                    sample.gyroZ_raw_dps,
                    sample.gyroMagnitude_raw_dps
                );

                Serial.println("----------------------------------");

                Serial.printf(
                    "EMA ACC [g]  X:%7.3f Y:%7.3f Z:%7.3f | MAG:%6.3f\n",
                    sample.accelX_g,
                    sample.accelY_g,
                    sample.accelZ_g,
                    sample.accelMagnitude_g
                );

                Serial.printf(
                    "EMA GYRO[dps] X:%7.2f Y:%7.2f Z:%7.2f | MAG:%6.2f\n",
                    sample.gyroX_dps,
                    sample.gyroY_dps,
                    sample.gyroZ_dps,
                    sample.gyroMagnitude_dps
                );

                Serial.println("----------------------------------");

                Serial.printf(
                    "MAHONY | Roll:%7.2f deg | Pitch:%7.2f deg\n",
                    sample.roll_deg,
                    sample.pitch_deg
                );

                Serial.printf(
                    "QUATERNION | q0:%7.3f q1:%7.3f q2:%7.3f q3:%7.3f\n",
                    sample.q0,
                    sample.q1,
                    sample.q2,
                    sample.q3
                );

                Serial.printf(
                    "TIMESTAMP: %lu ms\n",
                    static_cast<unsigned long>(sample.timestamp_ms)
                );

                Serial.println("==================================");
            }
        }
    }
}