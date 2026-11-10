
#include "task_manager.h"

void processingTask(void *parameter)
{
    Mpu6050Data sample{};

    // CSV header
    Serial.println(
        "timestamp_ms,"
        "ax_raw,ay_raw,az_raw,"
        "gx_raw,gy_raw,gz_raw,"
        "ax_ema,ay_ema,az_ema,"
        "gx_ema,gy_ema,gz_ema,"
        "q0,q1,q2,q3,"
        "roll_deg,pitch_deg"
    );

    for (;;)
    {
        if (xQueueReceive(
                getSensorQueue(),
                &sample,
                portMAX_DELAY) == pdTRUE)
        {
            Serial.printf(
                "%lu,"
                "%.3f,%.3f,%.3f,"
                "%.2f,%.2f,%.2f,"
                "%.3f,%.3f,%.3f,"
                "%.2f,%.2f,%.2f,"
                "%.5f,%.5f,%.5f,%.5f,"
                "%.2f,%.2f\n",

                static_cast<unsigned long>(sample.timestamp_ms),

                // Raw acceleration
                sample.accelX_raw_g,
                sample.accelY_raw_g,
                sample.accelZ_raw_g,

                // Raw gyroscope
                sample.gyroX_raw_dps,
                sample.gyroY_raw_dps,
                sample.gyroZ_raw_dps,

                // EMA acceleration
                sample.accelX_g,
                sample.accelY_g,
                sample.accelZ_g,

                // EMA gyroscope
                sample.gyroX_dps,
                sample.gyroY_dps,
                sample.gyroZ_dps,

                // Mahony quaternion
                sample.q0,
                sample.q1,
                sample.q2,
                sample.q3,

                // Attitude angles
                sample.roll_deg,
                sample.pitch_deg
            );
        }
    }
}
