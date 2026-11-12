#include "task_manager.h"

static void printSensorData(const Mpu6050Data &sample)
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

        sample.accelX_raw_g,
        sample.accelY_raw_g,
        sample.accelZ_raw_g,

        sample.gyroX_raw_dps,
        sample.gyroY_raw_dps,
        sample.gyroZ_raw_dps,

        sample.accelX_g,
        sample.accelY_g,
        sample.accelZ_g,

        sample.gyroX_dps,
        sample.gyroY_dps,
        sample.gyroZ_dps,

        sample.q0,
        sample.q1,
        sample.q2,
        sample.q3,

        sample.roll_deg,
        sample.pitch_deg
    );
}

static void printFallEvent(const FallDetectorResult &event)
{
    Serial.printf(
        "\n[EVENT] time=%lu | event=%s | state=%s\n",
        static_cast<unsigned long>(event.timestamp_ms),
        fallEventName(event.event),
        fallStateName(event.state)
    );

    Serial.printf(
        "[EVENT] angle=%.2f deg | accel=%.3f g | gyro=%.2f dps\n",
        event.postureAngle_deg,
        event.accelMagnitude_raw_g,
        event.gyroMagnitude_raw_dps
    );
}

void processingTask(void *parameter)
{
    Mpu6050Data sample{};
    FallDetectorResult event{};

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
        // Receive sensor data without blocking indefinitely.
        if (xQueueReceive(
                getSensorQueue(),
                &sample,
                pdMS_TO_TICKS(5)) == pdTRUE)
        {
            printSensorData(sample);
        }

        // Drain all pending fall/button events.
        while (xQueueReceive(
                   getFallEventQueue(),
                   &event,
                   0) == pdTRUE)
        {
            printFallEvent(event);
        }
    }
}