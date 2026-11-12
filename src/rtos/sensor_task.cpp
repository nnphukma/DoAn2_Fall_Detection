#include "task_manager.h"
#include "sensor_manager.h"
#include "config.h"

extern SensorManager sensorManager;

void sensorTask(void *parameter)
{
    Mpu6050Data sample{};
    FallDetector fallDetector;

    fallDetector.begin();

    for (;;)
    {
        // Wait for MPU6050 Data Ready interrupt.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!sensorManager.update(sample))
            continue;

        // Read buttons: active LOW.
        const bool sosPressed =
            digitalRead(Config::BTN_SOS_PIN) == LOW;

        const bool cancelPressed =
            digitalRead(Config::BTN_CANCEL_PIN) == LOW;

        // Run fall detection and button debounce.
        FallDetectorResult result = fallDetector.update(
            sample,
            sosPressed,
            cancelPressed
        );

        // Control local alarm.
        digitalWrite(
            Config::BUZZER_PIN,
            result.buzzerOn ? HIGH : LOW
        );

        digitalWrite(
            Config::FALL_LED_PIN,
            result.fallLedOn ? HIGH : LOW
        );

        // Send events to ProcessingTask.
        if (result.event != FallEventType::NONE)
        {
            xQueueSend(
                getFallEventQueue(),
                &result,
                0
            );
        }

        // Preserve existing sensor logging.
        xQueueSend(
            getSensorQueue(),
            &sample,
            0
        );
    }
}