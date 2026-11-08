#include "task_manager.h"
#include "sensor_manager.h"
#include "config.h"

extern SensorManager sensorManager;

void sensorTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(Config::SENSOR_PERIOD_MS);

    Mpu6050Data sample{};

    for (;;) {
        if (sensorManager.update(sample)) {
            // Do not block the sampling task if the consumer is late.
            xQueueSend(getSensorQueue(), &sample, 0);
        }

        vTaskDelayUntil(&lastWakeTime, period);
    }
}
