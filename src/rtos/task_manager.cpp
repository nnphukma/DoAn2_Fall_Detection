#include "task_manager.h"
#include "config.h"

void sensorTask(void *parameter);
void processingTask(void *parameter);

static QueueHandle_t sensorQueue = nullptr;

QueueHandle_t getSensorQueue() {
    return sensorQueue;
}

bool startRtosTasks() {
    sensorQueue = xQueueCreate(
        Config::SENSOR_QUEUE_LENGTH,
        sizeof(Mpu6050Data)
    );

    if (sensorQueue == nullptr) return false;

    BaseType_t sensorCreated = xTaskCreate(
        sensorTask,
        "SensorTask",
        4096,
        nullptr,
        3,
        nullptr
    );

    BaseType_t processingCreated = xTaskCreate(
        processingTask,
        "ProcessingTask",
        4096,
        nullptr,
        2,
        nullptr
    );

    return (sensorCreated == pdPASS && processingCreated == pdPASS);
}
