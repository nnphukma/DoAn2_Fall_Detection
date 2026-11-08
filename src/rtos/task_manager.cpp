#include "task_manager.h"
#include "sensor_manager.h"
#include "config.h"

extern SensorManager sensorManager;

void sensorTask(void *parameter);
void processingTask(void *parameter);

static QueueHandle_t sensorQueue = nullptr;

static TaskHandle_t sensorTaskHandle = nullptr;
static TaskHandle_t processingTaskHandle = nullptr;

// MPU6050 Data Ready ISR.
void IRAM_ATTR mpu6050DataReadyISR()
{
    if (sensorTaskHandle == nullptr)
        return;

    BaseType_t higherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(
        sensorTaskHandle,
        &higherPriorityTaskWoken
    );

    if (higherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

// Attach ESP32 GPIO interrupt to SensorTask.
static void attachMpu6050Interrupt()
{
    attachInterrupt(
        digitalPinToInterrupt(Config::MPU_INT_PIN),
        mpu6050DataReadyISR,
        RISING
    );
}

QueueHandle_t getSensorQueue()
{
    return sensorQueue;
}

bool startRtosTasks()
{
    sensorQueue = xQueueCreate(
        Config::SENSOR_QUEUE_LENGTH,
        sizeof(Mpu6050Data)
    );

    if (sensorQueue == nullptr)
        return false;

    // Create ProcessingTask first.
    BaseType_t processingCreated = xTaskCreate(
        processingTask,
        "ProcessingTask",
        4096,
        nullptr,
        2,
        &processingTaskHandle
    );

    if (processingCreated != pdPASS)
    {
        vQueueDelete(sensorQueue);
        sensorQueue = nullptr;

        return false;
    }

    // Create SensorTask.
    BaseType_t sensorCreated = xTaskCreate(
        sensorTask,
        "SensorTask",
        4096,
        nullptr,
        3,
        &sensorTaskHandle
    );

    if (sensorCreated != pdPASS)
    {
        vTaskDelete(processingTaskHandle);
        processingTaskHandle = nullptr;

        vQueueDelete(sensorQueue);
        sensorQueue = nullptr;

        return false;
    }

    // Attach GPIO interrupt after SensorTask exists.
    attachMpu6050Interrupt();

    // Enable MPU6050 Data Ready interrupt.
    if (!sensorManager.enableDataReadyInterrupt())
    {
        detachInterrupt(
            digitalPinToInterrupt(Config::MPU_INT_PIN)
        );

        vTaskDelete(sensorTaskHandle);
        vTaskDelete(processingTaskHandle);

        sensorTaskHandle = nullptr;
        processingTaskHandle = nullptr;

        vQueueDelete(sensorQueue);
        sensorQueue = nullptr;

        return false;
    }

    return true;
}