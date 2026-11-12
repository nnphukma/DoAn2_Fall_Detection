#include "task_manager.h"
#include "sensor_manager.h"
#include "config.h"
#include "ble_data_service.h"

extern SensorManager sensorManager;

void sensorTask(void *parameter);
void processingTask(void *parameter);
void bleTask(void *parameter);

static QueueHandle_t sensorQueue = nullptr;
static QueueHandle_t fallEventQueue = nullptr;
static QueueHandle_t bleLogQueue = nullptr;

static TaskHandle_t sensorTaskHandle = nullptr;
static TaskHandle_t processingTaskHandle = nullptr;
static TaskHandle_t bleTaskHandle = nullptr;
static uint32_t bleSequence = 0;
static volatile uint32_t bleQueueDrops = 0;

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
        portYIELD_FROM_ISR();
}

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

QueueHandle_t getFallEventQueue()
{
    return fallEventQueue;
}

QueueHandle_t getBleLogQueue()
{
    return bleLogQueue;
}

uint32_t bleQueueDroppedPackets()
{
    return bleQueueDrops;
}

void queueBleLogSample(const Mpu6050Data &sample, const FallDetectorResult &result)
{
    BleLogItem item{bleSequence++, sample, result};

    // No client: no queue allocation/copy. The sequence still advances, so a
    // later connection can identify the acquisition timeline.
    if (!bleClientSubscribed())
        return;

    // SensorTask is priority 3. This operation is strictly non-blocking.
    if (xQueueSend(bleLogQueue, &item, 0) != pdTRUE)
        ++bleQueueDrops;
}

bool startRtosTasks()
{
    sensorQueue = xQueueCreate(
        Config::SENSOR_QUEUE_LENGTH,
        sizeof(Mpu6050Data)
    );

    if (sensorQueue == nullptr)
        return false;

    fallEventQueue = xQueueCreate(
        Config::FALL_EVENT_QUEUE_LENGTH,
        sizeof(FallDetectorResult)
    );

    if (fallEventQueue == nullptr)
    {
        vQueueDelete(sensorQueue);
        sensorQueue = nullptr;
        return false;
    }

    bleLogQueue = xQueueCreate(Config::BLE_QUEUE_LENGTH, sizeof(BleLogItem));
    if (bleLogQueue == nullptr)
    {
        vQueueDelete(fallEventQueue);
        vQueueDelete(sensorQueue);
        fallEventQueue = nullptr;
        sensorQueue = nullptr;
        return false;
    }

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
        vQueueDelete(bleLogQueue);
        vQueueDelete(fallEventQueue);
        vQueueDelete(sensorQueue);

        bleLogQueue = nullptr;
        fallEventQueue = nullptr;
        sensorQueue = nullptr;

        return false;
    }

    BaseType_t bleCreated = xTaskCreate(
        bleTask,
        "BleTask",
        4096,
        nullptr,
        1,
        &bleTaskHandle
    );

    if (bleCreated != pdPASS)
    {
        vTaskDelete(processingTaskHandle);
        processingTaskHandle = nullptr;
        vQueueDelete(bleLogQueue);
        vQueueDelete(fallEventQueue);
        vQueueDelete(sensorQueue);
        bleLogQueue = nullptr;
        fallEventQueue = nullptr;
        sensorQueue = nullptr;
        return false;
    }

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
        vTaskDelete(bleTaskHandle);
        vTaskDelete(processingTaskHandle);

        bleTaskHandle = nullptr;
        processingTaskHandle = nullptr;

        vQueueDelete(bleLogQueue);
        vQueueDelete(fallEventQueue);
        vQueueDelete(sensorQueue);

        bleLogQueue = nullptr;
        fallEventQueue = nullptr;
        sensorQueue = nullptr;

        return false;
    }

    attachMpu6050Interrupt();

    if (!sensorManager.enableDataReadyInterrupt())
    {
        detachInterrupt(
            digitalPinToInterrupt(Config::MPU_INT_PIN)
        );

        vTaskDelete(sensorTaskHandle);
        vTaskDelete(bleTaskHandle);
        vTaskDelete(processingTaskHandle);

        sensorTaskHandle = nullptr;
        bleTaskHandle = nullptr;
        processingTaskHandle = nullptr;

        vQueueDelete(bleLogQueue);
        vQueueDelete(fallEventQueue);
        vQueueDelete(sensorQueue);

        bleLogQueue = nullptr;
        fallEventQueue = nullptr;
        sensorQueue = nullptr;

        return false;
    }

    return true;
}
