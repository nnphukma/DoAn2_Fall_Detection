#include "task_manager.h"
#include "sensor_manager.h"

#include "config.h"

extern SensorManager sensorManager;

void sensorTask(void *parameter)
{
    Mpu6050Data sample{};

    for (;;)
    {
        // Wait until MPU6050 Data Ready interrupt occurs.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Read sensor data.
        if (sensorManager.update(sample))
        {
            // Send latest sample to ProcessingTask.
            xQueueSend(
                getSensorQueue(),
                &sample,
                0
            );
        }
    }
}