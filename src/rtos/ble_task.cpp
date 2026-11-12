#include "task_manager.h"
#include "ble_data_service.h"

void bleTask(void *parameter)
{
    BleLogItem item{};

    for (;;)
    {
        if (xQueueReceive(getBleLogQueue(), &item, portMAX_DELAY) == pdTRUE)
        {
            // GATT work runs outside the time-critical SensorTask.
            bleNotifySample(item, bleQueueDroppedPackets());
        }
    }
}
