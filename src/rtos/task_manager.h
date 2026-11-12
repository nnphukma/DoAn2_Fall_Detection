#pragma once

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "mpu6050.h"
#include "fall_detector.h"

bool startRtosTasks();

QueueHandle_t getSensorQueue();
QueueHandle_t getFallEventQueue();
QueueHandle_t getBleLogQueue();
void queueBleLogSample(const Mpu6050Data &sample, const FallDetectorResult &result);
uint32_t bleQueueDroppedPackets();
