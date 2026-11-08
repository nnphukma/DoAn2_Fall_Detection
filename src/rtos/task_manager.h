#pragma once

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "mpu6050.h"

bool startRtosTasks();

QueueHandle_t getSensorQueue();