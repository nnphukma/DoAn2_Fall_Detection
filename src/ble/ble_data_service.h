#pragma once

#include <Arduino.h>
#include "mpu6050.h"
#include "fall_detector.h"

constexpr char BLE_DEVICE_NAME[] = "FallDetector_ESP32C3";
constexpr char BLE_SERVICE_UUID[] = "7c4c1000-8a3f-4c7d-a1f5-9b7f0c000001";
constexpr char BLE_DATA_CHARACTERISTIC_UUID[] = "7c4c1001-8a3f-4c7d-a1f5-9b7f0c000001";

struct BleLogItem
{
    uint32_t sequence;
    Mpu6050Data sample;
    FallDetectorResult result;
};

bool beginBleDataService();
bool bleClientSubscribed();
bool bleNotifySample(const BleLogItem &item, uint32_t droppedPackets);
uint32_t bleNotificationFailures();
