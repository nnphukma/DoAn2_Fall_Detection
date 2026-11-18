#pragma once
#include <Arduino.h>
#include "mpu6050.h"
#include "fall_detector.h"

namespace BleProtocol
{
constexpr uint8_t VERSION = 2;
constexpr uint8_t PACKET_SIZE = 67;
constexpr uint8_t FRAME_MAGIC = 0xA5;
constexpr uint8_t FRAME_HEADER_SIZE = 6;

// V2 extends V1 with battery voltage in mV and battery percentage (0..100).
void serializeSample(uint32_t sequence, uint32_t droppedPackets,
                     const Mpu6050Data &sample, const FallDetectorResult &result,
                     uint16_t batteryVoltageMv, uint8_t batteryPercent,
                     uint8_t output[PACKET_SIZE]);
uint16_t crc16Ccitt(const uint8_t *data, size_t length);
}
