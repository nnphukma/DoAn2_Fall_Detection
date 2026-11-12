#pragma once

#include <Arduino.h>
#include "mpu6050.h"
#include "fall_detector.h"

namespace BleProtocol
{
constexpr uint8_t VERSION = 1;
constexpr uint8_t PACKET_SIZE = 64;
constexpr uint8_t FRAME_MAGIC = 0xA5;
constexpr uint8_t FRAME_HEADER_SIZE = 6;

// Serialises a fixed, little-endian, padding-free packet. See docs/BLE_PROTOCOL.md.
void serializeSample(uint32_t sequence, uint32_t droppedPackets,
                     const Mpu6050Data &sample, const FallDetectorResult &result,
                     uint8_t output[PACKET_SIZE]);
uint16_t crc16Ccitt(const uint8_t *data, size_t length);
}
