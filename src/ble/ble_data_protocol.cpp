#include "ble_data_protocol.h"

namespace
{
void putU16(uint8_t *buffer, size_t &at, uint16_t value) { buffer[at++] = value & 0xff; buffer[at++] = value >> 8; }
void putU32(uint8_t *buffer, size_t &at, uint32_t value) { for (uint8_t i = 0; i < 4; ++i) buffer[at++] = value >> (8 * i); }
int16_t scaled(float value, float scale) {
    float converted = value * scale;
    if (converted > 32767.0f) return 32767;
    if (converted < -32768.0f) return -32768;
    return static_cast<int16_t>(converted >= 0 ? converted + 0.5f : converted - 0.5f);
}
void putI16(uint8_t *buffer, size_t &at, float value, float scale) { putU16(buffer, at, static_cast<uint16_t>(scaled(value, scale))); }
}

uint16_t BleProtocol::crc16Ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    while (length--) {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

void BleProtocol::serializeSample(uint32_t sequence, uint32_t droppedPackets,
                                  const Mpu6050Data &s, const FallDetectorResult &r,
                                  uint8_t output[PACKET_SIZE])
{
    size_t at = 0;
    output[at++] = VERSION;
    putU32(output, at, sequence);
    putU32(output, at, s.timestamp_ms);
    const float raw[] = {s.accelX_raw_g, s.accelY_raw_g, s.accelZ_raw_g, s.gyroX_raw_dps, s.gyroY_raw_dps, s.gyroZ_raw_dps};
    const float ema[] = {s.accelX_g, s.accelY_g, s.accelZ_g, s.gyroX_dps, s.gyroY_dps, s.gyroZ_dps};
    for (uint8_t i = 0; i < 3; ++i) putI16(output, at, raw[i], 1000.0f);
    for (uint8_t i = 3; i < 6; ++i) putI16(output, at, raw[i], 10.0f);
    for (uint8_t i = 0; i < 3; ++i) putI16(output, at, ema[i], 1000.0f);
    for (uint8_t i = 3; i < 6; ++i) putI16(output, at, ema[i], 10.0f);
    putI16(output, at, s.accelMagnitude_raw_g, 1000.0f);
    putI16(output, at, s.gyroMagnitude_raw_dps, 10.0f);
    putI16(output, at, r.postureAngle_deg, 100.0f);
    uint8_t flags = (r.postureValid ? 1 : 0) | (r.freeFallCondition ? 2 : 0) |
                    (r.freeFallTrigger ? 4 : 0) | (r.impactTrigger ? 8 : 0) |
                    (r.rotationTrigger ? 16 : 0) | (r.postureCondition ? 32 : 0) |
                    (r.postureConfirmed ? 64 : 0) | (r.stillnessCondition ? 128 : 0);
    output[at++] = flags;
    output[at++] = static_cast<uint8_t>(r.state);
    output[at++] = static_cast<uint8_t>(r.event);
    putU32(output, at, r.stateElapsed_ms);
    putU32(output, at, r.postureElapsed_ms);
    putU32(output, at, r.stillnessElapsed_ms);
    putU32(output, at, r.countdownRemaining_ms);
    putU32(output, at, droppedPackets);
    putU16(output, at, crc16Ccitt(output, at));
}
