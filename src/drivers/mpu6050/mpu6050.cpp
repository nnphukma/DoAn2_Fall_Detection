#include "mpu6050.h"

namespace {
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;
}

MPU6050::MPU6050(uint8_t address, TwoWire &wire)
    : _address(address), _wire(wire) {}

bool MPU6050::begin() {
    // Wake up device; select the X gyro PLL clock.
    if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
    delay(100);

    // DLPF_CFG = 3: accel bandwidth 44 Hz, gyro bandwidth 42 Hz.
    if (!writeRegister(REG_CONFIG, 0x03)) return false;

    // With DLPF enabled, internal sample rate is 1 kHz.
    // 1 kHz / (1 + 9) = 100 Hz.
    if (!writeRegister(REG_SMPLRT_DIV, 0x09)) return false;

    // Accelerometer +/-2g; gyro +/-250 degrees/second.
    if (!writeRegister(REG_ACCEL_CONFIG, 0x00)) return false;
    if (!writeRegister(REG_GYRO_CONFIG, 0x00)) return false;

    return testConnection();
}

bool MPU6050::testConnection() {
    uint8_t whoAmI = 0;
    if (!readRegisters(REG_WHO_AM_I, &whoAmI, 1)) return false;

    // MPU6050 normally returns 0x68. Accept 0x74 as observed on
    // some compatible modules, but verify the actual value if it differs.
    return (whoAmI == 0x68 || whoAmI == 0x74);
}

bool MPU6050::readRaw(Mpu6050RawData &data) {
    uint8_t buffer[14];

    if (!readRegisters(REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
        return false;
    }

    auto toInt16 = [](uint8_t high, uint8_t low) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
    };

    data.accelX = toInt16(buffer[0],  buffer[1]);
    data.accelY = toInt16(buffer[2],  buffer[3]);
    data.accelZ = toInt16(buffer[4],  buffer[5]);
    data.temperature = toInt16(buffer[6], buffer[7]);
    data.gyroX = toInt16(buffer[8],  buffer[9]);
    data.gyroY = toInt16(buffer[10], buffer[11]);
    data.gyroZ = toInt16(buffer[12], buffer[13]);

    return true;
}

bool MPU6050::readSensor(Mpu6050Data &data) {
    Mpu6050RawData raw{};
    if (!readRaw(raw)) return false;

    // Sensitivity at +/-2g: 16384 LSB/g.
    data.accelX_g = raw.accelX / 16384.0f;
    data.accelY_g = raw.accelY / 16384.0f;
    data.accelZ_g = raw.accelZ / 16384.0f;

    // Sensitivity at +/-250 dps: 131 LSB/(degree/second).
    data.gyroX_dps = raw.gyroX / 131.0f;
    data.gyroY_dps = raw.gyroY / 131.0f;
    data.gyroZ_dps = raw.gyroZ / 131.0f;

    // MPU6050 temperature conversion.
    data.temperature_C = raw.temperature / 340.0f + 36.53f;

    return true;
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(value);
    return (_wire.endTransmission() == 0);
}

bool MPU6050::readRegisters(uint8_t startReg, uint8_t *buffer, size_t length) {
    _wire.beginTransmission(_address);
    _wire.write(startReg);

    if (_wire.endTransmission(false) != 0) return false;

    size_t received = _wire.requestFrom(
        static_cast<uint8_t>(_address),
        static_cast<uint8_t>(length),
        static_cast<uint8_t>(true)
    );

    if (received != length) {
        while (_wire.available()) _wire.read();
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        buffer[i] = static_cast<uint8_t>(_wire.read());
    }

    return true;
}
