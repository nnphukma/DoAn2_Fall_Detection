#include "mpu6050.h"

#include <math.h>

namespace
{
    // MPU6050 registers
    constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    constexpr uint8_t REG_CONFIG       = 0x1A;
    constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;

    constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

    constexpr uint8_t REG_INT_PIN_CFG = 0x37;
    constexpr uint8_t REG_INT_ENABLE  = 0x38;

    constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    constexpr uint8_t REG_WHO_AM_I   = 0x75;

    // Sensor configuration
    constexpr float ACCEL_SCALE = 16384.0f;
    constexpr float GYRO_SCALE  = 131.0f;

    // Gyroscope calibration
    constexpr uint16_t CALIBRATION_SAMPLES = 300;
    constexpr uint16_t CALIBRATION_DELAY_MS = 10;

    // Maximum acceptable gyro standard deviation in raw counts.
    // 300 counts correspond to approximately 2.29 dps.
    constexpr float MAX_GYRO_STDDEV_RAW = 300.0f;
}

MPU6050::MPU6050(uint8_t address, TwoWire &wire)
    : _address(address), _wire(wire)
{
}

bool MPU6050::begin()
{
    // Disable MPU interrupts during initialization.
    if (!writeRegister(REG_INT_ENABLE, 0x00))
        return false;

    // Wake up device and select X gyro PLL clock.
    if (!writeRegister(REG_PWR_MGMT_1, 0x01))
        return false;

    delay(100);

    // DLPF_CFG = 3:
    // Accelerometer bandwidth: approximately 44 Hz.
    // Gyroscope bandwidth: approximately 42 Hz.
    if (!writeRegister(REG_CONFIG, 0x03))
        return false;

    // Internal sample rate = 1 kHz.
    // 1000 / (1 + 9) = 100 Hz.
    if (!writeRegister(REG_SMPLRT_DIV, 0x09))
        return false;

    // Accelerometer: +/-2g.
    if (!writeRegister(REG_ACCEL_CONFIG, 0x00))
        return false;

    // Gyroscope: +/-250 dps.
    if (!writeRegister(REG_GYRO_CONFIG, 0x00))
        return false;

    // INT configuration:
    // INT_LEVEL = 0: Active HIGH.
    // INT_OPEN = 0: Push-pull.
    // LATCH_INT_EN = 1: Latched interrupt.
    // INT_RD_CLEAR = 1: Clear interrupt when any register is read.
    if (!writeRegister(REG_INT_PIN_CFG, 0x30))
        return false;

    if (!testConnection())
        return false;

    // Calibrate gyroscope while the device is stationary.
    if (!calibrateGyroscope())
        return false;

    return true;
}

bool MPU6050::enableDataReadyInterrupt()
{
    // Enable Data Ready Interrupt.
    return writeRegister(REG_INT_ENABLE, 0x01);
}

bool MPU6050::testConnection()
{
    uint8_t whoAmI = 0;

    if (!readRegisters(REG_WHO_AM_I, &whoAmI, 1))
        return false;

    return (whoAmI == 0x68 || whoAmI == 0x74);
}

bool MPU6050::readRaw(Mpu6050RawData &data)
{
    uint8_t buffer[14];

    if (!readRegisters(REG_ACCEL_XOUT_H, buffer, sizeof(buffer)))
        return false;

    auto toInt16 = [](uint8_t high, uint8_t low) -> int16_t
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(high) << 8) | low
        );
    };

    data.accelX = toInt16(buffer[0], buffer[1]);
    data.accelY = toInt16(buffer[2], buffer[3]);
    data.accelZ = toInt16(buffer[4], buffer[5]);

    // buffer[6] and buffer[7] are temperature.
    // We intentionally ignore them.

    data.gyroX = toInt16(buffer[8], buffer[9]);
    data.gyroY = toInt16(buffer[10], buffer[11]);
    data.gyroZ = toInt16(buffer[12], buffer[13]);

    return true;
}

bool MPU6050::calibrateGyroscope()
{
    int64_t sumX = 0;
    int64_t sumY = 0;
    int64_t sumZ = 0;

    int64_t sumSqX = 0;
    int64_t sumSqY = 0;
    int64_t sumSqZ = 0;

    Mpu6050RawData raw{};

    // Reset previous calibration values.
    _gyroBiasX_dps = 0.0f;
    _gyroBiasY_dps = 0.0f;
    _gyroBiasZ_dps = 0.0f;

    _calibrated = false;

    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        if (!readRaw(raw))
            return false;

        sumX += raw.gyroX;
        sumY += raw.gyroY;
        sumZ += raw.gyroZ;

        sumSqX += static_cast<int64_t>(raw.gyroX) * raw.gyroX;
        sumSqY += static_cast<int64_t>(raw.gyroY) * raw.gyroY;
        sumSqZ += static_cast<int64_t>(raw.gyroZ) * raw.gyroZ;

        delay(CALIBRATION_DELAY_MS);
    }

    const float count = static_cast<float>(CALIBRATION_SAMPLES);

    const float meanX = sumX / count;
    const float meanY = sumY / count;
    const float meanZ = sumZ / count;

    // Calculate standard deviation to detect movement during calibration.
    const float varianceX = (sumSqX / count) - (meanX * meanX);
    const float varianceY = (sumSqY / count) - (meanY * meanY);
    const float varianceZ = (sumSqZ / count) - (meanZ * meanZ);

    const float stdX = sqrtf(fmaxf(varianceX, 0.0f));
    const float stdY = sqrtf(fmaxf(varianceY, 0.0f));
    const float stdZ = sqrtf(fmaxf(varianceZ, 0.0f));

    // Reject calibration if the device moved significantly.
    if (stdX > MAX_GYRO_STDDEV_RAW ||
        stdY > MAX_GYRO_STDDEV_RAW ||
        stdZ > MAX_GYRO_STDDEV_RAW)
    {
        return false;
    }

    // Convert gyro bias from raw counts to degrees per second.
    _gyroBiasX_dps = meanX / GYRO_SCALE;
    _gyroBiasY_dps = meanY / GYRO_SCALE;
    _gyroBiasZ_dps = meanZ / GYRO_SCALE;

    _calibrated = true;

    return true;
}

bool MPU6050::readSensor(Mpu6050Data &data)
{
    if (!_calibrated)
        return false;

    Mpu6050RawData raw{};

    if (!readRaw(raw))
        return false;

    // Accelerometer conversion: +/-2g.
    data.accelX_g = raw.accelX / ACCEL_SCALE;
    data.accelY_g = raw.accelY / ACCEL_SCALE;
    data.accelZ_g = raw.accelZ / ACCEL_SCALE;

    // Gyroscope conversion and bias compensation.
    data.gyroX_dps = (raw.gyroX / GYRO_SCALE) - _gyroBiasX_dps;
    data.gyroY_dps = (raw.gyroY / GYRO_SCALE) - _gyroBiasY_dps;
    data.gyroZ_dps = (raw.gyroZ / GYRO_SCALE) - _gyroBiasZ_dps;

    data.timestamp_ms = millis();

    return true;
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(value);

    return (_wire.endTransmission() == 0);
}

bool MPU6050::readRegisters(
    uint8_t startReg,
    uint8_t *buffer,
    size_t length
)
{
    _wire.beginTransmission(_address);
    _wire.write(startReg);

    if (_wire.endTransmission(false) != 0)
        return false;

    size_t received = _wire.requestFrom(
        static_cast<uint8_t>(_address),
        static_cast<uint8_t>(length),
        static_cast<uint8_t>(true)
    );

    if (received != length)
    {
        while (_wire.available())
            _wire.read();

        return false;
    }

    for (size_t i = 0; i < length; i++)
    {
        buffer[i] = static_cast<uint8_t>(_wire.read());
    }

    return true;
}