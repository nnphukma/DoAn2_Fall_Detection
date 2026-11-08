#pragma once

#include <Arduino.h>
#include <Wire.h>

struct Mpu6050RawData
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;

    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
};

struct Mpu6050Data
{
    float accelX_g;
    float accelY_g;
    float accelZ_g;

    float gyroX_dps;
    float gyroY_dps;
    float gyroZ_dps;

    uint32_t timestamp_ms;
};

class MPU6050
{
public:
    explicit MPU6050(uint8_t address = 0x68, TwoWire &wire = Wire);

    bool begin();
    bool enableDataReadyInterrupt();

    bool testConnection();
    bool readRaw(Mpu6050RawData &data);
    bool readSensor(Mpu6050Data &data);

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t startReg, uint8_t *buffer, size_t length);

    bool calibrateGyroscope();

    uint8_t _address;
    TwoWire &_wire;

    float _gyroBiasX_dps = 0.0f;
    float _gyroBiasY_dps = 0.0f;
    float _gyroBiasZ_dps = 0.0f;

    bool _calibrated = false;
};