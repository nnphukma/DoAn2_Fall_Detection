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
    // Raw data: Acceleration (g), Gyroscope (dps)
    float accelX_raw_g;
    float accelY_raw_g;
    float accelZ_raw_g;

    float gyroX_raw_dps;
    float gyroY_raw_dps;
    float gyroZ_raw_dps;

    // EMA filtered data
    float accelX_g;
    float accelY_g;
    float accelZ_g;

    float gyroX_dps;
    float gyroY_dps;
    float gyroZ_dps;

    // Motion magnitude
    float accelMagnitude_raw_g;
    float gyroMagnitude_raw_dps;

    float accelMagnitude_g;
    float gyroMagnitude_dps;

    // Mahony quaternion
    float q0;
    float q1;
    float q2;
    float q3;

    // Attitude angles
    float roll_deg;
    float pitch_deg;

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

    void updateEMA(Mpu6050Data &data);
    void updateMahony(Mpu6050Data &data, float dt);

    uint8_t _address;
    TwoWire &_wire;

    // Gyroscope calibration bias
    float _gyroBiasX_dps = 0.0f;
    float _gyroBiasY_dps = 0.0f;
    float _gyroBiasZ_dps = 0.0f;

    bool _calibrated = false;

    // EMA state
    bool _emaInitialized = false;

    float _emaAccelX = 0.0f;
    float _emaAccelY = 0.0f;
    float _emaAccelZ = 0.0f;

    float _emaGyroX = 0.0f;
    float _emaGyroY = 0.0f;
    float _emaGyroZ = 0.0f;

    // Mahony quaternion state
    float _q0 = 1.0f;
    float _q1 = 0.0f;
    float _q2 = 0.0f;
    float _q3 = 0.0f;

    // Timestamp for actual sampling interval
    uint32_t _lastUpdateMs = 0;
    bool _timestampInitialized = false;
};