#include "sensor_manager.h"
#include "config.h"

#include <Wire.h>

bool SensorManager::begin()
{
    // Configure MPU6050 INT pin.
    pinMode(Config::MPU_INT_PIN, INPUT);

    // Initialize I2C.
    Wire.begin(
        Config::MPU_SDA_PIN,
        Config::MPU_SCL_PIN
    );

    Wire.setClock(Config::I2C_CLOCK_HZ);

    // Initialize MPU6050 and perform gyro calibration.
    _ready = _mpu.begin();

    return _ready;
}

bool SensorManager::enableDataReadyInterrupt()
{
    if (!_ready)
        return false;

    return _mpu.enableDataReadyInterrupt();
}

bool SensorManager::update(Mpu6050Data &data)
{
    if (!_ready)
        return false;

    return _mpu.readSensor(data);
}

bool SensorManager::isReady() const
{
    return _ready;
}