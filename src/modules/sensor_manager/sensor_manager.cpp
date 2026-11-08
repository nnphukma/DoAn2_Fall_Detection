#include "sensor_manager.h"
#include "config.h"
#include <Wire.h>

bool SensorManager::begin() {
    Wire.begin(Config::MPU_SDA_PIN, Config::MPU_SCL_PIN);
    Wire.setClock(Config::I2C_CLOCK_HZ);

    _ready = _mpu.begin();
    return _ready;
}

bool SensorManager::update(Mpu6050Data &data) {
    if (!_ready) return false;
    return _mpu.readSensor(data);
}

bool SensorManager::isReady() const {
    return _ready;
}
