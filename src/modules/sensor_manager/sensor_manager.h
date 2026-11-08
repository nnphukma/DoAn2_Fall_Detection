#pragma once

#include "mpu6050.h"

class SensorManager
{
public:
    bool begin();

    bool enableDataReadyInterrupt();

    bool update(Mpu6050Data &data);

    bool isReady() const;

private:
    MPU6050 _mpu;
    bool _ready = false;
};