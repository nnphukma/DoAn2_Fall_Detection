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

    constexpr float MAX_GYRO_STDDEV_RAW = 300.0f;

    // EMA configuration
    // Sampling frequency: 100 Hz
    // Cutoff frequency: approximately 10 Hz
    constexpr float EMA_ALPHA = 0.4665f;

    // Mahony configuration
    constexpr float MAHONY_KP = 2.0f;
    constexpr float MAHONY_KI = 0.0f;

    constexpr float DEG_TO_RAD_FACTOR  = 0.01745329252f;
    constexpr float RAD_TO_DEG_FACTOR  = 57.29577951f;

    // Accept accelerometer feedback only when magnitude
    // is reasonably close to 1g.
    constexpr float ACCEL_MIN_VALID_G = 0.75f;
    constexpr float ACCEL_MAX_VALID_G = 1.25f;

    // Sampling interval limits
    constexpr float DEFAULT_DT = 0.01f;
    constexpr float MIN_DT = 0.005f;
    constexpr float MAX_DT = 0.03f;
}

// =====================================================
// Constructor
// =====================================================

MPU6050::MPU6050(uint8_t address, TwoWire &wire)
    : _address(address), _wire(wire)
{
}

// =====================================================
// Initialization
// =====================================================

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
    // Active HIGH, Push-pull, Latched interrupt.
    // Clear interrupt when any register is read.
    if (!writeRegister(REG_INT_PIN_CFG, 0x30))
        return false;

    if (!testConnection())
        return false;

    // Calibrate gyroscope while stationary.
    if (!calibrateGyroscope())
        return false;

    return true;
}

bool MPU6050::enableDataReadyInterrupt()
{
    return writeRegister(REG_INT_ENABLE, 0x01);
}

// =====================================================
// Connection test
// =====================================================

bool MPU6050::testConnection()
{
    uint8_t whoAmI = 0;

    if (!readRegisters(REG_WHO_AM_I, &whoAmI, 1))
        return false;

    return (whoAmI == 0x68 || whoAmI == 0x74);
}

// =====================================================
// Read raw sensor data
// =====================================================

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

    // Temperature bytes are intentionally ignored.

    data.gyroX = toInt16(buffer[8], buffer[9]);
    data.gyroY = toInt16(buffer[10], buffer[11]);
    data.gyroZ = toInt16(buffer[12], buffer[13]);

    return true;
}

// =====================================================
// Gyroscope calibration
// =====================================================

bool MPU6050::calibrateGyroscope()
{
    int64_t sumX = 0;
    int64_t sumY = 0;
    int64_t sumZ = 0;

    int64_t sumSqX = 0;
    int64_t sumSqY = 0;
    int64_t sumSqZ = 0;

    Mpu6050RawData raw{};

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

    const float varianceX = (sumSqX / count) - (meanX * meanX);
    const float varianceY = (sumSqY / count) - (meanY * meanY);
    const float varianceZ = (sumSqZ / count) - (meanZ * meanZ);

    const float stdX = sqrtf(fmaxf(varianceX, 0.0f));
    const float stdY = sqrtf(fmaxf(varianceY, 0.0f));
    const float stdZ = sqrtf(fmaxf(varianceZ, 0.0f));

    if (stdX > MAX_GYRO_STDDEV_RAW ||
        stdY > MAX_GYRO_STDDEV_RAW ||
        stdZ > MAX_GYRO_STDDEV_RAW)
    {
        return false;
    }

    _gyroBiasX_dps = meanX / GYRO_SCALE;
    _gyroBiasY_dps = meanY / GYRO_SCALE;
    _gyroBiasZ_dps = meanZ / GYRO_SCALE;

    _calibrated = true;

    return true;
}

// =====================================================
// EMA Low-pass Filter
// =====================================================

void MPU6050::updateEMA(Mpu6050Data &data)
{
    // Initialize EMA with the first sensor sample.
    if (!_emaInitialized)
    {
        _emaAccelX = data.accelX_raw_g;
        _emaAccelY = data.accelY_raw_g;
        _emaAccelZ = data.accelZ_raw_g;

        _emaGyroX = data.gyroX_raw_dps;
        _emaGyroY = data.gyroY_raw_dps;
        _emaGyroZ = data.gyroZ_raw_dps;

        _emaInitialized = true;
    }
    else
    {
        // Accelerometer
        _emaAccelX += EMA_ALPHA * (data.accelX_raw_g - _emaAccelX);
        _emaAccelY += EMA_ALPHA * (data.accelY_raw_g - _emaAccelY);
        _emaAccelZ += EMA_ALPHA * (data.accelZ_raw_g - _emaAccelZ);

        // Gyroscope
        _emaGyroX += EMA_ALPHA * (data.gyroX_raw_dps - _emaGyroX);
        _emaGyroY += EMA_ALPHA * (data.gyroY_raw_dps - _emaGyroY);
        _emaGyroZ += EMA_ALPHA * (data.gyroZ_raw_dps - _emaGyroZ);
    }

    // Store filtered values.
    data.accelX_g = _emaAccelX;
    data.accelY_g = _emaAccelY;
    data.accelZ_g = _emaAccelZ;

    data.gyroX_dps = _emaGyroX;
    data.gyroY_dps = _emaGyroY;
    data.gyroZ_dps = _emaGyroZ;
}

// =====================================================
// Mahony AHRS
// =====================================================

void MPU6050::updateMahony(Mpu6050Data &data, float dt)
{
    float ax = data.accelX_g;
    float ay = data.accelY_g;
    float az = data.accelZ_g;

    // Convert gyro from degrees/s to radians/s.
    float gx = data.gyroX_dps * DEG_TO_RAD_FACTOR ;
    float gy = data.gyroY_dps * DEG_TO_RAD_FACTOR ;
    float gz = data.gyroZ_dps * DEG_TO_RAD_FACTOR ;

    // Accelerometer magnitude.
    const float accelNorm = sqrtf(ax * ax + ay * ay + az * az);

    // Use accelerometer correction only when its magnitude
    // is close to gravity (1g).
    if (accelNorm >= ACCEL_MIN_VALID_G &&
        accelNorm <= ACCEL_MAX_VALID_G)
    {
        // Normalize accelerometer.
        ax /= accelNorm;
        ay /= accelNorm;
        az /= accelNorm;

        // Estimated gravity direction.
        const float halfvx =
            _q1 * _q3 - _q0 * _q2;

        const float halfvy =
            _q0 * _q1 + _q2 * _q3;

        const float halfvz =
            _q0 * _q0 - 0.5f + _q3 * _q3;

        // Error between estimated and measured gravity.
        const float halfex =
            ay * halfvz - az * halfvy;

        const float halfey =
            az * halfvx - ax * halfvz;

        const float halfez =
            ax * halfvy - ay * halfvx;

        // Proportional feedback.
        gx += MAHONY_KP * halfex;
        gy += MAHONY_KP * halfey;
        gz += MAHONY_KP * halfez;

        // Integral feedback is disabled (Ki = 0).
        if (MAHONY_KI > 0.0f)
        {
            // Reserved for future integral correction.
        }
    }

    // Quaternion derivative.
    const float qDot0 = 0.5f *
        (-_q1 * gx - _q2 * gy - _q3 * gz);

    const float qDot1 = 0.5f *
        (_q0 * gx + _q2 * gz - _q3 * gy);

    const float qDot2 = 0.5f *
        (_q0 * gy - _q1 * gz + _q3 * gx);

    const float qDot3 = 0.5f *
        (_q0 * gz + _q1 * gy - _q2 * gx);

    // Integrate quaternion.
    _q0 += qDot0 * dt;
    _q1 += qDot1 * dt;
    _q2 += qDot2 * dt;
    _q3 += qDot3 * dt;

    // Normalize quaternion.
    const float qNorm = sqrtf(
        _q0 * _q0 +
        _q1 * _q1 +
        _q2 * _q2 +
        _q3 * _q3
    );

    if (qNorm > 0.0f)
    {
        _q0 /= qNorm;
        _q1 /= qNorm;
        _q2 /= qNorm;
        _q3 /= qNorm;
    }

    // Calculate Roll.
    const float sinRoll =
        2.0f * (_q0 * _q1 + _q2 * _q3);

    const float cosRoll =
        1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2);

    const float roll = atan2f(sinRoll, cosRoll);

    // Calculate Pitch.
    float sinPitch =
        2.0f * (_q0 * _q2 - _q3 * _q1);

    // Clamp input to asin range.
    sinPitch = fmaxf(-1.0f, fminf(1.0f, sinPitch));

    const float pitch = asinf(sinPitch);

    // Export quaternion and attitude.
    data.q0 = _q0;
    data.q1 = _q1;
    data.q2 = _q2;
    data.q3 = _q3;

    data.roll_deg = roll * RAD_TO_DEG_FACTOR ;
    data.pitch_deg = pitch * RAD_TO_DEG_FACTOR ;
}

// =====================================================
// Read and process sensor data
// =====================================================

bool MPU6050::readSensor(Mpu6050Data &data)
{
    if (!_calibrated)
        return false;

    Mpu6050RawData raw{};

    if (!readRaw(raw))
        return false;

    // -------------------------------------------------
    // 1. Convert raw data to physical units
    // -------------------------------------------------

    data.accelX_raw_g = raw.accelX / ACCEL_SCALE;
    data.accelY_raw_g = raw.accelY / ACCEL_SCALE;
    data.accelZ_raw_g = raw.accelZ / ACCEL_SCALE;

    data.gyroX_raw_dps =
        (raw.gyroX / GYRO_SCALE) - _gyroBiasX_dps;

    data.gyroY_raw_dps =
        (raw.gyroY / GYRO_SCALE) - _gyroBiasY_dps;

    data.gyroZ_raw_dps =
        (raw.gyroZ / GYRO_SCALE) - _gyroBiasZ_dps;

    // -------------------------------------------------
    // 2. Raw motion magnitude
    // -------------------------------------------------

    data.accelMagnitude_raw_g = sqrtf(
        data.accelX_raw_g * data.accelX_raw_g +
        data.accelY_raw_g * data.accelY_raw_g +
        data.accelZ_raw_g * data.accelZ_raw_g
    );

    data.gyroMagnitude_raw_dps = sqrtf(
        data.gyroX_raw_dps * data.gyroX_raw_dps +
        data.gyroY_raw_dps * data.gyroY_raw_dps +
        data.gyroZ_raw_dps * data.gyroZ_raw_dps
    );

    // -------------------------------------------------
    // 3. EMA filtering
    // -------------------------------------------------

    updateEMA(data);

    // -------------------------------------------------
    // 4. Filtered motion magnitude
    // -------------------------------------------------

    data.accelMagnitude_g = sqrtf(
        data.accelX_g * data.accelX_g +
        data.accelY_g * data.accelY_g +
        data.accelZ_g * data.accelZ_g
    );

    data.gyroMagnitude_dps = sqrtf(
        data.gyroX_dps * data.gyroX_dps +
        data.gyroY_dps * data.gyroY_dps +
        data.gyroZ_dps * data.gyroZ_dps
    );

    // -------------------------------------------------
    // 5. Calculate sampling interval
    // -------------------------------------------------

    const uint32_t now = millis();

    float dt = DEFAULT_DT;

    if (_timestampInitialized)
    {
        dt = (now - _lastUpdateMs) / 1000.0f;

        if (dt < MIN_DT || dt > MAX_DT)
            dt = DEFAULT_DT;
    }

    _lastUpdateMs = now;
    _timestampInitialized = true;

    // -------------------------------------------------
    // 6. Mahony sensor fusion
    // -------------------------------------------------

    updateMahony(data, dt);

    data.timestamp_ms = now;

    return true;
}

// =====================================================
// Register write
// =====================================================

bool MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(value);

    return (_wire.endTransmission() == 0);
}

// =====================================================
// Register read
// =====================================================

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