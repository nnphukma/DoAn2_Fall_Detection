#include "fall_detector.h"

#include <math.h>

namespace
{
    // Candidate trigger thresholds.
    // The accelerometer must be configured for +/-4g in mpu6050.cpp.
    constexpr float FREE_FALL_THRESHOLD_G = 0.55f;
    constexpr uint32_t FREE_FALL_MIN_DURATION_MS = 80;

    constexpr float IMPACT_THRESHOLD_G = 1.80f;
    constexpr float ROTATION_THRESHOLD_DPS = 200.0f;

    // Posture verification.
    // The T1-T2-T3 test reached about 52 degrees during normal side leaning
    // and about 45 degrees during forward/backward bending.
    constexpr float POSTURE_ANGLE_THRESHOLD_DEG = 60.0f;
    constexpr uint32_t POSTURE_HOLD_MS = 1000;
    constexpr uint32_t POSTURE_VERIFY_TIMEOUT_MS = 3000;

    // Inactivity verification.
    constexpr float STILL_GYRO_MAX_DPS = 20.0f;
    constexpr float STILL_ACCEL_MIN_G = 0.85f;
    constexpr float STILL_ACCEL_MAX_G = 1.15f;
    constexpr uint32_t STILLNESS_CONFIRM_MS = 2000;
    constexpr uint32_t INACTIVITY_VERIFY_TIMEOUT_MS = 6000;

    // Hysteresis for posture recovery.
    constexpr float RECOVERY_ANGLE_DEG = 50.0f;
    constexpr uint32_t RECOVERY_HOLD_MS = 300;

    constexpr uint32_t ALARM_COUNTDOWN_MS = 10000;

    // Startup baseline capture.
    constexpr uint16_t BASELINE_SAMPLES = 100; // approximately 1 second
    constexpr float BASELINE_ACCEL_MIN_G = 0.85f;
    constexpr float BASELINE_ACCEL_MAX_G = 1.15f;
    constexpr float BASELINE_GYRO_MAX_DPS = 15.0f;

    // Slow baseline adaptation only while close to the original upright pose.
    constexpr float BASELINE_ADAPT_ANGLE_DEG = 20.0f;
    constexpr float BASELINE_ADAPT_ALPHA = 0.002f;

    constexpr float POSTURE_ACCEL_MIN_G = 0.80f;
    constexpr float POSTURE_ACCEL_MAX_G = 1.20f;

    constexpr uint8_t BUTTON_DEBOUNCE_SAMPLES = 3;
}

void FallDetector::begin()
{
    _state = FallState::CALIBRATING;

    _baselineReady = false;
    _baselineX = _baselineY = _baselineZ = 0.0f;

    _calibrationSumX = 0.0f;
    _calibrationSumY = 0.0f;
    _calibrationSumZ = 0.0f;
    _calibrationCount = 0;

    _stateStart_ms = 0;
    _countdownStart_ms = 0;

    _freeFallTiming = false;
    _postureTiming = false;
    _stillnessTiming = false;
    _uprightTiming = false;

    _sosDebounceCount = 0;
    _cancelDebounceCount = 0;
    _sosStable = false;
    _cancelStable = false;
}

bool FallDetector::updateButtonDebounce(
    bool rawPressed,
    uint8_t &counter,
    bool &stableState)
{
    const bool previousState = stableState;

    if (rawPressed)
    {
        if (counter < BUTTON_DEBOUNCE_SAMPLES)
            ++counter;
    }
    else
    {
        if (counter > 0)
            --counter;
    }

    if (counter >= BUTTON_DEBOUNCE_SAMPLES)
        stableState = true;
    else if (counter == 0)
        stableState = false;

    return stableState && !previousState;
}

void FallDetector::resetCandidate()
{
    _freeFallTiming = false;
    _postureTiming = false;
    _stillnessTiming = false;
    _uprightTiming = false;

    _freeFallStart_ms = 0;
    _postureStart_ms = 0;
    _stillnessStart_ms = 0;
    _uprightStart_ms = 0;
}

void FallDetector::resetToNormal()
{
    resetCandidate();
    _state = _baselineReady
        ? FallState::NORMAL_MONITOR
        : FallState::CALIBRATING;
}

void FallDetector::updateStartupBaseline(const Mpu6050Data &sample)
{
    const bool stationary =
        sample.accelMagnitude_raw_g >= BASELINE_ACCEL_MIN_G &&
        sample.accelMagnitude_raw_g <= BASELINE_ACCEL_MAX_G &&
        sample.gyroMagnitude_raw_dps <= BASELINE_GYRO_MAX_DPS;

    if (!stationary)
    {
        _calibrationSumX = 0.0f;
        _calibrationSumY = 0.0f;
        _calibrationSumZ = 0.0f;
        _calibrationCount = 0;
        return;
    }

    _calibrationSumX += sample.accelX_g;
    _calibrationSumY += sample.accelY_g;
    _calibrationSumZ += sample.accelZ_g;
    ++_calibrationCount;

    if (_calibrationCount < BASELINE_SAMPLES)
        return;

    const float norm = sqrtf(
        _calibrationSumX * _calibrationSumX +
        _calibrationSumY * _calibrationSumY +
        _calibrationSumZ * _calibrationSumZ
    );

    if (norm < 0.001f)
    {
        _calibrationSumX = 0.0f;
        _calibrationSumY = 0.0f;
        _calibrationSumZ = 0.0f;
        _calibrationCount = 0;
        return;
    }

    _baselineX = _calibrationSumX / norm;
    _baselineY = _calibrationSumY / norm;
    _baselineZ = _calibrationSumZ / norm;

    _baselineReady = true;
    _state = FallState::NORMAL_MONITOR;
    _stateStart_ms = sample.timestamp_ms;
}

bool FallDetector::calculatePostureAngle(
    const Mpu6050Data &sample,
    float &angle_deg) const
{
    if (!_baselineReady)
        return false;

    const float ax = sample.accelX_g;
    const float ay = sample.accelY_g;
    const float az = sample.accelZ_g;

    const float norm = sqrtf(ax * ax + ay * ay + az * az);

    // Only use the accelerometer direction when gravity dominates.
    if (norm < POSTURE_ACCEL_MIN_G || norm > POSTURE_ACCEL_MAX_G)
        return false;

    const float ux = ax / norm;
    const float uy = ay / norm;
    const float uz = az / norm;

    float dot =
        _baselineX * ux +
        _baselineY * uy +
        _baselineZ * uz;

    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;

    angle_deg = acosf(dot) * 57.29577951f;
    return true;
}

void FallDetector::slowlyUpdateBaseline(
    const Mpu6050Data &sample,
    float postureAngle_deg)
{
    const bool stationary =
        sample.accelMagnitude_raw_g >= BASELINE_ACCEL_MIN_G &&
        sample.accelMagnitude_raw_g <= BASELINE_ACCEL_MAX_G &&
        sample.gyroMagnitude_raw_dps <= BASELINE_GYRO_MAX_DPS;

    if (!stationary || postureAngle_deg > BASELINE_ADAPT_ANGLE_DEG)
        return;

    _baselineX += BASELINE_ADAPT_ALPHA * (sample.accelX_g - _baselineX);
    _baselineY += BASELINE_ADAPT_ALPHA * (sample.accelY_g - _baselineY);
    _baselineZ += BASELINE_ADAPT_ALPHA * (sample.accelZ_g - _baselineZ);

    const float norm = sqrtf(
        _baselineX * _baselineX +
        _baselineY * _baselineY +
        _baselineZ * _baselineZ
    );

    if (norm > 0.001f)
    {
        _baselineX /= norm;
        _baselineY /= norm;
        _baselineZ /= norm;
    }
}

FallDetectorResult FallDetector::makeResult(
    const Mpu6050Data &sample,
    FallEventType event,
    float postureAngle_deg) const
{
    FallDetectorResult result{};

    result.state = _state;
    result.event = event;
    result.timestamp_ms = sample.timestamp_ms;
    result.postureAngle_deg = postureAngle_deg;
    result.accelMagnitude_raw_g = sample.accelMagnitude_raw_g;
    result.gyroMagnitude_raw_dps = sample.gyroMagnitude_raw_dps;

    if (_state == FallState::ALARM_COUNTDOWN)
    {
        const uint32_t elapsed = sample.timestamp_ms - _countdownStart_ms;
        result.countdownRemaining_ms =
            (elapsed >= ALARM_COUNTDOWN_MS)
                ? 0
                : (ALARM_COUNTDOWN_MS - elapsed);
    }

    result.buzzerOn =
        (_state == FallState::ALARM_COUNTDOWN) ||
        (_state == FallState::FALL_CONFIRMED) ||
        (_state == FallState::MANUAL_SOS);

    result.fallLedOn = result.buzzerOn;

    return result;
}

FallDetectorResult FallDetector::update(
    const Mpu6050Data &sample,
    bool sosButtonPressed,
    bool cancelButtonPressed)
{
    const uint32_t now = sample.timestamp_ms;

    const bool sosEdge = updateButtonDebounce(
        sosButtonPressed,
        _sosDebounceCount,
        _sosStable
    );

    const bool cancelEdge = updateButtonDebounce(
        cancelButtonPressed,
        _cancelDebounceCount,
        _cancelStable
    );

    float postureAngle = 0.0f;
    const bool postureValid = calculatePostureAngle(sample, postureAngle);

    FallEventType event = FallEventType::NONE;

    // Cancel has priority if both buttons are pressed at the same time.
    if (cancelEdge &&
        (_state == FallState::EVENT_CANDIDATE ||
         _state == FallState::POSTURE_VERIFY ||
         _state == FallState::INACTIVITY_VERIFY ||
         _state == FallState::ALARM_COUNTDOWN ||
         _state == FallState::FALL_CONFIRMED ||
         _state == FallState::MANUAL_SOS))
    {
        resetToNormal();
        event = FallEventType::ALERT_CANCELLED;
    }
    else if (sosEdge &&
             _state != FallState::MANUAL_SOS &&
             _state != FallState::FALL_CONFIRMED)
    {
        resetCandidate();
        _state = FallState::MANUAL_SOS;
        event = FallEventType::MANUAL_SOS_TRIGGERED;
    }
    else
    {
        switch (_state)
        {
            case FallState::CALIBRATING:
            {
                updateStartupBaseline(sample);

                if (_baselineReady)
                    event = FallEventType::BASELINE_READY;

                break;
            }

            case FallState::NORMAL_MONITOR:
            {
                if (postureValid)
                    slowlyUpdateBaseline(sample, postureAngle);

                // Free-fall must persist for at least 80 ms.
                if (sample.accelMagnitude_raw_g < FREE_FALL_THRESHOLD_G)
                {
                    if (!_freeFallTiming)
                    {
                        _freeFallTiming = true;
                        _freeFallStart_ms = now;
                    }
                }
                else if (!_freeFallTiming ||
                         (now - _freeFallStart_ms) < FREE_FALL_MIN_DURATION_MS)
                {
                    _freeFallTiming = false;
                }

                const bool freeFallDetected =
                    _freeFallTiming &&
                    ((now - _freeFallStart_ms) >= FREE_FALL_MIN_DURATION_MS);

                const bool impactDetected =
                    sample.accelMagnitude_raw_g >= IMPACT_THRESHOLD_G;

                const bool rapidRotationDetected =
                    sample.gyroMagnitude_raw_dps >= ROTATION_THRESHOLD_DPS;

                if (freeFallDetected || impactDetected || rapidRotationDetected)
                {
                    _state = FallState::EVENT_CANDIDATE;
                    _stateStart_ms = now;
                    resetCandidate();
                    event = FallEventType::CANDIDATE_STARTED;
                }

                break;
            }

            case FallState::EVENT_CANDIDATE:
            {
                _state = FallState::POSTURE_VERIFY;
                _stateStart_ms = now;
                _postureTiming = false;
                break;
            }

            case FallState::POSTURE_VERIFY:
            {
                if (postureValid &&
                    postureAngle >= POSTURE_ANGLE_THRESHOLD_DEG)
                {
                    if (!_postureTiming)
                    {
                        _postureTiming = true;
                        _postureStart_ms = now;
                    }

                    if ((now - _postureStart_ms) >= POSTURE_HOLD_MS)
                    {
                        _state = FallState::INACTIVITY_VERIFY;
                        _stateStart_ms = now;
                        _stillnessTiming = false;
                        _uprightTiming = false;
                        event = FallEventType::POSTURE_CONFIRMED;
                    }
                }
                else
                {
                    _postureTiming = false;
                }

                if (_state == FallState::POSTURE_VERIFY &&
                    (now - _stateStart_ms) >= POSTURE_VERIFY_TIMEOUT_MS)
                {
                    resetToNormal();
                    event = FallEventType::CANDIDATE_REJECTED;
                }

                break;
            }

            case FallState::INACTIVITY_VERIFY:
            {
                const bool postureMaintained =
                    postureValid && postureAngle >= 55.0f;

                if (postureValid && postureAngle < RECOVERY_ANGLE_DEG)
                {
                    if (!_uprightTiming)
                    {
                        _uprightTiming = true;
                        _uprightStart_ms = now;
                    }
                }
                else
                {
                    _uprightTiming = false;
                }

                if (_uprightTiming &&
                    (now - _uprightStart_ms) >= RECOVERY_HOLD_MS)
                {
                    resetToNormal();
                    event = FallEventType::CANDIDATE_REJECTED;
                    break;
                }

                const bool still =
                    postureMaintained &&
                    sample.gyroMagnitude_raw_dps < STILL_GYRO_MAX_DPS &&
                    sample.accelMagnitude_raw_g >= STILL_ACCEL_MIN_G &&
                    sample.accelMagnitude_raw_g <= STILL_ACCEL_MAX_G;

                if (still)
                {
                    if (!_stillnessTiming)
                    {
                        _stillnessTiming = true;
                        _stillnessStart_ms = now;
                    }

                    if ((now - _stillnessStart_ms) >= STILLNESS_CONFIRM_MS)
                    {
                        _state = FallState::ALARM_COUNTDOWN;
                        _countdownStart_ms = now;
                        event = FallEventType::COUNTDOWN_STARTED;
                    }
                }
                else
                {
                    _stillnessTiming = false;
                }

                if (_state == FallState::INACTIVITY_VERIFY &&
                    (now - _stateStart_ms) >= INACTIVITY_VERIFY_TIMEOUT_MS)
                {
                    resetToNormal();
                    event = FallEventType::CANDIDATE_REJECTED;
                }

                break;
            }

            case FallState::ALARM_COUNTDOWN:
            {
                if ((now - _countdownStart_ms) >= ALARM_COUNTDOWN_MS)
                {
                    _state = FallState::FALL_CONFIRMED;
                    event = FallEventType::FALL_DETECTED;
                }

                break;
            }

            case FallState::FALL_CONFIRMED:
            case FallState::MANUAL_SOS:
            default:
                // Latched until BTN02 is pressed.
                break;
        }
    }

    return makeResult(sample, event, postureAngle);
}

const char *fallStateName(FallState state)
{
    switch (state)
    {
        case FallState::CALIBRATING:       return "CALIBRATING";
        case FallState::NORMAL_MONITOR:    return "NORMAL_MONITOR";
        case FallState::EVENT_CANDIDATE:   return "EVENT_CANDIDATE";
        case FallState::POSTURE_VERIFY:    return "POSTURE_VERIFY";
        case FallState::INACTIVITY_VERIFY: return "INACTIVITY_VERIFY";
        case FallState::ALARM_COUNTDOWN:   return "ALARM_COUNTDOWN";
        case FallState::FALL_CONFIRMED:    return "FALL_CONFIRMED";
        case FallState::MANUAL_SOS:        return "MANUAL_SOS";
        default:                          return "UNKNOWN";
    }
}

const char *fallEventName(FallEventType event)
{
    switch (event)
    {
        case FallEventType::NONE:                  return "NONE";
        case FallEventType::BASELINE_READY:        return "BASELINE_READY";
        case FallEventType::CANDIDATE_STARTED:     return "CANDIDATE_STARTED";
        case FallEventType::POSTURE_CONFIRMED:     return "POSTURE_CONFIRMED";
        case FallEventType::COUNTDOWN_STARTED:     return "COUNTDOWN_STARTED";
        case FallEventType::FALL_DETECTED:         return "FALL_DETECTED";
        case FallEventType::MANUAL_SOS_TRIGGERED:  return "MANUAL_SOS";
        case FallEventType::ALERT_CANCELLED:       return "ALERT_CANCELLED";
        case FallEventType::CANDIDATE_REJECTED:    return "CANDIDATE_REJECTED";
        default:                                   return "UNKNOWN";
    }
}
