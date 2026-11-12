#pragma once

#include <Arduino.h>
#include "mpu6050.h"

enum class FallState : uint8_t
{
    CALIBRATING,
    NORMAL_MONITOR,
    EVENT_CANDIDATE,
    POSTURE_VERIFY,
    INACTIVITY_VERIFY,
    ALARM_COUNTDOWN,
    FALL_CONFIRMED,
    MANUAL_SOS
};

enum class FallEventType : uint8_t
{
    NONE,
    BASELINE_READY,
    
    BTN_SOS_PRESSED,
    BTN_CANCEL_PRESSED,

    CANDIDATE_STARTED,
    POSTURE_CONFIRMED,
    COUNTDOWN_STARTED,
    FALL_DETECTED,
    MANUAL_SOS_TRIGGERED,
    ALERT_CANCELLED,
    CANDIDATE_REJECTED
};

struct FallDetectorResult
{
    FallState state = FallState::CALIBRATING;
    FallEventType event = FallEventType::NONE;

    uint32_t timestamp_ms = 0;

    float postureAngle_deg = 0.0f;
    float accelMagnitude_raw_g = 0.0f;
    float gyroMagnitude_raw_dps = 0.0f;

    uint32_t countdownRemaining_ms = 0;

    // Read-only diagnostic values exported for the BLE data logger. They do
    // not feed back into, or otherwise alter, the frozen FSM.
    bool postureValid = false;
    bool freeFallCondition = false;
    bool freeFallTrigger = false;
    bool impactTrigger = false;
    bool rotationTrigger = false;
    bool postureCondition = false;
    bool postureConfirmed = false;
    bool stillnessCondition = false;
    uint32_t stateElapsed_ms = 0;
    uint32_t postureElapsed_ms = 0;
    uint32_t stillnessElapsed_ms = 0;

    bool buzzerOn = false;
    bool fallLedOn = false;
};

class FallDetector
{
public:
    void begin();

    FallDetectorResult update(
        const Mpu6050Data &sample,
        bool sosButtonPressed,
        bool cancelButtonPressed
    );

private:
    static bool updateButtonDebounce(
        bool rawPressed,
        uint8_t &counter,
        bool &stableState
    );

    void resetCandidate();
    void resetToNormal();
    void updateStartupBaseline(const Mpu6050Data &sample);
    void slowlyUpdateBaseline(const Mpu6050Data &sample, float postureAngle_deg);
    bool calculatePostureAngle(
        const Mpu6050Data &sample,
        float &angle_deg
    ) const;

    FallDetectorResult makeResult(
        const Mpu6050Data &sample,
        FallEventType event,
        float postureAngle_deg
    ) const;

    FallState _state = FallState::CALIBRATING;

    // Baseline gravity vector captured while the device is stationary
    // and worn in its normal upright position at startup.
    bool _baselineReady = false;
    float _baselineX = 0.0f;
    float _baselineY = 0.0f;
    float _baselineZ = 0.0f;

    float _calibrationSumX = 0.0f;
    float _calibrationSumY = 0.0f;
    float _calibrationSumZ = 0.0f;
    uint16_t _calibrationCount = 0;

    // State timers
    uint32_t _stateStart_ms = 0;
    uint32_t _freeFallStart_ms = 0;
    uint32_t _postureStart_ms = 0;
    uint32_t _stillnessStart_ms = 0;
    uint32_t _uprightStart_ms = 0;

    bool _freeFallTiming = false;
    bool _postureTiming = false;
    bool _stillnessTiming = false;
    bool _uprightTiming = false;

    uint32_t _countdownStart_ms = 0;

    // 3-sample button debounce at approximately 100 Hz
    uint8_t _sosDebounceCount = 0;
    uint8_t _cancelDebounceCount = 0;
    bool _sosStable = false;
    bool _cancelStable = false;

    bool _diagPostureValid = false;
    bool _diagFreeFallCondition = false;
    bool _diagFreeFallTrigger = false;
    bool _diagImpactTrigger = false;
    bool _diagRotationTrigger = false;
    bool _diagPostureCondition = false;
    bool _diagPostureConfirmed = false;
    bool _diagStillnessCondition = false;
};

const char *fallStateName(FallState state);
const char *fallEventName(FallEventType event);
