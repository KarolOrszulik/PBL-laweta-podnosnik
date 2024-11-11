#pragma once

#include <Arduino.h>

class Motor
{
public:
    // setup
    Motor(uint8_t pinMotorA, uint8_t pinMotorB, uint8_t pinEncoderA, uint8_t pinEncoderB);
    void setPulsesPerMillimeter(int32_t pulsesPerMillimeter) {_pulsesPerMillimeter = pulsesPerMillimeter;}
    void setPositionTolerance(float tolerance) {_pulsesTolerance = positionToPulses(tolerance);}

    // movement
    void setTargetPosition(float targetPosition);
    void changeTargetPosition(float distance);
    void update(); // TODO: asynchronous
    void stop();
    void home();
    bool isHomed() const { return _isHomed; }

    // encoder
    bool isInRange() const { return _state == State::IN_RANGE; }
    float getPosition() const { return pulsesToPosition(_pulses); }
    int32_t getPulses() const { return _pulses; }
    int32_t getTargetPulses() const { return _targetPulses; }

    // interrupts
    void IRAM_ATTR isr();

private:
    static constexpr float MIN_POSITION = 1.0f;

    enum class State
    {
        TOO_LOW,
        TOO_HIGH,
        IN_RANGE
    };

    void detectDirection();
    void moveForward();
    void moveBackward();
    void resetPosition();

    int32_t getPulsesError() const
        { return _pulses - _targetPulses; }

    int32_t positionToPulses(float position) const
        { return position * _pulsesPerMillimeter; }

    float pulsesToPosition(int32_t pulses) const
        { return (float)pulses / _pulsesPerMillimeter; }

    uint8_t _pinMotorA;
    uint8_t _pinMotorB;
    uint8_t _pinEncoderA;
    uint8_t _pinEncoderB;

    bool _isHomed = false;
    bool _shouldUpdate = false;

    volatile int32_t _pulses = 0;
    int32_t _targetPulses = 0;
    
    int32_t _pulsesPerMillimeter = 1;
    int32_t _pulsesTolerance = 10;

    State _state = State::IN_RANGE;
};