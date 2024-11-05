#pragma once

#include <Arduino.h>

class Motor
{
public:
    // setup
    Motor(uint8_t pinA, uint8_t pinB, uint8_t pinEncoderA, uint8_t pinEncoderB);
    void setPulsesPerMillimeter(int32_t pulsesPerMillimeter) {_pulsesPerMillimeter = pulsesPerMillimeter;}
    void setPositionTolerance(float tolerance) {_pulsesTolerance = positionToPulses(tolerance);}

    // movement
    void setTargetPosition(float targetPosition);
    void move(float distance);
    void update();
    void stop();

    // encoder
    bool isInRange() { return _state == State::IN_RANGE; }
    float getPosition() { return pulsesToPosition(_pulses); }
    int32_t getPulses() { return _pulses; }
    void resetPosition()  { _pulses = 0; }

    int32_t getTargetPulses() { return _targetPulses; }

    // interrupts
    void IRAM_ATTR isr();

private:
    enum class State
    {
        TOO_LOW,
        TOO_HIGH,
        IN_RANGE
    };

    int32_t positionToPulses(float position) { return position * _pulsesPerMillimeter; }
    float pulsesToPosition(int32_t pulses) { return (float)pulses / _pulsesPerMillimeter; }
    int32_t getPulsesError() { return _targetPulses - _pulses; }

    uint8_t _pinA;
    uint8_t _pinB;
    uint8_t _pinEncoderA;
    uint8_t _pinEncoderB;

    volatile int32_t _pulses = 0;
    int32_t _targetPulses = 0;
    
    int32_t _pulsesPerMillimeter = 1;

    State _state = State::IN_RANGE;
    int32_t _pulsesTolerance = 10;
};