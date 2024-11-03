#pragma once

#include <Arduino.h>

class Motor
{
public:
    // setup
    Motor(uint8_t pinA, uint8_t pinB, uint8_t pinEncoderA, uint8_t pinEncoderB);
    void setPulsesPerMillimeter(int32_t pulsesPerMillimeter) {_pulsesPerMillimeter = pulsesPerMillimeter;}
    void setTolerance(int32_t tolerance) {_tolerance = tolerance;}

    // movement
    void setTargetPosition(int32_t targetPosition);
    void move(int32_t distance) { setTargetPosition(_position + distance); }
    void update();
    void stop();

    // encoder
    int32_t getPosition() { return _position; }
    void resetPosition() { _position = 0; }

    // interrupts
    void IRAM_ATTR isr();

private:
    enum class State
    {
        TOO_LOW,
        TOO_HIGH,
        IN_RANGE
    };

    uint8_t _pinA;
    uint8_t _pinB;
    uint8_t _pinEncoderA;
    uint8_t _pinEncoderB;

    int32_t _position = 0;
    int32_t _targetPosition = 0;
    
    int32_t _pulsesPerMillimeter = 1;

    State _state = State::IN_RANGE;
    int32_t _tolerance = 100;
};