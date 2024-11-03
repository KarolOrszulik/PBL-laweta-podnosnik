#include "Motor.h"


Motor::Motor(uint8_t pinA, uint8_t pinB, uint8_t pinEncoderA, uint8_t pinEncoderB)
    : _pinA(pinA), _pinB(pinB), _pinEncoderA(pinEncoderA), _pinEncoderB(pinEncoderB)
{
    pinMode(_pinA, OUTPUT);
    pinMode(_pinB, OUTPUT);
    pinMode(_pinEncoderA, INPUT);
    pinMode(_pinEncoderB, INPUT);
}

void Motor::setTargetPosition(int32_t targetPosition)
{
    _targetPosition = targetPosition;  
    update();
}

void Motor::update()
{
    uint32_t error = _targetPosition - _position;
    if (error < -_tolerance)
    {
        // move backward
        digitalWrite(_pinA, HIGH);
        digitalWrite(_pinB, LOW);
    }
    else if (error > _tolerance)
    {
        // move forward
        digitalWrite(_pinA, LOW);
        digitalWrite(_pinB, HIGH);
    }
    else
    {
        stop();
    }
}

void Motor::stop()
{
    // HIGH, HIGH is motor braking
    digitalWrite(_pinA, HIGH);
    digitalWrite(_pinB, HIGH);
}

void IRAM_ATTR Motor::isr()
{
    if (digitalRead(_pinEncoderB))
    {
        _position++;
    }
    else
    {
        _position--;
    }
}