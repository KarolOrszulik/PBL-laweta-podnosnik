#include "Motor.h"


Motor::Motor(uint8_t pinA, uint8_t pinB, uint8_t pinEncoderA, uint8_t pinEncoderB)
    : _pinA(pinA), _pinB(pinB), _pinEncoderA(pinEncoderA), _pinEncoderB(pinEncoderB)
{
    pinMode(_pinA, OUTPUT);
    pinMode(_pinB, OUTPUT);
    pinMode(_pinEncoderA, INPUT);
    pinMode(_pinEncoderB, INPUT);
}

void Motor::setTargetPosition(float targetPosition)
{
    _targetPulses = positionToPulses(targetPosition);

    const int32_t pulsesError = getPulsesError();

    if (pulsesError > _pulsesTolerance)
    {
        _state = State::TOO_HIGH;
        digitalWrite(_pinA, HIGH);
        digitalWrite(_pinB, LOW);
    }
    else if (pulsesError < -_pulsesTolerance)
    {
        _state = State::TOO_LOW;
        digitalWrite(_pinA, LOW);
        digitalWrite(_pinB, HIGH);
    }
    else
    {
        _state = State::IN_RANGE;
        stop();
    }
}

void Motor::move(float distance)
{
    setTargetPosition(pulsesToPosition(_targetPulses) + distance);
}

void Motor::update()
{
    const int32_t pulsesError = getPulsesError();

    if (pulsesError < _pulsesTolerance)
    {
        _state = State::IN_RANGE;
        stop();
    }
    
    // stop at zero-crossing

    // switch (_state)
    // {
    // case State::TOO_LOW:
    //     if (pulsesError > 0)
    //     {
    //         _state = State::IN_RANGE;
    //         stop();
    //     }
    //     break;
    // case State::TOO_HIGH:
    //     if (pulsesError < 0)
    //     {
    //         _state = State::IN_RANGE;
    //         stop();
    //     }
    //     break;
    // case State::IN_RANGE:
    //     stop();
    //     break;
    // }

}

void Motor::stop()
{
    digitalWrite(_pinA, HIGH);
    digitalWrite(_pinB, HIGH);
}

void IRAM_ATTR Motor::isr()
{
    if (digitalRead(_pinEncoderB))
    {
        _pulses++;
    }
    else
    {
        _pulses--;
    }
}