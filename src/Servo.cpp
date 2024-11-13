#include "Servo.h"


Servo::Servo(uint8_t pinMotorA, uint8_t pinMotorB, uint8_t pinEncoderA, uint8_t pinEncoderB)
    : _pinMotorA(pinMotorA), _pinMotorB(pinMotorB), _pinEncoderA(pinEncoderA), _pinEncoderB(pinEncoderB)
{
    pinMode(_pinMotorA, OUTPUT);
    pinMode(_pinMotorB, OUTPUT);
    pinMode(_pinEncoderA, INPUT);
    pinMode(_pinEncoderB, INPUT);
}

void Servo::detectDirection()
{
    Serial.println("Begin direction autodetection.");

    const int32_t startPulses = _pulses;

    moveForward();
    delay(100);

    stop();
    delay(500);
    
    const int32_t forwardPulses = _pulses - startPulses;

    if (forwardPulses < 0)
    {
        std::swap(_pinMotorA, _pinMotorB);
        Serial.println("Motor pins appear to be reversed, swapping motor pins.");
    }
    else
    {
        Serial.println("Motor pins appear to be in the correct order.");
    }

    Serial.println("End direction autodetection.");
}

void Servo::home()
{
    constexpr int msPerUpdate = 50;

    Serial.println("Begin homing sequence.");

    _shouldUpdate = false;

    detectDirection();

    moveBackward();
    int32_t previousPulses;
    do
    {
        previousPulses = _pulses;
        delay(msPerUpdate);
    } while (previousPulses != _pulses);

    stop();

    resetPosition();
    _isHomed = true;

    _shouldUpdate = true;

    setTargetPosition(MIN_POSITION);

    Serial.println("End homing sequence.");
}

void Servo::setTargetPosition(float targetPosition)
{
    if (!isHomed())
    {
        Serial.println("Motor is not homed. Home motor before moving. Aborting...");
        return;
    }

    targetPosition = std::max(targetPosition, MIN_POSITION);

    _targetPulses = positionToPulses(targetPosition);
    const int32_t pulsesError = getPulsesError();

    if (pulsesError > _pulsesTolerance)
    {
        _state = State::TOO_HIGH;
        moveBackward();
    }
    else if (pulsesError < -_pulsesTolerance)
    {
        _state = State::TOO_LOW;
        moveForward();
    }
    else
    {
        _state = State::IN_RANGE;
        stop();
    }
}

void Servo::changeTargetPosition(float distance)
{
    setTargetPosition(pulsesToPosition(_targetPulses) + distance);
}

void Servo::update()
{
    if (!_shouldUpdate) return;

    const int32_t pulsesError = getPulsesError();

    if ( _state == State::TOO_LOW && pulsesError > -_pulsesTolerance ||
         _state == State::TOO_HIGH && pulsesError < _pulsesTolerance)
    {
        _state = State::IN_RANGE;
    }

    if (_state == State::IN_RANGE)
    {
        stop();
    }
}

void Servo::stop()
{
    digitalWrite(_pinMotorA, HIGH);
    digitalWrite(_pinMotorB, HIGH);
}

void Servo::moveForward()
{
    digitalWrite(_pinMotorA, HIGH);
    digitalWrite(_pinMotorB, LOW);
}

void Servo::moveBackward()
{
    digitalWrite(_pinMotorA, LOW);
    digitalWrite(_pinMotorB, HIGH);
}

void Servo::resetPosition()
{
    _pulses = 0;
    _targetPulses = _pulses;
}

void IRAM_ATTR Servo::isr()
{
    if (digitalRead(_pinEncoderB))
        _pulses++;
    else
        _pulses--;
}