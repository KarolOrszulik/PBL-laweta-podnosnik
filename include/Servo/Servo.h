#pragma once

#include "Servo/IServo.h"

#include "Motor/IMotor.h"
#include "Encoder/IEncoder.h"
#include "Controller/IController.h"

class Servo : public IServo
{
public:
    Servo(IMotor* motor, IEncoder* encoder, IController* controller)
        : _motor(motor), _encoder(encoder), _controller(controller)
    {
    }

    void setPulsesPerMM(int32_t pulsesPerMM) override
    {
        _pulsesPerMM = pulsesPerMM;
    }

    void setTargetPosition(float position) override
    {
        if (position < _minPosition) position = _minPosition;
        if (position > _maxPosition) position = _maxPosition;

        _controller->setTargetValue(position);
    }

    void moveTargetPosition(float distance) override
    {
        const float currentTarget = _controller->getTargetValue();
        setTargetPosition(currentTarget + distance);
    }

    float getPositionError() override
    {
        const float currentPosition = pulsesToPosition(_encoder->getPulses());
        const float targetPosition  = _controller->getTargetValue();
        return currentPosition - targetPosition;
    }

    void update() override
    {
        if (!_shouldUpdate) return;

        const float currentPosition = pulsesToPosition(_encoder->getPulses());
        const float controlSignal = _controller->update(currentPosition);

        _motor->setSpeed(controlSignal * 255);
    }

    void enableUpdates() override
    {
        _shouldUpdate = true;
    }

    void disableUpdates() override
    {
        _shouldUpdate = false;
    }

    void stop() override
    {
        _motor->stop();
    }

    void home(bool homeDownward) override
    {
        constexpr int MS_BETWEEN_UPDATES = 50;

        disableUpdates();

        if (homeDownward)
            _motor->moveBackward();
        else
            _motor->moveForward();

        int32_t previousPulses;
        do
        {
            previousPulses = _encoder->getPulses();
            delay(MS_BETWEEN_UPDATES);
        } while (previousPulses != _encoder->getPulses());

        stop();

        _encoder->resetPulses();
        _controller->setTargetValue(0.0f);

        enableUpdates();
    }

    void setPositionLimits(float min, float max) override
    {
        _minPosition = min;
        _maxPosition = max;
    }

private:
    float pulsesToPosition(int32_t pulses) const
    {
        return (float)pulses / _pulsesPerMM;
    }

    int32_t positionToPulses(float position) const
    {
        return position * _pulsesPerMM;
    }


    IMotor* _motor;
    IEncoder* _encoder;
    IController* _controller;

    int32_t _pulsesPerMM = 1;
    int32_t _pulsesTolerance = 0;

    float _minPosition = 0.f;
    float _maxPosition = 3.402823466e+38;

    bool _shouldUpdate = false;
};