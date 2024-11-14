#pragma once

#include "Motor/IMotor.h"

#include "Pin/IPin.h"
#include <Arduino.h>

class L9110S_Motor : public IMotor
{
public:
    L9110S_Motor(IPin* pinFwd, IPin* pinBck)
        : _pinFwd(pinFwd), _pinBck(pinBck)
    {
        _pinFwd->setDirection(OUTPUT);
        _pinBck->setDirection(OUTPUT);

        this->stop();
    }

    void moveForward(uint8_t speed) override
    {
        _pinFwd->setState(HIGH);
        _pinBck->setState(LOW);
    }

    void moveBackward(uint8_t speed) override
    {
        _pinFwd->setState(LOW);
        _pinBck->setState(HIGH);
    }

    void stop() override
    {
        _pinFwd->setState(HIGH);
        _pinBck->setState(HIGH);
    }

private:
    IPin* _pinFwd = nullptr;
    IPin* _pinBck = nullptr;
};