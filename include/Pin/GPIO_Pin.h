#pragma once

#include "Pin/IInterruptablePin.h"

#include <Arduino.h>

class GPIO_Pin : public IInterruptablePin
{
public:
    GPIO_Pin(uint8_t pin) : _pin(pin)
    {}

    void setDirection(int direction)
    {
        pinMode(_pin, direction);
    }

    void setState(int value) override
    {
        analogWrite(_pin, 0);
        digitalWrite(_pin, value);
    }

    void setPWM(int value) override
    {
        analogWrite(_pin, value);
    }

    int getState() const override
    {
        return digitalRead(_pin);
    }

    int getPinNumber() const override
    {
        return _pin;
    }

private:
    uint8_t _pin  = 0;
};