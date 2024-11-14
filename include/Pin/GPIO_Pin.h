#pragma once

#include "Pin/IPin.h"

#include <Arduino.h>

class GPIO_Pin : public IPin
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
        digitalWrite(_pin, value);
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