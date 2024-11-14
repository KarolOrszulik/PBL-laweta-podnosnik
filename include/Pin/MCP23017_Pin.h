#pragma once

#include "IPin.h"
#include "Adafruit_MCP23X17.h"

class MCP23017_Pin : public IPin
{
public:
    MCP23017_Pin(Adafruit_MCP23X17* mcp, uint8_t pin) : _mcp(mcp), _pin(pin)
    {
        mcp->pinMode(pin, OUTPUT);
        mcp->digitalWrite(pin, LOW);
    }

    void setDirection(int direction) override
    {
        _mcp->pinMode(_pin, direction);
    }

    void setState(int state) override
    {
        _mcp->digitalWrite(_pin, state);
    }

    int getState() const override
    {
        return _mcp->digitalRead(_pin);
    }

    int getPinNumber() const override
    {
        return _pin;
    }

private:
    Adafruit_MCP23X17* _mcp = nullptr;
    uint8_t _pin = 0;
};