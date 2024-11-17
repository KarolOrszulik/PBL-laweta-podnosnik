#pragma once

#include "Encoder/IEncoder.h"

#include "Pin/GPIO_Pin.h"

class Encoder : public IEncoder
{
public:
    Encoder(GPIO_Pin* pinA, GPIO_Pin* pinB)
        : _pinTrig(pinA), _pinDir(pinB)
    {
        _pinTrig->setDirection(INPUT);
        _pinDir->setDirection(INPUT);
    }

    int32_t getPulses() override
    {
        return _pulses;
    }

    void resetPulses() override
    {
        _pulses = 0;
    }

    void IRAM_ATTR isr() override
    {
        if (_pinDir->getState())
            _pulses++;
        else
            _pulses--;
    }

    int getTriggerPin() override
    {
        return _pinTrig->getPinNumber();
    }

private:
    IPin* _pinTrig = nullptr;
    IPin* _pinDir  = nullptr;

    volatile int32_t _pulses = 0;
};