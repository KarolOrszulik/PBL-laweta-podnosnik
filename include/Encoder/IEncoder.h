#pragma once

#include <Arduino.h>
#include <cstdint>

class IEncoder
{
public:
    virtual int32_t getPulses() = 0;
    virtual void resetPulses() = 0;
    virtual int getTriggerPin() = 0;
    virtual IRAM_ATTR void isr() = 0;
};