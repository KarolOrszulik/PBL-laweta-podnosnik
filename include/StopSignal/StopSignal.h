#pragma once

#include <Arduino.h>
#include "pins.h"
#include "Pin/GPIO_Pin.h"

class StopSignal
{
public:
    static StopSignal* instance()
    {
        static StopSignal instance;
        return &instance;
    }

    void stop()
    {
        _stopPin->setState(STOP_STATE);
        Serial.println("[STOP_SIGNAL] Stop");
    }

    void clearStop()
    {
        _stopPin->setState(GO_STATE);
        Serial.println("[STOP_SIGNAL] Stop cleared");
    }

private:
    static constexpr int STOP_STATE = LOW;
    static constexpr int GO_STATE = HIGH;

    StopSignal()
    {
        _stopPin->setDirection(OUTPUT);
        stop();
    }
    StopSignal(const StopSignal&) = delete;
    StopSignal& operator=(const StopSignal&) = delete;

    IPin* _stopPin = new GPIO_Pin(STOP_SIGNAL_PIN);
};