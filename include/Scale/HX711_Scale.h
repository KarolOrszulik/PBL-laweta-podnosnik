#pragma once

#include "Scale/IScale.h"

#include "HX711.h"
#include "Pin/GPIO_Pin.h"

class HX711_Scale : public IScale
{
public:
    HX711_Scale(GPIO_Pin* pinDT, GPIO_Pin* pinSCK)
        : _pinDT(pinDT), _pinSCK(pinSCK)
    {
        _hx711.begin(pinDT->getPinNumber(), pinSCK->getPinNumber());
    }

    virtual void tare(int numSamples) override
    {
        _offset = _hx711.read_average(numSamples);
    }

    virtual void setScaleByReading(float weight, int numReadings) override
    {
        _scale = (_hx711.read_average(numReadings) - _offset) / weight;
    }

    virtual void setScale(float scale = 1) override
    {
        _scale = scale;
    }

    virtual float getWeight(int numSamples) override
    {
        return (_hx711.read_average(numSamples) - _offset) / _scale;
    }

private:
    HX711 _hx711;
    float _scale = 1;
    long _offset = 0;

    IPin* _pinDT = nullptr;
    IPin* _pinSCK = nullptr;
};