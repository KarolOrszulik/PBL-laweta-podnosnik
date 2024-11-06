#pragma once

#include "HX711.h"

class Scale
{
public:
    Scale(uint8_t pinDT, uint8_t pinSCK)
    {
        _hx711.begin(pinDT, pinSCK);
    }

    void setCalibrationValues(float scale, long offset)
    {
        _scale = scale;
        _offset = offset;
    }

    void tare();
    void calibrate(float weight);
    float getWeight();

private:
    static constexpr int NUM_READINGS = 10;

    HX711 _hx711;
    float _scale = 1;
    long _offset = 0;
};