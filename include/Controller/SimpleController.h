#pragma once

#include "Controller/IController.h"

#include <cstdint>

class SimpleController : public IController
{
public:
    SimpleController(float tolerance)
        : _tolerance(tolerance)
    {
    }

    void setTargetValue(float targetValue) override
    {
        _targetValue = targetValue;
    }

    float getTargetValue() const override
    {
        return _targetValue;
    }

    float update(float newValue) override
    {
        if (newValue > _targetValue + _tolerance)
        {
            return -1.0f;
        }
        else if (newValue < _targetValue - _tolerance)
        {
            return 1.0f;
        }
        else
        {
            return 0.0f;
        }
    }

private:
    float _targetValue = 0;
    float _tolerance = 0;
};