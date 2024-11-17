#pragma once

class IController
{
public:
    virtual void setTargetValue(float value) = 0;
    virtual float getTargetValue() const = 0;

    virtual float update(float newValue) = 0;   // returns the conrol signal in the range [-1, 1]
};