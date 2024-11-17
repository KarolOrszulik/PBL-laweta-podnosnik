#pragma once

class IScale
{
public:
    virtual void tare(int numSamples = 10) = 0;
    virtual void setScaleByReading(float weight, int numSamples = 10) = 0;
    virtual void setScale(float scale) = 0;
    virtual float getWeight(int numSamples = 1) = 0;
};