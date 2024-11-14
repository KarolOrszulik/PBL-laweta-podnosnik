#pragma once

#include <cstdint>

class IPin
{
public:
    virtual void setDirection(int direction) = 0;
    virtual void setState(int state) = 0;
    virtual int getState() const = 0;
    virtual int getPinNumber() const = 0;
};