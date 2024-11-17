#pragma once

#include <cstdint>

class IMotor
{
public:
    virtual void setSpeed(int speed) = 0;   // speed in the range [-255, 255]
    virtual void moveForward(uint8_t speed = 255) = 0;
    virtual void moveBackward(uint8_t speed = 255) = 0;
    virtual void stop() = 0;
};