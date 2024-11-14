#pragma once

#include <cstdint>

class IMotor
{
public:
    virtual void moveForward(uint8_t speed = 255) = 0;
    virtual void moveBackward(uint8_t speed = 255) = 0;
    virtual void stop() = 0;
};