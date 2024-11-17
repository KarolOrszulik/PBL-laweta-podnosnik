#pragma once

#include <cstdint>

class IServo
{
public:
    virtual void setPulsesPerMM(int32_t pulsesPerMM) = 0;

    virtual void setTargetPosition(float position) = 0;
    virtual void moveTargetPosition(float distance) = 0;

    virtual float getPositionError() = 0;

    virtual void home(bool homeDownward) = 0;

    virtual void setPositionLimits(float min, float max) = 0;

    virtual void update() = 0;
    virtual void enableUpdates() = 0;
    virtual void disableUpdates() = 0;
    
    virtual void stop() = 0;
};