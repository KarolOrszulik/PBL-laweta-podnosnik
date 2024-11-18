#pragma once

#include <cstdint>

using MQTTCallback = void(*)(char*, uint8_t*, unsigned int);
using CString = const char*;

class IMQTTClient
{
public:
    virtual void ensureConnection() = 0;
    virtual void loop() = 0;

    virtual void subscribe(CString topic) = 0;
    virtual void publish(CString topic, CString payload) = 0;
    virtual void setMessageCallback(MQTTCallback callback) = 0;

    virtual void setDefaultCallback() = 0;
};
