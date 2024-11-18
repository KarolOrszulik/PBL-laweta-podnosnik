#pragma once

#include <cstdint>

using MQTTCallback = void(*)(char*, uint8_t*, unsigned int);

class IMQTTClient
{
public:
    virtual void ensureConnection() = 0;
    virtual void loop() = 0;

    virtual void setSubscribedTopic(const char* topic) = 0;
    virtual void publish(const char* topic, const char* payload) = 0;
    virtual void setMessageCallback(MQTTCallback callback) = 0;
};
