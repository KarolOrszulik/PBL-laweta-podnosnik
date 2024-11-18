#pragma once

#include "MQTTClient/IMQTTClient.h"

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>


class MQTTClient : public IMQTTClient
{
public:
    MQTTClient(String const& clientName, const char* serverIP, uint16_t port, WiFiClient& client)
        : _clientName(clientName), _client(client)
    {
        _client.setServer(serverIP, port);
    }
    
    void ensureConnection() override
    {
        while (!_client.connected())
        {
            Serial.print("Attempting MQTT connection...");
            if (_client.connect(_clientName.c_str()))
            {
                Serial.println("connected");
                _client.subscribe(_subscribedTopic.c_str());
            }
            else
            {
                constexpr int TIMEOUT_S = 3;
                Serial.println("failed, rc=" + String(_client.state()) + ", trying again in " + String(TIMEOUT_S) + "s");
                delay(TIMEOUT_S * 1000);
            }
        }
    }

    void loop() override
    {
        _client.loop();
    }

    void setSubscribedTopic(const char* topic) override
    {
        _subscribedTopic = String(topic);
    }

    void publish(const char* topic, const char* payload) override
    {
        _client.publish(topic, payload);
    }

    void setMessageCallback(MQTTCallback callback) override
    {
        _client.setCallback(callback);
    }

private:
    PubSubClient _client;
    MQTTCallback _callback;
    String _clientName;
    String _subscribedTopic;
};