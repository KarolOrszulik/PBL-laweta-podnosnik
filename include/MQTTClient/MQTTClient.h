#pragma once

#include "MQTTClient/IMQTTClient.h"

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>


class MQTTClient : public IMQTTClient
{
public:
    MQTTClient(CString server, uint16_t port, WiFiClient& client)
        : _client(client)
    {
        _client.setServer(server, port);
    }
    
    void ensureConnection() override
    {
        while (!_client.connected())
        {
            Serial.print("Attempting MQTT connection...");
            if (_client.connect("ESP32-Podnosnik"))
            {
                Serial.println("connected");
                _client.subscribe("podnosnik/command");
            }
            else
            {
                Serial.print("failed, rc=");
                Serial.print(_client.state());
                Serial.println(" try again in 5 seconds");
                delay(5000);
            }
        }
    }

    void loop() override
    {
        _client.loop();
    }

    void subscribe(CString topic) override
    {
        _client.subscribe(topic);
    }

    void publish(CString topic, CString payload) override
    {
        _client.publish(topic, payload);
    }

    void setMessageCallback(MQTTCallback callback) override
    {
        _client.setCallback(callback);
    }

    void setDefaultCallback() override
    {
        setMessageCallback([](char* topic, uint8_t* payload, unsigned int length)
        {
            Serial.print("MQTT message [" + String(topic) + "] ");
            for (int i = 0; i < length; i++)
                Serial.print((char)payload[i]);
            Serial.println();
        });
    }

private:
    PubSubClient _client;
    MQTTCallback _callback;
};