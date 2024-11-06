#include "Scale.h"

void Scale::tare()
{
    _offset = _hx711.read_average(NUM_READINGS);
    Serial.println("Offset: " + String(_offset));
}

void Scale::calibrate(float weight)
{
    auto waitForKey = [] {
        while (!Serial.available())
            ;
        while (Serial.available())
            Serial.read();
    };

    Serial.println("Empty the scale and press any key to continue");
    waitForKey();
    

    this->tare();


    Serial.println("Place a " + String(weight) + "g weight on the scale and press any key to continue");
    waitForKey();

    _scale = (_hx711.read_average(NUM_READINGS) - _offset) / weight;

    Serial.println("Scale: " + String(_scale));
}

float Scale::getWeight()
{
    return (_hx711.read_average(NUM_READINGS) - _offset) / _scale;
}