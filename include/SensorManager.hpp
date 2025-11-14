#pragma once
#include <Arduino.h>
#include <Wire.h>

class SensorManager {
public:
    void begin();

    float readSPD();
    float readSFM();
    float readSSC();

    float fuse(float diffPress, float flow);

private:
    uint8_t msb, lsb;
    int16_t combined;
};
