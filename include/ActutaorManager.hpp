#pragma once
#include <Arduino.h>
#include <Wire.h>

class ActuatorManager {
public:
    void begin(uint8_t pwmPin, uint8_t dacAddr);

    void setPWM(uint16_t value);
    void setDAC(uint16_t value);

    void enableExternal(bool ext);

private:
    bool externalPWM = true;
    uint8_t pwmPin;
    uint8_t dacAddress;
};
