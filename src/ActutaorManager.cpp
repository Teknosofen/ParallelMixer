#include "ActuatorManager.h"

void ActuatorManager::begin(uint8_t pwmPin_, uint8_t dacAddr_) {
    pwmPin = pwmPin_;
    dacAddress = dacAddr_;
    pinMode(pwmPin, OUTPUT);
}

void ActuatorManager::enableExternal(bool ext) {
    externalPWM = ext;
}

void ActuatorManager::setPWM(uint16_t val) {
    if (!externalPWM) {
        analogWrite(pwmPin, val);
    }
}

void ActuatorManager::setDAC(uint16_t val) {
    if (externalPWM) {
        Wire.beginTransmission(dacAddress);
        Wire.write(64);
        Wire.write(val >> 4);
        Wire.write((val & 0x0F) << 4);
        Wire.endTransmission();
    }
}
