#pragma once
#include <Arduino.h>

struct SystemConfig {
    uint32_t loopMicros = 750;        // 750 µs tick
    bool quiet = false;
};

struct ControlConfig {
    int controllerMode = 0;           // PID / manual / open-loop
    float Pgain = 1.0f;
    float Igain = 1.0f;
    float Dgain = 0.0f;

    float flowSetpoint = 0.0f;        // digital reference
    bool externalPWM = true;          // DAC vs PWM
    int analogFlowSource = 0;         // analog selector
};

struct WaveformConfig {
    int opID = 0;                     // operation selector
    int waveformType = 0;             // sine / square / etc.
    float frequency = 1.0f;
    float amplitude = 0.0f;
    float offset = 0.0f;
};
