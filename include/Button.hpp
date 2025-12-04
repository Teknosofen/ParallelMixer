#pragma once
#include <Arduino.h>

class Button {
public:
    Button(uint8_t pin, unsigned long longPressMs = 1000, unsigned long debounceMs = 50);

    void begin();
    void handleInterrupt();
    void update();

    bool wasPressed();
    bool wasReleased();
    bool wasLongPress();

private:
    uint8_t _pin;
    unsigned long _longPressMs;
    unsigned long _debounceMs;

    volatile bool _pressedFlag = false;
    volatile bool _releasedFlag = false;
    volatile unsigned long _lastInterruptTime = 0;
    volatile unsigned long _pressStartTime = 0;
    volatile unsigned long _pressDuration = 0;

    bool _longPressFlag = false;
};
