#include "Button.hpp"

Button::Button(uint8_t pin, unsigned long longPressMs, unsigned long debounceMs)
    : _pin(pin), _longPressMs(longPressMs), _debounceMs(debounceMs) {}

void Button::begin() {
    pinMode(_pin, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(_pin), [](void* arg) {
        reinterpret_cast<Button*>(arg)->handleInterrupt();
    }, this, CHANGE);
}

void IRAM_ATTR Button::handleInterrupt() {
    unsigned long currentTime = millis();
    if (currentTime - _lastInterruptTime < _debounceMs) return;
    _lastInterruptTime = currentTime;

    if (digitalRead(_pin) == LOW) {
        // pressed
        _pressedFlag = true;
        _pressStartTime = currentTime;
    } else {
        // released
        _releasedFlag = true;
        _pressDuration = currentTime - _pressStartTime;
        if (_pressDuration >= _longPressMs) {
            _longPressFlag = true;
        }
    }
}

void Button::update() {
    // nothing fancy now, but could be expanded later
}

bool Button::wasPressed() {
    if (_pressedFlag) {
        _pressedFlag = false;
        return true;
    }
    return false;
}

bool Button::wasReleased() {
    if (_releasedFlag) {
        _releasedFlag = false;
        return true;
    }
    return false;
}

bool Button::wasLongPress() {
    if (_longPressFlag) {
        _longPressFlag = false;
        return true;
    }
    return false;
}
