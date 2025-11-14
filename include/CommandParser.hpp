#pragma once
#include <Arduino.h>

class CommandParser {
public:
    void begin();
    bool feed(char c);   // returns true if a command completed
    void parse(const String &cmd);

    // Public variables the main code can read:
    int quiet = 0;
    int controllerMode = 0;
    float Pgain = 1, Igain = 1, Dgain = 0;
    float digitalFlowRef = 0;
    int analogFlowSelect = 0;
    int externalPWM = 1;

private:
    String buffer;
};
