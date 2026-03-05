// OutputFormatting.hpp - Serial output formatting and WiFi data push
// Extracted from main.cpp

#ifndef OUTPUT_FORMATTING_HPP
#define OUTPUT_FORMATTING_HPP

#include <Arduino.h>

// Serial output (multiple quiet_mode formats)
void outputData();

// WiFi server status push + serial output dispatch
void updateSerialOutput();

// Buffer high-speed data for WiFi web dashboard
void bufferWiFiData();

#endif // OUTPUT_FORMATTING_HPP
