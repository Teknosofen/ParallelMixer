// DisplayUpdate.hpp - TFT display and button handling
// Extracted from main.cpp

#ifndef DISPLAY_UPDATE_HPP
#define DISPLAY_UPDATE_HPP

#include <Arduino.h>

// Initialize the TFT display layout on first loop iteration
void initDisplay();

// Update TFT display fields and O2 reading at their own rates
void updateDisplay();

// Handle WiFi toggle and ventilator settings display button presses
void handleButtons();

#endif // DISPLAY_UPDATE_HPP
