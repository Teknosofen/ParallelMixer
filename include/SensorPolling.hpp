// SensorPolling.hpp - Sensor reading and polling functions
// Extracted from main.cpp

#ifndef SENSOR_POLLING_HPP
#define SENSOR_POLLING_HPP

#include <Arduino.h>

// ELVH helpers — single sensor may be on either bus
float getELVH_Pressure();
float getELVH_Temperature();

// Sensor polling functions (called from main loop)
void pollPressureSensors(uint32_t now);
void readFastSensors(uint32_t now);
void pollFDO2(uint32_t now);

#endif // SENSOR_POLLING_HPP
