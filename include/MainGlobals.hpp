// MainGlobals.hpp - Extern declarations for shared state defined in main.cpp
// Used by extracted subsystem modules (SensorPolling, OutputFormatting, DisplayUpdate)

#ifndef MAIN_GLOBALS_HPP
#define MAIN_GLOBALS_HPP

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "main.hpp"
#include "PinConfig.h"
#include "SensorReader.hpp"
#include "ActuatorControl.hpp"
#include "CommandParser.hpp"
#include "ImageRenderer.hpp"
#include "Button.hpp"
#include "PMixerWiFiServer.hpp"
#include "SerialMuxRouter.hpp"
#include "FDO2_Sensor.h"
#include "VentilatorController.hpp"
#include "LocalValveController.hpp"
#include "ValveCharacterizer.hpp"

// Number of MUX channels (shared constant)
static const uint8_t NUM_MUX_CHANNELS = 6;

// Convenience macro: currently selected actuator
#define actuator actuators[sysConfig.mux_channel]

// Hardware
extern TFT_eSPI tft;
extern ImageRenderer renderer;

// Sensor readers on separate I2C buses
extern SensorReader* sensors_bus0;
extern SensorReader* sensors_bus1;

// Actuator controls — one per MUX channel
extern ActuatorControl actuators[NUM_MUX_CHANNELS];

// Subsystems
extern CommandParser parser;
extern SerialMuxRouter muxRouter;
extern VentilatorController ventilator;
extern LocalValveController localValveCtrl;
extern ValveCharacterizer valveChar;

// UI
extern Button interactionKey1;
extern Button interactionKey2;
extern PMixerWiFiServer wifiServer;
extern bool showVentilatorSettings;

// FDO2 Optical Oxygen Sensor
extern FDO2_Sensor fdo2Sensor;
extern FDO2_Sensor::MeasurementData fdo2Data;
extern bool fdo2Initialized;

// System configuration and sensor data
extern SystemConfig sysConfig;
extern SensorData sensorData_bus0;
extern SensorData sensorData_bus1;

// Sensor initialization flags
extern bool sensorsInitialized;
extern bool sensors_bus1_initialized;

#endif // MAIN_GLOBALS_HPP
