// Main.ino - Complete refactored ventilator control system
// For Arduino environment
// Date: 2024

#include <Arduino.h>
#include "SensorReader.hpp"
#include "ActuatorControl.hpp"
#include "CommandParser.hpp"

// Pin definitions
#define Flow_Input_Analogue_pin 1
#define Flow_Output_Analogue_pin 3
#define Valve_ctrl_Analogue_pin 6

// System objects
SensorReader sensors(Flow_Input_Analogue_pin, Flow_Output_Analogue_pin);
ActuatorControl actuator(Valve_ctrl_Analogue_pin);
CommandParser parser;

// System configuration
SystemConfig sysConfig;
SensorData sensorData;

// Timing
uint32_t past_time;

void outputData() {
  switch (sysConfig.quiet_mode) {
    case 0:  // Verbose: dP, Flow, SupplyP, Fused Flow, Valve signal
      Serial.print(sensorData.differential_pressure, 2);
      Serial.print(" ");
      Serial.print(sensorData.flow, 2);
      Serial.print(" ");
      Serial.print(sensorData.supply_pressure, 2);
      Serial.print(" ");
      Serial.print(sensorData.fused_flow, 2);
      Serial.print(" ");
      Serial.println(actuator.getValveControlSignal());
      break;
      
    case 1:  // Quiet - no output
      break;
      
    case 2:  // Debug: Integrator, Error, Valve, Flow
      {
        ControlState state = actuator.getControlState();
        Serial.print("I ");
        Serial.print(state.integrator, 1);
        Serial.print(" E ");
        Serial.print(state.error, 1);
        Serial.print(" V ");
        Serial.print(actuator.getValveControlSignal());
        Serial.print(" F ");
        Serial.println(sensorData.fused_flow);
      }
      break;
      
    case 3:  // Special - controlled by actuator.execute()
      // Output is handled in actuator.execute() for mode 3
      break;
      
    case 4:  // Abbreviated: dP, Flow, Valve signal
      Serial.print(sensorData.differential_pressure, 2);
      Serial.print(" ");
      Serial.print(sensorData.flow, 2);
      Serial.print(" ");
      Serial.println(actuator.getValveControlSignal());
      break;
      
    case 5:  // Analog input
      Serial.print("A1 in ");
      Serial.println(sensorData.flow_ref_analogue);
      break;
      
    case 6:  // Flow, SupplyP, FusedFlow
      Serial.print(sensorData.flow, 2);
      Serial.print(" ");
      Serial.print(sensorData.supply_pressure, 2);
      Serial.print(" ");
      Serial.println(sensorData.fused_flow, 2);
      break;
  }
}

void setup() {
  // Initialize system configuration with defaults
  sysConfig.delta_t = 750;  // Sampling time in microseconds
  sysConfig.quiet_mode = 0;  // Verbose output
  sysConfig.flow_setting_is_analog = false;  // Use digital flow reference
  sysConfig.digital_flow_reference = 0.0;  // Flow reference in L/min
  
  // Initialize communication
  parser.begin(250000);
  
  // Initialize sensors
  if (!sensors.initialize()) {
    Serial.println("Sensor initialization failed!");
  }
  
  // Initialize actuator
  actuator.initialize();
  
  // Start timing
  past_time = micros();
  
  Serial.println("System initialized");
  Serial.println("Type ? for help");
}

void loop() {
  // Main control loop timing
  if ((micros() - past_time) >= sysConfig.delta_t) {
    past_time = micros();
    
    // Update sensor readings
    sensors.update(sensorData);
    
    // Output data based on quiet mode
    outputData();
    
    // Execute control - integrated into ActuatorControl
    float flow_ref = sysConfig.flow_setting_is_analog ? 
                     sensorData.flow_ref_analogue : 
                     sysConfig.digital_flow_reference;
    actuator.execute(flow_ref, sensorData.fused_flow, sysConfig.quiet_mode);
  }
  
  // Check for incoming commands - integrated into CommandParser
  parser.update();
  parser.processCommands(sysConfig, actuator);
}