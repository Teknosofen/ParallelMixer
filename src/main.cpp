// Main.ino - Complete refactored ventilator control system
// For Arduino environment
// Date: 2024

#include <Arduino.h>
#include "main.hpp"
#include <TFT_eSPI.h>
#include "SensorReader.hpp"
#include "ActuatorControl.hpp"
#include "CommandParser.hpp"
#include "ImageRenderer.hpp"
#include "Button.hpp"
#define PMIXER_GRAPH_DISPLAY_POINTS 512  // Show 512 samples on graph
#include "PMixerWiFiServer.hpp"

TFT_eSPI tft = TFT_eSPI(); // Initialize the display
TFT_eSprite spriteLeft = TFT_eSprite(&tft); // Create sprite object left
ImageRenderer renderer(tft);

// System objects
SensorReader sensors(Flow_Input_Analogue_pin, Flow_Output_Analogue_pin);
ActuatorControl actuator(Valve_ctrl_Analogue_pin);
CommandParser parser;

Button interactionKey1(INTERACTION_BUTTON_PIN);  // GPIO14 Key 2
PMixerWiFiServer wifiServer(PMIXERSSID, PMIXERPWD); // WiFi server for PMixer

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
  renderer.begin(); //initialize display

  // Initialize system configuration with defaults
  sysConfig.delta_t = 100000;  // Sampling time in microseconds
  sysConfig.quiet_mode = 0;  // Verbose output
  sysConfig.flow_setting_is_analog = false;  // Use digital flow reference
  sysConfig.digital_flow_reference = 0.0;  // Flow reference in L/min
  
  // Initialize communication
  parser.begin(250000);
  delay(12000); // Wait for serial to stabilize
  
  Serial.println("Parallel mixer setup started");

  interactionKey1.begin();

  // Initialize sensors
  if (!sensors.initialize()) {
    Serial.println("Sensor initialization failed!");
  }
  
  wifiServer.start();
  Serial.println("WiFi server started");
  // Initialize actuator
  actuator.initialize();
  
  // Start timing
  past_time = micros();
  
  Serial.println("System initialized");
  Serial.println("Type ? for help");
}

void loop() {
  // initial setup
  static bool initLoop = false; // Flag to check if loop has been initialized
  if (!initLoop) {
    initLoop = true; // Set the flag to true to indicate loop has been initialized
    hostCom.println("P-mixer loop start init");
    renderer.clear(); // Clear the display with blue color
    renderer.drawLabel();
    renderer.drawStatusField();
    renderer.drawWiFiField();
    renderer.drawWiFiAPIP("WiFi OFF      ", "No SSID        ");
    renderer.drawWiFiPromt("Press key to enable ");

    hostCom.println("P-mixer now initialized");
  } // init loop

  // Main control loop timing
  if ((micros() - past_time) >= sysConfig.delta_t) {
    past_time = micros();
    
    // Update sensor readings
    // sensors.update(sensorData);
    
    // Output data based on quiet mode
    outputData();
    
    // Execute control - integrated into ActuatorControl
    float flow_ref = sysConfig.flow_setting_is_analog ? 
                     sensorData.flow_ref_analogue : 
                     sysConfig.digital_flow_reference;
    actuator.execute(flow_ref, sensorData.fused_flow, sysConfig.quiet_mode);

    wifiServer.updateFlow(sensorData.flow);           // Push flow data
    wifiServer.updatePressure(sensorData.supply_pressure);   // Push pressure data
    wifiServer.updateValveSignal(actuator.getValveControlSignal());  // Push valve signal (also records history)
    // wifiServer.updateMode("Manual");            // Push mode string


  }
  
  // Check for incoming commands - integrated into CommandParser
  parser.update();
  parser.processCommands(sysConfig, actuator);

  // Handle user interface update
  static uint32_t UILoopStartTime = micros(); // Record the start time of the loop
  if (micros() - UILoopStartTime > SET_UI_UPDATE_TIME) { // time loop, Check if enough time has passed since the last loop iteration
    UILoopStartTime = micros(); // Reset the start time for the next loop
    renderer.drawControllerMode("Idle"); // Draw the controller mode
    ControllerMode presentMode = actuator.getControllerMode(); // Get the current controller mode
    String ctrlMode = "Valve Set"; // Default mode string
    switch (presentMode) {
      case PID_CONTROL:
      ctrlMode = "PID"; // Set mode string for PID control
        break;
      case VALVE_SET_VALUE_CONTROL:
        ctrlMode = "Valve Set"; // Set mode string for Valve Set control
        break;
      case SINE_CONTROL:
        ctrlMode = "Sine"; // Set mode string for Sine control
        break;
      case STEP_CONTROL:
        ctrlMode = "Step"; // Set mode string for Step control
        break;
      case TRIANGLE_CONTROL:
        ctrlMode = "Triangle"; // Set mode string for Triangle control
        break;
      default:
        ctrlMode = "Unknown"; // Set mode string for unknown control mode
        break;
    }
    renderer.drawControllerMode(ctrlMode);
    wifiServer.updateMode(ctrlMode); // Push the current mode to the WiFi server
    
    renderer.drawFlow(String(sensorData.flow, 2) + " L/min");
    renderer.drawPressure(String(sensorData.supply_pressure, 2) + " kPa");
    renderer.drawValveCtrlSignal(String(actuator.getValveControlSignal(), 2));
  }

  if (interactionKey1.wasReleased()) {
    if (interactionKey1.wasLongPress()) {
      hostCom.println("Key1 long press (>1s)");
      // myWiFiServer.begin();
      wifiServer.start();
      // hostCom.println("Started WiFi");
      // hostCom.printf("Access Point IP: %s\n", myWiFiServer.getApIpAddress());
      // renderer.drawWiFiAPIP(myWiFiServer.getApIpAddress() + "  ", LOGGER_SSID);
      renderer.drawWiFiAPIP("WiFi ON       ", wifiServer.getApIpAddress());
      renderer.drawWiFiPromt("Press key to disable");
      // Add QR stuff here

    } else {                                    // Stop WiFi Access Point
      hostCom.println("interactionKey1 short press");
      wifiServer.stop();   // Stop WiFi AP
      // WiFi.softAPdisconnect(true);              // true = erase settings
      // WiFi.mode(WIFI_OFF);                      // turn off WiFi radio
      hostCom.println("WiFi Access Point stopped");
      renderer.drawWiFiAPIP("WiFi OFF      ", "No SSID        ");
      renderer.drawWiFiPromt("Press key to enable");
    }
  }
  // Handle web requests (must call in loop!)
  wifiServer.handleClient();
}