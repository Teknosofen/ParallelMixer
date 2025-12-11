// Main.cpp - Complete refactored ventilator control system with Dual I2C + Dual Serial
// For T-Display S3
// Date: 2024

#include <Arduino.h>
#include "main.hpp"
#include <TFT_eSPI.h>
#include "PinConfig.h"            // NEW: Pin configuration
#include "SensorReader.hpp"
#include "ActuatorControl.hpp"
#include "CommandParser.hpp"
#include "ImageRenderer.hpp"
#include "Button.hpp"
#define PMIXER_GRAPH_DISPLAY_POINTS 512
#include "PMixerWiFiServer.hpp"

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spriteLeft = TFT_eSprite(&tft);
ImageRenderer renderer(tft);

// NEW: Two sensor readers on separate I2C buses
SensorReader sensors_bus0(&Wire, "Bus0");
SensorReader sensors_bus1(&Wire1, "Bus1");

ActuatorControl actuator(Valve_ctrl_Analogue_pin);
CommandParser parser;

Button interactionKey1(INTERACTION_BUTTON_PIN);
PMixerWiFiServer wifiServer(PMIXERSSID, PMIXERPWD);

// System configuration
SystemConfig sysConfig;
SensorData sensorData_bus0;  // Data from Bus 0 sensors
SensorData sensorData_bus1;  // Data from Bus 1 sensors

// Timing
uint32_t past_time;

void outputData() {
  // Output data from Bus 0 (primary sensors)
  switch (sysConfig.quiet_mode) {
    case 0:  // Verbose: dP, Flow, SupplyP, Valve signal
      Serial.print("[Bus0] ");
      Serial.print(sensorData_bus0.differential_pressure, 2);
      Serial.print(" ");
      Serial.print(sensorData_bus0.flow, 2);
      Serial.print(" ");
      Serial.print(sensorData_bus0.supply_pressure, 2);
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
        Serial.println(sensorData_bus0.flow);
      }
      break;
      
    case 3:  // Special - controlled by actuator.execute()
      break;
      
    case 4:  // Abbreviated: dP, Flow, Valve signal
      Serial.print(sensorData_bus0.differential_pressure, 2);
      Serial.print(" ");
      Serial.print(sensorData_bus0.flow, 2);
      Serial.print(" ");
      Serial.println(actuator.getValveControlSignal());
      break;
      
    case 5:  // Flow and Supply Pressure
      Serial.print("Flow: ");
      Serial.print(sensorData_bus0.flow, 2);
      Serial.print(" SupplyP: ");
      Serial.println(sensorData_bus0.supply_pressure, 2);
      break;
      
    case 6:  // Flow, SupplyP from both buses
      Serial.print("[Bus0] ");
      Serial.print(sensorData_bus0.flow, 2);
      Serial.print(" ");
      Serial.print(sensorData_bus0.supply_pressure, 2);
      Serial.print(" | [Bus1] ");
      Serial.print(sensorData_bus1.flow, 2);
      Serial.print(" ");
      Serial.println(sensorData_bus1.supply_pressure, 2);
      break;
  }
}

void setup() {
  // CRITICAL: Enable display power first!
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  delay(100);
  
  renderer.begin(); // Initialize display

  // Initialize system configuration with defaults
  sysConfig.delta_t = 100000;  // Sampling time in microseconds
  sysConfig.quiet_mode = 0;    // Verbose output
  sysConfig.digital_flow_reference = 0.0;  // Flow reference in L/min
  
  // Initialize USB Serial
  parser.begin(250000);
  delay(2000);  // Reduced from 12000 - usually not needed with USB CDC
  
  Serial.println("\n=== P-Mixer Dual I2C + Dual Serial Setup ===\n");

  // ============================================================================
  // Initialize I2C Buses BEFORE sensors
  // ============================================================================
  
  Serial.println("Initializing I2C buses...");
  
  // I2C Bus 0 (Wire) - GPIO43/44
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, 500000);
  Serial.printf("✅ I2C Bus 0: SDA=GPIO%d, SCL=GPIO%d\n", I2C0_SDA_PIN, I2C0_SCL_PIN);
  
  // I2C Bus 1 (Wire1) - GPIO10/11
  Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 500000);
  Serial.printf("✅ I2C Bus 1: SDA=GPIO%d, SCL=GPIO%d\n\n", I2C1_SDA_PIN, I2C1_SCL_PIN);

  // ============================================================================
  // Initialize Serial Ports for external microcontrollers
  // ============================================================================
  
  Serial.println("Initializing Serial ports...");
  
  // Serial1 - First external microcontroller
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial.printf("✅ Serial1: TX=GPIO%d, RX=GPIO%d\n", SERIAL1_TX_PIN, SERIAL1_RX_PIN);
  
  // Serial2 - Second external microcontroller
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
  Serial.printf("✅ Serial2: TX=GPIO%d, RX=GPIO%d\n\n", SERIAL2_TX_PIN, SERIAL2_RX_PIN);

  // ============================================================================
  // Initialize Sensors on both buses
  // ============================================================================
  
  interactionKey1.begin();

  Serial.println("Initializing sensors...");
  
  // Initialize sensors on Bus 0
  if (!sensors_bus0.initialize()) {
    Serial.println("❌ Bus 0 sensor initialization failed!");
  }
  
  // Initialize sensors on Bus 1
  if (!sensors_bus1.initialize()) {
    Serial.println("❌ Bus 1 sensor initialization failed!");
  }
  
  Serial.println();

  // ============================================================================
  // Initialize other systems
  // ============================================================================
  
  wifiServer.start();
  Serial.println("✅ WiFi server started\n");
  
  actuator.initialize();
  Serial.println("✅ Actuator initialized\n");
  
  // Start timing
  past_time = micros();
  
  Serial.println("=== System initialized ===");
  Serial.println("Type ? for help\n");
  
  // Test Serial ports
  Serial1.println("Hello from ESP32 on Serial1");
  Serial2.println("Hello from ESP32 on Serial2");
}

void loop() {
  // Initial setup
  static bool initLoop = false;
  if (!initLoop) {
    initLoop = true;
    hostCom.println("P-mixer loop start init");
    renderer.clear();
    renderer.drawLabel();
    renderer.drawStatusField();
    renderer.drawWiFiField();
    renderer.drawWiFiAPIP("WiFi OFF      ", "No SSID        ");
    renderer.drawWiFiPromt("Press key to enable ");
    hostCom.println("P-mixer now initialized");
  }

  // ============================================================================
  // Main control loop timing
  // ============================================================================
  
  if ((micros() - past_time) >= sysConfig.delta_t) {
    past_time = micros();
    
    // Update sensor readings from BOTH buses
    sensors_bus0.update(sensorData_bus0);
    sensors_bus1.update(sensorData_bus1);
    
    // Output data based on quiet mode
    outputData();
    
    // Execute control - using Bus 0 as primary
    actuator.execute(sysConfig.digital_flow_reference, sensorData_bus0.flow, sysConfig.quiet_mode);

    // Update WiFi server with Bus 0 data (primary)
    wifiServer.updateFlow(sensorData_bus0.flow);
    wifiServer.updatePressure(sensorData_bus0.supply_pressure);
    wifiServer.updateValveSignal(actuator.getValveControlSignal());
  }
  
  // ============================================================================
  // Handle Serial communication with external microcontrollers
  // ============================================================================
  
  // Check for data from Serial1
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    Serial.printf("[Serial1 RX]: %s\n", data.c_str());
    // Process data from external MCU 1
  }
  
  // Check for data from Serial2
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    Serial.printf("[Serial2 RX]: %s\n", data.c_str());
    // Process data from external MCU 2
  }
  
  // Send data to external microcontrollers (example)
  static uint32_t lastSerialSend = 0;
  if (millis() - lastSerialSend > 1000) {  // Send every 1 second
    lastSerialSend = millis();
    
    // Send to Serial1
    Serial1.printf("Flow:%0.2f,Press:%0.2f\n", 
                   sensorData_bus0.flow, 
                   sensorData_bus0.supply_pressure);
    
    // Send to Serial2
    Serial2.printf("Flow:%0.2f,Press:%0.2f\n", 
                   sensorData_bus1.flow, 
                   sensorData_bus1.supply_pressure);
  }

  // ============================================================================
  // Command parser and UI updates
  // ============================================================================
  
  parser.update();
  parser.processCommands(sysConfig, actuator);

  // Handle user interface update
  static uint32_t UILoopStartTime = micros();
  if (micros() - UILoopStartTime > SET_UI_UPDATE_TIME) {
    UILoopStartTime = micros();
    
    ControllerMode presentMode = actuator.getControllerMode();
    String ctrlMode = "Valve Set";
    switch (presentMode) {
      case PID_CONTROL:
        ctrlMode = "PID";
        break;
      case VALVE_SET_VALUE_CONTROL:
        ctrlMode = "Valve Set";
        break;
      case SINE_CONTROL:
        ctrlMode = "Sine";
        break;
      case STEP_CONTROL:
        ctrlMode = "Step";
        break;
      case TRIANGLE_CONTROL:
        ctrlMode = "Triangle";
        break;
      default:
        ctrlMode = "Unknown";
        break;
    }
    
    renderer.drawControllerMode(ctrlMode);
    wifiServer.updateMode(ctrlMode);
    
    // Display Bus 0 data (primary)
    renderer.drawFlow(String(sensorData_bus0.flow, 2) + " L/min");
    renderer.drawPressure(String(sensorData_bus0.supply_pressure, 2) + " kPa");
    renderer.drawValveCtrlSignal(String(actuator.getValveControlSignal(), 2));
  }

  // ============================================================================
  // WiFi control button handling
  // ============================================================================
  
  if (interactionKey1.wasReleased()) {
    if (interactionKey1.wasLongPress()) {
      hostCom.println("Key1 long press (>1s)");
      wifiServer.start();
      renderer.drawWiFiAPIP("WiFi ON       ", wifiServer.getApIpAddress());
      renderer.drawWiFiPromt("Press key to disable");
    } else {
      hostCom.println("interactionKey1 short press");
      wifiServer.stop();
      hostCom.println("WiFi Access Point stopped");
      renderer.drawWiFiAPIP("WiFi OFF      ", "No SSID        ");
      renderer.drawWiFiPromt("Press key to enable");
    }
  }
  
  // Handle web requests
  wifiServer.handleClient();
}
