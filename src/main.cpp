// Main.cpp - Complete refactored ventilator control system with Dual I2C + Dual Serial
// For T-Display S3
// Date: 2024

#include <Arduino.h>
#include <main.hpp>
#include <TFT_eSPI.h>
#include "PinConfig.h"            // Pin configuration
#include "SensorReader.hpp"
#include "ActuatorControl.hpp"
#include "CommandParser.hpp"
#include "ImageRenderer.hpp"
#include "Button.hpp"
#define PMIXER_GRAPH_DISPLAY_POINTS 512
#include "PMixerWiFiServer.hpp"

// ---------------------------------
// logo BMP file as well as factory default config files are stored in SPIFFS
// to send to device run:
// pio run --target uploadfs
//
// to format SPIFFS run:
// pio run --target cleanfs
// ---------------------------------


// in the TFT_eSPI TFT_eSPI_User_setup_Select.H ensure that the 
// line 133 #include <User_Setups/Setup206_LilyGo_T_Display_S3.h>     // For the LilyGo T-Display S3 based ESP32S3 with ST7789 170 x 320 TFT

// In the TFT_eSPI_User_setup.H 
// line 55 #define ST7789_DRIVER      // Full configuration option, define additional parameters below for this display
// line 87 #define TFT_WIDTH  170 // ST7789 170 x 320
// line 92 #define TFT_HEIGHT 320 // ST7789 240 x 320


TFT_eSPI tft = TFT_eSPI();
ImageRenderer renderer(tft);

// Two sensor readers on separate I2C buses - use pointers to avoid initialization issues
SensorReader* sensors_bus0;   // GPIO43/44 - includes SFM3505
// SensorReader* sensors_bus1;   // GPIO10/11 - DISABLED FOR NOW

ActuatorControl actuator(Valve_ctrl_Analogue_pin);
CommandParser parser;

Button interactionKey1(INTERACTION_BUTTON_PIN);
PMixerWiFiServer wifiServer(PMIXERSSID, PMIXERPWD);

// System configuration
SystemConfig sysConfig;
SensorData sensorData_bus0;  // Data from Bus 0 sensors (includes SFM3505)
SensorData sensorData_bus1;  // Data from Bus 1 sensors

// Sensor initialization status
bool sensorsInitialized = false;

// Timing
uint32_t past_time;

void outputData() {
  // Output data from Bus 0 (primary sensors)
  switch (sysConfig.quiet_mode) {
    case 0:  // Verbose: Pressure, Air flow, Valve signal
      hostCom.printf("P: %.2f\tAir: %.3f\tValve: %.2f\n",
                     sensorData_bus0.supply_pressure,
                     sensorData_bus0.sfm3505_air_flow,
                     actuator.getValveControlSignal());
      break;

    case 1:  // Quiet - no output
      break;

    case 2:  // Debug: Integrator, Error, Valve, Flow, Air
      {
        ControlState state = actuator.getControlState();
        hostCom.printf("I %.1f E %.1f V %.2f F %.2f Air %.3f\n",
                       state.integrator,
                       state.error,
                       actuator.getValveControlSignal(),
                       sensorData_bus0.flow,
                       sensorData_bus0.sfm3505_air_flow);
      }
      break;

    case 3:  // Special - controlled by actuator.execute()
      break;

    case 4:  // Abbreviated: dP, Flow, Valve signal
      hostCom.printf("%.2f %.2f %.2f\n",
                     sensorData_bus0.differential_pressure,
                     sensorData_bus0.flow,
                     actuator.getValveControlSignal());
      break;

    case 5:  // Flow, Supply Pressure, and SFM3505 Air
      hostCom.printf("Flow: %.2f SupplyP: %.2f SFM3505_Air: %.3f\n",
                     sensorData_bus0.flow,
                     sensorData_bus0.supply_pressure,
                     sensorData_bus0.sfm3505_air_flow);
      break;

    case 6:  // SFM3505 data from both buses
      hostCom.printf("[Bus0] O2: %.3f Air: %.3f | [Bus1] O2: %.3f Air: %.3f\n",
                     sensorData_bus0.sfm3505_o2_flow,
                     sensorData_bus0.sfm3505_air_flow,
                     sensorData_bus1.sfm3505_o2_flow,
                     sensorData_bus1.sfm3505_air_flow);
      break;
  }
}

void setup() {
  // ============================================================================
  // CRITICAL: SFM3505 Power-Up Reset - MUST BE FIRST!
  // ============================================================================
  // Per Sensirion documentation: SDA/SCL must be LOW for 31ms after power-on
  // This MUST happen immediately, before any other initialization

  // Step 1: Disable I2C pull-ups by controlling GPIO21
  pinMode(I2C0_PULLUP_CTRL_PIN, OUTPUT);
  digitalWrite(I2C0_PULLUP_CTRL_PIN, LOW);  // Turn OFF pull-ups via transistor

  // Step 2: Actively pull SDA/SCL LOW via GPIO
  pinMode(I2C0_SDA_PIN, OUTPUT);
  pinMode(I2C0_SCL_PIN, OUTPUT);
  digitalWrite(I2C0_SDA_PIN, LOW);
  digitalWrite(I2C0_SCL_PIN, LOW);

  // Step 3: Hold LOW for 35ms (> 31ms requirement)
  delay(35);

  // Initialize USB Serial - use default USB CDC
  Serial.begin();  // For ESP32-S3 USB CDC, no baud rate needed
  delay(2500);  // Give more time for Serial to initialize

  // Enable display power
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  delay(100);

  // Initialize display
  renderer.begin();
  renderer.showBootScreen(parallelVerLbl, __DATE__, __TIME__);
  delay(2000);

  // Initialize system configuration
  sysConfig.delta_t = 10000;          // Main loop: 10ms = 100Hz
  sysConfig.PressSamplTime = 10000;    // Pressure: 10ms = 100Hz
  sysConfig.quiet_mode = 0;            // Verbose output
  sysConfig.digital_flow_reference = 0.0;

  // Initialize command parser
  parser.begin(250000);

  // Print startup banner
  hostCom.println("\n╔═════════════════════════════════╗");
  hostCom.printf( "║  %s   ║\n", parallelVerLbl);
  hostCom.printf( "║  Build: %s %s      ║\n", __DATE__, __TIME__);
  hostCom.println("╚═══════════════════════════════════╝\n");

  // ============================================================================
  // Initialize I2C Bus 0 (Wire) - GPIO43/44
  // ============================================================================
  hostCom.printf("I2C Bus 0: SDA=GPIO%d, SCL=GPIO%d @ %dkHz\n",
                 I2C0_SDA_PIN, I2C0_SCL_PIN, I2C0_CLOCK_FREQ / 1000);

  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, I2C0_CLOCK_FREQ);
  digitalWrite(I2C0_PULLUP_CTRL_PIN, HIGH);  // Enable pull-ups via GPIO21
  delay(10);

  renderer.showLinesOnScreen("SCANNING", "I2C-bus", "");

  // Scan I2C bus for devices
  hostCom.println("Scanning I2C bus...");
  int deviceCount = 0;

  for (byte address = 0x01; address < 0x7F; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      hostCom.printf("  0x%02X", address);

      // Identify known devices
      if (address == 0x2E) hostCom.print(" - SFM3505 (flow)");
      else if (address == 0x28) hostCom.print(" - ABP2 (pressure)");
      else if (address == 0x60) hostCom.print(" - MCP4725 (DAC)");
      else if (address == 0x76) hostCom.print(" - BME280");

      hostCom.println();
      deviceCount++;
    }
    delay(5);
  }

  if (deviceCount == 0) {
    hostCom.println("  ⚠️ No I2C devices found!");
  } else {
    hostCom.printf("  ✅ Found %d device(s)\n", deviceCount);
  }
  hostCom.println();
  delay(1000);

  // Create sensor reader
  sensors_bus0 = new SensorReader(&Wire, "Bus0");

  // ============================================================================
  // Initialize Serial1 (external actuator communication)
  // ============================================================================
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  hostCom.printf("Serial1: TX=GPIO%d, RX=GPIO%d @ 115200 baud\n\n", SERIAL1_TX_PIN, SERIAL1_RX_PIN);

  // ============================================================================
  // Initialize sensors
  // ============================================================================
  interactionKey1.begin();

  sensorsInitialized = sensors_bus0->initialize();
  if (!sensorsInitialized) {
    hostCom.println("⚠️ Sensor initialization failed - check connections\n");
  }

  // ============================================================================
  // Initialize systems
  // ============================================================================
  wifiServer.start();
  actuator.initialize();

  past_time = micros();

  hostCom.println("╔═══════════════════════════════╗");
  hostCom.println("║         SYSTEM READY          ║");
  hostCom.println("╚═══════════════════════════════╝");
  hostCom.println("Commands: ? = help, ! = settings\n");
}

void loop() {
  // Initialize display on first loop
  static bool initLoop = false;
  if (!initLoop) {
    initLoop = true;
    renderer.clear();
    renderer.drawLabel();
    renderer.drawStatusField();
    renderer.drawWiFiField();
    renderer.drawWiFiAPIP("WiFi OFF      ", "No SSID        ");
    renderer.drawWiFiPromt("Press key to enable ");
  }

  // ============================================================================
  // ABP2 Pressure Sensor - Asynchronous Measurement (100Hz with 200Hz loop)
  // ============================================================================
  static uint32_t lastPressureTime = 0;
  static bool pressureCommandSent = false;
  uint32_t currentTime = micros();

  if (sensorsInitialized && (currentTime - lastPressureTime) >= sysConfig.PressSamplTime) {
    lastPressureTime = currentTime;

    if (!pressureCommandSent) {
      // Send measurement command (0xAA 0x00 0x00)
      sensors_bus0->startABP2Measurement();
      pressureCommandSent = true;
    } else {
      // Read result (5ms after command, which happens automatically due to 10ms cycle)
      float pressure;
      uint8_t status;
      if (sensors_bus0->readABP2Pressure(pressure, status)) {
        sensorData_bus0.supply_pressure = pressure;
      }
      pressureCommandSent = false;  // Next cycle will send command again
    }
  }

  // ============================================================================
  // Main control loop timing
  // ============================================================================

  if ((micros() - past_time) >= sysConfig.delta_t) {
    past_time = micros();

    // Only read sensors if they were successfully initialized
    if (sensorsInitialized) {
      // ========================================================================
      // Read SFM3505 O2 flow from Bus 0 (GPIO43/44)
      // ========================================================================
      float sfm_air, sfm_o2;
      if (sensors_bus0->readSFM3505AllFlows(sfm_air, sfm_o2)) {
        // Store in sensorData structure
        sensorData_bus0.sfm3505_air_flow = sfm_air;
        sensorData_bus0.sfm3505_o2_flow = sfm_o2;
      } else {
        // SFM3505 read failed - set to 0
        sensorData_bus0.sfm3505_air_flow = 0.0;
        sensorData_bus0.sfm3505_o2_flow = 0.0;
      }

      // Optional: Also read SFM3505 from Bus 1 if present - DISABLED FOR NOW
      // if (sensors_bus1->readSFM3505AllFlows(sfm_air, sfm_o2)) {
      //   sensorData_bus1.sfm3505_air_flow = sfm_air;
      //   sensorData_bus1.sfm3505_o2_flow = sfm_o2;
      // }

      // NOTE: ABP2 pressure is now read asynchronously above (see "ABP2 Pressure Sensor" section)
      // No need to call sensors_bus0->update() for pressure anymore

      // Output data based on quiet mode
      outputData();
    }

    // ========================================================================
    // Execute control - using Bus 0 as primary
    // Can use either legacy flow OR SFM3505 O2 flow
    // ========================================================================

    // Option 1: Use legacy flow sensor
    actuator.execute(sysConfig.digital_flow_reference, sensorData_bus0.flow, sysConfig.quiet_mode);

    // Option 2: Use SFM3505 O2 flow (uncomment to use)
    // actuator.execute(sysConfig.digital_flow_reference, sensorData_bus0.sfm3505_o2_flow, sysConfig.quiet_mode);

    // ========================================================================
    // Update WiFi server with Bus 0 data (primary)
    // ========================================================================
    wifiServer.updateFlow(sensorData_bus0.sfm3505_air_flow);  // Send SFM3505 Air flow to web
    wifiServer.updatePressure(sensorData_bus0.supply_pressure);  // Uses async ABP2 reading
    wifiServer.updateValveSignal(actuator.getValveControlSignal());
  }
  
  // ============================================================================
  // Handle Serial1 communication with external microcontroller
  // ============================================================================

  // Check for data from Serial1
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    hostCom.printf("[Serial1 RX]: %s\n", data.c_str());
    // Process data from external MCU
  }

  // Serial2 - DISABLED FOR NOW
  // if (Serial2.available()) {
  //   String data = Serial2.readStringUntil('\n');
  //   hostCom.printf("[Serial2 RX]: %s\n", data.c_str());
  // }

  // Send data to external microcontroller via Serial1 (including SFM3505 data)
  static uint32_t lastSerialSend = 0;
  if (millis() - lastSerialSend > 1000) {  // Send every 1 second
    lastSerialSend = millis();

    // Send to Serial1 - include SFM3505 Air flow
    Serial1.printf("Flow:%0.2f,Press:%0.2f,Air:%0.3f\n",
                   sensorData_bus0.flow,
                   sensorData_bus0.supply_pressure,
                   sensorData_bus0.sfm3505_air_flow);
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

    // Display Bus 0 data (primary) - now showing SFM3505 Air flow
    renderer.drawFlow(String(sensorData_bus0.sfm3505_air_flow, 3) + " slm Air");  // SFM3505 Air
    renderer.drawPressure(String(sensorData_bus0.supply_pressure, 2) + " kPa");
    renderer.drawValveCtrlSignal(String(actuator.getValveControlSignal()));
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
