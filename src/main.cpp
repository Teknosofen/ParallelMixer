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
    case 0:  // Verbose: dP, Flow, SupplyP, SFM3505, Valve signal
      hostCom.print("[Bus0] ");
      hostCom.print("dP:");
      hostCom.print(sensorData_bus0.differential_pressure, 2);
      hostCom.print(" Flow:");
      hostCom.print(sensorData_bus0.flow, 2);
      hostCom.print(" SupplyP:");
      hostCom.print(sensorData_bus0.supply_pressure, 2);
      hostCom.print(" SFM3505_O2:");
      hostCom.print(sensorData_bus0.sfm3505_o2_flow, 2);
      hostCom.print(" Valve:");
      hostCom.println(actuator.getValveControlSignal());
      break;

    case 1:  // Quiet - no output
      break;

    case 2:  // Debug: Integrator, Error, Valve, Flow
      {
        ControlState state = actuator.getControlState();
        hostCom.print("I ");
        hostCom.print(state.integrator, 1);
        hostCom.print(" E ");
        hostCom.print(state.error, 1);
        hostCom.print(" V ");
        hostCom.print(actuator.getValveControlSignal());
        hostCom.print(" F ");
        hostCom.print(sensorData_bus0.flow);
        hostCom.print(" O2 ");
        hostCom.println(sensorData_bus0.sfm3505_o2_flow);
      }
      break;

    case 3:  // Special - controlled by actuator.execute()
      break;

    case 4:  // Abbreviated: dP, Flow, Valve signal
      hostCom.print(sensorData_bus0.differential_pressure, 2);
      hostCom.print(" ");
      hostCom.print(sensorData_bus0.flow, 2);
      hostCom.print(" ");
      hostCom.println(actuator.getValveControlSignal());
      break;

    case 5:  // Flow and Supply Pressure + SFM3505
      hostCom.print("Flow: ");
      hostCom.print(sensorData_bus0.flow, 2);
      hostCom.print(" SupplyP: ");
      hostCom.print(sensorData_bus0.supply_pressure, 2);
      hostCom.print(" SFM3505_O2: ");
      hostCom.println(sensorData_bus0.sfm3505_o2_flow, 2);
      break;

    case 6:  // SFM3505 data from both buses
      hostCom.print("[Bus0] O2:");
      hostCom.print(sensorData_bus0.sfm3505_o2_flow, 2);
      hostCom.print(" Air:");
      hostCom.print(sensorData_bus0.sfm3505_air_flow, 2);
      hostCom.print(" | [Bus1] O2:");
      hostCom.print(sensorData_bus1.sfm3505_o2_flow, 2);
      hostCom.print(" Air:");
      hostCom.println(sensorData_bus1.sfm3505_air_flow, 2);
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
  Serial.println("\n=== Boot Start ===");
  Serial.println("✅ SFM3505 reset sequence applied:");
  Serial.println("   - Pull-ups disabled via GPIO21");
  Serial.println("   - SDA/SCL held LOW for 35ms from power-on");
  Serial.println(parallelVerLbl);
  Serial.print("Build: ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  Serial.flush();
  // Try multiple times to ensure Serial output works
  // for (int i = 0; i < 5; i++) {
  //   Serial.println("\n=== Boot Start ===");
  //   Serial.flush();
  //   delay(100);
  // }

  // CRITICAL: Enable display power first!
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  delay(100);

  Serial.println("Display power enabled");

  renderer.begin(); // Initialize display

  Serial.println("Display initialized");

  // Show boot screen with version info for 2 seconds
  renderer.showBootScreen(parallelVerLbl, __DATE__, __TIME__);
  delay(3000);

  Serial.println("Boot screen displayed");

  // Initialize system configuration with defaults
  sysConfig.delta_t = 100000;  // Sampling time in microseconds
  sysConfig.quiet_mode = 0;    // Verbose output
  sysConfig.digital_flow_reference = 0.0;  // Flow reference in L/min

  Serial.println("Config initialized");

  // Initialize USB Serial parser (already initialized above)
  parser.begin(250000);

  hostCom.println("\n=== P-Mixer with SFM3505 O2 Flow Sensor ===\n");

  // ============================================================================
  // Initialize I2C Buses BEFORE sensors
  // ============================================================================

  hostCom.println("Initializing I2C buses...");

  // Initialize I2C Bus 0 (Wire) - GPIO43/44 - SFM3505 on this bus
  // Note: Reset sequence was already applied at the very start of setup()
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, 100000);  // 100kHz for stability

  // NOW enable the pull-up resistors via GPIO21 (after I2C init)
  digitalWrite(I2C0_PULLUP_CTRL_PIN, HIGH);  // Turn ON pull-ups via transistor
  hostCom.printf("✅ I2C Bus 0: SDA=GPIO%d, SCL=GPIO%d (Pull-ups enabled via GPIO%d)\n",
                 I2C0_SDA_PIN, I2C0_SCL_PIN, I2C0_PULLUP_CTRL_PIN);
  delay(10);  // Additional settling time after pull-up enable

  // Quick scan of expected I2C addresses only
  hostCom.println("Scanning I2C bus - checking expected addresses...");
  int deviceCount = 0;

  // List of expected sensor addresses
  byte expectedAddresses[] = {0x2E, 0x25, 0x40, 0x58, 0x76};
  const char* deviceNames[] = {"SFM3505", "SPD sensor", "Legacy SFM", "SSC sensor", "BME280"};

  for (int i = 0; i < 5; i++) {
    byte address = expectedAddresses[i];
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      hostCom.printf("  ✅ Device found at address 0x%02X (%s)\n", address, deviceNames[i]);
      deviceCount++;
    } else {
      hostCom.printf("  ❌ No device at 0x%02X (%s) - Error: %d\n", address, deviceNames[i], error);
    }

    delay(5);  // Small delay for stability
  }

  if (deviceCount == 0) {
    hostCom.println("\n  ⚠️⚠️⚠️ NO I2C DEVICES FOUND! ⚠️⚠️⚠️");
    hostCom.println("  Despite signals on oscilloscope, no device is ACKing.");
    hostCom.println("  Possible causes:");
    hostCom.println("  1. Sensor in wrong mode or needs power cycle");
    hostCom.println("  2. Pull-up resistors too weak/strong");
    hostCom.println("  3. Timing issues - try different I2C speed");
    hostCom.println("  4. Sensor damaged");
  } else {
    hostCom.printf("\n  ✅ Found %d device(s) on I2C bus\n", deviceCount);
  }

  // I2C Bus 1 (Wire1) - GPIO10/11 - DISABLED FOR NOW
  // Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 100000);
  // hostCom.printf("✅ I2C Bus 1: SDA=GPIO%d, SCL=GPIO%d\n\n", I2C1_SDA_PIN, I2C1_SCL_PIN);

  // Now create sensor readers after I2C buses are initialized
  sensors_bus0 = new SensorReader(&Wire, "Bus0");
  // sensors_bus1 = new SensorReader(&Wire1, "Bus1");  // DISABLED FOR NOW
  hostCom.println("✅ Sensor reader objects created");

  // ============================================================================
  // Initialize Serial Ports for external microcontrollers
  // ============================================================================

  hostCom.println("Initializing Serial ports...");

  // Serial1 - External microcontroller communication
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  hostCom.printf("✅ Serial1: TX=GPIO%d, RX=GPIO%d\n\n", SERIAL1_TX_PIN, SERIAL1_RX_PIN);

  // Serial2 - DISABLED FOR NOW
  // Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
  // hostCom.printf("✅ Serial2: TX=GPIO%d, RX=GPIO%d\n\n", SERIAL2_TX_PIN, SERIAL2_RX_PIN);

  // ============================================================================
  // Initialize Sensors on both buses
  // ============================================================================

  interactionKey1.begin();

  hostCom.println("Initializing sensors...");

  // Initialize sensors on Bus 0 (includes SFM3505 at 0x2E)
  sensorsInitialized = sensors_bus0->initialize();
  if (!sensorsInitialized) {
    hostCom.println("❌ Bus 0 sensor initialization failed!");
    hostCom.println("   System will continue without sensors - connect SFM3505 and restart");
  }

  // Initialize sensors on Bus 1 - DISABLED FOR NOW
  // if (!sensors_bus1->initialize()) {
  //   hostCom.println("❌ Bus 1 sensor initialization failed!");
  // }

  hostCom.println();

  // ============================================================================
  // Initialize other systems
  // ============================================================================

  wifiServer.start();
  hostCom.println("✅ WiFi server started\n");

  actuator.initialize();
  hostCom.println("✅ Actuator initialized\n");

  // Start timing
  past_time = micros();

  hostCom.println("=== System initialized ===");
  hostCom.println("SFM3505 O2 flow will be read from Bus 0 (GPIO43/44)");
  hostCom.println("Type ? for help\n");
  
  // Test Serial1
  Serial1.println("Hello from ESP32 on Serial1");
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

    // Only read sensors if they were successfully initialized
    if (sensorsInitialized) {
      // ========================================================================
      // Read LEGACY sensors from Bus 0 only
      // ========================================================================
      sensors_bus0->update(sensorData_bus0);
      // sensors_bus1->update(sensorData_bus1);  // DISABLED FOR NOW

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
    wifiServer.updateFlow(sensorData_bus0.sfm3505_o2_flow);  // Send SFM3505 O2 flow to web
    wifiServer.updatePressure(sensorData_bus0.supply_pressure);
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

    // Send to Serial1 - include SFM3505 O2 flow
    Serial1.printf("Flow:%0.2f,Press:%0.2f,O2:%0.2f\n",
                   sensorData_bus0.flow,
                   sensorData_bus0.supply_pressure,
                   sensorData_bus0.sfm3505_o2_flow);
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
    
    // Display Bus 0 data (primary) - now showing SFM3505 O2 flow
    renderer.drawFlow(String(sensorData_bus0.sfm3505_o2_flow, 2) + " slm O2");  // SFM3505 O2
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
