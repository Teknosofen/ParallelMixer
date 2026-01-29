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
#include "SerialMuxRouter.hpp"

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

// Array of actuator controls - one per MUX channel for parallel operation
// Each channel can run its own signal generator independently
// Note: Pin parameter unused - all valve control via serial MUX commands
// Channels: 0 = direct (no prefix), 1-5 = MUX channels with prefix
static const uint8_t NUM_MUX_CHANNELS = 6;
ActuatorControl actuators[NUM_MUX_CHANNELS] = {
  ActuatorControl(0), ActuatorControl(0), ActuatorControl(0),
  ActuatorControl(0), ActuatorControl(0), ActuatorControl(0)
};
// Reference to currently selected actuator (for command interface)
#define actuator actuators[sysConfig.mux_channel]

CommandParser parser;
SerialMuxRouter muxRouter(&Serial1);

Button interactionKey1(INTERACTION_BUTTON_PIN);
PMixerWiFiServer wifiServer(PMIXERSSID, PMIXERPWD);

// System configuration
SystemConfig sysConfig;
SensorData sensorData_bus0;  // Data from Bus 0 sensors (includes SFM3505)
SensorData sensorData_bus1;  // Data from Bus 1 sensors

// Sensor initialization status
bool sensorsInitialized = false;

// Timing
uint32_t past_time;          // For GUI/serial output timing
uint32_t control_past_time;  // For control system execution timing

void outputData() {
  // Output data from Bus 0 (primary sensors)
  switch (sysConfig.quiet_mode) {
    case 0:  // Verbose: Supply Pressure, Low Pressure, Temp, Air flow, Valve signal
      hostCom.printf("P: %.2f\tLP: %.2f\tT: %.1f\tAir: %.3f\tValve: %.2f\n",
                     sensorData_bus0.supply_pressure,
                     sensorData_bus0.abpd_pressure,
                     sensorData_bus0.abpd_temperature,
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

    case 3:  // Special - controller mode + setting + Serial1 actuator data
      {
        // Get controller mode
        ControllerMode mode = actuator.getControllerMode();
        String modeStr;
        switch (mode) {
          case PID_CONTROL:
            modeStr = "PID";
            break;
          case VALVE_SET_VALUE_CONTROL:
            modeStr = "Set";
            break;
          case SINE_CONTROL:
            modeStr = "Sine";
            break;
          case STEP_CONTROL:
            modeStr = "Step";
            break;
          case TRIANGLE_CONTROL:
            modeStr = "Triangle";
            break;
          case SWEEP_CONTROL:
            modeStr = "Sweep";
            break;
          default:
            modeStr = "Unknown";
            break;
        }

        // Get valve control signal and MUX router data (selected channel)
        uint8_t ch = sysConfig.mux_channel;
        float localValveCtrl = actuator.getValveControlSignal();
        float actuatorCurrent = muxRouter.getCurrent(ch);

        // Print unified output: Mode, MUX channel, Setting, and Received data
        if (!muxRouter.isCurrentStale(ch)) {
          hostCom.printf("[%s M%d] V=%.2f%% -> I=%.3fA",
                         modeStr.c_str(), ch, localValveCtrl, actuatorCurrent);

          // Show other available measurements if not stale
          if (!muxRouter.isActualFlowStale(ch)) {
            hostCom.printf(" F=%.2f", muxRouter.getActualFlow(ch));
          }
          if (!muxRouter.isActualPressureStale(ch)) {
            hostCom.printf(" P=%.2f", muxRouter.getActualPressure(ch));
          }
          if (!muxRouter.isBlowerRPMStale(ch)) {
            hostCom.printf(" R=%.1f", muxRouter.getBlowerRPM(ch));
          }
          hostCom.printf("\n");
        } else {
          hostCom.printf("[%s M%d] V=%.2f%% -> Actuator: STALE data\n",
                         modeStr.c_str(), ch, localValveCtrl);
        }
      }
      break;

    case 4:  // Abbreviated: dP, Flow, Valve signal
      hostCom.printf("%.2f %.2f %.2f\n",
                     sensorData_bus0.differential_pressure,
                     sensorData_bus0.flow,
                     actuator.getValveControlSignal());
      break;

    case 5:  // Flow, Supply Pressure, Low Pressure, Temp, and SFM3505 Air
      hostCom.printf("Flow: %.2f SupplyP: %.2f LowP: %.2f Temp: %.1f SFM3505_Air: %.3f\n",
                     sensorData_bus0.flow,
                     sensorData_bus0.supply_pressure,
                     sensorData_bus0.abpd_pressure,
                     sensorData_bus0.abpd_temperature,
                     sensorData_bus0.sfm3505_air_flow);
      break;

    case 6:  // SFM3505 data from both buses
      hostCom.printf("[Bus0] O2: %.3f Air: %.3f | [Bus1] O2: %.3f Air: %.3f\n",
                     sensorData_bus0.sfm3505_o2_flow,
                     sensorData_bus0.sfm3505_air_flow,
                     sensorData_bus1.sfm3505_o2_flow,
                     sensorData_bus1.sfm3505_air_flow);
      break;

    // Supply Pressure (kPa, 2 decimals)
    // Low Pressure (kPa, 2 decimals)
    // Temperature (°C, 1 decimal)
    // Air Flow (L/min, 3 decimals)
    // Valve Control Signal (%, 2 decimals)
    // Actuator Current (A, 3 decimals)
    case 7:  // High-speed data logging: tab-separated, no labels (minimal bandwidth)
      {
        float localValveCtrl = actuator.getValveControlSignal();
        float actuatorCurrent = muxRouter.getCurrent(sysConfig.mux_channel);
        // Format: Timestamp_ms\tSupplyPressure\tLowPressure\tTemp\tAirFlow\tValveSignal\tCurrent\n
        hostCom.printf("%lu\t%.2f\t%.2f\t%.1f\t%.3f\t%.2f\t%.3f\n",
                       millis(),
                       sensorData_bus0.supply_pressure,
                       sensorData_bus0.abpd_pressure,
                       sensorData_bus0.abpd_temperature,
                       sensorData_bus0.sfm3505_air_flow,
                       localValveCtrl,
                       actuatorCurrent);
      }
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

  delay(2000);  // Give more time for Serial to initialize
  
  // Enable display power
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  
  // Initialize display
  renderer.begin();
  renderer.showBootScreen(parallelVerLbl, __DATE__, __TIME__);
  delay(2000);

  // Initialize system configuration
  sysConfig.delta_t = 100000;          // GUI/Serial output: 100ms = 10Hz (T command)
  sysConfig.control_interval = 10000;  // Control execution: 10ms = 100Hz (X command)
  sysConfig.PressSamplTime = 100000;   // Pressure: 100ms = 10Hz
  sysConfig.quiet_mode = 0;            // Verbose output
  sysConfig.digital_flow_reference = 0.0;
  sysConfig.mux_channel = 0;           // MUX channel 0 = direct (no MUX prefix)

  // Initialize command parser
  parser.begin(1000000);

  // Print startup banner
  hostCom.println("\n╔═══════════════════════════════════╗");
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
  // delay(10);

  // Scan I2C bus for devices
  hostCom.println("Scanning I2C bus...");
  int deviceCount = 0;
  String bus0Addresses = "";

  for (byte address = 0x01; address < 0x7F; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      hostCom.printf("  0x%02X", address);

      // Build address string for screen display
      if (deviceCount > 0) bus0Addresses += " ";
      bus0Addresses += "0x";
      if (address < 0x10) bus0Addresses += "0";
      bus0Addresses += String(address, HEX);

      // Identify known devices
      if (address == 0x2E) hostCom.print(" - SFM3505 (flow)");
      else if (address == 0x28) hostCom.print(" - ABP2 (pressure)");
      else if (address == 0x60) hostCom.print(" - MCP4725 (DAC)");
      else if (address == 0x76) hostCom.print(" - BME280");

      // ABPDLNN100MG2A3 has address 0x18
      else if (address == 0x18) hostCom.print(" - ABPDLNN100MG2A3 (pressure)"); // wrong address, unfortunately it also whas 0x28
      else hostCom.print(" - unknown device");
      
      hostCom.println();
      deviceCount++;
    }
    delay(5);
  }

  if (deviceCount == 0) {
    hostCom.println("  ⚠️ No I2C devices found!");
    bus0Addresses = "None found";
  } else {
    hostCom.printf("  ✅ Found %d device(s)\n", deviceCount);
  }
  hostCom.println();

  // Display I2C scan results on screen
  String line2 = "Bus 0: " + bus0Addresses;
  renderer.showLinesOnScreen("I2C devices found", line2.c_str(), "Bus 1: N/A");
  hostCom.printf("I2C Bus 0 addr: %s\n", bus0Addresses.c_str());
  delay(2000);

  // Create sensor reader
  sensors_bus0 = new SensorReader(&Wire, "Bus0");

  // ============================================================================
  // Initialize Serial1 (external actuator communication)
  // ============================================================================
  Serial1.begin(460800, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN); // was 115200
  hostCom.printf("Serial1: TX=GPIO%d, RX=GPIO%d @ 460800 baud\n\n", SERIAL1_TX_PIN, SERIAL1_RX_PIN);

  // Initialize asynchronous serial reader for actuator data
  muxRouter.begin();

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
  // WiFi starts disabled - use long key press to enable

  // Initialize all actuator controls and link to MUX router
  for (uint8_t i = 0; i < NUM_MUX_CHANNELS; i++) {
    actuators[i].initialize();
    actuators[i].setSerialMuxRouter(&muxRouter, i);
  }
  hostCom.printf("Initialized %d MUX channels for parallel operation\n", NUM_MUX_CHANNELS);

  past_time = micros();
  control_past_time = micros();

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
    renderer.drawWiFiAPIP("WiFi OFF      ", "No SSID    ");
    renderer.drawWiFiPromt("Long press: enable");
  }

  // ============================================================================
  // ABP2 Pressure Sensor - Asynchronous Measurement (100Hz with 200Hz loop)
  // ============================================================================
  static uint32_t lastPressureTime = 0;
  static bool pressureCommandSent = false;
  uint32_t currentTime = micros();

  // Check if it's time to handle pressure measurement (only if ABP2 detected)
  if (sensorsInitialized && sensors_bus0->hasABP2() && (currentTime - lastPressureTime) >= sysConfig.PressSamplTime) {
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
  } else if (!sensors_bus0->hasABP2()) {
    // No ABP2 detected - set to invalid indicator
    sensorData_bus0.supply_pressure = -9.9;
  }

  // ============================================================================
  // Control system execution timing (X command - typically faster)
  // ============================================================================

  if ((micros() - control_past_time) >= sysConfig.control_interval) {
    control_past_time = micros();

    // ========================================================================
    // Read SFM3505 Flow Sensor (now in fast control loop)
    // ========================================================================
    if (sensorsInitialized && sensors_bus0->hasSFM3505()) {
      float sfm_air, sfm_o2;
      if (sensors_bus0->readSFM3505AllFlows(sfm_air, sfm_o2)) {
        sensorData_bus0.sfm3505_air_flow = sfm_air;
        sensorData_bus0.sfm3505_o2_flow = sfm_o2;
      } else {
        sensorData_bus0.sfm3505_air_flow = -9.9;
        sensorData_bus0.sfm3505_o2_flow = -9.9;
      }
    } else if (!sensors_bus0->hasSFM3505()) {
      // No SFM3505 detected - set to invalid indicator
      sensorData_bus0.sfm3505_air_flow = -9.9;
      sensorData_bus0.sfm3505_o2_flow = -9.9;
    }

    // ========================================================================
    // Read ABPD Low Pressure Sensor (now in fast control loop)
    // ========================================================================
    static uint32_t lastABPDTime = 0;
    if (sensorsInitialized && sensors_bus0->hasABPD() && (currentTime - lastABPDTime) >= sysConfig.PressSamplTime) {
      lastABPDTime = currentTime;

      float abpd_pressure, abpd_temp;
      uint8_t abpd_status;
      if (sensors_bus0->readABPDPressureTemp(abpd_pressure, abpd_temp, abpd_status)) {
        sensorData_bus0.abpd_pressure = abpd_pressure;
        sensorData_bus0.abpd_temperature = abpd_temp;
      } else {
        sensorData_bus0.abpd_pressure = -9.9;
        sensorData_bus0.abpd_temperature = -9.9;
      }
    } else if (!sensors_bus0->hasABPD()) {
      // No ABPD detected - set to invalid indicator
      sensorData_bus0.abpd_pressure = -9.9;
      sensorData_bus0.abpd_temperature = -9.9;
    }

    // ========================================================================
    // Execute control for ALL MUX channels in parallel
    // Each channel runs its own signal generator independently
    // ========================================================================
    for (uint8_t i = 0; i < NUM_MUX_CHANNELS; i++) {
      // Note: For now, all channels share the same flow reference and sensor data
      // In future, each channel could have independent references
      actuators[i].execute(sysConfig.digital_flow_reference, sensorData_bus0.flow, sysConfig.quiet_mode);
    }

    // ========================================================================
    // Update WiFi buffer at high-speed control rate (for 50+ Hz web updates)
    // ========================================================================


    
    // Log data from the selected MUX channel
    wifiServer.addDataPoint(sensorData_bus0.sfm3505_air_flow,
                           sensorData_bus0.supply_pressure,
                           actuator.getValveControlSignal(),
                           muxRouter.getCurrent(sysConfig.mux_channel),
                           sensorData_bus0.abpd_pressure,
                           sensorData_bus0.abpd_temperature);
  }

  // ============================================================================
  // GUI/Serial output timing (T command - typically slower)
  // ============================================================================

  if ((micros() - past_time) >= sysConfig.delta_t) {
    past_time = micros();

    // Only output data if sensors were successfully initialized
    // NOTE: Sensor reading now happens in fast control loop (control_interval)
    // NOTE: ABP2 and ABPD pressure sensors are read asynchronously above
    if (sensorsInitialized) {
      // Output data based on quiet mode
      outputData();
    }

    // ========================================================================
    // Update WiFi server with Bus 0 data (primary)
    // ========================================================================
    wifiServer.updateFlow(sensorData_bus0.sfm3505_air_flow);  // Send SFM3505 Air flow to web
    wifiServer.updatePressure(sensorData_bus0.supply_pressure);  // Uses async ABP2 reading
    wifiServer.updateLowPressure(sensorData_bus0.abpd_pressure);  // Send ABPD low pressure to web
    wifiServer.updateTemperature(sensorData_bus0.abpd_temperature);  // Send ABPD temperature to web
    wifiServer.updateValveSignal(actuator.getValveControlSignal());
    wifiServer.updateCurrent(muxRouter.getCurrent(sysConfig.mux_channel));  // Send current from selected MUX channel to web
  }
  
  // ============================================================================
  // Handle Serial1 communication with external microcontroller (Asynchronous)
  // ============================================================================

  // Update asynchronous serial reader - processes incoming data non-blocking
  muxRouter.update();

  // Serial2 - DISABLED FOR NOW
  // if (Serial2.available()) {
  //   String data = Serial2.readStringUntil('\n');
  //   hostCom.printf("[Serial2 RX]: %s\n", data.c_str());
  // }

  // Serial1 actuator data output is now in outputData() function for quiet_mode == 3

  // ============================================================================
  // Command parser and UI updates
  // ============================================================================

  parser.update();
  parser.processCommands(sysConfig, actuator);

  // Handle MUX channel change - switch to different actuator instance
  static uint8_t lastMuxChannel = 0;
  if (sysConfig.mux_channel != lastMuxChannel) {
    ControllerMode mode = actuators[sysConfig.mux_channel].getControllerMode();
    hostCom.printf("Switched to MUX channel %d (mode: %d)\n", sysConfig.mux_channel, mode);
    lastMuxChannel = sysConfig.mux_channel;
  }

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
      case SWEEP_CONTROL:
        ctrlMode = "Sweep";
        break;
      default:
        ctrlMode = "Unknown";
        break;
    }
    
    // Include MUX channel in mode display
    String modeWithChannel = ctrlMode + " M" + String(sysConfig.mux_channel);
    renderer.drawControllerMode(modeWithChannel);
    wifiServer.updateMode(modeWithChannel);

    // Display Bus 0 data (primary) - now showing SFM3505 Air flow and ABPD data
    renderer.drawFlow(String(sensorData_bus0.sfm3505_air_flow, 3) + " slm Air");  // SFM3505 Air
    // Show both supply pressure (ABP2) and low pressure (ABPD) with temperature
    String pressureStr = "HP: " + String(sensorData_bus0.supply_pressure, 1) + " LP: " +
                         String(sensorData_bus0.abpd_pressure, 1) + " " +
                         String(sensorData_bus0.abpd_temperature, 1) + " C";
    renderer.drawPressure(pressureStr);
    renderer.drawValveCtrlSignal(String(actuator.getValveControlSignal()));
    // Show current from selected MUX channel
    renderer.drawCurrent(String(muxRouter.getCurrent(sysConfig.mux_channel), 3) + " A");
  }

  // ============================================================================
  // WiFi control button handling
  // ============================================================================
  
  if (interactionKey1.wasReleased()) {
    if (interactionKey1.wasLongPress()) {
      hostCom.println("Key1 long press (>1s) - Enabling WiFi");
      wifiServer.start();
      renderer.drawWiFiAPIP(wifiServer.getApIpAddress(), PMIXERSSID);
      renderer.drawWiFiPromt("Short press: disable");
    } else {
      hostCom.println("Key1 short press - Disabling WiFi");
      wifiServer.stop();
      hostCom.println("WiFi Access Point stopped");
      renderer.drawWiFiAPIP("WiFi OFF", "No SSID");
      renderer.drawWiFiPromt("Long press: enable");
    }
  }
  
  // Handle web requests
  wifiServer.handleClient();
}
