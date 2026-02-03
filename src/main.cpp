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
#include "FDO2_Sensor.h"
#include "VentilatorController.hpp"

// ---------------------------------
// logo BMP file as well as factory default config files arcan be stored in SPIFFS
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
SensorReader* sensors_bus0;   // GPIO43/44 - includes SFM3505, ABP2, ABPD
SensorReader* sensors_bus1;   // GPIO10/11 - includes SFM3505, ABP2

// Array of actuator controls - one per MUX channel for parallel operation
// Each channel can run its own signal generator independently
// Note: Pin parameter unused - all valve control via serial MUX commands
// Channels: 0 = direct (no prefix), 1-5 = MUX channels with prefix
static const uint8_t NUM_MUX_CHANNELS = 6;
ActuatorControl actuators[NUM_MUX_CHANNELS];
// Reference to currently selected actuator (for command interface)
#define actuator actuators[sysConfig.mux_channel]

CommandParser parser;
SerialMuxRouter muxRouter(&Serial1);
VentilatorController ventilator;

Button interactionKey1(INTERACTION_BUTTON_PIN);
Button interactionKey2(BOOT_BUTTON_PIN);  // Boot button for settings display
PMixerWiFiServer wifiServer(PMIXERSSID, PMIXERPWD);

// Display mode: false = live data, true = ventilator settings
bool showVentilatorSettings = false;

// FDO2 Optical Oxygen Sensor on Serial2
FDO2_Sensor fdo2Sensor(Serial2);
FDO2_Sensor::MeasurementData fdo2Data;
bool fdo2Initialized = false;

// System configuration
SystemConfig sysConfig;
SensorData sensorData_bus0;  // Data from Bus 0 sensors (includes SFM3505)
SensorData sensorData_bus1;  // Data from Bus 1 sensors

// Sensor initialization status
bool sensorsInitialized = false;      // Bus 0 sensors
bool sensors_bus1_initialized = false; // Bus 1 sensors

// Timing
uint32_t past_time;          // For GUI/serial output timing
uint32_t control_past_time;  // For control system execution timing

void outputData() {
  // Output data from Bus 0 (primary sensors)
  switch (sysConfig.quiet_mode) {
    case 0:  // Verbose: Supply Pressure, Low Pressure, Temp, Air flow (Bus0 & Bus1), Valve signal
      hostCom.printf("P: %.2f\tLP: %.2f\tT: %.1f\tAir0: %.3f\tAir1: %.3f\tP1: %.2f\tValve: %.2f\n",
                     sensorData_bus0.supply_pressure,
                     sensorData_bus0.abpd_pressure,
                     sensorData_bus0.abpd_temperature,
                     sensorData_bus0.sfm3505_air_flow,
                     sensorData_bus1.sfm3505_air_flow,
                     sensorData_bus1.supply_pressure,
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

    case 5:  // Labeled: same data as q6, with labels for readability
      {
        float localValveCtrl = actuator.getValveControlSignal();
        float actuatorCurrent = muxRouter.getCurrent(sysConfig.mux_channel);
        float o2_hPa = fdo2Initialized ? fdo2Data.oxygenPartialPressure_hPa : -9.9f;
        float o2_percent = fdo2Initialized ?
          fdo2Sensor.convertToPercentO2(fdo2Data.oxygenPartialPressure_hPa, fdo2Data.ambientPressure_mbar) : -9.9f;
        hostCom.printf("SP0:%.2f LP:%.2f T:%.1f Air0:%.3f Air1:%.3f SP1:%.2f V:%.2f I:%.3f O2:%.2fhPa %.2f%%\n",
                       sensorData_bus0.supply_pressure,
                       sensorData_bus0.abpd_pressure,
                       sensorData_bus0.abpd_temperature,
                       sensorData_bus0.sfm3505_air_flow,
                       sensorData_bus1.sfm3505_air_flow,
                       sensorData_bus1.supply_pressure,
                       localValveCtrl,
                       actuatorCurrent,
                       o2_hPa,
                       o2_percent);
      }
      break;

    // High-speed TSV: Time_ms, SupplyP0, LowP, Temp, Air0, Air1, SupplyP1, Valve%, Current, O2_hPa, O2%
    case 6:  // High-speed data logging: tab-separated, no labels (minimal bandwidth)
      {
        float localValveCtrl = actuator.getValveControlSignal();
        float actuatorCurrent = muxRouter.getCurrent(sysConfig.mux_channel);
        float o2_hPa = fdo2Initialized ? fdo2Data.oxygenPartialPressure_hPa : -9.9f;
        float o2_percent = fdo2Initialized ?
          fdo2Sensor.convertToPercentO2(fdo2Data.oxygenPartialPressure_hPa, fdo2Data.ambientPressure_mbar) : -9.9f;
        hostCom.printf("%lu\t%.2f\t%.2f\t%.1f\t%.3f\t%.3f\t%.2f\t%.2f\t%.3f\t%.2f\t%.2f\n",
                       millis(),
                       sensorData_bus0.supply_pressure,
                       sensorData_bus0.abpd_pressure,
                       sensorData_bus0.abpd_temperature,
                       sensorData_bus0.sfm3505_air_flow,
                       sensorData_bus1.sfm3505_air_flow,
                       sensorData_bus1.supply_pressure,
                       localValveCtrl,
                       actuatorCurrent,
                       o2_hPa,
                       o2_percent);
      }
      break;

    case 7:  // FDO2 Oxygen Sensor data
      if (fdo2Initialized) {
        hostCom.printf("O2: %.2f hPa (%.2f%%) | Temp: %.1f°C | Status: 0x%02X %s\n",
                       fdo2Data.oxygenPartialPressure_hPa,
                       fdo2Sensor.convertToPercentO2(fdo2Data.oxygenPartialPressure_hPa, 
                                                      fdo2Data.ambientPressure_mbar),
                       fdo2Data.temperature_C,
                       fdo2Data.status,
                       fdo2Sensor.getStatusString(fdo2Data.status).c_str());
      } else {
        hostCom.println("FDO2 not initialized");
      }
      break;

    case 8:  // FDO2 Extended/Raw data
      if (fdo2Initialized) {
        hostCom.printf("O2: %.2f hPa | T: %.1f°C | Phase: %.3f° | Signal: %.1f mV | Amb: %.1f mV | P: %.1f mbar | RH: %.1f%%\n",
                       fdo2Data.oxygenPartialPressure_hPa,
                       fdo2Data.temperature_C,
                       fdo2Data.phaseShift_deg,
                       fdo2Data.signalIntensity_mV,
                       fdo2Data.ambientLight_mV,
                       fdo2Data.ambientPressure_mbar,
                       fdo2Data.relativeHumidity_percent);
      } else {
        hostCom.println("FDO2 not initialized");
      }
      break;
  }
}

void setup() {
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

  // Create sensor reader for Bus 0
  sensors_bus0 = new SensorReader(&Wire, "Bus0", I2C0_CLOCK_FREQ);

  // ============================================================================
  // Initialize I2C Bus 1 (Wire1) - GPIO10/11
  // ============================================================================
  hostCom.printf("\nI2C Bus 1: SDA=GPIO%d, SCL=GPIO%d @ %dkHz\n",
                 I2C1_SDA_PIN, I2C1_SCL_PIN, I2C0_CLOCK_FREQ / 1000);

  Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, I2C0_CLOCK_FREQ);

  // Scan I2C Bus 1 for devices
  hostCom.println("Scanning I2C Bus 1...");
  int bus1DeviceCount = 0;
  String bus1Addresses = "";

  for (byte address = 0x01; address < 0x7F; address++) {
    Wire1.beginTransmission(address);
    if (Wire1.endTransmission() == 0) {
      hostCom.printf("  0x%02X", address);

      // Build address string for screen display
      if (bus1DeviceCount > 0) bus1Addresses += " ";
      bus1Addresses += "0x";
      if (address < 0x10) bus1Addresses += "0";
      bus1Addresses += String(address, HEX);

      // Identify known devices
      if (address == 0x2E) hostCom.print(" - SFM3505 (flow)");
      else if (address == 0x28) hostCom.print(" - ABP2 (pressure)");
      else if (address == 0x18) hostCom.print(" - ABPDLNN100MG2A3 (pressure)");
      else if (address == 0x60) hostCom.print(" - MCP4725 (DAC)");
      else hostCom.print(" - unknown device");

      hostCom.println();
      bus1DeviceCount++;
    }
    delay(5);
  }

  if (bus1DeviceCount == 0) {
    hostCom.println("  ⚠️ No I2C devices found on Bus 1!");
    bus1Addresses = "None";
  } else {
    hostCom.printf("  ✅ Found %d device(s) on Bus 1\n", bus1DeviceCount);
  }
  hostCom.println();

  // Display I2C scan results on screen (both buses)
  String line2 = "Bus0: " + bus0Addresses;
  String line3 = "Bus1: " + bus1Addresses;
  renderer.showLinesOnScreen("I2C devices found", line2.c_str(), line3.c_str());
  hostCom.printf("I2C Bus 0 addr: %s\n", bus0Addresses.c_str());
  hostCom.printf("I2C Bus 1 addr: %s\n", bus1Addresses.c_str());
  delay(2000);

  // Create sensor reader for Bus 1
  sensors_bus1 = new SensorReader(&Wire1, "Bus1", I2C0_CLOCK_FREQ);

  // ============================================================================
  // Initialize Serial1 (external actuator communication)
  // ============================================================================
  
  // Initialize Serial1 for actuator communication via MUX
  Serial1.begin(230400, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN); // was 115200
  hostCom.printf("Serial1: TX=GPIO%d, RX=GPIO%d @ 230400 baud\n\n", SERIAL1_TX_PIN, SERIAL1_RX_PIN);

  // Initialize asynchronous serial reader for actuator data
  muxRouter.begin();

  // ============================================================================
  // Initialize Serial2 (FDO2 Optical Oxygen Sensor)
  // ============================================================================
  
  // Serial2 pins are defined in PinConfig.h:
  // SERIAL2_TX_PIN = GPIO12, SERIAL2_RX_PIN = GPIO13
  
  hostCom.println("Initializing FDO2 Optical Oxygen Sensor on Serial2...");
  hostCom.printf("Serial2: TX=GPIO%d, RX=GPIO%d @ 19200 baud\n", SERIAL2_TX_PIN, SERIAL2_RX_PIN);
  
  fdo2Initialized = fdo2Sensor.begin(19200, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
  
  if (fdo2Initialized) {
    FDO2_Sensor::DeviceInfo info;
    if (fdo2Sensor.getDeviceInfo(info)) {
      hostCom.printf("✅ FDO2 Connected: Device ID=%d, FW=%d.%02d, Channels=%d\n", 
                     info.deviceId, info.firmwareRevision / 100, info.firmwareRevision % 100, info.numChannels);
      
      uint64_t uniqueId;
      if (fdo2Sensor.getUniqueId(uniqueId)) {
        hostCom.printf("   Serial Number: %llu\n", uniqueId);
      }
      
      // Flash LED to confirm communication
      fdo2Sensor.indicateLogo();
    } else {
      hostCom.println("⚠️ FDO2 communication error");
      fdo2Initialized = false;
    }
  } else {
    hostCom.println("⚠️ FDO2 initialization failed - check Serial2 connections\n");
  }
  hostCom.println();

  // ============================================================================
  // Initialize sensors on both I2C buses
  // ============================================================================
  interactionKey1.begin();
  interactionKey2.begin();  // Boot button for settings display

  // Initialize Bus 0 sensors
  sensorsInitialized = sensors_bus0->initialize();
  if (!sensorsInitialized) {
    hostCom.println("⚠️ Bus 0 sensor initialization failed - check connections\n");
  }

  // Initialize Bus 1 sensors
  sensors_bus1_initialized = sensors_bus1->initialize();
  if (!sensors_bus1_initialized) {
    hostCom.println("⚠️ Bus 1 sensor initialization failed - check connections\n");
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

  // Initialize ventilator controller (HLC)
  ventilator.begin(&muxRouter);
  hostCom.println("Ventilator HLC initialized (use VS for status, VO1 to start)");

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
    // Read Bus 1 SFM3505 Flow Sensor
    // ========================================================================
    if (sensors_bus1_initialized && sensors_bus1->hasSFM3505()) {
      float sfm_air, sfm_o2;
      if (sensors_bus1->readSFM3505AllFlows(sfm_air, sfm_o2)) {
        sensorData_bus1.sfm3505_air_flow = sfm_air;
        sensorData_bus1.sfm3505_o2_flow = sfm_o2;
      } else {
        sensorData_bus1.sfm3505_air_flow = -9.9;
        sensorData_bus1.sfm3505_o2_flow = -9.9;
      }
    } else if (!sensors_bus1_initialized || !sensors_bus1->hasSFM3505()) {
      sensorData_bus1.sfm3505_air_flow = -9.9;
      sensorData_bus1.sfm3505_o2_flow = -9.9;
    }

    // ========================================================================
    // Read Bus 1 ABP2 Pressure Sensor (Asynchronous)
    // ========================================================================
    static uint32_t lastBus1PressureTime = 0;
    static bool bus1PressureCommandSent = false;

    if (sensors_bus1_initialized && sensors_bus1->hasABP2() && (currentTime - lastBus1PressureTime) >= sysConfig.PressSamplTime) {
      lastBus1PressureTime = currentTime;

      if (!bus1PressureCommandSent) {
        sensors_bus1->startABP2Measurement();
        bus1PressureCommandSent = true;
      } else {
        float pressure;
        uint8_t status;
        if (sensors_bus1->readABP2Pressure(pressure, status)) {
          sensorData_bus1.supply_pressure = pressure;
        }
        bus1PressureCommandSent = false;
      }
    } else if (!sensors_bus1_initialized || !sensors_bus1->hasABP2()) {
      sensorData_bus1.supply_pressure = -9.9;
    }

    // ========================================================================
    // Read FDO2 Optical Oxygen Sensor - ASYNC (non-blocking)
    // Sample rate: FDO2_SAMPLE_TIME_US (default 500ms = 2Hz, set in main.hpp)
    // State machine: START -> WAIT_RESPONSE -> PROCESS -> IDLE -> START...
    // ========================================================================
    static uint32_t lastFDO2StartTime = 0;
    static bool fdo2MeasurementPending = false;

    if (fdo2Initialized) {
      // Check if response is ready (non-blocking)
      if (fdo2MeasurementPending && fdo2Sensor.isResponseReady()) {
        // Parse the result
        if (fdo2Sensor.getAsyncResult(fdo2Data)) {
          // Check for fatal errors
          if (fdo2Sensor.hasFatalError(fdo2Data.status)) {
            if (sysConfig.quiet_mode == 0) {
              hostCom.printf("⚠️ FDO2 Fatal Error: %s\n",
                             fdo2Sensor.getStatusString(fdo2Data.status).c_str());
            }
          }
          // Optionally log warnings in verbose mode
          else if (sysConfig.quiet_mode == 0 && fdo2Sensor.hasWarning(fdo2Data.status)) {
            hostCom.printf("⚠️ FDO2 Warning: %s\n",
                           fdo2Sensor.getStatusString(fdo2Data.status).c_str());
          }
        }
        fdo2MeasurementPending = false;
      }

      // Start new measurement at FDO2_SAMPLE_TIME_US intervals (if not already pending)
      if (!fdo2MeasurementPending && (currentTime - lastFDO2StartTime) >= FDO2_SAMPLE_TIME_US) {
        lastFDO2StartTime = currentTime;
        if (fdo2Sensor.startMeasurementAsync(true)) {  // true = include raw data
          fdo2MeasurementPending = true;
        }
      }
    }

    // ========================================================================
    // Ventilator HLC Update (if running)
    // ========================================================================
    if (ventilator.isRunning()) {
      // Gather measurements for ventilator
      VentilatorMeasurements ventMeas;
      ventMeas.airwayPressure_mbar = sensorData_bus0.abpd_pressure;
      ventMeas.inspFlow_slm = sensorData_bus0.sfm3505_air_flow;
      ventMeas.expFlow_slm = 0;  // TODO: Add expiratory flow sensor if available
      ventMeas.deliveredO2_percent = fdo2Initialized ?
        fdo2Sensor.convertToPercentO2(fdo2Data.oxygenPartialPressure_hPa, fdo2Data.ambientPressure_mbar) : 21.0f;

      // Update ventilator state machine
      ventilator.update(ventMeas, currentTime);

      // Apply ventilator outputs to actuators (MUX channels 1-3)
      VentilatorOutputs ventOut = ventilator.getOutputs();

      // Channel 1: Air valve - flow setpoint
      muxRouter.sendSetFlow(MUX_AIR_VALVE, ventOut.airValveFlow_slm);

      // Channel 2: O2 valve - flow setpoint
      muxRouter.sendSetFlow(MUX_O2_VALVE, ventOut.o2ValveFlow_slm);

      // Channel 3: Exp valve - pressure or position control
      if (ventOut.expValveClosed) {
        muxRouter.sendCommand(MUX_EXP_VALVE, 'V', 0);  // Close valve
      } else if (ventOut.expValveAsPressure) {
        muxRouter.sendSetPressure(MUX_EXP_VALVE, ventOut.expValveSetpoint);
      } else {
        muxRouter.sendCommand(MUX_EXP_VALVE, 'V', ventOut.expValveSetpoint);
      }
    }

    // ========================================================================
    // Execute control for ALL MUX channels in parallel (when ventilator not running)
    // Each channel runs its own signal generator independently
    // ========================================================================
    if (!ventilator.isRunning()) {
      for (uint8_t i = 0; i < NUM_MUX_CHANNELS; i++) {
        // Note: For now, all channels share the same flow reference and sensor data
        // In future, each channel could have independent references
        actuators[i].execute(sysConfig.digital_flow_reference, sensorData_bus0.flow, sysConfig.quiet_mode);
      }
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
                           sensorData_bus0.abpd_temperature,
                           sensorData_bus1.sfm3505_air_flow,
                           sensorData_bus1.supply_pressure);
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

    // ========================================================================
    // Update WiFi server with Bus 1 data
    // ========================================================================
    wifiServer.updateFlow2(sensorData_bus1.sfm3505_air_flow);  // Send Bus 1 SFM3505 Air flow to web
    wifiServer.updatePressure2(sensorData_bus1.supply_pressure);  // Send Bus 1 ABP2 pressure to web

    // ========================================================================
    // Update WiFi server with ventilator settings
    // ========================================================================
    VentilatorConfig cfg = ventilator.getConfig();
    VentilatorStatus st = ventilator.getStatus();
    wifiServer.updateVentilatorSettings(
        ventilator.isRunning(),
        ventilator.getStateString(),
        cfg.respRate,
        cfg.tidalVolume_mL,
        cfg.ieRatio,
        cfg.maxPressure_mbar,
        cfg.peep_mbar,
        cfg.maxInspFlow_slm,
        cfg.targetFiO2,
        st.breathCount,
        st.peakPressure_mbar,
        st.measuredVt_mL
    );
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
  // Try ventilator commands first (two-character commands)
  if (!parser.processVentilatorCommands(ventilator)) {
    // If not a ventilator command, try regular commands
    parser.processCommands(sysConfig, actuator);
  }

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

    // Only update live data display when not showing ventilator settings
    if (!showVentilatorSettings) {
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
      renderer.drawFlow(String(sensorData_bus0.sfm3505_air_flow, 3) + " slm Air");  // SFM3505 Air Bus 0
      renderer.drawFlow2(String(sensorData_bus1.sfm3505_air_flow, 3) + " slm Air");  // SFM3505 Air Bus 1
      // Show Bus 0 supply pressure (ABP2) in bar, low pressure (ABPD), and temperature
      String pressureStr = "P0: " + String(sensorData_bus0.supply_pressure / 100.0, 2) + " LP: " +
                           String(sensorData_bus0.abpd_pressure, 1) + " " +
                           String(sensorData_bus0.abpd_temperature, 1) + " C";
      renderer.drawPressure(pressureStr);
      // Show Bus 1 supply pressure (ABP2)
      renderer.drawPressure2(String(sensorData_bus1.supply_pressure / 100.0, 2) + " bar");
      renderer.drawValveCtrlSignal(String(actuator.getValveControlSignal()));
      // Show current from selected MUX channel
      renderer.drawCurrent(String(muxRouter.getCurrent(sysConfig.mux_channel), 3) + " A");
    }
  }

  // ============================================================================
  // O2 sensor display update (slower rate - 0.5 second interval)
  // ============================================================================
  static uint32_t O2DisplayLoopStartTime = micros();
  if (micros() - O2DisplayLoopStartTime > FDO2_SAMPLE_TIME_US) {
    O2DisplayLoopStartTime = micros();

    // Update O2 display only when not showing ventilator settings
    if (!showVentilatorSettings) {
      if (fdo2Initialized) {
        renderer.drawO2(String(fdo2Data.oxygenPartialPressure_hPa, 1) + " hPa");
      } else {
        renderer.drawO2("N/A");
      }
    }
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

  // ============================================================================
  // Boot button handling - Ventilator settings display
  // ============================================================================
  if (interactionKey2.wasReleased()) {
    if (interactionKey2.wasLongPress()) {
      // Long press: Show ventilator settings
      hostCom.println("Key2 long press - Showing ventilator settings");
      showVentilatorSettings = true;

      // Get ventilator config and status
      VentilatorConfig cfg = ventilator.getConfig();
      VentilatorStatus st = ventilator.getStatus();

      // Format the settings strings
      String line1 = String("Vent: ") + (ventilator.isRunning() ? "ON" : "OFF") +
                     "  " + ventilator.getStateString();
      String line2 = "RR=" + String(cfg.respRate, 1) + " VT=" + String(cfg.tidalVolume_mL, 0) +
                     " IE=" + String(cfg.ieRatio, 2);
      String line3 = "PI=" + String(cfg.maxPressure_mbar, 1) + " PE=" + String(cfg.peep_mbar, 1) +
                     " MF=" + String(cfg.maxInspFlow_slm, 1);
      String line4;
      if (ventilator.isRunning()) {
        line4 = "#" + String(st.breathCount) + " PkP=" + String(st.peakPressure_mbar, 1) +
                " Vt=" + String(st.measuredVt_mL, 0);
      } else {
        line4 = "FiO2=" + String(cfg.targetFiO2 * 100, 0) + "%";
      }

      renderer.showVentilatorSettings(line1.c_str(), line2.c_str(), line3.c_str(), line4.c_str());
    } else {
      // Short press: Return to live data
      if (showVentilatorSettings) {
        hostCom.println("Key2 short press - Returning to live data");
        showVentilatorSettings = false;

        // Redraw the normal UI
        renderer.clear();
        renderer.drawLabel();
        renderer.drawStatusField();
        renderer.drawWiFiField();
        if (wifiServer.isRunning()) {
          renderer.drawWiFiAPIP(wifiServer.getApIpAddress(), PMIXERSSID);
          renderer.drawWiFiPromt("Short press: disable");
        } else {
          renderer.drawWiFiAPIP("WiFi OFF", "No SSID");
          renderer.drawWiFiPromt("Long press: enable");
        }
      }
    }
  }

  // Handle web requests
  wifiServer.handleClient();
}
