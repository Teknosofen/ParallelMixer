// Main.cpp - Complete refactored ventilator control system with Dual I2C + Dual Serial
// For T-Display S3
// Date: 2024

#include <Arduino.h>
#include "MainGlobals.hpp"
#include "SensorPolling.hpp"
#include "OutputFormatting.hpp"
#include "DisplayUpdate.hpp"

// ---------------------------------
// logo BMP file as well as factory default config files can be stored in SPIFFS
// to send to device run:
// pio run --target uploadfs
//
// to format SPIFFS run:
// pio run --target cleanfs
// ---------------------------------

// ============================================================================
// Global state — definitions (declared extern in MainGlobals.hpp)
// ============================================================================

TFT_eSPI tft = TFT_eSPI();
ImageRenderer renderer(tft);

// Two sensor readers on separate I2C buses - use pointers to avoid initialization issues
SensorReader* sensors_bus0;   // GPIO43/44 - includes SFM3505, ABP2, ELVH
SensorReader* sensors_bus1;   // GPIO10/11 - includes SFM3505, ABP2

// Array of actuator controls - one per MUX channel for parallel operation
ActuatorControl actuators[NUM_MUX_CHANNELS];

CommandParser parser;
SerialMuxRouter muxRouter(&Serial1);
VentilatorController ventilator;
LocalValveController localValveCtrl;
ValveCharacterizer valveChar;

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

// LLC manual test mode (LF command)
static bool  llcManualTestActive = false;
static float llcManualAirFlow_slm = 0.0f;
static float llcManualO2Flow_slm  = 0.0f;
static float llcManualMaxPressure_mbar = 9999.0f;  // No pressure limit in pure flow test (LP to override)

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
      else if (address == 0x48) hostCom.print(" - ELVH-M100D (low pressure)");
      else hostCom.print(" - unknown device");
      
      hostCom.println();
      deviceCount++;
    }
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
      if (address == 0x2E) hostCom.print(" - SFM3505/SFM3304 (flow)");
      else if (address == 0x28) hostCom.print(" - ABP2 (pressure)");
      else if (address == 0x48) hostCom.print(" - ELVH-M100D (low pressure)");
      else if (address == 0x60) hostCom.print(" - MCP4725 (DAC)");
      else hostCom.print(" - unknown device");

      hostCom.println();
      bus1DeviceCount++;
    }
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
  // Configure actuator channel 4 for direct PWM output (motor controller on GPIO21)
  actuators[4].setPwmOutput(21, 20000, 8);  // GPIO21, 20 kHz, 8-bit (0-255)
  hostCom.printf("Initialized %d MUX channels for parallel operation\n", NUM_MUX_CHANNELS);
  hostCom.println("  Channel 4: direct PWM on GPIO21 (20 kHz, 8-bit)");

  // Initialize ventilator controller (HLC)
  ventilator.begin(&muxRouter);
  wifiServer.setVentilatorController(&ventilator);
  hostCom.println("Ventilator HLC initialized (use VS for status, VO1 to start)");

  // Initialize local valve controller (LLC)
  // Sits between HLC outputs and MUX serial commands
  localValveCtrl.begin(&muxRouter, LocalValveControllerConfig{});
  localValveCtrl.setDefaults();
  localValveCtrl.setEnabled(false);  // Start disabled (pass-through to downstream controllers)
  hostCom.println("Local valve controller initialized (disabled, use LE1 to enable)");

  // Initialize valve characterizer
  valveChar.begin(&muxRouter);
  hostCom.println("Valve characterizer ready (CC<ch> to start, CX to abort)");

  past_time = micros();
  control_past_time = micros();

  hostCom.println("╔═══════════════════════════════╗");
  hostCom.println("║         SYSTEM READY          ║");
  hostCom.println("╚═══════════════════════════════╝");
  hostCom.println("Commands: ? = help, ! = settings\n");
}

// ============================================================================
// Control functions (kept in main.cpp — tightly coupled to setup/loop)
// ============================================================================

void updateVentilatorControl(uint32_t now) {
  // --- LLC manual test mode (LF command) ---
  if (llcManualTestActive && !ventilator.isRunning()) {
    LocalValveControllerSetpoints llcSP;
    llcSP.airFlow_slm      = llcManualAirFlow_slm;
    llcSP.o2Flow_slm       = llcManualO2Flow_slm;
    llcSP.maxPressure_mbar = llcManualMaxPressure_mbar;
    llcSP.expPressure_mbar = 0;
    llcSP.expValveClosed   = false;
    llcSP.expValveActive   = false;

    LocalValveControllerMeasurements llcMeas;
    llcMeas.airFlow_slm          = sensorData_bus0.sfm3505_air_flow;
    llcMeas.o2Flow_slm           = sensorData_bus1.sfm3505_air_flow;
    llcMeas.airSupplyPressure_kPa = sensorData_bus0.supply_pressure;
    llcMeas.o2SupplyPressure_kPa  = sensorData_bus1.supply_pressure;
    llcMeas.airwayPressure_mbar   = getELVH_Pressure();

    float dt = sysConfig.control_interval / 1000000.0f;
    localValveCtrl.update(llcSP, llcMeas, dt);

    // Debug: print LLC feedback every ~1 second (100 cycles at 100Hz)
    static uint16_t lfDebugCounter = 0;
    if (++lfDebugCounter >= 100) {
      lfDebugCounter = 0;
      auto airSt = localValveCtrl.getAirValveStatus();
      auto o2St  = localValveCtrl.getO2ValveStatus();
      Serial.printf("[LF] SP: air=%.1f o2=%.1f | Meas: air=%.2f o2=%.2f | Psup=%.0f/%.0f kPa | dt=%.4f\n",
                    llcSP.airFlow_slm, llcSP.o2Flow_slm,
                    llcMeas.airFlow_slm, llcMeas.o2Flow_slm,
                    llcMeas.airSupplyPressure_kPa, llcMeas.o2SupplyPressure_kPa, dt);
      Serial.printf("     Air: FF=%.2f PI=%.2f Out=%.2f V%% | O2: FF=%.2f PI=%.2f Out=%.2f V%%\n",
                    airSt.feedforwardCurrent_A, airSt.flowPIOutput_A, airSt.outputCurrent_A,
                    o2St.feedforwardCurrent_A, o2St.flowPIOutput_A, o2St.outputCurrent_A);
    }
    return;
  }

  // --- Normal ventilator mode ---
  if (!ventilator.isRunning()) return;

  // Auto-enable LLC when ventilator runs (motor drivers need V% commands from LLC)
  if (!localValveCtrl.isEnabled()) {
    localValveCtrl.setEnabled(true);
    localValveCtrl.reset();
    Serial.println("[VENT] LLC auto-enabled for ventilator mode");
  }

  VentilatorMeasurements ventMeas;
  ventMeas.airwayPressure_mbar = getELVH_Pressure();
  ventMeas.inspFlow_slm = sensorData_bus0.sfm3505_air_flow + sensorData_bus1.sfm3505_air_flow;
  ventMeas.expFlow_slm = 0;  // TODO: Add expiratory flow sensor if available
  ventMeas.deliveredO2_percent = fdo2Initialized ?
    fdo2Sensor.convertToPercentO2(fdo2Data.oxygenPartialPressure_hPa, fdo2Data.ambientPressure_mbar) : 21.0f;

  // Track state transitions for debug output
  static VentilatorState lastState = VENT_OFF;
  VentilatorState prevState = ventilator.getState();

  ventilator.update(ventMeas, now);

  VentilatorState curState = ventilator.getState();
  if (curState != lastState) {
    VentilatorOutputs dbgOut = ventilator.getOutputs();
    Serial.printf("[VENT] %s -> %s  AirF=%.1f O2F=%.1f ExpCl=%d ExpP=%.1f\n",
                  (lastState == VENT_OFF ? "OFF" :
                   lastState == VENT_INSP_PHASE1 ? "INSP1" :
                   lastState == VENT_INSP_PHASE2 ? "INSP2" :
                   lastState == VENT_INSP_PAUSE ? "PAUSE" :
                   lastState == VENT_EXP_NON_TRIG ? "EXP" : "SYNC"),
                  ventilator.getStateString(),
                  dbgOut.airValveFlow_slm, dbgOut.o2ValveFlow_slm,
                  dbgOut.expValveClosed, dbgOut.expValveSetpoint);
    lastState = curState;
  }

  VentilatorOutputs ventOut = ventilator.getOutputs();
  VentilatorConfig  ventCfg = ventilator.getConfig();

  // Build LLC setpoints from HLC outputs
  LocalValveControllerSetpoints llcSP;
  llcSP.airFlow_slm = ventOut.airValveFlow_slm;
  llcSP.o2Flow_slm = ventOut.o2ValveFlow_slm;
  llcSP.maxPressure_mbar = ventCfg.maxPressure_mbar;
  llcSP.expPressure_mbar = ventOut.expValveSetpoint;
  llcSP.expValveClosed = ventOut.expValveClosed;
  llcSP.expValveActive = ventOut.expValveAsPressure;

  // Build LLC measurements from sensor data
  LocalValveControllerMeasurements llcMeas;
  llcMeas.airFlow_slm = sensorData_bus0.sfm3505_air_flow;
  llcMeas.o2Flow_slm = sensorData_bus1.sfm3505_air_flow;
  llcMeas.airSupplyPressure_kPa = sensorData_bus0.supply_pressure;
  llcMeas.o2SupplyPressure_kPa = sensorData_bus1.supply_pressure;
  llcMeas.airwayPressure_mbar = getELVH_Pressure();

  // LLC handles valve current computation OR pass-through
  float dt = 0.01f;  // 100 Hz control loop = 10 ms
  localValveCtrl.update(llcSP, llcMeas, dt);
}

/** Execute actuator control for all MUX channels (when ventilator not running). */
void executeActuatorControl() {
  if (ventilator.isRunning()) return;
  if (llcManualTestActive) return;  // LLC manual test owns the valve outputs

  for (uint8_t i = 0; i < NUM_MUX_CHANNELS; i++) {
    actuators[i].execute(sysConfig.digital_flow_reference, sensorData_bus0.flow, sysConfig.quiet_mode);
  }
}

/** Process serial communication, commands, and MUX channel changes. */
void processSerialCommands() {
  muxRouter.update();

  parser.update();

  // Check for LLC commands before ventilator/general commands
  // LE0/LE1 — Local valve controller enable
  // LS — LLC status
  if (parser.isCommandComplete()) {
    String cmd = parser.getCommandString();
    cmd.trim();
    String upper = cmd.substring(0, 2);
    upper.toUpperCase();

    if (upper == "LE") {
      String val = cmd.substring(2);
      val.trim();
      if (val.length() > 0) {
        bool en = (val.toInt() != 0);
        localValveCtrl.setEnabled(en);
        if (en) localValveCtrl.reset();
        Serial.printf("LLC %s\n", en ? "ENABLED (local valve control)" : "DISABLED (pass-through)");
      } else {
        Serial.printf("LE=%d\n", localValveCtrl.isEnabled() ? 1 : 0);
      }
      parser.clearCommand();
    }
    else if (upper == "LS") {
      Serial.printf("LLC: %s\n", localValveCtrl.isEnabled() ? "ENABLED" : "DISABLED");
      if (localValveCtrl.isEnabled()) {
        InspValveStatus air = localValveCtrl.getAirValveStatus();
        InspValveStatus o2  = localValveCtrl.getO2ValveStatus();
        ExpValveStatus  exp = localValveCtrl.getExpValveStatus();
        Serial.printf("  Air: I=%.3fA FF=%.3f FlowPI=%.3f PressPI=%.3f %s\n",
                      air.outputCurrent_A, air.feedforwardCurrent_A,
                      air.flowPIOutput_A, air.pressurePIOutput_A,
                      air.pressureLimiting ? "PLIM" : "FLOW");
        Serial.printf("  O2:  I=%.3fA FF=%.3f FlowPI=%.3f PressPI=%.3f %s\n",
                      o2.outputCurrent_A, o2.feedforwardCurrent_A,
                      o2.flowPIOutput_A, o2.pressurePIOutput_A,
                      o2.pressureLimiting ? "PLIM" : "FLOW");
        Serial.printf("  Exp: I=%.3fA FF=%.3f PressPI=%.3f\n",
                      exp.outputCurrent_A, exp.feedforwardCurrent_A,
                      exp.pressurePIOutput_A);
      }
      parser.clearCommand();
    }
    else if (upper == "LF") {
      // LLC manual flow test: LF<airflow>[,<o2flow>]
      String params = cmd.substring(2);
      params.trim();
      if (params.length() > 0) {
        int comma = params.indexOf(',');
        if (comma < 0) {
          llcManualAirFlow_slm = params.toFloat();
          llcManualO2Flow_slm = 0.0f;
        } else {
          llcManualAirFlow_slm = params.substring(0, comma).toFloat();
          llcManualO2Flow_slm  = params.substring(comma + 1).toFloat();
        }
        llcManualTestActive = true;
        if (!localValveCtrl.isEnabled()) {
          localValveCtrl.setEnabled(true);
          localValveCtrl.reset();
          Serial.println("[LF] LLC auto-enabled");
        }
        Serial.printf("[LF] Manual test: Air=%.1f slm, O2=%.1f slm, Plim=%.0f mbar\n",
                      llcManualAirFlow_slm, llcManualO2Flow_slm, llcManualMaxPressure_mbar);
        Serial.println("[LF] Use LS for status, LX to stop");
      } else {
        if (llcManualTestActive) {
          Serial.printf("LF: Air=%.1f slm, O2=%.1f slm (active)\n",
                        llcManualAirFlow_slm, llcManualO2Flow_slm);
        } else {
          Serial.println("Usage: LF<air_slm>[,<o2_slm>]  e.g. LF10 or LF20,5");
        }
      }
      parser.clearCommand();
    }
    else if (upper == "LX") {
      // Stop LLC manual test
      if (llcManualTestActive) {
        llcManualTestActive = false;
        llcManualAirFlow_slm = 0.0f;
        llcManualO2Flow_slm = 0.0f;
        localValveCtrl.reset();
        // Send zero current to all valve channels to ensure safe stop
        muxRouter.sendCommand(1, 'V', 0.0f);  // Air
        muxRouter.sendCommand(2, 'V', 0.0f);  // O2
        muxRouter.sendCommand(3, 'V', 0.0f);  // Exp
        Serial.println("[LX] Manual test stopped, valves zeroed");
      } else {
        Serial.println("[LX] No manual test running");
      }
      parser.clearCommand();
    }
    else if (upper == "LP") {
      // Set LLC manual test pressure limit
      String val = cmd.substring(2);
      val.trim();
      if (val.length() > 0) {
        llcManualMaxPressure_mbar = val.toFloat();
        Serial.printf("[LP] Pressure limit = %.0f mbar\n", llcManualMaxPressure_mbar);
      } else {
        Serial.printf("LP=%.0f mbar\n", llcManualMaxPressure_mbar);
      }
      parser.clearCommand();
    }
    else if (upper == "CC") {
      // Valve characterization: CC<channel>[,maxV[,stepV[,settleMs]]]
      String params = cmd.substring(2);
      params.trim();
      if (params.length() > 0) {
        CharacterizationConfig charCfg;
        charCfg.maxVoltage = 12.0f;
        charCfg.stepVoltage = 0.1f;
        charCfg.settleTime_ms = 200;
        charCfg.samplesPerStep = 10;

        int comma1 = params.indexOf(',');
        if (comma1 < 0) {
          charCfg.muxChannel = params.toInt();
        } else {
          charCfg.muxChannel = params.substring(0, comma1).toInt();
          String rest = params.substring(comma1 + 1);
          int comma2 = rest.indexOf(',');
          if (comma2 < 0) {
            charCfg.maxVoltage = rest.toFloat();
          } else {
            charCfg.maxVoltage = rest.substring(0, comma2).toFloat();
            rest = rest.substring(comma2 + 1);
            int comma3 = rest.indexOf(',');
            if (comma3 < 0) {
              charCfg.stepVoltage = rest.toFloat();
            } else {
              charCfg.stepVoltage = rest.substring(0, comma3).toFloat();
              charCfg.settleTime_ms = rest.substring(comma3 + 1).toInt();
            }
          }
        }
        const char* chName = (charCfg.muxChannel == 1) ? "Air" : 
                             (charCfg.muxChannel == 2) ? "O2" : "??";
        Serial.printf("[CC] Channel=%d (%s), maxV=%.1f%%, stepV=%.2f%%, settle=%lums, samples=%d\n",
                      charCfg.muxChannel, chName,
                      charCfg.maxVoltage, charCfg.stepVoltage,
                      charCfg.settleTime_ms, charCfg.samplesPerStep);
        valveChar.start(charCfg);
      } else {
        Serial.println("Usage: CC<ch>[,maxV[,stepV[,settleMs]]]");
        Serial.println("  ch: 1=air, 2=O2");
        Serial.println("  Example: CC1,12,0.1,200");
      }
      parser.clearCommand();
    }
    else if (upper == "CX") {
      valveChar.abort();
      parser.clearCommand();
    }
  }

  if (!parser.processVentilatorCommands(ventilator)) {
    parser.processCommands(sysConfig, actuator);
  }

  static uint8_t lastMuxChannel = 0;
  if (sysConfig.mux_channel != lastMuxChannel) {
    ControllerMode mode = actuators[sysConfig.mux_channel].getControllerMode();
    hostCom.printf("Switched to MUX channel %d (mode: %d)\n", sysConfig.mux_channel, mode);
    lastMuxChannel = sysConfig.mux_channel;
  }
}

void loop() {
  static bool firstRun = true;
  if (firstRun) {
    initDisplay();
    firstRun = false;
  }

  uint32_t now = micros();

  // ABP2 pressure sensors (async, every loop)
  pollPressureSensors(now);

  // Fast control loop (control_interval, default 10ms = 100Hz)
  if ((now - control_past_time) >= sysConfig.control_interval) {
    control_past_time = now;

    readFastSensors(now);

    // Valve characterizer update (non-blocking state machine)
    valveChar.update(sensorData_bus0, sensorData_bus1, getELVH_Pressure());

    pollFDO2(now);

    updateVentilatorControl(now);
    executeActuatorControl();
    bufferWiFiData();
  }

  // Serial/GUI output and WiFi status push (delta_t, default 100ms = 10Hz)
  if ((now - past_time) >= sysConfig.delta_t) {
    past_time = now;
    updateSerialOutput();
  }

  // Serial communication and command processing
  processSerialCommands();

  // Display updates (UI + O2 at their own rates)
  updateDisplay();

  // Button input handling (WiFi toggle + ventilator settings)
  handleButtons();

  // WiFi client request handling
  wifiServer.handleClient();
}
