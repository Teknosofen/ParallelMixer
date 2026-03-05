#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include "Wire.h"
#include "PinConfig.h"

// ============================================================================
// PRESSURE SENSORS - ABP2 (0x28) + ELVH (0x48) on same bus, no conflict
// ============================================================================

// Sensor I2C addresses
#define I2Cadr_SFM 0x40  // Flow meter (legacy, disabled)
#define I2Cadr_SPD 0x25  // Differential pressure sensor (legacy, disabled)
#define I2Cadr_SSC 0x58  // Supply pressure sensor (legacy, disabled)
#define I2Cadr_SFM3505 0x2E  // SFM3505 flow sensor (ACTIVE)
#define I2Cadr_SFM3304 0x2E  // SFM3304 flow sensor (same address — only ONE of SFM3505/SFM3304 per bus!)

// Pressure sensor addresses (no conflict - different addresses)
#define I2Cadr_ABP2 0x28  // ABP2DSNT150PG2A3XX Honeywell high pressure sensor
#define I2Cadr_ELVH 0x48  // ELVH-M100D...4A4 low pressure sensor

// Sensor commands (legacy sensors)
#define SFM_com0 0x20  // SW reset
#define SFM_com1 0x10  // Start continuous measurement
#define SFM_com2 0x00
#define SPD_com1 0x36  // Start continuous measurement
#define SPD_com2 0x08

// SFM3505 commands
#define SFM3505_CMD_START_CONTINUOUS 0x3603
#define SFM3505_CMD_STOP_CONTINUOUS 0x3FF9

// SFM3505 CRC validation (set to 0 to disable CRC checks for debugging)
#define SFM3505_ENABLE_CRC_CHECK 0  // TEMPORARILY DISABLED for testing

// SFM3304 commands
#define SFM3304_CMD_START_CONTINUOUS      0x3603
#define SFM3304_CMD_STOP_CONTINUOUS       0x3FF9
#define SFM3304_CMD_READ_SCALE_OFFSET     0x3661
#define SFM3304_CMD_CONFIGURE_AVERAGING   0x366A
#define SFM3304_CMD_ENTER_SLEEP           0x3677
#define SFM3304_CMD_READ_PRODUCT_ID       0xE102

struct SensorData {
  // Legacy/existing sensors (disabled)
  float differential_pressure = -9.9;  // SPD pressure in mBar (disabled)
  float flow = -9.9;                   // Legacy SFM flow in L/min from 0x40 (disabled)
  float supply_pressure = -9.9;        // ABP2 pressure in kPa (ACTIVE)

  // SFM3505 flow sensor data (ACTIVE)
  float sfm3505_air_flow = -9.9;       // SFM3505 air flow in slm
  float sfm3505_o2_flow = -9.9;        // SFM3505 O2 flow in slm

  // SFM3304 flow sensor data (alternative to SFM3505 per bus)
  float sfm3304_flow = -9.9;           // SFM3304 flow in slm
  float sfm3304_temperature = -9.9;    // SFM3304 temperature in °C
  uint16_t sfm3304_status_word = 0;    // SFM3304 status word

  // ELVH-M100D low pressure sensor data (ACTIVE)
  float elvh_pressure = -9.9;           // ELVH low pressure in mbar
  float elvh_temperature = -9.9;        // ELVH temperature in °C
};

struct SensorDataRaw {
  uint32_t sfm3505_air_flow_raw;  // SFM3505 raw air flow
  uint32_t sfm3505_o2_flow_raw;   // SFM3505 raw O2 flow
};

class SensorReader {
public:
  // Constructor now takes TwoWire pointer for I2C bus selection
  // clockFreq defaults to I2C0_CLOCK_FREQ from PinConfig.h
  SensorReader(TwoWire* wire, const char* name = "Sensor", uint32_t clockFreq = I2C0_CLOCK_FREQ);

  bool initialize();
  void update(SensorData& data);
  
  // SFM3505 methods - scaled (float) values
  bool readSFM3505AirFlow(float& airFlow);
  bool readSFM3505AllFlows(float& airFlow, float& o2Flow);
  
  // SFM3505 methods - raw (uint32_t) values
  bool readSFM3505AirFlowRaw(uint32_t& airFlowRaw);
  bool readSFM3505AllFlowsRaw(uint32_t& airFlowRaw, uint32_t& o2FlowRaw);
  
  // SFM3505 control methods
  bool startSFM3505Measurement();
  bool stopSFM3505Measurement();

  // SFM3304 methods
  bool startSFM3304Measurement();
  bool stopSFM3304Measurement();
  bool readSFM3304Data(float& flow, float& temperature, uint16_t& statusWord);
  bool readSFM3304Flow(float& flow);
  bool readSFM3304ScaleOffset(int16_t& scaleFactor, int16_t& offset);
  bool readSFM3304ProductId(uint32_t& productId, uint64_t& serialNumber);
  bool configureSFM3304Averaging(uint16_t avgWindow);

  // ABP2 pressure sensor methods - asynchronous measurement
  // Usage in main loop at 200Hz:
  //   if (iteration % 2 == 0) {  // Every other cycle (100Hz)
  //     sensor.startABP2Measurement();  // Start conversion
  //   } else {
  //     float pressure; uint8_t status;
  //     if (sensor.readABP2Pressure(pressure, status)) {
  //       // Use pressure value
  //     }
  //   }
  bool startABP2Measurement();           // Send 0xAA 0x00 0x00 command
  bool readABP2Pressure(float& pressure_kpa, uint8_t& status_byte);  // Read 7 bytes (no delays!)
  bool isABP2Busy();                     // Check busy flag without full read

  // ELVH low pressure sensor methods - direct read (no command phase)
  // The ELVH continuously measures; just read 4 bytes via I2C.
  // WARNING: Do NOT send a write (beginTransmission+endTransmission) to this sensor
  // without data — per datasheet, START+STOP without clock transitions corrupts I2C state.
  bool readELVHPressureTemp(float& pressure_mbar, float& temperature_c, uint8_t& status_byte);

  const char* getName() const { return _name; }

  // Sensor detection status - check these before reading
  bool hasSFM3505() const { return _hasSFM3505; }
  bool hasSFM3304() const { return _hasSFM3304; }
  bool hasABP2() const { return _hasABP2; }
  bool hasELVH() const { return _hasELVH; }

private:
  TwoWire* _wire;              // Pointer to I2C bus (Wire or Wire1)
  const char* _name;           // For debugging
  uint32_t _clockFreq;         // I2C clock frequency for this bus

  // Sensor detection flags
  bool _hasSFM3505;
  bool _hasSFM3304;
  bool _hasABP2;
  bool _hasELVH;

  // SFM3304 calibration data (read from sensor at init)
  int16_t _sfm3304_scaleFactor;
  int16_t _sfm3304_offset;
  
  // Legacy sensor reading methods
  float readDifferentialPressure();
  float readFlow();
  float readSupplyPressure();
  
  // SFM3505 helper methods (minimal implementation)
  bool sendSFM3505Command(uint16_t command);
  bool readSFM3505Raw(uint8_t* buffer, uint8_t length);
  float scaleSFM3505Flow(uint32_t rawValue);
  uint8_t calculateCRC8(const uint8_t* data, uint8_t len);
  
  // SFM3304 helper methods
  bool sendSFM3304Command(uint16_t command);
  bool sendSFM3304CommandWithArg(uint16_t command, uint16_t argument);
  bool identifySFM3304();
  
  bool initSensorI2C(uint8_t address, uint8_t cmd1, uint8_t cmd2, const char* name);
};

#endif // SENSOR_READER_H