#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include "Wire.h"
#include "PinConfig.h"

// Sensor I2C addresses
#define I2Cadr_SFM 0x40  // Flow meter (legacy, disabled)
#define I2Cadr_SPD 0x25  // Differential pressure sensor (legacy, disabled)
#define I2Cadr_SSC 0x58  // Supply pressure sensor (legacy, disabled)
#define I2Cadr_SFM3505 0x2E  // SFM3505 flow sensor (ACTIVE)
#define I2Cadr_ABP2 0x28  // ABP2DSNT150PG2A3XX Honeywell pressure sensor (ACTIVE)
#define I2Cadr_ABPD 0x18  // ABPDLNN100MG2A3 Honeywell low pressure sensor (ACTIVE)

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

struct SensorData {
  // Legacy/existing sensors (disabled)
  float differential_pressure;  // SPD pressure in mBar (disabled)
  float flow;                   // Legacy SFM flow in L/min from 0x40 (disabled)
  float supply_pressure;        // ABP2 pressure in kPa (ACTIVE)

  // SFM3505 flow sensor data (ACTIVE)
  float sfm3505_air_flow;       // SFM3505 air flow in slm
  float sfm3505_o2_flow;        // SFM3505 O2 flow in slm

  // ABPDLNN100MG2A3 low pressure sensor data (ACTIVE)
  float abpd_pressure;          // ABPD low pressure in kPa
  float abpd_temperature;       // ABPD temperature in °C
};

struct SensorDataRaw {
  uint32_t sfm3505_air_flow_raw;  // SFM3505 raw air flow
  uint32_t sfm3505_o2_flow_raw;   // SFM3505 raw O2 flow
};

class SensorReader {
public:
  // Constructor now takes TwoWire pointer for I2C bus selection
  SensorReader(TwoWire* wire, const char* name = "Sensor");
  
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

  // ABPDLNN100MG2A3 low pressure sensor methods - synchronous measurement
  // Returns pressure in kPa (100 mbar range) and temperature in °C
  bool readABPDPressureTemp(float& pressure_kpa, float& temperature_c, uint8_t& status_byte);

  const char* getName() const { return _name; }

  // Sensor detection status - check these before reading
  bool hasSFM3505() const { return _hasSFM3505; }
  bool hasABP2() const { return _hasABP2; }
  bool hasABPD() const { return _hasABPD; }

private:
  TwoWire* _wire;              // Pointer to I2C bus (Wire or Wire1)
  const char* _name;           // For debugging

  // Sensor detection flags
  bool _hasSFM3505;
  bool _hasABP2;
  bool _hasABPD;
  
  // Legacy sensor reading methods
  float readDifferentialPressure();
  float readFlow();
  float readSupplyPressure();
  
  // SFM3505 helper methods (minimal implementation)
  bool sendSFM3505Command(uint16_t command);
  bool readSFM3505Raw(uint8_t* buffer, uint8_t length);
  float scaleSFM3505Flow(uint32_t rawValue);
  uint8_t calculateCRC8(const uint8_t* data, uint8_t len);
  
  bool initSensorI2C(uint8_t address, uint8_t cmd1, uint8_t cmd2, const char* name);
};

#endif // SENSOR_READER_H