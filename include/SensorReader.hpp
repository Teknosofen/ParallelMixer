#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include "Wire.h"

// Sensor I2C addresses
#define I2Cadr_SFM 0x40  // Flow meter (legacy)
#define I2Cadr_SPD 0x25  // Differential pressure sensor
#define I2Cadr_SSC 0x58  // Supply pressure sensor
#define I2Cadr_SFM3505 0x2E  // SFM3505 flow sensor
#define I2Cadr_ABP2 0x29  // ABP2 press sensor sensor

// Sensor commands (legacy sensors)
#define SFM_com0 0x20  // SW reset
#define SFM_com1 0x10  // Start continuous measurement
#define SFM_com2 0x00
#define SPD_com1 0x36  // Start continuous measurement
#define SPD_com2 0x08

// SFM3505 commands
#define SFM3505_CMD_START_CONTINUOUS 0x3603
#define SFM3505_CMD_STOP_CONTINUOUS 0x3FF9

struct SensorData {
  // Legacy/existing sensors
  float differential_pressure;  // SPD pressure in mBar
  float flow;                   // Legacy SFM flow in L/min (from 0x40)
  float supply_pressure;        // SSC pressure in PSI
  
  // SFM3505 flow sensor data
  float sfm3505_air_flow;       // SFM3505 air flow in slm
  float sfm3505_o2_flow;        // SFM3505 O2 flow in slm
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
  
  const char* getName() const { return _name; }

private:
  TwoWire* _wire;              // Pointer to I2C bus (Wire or Wire1)
  const char* _name;           // For debugging
  
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