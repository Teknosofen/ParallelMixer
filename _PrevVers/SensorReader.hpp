#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include "Wire.h"

// Sensor I2C addresses
#define I2Cadr_SFM 0x40  // Flow meter
#define I2Cadr_SPD 0x25  // Differential pressure sensor
#define I2Cadr_SSC 0x58  // Supply pressure sensor

// Sensor commands
#define SFM_com0 0x20  // SW reset
#define SFM_com1 0x10  // Start continuous measurement
#define SFM_com2 0x00
#define SPD_com1 0x36  // Start continuous measurement
#define SPD_com2 0x08

struct SensorData {
  float differential_pressure;  // SPD pressure in mBar
  float flow;                   // Flow in L/min
  float supply_pressure;        // SSC pressure in PSI
};

class SensorReader {
public:
  // Constructor now takes TwoWire pointer for I2C bus selection
  SensorReader(TwoWire* wire, const char* name = "Sensor");
  
  bool initialize();
  void update(SensorData& data);
  
  const char* getName() const { return _name; }

private:
  TwoWire* _wire;              // Pointer to I2C bus (Wire or Wire1)
  const char* _name;           // For debugging
  
  float readDifferentialPressure();
  float readFlow();
  float readSupplyPressure();
  
  bool initSensorI2C(uint8_t address, uint8_t cmd1, uint8_t cmd2, const char* name);
};

#endif
