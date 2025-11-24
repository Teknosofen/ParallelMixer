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
  float fused_flow;            // Calculated fused flow value
  uint16_t flow_ref_analogue;  // Analogue flow reference reading
};

struct SensorFusionConfig {
  float diff_press_scale_factor;
  float diff_press_exponent;
  float flow_low_limit;
  float flow_high_limit;
};

class SensorReader {
public:
  SensorReader(uint8_t flow_input_pin, uint8_t flow_output_pin);
  
  bool initialize();
  void update(SensorData& data);
  float calculateFusedFlow(float diffPress, float flow);
  
  void setFusionConfig(const SensorFusionConfig& config);
  SensorFusionConfig getFusionConfig() const;

private:
  uint8_t _flow_input_pin;
  uint8_t _flow_output_pin;
  SensorFusionConfig _fusion_config;
  
  float readDifferentialPressure();
  float readFlow();
  float readSupplyPressure();
  float readFlowSetValue();
  
  bool initSensorI2C(uint8_t address, uint8_t cmd1, uint8_t cmd2, const char* name);
};

#endif