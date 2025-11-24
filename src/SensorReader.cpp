#include "SensorReader.hpp"

SensorReader::SensorReader(uint8_t flow_input_pin, uint8_t flow_output_pin)
  : _flow_input_pin(flow_input_pin), _flow_output_pin(flow_output_pin) {
  
  // Default fusion configuration from original code
  _fusion_config.diff_press_scale_factor = 2.01;
  _fusion_config.diff_press_exponent = 0.53;
  _fusion_config.flow_low_limit = 1.0;
  _fusion_config.flow_high_limit = 10.0;
}

bool SensorReader::initialize() {
  Wire.begin();
  Wire.setClock(500000);
  
  // SW reset Sensirion flow measurement
  Wire.beginTransmission(I2Cadr_SFM);
  Wire.write(SFM_com0);
  Wire.write(SFM_com2);
  if (Wire.endTransmission() != 0) {
    Serial.println("err SFM SW reset");
    return false;
  }
  delay(70);
  
  // SW reset Sensirion pressure measurement
  Wire.beginTransmission(I2Cadr_SPD);
  Wire.write(0x00);
  Wire.write(0x06);
  if (Wire.endTransmission() != 0) {
    Serial.println("err SPD SW reset");
    return false;
  }
  delay(10);
  
  // Start continuous pressure measurement
  if (!initSensorI2C(I2Cadr_SPD, SPD_com1, SPD_com2, "SPD")) return false;
  delay(10);
  
  // Start continuous flow measurement
  if (!initSensorI2C(I2Cadr_SFM, SFM_com1, SFM_com2, "SFM")) return false;
  delay(10);
  
  return true;
}

bool SensorReader::initSensorI2C(uint8_t address, uint8_t cmd1, uint8_t cmd2, const char* name) {
  Wire.beginTransmission(address);
  Wire.write(cmd1);
  Wire.write(cmd2);
  if (Wire.endTransmission() != 0) {
    Serial.print("err ");
    Serial.print(name);
    Serial.println(" cont meas setup");
    return false;
  }
  return true;
}

void SensorReader::update(SensorData& data) {
  data.differential_pressure = readDifferentialPressure();
  data.flow = readFlow();
  data.supply_pressure = readSupplyPressure();
  data.flow_ref_analogue = readFlowSetValue();
  data.fused_flow = calculateFusedFlow(data.differential_pressure, data.flow);
  
  // Output fused flow to analog pin
  analogWrite(_flow_output_pin, int(data.fused_flow));
}

float SensorReader::readDifferentialPressure() {
  int16_t combined_local;
  uint8_t msb_local, lsb_local;
  
  Wire.requestFrom(I2Cadr_SPD, 3);
  msb_local = Wire.read();
  lsb_local = Wire.read();
  combined_local = msb_local << 8;
  combined_local |= lsb_local;
  
  return (float(combined_local) * 0.95 / 240);
}

float SensorReader::readFlow() {
  uint16_t combined_local;
  uint8_t msb_local, lsb_local;
  
  Wire.requestFrom(I2Cadr_SFM, 3);
  msb_local = Wire.read();
  lsb_local = Wire.read();
  combined_local = msb_local << 8;
  combined_local |= lsb_local;
  
  return (float(combined_local) - 10000) / 120.0;
}

float SensorReader::readSupplyPressure() {
  uint16_t p;
  uint8_t msb_local, lsb_local;
  
  Wire.requestFrom(I2Cadr_SSC, 4);
  msb_local = Wire.read();
  lsb_local = Wire.read();
  p = ((msb_local & 0x3f) << 8) | lsb_local;
  
  // Read remaining bytes
  Wire.read();
  Wire.read();
  Wire.endTransmission();
  
  return (p - 1638.0) * (6.895) / (13107.0);
}

float SensorReader::readFlowSetValue() {
  const float flowScaling = 200.0 / 1023.0;
  analogRead(_flow_input_pin);
  return flowScaling * float(analogRead(_flow_input_pin));
}

float SensorReader::calculateFusedFlow(float diffPress, float flow) {
  if (diffPress <= 0.0) diffPress = 0.0;
  
  float dPCalculatedFlow = _fusion_config.diff_press_scale_factor * 
                           pow(diffPress, _fusion_config.diff_press_exponent);
  
  if (flow < _fusion_config.flow_low_limit) {
    return dPCalculatedFlow;
  } else if (flow < _fusion_config.flow_high_limit) {
    float weighingFactor = (flow - _fusion_config.flow_low_limit) / 
                           (_fusion_config.flow_high_limit - _fusion_config.flow_low_limit);
    return dPCalculatedFlow * (1 - weighingFactor) + flow * weighingFactor;
  } else {
    return flow;
  }
}

void SensorReader::setFusionConfig(const SensorFusionConfig& config) {
  _fusion_config = config;
}

SensorFusionConfig SensorReader::getFusionConfig() const {
  return _fusion_config;
}