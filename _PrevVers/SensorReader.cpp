#include "SensorReader.hpp"

SensorReader::SensorReader(TwoWire* wire, const char* name)
  : _wire(wire), _name(name) {
  // No configuration needed
}

bool SensorReader::initialize() {
  // NOTE: Wire.begin() should be called in main() before calling initialize()
  // This allows main to control I2C pin configuration
  
  _wire->setClock(500000);
  
  Serial.printf("Initializing %s sensors...\n", _name);
  
  // SW reset Sensirion flow measurement
  _wire->beginTransmission(I2Cadr_SFM);
  _wire->write(SFM_com0);
  _wire->write(SFM_com2);
  if (_wire->endTransmission() != 0) {
    Serial.printf("[%s] err SFM SW reset\n", _name);
    return false;
  }
  delay(70);
  
  // SW reset Sensirion pressure measurement
  _wire->beginTransmission(I2Cadr_SPD);
  _wire->write(0x00);
  _wire->write(0x06);
  if (_wire->endTransmission() != 0) {
    Serial.printf("[%s] err SPD SW reset\n", _name);
    return false;
  }
  delay(10);
  
  // Start continuous pressure measurement
  if (!initSensorI2C(I2Cadr_SPD, SPD_com1, SPD_com2, "SPD")) return false;
  delay(10);
  
  // Start continuous flow measurement
  if (!initSensorI2C(I2Cadr_SFM, SFM_com1, SFM_com2, "SFM")) return false;
  delay(10);
  
  Serial.printf("[%s] ✅ All sensors initialized\n", _name);
  return true;
}

bool SensorReader::initSensorI2C(uint8_t address, uint8_t cmd1, uint8_t cmd2, const char* name) {
  _wire->beginTransmission(address);
  _wire->write(cmd1);
  _wire->write(cmd2);
  if (_wire->endTransmission() != 0) {
    Serial.printf("[%s] err %s cont meas setup\n", _name, name);
    return false;
  }
  return true;
}

void SensorReader::update(SensorData& data) {
  data.differential_pressure = readDifferentialPressure();
  data.flow = readFlow();
  data.supply_pressure = readSupplyPressure();
}

float SensorReader::readDifferentialPressure() {
  int16_t combined_local;
  uint8_t msb_local, lsb_local;
  
  _wire->requestFrom(I2Cadr_SPD, 3);
  msb_local = _wire->read();
  lsb_local = _wire->read();
  combined_local = msb_local << 8;
  combined_local |= lsb_local;
  
  return (float(combined_local) * 0.95 / 240);
}

float SensorReader::readFlow() {
  uint16_t combined_local;
  uint8_t msb_local, lsb_local;
  
  _wire->requestFrom(I2Cadr_SFM, 3);
  msb_local = _wire->read();
  lsb_local = _wire->read();
  combined_local = msb_local << 8;
  combined_local |= lsb_local;
  
  return (float(combined_local) - 10000) / 120.0;
}

float SensorReader::readSupplyPressure() {
  uint16_t p;
  uint8_t msb_local, lsb_local;
  
  _wire->requestFrom(I2Cadr_SSC, 4);
  msb_local = _wire->read();
  lsb_local = _wire->read();
  p = ((msb_local & 0x3f) << 8) | lsb_local;
  
  // Read remaining bytes
  _wire->read();
  _wire->read();
  _wire->endTransmission();
  
  return (p - 1638.0) * (6.895) / (13107.0);
}
