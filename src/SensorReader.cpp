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
  
  // ============================================================================
  // Initialize SFM3505 flow sensor (minimal implementation)
  // ============================================================================
  
  // Start SFM3505 continuous measurement
  if (startSFM3505Measurement()) {
    Serial.printf("[%s] ✅ SFM3505 initialized and started\n", _name);
  } else {
    Serial.printf("[%s] ⚠️ SFM3505 not detected or initialization failed\n", _name);
  }
  
  // ============================================================================
  // Initialize LEGACY sensors (keep existing functionality)
  // ============================================================================
  
  // SW reset Sensirion flow measurement (legacy sensor at 0x40)
  _wire->beginTransmission(I2Cadr_SFM);
  _wire->write(SFM_com0);
  _wire->write(SFM_com2);
  if (_wire->endTransmission() != 0) {
    Serial.printf("[%s] err Legacy SFM SW reset\n", _name);
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
  // This method updates LEGACY sensor readings (SPD, legacy SFM, SSC)
  // For SFM3505, use the dedicated readSFM3505xxx() methods
  
  data.differential_pressure = readDifferentialPressure();
  data.flow = readFlow();  // Legacy SFM sensor at 0x40
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

// ============================================================================
// SFM3505 Flow Sensor Methods (Minimal Implementation)
// ============================================================================

bool SensorReader::readSFM3505AirFlow(float& airFlow) {
  uint8_t buffer[6];  // Need 6 bytes even though we only use first 3
  
  if (!readSFM3505Raw(buffer, 6)) {
    return false;
  }
  
  // Extract air flow (first 3 bytes)
  uint32_t rawValue = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
  airFlow = scaleSFM3505Flow(rawValue);
  
  return true;
}

bool SensorReader::readSFM3505AllFlows(float& airFlow, float& o2Flow) {
  uint8_t buffer[6];
  
  if (!readSFM3505Raw(buffer, 6)) {
    return false;
  }
  
  // Extract air flow (first 3 bytes)
  uint32_t airRaw = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
  airFlow = scaleSFM3505Flow(airRaw);
  
  // Extract O2 flow (second 3 bytes)
  uint32_t o2Raw = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
  o2Flow = scaleSFM3505Flow(o2Raw);
  
  return true;
}

bool SensorReader::readSFM3505AirFlowRaw(uint32_t& airFlowRaw) {
  uint8_t buffer[6];
  
  if (!readSFM3505Raw(buffer, 6)) {
    return false;
  }
  
  airFlowRaw = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
  return true;
}

bool SensorReader::readSFM3505AllFlowsRaw(uint32_t& airFlowRaw, uint32_t& o2FlowRaw) {
  uint8_t buffer[6];
  
  if (!readSFM3505Raw(buffer, 6)) {
    return false;
  }
  
  airFlowRaw = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
  o2FlowRaw = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
  
  return true;
}

bool SensorReader::startSFM3505Measurement() {
  if (!sendSFM3505Command(SFM3505_CMD_START_CONTINUOUS)) {
    return false;
  }
  delay(4);  // Wait for measurement to start
  return true;
}

bool SensorReader::stopSFM3505Measurement() {
  return sendSFM3505Command(SFM3505_CMD_STOP_CONTINUOUS);
}

// ============================================================================
// SFM3505 Helper Methods
// ============================================================================

bool SensorReader::sendSFM3505Command(uint16_t command) {
  _wire->beginTransmission(I2Cadr_SFM3505);
  _wire->write((uint8_t)(command >> 8));    // MSB
  _wire->write((uint8_t)(command & 0xFF));  // LSB
  
  if (_wire->endTransmission() != 0) {
    return false;
  }
  
  return true;
}

bool SensorReader::readSFM3505Raw(uint8_t* buffer, uint8_t length) {
  // SFM3505 returns data with CRC: [byte0][byte1][byte2][CRC] [byte3][byte4][byte5][CRC]
  // We need to read 9 bytes total (3 + CRC + 3 + CRC + 1)
  uint8_t rawBuffer[9];
  
  uint8_t bytesRead = _wire->requestFrom(I2Cadr_SFM3505, (uint8_t)9);
  
  if (bytesRead != 9) {
    Serial.printf("[%s] SFM3505 read error: expected 9 bytes, got %d\n", _name, bytesRead);
    return false;
  }
  
  for (uint8_t i = 0; i < 9; i++) {
    rawBuffer[i] = _wire->read();
  }
  
  // Validate CRC for first 3 bytes
  uint8_t crc1 = calculateCRC8(rawBuffer, 3);
  if (crc1 != rawBuffer[3]) {
    Serial.printf("[%s] SFM3505 CRC error on air flow\n", _name);
    return false;
  }
  
  // Validate CRC for second 3 bytes
  uint8_t crc2 = calculateCRC8(&rawBuffer[4], 3);
  if (crc2 != rawBuffer[7]) {
    Serial.printf("[%s] SFM3505 CRC error on O2 flow\n", _name);
    return false;
  }
  
  // Copy data without CRC bytes
  buffer[0] = rawBuffer[0];
  buffer[1] = rawBuffer[1];
  buffer[2] = rawBuffer[2];
  buffer[3] = rawBuffer[4];
  buffer[4] = rawBuffer[5];
  buffer[5] = rawBuffer[6];
  
  return true;
}

float SensorReader::scaleSFM3505Flow(uint32_t rawValue) {
  // Scaling formula from SFM3505 datasheet:
  // Flow (slm) = (rawValue - offset) / scale_factor
  // offset = 8388608 (2^23)
  // scale_factor = 25600
  
  int32_t signedValue = (int32_t)rawValue;
  float flow = ((float)(signedValue - 8388608)) / 25600.0;
  
  return flow;
}

uint8_t SensorReader::calculateCRC8(const uint8_t* data, uint8_t len) {
  // CRC-8 polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
  // Used by Sensirion sensors
  
  uint8_t crc = 0xFF;  // Initial value
  
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  
  return crc;
}
