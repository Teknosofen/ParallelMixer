#include "SensorReader.hpp"

SensorReader::SensorReader(TwoWire* wire, const char* name)
  : _wire(wire), _name(name), _hasSFM3505(false), _hasABP2(false), _hasABPD(false) {
  // Detection flags will be set during initialize()
}

bool SensorReader::initialize() {
  // NOTE: Wire.begin() should be called in main() before calling initialize()
  // This allows main to control I2C pin configuration

  _wire->setClock(I2C0_CLOCK_FREQ);  // Set I2C clock frequency (defined in PinConfig.h)

  Serial.printf("Initializing %s sensors...\n", _name);

  // Reset detection flags
  _hasSFM3505 = false;
  _hasABP2 = false;
  _hasABPD = false;

  // ============================================================================
  // Detect SFM3505 flow sensor
  // ============================================================================
  Serial.printf("[%s] Checking for SFM3505 at address 0x%02X...\n", _name, I2Cadr_SFM3505);
  _wire->beginTransmission(I2Cadr_SFM3505);
  byte error = _wire->endTransmission();

  if (error == 0) {
    Serial.printf("[%s] ✅ SFM3505 detected\n", _name);
    _hasSFM3505 = true;

    // Start continuous measurement
    delay(50);
    Serial.printf("[%s] Starting SFM3505 continuous measurement...\n", _name);
    if (startSFM3505Measurement()) {
      Serial.printf("[%s] ✅ SFM3505 measurement started\n", _name);
    } else {
      Serial.printf("[%s] ⚠️ SFM3505 start command failed (may already be running)\n", _name);
    }
  } else {
    Serial.printf("[%s] ⚠️ SFM3505 not detected at 0x%02X\n", _name);
  }

  // ============================================================================
  // Detect ABP2 Pressure Sensor
  // ============================================================================
  Serial.printf("[%s] Checking for ABP2 at address 0x%02X...\n", _name, I2Cadr_ABP2);
  _wire->beginTransmission(I2Cadr_ABP2);
  byte abp2_error = _wire->endTransmission();

  if (abp2_error == 0) {
    Serial.printf("[%s] ✅ ABP2 detected (asynchronous measurement)\n", _name);
    _hasABP2 = true;
  } else {
    Serial.printf("[%s] ⚠️ ABP2 not detected at 0x%02X\n", _name);
  }

  // ============================================================================
  // Detect ABPD Low Pressure Sensor
  // ============================================================================
  Serial.printf("[%s] Checking for ABPD at address 0x%02X...\n", _name, I2Cadr_ABPD);
  _wire->beginTransmission(I2Cadr_ABPD);
  byte abpd_error = _wire->endTransmission();

  if (abpd_error == 0) {
    Serial.printf("[%s] ✅ ABPD detected\n", _name);
    _hasABPD = true;
  } else {
    Serial.printf("[%s] ⚠️ ABPD not detected at 0x%02X\n", _name);
  }

  // ============================================================================
  // Summary
  // ============================================================================
  int detectedCount = (_hasSFM3505 ? 1 : 0) + (_hasABP2 ? 1 : 0) + (_hasABPD ? 1 : 0);
  Serial.printf("[%s] Sensor detection complete: %d/3 sensors found\n", _name, detectedCount);

  // Return true if at least one sensor was detected
  return (detectedCount > 0);

  // ============================================================================
  // LEGACY SENSORS DISABLED - ONLY USING SFM3505
  // ============================================================================
  // The following legacy sensor code has been commented out:
  // - Legacy SFM sensor at 0x40
  // - SPD pressure sensor at 0x25
  // - SSC sensor at 0x58
  //
  // To re-enable, uncomment the code below:
  /*
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
  */
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
  // NOTE: ABP2 pressure sensor is now read asynchronously in main.cpp
  // See startABP2Measurement() and readABP2Pressure() for async operation
  // This update() method is DEPRECATED for pressure reading

  // LEGACY SENSORS DISABLED
  data.differential_pressure = 0.0;
  data.flow = 0.0;  // Legacy SFM sensor disabled
  // data.supply_pressure is now updated by main.cpp async loop
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
  // ABP2DSNT150PG2A3XX Honeywell pressure sensor at 0x28
  // 14-bit output, 150 PSI gauge pressure range
  // Transfer function: 10% to 90% of 2^14 counts (1638 to 14746)

  static bool errorPrinted = false;  // Only print errors once to reduce spam
  static uint8_t readCount = 0;
  uint8_t status, msb_local, lsb_local;

  // ABP2 is always measuring - just read the data
  // No command needed, sensor continuously updates
  // NOTE: This method is DEPRECATED - use startABP2Measurement() + readABP2Pressure() instead
  // This old method has NO DELAYS - caller must handle timing!

  // Request 7 bytes total per ABP2 datasheet section 6.4:
  // Byte 0: Status (8-bit)
  // Bytes 1-3: Pressure (24-bit)
  // Bytes 4-6: Temperature (24-bit)
  uint8_t bytesReceived = _wire->requestFrom(I2Cadr_ABP2, (uint8_t)7);

  if (bytesReceived != 7) {
    if (!errorPrinted || readCount < 3) {
      Serial.printf("[%s] ABP2 read error: expected 7 bytes, got %d\n", _name, bytesReceived);
      if (bytesReceived == 0) {
        Serial.printf("[%s]    Sensor not responding - check wiring/power\n", _name);
      }
      if (readCount >= 3) {
        Serial.printf("[%s]    This message will not repeat\n", _name);
        errorPrinted = true;
      }
      readCount++;
    }
    // Clear any partial data
    while (_wire->available()) {
      _wire->read();
    }
    return 0.0;
  }

  readCount++;

  // Read all 7 bytes
  uint8_t status_byte = _wire->read();     // Byte 0: Status
  uint8_t press_msb = _wire->read();       // Byte 1: Pressure [23:16]
  uint8_t press_mid = _wire->read();       // Byte 2: Pressure [15:8]
  uint8_t press_lsb = _wire->read();       // Byte 3: Pressure [7:0]
  uint8_t temp_msb = _wire->read();        // Byte 4: Temperature [23:16]
  uint8_t temp_mid = _wire->read();        // Byte 5: Temperature [15:8]
  uint8_t temp_lsb = _wire->read();        // Byte 6: Temperature [7:0]

  // Parse individual status bits according to Table 20:
  // Bit 7: always 0
  // Bit 6: Power indication (1=powered, 0=not powered)
  // Bit 5: Busy flag (1=busy)
  // Bit 4-3: always 0
  // Bit 2: Memory integrity (0=passed, 1=failed)
  // Bit 1: always 0
  // Bit 0: Math saturation (1=occurred)

  bool powered = (status_byte & 0x40) != 0;        // Bit 6
  bool busy = (status_byte & 0x20) != 0;           // Bit 5
  bool memory_error = (status_byte & 0x04) != 0;   // Bit 2
  bool math_sat = (status_byte & 0x01) != 0;       // Bit 0

  // Check for error conditions
  if (!powered) {
    if (!errorPrinted) {
      Serial.printf("[%s] ❌ ABP2 error: Device not powered (status=0x%02X)\n", _name, status_byte);
      errorPrinted = true;
    }
    return 0.0;
  }

  if (memory_error) {
    if (!errorPrinted) {
      Serial.printf("[%s] ❌ ABP2 error: Memory integrity check failed (status=0x%02X)\n", _name, status_byte);
      errorPrinted = true;
    }
    return 0.0;
  }

  if (math_sat && readCount < 5) {
    Serial.printf("[%s] ⚠️  ABP2 warning: Math saturation occurred (status=0x%02X)\n", _name, status_byte);
  }

  if (busy && readCount < 5) {
    Serial.printf("[%s] ⚠️  ABP2 warning: Device busy (status=0x%02X)\n", _name, status_byte);
  }

  // Extract 24-bit pressure value from 3 pressure bytes
  uint32_t pressure_counts = ((uint32_t)press_msb << 16) | ((uint32_t)press_mid << 8) | press_lsb;

  // Debug: Print raw values on first 10 reads to observe behavior
  // if (readCount <= 10) {
  //   Serial.printf("[%s] ABP2 raw: Status=0x%02X [PWR=%d BSY=%d MEM=%d SAT=%d], Press=0x%02X%02X%02X, Temp=0x%02X%02X%02X, counts=%lu\n",
  //                 _name, status_byte, powered, busy, memory_error, math_sat,
  //                 press_msb, press_mid, press_lsb, temp_msb, temp_mid, temp_lsb,
  //                 (unsigned long)pressure_counts);
  // }

  // Special message if status byte changes
  static uint8_t lastStatusByte = 0xFF;
  if (status_byte != lastStatusByte && readCount > 1) {
    Serial.printf("[%s] 🔔 ABP2 status changed: 0x%02X -> 0x%02X\n", _name, lastStatusByte, status_byte);
    lastStatusByte = status_byte;
  }
  if (readCount == 1) {
    lastStatusByte = status_byte;
  }

  // Convert counts to pressure using 24-bit transfer function
  // P = (counts - 0.1*2^24) * (Pmax - Pmin) / (0.8*2^24)
  // For 150 PSI gauge: Pmin = 0, Pmax = 150 PSI
  // Outputmin = 0.1 * 2^24 = 1,677,722 counts
  // Outputmax = 0.9 * 2^24 = 15,099,494 counts
  // Output span = 15,099,494 - 1,677,722 = 13,421,772 counts

  float pressure_psi = ((float)pressure_counts - 1677722.0) * 150.0 / 13421772.0;

  // Convert PSI to kPa (1 PSI = 6.89476 kPa)
  float pressure_kpa = pressure_psi * 6.89476;

  return pressure_kpa;
}

// ============================================================================
// ABP2 Pressure Sensor Methods - Asynchronous Operation (NEW)
// ============================================================================

bool SensorReader::startABP2Measurement() {
  // Send Output Measurement Command per Table 21
  // Command: 0xAA 0x00 0x00
  _wire->beginTransmission(I2Cadr_ABP2);
  _wire->write(0xAA);  // Command byte
  _wire->write(0x00);  // Data byte 1
  _wire->write(0x00);  // Data byte 2

  byte error = _wire->endTransmission();
  if (error != 0) {
    Serial.printf("[%s] ❌ ABP2 measurement command failed (error=%d)\n", _name, error);
    return false;
  }

  return true;
}

bool SensorReader::isABP2Busy() {
  // Quick check: read just the status byte to check busy flag
  uint8_t bytesReceived = _wire->requestFrom(I2Cadr_ABP2, (uint8_t)1);

  if (bytesReceived != 1) {
    return true;  // Assume busy if can't read
  }

  uint8_t status_byte = _wire->read();
  bool busy = (status_byte & 0x20) != 0;  // Bit 5

  return busy;
}

bool SensorReader::readABP2Pressure(float& pressure_kpa, uint8_t& status_byte) {
  // Read 7 bytes: Status + Pressure(3) + Temperature(3)
  // NO DELAYS - caller handles timing!

  uint8_t bytesReceived = _wire->requestFrom(I2Cadr_ABP2, (uint8_t)7);

  if (bytesReceived != 7) {
    Serial.printf("[%s] ABP2 read error: expected 7 bytes, got %d\n", _name, bytesReceived);
    // Clear any partial data
    while (_wire->available()) {
      _wire->read();
    }
    return false;
  }

  // Read all 7 bytes
  status_byte = _wire->read();                 // Byte 0: Status
  uint8_t press_msb = _wire->read();           // Byte 1: Pressure [23:16]
  uint8_t press_mid = _wire->read();           // Byte 2: Pressure [15:8]
  uint8_t press_lsb = _wire->read();           // Byte 3: Pressure [7:0]
  uint8_t temp_msb = _wire->read();            // Byte 4: Temperature [23:16]
  uint8_t temp_mid = _wire->read();            // Byte 5: Temperature [15:8]
  uint8_t temp_lsb = _wire->read();            // Byte 6: Temperature [7:0]

  // Parse individual status bits according to Table 20
  bool powered = (status_byte & 0x40) != 0;        // Bit 6
  bool busy = (status_byte & 0x20) != 0;           // Bit 5
  bool memory_error = (status_byte & 0x04) != 0;   // Bit 2
  bool math_sat = (status_byte & 0x01) != 0;       // Bit 0

  // Check for error conditions
  if (!powered) {
    Serial.printf("[%s] ❌ ABP2 error: Device not powered (status=0x%02X)\n", _name, status_byte);
    pressure_kpa = 0.0;
    return false;
  }

  if (memory_error) {
    Serial.printf("[%s] ❌ ABP2 error: Memory integrity failed (status=0x%02X)\n", _name, status_byte);
    pressure_kpa = 0.0;
    return false;
  }

  // Warnings (non-fatal)
  static bool warn_once_sat = false;
  if (math_sat && !warn_once_sat) {
    Serial.printf("[%s] ⚠️  ABP2 warning: Math saturation (status=0x%02X)\n", _name, status_byte);
    warn_once_sat = true;
  }

  static bool warn_once_busy = false;
  if (busy && !warn_once_busy) {
    Serial.printf("[%s] ⚠️  ABP2 warning: Busy flag set during read (status=0x%02X)\n", _name, status_byte);
    warn_once_busy = true;
  }

  // Extract 24-bit pressure value
  uint32_t pressure_counts = ((uint32_t)press_msb << 16) |
                              ((uint32_t)press_mid << 8) |
                              press_lsb;

  // Debug output for first few reads
  // static uint8_t debug_count = 0;
  // if (debug_count < 5) {
  //   Serial.printf("[%s] ABP2: Status=0x%02X [PWR=%d BSY=%d MEM=%d SAT=%d], Press=0x%06lX (%lu counts)\n",
  //                 _name, status_byte, powered, busy, memory_error, math_sat,
  //                 (unsigned long)pressure_counts, (unsigned long)pressure_counts);
  //   debug_count++;
  // }

  // Convert counts to pressure using 24-bit transfer function
  // P = (counts - 0.1*2^24) * (Pmax - Pmin) / (0.8*2^24)
  // For 150 PSI gauge: Pmin = 0, Pmax = 150 PSI
  // Outputmin = 0.1 * 2^24 = 1,677,722 counts
  // Outputmax = 0.9 * 2^24 = 15,099,494 counts
  // Output span = 15,099,494 - 1,677,722 = 13,421,772 counts

  float pressure_psi = ((float)pressure_counts - 1677722.0) * 150.0 / 13421772.0;

  // Convert PSI to kPa (1 PSI = 6.89476 kPa)
  pressure_kpa = pressure_psi * 6.89476;

  return true;
}

// ============================================================================
// ABPDLNN100MG2A3 Low Pressure Sensor Methods
// ============================================================================
// Honeywell ABPDLNN100MG2A3 - 0-100 mbar differential/low pressure sensor
// 14-bit digital output with temperature
// Based on SSC protocol (similar to ABP2 but different output format)

bool SensorReader::readABPDPressureTemp(float& pressure_kpa, float& temperature_c, uint8_t& status_byte) {
  // Request 4 bytes from ABPD sensor
  uint8_t bytesRead = _wire->requestFrom(I2Cadr_ABPD, (uint8_t)4);

  if (bytesRead != 4) {
    Serial.printf("[%s] ❌ ABPD error: Expected 4 bytes, got %d\n", _name, bytesRead);
    pressure_kpa = 0.0;
    temperature_c = 0.0;
    return false;
  }

  // Read all 4 bytes
  uint8_t msb = _wire->read();      // Byte 0: Status (2 bits) + Pressure MSB (6 bits)
  uint8_t lsb = _wire->read();      // Byte 1: Pressure LSB (8 bits)
  uint8_t temp_msb = _wire->read(); // Byte 2: Temperature MSB (8 bits)
  uint8_t temp_lsb = _wire->read(); // Byte 3: Temperature LSB (3 bits valid)

  // Extract status bits (bits 7-6 of first byte)
  status_byte = msb >> 6;

  // Extract 14-bit pressure value
  // Pressure is in bits [13:0] across bytes 0-1
  // Mask out the 2 status bits from MSB, then combine with LSB
  uint16_t pressure_counts = ((msb & 0x3F) << 8) | lsb;

  // Extract 11-bit temperature value
  // Temperature is in bits [10:0] across bytes 2-3
  // Byte 2 has bits [10:3], Byte 3 has bits [2:0] in upper 3 bits
  uint16_t temp_counts = (temp_msb << 3) | (temp_lsb >> 5);

  // Convert pressure counts to pressure using transfer function
  // P = (counts - Output_min) * (Pmax - Pmin) / (Output_max - Output_min) + Pmin
  // For 100 mbar range (0-100 mbar):
  //   Output_min = 10% of 2^14 = 1638 counts
  //   Output_max = 90% of 2^14 = 14745 counts
  //   Pmin = 0 mbar, Pmax = 100 mbar
  //   Span = 14745 - 1638 = 13107 counts

  float pressure_mbar = ((float)pressure_counts - 1638.0) * 100.0 / 13107.0;

  // Convert mbar to kPa (1 mbar = 0.1 kPa)
  pressure_kpa = pressure_mbar * 0.1;

  // Convert temperature counts to temperature in °C
  // T = (counts / 2047) * 200 - 50
  // Range: -50°C to +150°C over 11 bits (0-2047)
  temperature_c = ((float)temp_counts / 2047.0) * 200.0 - 50.0;

  return true;
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
  Serial.printf("[%s] Sending SFM3505 command: 0x%04X to address 0x%02X\n", _name, command, I2Cadr_SFM3505);
  Serial.printf("[%s]    MSB=0x%02X, LSB=0x%02X\n", _name, (uint8_t)(command >> 8), (uint8_t)(command & 0xFF));

  // Try with explicit stop=true (default behavior)
  _wire->beginTransmission(I2Cadr_SFM3505);

  size_t bytes_written = _wire->write((uint8_t)(command >> 8));    // MSB
  bytes_written += _wire->write((uint8_t)(command & 0xFF));  // LSB

  Serial.printf("[%s]    Wrote %d bytes to buffer\n", _name, bytes_written);

  byte error = _wire->endTransmission(true);  // Explicit STOP condition

  if (error != 0) {
    Serial.printf("[%s] ❌ Command send failed with error: %d\n", _name, error);
    Serial.printf("[%s]    Error codes: 1=data too long, 2=NACK on addr, 3=NACK on data, 4=other, 5=timeout\n", _name);

    // Try to diagnose - check if it's NACK on address vs data
    if (error == 2) {
      Serial.printf("[%s]    Error 2 = NACK on transmit ADDRESS\n", _name);
      Serial.printf("[%s]    Device may be in sleep mode or wrong sensor type\n", _name);
    }

    return false;
  }

  Serial.printf("[%s] ✅ Command sent successfully\n", _name);
  return true;
}

bool SensorReader::readSFM3505Raw(uint8_t* buffer, uint8_t length) {
  // SFM3505 byte format (verified from colleague's working code):
  // Byte 0: Air Flow [23:16]
  // Byte 1: Air Flow [15:8]
  // Byte 2: CRC1 (covers bytes 0-1)
  // Byte 3: Air Flow [7:0]
  // Byte 4: O2 Flow [23:16]
  // Byte 5: CRC2 (covers byte 4)
  // Byte 6: O2 Flow [15:8]
  // Byte 7: O2 Flow [7:0]
  // Byte 8: CRC3 (covers bytes 6-7)

  uint8_t bytesRead = _wire->requestFrom(I2Cadr_SFM3505, (uint8_t)9);

  if (bytesRead != 9) {
    Serial.printf("[%s] SFM3505 read error: expected 9 bytes, got %d\n", _name, bytesRead);
    return false;
  }

  // Read all bytes into temporary buffer for CRC validation
  uint8_t rawData[9];
  for (uint8_t i = 0; i < 9; i++) {
    rawData[i] = _wire->read();
  }

  // Debug: Print raw data received (only first time or on error)
  // static bool firstRead = true;
  // if (firstRead) {
  //   Serial.printf("[%s] SFM3505 raw data (first read): ", _name);
  //   for (uint8_t i = 0; i < 9; i++) {
  //     Serial.printf("0x%02X ", rawData[i]);
  //   }
  //   Serial.println();
  //   firstRead = false;
  // }

#if SFM3505_ENABLE_CRC_CHECK
  // Validate CRC1 (covers Air Flow bytes [23:16] and [15:8])
  uint8_t crc1_calculated = calculateCRC8(&rawData[0], 2);
  if (crc1_calculated != rawData[2]) {
    Serial.printf("[%s] ❌ SFM3505 CRC1 error: calculated 0x%02X, received 0x%02X\n",
                  _name, crc1_calculated, rawData[2]);
    Serial.printf("[%s]    Data: [0]=0x%02X [1]=0x%02X\n", _name, rawData[0], rawData[1]);
    return false;
  }

  // Validate CRC2 (covers O2 Flow byte [23:16])
  uint8_t crc2_calculated = calculateCRC8(&rawData[4], 1);
  if (crc2_calculated != rawData[5]) {
    Serial.printf("[%s] ❌ SFM3505 CRC2 error: calculated 0x%02X, received 0x%02X\n",
                  _name, crc2_calculated, rawData[5]);
    Serial.printf("[%s]    Data: [4]=0x%02X\n", _name, rawData[4]);
    return false;
  }

  // Validate CRC3 (covers O2 Flow bytes [15:8] and [7:0])
  uint8_t crc3_calculated = calculateCRC8(&rawData[6], 2);
  if (crc3_calculated != rawData[8]) {
    Serial.printf("[%s] ❌ SFM3505 CRC3 error: calculated 0x%02X, received 0x%02X\n",
                  _name, crc3_calculated, rawData[8]);
    Serial.printf("[%s]    Data: [6]=0x%02X [7]=0x%02X\n", _name, rawData[6], rawData[7]);
    return false;
  }

  Serial.printf("[%s] ✅ All CRCs valid\n", _name);
#endif
  // Note: CRC validation is currently disabled in SensorReader.hpp
  // Data is read without CRC checking for maximum reliability

  // All CRCs valid - extract data bytes (CORRECTED byte order)
  buffer[0] = rawData[0];  // Air [23:16]
  buffer[1] = rawData[1];  // Air [15:8]
  buffer[2] = rawData[3];  // Air [7:0] - AFTER CRC1!
  buffer[3] = rawData[4];  // O2 [23:16]
  buffer[4] = rawData[6];  // O2 [15:8] - AFTER CRC2!
  buffer[5] = rawData[7];  // O2 [7:0]

  // Debug: Show extracted values
  // static uint8_t debugCount = 0;
  // if (debugCount < 3) {  // Only show first 3 reads
  //   uint32_t airRaw = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
  //   uint32_t o2Raw = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
  //   float airFlow = scaleSFM3505Flow(airRaw);
  //   float o2Flow = scaleSFM3505Flow(o2Raw);
  //
  //   Serial.printf("[%s] Air: raw=0x%06X, flow=%.2f slm | O2: raw=0x%06X, flow=%.2f slm\n",
  //                 _name, airRaw, airFlow, o2Raw, o2Flow);
  //   debugCount++;
  // }

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
