#include "SensorReader.hpp"

SensorReader::SensorReader(TwoWire* wire, const char* name, uint32_t clockFreq)
  : _wire(wire), _name(name), _clockFreq(clockFreq),
    _hasSFM3505(false), _hasSFM3304(false), _hasABP2(false), _hasELVH(false),
    _sfm3304_scaleFactor(120), _sfm3304_offset(0) {
  // Detection flags will be set during initialize()
  // SFM3304 defaults: scale=120, offset=0 (typical for air calibration)
}

bool SensorReader::initialize() {
  // NOTE: Wire.begin() should be called in main() before calling initialize()
  // This allows main to control I2C pin configuration

  _wire->setClock(_clockFreq);  // Set I2C clock frequency (passed in constructor)

  Serial.printf("Initializing %s sensors (I2C clock: %lu Hz)...\n", _name, (unsigned long)_clockFreq);

  // Reset detection flags
  _hasSFM3505 = false;
  _hasSFM3304 = false;
  _hasABP2 = false;
  _hasELVH = false;

  // ============================================================================
  // SFM3505 Flow Sensor at 0x2E
  // NOTE: The I2C bus scan in main.cpp sends empty writes to 0x2E which can
  // corrupt the SFM3505 I2C state machine. We do a General Call Reset first
  // to recover, then skip the detection scan and go straight to start command.
  // ============================================================================

  // General Call Reset — resets all Sensirion sensors on this bus
  Serial.printf("[%s] Sending I2C General Call Reset (0x00, 0x06)...\n", _name);
  _wire->beginTransmission(0x00);  // General Call address
  _wire->write(0x06);              // Reset command
  _wire->endTransmission();
  delay(100);  // SFM3505 needs up to 100ms to boot after reset

  // Now start continuous measurement directly — no detection scan needed
  Serial.printf("[%s] Starting SFM3505 continuous measurement at 0x%02X...\n", _name, I2Cadr_SFM3505);
  if (startSFM3505Measurement()) {
    Serial.printf("[%s] ✅ SFM3505 detected and measurement started\n", _name);
    _hasSFM3505 = true;
  } else {
    // Retry once — sensor may need more time after reset
    delay(50);
    Serial.printf("[%s] Retrying SFM3505 start...\n", _name);
    if (startSFM3505Measurement()) {
      Serial.printf("[%s] ✅ SFM3505 measurement started (retry)\n", _name);
      _hasSFM3505 = true;
    } else {
      Serial.printf("[%s] ⚠️ No SFM3505 at 0x%02X (start command failed)\n", _name, I2Cadr_SFM3505);
    }
  }

  // ============================================================================
  // Detect Pressure Sensors (ABP2 at 0x28, ELVH at 0x48 - no conflict)
  // ============================================================================
  // ABP2 high pressure sensor
  Serial.printf("[%s] Checking for ABP2 at address 0x%02X...\n", _name, I2Cadr_ABP2);
  _wire->beginTransmission(I2Cadr_ABP2);
  byte abp2_error = _wire->endTransmission();

  if (abp2_error == 0) {
    Serial.printf("[%s] ✅ ABP2 detected (asynchronous measurement)\n", _name);
    _hasABP2 = true;
  } else {
    Serial.printf("[%s] ⚠️ ABP2 not detected at 0x%02X\n", _name);
  }

  // ELVH low pressure sensor
  Serial.printf("[%s] Checking for ELVH at address 0x%02X...\n", _name, I2Cadr_ELVH);
  _wire->beginTransmission(I2Cadr_ELVH);
  byte elvh_error = _wire->endTransmission();

  if (elvh_error == 0) {
    Serial.printf("[%s] ✅ ELVH detected\n", _name);
    _hasELVH = true;

    // Per DS_0376: the detection scan (START+addr+STOP without clock data)
    // corrupts the ELVH I2C state. Do a dummy read to clear the error,
    // then a second read to get valid data (datasheet says "a second Start
    // condition must be set, which clears the error").
    _wire->requestFrom(I2Cadr_ELVH, (uint8_t)4);  // Recovery read (may return garbage)
    while (_wire->available()) _wire->read();       // Flush
    delay(5);
    _wire->requestFrom(I2Cadr_ELVH, (uint8_t)4);  // Second read to confirm clean state
    while (_wire->available()) _wire->read();       // Flush
    Serial.printf("[%s] ELVH I2C state recovered after detection scan\n", _name);
  } else {
    Serial.printf("[%s] ⚠️ ELVH not detected at 0x%02X\n", _name);
  }

  // ============================================================================
  // Summary
  // ============================================================================
  int detectedCount = (_hasSFM3505 ? 1 : 0) + (_hasSFM3304 ? 1 : 0) + (_hasABP2 ? 1 : 0) + (_hasELVH ? 1 : 0);
  Serial.printf("[%s] Sensor detection complete: %d sensors found\n", _name, detectedCount);

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
// ELVH-M100D Low Pressure Sensor Methods
// ============================================================================
// ELVH-M100D...4A4 - 0-100 mbar differential/low pressure sensor at 0x48
// 14-bit digital output with temperature
//
// IMPORTANT (per DS_0376 datasheet):
//   The ELVH continuously measures. Just send START + address(READ) + read 4 bytes + STOP.
//   Do NOT send START+address(WRITE)+STOP without data — this creates a communication
//   error that requires an extra START to recover from.

bool SensorReader::readELVHPressureTemp(float& pressure_mbar, float& temperature_c, uint8_t& status_byte) {
  // Direct read: START + slave address (read) + 4 data bytes + STOP
  uint8_t bytesRead = _wire->requestFrom(I2Cadr_ELVH, (uint8_t)4);

  if (bytesRead != 4) {
    Serial.printf("[%s] ❌ ELVH error: Expected 4 bytes, got %d\n", _name, bytesRead);
    pressure_mbar = 0.0;
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
  uint16_t pressure_counts = ((msb & 0x3F) << 8) | lsb;

  // Extract 11-bit temperature value
  uint16_t temp_counts = (temp_msb << 3) | (temp_lsb >> 5);

  // Convert pressure counts to mbar using transfer function
  // For ELVHM1DHRRDCN4A4: differential ±100 mbar range (-100 to +100 mbar)
  //   Output_min = 10% of 2^14 = 1638 counts  → -100 mbar
  //   Output_mid = 50% of 2^14 = 8192 counts  →    0 mbar
  //   Output_max = 90% of 2^14 = 14745 counts → +100 mbar
  //   Span = 14745 - 1638 = 13107 counts over 200 mbar range
  pressure_mbar = ((float)pressure_counts - 1638.0) * 200.0 / 13107.0 - 100.0;

  // Convert temperature counts to °C
  // T = (counts / 2047) * 200 - 50
  temperature_c = ((float)temp_counts / 2047.0) * 200.0 - 50.0;

  // Debug: print first few reads to serial
  static uint8_t elvh_debug_count = 0;
  if (elvh_debug_count < 10) {
    Serial.printf("[%s] ELVH raw: S=%d P_counts=%u T_counts=%u -> %.2f mbar, %.1f C\n",
                  _name, status_byte, pressure_counts, temp_counts, pressure_mbar, temperature_c);
    elvh_debug_count++;
  }

  return true;
}

// ============================================================================
// SFM3505 Flow Sensor Methods (Minimal Implementation)
// ============================================================================

bool SensorReader::readSFM3505AirFlow(float& airFlow) {
  uint8_t buffer[6];
  
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

  // Debug: Print raw data received (first 5 reads)
  static uint8_t rawDebugCount = 0;
  if (rawDebugCount < 5) {
    Serial.printf("[%s] SFM3505 raw 9 bytes: ", _name);
    for (uint8_t i = 0; i < 9; i++) {
      Serial.printf("0x%02X ", rawData[i]);
    }
    Serial.println();
    rawDebugCount++;
  }

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
  static uint8_t debugCount = 0;
  if (debugCount < 5) {
    uint32_t airRaw = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    uint32_t o2Raw = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
    float airFlow = scaleSFM3505Flow(airRaw);
    float o2Flow = scaleSFM3505Flow(o2Raw);

    Serial.printf("[%s] Air: raw=0x%06X, flow=%.2f slm | O2: raw=0x%06X, flow=%.2f slm\n",
                  _name, airRaw, airFlow, o2Raw, o2Flow);
    debugCount++;
  }

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

// ============================================================================
// SFM3304 Flow Sensor Methods
// ============================================================================
// SFM3304-D: Single-use proximal flow sensor
//   - I2C address 0x2E (same as SFM3505!)
//   - 16-bit signed flow + 16-bit temperature + 16-bit status word
//   - Each word followed by CRC8
//   - Flow range: ±250 slm (air calibration)
//   - Max operating pressure: 1.14 bar
//   - Scale factor and offset read from sensor via 0x3661
// ============================================================================

bool SensorReader::identifySFM3304() {
  // Try to read the SFM3304 product identifier (0xE102).
  // The SFM3505 does NOT support this command (its product-ID is 0x365B)
  // so a successful read with a known product prefix means SFM3304.
  uint32_t productId;
  uint64_t serialNumber;
  if (readSFM3304ProductId(productId, serialNumber)) {
    // SFM3304 product identifiers start with 0x07050500 (datasheet Table 13)
    // Upper 24 bits encode product; lower 8 bits = revision
    uint32_t productBase = productId & 0xFFFFFF00;
    Serial.printf("[%s] Flow sensor product ID: 0x%08lX, serial: %llu\n",
                  _name, (unsigned long)productId, (unsigned long long)serialNumber);
    // Accept any SFM3304 variant (0x070505xx)
    if (productBase == 0x07050500) {
      return true;
    }
    // Also accept broadly — if the read worked and gave a valid response,
    // it's likely an SFM3304 (the SFM3505 would NAK on 0xE102)
    Serial.printf("[%s] Unknown product ID 0x%08lX — assuming SFM3304\n",
                  _name, (unsigned long)productId);
    return true;
  }
  return false;
}

bool SensorReader::sendSFM3304Command(uint16_t command) {
  _wire->beginTransmission(I2Cadr_SFM3304);
  _wire->write((uint8_t)(command >> 8));    // MSB
  _wire->write((uint8_t)(command & 0xFF));  // LSB
  byte error = _wire->endTransmission(true);
  if (error != 0) {
    Serial.printf("[%s] ❌ SFM3304 command 0x%04X failed (error=%d)\n", _name, command, error);
    return false;
  }
  return true;
}

bool SensorReader::sendSFM3304CommandWithArg(uint16_t command, uint16_t argument) {
  // Some SFM3304 commands take a 16-bit argument word + CRC
  uint8_t argBytes[2] = { (uint8_t)(argument >> 8), (uint8_t)(argument & 0xFF) };
  uint8_t argCrc = calculateCRC8(argBytes, 2);

  _wire->beginTransmission(I2Cadr_SFM3304);
  _wire->write((uint8_t)(command >> 8));
  _wire->write((uint8_t)(command & 0xFF));
  _wire->write(argBytes[0]);
  _wire->write(argBytes[1]);
  _wire->write(argCrc);
  byte error = _wire->endTransmission(true);
  if (error != 0) {
    Serial.printf("[%s] ❌ SFM3304 command 0x%04X(arg=0x%04X) failed (error=%d)\n",
                  _name, command, argument, error);
    return false;
  }
  return true;
}

bool SensorReader::startSFM3304Measurement() {
  // Start continuous measurement (air calibration): command 0x3603
  // First measurement available after 4 ms; allow ~50 ms for small
  // accuracy deviations to settle.
  if (!sendSFM3304Command(SFM3304_CMD_START_CONTINUOUS)) {
    return false;
  }
  delay(4);  // Wait for first measurement
  return true;
}

bool SensorReader::stopSFM3304Measurement() {
  // Sensor needs up to 0.5 ms to power down heater and enter idle
  if (!sendSFM3304Command(SFM3304_CMD_STOP_CONTINUOUS)) {
    return false;
  }
  delay(1);
  return true;
}

bool SensorReader::readSFM3304Data(float& flow, float& temperature, uint16_t& statusWord) {
  // During continuous measurement, read 9 bytes:
  //   Bytes 0-1: Flow (int16, signed)
  //   Byte  2:   CRC8 of bytes 0-1
  //   Bytes 3-4: Temperature (int16)
  //   Byte  5:   CRC8 of bytes 3-4
  //   Bytes 6-7: Status word (uint16)
  //   Byte  8:   CRC8 of bytes 6-7

  uint8_t bytesRead = _wire->requestFrom(I2Cadr_SFM3304, (uint8_t)9);
  if (bytesRead != 9) {
    static bool errorPrinted = false;
    if (!errorPrinted) {
      Serial.printf("[%s] SFM3304 read error: expected 9 bytes, got %d\n", _name, bytesRead);
      errorPrinted = true;
    }
    while (_wire->available()) _wire->read();  // flush
    return false;
  }

  uint8_t rawData[9];
  for (uint8_t i = 0; i < 9; i++) {
    rawData[i] = _wire->read();
  }

  // Validate CRC for each word
  uint8_t crc_flow = calculateCRC8(&rawData[0], 2);
  uint8_t crc_temp = calculateCRC8(&rawData[3], 2);
  uint8_t crc_stat = calculateCRC8(&rawData[6], 2);

  if (crc_flow != rawData[2] || crc_temp != rawData[5] || crc_stat != rawData[8]) {
    static uint32_t crcErrorCount = 0;
    crcErrorCount++;
    if (crcErrorCount <= 5) {
      Serial.printf("[%s] ⚠️ SFM3304 CRC error #%lu: flow[%02X vs %02X] temp[%02X vs %02X] stat[%02X vs %02X]\n",
                    _name, (unsigned long)crcErrorCount,
                    crc_flow, rawData[2], crc_temp, rawData[5], crc_stat, rawData[8]);
    }
    return false;
  }

  // Extract raw signed values
  int16_t rawFlow = (int16_t)((rawData[0] << 8) | rawData[1]);
  int16_t rawTemp = (int16_t)((rawData[3] << 8) | rawData[4]);
  statusWord      = (uint16_t)((rawData[6] << 8) | rawData[7]);

  // Scale to physical units
  // Flow: (rawFlow - offset) / scaleFactor   → slm
  // Temperature: rawTemp / 200.0             → °C
  if (_sfm3304_scaleFactor != 0) {
    flow = (float)(rawFlow - _sfm3304_offset) / (float)_sfm3304_scaleFactor;
  } else {
    flow = 0.0f;
  }
  temperature = (float)rawTemp / 200.0f;

  return true;
}

bool SensorReader::readSFM3304Flow(float& flow) {
  // Read only the first 3 bytes (flow word + CRC) — aborts early with NACK
  // This is faster when temperature/status are not needed.
  uint8_t bytesRead = _wire->requestFrom(I2Cadr_SFM3304, (uint8_t)3);
  if (bytesRead != 3) {
    return false;
  }

  uint8_t rawData[3];
  rawData[0] = _wire->read();
  rawData[1] = _wire->read();
  rawData[2] = _wire->read();

  // CRC check
  uint8_t crc = calculateCRC8(rawData, 2);
  if (crc != rawData[2]) {
    return false;
  }

  int16_t rawFlow = (int16_t)((rawData[0] << 8) | rawData[1]);
  if (_sfm3304_scaleFactor != 0) {
    flow = (float)(rawFlow - _sfm3304_offset) / (float)_sfm3304_scaleFactor;
  } else {
    flow = 0.0f;
  }
  return true;
}

bool SensorReader::readSFM3304ScaleOffset(int16_t& scaleFactor, int16_t& offset) {
  // Command 0x3661 with argument = start measurement command code (0x3603 for air)
  // Must be in idle mode (not measuring) to call this.
  // Response: 9 bytes = scaleFactor(2) + CRC + offset(2) + CRC + unit(2) + CRC

  if (!sendSFM3304CommandWithArg(SFM3304_CMD_READ_SCALE_OFFSET,
                                  SFM3304_CMD_START_CONTINUOUS)) {
    return false;
  }
  delay(2);

  uint8_t bytesRead = _wire->requestFrom(I2Cadr_SFM3304, (uint8_t)9);
  if (bytesRead != 9) {
    Serial.printf("[%s] SFM3304 readScaleOffset: expected 9 bytes, got %d\n", _name, bytesRead);
    while (_wire->available()) _wire->read();
    return false;
  }

  uint8_t rawData[9];
  for (uint8_t i = 0; i < 9; i++) {
    rawData[i] = _wire->read();
  }

  // Validate CRCs
  if (calculateCRC8(&rawData[0], 2) != rawData[2] ||
      calculateCRC8(&rawData[3], 2) != rawData[5] ||
      calculateCRC8(&rawData[6], 2) != rawData[8]) {
    Serial.printf("[%s] ❌ SFM3304 readScaleOffset CRC error\n", _name);
    return false;
  }

  scaleFactor = (int16_t)((rawData[0] << 8) | rawData[1]);
  offset      = (int16_t)((rawData[3] << 8) | rawData[4]);
  uint16_t unit = (uint16_t)((rawData[6] << 8) | rawData[7]);

  Serial.printf("[%s] SFM3304 scale=%d, offset=%d, unit=0x%04X\n",
                _name, scaleFactor, offset, unit);
  return true;
}

bool SensorReader::readSFM3304ProductId(uint32_t& productId, uint64_t& serialNumber) {
  // Command 0xE102 — only works in idle mode
  // Response: 18 bytes = productId(4 bytes in 2 words with CRC) + serialNumber(8 bytes in 4 words with CRC)
  // = 2×(2+1) + 4×(2+1) = 6 + 12 = 18 bytes

  if (!sendSFM3304Command(SFM3304_CMD_READ_PRODUCT_ID)) {
    return false;
  }
  delay(2);

  uint8_t bytesRead = _wire->requestFrom(I2Cadr_SFM3304, (uint8_t)18);
  if (bytesRead != 18) {
    Serial.printf("[%s] SFM3304 readProductId: expected 18 bytes, got %d\n", _name, bytesRead);
    while (_wire->available()) _wire->read();
    return false;
  }

  uint8_t rawData[18];
  for (uint8_t i = 0; i < 18; i++) {
    rawData[i] = _wire->read();
  }

  // Validate CRCs for all 6 words
  for (uint8_t w = 0; w < 6; w++) {
    uint8_t idx = w * 3;
    if (calculateCRC8(&rawData[idx], 2) != rawData[idx + 2]) {
      Serial.printf("[%s] ❌ SFM3304 productId CRC error at word %d\n", _name, w);
      return false;
    }
  }

  // Extract product identifier (2 words = 4 data bytes)
  productId = ((uint32_t)rawData[0] << 24) | ((uint32_t)rawData[1] << 16) |
              ((uint32_t)rawData[3] << 8)  | rawData[4];

  // Extract serial number (4 words = 8 data bytes)
  serialNumber = ((uint64_t)rawData[6]  << 56) | ((uint64_t)rawData[7]  << 48) |
                 ((uint64_t)rawData[9]  << 40) | ((uint64_t)rawData[10] << 32) |
                 ((uint64_t)rawData[12] << 24) | ((uint64_t)rawData[13] << 16) |
                 ((uint64_t)rawData[15] << 8)  | (uint64_t)rawData[16];

  return true;
}

bool SensorReader::configureSFM3304Averaging(uint16_t avgWindow) {
  // Command 0x366A with argument = averaging window
  // N=0: average-until-read (default)
  // N=1..128: fixed-N averaging
  // Must be in idle mode (stop measurement first, then restart)
  return sendSFM3304CommandWithArg(SFM3304_CMD_CONFIGURE_AVERAGING, avgWindow);
}
