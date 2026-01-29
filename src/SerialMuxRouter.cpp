#include "SerialMuxRouter.hpp"

// Static constant definition
const float SerialMuxRouter::INVALID_VALUE = -9.9f;

SerialMuxRouter::SerialMuxRouter(HardwareSerial* serial)
  : _serial(serial) {
  _receiveBuffer.reserve(MUX_SERIAL_MAX_MSG_LEN);
}

void SerialMuxRouter::begin() {
  _receiveBuffer = "";

  // Initialize all per-address data
  for (uint8_t i = 0; i < MAX_ADDRESSES; i++) {
    _current[i] = INVALID_VALUE;
    _blowerRPM[i] = INVALID_VALUE;
    _actualFlow[i] = INVALID_VALUE;
    _actualPressure[i] = INVALID_VALUE;

    _currentTimestamp[i] = 0;
    _blowerRPMTimestamp[i] = 0;
    _actualFlowTimestamp[i] = 0;
    _actualPressureTimestamp[i] = 0;
  }
}

void SerialMuxRouter::update() {
  // Process all available characters
  while (_serial->available()) {
    char c = _serial->read();

    if (c == '\n') {
      // End of message - process it
      if (_receiveBuffer.length() >= 2) {
        processMessage(_receiveBuffer);
      }
      _receiveBuffer = "";
    } else if (c == '\r') {
      // Ignore carriage return
      continue;
    } else {
      _receiveBuffer += c;

      // Prevent buffer overflow
      if (_receiveBuffer.length() >= MUX_SERIAL_MAX_MSG_LEN) {
        _receiveBuffer = "";
      }
    }
  }
}

void SerialMuxRouter::clearBuffer() {
  _receiveBuffer = "";
  while (_serial->available()) {
    _serial->read();
  }
}

void SerialMuxRouter::processMessage(const String& message) {
  // Message format:
  //   Without MUX: <CMD>VALUE      (e.g., "I1.25")
  //   With MUX:    <ADDR><CMD>VALUE (e.g., "3I1.25")

  if (message.length() < 2) {
    return;  // Invalid message
  }

  uint8_t address = 0;  // Default to address 0 (no MUX)
  size_t cmdIndex = 0;

  // Check if first character is a digit (address prefix)
  char firstChar = message[0];
  if (firstChar >= '0' && firstChar <= '9') {
    address = firstChar - '0';
    cmdIndex = 1;

    // Need at least 2 more characters after address
    if (message.length() < 3) {
      return;  // Invalid message
    }
  }

  // Extract command character and value
  char command = message[cmdIndex];
  float value = message.substring(cmdIndex + 1).toFloat();
  uint32_t timestamp = millis();

  // Route to correct storage based on command
  switch (command) {
    case CMD_RX_CURRENT:
    case 'i':  // Allow lowercase too
      _current[address] = value;
      _currentTimestamp[address] = timestamp;
      #ifdef DEBUG_SERIAL_MUX_ROUTER
      Serial.printf("[MuxRouter] Addr %d Current: %.3f A\n", address, value);
      #endif
      break;

    case CMD_RX_BLOWER_RPM:
    case 'r':  // Allow lowercase too
      _blowerRPM[address] = value;
      _blowerRPMTimestamp[address] = timestamp;
      #ifdef DEBUG_SERIAL_MUX_ROUTER
      Serial.printf("[MuxRouter] Addr %d BlowerRPM: %.1f\n", address, value);
      #endif
      break;

    case CMD_RX_FLOW:
    case 'f':  // Allow lowercase too
      _actualFlow[address] = value;
      _actualFlowTimestamp[address] = timestamp;
      #ifdef DEBUG_SERIAL_MUX_ROUTER
      Serial.printf("[MuxRouter] Addr %d Flow: %.2f\n", address, value);
      #endif
      break;

    case CMD_RX_PRESSURE:
    case 'p':  // Allow lowercase too
      _actualPressure[address] = value;
      _actualPressureTimestamp[address] = timestamp;
      #ifdef DEBUG_SERIAL_MUX_ROUTER
      Serial.printf("[MuxRouter] Addr %d Pressure: %.2f\n", address, value);
      #endif
      break;

    default:
      // Unknown command - ignore or log
      #ifdef DEBUG_SERIAL_MUX_ROUTER
      Serial.printf("[MuxRouter] Unknown cmd '%c' from addr %d\n", command, address);
      #endif
      break;
  }
}

// ==========================================================================
// Getters
// ==========================================================================

float SerialMuxRouter::getCurrent(uint8_t address) const {
  if (!isValidAddress(address)) return INVALID_VALUE;
  return _current[address];
}

float SerialMuxRouter::getBlowerRPM(uint8_t address) const {
  if (!isValidAddress(address)) return INVALID_VALUE;
  return _blowerRPM[address];
}

float SerialMuxRouter::getActualFlow(uint8_t address) const {
  if (!isValidAddress(address)) return INVALID_VALUE;
  return _actualFlow[address];
}

float SerialMuxRouter::getActualPressure(uint8_t address) const {
  if (!isValidAddress(address)) return INVALID_VALUE;
  return _actualPressure[address];
}

// ==========================================================================
// Timestamps
// ==========================================================================

uint32_t SerialMuxRouter::getCurrentTimestamp(uint8_t address) const {
  if (!isValidAddress(address)) return 0;
  return _currentTimestamp[address];
}

uint32_t SerialMuxRouter::getBlowerRPMTimestamp(uint8_t address) const {
  if (!isValidAddress(address)) return 0;
  return _blowerRPMTimestamp[address];
}

uint32_t SerialMuxRouter::getActualFlowTimestamp(uint8_t address) const {
  if (!isValidAddress(address)) return 0;
  return _actualFlowTimestamp[address];
}

uint32_t SerialMuxRouter::getActualPressureTimestamp(uint8_t address) const {
  if (!isValidAddress(address)) return 0;
  return _actualPressureTimestamp[address];
}

// ==========================================================================
// Stale Detection
// ==========================================================================

bool SerialMuxRouter::isCurrentStale(uint8_t address, uint32_t max_age_ms) const {
  if (!isValidAddress(address)) return true;
  return isStale(_currentTimestamp[address], max_age_ms);
}

bool SerialMuxRouter::isBlowerRPMStale(uint8_t address, uint32_t max_age_ms) const {
  if (!isValidAddress(address)) return true;
  return isStale(_blowerRPMTimestamp[address], max_age_ms);
}

bool SerialMuxRouter::isActualFlowStale(uint8_t address, uint32_t max_age_ms) const {
  if (!isValidAddress(address)) return true;
  return isStale(_actualFlowTimestamp[address], max_age_ms);
}

bool SerialMuxRouter::isActualPressureStale(uint8_t address, uint32_t max_age_ms) const {
  if (!isValidAddress(address)) return true;
  return isStale(_actualPressureTimestamp[address], max_age_ms);
}

// ==========================================================================
// Send Commands
// ==========================================================================

bool SerialMuxRouter::sendSetCurrent(uint8_t address, float value) {
  return sendCommand(address, CMD_TX_SET_CURRENT, value);
}

bool SerialMuxRouter::sendSetVoltage(uint8_t address, float value) {
  return sendCommand(address, CMD_TX_SET_VOLTAGE, value);
}

bool SerialMuxRouter::sendSetFlow(uint8_t address, float value) {
  return sendCommand(address, CMD_TX_SET_FLOW, value);
}

bool SerialMuxRouter::sendSetPressure(uint8_t address, float value) {
  return sendCommand(address, CMD_TX_SET_PRESSURE, value);
}

bool SerialMuxRouter::sendSetBlowerRPM(uint8_t address, float value) {
  return sendCommand(address, CMD_TX_SET_BLOWER_RPM, value);
}

bool SerialMuxRouter::sendCommand(uint8_t address, char command, float value) {
  if (!isValidAddress(address)) {
    return false;
  }

  String msg;

  // Address 0 = no prefix (backward compatible)
  if (address > 0) {
    msg = String((char)('0' + address));
  }

  msg += String(command) + String(value, 2) + "\n";
  _serial->print(msg);

  #ifdef DEBUG_SERIAL_MUX_ROUTER
  Serial.printf("[MuxRouter] Sent to addr %d: %s", address, msg.c_str());
  #endif

  return true;
}

// ==========================================================================
// Private Helpers
// ==========================================================================

bool SerialMuxRouter::isValidAddress(uint8_t address) const {
  return address < MAX_ADDRESSES;
}

bool SerialMuxRouter::isStale(uint32_t timestamp, uint32_t max_age_ms) const {
  if (timestamp == 0) {
    return true;  // Never received
  }
  return (millis() - timestamp) > max_age_ms;
}
