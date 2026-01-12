#include "SerialActuatorReader.hpp"

SerialActuatorReader::SerialActuatorReader(HardwareSerial* serial)
  : _serial(serial),
    _valveActuatorCurrent(0.0),
    _valveActuatorMisc(0.0),
    _valveActuatorMiscCmd('\0'),
    _currentTimestamp(0),
    _miscTimestamp(0) {
  _receiveBuffer.reserve(SERIAL_ACTUATOR_MAX_MSG_LEN);
}

void SerialActuatorReader::begin() {
  _receiveBuffer = "";
  _valveActuatorCurrent = 0.0;
  _valveActuatorMisc = 0.0;
  _valveActuatorMiscCmd = '\0';
  _currentTimestamp = 0;
  _miscTimestamp = 0;
}

void SerialActuatorReader::update() {
  // Process all available characters
  while (_serial->available()) {
    char c = _serial->read();

    if (c == '\n') {
      // End of message - process it
      if (_receiveBuffer.length() >= 2) {
        processMessage(_receiveBuffer);
      }
      // Clear buffer for next message
      _receiveBuffer = "";
    } else if (c == '\r') {
      // Ignore carriage return
      continue;
    } else {
      // Add character to buffer
      _receiveBuffer += c;

      // Prevent buffer overflow
      if (_receiveBuffer.length() >= SERIAL_ACTUATOR_MAX_MSG_LEN) {
        // Buffer overflow - discard and start fresh
        _receiveBuffer = "";
      }
    }
  }
}

float SerialActuatorReader::getValveActuatorCurrent() const {
  return _valveActuatorCurrent;
}

float SerialActuatorReader::getValveActuatorMisc() const {
  return _valveActuatorMisc;
}

char SerialActuatorReader::getValveActuatorMiscCommand() const {
  
  return _valveActuatorMiscCmd;
}

uint32_t SerialActuatorReader::getCurrentTimestamp() const {
  return _currentTimestamp;
}

uint32_t SerialActuatorReader::getMiscTimestamp() const {
  return _miscTimestamp;
}

bool SerialActuatorReader::isCurrentStale(uint32_t max_age_ms) const {
  if (_currentTimestamp == 0) {
    return true;  // Never received
  }
  return (millis() - _currentTimestamp) > max_age_ms;
}

bool SerialActuatorReader::isMiscStale(uint32_t max_age_ms) const {
  if (_miscTimestamp == 0) {
    return true;  // Never received
  }
  return (millis() - _miscTimestamp) > max_age_ms;
}

void SerialActuatorReader::clearBuffer() {
  _receiveBuffer = "";
  // Also clear any pending data in hardware buffer
  while (_serial->available()) {
    _serial->read();
  }
}

void SerialActuatorReader::processMessage(const String& message) {
  // Message format: <CHR>FLOAT
  // Example: "I1.25" or "V50.5"

  if (message.length() < 2) {
    return;  // Invalid message
  }

  // Extract command character and value
  char command = message[0];
  float value = message.substring(1).toFloat();

  // Store based on command type
  if (command == 'I' || command == 'i') {
    // Current measurement
    _valveActuatorCurrent = value;
    _currentTimestamp = millis();

//    Serial.printf("[SerialActuatorReader] Misc: %c%.2f timestamp: %lu\n", command, value, micros());

    #ifdef DEBUG_SERIAL_ACTUATOR_READER
    Serial.printf("[SerialActuatorReader] Current: %.3f A\n", value);
    #endif
  } else {
    // Misc data (V, P, F, or any other command)
    _valveActuatorMisc = value;
    _valveActuatorMiscCmd = command;
    _miscTimestamp = millis();

    #ifdef DEBUG_SERIAL_ACTUATOR_READER
    Serial.printf("[SerialActuatorReader] Misc: %c%.2f timestamp: %lu\n", command, value, _miscTimestamp);

    #endif
  }
}

// ============================================================================
// Serial Transmission Methods
// ============================================================================

bool SerialActuatorReader::sendSerialCommand(char command, float value) {
  // Send command: <CHR>FLOAT\n
  // Note: Caller is responsible for value validation/limiting
  String cmd = String(command) + String(value, 2) + "\n";
  _serial->print(cmd);

  // Optional: Add debug output
  #ifdef DEBUG_SERIAL_ACTUATOR
  Serial.printf("[SerialActuatorReader] Sent: %s", cmd.c_str());
  #endif

  return true;
}

bool SerialActuatorReader::readSerialResponse(char& command, float& value, uint32_t timeout_ms) {
  // Read response from serial with timeout
  // Expected format: <CHR>FLOAT\n

  uint32_t start_time = millis();
  String response = "";

  while (millis() - start_time < timeout_ms) {
    if (_serial->available()) {
      char c = _serial->read();

      if (c == '\n') {
        // End of message, parse it
        if (response.length() >= 2) {
          command = response[0];
          value = response.substring(1).toFloat();

          #ifdef DEBUG_SERIAL_ACTUATOR
          Serial.printf("[SerialActuatorReader] Received: %c%.2f\n", command, value);
          #endif

          return true;
        }
        return false;  // Malformed message
      } else {
        response += c;

        // Prevent buffer overflow
        if (response.length() > 32) {
          return false;
        }
      }
    }

    // Small delay to prevent tight polling
    delayMicroseconds(100);
  }

  // Timeout occurred
  #ifdef DEBUG_SERIAL_ACTUATOR
  Serial.println("[SerialActuatorReader] Read timeout");
  #endif

  return false;
}

bool SerialActuatorReader::readSerialMeasurement(char& command, float& value, uint32_t timeout_ms) {
  // Alias for readSerialResponse - same protocol for measurements
  // Kept as separate method for semantic clarity
  return readSerialResponse(command, value, timeout_ms);
}
