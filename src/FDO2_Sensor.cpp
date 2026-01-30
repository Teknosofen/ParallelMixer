#include "FDO2_Sensor.h"

FDO2_Sensor::FDO2_Sensor(HardwareSerial& serial)
    : _serial(serial), _timeout(2000), _lastError(0), _crcEnabled(false),
      _asyncPending(false), _asyncIncludeRaw(true), _asyncStartTime(0) {
}

bool FDO2_Sensor::begin(uint32_t baudRate, int8_t rxPin, int8_t txPin) {
    // For ESP32, Serial2 needs explicit pin configuration
    if (rxPin >= 0 && txPin >= 0) {
        _serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    } else {
        _serial.begin(baudRate);
    }
    
    delay(1100);  // Wait for module power-up (needs ~1 sec)
    
    clearSerialBuffer();
    
    // Test communication by getting device info
    DeviceInfo info;
    return getDeviceInfo(info);
}

void FDO2_Sensor::end() {
    _serial.end();
}

bool FDO2_Sensor::getDeviceInfo(DeviceInfo& info) {
    if (!sendCommand("#VERS")) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    return parseDeviceInfo(response, info);
}

bool FDO2_Sensor::getUniqueId(uint64_t& id) {
    if (!sendCommand("#IDNR")) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    return parseUniqueId(response, id);
}

bool FDO2_Sensor::indicateLogo() {
    if (!sendCommand("#LOGO")) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    return response.startsWith("#LOGO");
}

bool FDO2_Sensor::measureOxygen(MeasurementData& data) {
    if (!sendCommand("#MOXY")) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    return parseMeasurement(response, data, false);
}

bool FDO2_Sensor::measureOxygenRaw(MeasurementData& data) {
    if (!sendCommand("#MRAW")) return false;

    String response;
    if (!waitForResponse(response)) return false;

    return parseMeasurement(response, data, true);
}

// ============================================================================
// Asynchronous Measurement Methods (Non-blocking)
// ============================================================================

bool FDO2_Sensor::startMeasurementAsync(bool includeRaw) {
    // Don't start if already pending
    if (_asyncPending) return false;

    clearSerialBuffer();

    String cmd = includeRaw ? "#MRAW" : "#MOXY";
    _serial.print(cmd);
    _serial.print('\r');

    _asyncPending = true;
    _asyncIncludeRaw = includeRaw;
    _asyncResponse = "";
    _asyncStartTime = millis();

    return true;
}

bool FDO2_Sensor::isResponseReady() {
    if (!_asyncPending) return false;

    // Check for timeout
    if (millis() - _asyncStartTime > _timeout) {
        _asyncPending = false;
        _lastError = -22;  // Timeout
        return false;
    }

    // Read available characters (non-blocking)
    while (_serial.available()) {
        char c = _serial.read();

        if (c == '\r') {
            // End of response - response is ready
            return true;
        }

        _asyncResponse += c;

        // Prevent buffer overflow
        if (_asyncResponse.length() > 512) {
            _asyncPending = false;
            _lastError = -22;
            return false;
        }
    }

    return false;  // Not yet complete
}

bool FDO2_Sensor::getAsyncResult(MeasurementData& data) {
    if (!_asyncPending) return false;

    // Mark as no longer pending
    _asyncPending = false;

    // Verify CRC if enabled
    if (_crcEnabled && !verifyCRC(_asyncResponse)) {
        return false;
    }

    return parseMeasurement(_asyncResponse, data, _asyncIncludeRaw);
}

bool FDO2_Sensor::calibrateZeroOxygen() {
    if (!sendCommand("#CALO")) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    return response.startsWith("#CALO");
}

bool FDO2_Sensor::calibrateAtPartialPressure(float pO2_hPa) {
    int32_t pO2_milliHPa = (int32_t)(pO2_hPa * 1000.0f);
    String cmd = "#CAHI " + String(pO2_milliHPa);
    
    if (!sendCommand(cmd)) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    return response.startsWith("#CAHI");
}

bool FDO2_Sensor::setBaudRate(uint32_t baudRate) {
    String cmd = "#BAUD " + String(baudRate);
    
    if (!sendCommand(cmd)) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    if (response.startsWith("#BAUD")) {
        delay(50);  // Allow time for baud rate to change
        _serial.updateBaudRate(baudRate);
        clearSerialBuffer();
        return true;
    }
    
    return false;
}

bool FDO2_Sensor::enableCRC(bool enable) {
    String cmd = "#CRCE " + String(enable ? 1 : 0);
    
    if (!sendCommand(cmd)) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    if (response.startsWith("#CRCE")) {
        _crcEnabled = enable;
        return true;
    }
    
    return false;
}

bool FDO2_Sensor::enableBroadcast(uint16_t interval_ms) {
    if (interval_ms < 100 || interval_ms > 10000) {
        _lastError = -1;
        return false;
    }
    
    String cmd = "#BCST " + String(interval_ms);
    
    if (!sendCommand(cmd)) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    return response.startsWith("#BCST");
}

bool FDO2_Sensor::disableBroadcast() {
    if (!sendCommand("#BCST 0")) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    return response.startsWith("#BCST");
}

bool FDO2_Sensor::readUserMemory(uint8_t startAddr, uint8_t count, int32_t* values) {
    if (startAddr > 63 || count < 1 || count > 64 || (startAddr + count) > 64) {
        _lastError = -1;
        return false;
    }
    
    String cmd = "#RDUM " + String(startAddr) + " " + String(count);
    
    if (!sendCommand(cmd)) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    // Parse response: "#RDUM R N Y1 ... YN"
    if (!response.startsWith("#RDUM")) return false;
    
    int spaceCount = 0;
    int startIdx = 0;
    
    // Skip "#RDUM R N"
    for (int i = 0; i < response.length() && spaceCount < 2; i++) {
        if (response[i] == ' ') spaceCount++;
        if (spaceCount == 2) startIdx = i + 1;
    }
    
    // Parse values
    String remaining = response.substring(startIdx);
    remaining.trim();
    
    for (uint8_t i = 0; i < count; i++) {
        int spacePos = remaining.indexOf(' ');
        String valueStr;
        
        if (spacePos == -1) {
            valueStr = remaining;
        } else {
            valueStr = remaining.substring(0, spacePos);
            remaining = remaining.substring(spacePos + 1);
        }
        
        values[i] = parseSignedInt(valueStr);
    }
    
    return true;
}

bool FDO2_Sensor::writeUserMemory(uint8_t startAddr, uint8_t count, const int32_t* values) {
    if (startAddr > 63 || count < 1 || count > 64 || (startAddr + count) > 64) {
        _lastError = -1;
        return false;
    }
    
    String cmd = "#WRUM " + String(startAddr) + " " + String(count);
    
    for (uint8_t i = 0; i < count; i++) {
        cmd += " " + String(values[i]);
    }
    
    if (!sendCommand(cmd)) return false;
    
    String response;
    if (!waitForResponse(response)) return false;
    
    return response.startsWith("#WRUM");
}

bool FDO2_Sensor::isMeasurementValid(const MeasurementData& data) {
    if (!data.valid) return false;
    
    // Check for fatal errors
    if (hasFatalError(data.status)) return false;
    
    return true;
}

bool FDO2_Sensor::hasFatalError(uint32_t status) {
    const uint32_t fatalMask = FDO2_STATUS_SIGNAL_TOO_LOW | 
                                FDO2_STATUS_SIGNAL_TOO_HIGH | 
                                FDO2_STATUS_REF_TOO_LOW | 
                                FDO2_STATUS_REF_TOO_HIGH | 
                                FDO2_STATUS_TEMP_FAILURE;
    
    return (status & fatalMask) != 0;
}

bool FDO2_Sensor::hasWarning(uint32_t status) {
    const uint32_t warningMask = FDO2_STATUS_DETECTOR_REDUCED | 
                                  FDO2_STATUS_HUMIDITY_HIGH;
    
    return (status & warningMask) != 0;
}

String FDO2_Sensor::getStatusString(uint32_t status) {
    if (status == 0) return "OK";
    
    String result = "";
    
    if (status & FDO2_STATUS_DETECTOR_REDUCED) 
        result += "WARN: Detector reduced; ";
    if (status & FDO2_STATUS_SIGNAL_TOO_LOW) 
        result += "FATAL: Signal too low; ";
    if (status & FDO2_STATUS_SIGNAL_TOO_HIGH) 
        result += "FATAL: Signal too high; ";
    if (status & FDO2_STATUS_REF_TOO_LOW) 
        result += "FATAL: Reference too low; ";
    if (status & FDO2_STATUS_REF_TOO_HIGH) 
        result += "FATAL: Reference too high; ";
    if (status & FDO2_STATUS_TEMP_FAILURE) 
        result += "FATAL: Temp sensor failure; ";
    if (status & FDO2_STATUS_HUMIDITY_HIGH) 
        result += "WARN: High humidity (>90%); ";
    if (status & FDO2_STATUS_PRESSURE_FAILURE) 
        result += "ERROR: Pressure sensor failure; ";
    if (status & FDO2_STATUS_HUMIDITY_FAILURE) 
        result += "ERROR: Humidity sensor failure; ";
    
    return result;
}

float FDO2_Sensor::convertToPercentO2(float pO2_hPa, float ambientPressure_mbar) {
    return 100.0f * pO2_hPa / ambientPressure_mbar;
}

String FDO2_Sensor::getLastErrorString() {
    switch (_lastError) {
        case 0: return "No error";
        case -1: return "General error";
        case -21: return "UART Parse error";
        case -22: return "UART Rx error";
        case -23: return "UART Header error";
        case -26: return "UART Request error";
        case -60: return "Invalid calibration";
        default: return "Unknown error: " + String(_lastError);
    }
}

// Private methods

bool FDO2_Sensor::sendCommand(const String& cmd) {
    clearSerialBuffer();
    
    _serial.print(cmd);
    _serial.print('\r');  // Command terminator
    
    return true;
}

bool FDO2_Sensor::waitForResponse(String& response) {
    response = "";
    unsigned long startTime = millis();
    bool gotData = false;
    
    while (millis() - startTime < _timeout) {
        if (_serial.available()) {
            gotData = true;
            char c = _serial.read();
            
            if (c == '\r') {
                // End of response
                if (_crcEnabled) {
                    return verifyCRC(response);
                }
                return true;
            }
            
            response += c;
            
            // Prevent buffer overflow
            if (response.length() > 512) {
                _lastError = -22;
                return false;
            }
        } else if (gotData) {
            // If we've started receiving data but nothing new arrives for 100ms, timeout
            delay(1);
        }
    }
    
    // Timeout - check if we got any data at all
    if (response.length() > 0) {
        // Got partial response
        _lastError = -22;
    } else {
        // No response at all - sensor likely not connected
        _lastError = -22;
    }
    return false;
}

bool FDO2_Sensor::parseResponse(const String& expected, const String& response) {
    // Check for error response
    if (response.startsWith("#ERRO")) {
        int16_t errorCode;
        if (parseError(response, errorCode)) {
            _lastError = errorCode;
        }
        return false;
    }
    
    return response.startsWith(expected);
}

uint16_t FDO2_Sensor::calculateCRC16(const String& data) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < data.length(); i++) {
        crc ^= (uint8_t)data[i];
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

bool FDO2_Sensor::verifyCRC(const String& response) {
    int colonPos = response.lastIndexOf(':');
    
    if (colonPos == -1) return false;
    
    String dataStr = response.substring(0, colonPos);
    String crcStr = response.substring(colonPos + 2);  // Skip ": "
    
    uint16_t receivedCRC = (uint16_t)parseSignedInt(crcStr);
    uint16_t calculatedCRC = calculateCRC16(dataStr);
    
    return receivedCRC == calculatedCRC;
}

bool FDO2_Sensor::parseDeviceInfo(const String& response, DeviceInfo& info) {
    if (!response.startsWith("#VERS")) return false;
    
    // Parse "#VERS D N R S"
    int values[4];
    int valueIdx = 0;
    String remaining = response.substring(6);  // Skip "#VERS "
    
    while (remaining.length() > 0 && valueIdx < 4) {
        int spacePos = remaining.indexOf(' ');
        String valueStr;
        
        if (spacePos == -1) {
            valueStr = remaining;
            remaining = "";
        } else {
            valueStr = remaining.substring(0, spacePos);
            remaining = remaining.substring(spacePos + 1);
        }
        
        values[valueIdx++] = parseSignedInt(valueStr);
    }
    
    if (valueIdx == 4) {
        info.deviceId = values[0];
        info.numChannels = values[1];
        info.firmwareRevision = values[2];
        info.sensorTypes = values[3];
        return true;
    }
    
    return false;
}

bool FDO2_Sensor::parseMeasurement(const String& response, MeasurementData& data, bool includeRaw) {
    data.valid = false;
    
    String prefix = includeRaw ? "#MRAW" : "#MOXY";
    if (!response.startsWith(prefix)) return false;
    
    // Parse values
    String remaining = response.substring(prefix.length() + 1);  // Skip command and space
    remaining.trim();
    
    int32_t values[8] = {0};
    int valueIdx = 0;
    int maxValues = includeRaw ? 8 : 3;
    
    while (remaining.length() > 0 && valueIdx < maxValues) {
        int spacePos = remaining.indexOf(' ');
        String valueStr;
        
        if (spacePos == -1) {
            valueStr = remaining;
            remaining = "";
        } else {
            valueStr = remaining.substring(0, spacePos);
            remaining = remaining.substring(spacePos + 1);
        }
        
        values[valueIdx++] = parseSignedInt(valueStr);
    }
    
    if (valueIdx >= 3) {
        // O, T, S are always present
        data.oxygenPartialPressure_hPa = values[0] / 1000.0f;
        data.temperature_C = values[1] / 1000.0f;
        data.status = values[2];
        
        if (includeRaw && valueIdx >= 8) {
            data.phaseShift_deg = values[3] / 1000.0f;
            data.signalIntensity_mV = values[4] / 1000.0f;
            data.ambientLight_mV = values[5] / 1000.0f;
            data.ambientPressure_mbar = values[6] / 1000.0f;
            data.relativeHumidity_percent = values[7] / 1000.0f;
        }
        
        data.valid = true;
        return true;
    }
    
    return false;
}

bool FDO2_Sensor::parseUniqueId(const String& response, uint64_t& id) {
    if (!response.startsWith("#IDNR")) return false;
    
    String idStr = response.substring(6);  // Skip "#IDNR "
    idStr.trim();
    
    id = parseUnsigned64(idStr);
    return true;
}

bool FDO2_Sensor::parseError(const String& response, int16_t& errorCode) {
    if (!response.startsWith("#ERRO")) return false;
    
    String errorStr = response.substring(6);  // Skip "#ERRO "
    errorStr.trim();
    
    errorCode = parseSignedInt(errorStr);
    return true;
}

void FDO2_Sensor::clearSerialBuffer() {
    while (_serial.available()) {
        _serial.read();
    }
}

int32_t FDO2_Sensor::parseSignedInt(const String& str) {
    return str.toInt();
}

uint64_t FDO2_Sensor::parseUnsigned64(const String& str) {
    uint64_t result = 0;
    
    for (size_t i = 0; i < str.length(); i++) {
        char c = str[i];
        if (c >= '0' && c <= '9') {
            result = result * 10 + (c - '0');
        }
    }
    
    return result;
}
