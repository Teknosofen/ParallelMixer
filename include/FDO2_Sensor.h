#ifndef FDO2_SENSOR_H
#define FDO2_SENSOR_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Status bit definitions from datasheet
#define FDO2_STATUS_DETECTOR_REDUCED    (1 << 0)  // WARNING: Detector amplification reduced
#define FDO2_STATUS_SIGNAL_TOO_LOW      (1 << 1)  // FATAL: O2 signal too low
#define FDO2_STATUS_SIGNAL_TOO_HIGH     (1 << 2)  // FATAL: O2 signal too high
#define FDO2_STATUS_REF_TOO_LOW         (1 << 3)  // FATAL: O2 reference too low
#define FDO2_STATUS_REF_TOO_HIGH        (1 << 4)  // FATAL: O2 reference too high
#define FDO2_STATUS_TEMP_FAILURE        (1 << 5)  // FATAL: Temperature sensor failure
#define FDO2_STATUS_HUMIDITY_HIGH       (1 << 7)  // WARNING: Humidity >90%RH
#define FDO2_STATUS_PRESSURE_FAILURE    (1 << 9)  // ERROR: Pressure sensor failure
#define FDO2_STATUS_HUMIDITY_FAILURE    (1 << 10) // ERROR: Humidity sensor failure

class FDO2_Sensor {
public:
    // Measurement data structure
    struct MeasurementData {
        float oxygenPartialPressure_hPa;  // O2 partial pressure in hPa
        float temperature_C;                // Temperature in °C
        uint32_t status;                    // Status bits
        bool valid;                         // Overall validity flag
        
        // Extended data (from #MRAW command)
        float phaseShift_deg;               // Phase shift in degrees
        float signalIntensity_mV;           // Signal intensity in mV
        float ambientLight_mV;              // Ambient light in mV
        float ambientPressure_mbar;         // Ambient pressure in mbar
        float relativeHumidity_percent;     // Relative humidity in %
    };

    struct DeviceInfo {
        uint8_t deviceId;
        uint8_t numChannels;
        uint16_t firmwareRevision;
        uint8_t sensorTypes;
        uint64_t uniqueId;
    };

    // Constructor
    FDO2_Sensor(HardwareSerial& serial = Serial2);
    
    // Initialization
    bool begin(uint32_t baudRate = 19200, int8_t rxPin = -1, int8_t txPin = -1);
    void end();
    
    // Device information
    bool getDeviceInfo(DeviceInfo& info);
    bool getUniqueId(uint64_t& id);
    bool indicateLogo();  // Flash LED 4 times for identification
    
    // Measurement commands (blocking)
    bool measureOxygen(MeasurementData& data);
    bool measureOxygenRaw(MeasurementData& data);  // Includes extended raw data

    // Asynchronous measurement (non-blocking) - use this in main loop
    bool startMeasurementAsync(bool includeRaw = true);  // Start async measurement
    bool isResponseReady();                               // Check if response received (non-blocking)
    bool getAsyncResult(MeasurementData& data);          // Parse result if ready
    
    // Calibration commands (WARNING: These consume flash cycles!)
    bool calibrateZeroOxygen();           // Calibrate at 0% O2
    bool calibrateAtPartialPressure(float pO2_hPa);  // Calibrate at given pO2
    
    // Configuration
    bool setBaudRate(uint32_t baudRate);
    bool enableCRC(bool enable);          // Enable/disable CRC checking
    bool enableBroadcast(uint16_t interval_ms);  // 100-10000 ms
    bool disableBroadcast();
    
    // User memory (64 x 32-bit signed integers in flash)
    bool readUserMemory(uint8_t startAddr, uint8_t count, int32_t* values);
    bool writeUserMemory(uint8_t startAddr, uint8_t count, const int32_t* values);
    
    // Utility functions
    bool isMeasurementValid(const MeasurementData& data);
    bool hasFatalError(uint32_t status);
    bool hasWarning(uint32_t status);
    String getStatusString(uint32_t status);
    float convertToPercentO2(float pO2_hPa, float ambientPressure_mbar);
    
    // Timeout settings
    void setTimeout(uint32_t timeout_ms) { _timeout = timeout_ms; }
    
    // Get last error
    int16_t getLastError() { return _lastError; }
    String getLastErrorString();

private:
    HardwareSerial& _serial;
    uint32_t _timeout;
    int16_t _lastError;
    bool _crcEnabled;

    // Async state tracking
    String _asyncResponse;
    bool _asyncPending;
    bool _asyncIncludeRaw;
    uint32_t _asyncStartTime;
    
    // Command sending and response parsing
    bool sendCommand(const String& cmd);
    bool waitForResponse(String& response);
    bool parseResponse(const String& expected, const String& response);
    
    // CRC calculation (CRC-16-IBM / MODBUS)
    uint16_t calculateCRC16(const String& data);
    bool verifyCRC(const String& response);
    
    // Response parsing helpers
    bool parseDeviceInfo(const String& response, DeviceInfo& info);
    bool parseMeasurement(const String& response, MeasurementData& data, bool includeRaw);
    bool parseUniqueId(const String& response, uint64_t& id);
    bool parseError(const String& response, int16_t& errorCode);
    
    // Utility functions
    void clearSerialBuffer();
    int32_t parseSignedInt(const String& str);
    uint64_t parseUnsigned64(const String& str);
};

#endif // FDO2_SENSOR_H
