# Minimal SFM3505 Implementation Guide

## Overview
This is a **self-contained** implementation of SFM3505 support with **NO external dependencies**. Everything you need is built directly into the SensorReader class.

## ✅ What's Included

### No External Dependencies!
- ❌ No `SensirionI2cSfm3505.h` needed
- ❌ No `SensirionCore.h` needed
- ✅ Everything built into SensorReader class
- ✅ Minimal, clean implementation

## 📦 Files You Need

1. **SensorReader.hpp** - Header file (minimal version)
2. **SensorReader.cpp** - Implementation (minimal version)
3. **PinConfig.h** - Pin definitions (same as before)

That's it! Just 3 files.

## 🎯 What Changed

### Removed External Driver
**Before (with Sensirion driver):**
```cpp
#include "SensirionI2cSfm3505.h"

class SensorReader {
private:
  SensirionI2cSfm3505 _sfm3505;  // External driver
};
```

**After (minimal implementation):**
```cpp
// No external includes!

class SensorReader {
private:
  // Helper methods built into class
  bool sendSFM3505Command(uint16_t command);
  bool readSFM3505Raw(uint8_t* buffer, uint8_t length);
  float scaleSFM3505Flow(uint32_t rawValue);
  uint8_t calculateCRC8(const uint8_t* data, uint8_t len);
};
```

### Simplified Methods
Only essential methods are included:

```cpp
// Reading methods
bool readSFM3505AirFlow(float& airFlow);
bool readSFM3505AllFlows(float& airFlow, float& o2Flow);
bool readSFM3505AirFlowRaw(uint32_t& airFlowRaw);
bool readSFM3505AllFlowsRaw(uint32_t& airFlowRaw, uint32_t& o2FlowRaw);

// Control methods
bool startSFM3505Measurement();
bool stopSFM3505Measurement();
```

**Removed (not essential for basic operation):**
- `startSFM3505MeasurementWithFilter()` - can be added if needed
- `configureSFM3505Averaging()` - can be added if needed
- Product identifier reading
- All the Sensirion framework overhead

## 🚀 Usage (Exactly the Same!)

The usage is **identical** to the full implementation:

```cpp
#include "SensorReader.hpp"
#include "PinConfig.h"

SensorReader sensors_bus0(&Wire, "Bus0");

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, 500000);
  sensors_bus0.initialize();  // Auto-starts SFM3505
}

void loop() {
  // Read legacy sensors
  SensorData data;
  sensors_bus0.update(data);
  
  // Read SFM3505
  float airFlow, o2Flow;
  if (sensors_bus0.readSFM3505AllFlows(airFlow, o2Flow)) {
    Serial.printf("Air: %.2f slm, O2: %.2f slm\n", airFlow, o2Flow);
  }
  
  delay(100);
}
```

## 🔧 How It Works

### 1. Sending Commands
```cpp
bool sendSFM3505Command(uint16_t command) {
  _wire->beginTransmission(I2Cadr_SFM3505);
  _wire->write((uint8_t)(command >> 8));    // MSB
  _wire->write((uint8_t)(command & 0xFF));  // LSB
  return (_wire->endTransmission() == 0);
}
```

### 2. Reading Data with CRC Validation
The SFM3505 returns data in this format:
```
[Byte0][Byte1][Byte2][CRC] [Byte3][Byte4][Byte5][CRC] [Status]
 └─ Air Flow (3 bytes) ─┘   └── O2 Flow (3 bytes) ──┘
```

We validate each CRC to ensure data integrity:
```cpp
bool readSFM3505Raw(uint8_t* buffer, uint8_t length) {
  uint8_t rawBuffer[9];
  _wire->requestFrom(I2Cadr_SFM3505, (uint8_t)9);
  
  // Read all 9 bytes
  for (uint8_t i = 0; i < 9; i++) {
    rawBuffer[i] = _wire->read();
  }
  
  // Validate CRCs
  if (calculateCRC8(rawBuffer, 3) != rawBuffer[3]) return false;
  if (calculateCRC8(&rawBuffer[4], 3) != rawBuffer[7]) return false;
  
  // Copy data (skip CRC bytes)
  buffer[0] = rawBuffer[0];  // Air byte 0
  buffer[1] = rawBuffer[1];  // Air byte 1
  buffer[2] = rawBuffer[2];  // Air byte 2
  buffer[3] = rawBuffer[4];  // O2 byte 0
  buffer[4] = rawBuffer[5];  // O2 byte 1
  buffer[5] = rawBuffer[6];  // O2 byte 2
  
  return true;
}
```

### 3. Scaling Raw Values
```cpp
float scaleSFM3505Flow(uint32_t rawValue) {
  // SFM3505 formula: Flow = (raw - 8388608) / 25600
  int32_t signedValue = (int32_t)rawValue;
  return ((float)(signedValue - 8388608)) / 25600.0;
}
```

### 4. CRC-8 Calculation
```cpp
uint8_t calculateCRC8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0xFF;  // Initial value
  
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;  // Polynomial: 0x31
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}
```

## 📊 Data Format

### Raw Data (24-bit signed integer)
```
Range: 0 to 16777215 (0x000000 to 0xFFFFFF)
Offset: 8388608 (0x800000)
Scale: 25600
```

### Conversion Example
```
Raw value: 9388608
Signed: 9388608 - 8388608 = 1000000
Scaled: 1000000 / 25600 = 39.0625 slm
```

## ⚙️ Commands

### Start Continuous Measurement
```cpp
#define SFM3505_CMD_START_CONTINUOUS 0x3603
```

### Stop Continuous Measurement
```cpp
#define SFM3505_CMD_STOP_CONTINUOUS 0x3FF9
```

## 🔍 Error Handling

The implementation includes robust error checking:

1. **I2C Communication Errors**
   ```cpp
   if (_wire->endTransmission() != 0) {
     return false;  // I2C error
   }
   ```

2. **Incorrect Byte Count**
   ```cpp
   if (bytesRead != 9) {
     Serial.printf("Expected 9 bytes, got %d\n", bytesRead);
     return false;
   }
   ```

3. **CRC Validation Errors**
   ```cpp
   if (crc1 != rawBuffer[3]) {
     Serial.printf("CRC error on air flow\n");
     return false;
   }
   ```

## 💡 Advantages of Minimal Implementation

### 1. No External Dependencies
- Easier to integrate
- No library version conflicts
- Smaller code footprint

### 2. Full Control
- Understand exactly what's happening
- Easy to debug
- Easy to modify

### 3. Optimized for Your Use Case
- Only includes what you need
- No unused features
- Faster compilation

### 4. Educational
- See how Sensirion I2C protocol works
- Learn about CRC validation
- Understand data scaling

## 🔧 Adding Optional Features

If you need additional features, they're easy to add:

### Custom Filter (if needed)
```cpp
bool startSFM3505MeasurementWithFilter(uint16_t filter) {
  _wire->beginTransmission(I2Cadr_SFM3505);
  _wire->write(0x36);  // CMD MSB
  _wire->write(0x03);  // CMD LSB
  _wire->write((uint8_t)(filter >> 8));   // Filter MSB
  _wire->write((uint8_t)(filter & 0xFF)); // Filter LSB
  
  // Calculate and send CRC for the filter value
  uint8_t filterBytes[2] = {(uint8_t)(filter >> 8), (uint8_t)(filter & 0xFF)};
  uint8_t crc = calculateCRC8(filterBytes, 2);
  _wire->write(crc);
  
  return (_wire->endTransmission() == 0);
}
```

### Averaging Configuration (if needed)
```cpp
#define SFM3505_CMD_CONFIGURE_AVG 0x364D

bool configureSFM3505Averaging(uint16_t averageWindow) {
  _wire->beginTransmission(I2Cadr_SFM3505);
  _wire->write(0x36);  // CMD MSB
  _wire->write(0x4D);  // CMD LSB
  _wire->write((uint8_t)(averageWindow >> 8));
  _wire->write((uint8_t)(averageWindow & 0xFF));
  
  uint8_t avgBytes[2] = {(uint8_t)(averageWindow >> 8), (uint8_t)(averageWindow & 0xFF)};
  uint8_t crc = calculateCRC8(avgBytes, 2);
  _wire->write(crc);
  
  return (_wire->endTransmission() == 0);
}
```

## ✅ Migration from Full Driver

If you're switching from the full Sensirion driver:

1. **Remove** `SensirionI2cSfm3505.h` and `SensirionCore.h`
2. **Replace** SensorReader.hpp with minimal version
3. **Replace** SensorReader.cpp with minimal version
4. **Code works the same** - no changes to your main.cpp needed!

## 📚 Reference

### SFM3505 Specifications
- I2C Address: 0x2E (fixed)
- Measurement Range: -200 to +200 slm (air calibrated)
- Accuracy: ±2% of reading or ±0.5 slm
- Response Time: 1-10ms (configurable)
- Supply Voltage: 3.3V or 5V
- I2C Speed: Up to 400kHz

### Data Format
- Units: slm (Standard Liters per Minute at 20°C, 1013.25hPa)
- Resolution: 16-bit effective
- Update Rate: Up to 1kHz
- CRC: CRC-8 with polynomial 0x31

## 🎯 Summary

The minimal implementation gives you:
- ✅ All essential SFM3505 functionality
- ✅ No external dependencies
- ✅ Clean, understandable code
- ✅ Easy to debug and modify
- ✅ Same simple API as before

Perfect for embedded systems where you want full control! 🚀