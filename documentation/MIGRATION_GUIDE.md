# Migration Guide: Dual I2C + Dual Serial for T-Display S3

> ⚠️ **NOTE**: This guide describes a migration that has been completed. For the current system architecture, see [CURRENT_ARCHITECTURE.md](../CURRENT_ARCHITECTURE.md).
>
> This document is kept for historical reference.

## Overview
This guide shows how to migrate your existing P-Mixer code to support:
- ✅ Two independent I2C buses for sensors with same addresses
- ✅ Two Serial ports for external microcontroller communication
- ✅ Proper pin usage for LilyGo T-Display S3

## 📌 Pin Assignments

### I2C Buses
```
I2C Bus 0 (Wire):  GPIO43 (SDA), GPIO44 (SCL)
I2C Bus 1 (Wire1): GPIO10 (SDA), GPIO11 (SCL)
```

### Serial Ports
```
Serial1: GPIO1 (TX), GPIO2 (RX)  - External MCU 1
Serial2: GPIO12 (TX), GPIO13 (RX) - External MCU 2
```

### Still Available
```
GPIO3: Available for other use (ADC capable)
```

### Display (Already configured)
```
Display Power: GPIO15 (MUST set HIGH!)
Buttons: GPIO0, GPIO14
```

## 🔧 Hardware Setup

### I2C Wiring
**Each I2C bus needs pull-up resistors (4.7kΩ typical):**

```
I2C Bus 0:
  ESP32 GPIO43 ──┬── 4.7kΩ to 3.3V
                 ├── SFM Sensor 1 SDA (0x40)
                 ├── SPD Sensor 1 SDA (0x25)
                 └── SSC Sensor 1 SDA (0x58)
  
  ESP32 GPIO44 ──┴── 4.7kΩ to 3.3V (SCL for all)

I2C Bus 1:
  ESP32 GPIO10 ──┬── 4.7kΩ to 3.3V
                 ├── SFM Sensor 2 SDA (0x40)
                 ├── SPD Sensor 2 SDA (0x25)
                 └── SSC Sensor 2 SDA (0x58)
  
  ESP32 GPIO11 ──┴── 4.7kΩ to 3.3V (SCL for all)
```

### Serial Wiring
```
Serial1:
  ESP32 GPIO1 (TX) ──► External MCU 1 RX
  ESP32 GPIO2 (RX) ◄── External MCU 1 TX
  
Serial2:
  ESP32 GPIO12 (TX) ──► External MCU 2 RX
  ESP32 GPIO13 (RX) ◄── External MCU 2 TX
```

**Important:** Connect all GND together!

## 📝 Code Changes

### 1. Add PinConfig.h
Create a new file `PinConfig.h` with all pin definitions (see provided file).

### 2. Update SensorReader.hpp
**OLD:**
```cpp
class SensorReader {
public:
  SensorReader(uint8_t flow_input_pin, uint8_t flow_output_pin);
  bool initialize();
  float calculateFusedFlow(float diffPress, float flow);
  void setFusionConfig(const SensorFusionConfig& config);
```

**NEW:**
```cpp
class SensorReader {
public:
  SensorReader(TwoWire* wire, const char* name = "Sensor");
  bool initialize();
  const char* getName() const { return _name; }
  
private:
  TwoWire* _wire;  // NEW: I2C bus pointer
  const char* _name;  // NEW: For debugging
```

**REMOVED:**
- Sensor fusion config and methods
- Analog flow input/output pins
- `calculateFusedFlow()` method
- `setFusionConfig()` and `getFusionConfig()` methods

### 3. Update SensorData struct
**OLD:**
```cpp
struct SensorData {
  float differential_pressure;
  float flow;
  float supply_pressure;
  float fused_flow;            // REMOVED
  uint16_t flow_ref_analogue;  // REMOVED
};
```

**NEW:**
```cpp
struct SensorData {
  float differential_pressure;  // SPD pressure in mBar
  float flow;                   // Flow in L/min
  float supply_pressure;        // SSC pressure in PSI
};
```
### 4. Update SensorReader.cpp
Replace all `Wire.` with `_wire->`:
```cpp
// OLD:
Wire.beginTransmission(address);
Wire.write(cmd);
Wire.endTransmission();

// NEW:
_wire->beginTransmission(address);
_wire->write(cmd);
_wire->endTransmission();
```

**Remove** `Wire.begin()` from `initialize()` - now done in main().

**Remove** these functions:
- `readFlowSetValue()`
- `calculateFusedFlow()`
- `setFusionConfig()`
- `getFusionConfig()`

### 5. Update main.cpp

**OLD:**
```cpp
SensorReader sensors(Flow_Input_Analogue_pin, Flow_Output_Analogue_pin);

void setup() {
  sensors.initialize();
}

void loop() {
  // Uses fused_flow and analog reference
  float flow_ref = sysConfig.flow_setting_is_analog ? 
                   sensorData.flow_ref_analogue : 
                   sysConfig.digital_flow_reference;
  actuator.execute(flow_ref, sensorData.fused_flow, sysConfig.quiet_mode);
}
```

**NEW:**
```cpp
#include "PinConfig.h"

// Create two sensor instances - NO analog pins needed
SensorReader sensors_bus0(&Wire, "Bus0");
SensorReader sensors_bus1(&Wire1, "Bus1");

SensorData sensorData_bus0;
SensorData sensorData_bus1;

void setup() {
  // CRITICAL: Enable display power!
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  delay(100);
  
  // Initialize I2C buses BEFORE sensors
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, 500000);
  Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 500000);
  
  // Initialize Serial ports
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
  
  // Now initialize sensors
  sensors_bus0.initialize();
  sensors_bus1.initialize();
}

void loop() {
  // Update both sensor buses
  sensors_bus0.update(sensorData_bus0);
  sensors_bus1.update(sensorData_bus1);
  
  // Use direct flow measurement (no fusion)
  actuator.execute(sysConfig.digital_flow_reference, 
                   sensorData_bus0.flow, 
                   sysConfig.quiet_mode);
  
  // Check Serial ports
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    // Process data from MCU 1
  }
  
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    // Process data from MCU 2
  }
}
```

## 🔍 Testing Strategy

### Step 1: Test I2C Bus 0 Only
```cpp
void setup() {
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN);
  sensors_bus0.initialize();
  
  // Scan for devices
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
    }
  }
}
```
Expected: Should find 0x40 (SFM), 0x25 (SPD), 0x58 (SSC)

### Step 2: Test I2C Bus 1 Only
```cpp
void setup() {
  Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);
  sensors_bus1.initialize();
  
  // Same scan on Wire1
}
```
Expected: Should find same addresses (0x40, 0x25, 0x58)

### Step 3: Test Both Buses Together
```cpp
void loop() {
  sensors_bus0.update(sensorData_bus0);
  sensors_bus1.update(sensorData_bus1);
  
  Serial.printf("Bus0: Flow=%.2f | Bus1: Flow=%.2f\n", 
                sensorData_bus0.flow, 
                sensorData_bus1.flow);
}
```
Expected: Different values from each bus

### Step 4: Test Serial Ports
```cpp
void setup() {
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
}

void loop() {
  Serial1.println("Hello Serial1");
  Serial2.println("Hello Serial2");
  delay(1000);
}
```
Expected: See messages on connected devices

## ⚠️ Common Issues

### Issue: No devices found on I2C scan
**Fix:**
- Check pull-up resistors (4.7kΩ on both SDA and SCL)
- Verify wiring (SDA to SDA, SCL to SCL)
- Check sensor power (3.3V)
- Try reducing speed: `Wire.begin(21, 22, 100000);`

### Issue: Both buses show same data
**Fix:**
- Verify sensors are physically connected to different buses
- Check no wires crossed between buses
- Confirm using `sensors_bus0` and `sensors_bus1` separately

### Issue: Display stays black
**Fix:**
```cpp
pinMode(15, OUTPUT);
digitalWrite(15, HIGH);  // MUST do this first!
delay(100);
```

### Issue: Serial communication not working
**Fix:**
- Check TX/RX are not swapped
- Verify baud rate matches on both ends
- Ensure common GND between devices
- Check ESP32 is 3.3V, other device may need level shifter for 5V

### Issue: GPIO43/44 conflict with USB Serial
**Solution:** GPIO43/44 work fine for I2C even with USB CDC enabled. If you disable CDC, they become UART0 TX/RX but can still be used for I2C.

## 📊 Migration Checklist

- [ ] Add `PinConfig.h` to project
- [ ] Update `SensorReader.hpp` - add `TwoWire*` parameter, remove fusion
- [ ] Update `SensorReader.cpp` - replace `Wire.` with `_wire->`
- [ ] Remove sensor fusion methods from `SensorReader.cpp`
- [ ] Remove analog pin references from constructor
- [ ] Remove `Wire.begin()` from `SensorReader::initialize()`
- [ ] Update `main.cpp` includes
- [ ] Create two `SensorReader` instances (bus0 and bus1)
- [ ] Initialize `Wire` and `Wire1` in `setup()` before sensors
- [ ] Add Serial1 and Serial2 initialization
- [ ] Update `loop()` to read from both sensor buses
- [ ] Remove references to `fused_flow` and `flow_ref_analogue`
- [ ] Update actuator.execute() to use direct flow measurement
- [ ] Add Serial port communication handlers
- [ ] Test each I2C bus independently
- [ ] Test both buses together
- [ ] Test Serial communication
- [ ] Verify display still works

## 💡 Tips

1. **Always initialize I2C buses before calling `sensor.initialize()`**
2. **Set GPIO15 HIGH before initializing display**
3. **Use descriptive names** ("Bus0", "Bus1") for debugging
4. **Keep I2C wires short** (<30cm for 400kHz)
5. **Test one thing at a time** - I2C first, then Serial

## 🎯 Next Steps

After migration works:
1. Add error handling for sensor failures
2. Implement sensor redundancy logic
3. Add data validation between buses
4. Create Serial communication protocol
5. Add health monitoring for external MCUs

Good luck! 🚀
