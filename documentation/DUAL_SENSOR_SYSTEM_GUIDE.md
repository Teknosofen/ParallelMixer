# Complete Sensor System - Legacy + SFM3505

> ⚠️ **NOTE**: This guide describes a dual-sensor system that is no longer actively used. Legacy sensors are now disabled in the current implementation. For the current system architecture, see [CURRENT_ARCHITECTURE.md](../CURRENT_ARCHITECTURE.md).
>
> This document is kept for historical reference and for users who may want to re-enable legacy sensors.

## Overview
The SensorReader class now supports **BOTH** the original legacy sensors **AND** the new SFM3505 sensor simultaneously. You can use them together or separately as needed.

## 📊 Supported Sensors

### Legacy Sensors (Original)
| Sensor | I2C Address | Measurement | Units | Method |
|--------|-------------|-------------|-------|--------|
| SFM | 0x40 | Flow | L/min | `update()` |
| SPD | 0x25 | Differential Pressure | mBar | `update()` |
| SSC | 0x58 | Supply Pressure | PSI | `update()` |

### New Sensor
| Sensor | I2C Address | Measurement | Units | Method |
|--------|-------------|-------------|-------|--------|
| SFM3505 | 0x2E | Air Flow | slm | `readSFM3505xxx()` |
| SFM3505 | 0x2E | O2 Flow | slm | `readSFM3505xxx()` |

## 🎯 SensorData Structure

```cpp
struct SensorData {
  // Legacy sensors (read with update())
  float differential_pressure;  // SPD (0x25) - mBar
  float flow;                   // SFM (0x40) - L/min
  float supply_pressure;        // SSC (0x58) - PSI
  
  // SFM3505 (read with readSFM3505xxx())
  float sfm3505_air_flow;       // SFM3505 (0x2E) - slm
  float sfm3505_o2_flow;        // SFM3505 (0x2E) - slm
};
```

## 🚀 Usage Patterns

### Pattern 1: Legacy Sensors Only
```cpp
SensorData data;

void loop() {
  sensors_bus0.update(data);  // Reads SFM, SPD, SSC
  
  Serial.printf("Flow: %.2f L/min\n", data.flow);
  Serial.printf("Diff Press: %.2f mBar\n", data.differential_pressure);
  Serial.printf("Supply Press: %.2f PSI\n", data.supply_pressure);
}
```
**Use when:** You only have the original sensors

### Pattern 2: SFM3505 Only
```cpp
void loop() {
  float airFlow, o2Flow;
  sensors_bus0.readSFM3505AllFlows(airFlow, o2Flow);
  
  Serial.printf("Air: %.2f slm, O2: %.2f slm\n", airFlow, o2Flow);
}
```
**Use when:** You only have the SFM3505 sensor

### Pattern 3: Both Sensors (Recommended)
```cpp
SensorData data;

void loop() {
  // Read legacy sensors
  sensors_bus0.update(data);
  
  // Read SFM3505
  float sfm_air, sfm_o2;
  if (sensors_bus0.readSFM3505AllFlows(sfm_air, sfm_o2)) {
    data.sfm3505_air_flow = sfm_air;
    data.sfm3505_o2_flow = sfm_o2;
  }
  
  // Now you have ALL sensor data
  Serial.printf("Legacy Flow: %.2f L/min\n", data.flow);
  Serial.printf("SFM3505 Air: %.2f slm\n", data.sfm3505_air_flow);
  Serial.printf("SFM3505 O2:  %.2f slm\n", data.sfm3505_o2_flow);
}
```
**Use when:** You have both sensor types for redundancy/comparison

### Pattern 4: Maximum Configuration (Dual Bus)
```cpp
// Bus 0: Full sensor set
SensorData data_bus0;
sensors_bus0.update(data_bus0);
sensors_bus0.readSFM3505AllFlows(data_bus0.sfm3505_air_flow, 
                                  data_bus0.sfm3505_o2_flow);

// Bus 1: Full sensor set
SensorData data_bus1;
sensors_bus1.update(data_bus1);
sensors_bus1.readSFM3505AllFlows(data_bus1.sfm3505_air_flow, 
                                  data_bus1.sfm3505_o2_flow);
```
**Use when:** Maximum redundancy needed

## 📋 Method Reference

### Legacy Sensor Methods
```cpp
// Updates all legacy sensors (SFM 0x40, SPD, SSC)
void update(SensorData& data);
```

### SFM3505 Methods
```cpp
// Scaled (float) readings
bool readSFM3505AirFlow(float& airFlow);
bool readSFM3505AllFlows(float& airFlow, float& o2Flow);

// Raw (uint32_t) readings
bool readSFM3505AirFlowRaw(uint32_t& airFlowRaw);
bool readSFM3505AllFlowsRaw(uint32_t& airFlowRaw, uint32_t& o2FlowRaw);

// Control
bool startSFM3505Measurement();
bool startSFM3505MeasurementWithFilter(uint16_t filter);
bool stopSFM3505Measurement();
bool configureSFM3505Averaging(uint16_t averageWindow);
```

## ⚙️ Initialization

Both sensor types are initialized automatically in `initialize()`:

```cpp
void setup() {
  Wire.begin(43, 44);
  sensors_bus0.initialize();  // Initializes BOTH legacy and SFM3505
}
```

**What happens:**
1. SFM3505 is initialized and started (0x2E)
2. Legacy SFM is reset and started (0x40)
3. SPD is initialized (0x25)
4. SSC is ready (0x58)

## 🔧 Hardware Configurations

### Configuration A: Single Bus, Legacy Only
```
I2C Bus 0:
  ├── SFM (0x40)
  ├── SPD (0x25)
  └── SSC (0x58)
```

### Configuration B: Single Bus, SFM3505 Only
```
I2C Bus 0:
  └── SFM3505 (0x2E)
```

### Configuration C: Single Bus, Both Types
```
I2C Bus 0:
  ├── SFM (0x40)
  ├── SPD (0x25)
  ├── SSC (0x58)
  └── SFM3505 (0x2E)
```
**Perfect for:** Sensor comparison, gradual migration, redundancy

### Configuration D: Dual Bus, Full Redundancy
```
I2C Bus 0:                    I2C Bus 1:
  ├── SFM (0x40)               ├── SFM (0x40)
  ├── SPD (0x25)               ├── SPD (0x25)
  ├── SSC (0x58)               ├── SSC (0x58)
  └── SFM3505 (0x2E)           └── SFM3505 (0x2E)
```
**Perfect for:** Critical systems, full redundancy

## 📊 Data Flow Example

```cpp
void controlLoop() {
  SensorData data;
  
  // 1. Read all sensors
  sensors_bus0.update(data);  // Legacy: flow, diff_press, supply_press
  
  float sfm_air, sfm_o2;
  sensors_bus0.readSFM3505AllFlows(sfm_air, sfm_o2);  // SFM3505
  
  // 2. Use the data you need
  
  // Option A: Use legacy flow for control
  actuator.execute(flow_ref, data.flow, quiet_mode);
  
  // Option B: Use SFM3505 for control
  actuator.execute(flow_ref, sfm_air, quiet_mode);
  
  // Option C: Use sensor fusion
  float fusedFlow = (data.flow + sfm_air) / 2.0;
  actuator.execute(flow_ref, fusedFlow, quiet_mode);
  
  // 3. Display/log all data
  Serial.printf("Legacy: %.2f L/min, SFM3505: %.2f slm\n", 
                data.flow, sfm_air);
}
```

## ⚠️ Important Notes

### Units Are Different!
- **Legacy SFM (0x40):** L/min (at ambient conditions)
- **SFM3505 (0x2E):** slm (Standard Liters per Minute at 20°C, 1013.25hPa)

To compare directly, you may need temperature/pressure corrections.

### Error Handling
```cpp
// Legacy sensors - returns void
sensors.update(data);  // Always succeeds, may give 0.0 on error

// SFM3505 - returns bool
if (!sensors.readSFM3505AirFlow(airFlow)) {
  // I2C communication failed
  Serial.println("SFM3505 read error!");
}
```

### Sensor Availability
If a sensor is not present on the I2C bus:
- **Legacy sensors:** Will show communication errors but won't crash
- **SFM3505:** Returns `false`, shows warning in initialize()

You can safely call all methods even if some sensors are missing.

## 🎯 Migration Path

### Step 1: Current System (Legacy Only)
```cpp
sensors.update(data);
// Use data.flow, data.differential_pressure, data.supply_pressure
```

### Step 2: Add SFM3505 (Both Running)
```cpp
sensors.update(data);  // Keep existing code
sensors.readSFM3505AllFlows(air, o2);  // Add new sensor
// Compare and validate
```

### Step 3: Transition to SFM3505
```cpp
// Still read legacy for backup
sensors.update(data);

// Use SFM3505 as primary
if (sensors.readSFM3505AirFlow(airFlow)) {
  // Use SFM3505
} else {
  // Fallback to legacy
  airFlow = data.flow;
}
```

### Step 4: SFM3505 Only (Optional)
```cpp
sensors.readSFM3505AllFlows(air, o2);
// Legacy sensors can stay connected but unused
```

## 💡 Best Practices

1. **Always check SFM3505 return values** (returns bool)
2. **Use appropriate units** (L/min vs slm)
3. **Keep legacy sensors** for redundancy during transition
4. **Log both sensors** to validate new sensor performance
5. **Use sensor fusion** for critical applications

## 🔗 Quick Reference

**Read legacy sensors:**
```cpp
sensors.update(data);
```

**Read SFM3505:**
```cpp
sensors.readSFM3505AllFlows(air, o2);
```

**Read everything:**
```cpp
sensors.update(data);
sensors.readSFM3505AllFlows(data.sfm3505_air_flow, data.sfm3505_o2_flow);
```

That's it! Both sensor systems work perfectly together. 🎉
