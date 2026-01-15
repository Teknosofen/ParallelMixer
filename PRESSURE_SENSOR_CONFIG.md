# Pressure Sensor Configuration Guide

## I2C Address Conflict

Both pressure sensors use the same I2C address (0x28), which creates a conflict:
- **ABP2DSNT150PG2A3XX** (High pressure sensor) → Address: 0x28
- **ABPDLNN100MG2A3** (Low pressure sensor) → Address: 0x28

Until an I2C multiplexer is added, only ONE sensor can be used at a time.

## Compile-Time Switch Configuration

### Location
Open the file: `include/SensorReader.hpp`

### Configuration (Lines 12-13)

```cpp
// Uncomment ONE of the following lines to select which pressure sensor to use:
#define USE_ABP2_PRESSURE_SENSOR    // High pressure sensor (ABP2DSNT150PG2A3XX)
// #define USE_ABPD_PRESSURE_SENSOR    // Low pressure sensor (ABPDLNN100MG2A3)
```

### Option 1: Use ABP2 High Pressure Sensor (Default)

**Configuration:**
```cpp
#define USE_ABP2_PRESSURE_SENSOR    // High pressure sensor (ABP2DSNT150PG2A3XX)
// #define USE_ABPD_PRESSURE_SENSOR    // Low pressure sensor (ABPDLNN100MG2A3)
```

**Result:**
- ABP2 sensor will be detected and read
- ABPD sensor will be disabled
- Serial output: `✅ ABP2 detected (asynchronous measurement)`
- Data appears in `sensorData_bus0.supply_pressure`

### Option 2: Use ABPD Low Pressure Sensor

**Configuration:**
```cpp
// #define USE_ABP2_PRESSURE_SENSOR    // High pressure sensor (ABP2DSNT150PG2A3XX)
#define USE_ABPD_PRESSURE_SENSOR    // Low pressure sensor (ABPDLNN100MG2A3)
```

**Result:**
- ABPD sensor will be detected and read
- ABP2 sensor will be disabled
- Serial output: `✅ ABPD detected`
- Data appears in `sensorData_bus0.abpd_pressure` and `sensorData_bus0.abpd_temperature`

## Important Notes

1. **Only uncomment ONE definition** at a time
2. **You must recompile and upload** after changing the switch
3. If neither is defined, compilation will fail with an error message
4. All other functionality remains intact:
   - SFM3505 flow sensor continues to work
   - Web interface buffering works normally
   - High-speed data collection is unaffected

## Future: Using Both Sensors with I2C Multiplexer

When you add an I2C multiplexer (e.g., TCA9548A), you can:
1. Connect both sensors to different mux channels
2. Modify the detection code in `SensorReader.cpp` to switch between mux channels
3. Enable both `USE_ABP2_PRESSURE_SENSOR` and `USE_ABPD_PRESSURE_SENSOR`
4. Read both sensors independently

## Sensor Specifications

### ABP2DSNT150PG2A3XX (High Pressure)
- **Type:** Absolute pressure
- **Range:** 0-150 PSI (0-10.3 bar)
- **I2C Address:** 0x28
- **Read Method:** Asynchronous (send command, wait, read result)
- **Output:** Pressure value only

### ABPDLNN100MG2A3 (Low Pressure)
- **Type:** Differential/low pressure
- **Range:** 0-100 mbar (0-1.45 PSI)
- **I2C Address:** 0x28
- **Read Method:** Direct read
- **Output:** Pressure and temperature

## Troubleshooting

**Problem:** Compilation error about pressure sensor definition
**Solution:** Make sure exactly ONE of the two `#define` statements is uncommented

**Problem:** Sensor not detected during startup
**Solution:**
1. Verify physical I2C connections
2. Check that the correct sensor is connected for your compile switch setting
3. Review serial output for I2C scan results

**Problem:** Wrong data on web interface
**Solution:** Verify you've selected the correct sensor in the compile switch and recompiled

## Code References

- **Switch definition:** `include/SensorReader.hpp` lines 8-23
- **Detection logic:** `src/SensorReader.cpp` lines 44-77
- **Main loop usage:** `src/main.cpp` lines 358-421
