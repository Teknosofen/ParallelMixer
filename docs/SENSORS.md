# ParallelMixer Sensor Integration Guide

**Last Updated**: 2026-02-17

This document covers integration details, data formats, and usage patterns for all sensors in the ParallelMixer system. For wiring and hardware circuits, see [HARDWARE.md](HARDWARE.md).

---

## Table of Contents

1. [SFM3505 Flow Sensor](#sfm3505-flow-sensor)
2. [ABP2 High Pressure Sensor (Async)](#abp2-high-pressure-sensor)
3. [ABPD Low Pressure Sensor](#abpd-low-pressure-sensor)
4. [FDO2 Optical Oxygen Sensor](#fdo2-optical-oxygen-sensor)
5. [Default Values & Error Handling](#default-values--error-handling)

---

## SFM3505 Flow Sensor

**Model**: Sensirion SFM3505  
**I2C Address**: 0x2E  
**Channels**: 2 (Air + O2)  
**Resolution**: 24-bit signed integer  
**Update Rate**: Up to 1 kHz (sampled at control loop rate)  
**Detection**: `hasSFM3505()`

### Data Format (9 Bytes)

The SFM3505 intersperses CRC bytes **in the middle** of the data — not at the end:

```
Byte 0: Air Flow [23:16]  ┐
Byte 1: Air Flow [15:8]   ├─ Covered by CRC1 (byte 2)
Byte 2: CRC1              ┘
Byte 3: Air Flow [7:0]       (standalone)
Byte 4: O2 Flow [23:16]   ── Covered by CRC2 (byte 5)
Byte 5: CRC2
Byte 6: O2 Flow [15:8]    ┐
Byte 7: O2 Flow [7:0]     ├─ Covered by CRC3 (byte 8)
Byte 8: CRC3              ┘
```

### Reading Sequence

Bytes must be read sequentially, skipping CRC bytes in the correct positions:

```cpp
buffer[0] = Wire.read();  // Air [23:16]
buffer[1] = Wire.read();  // Air [15:8]
uint8_t crc1 = Wire.read(); // CRC1 (skip)
buffer[2] = Wire.read();  // Air [7:0]  ← AFTER CRC

buffer[3] = Wire.read();  // O2 [23:16]
uint8_t crc2 = Wire.read(); // CRC2 (skip)
buffer[4] = Wire.read();  // O2 [15:8]  ← AFTER CRC
buffer[5] = Wire.read();  // O2 [7:0]
uint8_t crc3 = Wire.read(); // CRC3 (skip)
```

### Scaling Formula

```cpp
float flow_slm = (float(rawValue) - 8388608.0) / 25600.0;
```

Where:
- `8388608` = 2^23 (offset for signed 24-bit value)
- `25600` = scale factor for SFM3505

### CRC-8 Validation

**Algorithm**: Sensirion CRC-8  
**Polynomial**: 0x31 (x^8 + x^5 + x^4 + 1)  
**Initialization**: 0xFF  
**Bit order**: MSB first

CRC coverage per group:
- **CRC1**: Validates bytes 0-1 (Air high bytes)
- **CRC2**: Validates byte 4 (O2 high byte)
- **CRC3**: Validates bytes 6-7 (O2 low bytes)

```cpp
uint8_t crc1_calculated = calculateCRC8(&rawData[0], 2);
if (crc1_calculated != rawData[2]) { /* CRC error */ }
```

**Compile-time control** in `include/SensorReader.hpp`:
```cpp
#define SFM3505_ENABLE_CRC_CHECK 1  // Set to 0 to disable for debugging
```

### Initialization

The sensor uses continuous measurement mode:
```cpp
// Start command: 0x3603
Wire.beginTransmission(0x2E);
Wire.write(highByte(0x3603));
Wire.write(lowByte(0x3603));
Wire.endTransmission();
```

### Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| No response at 0x2E | Wiring / power | Check connections, pull-ups |
| CRC errors | I2C noise | Shorten cables, add 100 nF caps near sensor, reduce clock |
| Zero flow reads OK but positive flows wrong | Byte order issue | Verify CRC bytes are skipped at correct positions |
| Sensor doesn't initialize | Pull-up timing | See [SFM3505 Pull-Up Circuit](HARDWARE.md#sfm3505-pull-up-control-circuit) |

---

## ABP2 High Pressure Sensor

**Model**: ABP2DSNT150PG2A3XX (Honeywell)  
**I2C Address**: 0x28  
**Type**: Absolute pressure  
**Range**: 0-150 PSI (0-1034 kPa)  
**Resolution**: 24-bit  
**Compile-time enable**: `#define USE_ABP2_PRESSURE_SENSOR`  
**Detection**: `hasABP2()`

### Asynchronous Measurement Protocol

The ABP2 uses a command-based protocol — no blocking delays in the read path:

1. **Send** measurement command: `0xAA 0x00 0x00`
2. **Wait** ≥5 ms (handled by loop timing, not a blocking `delay()`)
3. **Read** 7 bytes: `[Status][Pressure MSB/MID/LSB][Temp MSB/MID/LSB]`

### Status Byte

| Bit | Meaning |
|-----|---------|
| 6 | Power indication (1 = powered) |
| 5 | Busy flag (1 = converting) |
| 2 | Memory integrity (1 = failed) |
| 0 | Math saturation (1 = occurred) |

### Scaling

```cpp
// 24-bit transfer function
float pressure_PSI = (counts - 1677722) * 150.0 / 13421772.0;
float pressure_kPa = pressure_PSI * 6.89476;
```

### Usage Pattern in Main Loop

Alternating command/read at the configured `PressSamplTime` interval:

```cpp
static uint32_t lastPressureTime = 0;
static bool pressureCommandSent = false;

if ((micros() - lastPressureTime) >= sysConfig.PressSamplTime) {
    lastPressureTime = micros();

    if (!pressureCommandSent) {
        sensors_bus0->startABP2Measurement();   // Send 0xAA command
        pressureCommandSent = true;
    } else {
        float pressure;
        uint8_t status;
        if (sensors_bus0->readABP2Pressure(pressure, status)) {
            sensorData_bus0.supply_pressure = pressure;
        }
        pressureCommandSent = false;
    }
}
```

### Timing

```
Time:     0ms    10ms   20ms   30ms
          |      |      |      |
Action:   CMD    READ   CMD    READ
          └─10ms──→
```

- **Effective sample rate**: 50 Hz at default 100 ms interval (alternating)
- **Minimum interval**: ~5 ms (to allow conversion to complete)
- **Configurable**: `sysConfig.PressSamplTime` (microseconds)

### Busy Polling (Higher Rate)

For faster reads, poll the busy flag instead of fixed timing:

```cpp
if (!pressureCommandSent) {
    sensors_bus0->startABP2Measurement();
    pressureCommandSent = true;
} else if (!sensors_bus0->isABP2Busy()) {
    sensors_bus0->readABP2Pressure(pressure, status);
    pressureCommandSent = false;
}
```

---

## ABPD Low Pressure Sensor

**Model**: ABPDLNN100MG2A3 (Honeywell)  
**I2C Address**: 0x28 (⚠️ conflicts with ABP2)  
**Type**: Differential / low pressure  
**Range**: 0-100 mbar (0-10 kPa)  
**Resolution**: 14-bit pressure + 11-bit temperature  
**Compile-time enable**: `#define USE_ABPD_PRESSURE_SENSOR`  
**Detection**: `hasABPD()`

### Data Format (4 Bytes)

```
[0] Status(2 bits) + Pressure[13:8]
[1] Pressure[7:0]
[2] Temperature[10:3]
[3] Temperature[2:0] (upper 3 bits)
```

### Scaling

```cpp
// 14-bit pressure
float pressure_mbar = (counts - 1638) * 100.0 / 13107.0;
float pressure_kPa = pressure_mbar * 0.1;

// 11-bit temperature
float temperature_C = (counts / 2047.0) * 200.0 - 50.0;
```

### Usage

Direct (synchronous) read — no command phase required:

```cpp
float pressure_kpa, temperature_c;
uint8_t status;
if (sensors_bus0->readABPDPressureTemp(pressure_kpa, temperature_c, status)) {
    sensorData_bus0.abpd_pressure = pressure_kpa;
    sensorData_bus0.abpd_temperature = temperature_c;
}
```

---

## FDO2 Optical Oxygen Sensor

**Model**: Pyroscience FDO2  
**Interface**: Serial2 (UART) at 19200 baud  
**Measurement time**: ~200-500 ms per sample  
**Response time (t63)**: <2 seconds  
**Connector**: Molex 560020-0420

### Measurement Data

```cpp
struct MeasurementData {
    float oxygenPartialPressure_hPa;  // O2 partial pressure
    float temperature_C;
    uint32_t status;
    bool valid;

    // Extended data (from #MRAW command)
    float phaseShift_deg;
    float signalIntensity_mV;
    float ambientLight_mV;
    float ambientPressure_mbar;
    float relativeHumidity_percent;
};
```

### Async Usage (Non-Blocking)

The FDO2 is read asynchronously in the control loop:

```cpp
// Start measurement (every FDO2_SAMPLE_TIME_US = 500ms)
if (!fdo2MeasurementPending && (micros() - lastFDO2StartTime) >= FDO2_SAMPLE_TIME_US) {
    lastFDO2StartTime = micros();
    if (fdo2Sensor.startMeasurementAsync(true)) {  // true = include raw data
        fdo2MeasurementPending = true;
    }
}

// Check if response is ready (non-blocking)
if (fdo2MeasurementPending && fdo2Sensor.isResponseReady()) {
    if (fdo2Sensor.getAsyncResult(fdo2Data)) {
        // Data is in fdo2Data
    }
    fdo2MeasurementPending = false;
}
```

### O2 Conversion

```cpp
float percentO2 = fdo2Sensor.convertToPercentO2(
    fdo2Data.oxygenPartialPressure_hPa,
    fdo2Data.ambientPressure_mbar
);
// Formula: %O2 = 100 × pO2[hPa] / P[mbar]
```

### Serial Output Modes

| Quiet Mode | Content |
|------------|---------|
| Q7 | `O2: 203.45 hPa (20.06%) \| Temp: 23.5°C \| Status: 0x00 OK` |
| Q8 | Extended: phase shift, signal intensity, ambient light, pressure, humidity |

### Status Bits

| Bit | Severity | Description |
|-----|----------|-------------|
| 0 | WARNING | Detector amplification reduced |
| 1 | FATAL | O2 signal too low (<20 mV) |
| 2 | FATAL | O2 signal or ambient light too high |
| 3 | FATAL | O2 reference too low (<20 mV) |
| 4 | FATAL | O2 reference too high (>2400 mV) |
| 5 | FATAL | Temperature sensor failure |
| 7 | WARNING | Humidity >90% RH inside housing |
| 9 | ERROR | Pressure sensor failure |
| 10 | ERROR | Humidity sensor failure |

Normal operation: Status = `0x00` or `0x01`.

### Calibration

**Warning**: Each calibration consumes one flash cycle (~20,000 lifetime max).

```cpp
// Calibrate at known O2 partial pressure
float pO2_hPa = 0.209 * ambientPressure_mbar;
fdo2Sensor.calibrateAtPartialPressure(pO2_hPa);
```

### Device Identification

```cpp
FDO2_Sensor::DeviceInfo info;
if (fdo2Sensor.getDeviceInfo(info)) {
    Serial.printf("ID=%d, FW=%d.%02d, Channels=%d\n",
                  info.deviceId,
                  info.firmwareRevision / 100,
                  info.firmwareRevision % 100,
                  info.numChannels);
}
```

### Operating Limits

| Parameter | Range |
|-----------|-------|
| Temperature | -10 to 60°C (optimal 10-40°C) |
| Max pressure | 20 bar absolute, 3 bar differential |
| Humidity | Non-condensing on backside |
| Warm-up time | 3 minutes for full accuracy |

### Troubleshooting

| Symptom | Check |
|---------|-------|
| Not initializing | Pin assignments in PinConfig.h, TX↔RX crossing, power (3.3-5V) |
| Invalid readings | Status bits, warm-up time (3 min), ambient light |
| Communication errors | Baud rate (19200), cable length, shielding |

---

## Default Values & Error Handling

All sensor readings use **-9.9** as the invalid/unavailable indicator:

| Condition | Result |
|-----------|--------|
| Sensor not detected during initialization | -9.9 |
| Communication failure | -9.9 |
| Read timeout | -9.9 |
| Sensor disabled by compile switch | -9.9 |

**Affected fields**: `supply_pressure`, `sfm3505_air_flow`, `sfm3505_o2_flow`, `abpd_pressure`, `abpd_temperature`

This makes it immediately obvious which data is valid vs. invalid across serial output, web interface, and LCD display.
