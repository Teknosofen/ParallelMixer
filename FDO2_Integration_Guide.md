# FDO2 Optical Oxygen Sensor Integration

## Overview
The FDO2 optical oxygen sensor has been integrated into your P-Mixer ventilator control system. The sensor communicates via UART on Serial2 at 19200 baud (default).

## Hardware Connection
The FDO2 sensor connects to your ESP32 via Serial2:
- **TX (ESP32)** → **RX (FDO2, Pin 3)**
- **RX (ESP32)** → **TX (FDO2, Pin 2)**
- **VCC (3.3-5V)** → **VCC (FDO2, Pin 4)**
- **GND** → **GND (FDO2, Pin 1)**

Connector: Molex 560020-0420

## Integration Details

### Files Added
1. **FDO2_Sensor.h** - Sensor class header with all command definitions
2. **FDO2_Sensor.cpp** - Complete implementation of UART protocol
3. **main.cpp** - Modified to include FDO2 sensor reading

### Features Implemented
- Automatic sensor initialization on startup
- Periodic oxygen measurement with extended raw data (#MRAW command)
- Error checking and status monitoring
- Two new output modes (quiet_mode 8 and 9)
- Integration with existing sensor timing system

## Usage

### Viewing FDO2 Data

#### Basic Oxygen Data (quiet_mode = 8)
Shows oxygen partial pressure, percentage, temperature, and status:
```
O2: 203.45 hPa (20.06%) | Temp: 23.5°C | Status: 0x00 OK
```

#### Extended Raw Data (quiet_mode = 9)
Shows all sensor measurements including phase shift, signal intensity, ambient light, pressure, and humidity:
```
O2: 203.45 hPa | T: 23.5°C | Phase: 24.385° | Signal: 325.1 mV | Amb: 5.2 mV | P: 1013.7 mbar | RH: 45.3%
```

### Command Examples

Change output mode to show FDO2 data:
```
Q 8    // Basic O2 data
Q 9    // Extended raw data
```

### Accessing FDO2 Functions

The FDO2 sensor object `fdo2Sensor` and measurement data `fdo2Data` are globally accessible. You can add custom commands to your CommandParser to:

#### Get Device Info
```cpp
FDO2_Sensor::DeviceInfo info;
if (fdo2Sensor.getDeviceInfo(info)) {
    Serial.printf("Device ID: %d, FW: %d.%02d\n", 
                  info.deviceId, 
                  info.firmwareRevision / 100, 
                  info.firmwareRevision % 100);
}
```

#### Manual Measurement
```cpp
FDO2_Sensor::MeasurementData data;
if (fdo2Sensor.measureOxygen(data)) {
    Serial.printf("O2: %.2f hPa\n", data.oxygenPartialPressure_hPa);
}
```

#### Calibration (WARNING: Consumes flash cycles!)
```cpp
// Calibrate at ambient air (20.9% O2 at known pressure)
float ambientPressure_mbar = 1013.25;
float pO2_hPa = 0.209 * ambientPressure_mbar;
fdo2Sensor.calibrateAtPartialPressure(pO2_hPa);
```

#### Check Measurement Validity
```cpp
if (fdo2Sensor.isMeasurementValid(fdo2Data)) {
    // Data is valid
} else {
    Serial.println(fdo2Sensor.getStatusString(fdo2Data.status));
}
```

## Data Structure

```cpp
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
```

## Status Bits

The sensor reports status information that indicates warnings and errors:

- **Bit 0**: WARNING - Detector amplification reduced (measurement still valid)
- **Bit 1**: FATAL - O2 signal intensity too low (<20mV)
- **Bit 2**: FATAL - O2 signal or ambient light too high
- **Bit 3**: FATAL - O2 reference signal too low (<20mV)
- **Bit 4**: FATAL - O2 reference signal too high (>2400mV)
- **Bit 5**: FATAL - Temperature sensor failure
- **Bit 7**: WARNING - Humidity >90%RH inside housing
- **Bit 9**: ERROR - Pressure sensor failure (doesn't affect O2 reading)
- **Bit 10**: ERROR - Humidity sensor failure (doesn't affect O2 reading)

**Normal operation should show Status = 0x00 or 0x01**

## Timing Considerations

- FDO2 measurements are taken at the same rate as GUI/Serial output (`sysConfig.delta_t`)
- Default is 100ms (10 Hz), matching other sensor outputs
- Each measurement takes ~200-500ms to complete
- The sensor has a response time (t63) of <2 seconds

## Conversion to Volume Percent

To convert partial pressure to volume percent O2:

```cpp
float percentO2 = fdo2Sensor.convertToPercentO2(
    fdo2Data.oxygenPartialPressure_hPa,
    fdo2Data.ambientPressure_mbar
);
```

Formula: %O2 = 100 × pO2[hPa] / P[mbar]

## Important Notes

### Calibration Warning
The FDO2 comes factory-calibrated. Only perform calibration if necessary, as each calibration command consumes one flash cycle (max ~20,000 cycles lifetime).

### Sensor Placement
- The venting capillary on the backside must be at atmospheric pressure
- Protect the sensor from direct sunlight
- Ensure the sensing membrane does not reach dew point

### Operating Conditions
- Temperature range: -10 to 60°C (optimal 10-40°C)
- Maximum pressure: 20 bar absolute, 3 bar differential
- Non-condensing humidity on backside

## Advanced Features

### User Memory
The sensor has 64 x 32-bit signed integers of non-volatile storage:

```cpp
int32_t values[10];
// Write
fdo2Sensor.writeUserMemory(0, 10, values);
// Read
fdo2Sensor.readUserMemory(0, 10, values);
```

### Broadcast Mode
For continuous autonomous measurements (not recommended in this application):

```cpp
fdo2Sensor.enableBroadcast(1000);  // Broadcast every 1000ms
// ... handle incoming data asynchronously
fdo2Sensor.disableBroadcast();
```

### Baud Rate Change
```cpp
fdo2Sensor.setBaudRate(115200);  // Change to 115200 baud
```

## Troubleshooting

### Sensor Not Initializing
1. Check Serial2 pin assignments in PinConfig.h
2. Verify physical connections (TX ↔ RX crossed)
3. Check power supply (3.3-5V)
4. Monitor startup messages for FDO2 initialization status

### Invalid Readings
1. Check status bits - fatal errors indicate hardware issues
2. Ensure proper warm-up time (3 minutes for full accuracy)
3. Verify ambient light is not excessive
4. Check that sensor is not at dew point temperature

### Communication Errors
1. Verify baud rate (default 19200)
2. Check for electrical noise on serial lines
3. Ensure stable power supply during measurements
4. Monitor error codes with `fdo2Sensor.getLastError()`

## Error Codes

- **-1**: General error
- **-21**: UART parse error
- **-22**: UART RX error (timeout or incomplete data)
- **-23**: UART header error
- **-26**: UART request error (unknown command)
- **-60**: Invalid calibration (out of range)

## Performance

- **Measuring Range**: 0-1000 hPa (0-100% O2) typical, up to 2000 hPa max
- **Accuracy**: ±0.2 hPa at 10 hPa, ±5 hPa at 200 hPa @ 10-40°C
- **Resolution**: ±0.1 hPa at 10 hPa, ±1 hPa at 200 hPa
- **Drift**: <1% O2/year (at 21% O2, 25°C, protected from sunlight)
- **Response Time**: t63 < 2 seconds
- **Lifetime**: >5 years typical, >500 million measurements

## Chemical Compatibility

Compatible with most gases including CH4, CO, CO2, H2S, NO, N2O, and moisture.
**Not compatible** with Cl2, NO2, or high concentrations of volatile organic compounds.
See datasheet section 3 for full compatibility information.
