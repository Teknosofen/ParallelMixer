# ParallelMixer Current Architecture

**Last Updated**: 2026-01-15

## Overview

ParallelMixer is an ESP32-S3 based ventilator/mixer control system with real-time sensor monitoring, PID control, web interface, and LCD display. This document describes the current architecture as implemented.

## Hardware Platform

- **MCU**: ESP32-S3 (LilyGo T-Display S3)
- **Display**: 170x320 TFT LCD (ST7789)
- **I2C Bus**: GPIO43 (SDA) / GPIO44 (SCL) @ 500kHz
- **Serial Actuator**: Serial1 @ 115200 baud
- **Control Output**: Analog valve control (PWM/DAC)
- **User Input**: Single button (GPIO14) with long/short press detection

## Active Sensors

### Flow Sensor
- **Model**: SFM3505 (Sensirion)
- **I2C Address**: 0x2E
- **Measurements**: Air flow + O2 flow (slm)
- **Update Rate**: Up to 1kHz (currently 100Hz)
- **Detection**: `hasSFM3505()`

### Pressure Sensors (Compile-Time Selection)

**Option 1: High Pressure (Default)**
- **Model**: ABP2DSNT150PG2A3XX (Honeywell)
- **I2C Address**: 0x28
- **Range**: 0-150 PSI (0-10.3 bar)
- **Measurement**: Absolute pressure
- **Read Method**: Asynchronous (command → wait → read)
- **Enable**: `#define USE_ABP2_PRESSURE_SENSOR`
- **Detection**: `hasABP2()`

**Option 2: Low Pressure**
- **Model**: ABPDLNN100MG2A3 (Honeywell)
- **I2C Address**: 0x28 ⚠️ Same as ABP2!
- **Range**: 0-100 mbar (0-1.45 PSI)
- **Measurements**: Differential pressure + temperature
- **Read Method**: Direct read
- **Enable**: `#define USE_ABPD_PRESSURE_SENSOR`
- **Detection**: `hasABPD()`

**Note**: Both pressure sensors use the same I2C address. Only ONE can be used at a time until an I2C multiplexer is added. See [PRESSURE_SENSOR_CONFIG.md](PRESSURE_SENSOR_CONFIG.md).

### Serial Actuator
- **Interface**: Serial1 (UART)
- **Protocol**: `<CHR>FLOAT\n` (e.g., "I1.25\n")
- **Measurements**: Current ('I' command) + misc data
- **Update**: Asynchronous non-blocking reception
- **Default Value**: -9.9 when no data received

## Default Values System

All sensor and actuator readings use **-9.9** as the invalid/unavailable indicator:
- Sensor not detected during initialization
- Communication failure
- Read timeout
- Sensor disabled by compile switch

This makes it immediately obvious which data is valid vs. invalid in serial output, web interface, and LCD display.

**Affected Values**:
- `supply_pressure` (ABP2)
- `sfm3505_air_flow` (SFM3505)
- `sfm3505_o2_flow` (SFM3505)
- `abpd_pressure` (ABPD)
- `abpd_temperature` (ABPD)
- `valveActuatorCurrent` (Serial1)
- `valveActuatorMisc` (Serial1)

## Control System

### Dual-Rate Loop Architecture

**Fast Control Loop** (X command - `control_interval`)
- **Default Rate**: 100Hz (10ms)
- **Configurable Range**: 1-1000Hz
- **Operations**:
  - Read all sensors (SFM3505, ABP2/ABPD)
  - Execute PID/valve control
  - Buffer data for web interface
  - Update serial actuator
- **Command**: `X10000` = 10ms = 100Hz

**Slow Output Loop** (T command - `delta_t`)
- **Default Rate**: 10Hz (100ms)
- **Configurable Range**: 1-100Hz
- **Operations**:
  - Serial output (printf)
  - LCD display updates
  - Mode display
- **Command**: `T100000` = 100ms = 10Hz

### Control Modes
1. **PID Control**: Closed-loop flow control with configurable PID gains
2. **Valve Set Value**: Open-loop valve position control (0-100%)

## WiFi System

### Default State
- **WiFi starts DISABLED** to save power and reduce EMI
- No automatic startup
- Must be manually enabled via button or code

### Button Control
- **Long Press (>1s)**: Enable WiFi AP
  - SSID configurable (default: "P-Mixer")
  - IP: 192.168.4.1
  - LCD shows: "Short press: disable"
- **Short Press**: Disable WiFi AP
  - LCD shows: "Long press: enable"

### Web Interface

**Data Collection**:
- **Rate**: Matches control_interval (e.g., 100Hz)
- **Method**: Buffered collection with batch transfer
- **Endpoint**: `/dataBuffer` - returns array of samples
- **Fetch Interval**: 200ms (client-side)
- **Samples per Fetch**: ~20 at 100Hz collection rate

**Display**:
- 6 real-time data series (lines only, no data points)
  - Flow (SFM3505 air)
  - Pressure (ABP2 or ABPD)
  - Low Pressure (ABPD if enabled)
  - Temperature (ABPD if enabled)
  - Valve Signal (0-100%)
  - Current (Serial actuator)
- Live value cards
- Mode indicator
- Chart displays last 512 points

**Features**:
- CSV export with all historical data
- Clear graph button
- Automatic data trimming
- No SD card required (RAM only)

**Chart Configuration**:
- Lines only (pointRadius: 0)
- Line width: 2px
- Smooth curves (tension: 0.3)
- No fill under lines
- Update without animation for performance

## Serial Commands

### Control Commands
- `X<microseconds>` - Set control interval (e.g., X10000 = 10ms)
- `T<microseconds>` - Set output interval (e.g., T100000 = 100ms)
- `V<percentage>` - Set valve position 0-100% (set value mode)
- `Q<mode>` - Set quiet mode (0=verbose, 1=quiet, 2=debug, 3=special)
- `M` - Switch to PID control mode
- `S` - Switch to set value control mode

### PID Tuning Commands
- `P<value>` - Set proportional gain
- `I<value>` - Set integral gain
- `D<value>` - Set derivative gain
- `R<value>` - Set flow reference (setpoint)

### System Commands
- `?` - Print help
- `C` - Print current configuration

## File Structure

### Core Classes

**SensorReader** (`SensorReader.hpp/cpp`)
- Manages all I2C sensors
- Independent sensor detection with flags
- Minimal implementation (no external drivers)
- Built-in CRC validation for SFM3505
- Methods: `initialize()`, `readSFM3505AllFlows()`, `readABP2Pressure()`, `readABPDPressureTemp()`
- Detection: `hasSFM3505()`, `hasABP2()`, `hasABPD()`

**ActuatorControl** (`ActuatorControl.hpp/cpp`)
- PID controller implementation
- Valve position control (0-100%)
- Mode switching (PID vs Set Value)
- Hardware abstraction for valve output
- Integral anti-windup
- Configurable gains and setpoint

**SerialActuatorReader** (`SerialActuatorReader.hpp/cpp`)
- Non-blocking serial reception
- Protocol: `<CHR>FLOAT\n`
- Stores current ('I') and misc data
- Timestamp tracking for staleness detection
- Blocking and non-blocking read methods
- Default value: -9.9 when no data

**PMixerWiFiServer** (`PMixerWiFiServer.hpp/cpp`)
- WiFi AP management
- High-speed data buffering (vectors)
- Multiple endpoints: `/`, `/data`, `/dataBuffer`, `/history`
- Chart.js-based visualization
- CSV export functionality
- Mode: lines only, no data points

**ImageRenderer** (`ImageRenderer.hpp/cpp`)
- TFT LCD drawing interface
- Status display (flow, pressure, temp, valve, current)
- WiFi status display
- Mode indicator
- Sensor value formatting

**CommandParser** (`CommandParser.hpp/cpp`)
- Serial command parsing
- Configuration management
- Help system

**Button** (`Button.hpp/cpp`)
- Interrupt-based button handling
- Long press detection (>1s)
- Debouncing (50ms)
- Non-polling operation

### Data Structures

**SensorData**
```cpp
struct SensorData {
  float differential_pressure = -9.9;  // Legacy (disabled)
  float flow = -9.9;                   // Legacy (disabled)
  float supply_pressure = -9.9;        // ABP2 pressure
  float sfm3505_air_flow = -9.9;       // SFM3505 air flow
  float sfm3505_o2_flow = -9.9;        // SFM3505 O2 flow
  float abpd_pressure = -9.9;          // ABPD pressure
  float abpd_temperature = -9.9;       // ABPD temperature
};
```

**SystemConfig**
```cpp
struct SystemConfig {
  uint32_t control_interval;      // Fast control loop period (µs)
  uint32_t delta_t;               // Slow output loop period (µs)
  uint32_t PressSamplTime;        // ABP2 sampling time (µs)
  uint8_t quiet_mode;             // Output verbosity
  float digital_flow_reference;   // Flow setpoint
};
```

## Timing Behavior

### Typical Configuration (100Hz control, 10Hz output)
```
Control Loop (10ms):
├─ Read SFM3505 (air + O2 flow)
├─ Read ABP2/ABPD (pressure + temp)
├─ Execute PID/valve control
├─ Buffer data for web
└─ Update serial actuator

Output Loop (100ms):
├─ Serial printf output
├─ LCD display update
└─ Mode update
```

### Web Data Flow
```
ESP32 Side (100Hz):
├─ addDataPoint() called in control loop
├─ Data buffered in vectors
└─ ~20 samples accumulated per 200ms

Client Side (5Hz):
├─ Fetch /dataBuffer every 200ms
├─ Receive ~20 samples as arrays
├─ Process all samples in one batch
├─ Update chart once (no animation)
└─ Buffer cleared after fetch
```

## Future Enhancements

### With I2C Multiplexer
When an I2C multiplexer (e.g., TCA9548A) is added:
- Use both ABP2 and ABPD simultaneously
- Connect sensors to different mux channels
- Modify SensorReader to switch channels
- Remove compile-time pressure sensor selection
- Enable both `USE_ABP2_PRESSURE_SENSOR` and `USE_ABPD_PRESSURE_SENSOR`

### Potential Higher Data Rates
The buffered architecture supports even higher rates:
- Current: 100Hz collection, 200ms fetch = ~20 samples/fetch
- 200Hz: `X5000` = ~40 samples/fetch
- 500Hz: `X2000` = ~100 samples/fetch
- Adjust web fetch interval if needed (currently 200ms)

## Legacy Features (Disabled)

The following sensors and features are present in code but disabled:
- **Legacy SFM** (0x40) - Original flow sensor
- **SPD** (0x25) - Differential pressure sensor
- **SSC** (0x58) - Supply pressure sensor
- **Sensor Fusion** - Multi-sensor combination algorithms
- **Analog Flow I/O** - Analog flow reference pins

These remain in code for historical reference and potential future use.

## Key Design Decisions

1. **Default -9.9 Values**: Clear indication of invalid data across all outputs
2. **Sensor Detection Flags**: Check availability before reading to prevent I2C errors
3. **Dual-Rate Loops**: Separate fast control from slow output for optimal performance
4. **WiFi Disabled by Default**: Power savings and EMI reduction
5. **Buffered Web Transfer**: High data rates with minimal WiFi overhead
6. **Lines-Only Chart**: Clearer visualization at high sample rates
7. **Non-Blocking Serial**: Async actuator data reception without delays
8. **Compile-Time Sensor Selection**: Workaround for I2C address conflict

## Performance Characteristics

- **Sensor Read Time**: ~2-5ms (I2C @ 500kHz)
- **Control Calculation**: <1ms (PID computation)
- **WiFi Overhead**: Minimal (batch transfers)
- **Memory Usage**: ~10KB for 512-point web buffer
- **CPU Usage**: ~10-20% at 100Hz control rate
- **Typical Loop Time**: 5-8ms (leaves margin for 10ms control interval)

## References

- [PRESSURE_SENSOR_CONFIG.md](PRESSURE_SENSOR_CONFIG.md) - Pressure sensor selection guide
- [documentation/README_WIFI.md](documentation/README_WIFI.md) - WiFi system details
- [documentation/MinimalExample.md](documentation/MinimalExample.md) - Basic usage examples
- [SFM3505_CRC_IMPLEMENTATION.md](SFM3505_CRC_IMPLEMENTATION.md) - Flow sensor CRC details
- [ABP2_ASYNC_USAGE.md](ABP2_ASYNC_USAGE.md) - Pressure sensor async pattern
- [SERIAL_REFACTORING.md](SERIAL_REFACTORING.md) - Serial actuator architecture

---

**Version**: 1.0
**Date**: 2026-01-15
**Hardware**: ESP32-S3 T-Display, SFM3505, ABP2/ABPD, Serial Actuator
**Firmware**: ParallelMixer v2.x (post-refactoring)
