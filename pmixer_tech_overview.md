# P-Mixer Ventilator Control System - Technical Overview

**Last Updated**: 2026-02-03  
**Version**: Based on current codebase  
**Platform**: ESP32-S3 (LilyGo T-Display S3)

---

## Table of Contents
1. [System Overview](#system-overview)
2. [Hardware Configuration](#hardware-configuration)
3. [Core Architecture](#core-architecture)
4. [Sensor Management](#sensor-management)
5. [Control Systems](#control-systems)
6. [Communication Protocols](#communication-protocols)
7. [Web Interface](#web-interface)
8. [User Interface](#user-interface)
9. [Data Structures](#data-structures)
10. [Timing & Performance](#timing--performance)
11. [Configuration](#configuration)
12. [Safety Features](#safety-features)
13. [Extension Points](#extension-points)

---

## System Overview

P-Mixer is an embedded ventilator control system providing:
- **Dual I2C bus** sensor management (parallel measurement capability)
- **Multi-channel actuator control** via serial multiplexing (6 independent channels)
- **High-level ventilator control** with complete breath cycle management
- **Real-time web monitoring** with high-speed data streaming
- **TFT display** for local status monitoring
- **Optical O2 sensing** with asynchronous measurement

### Key Capabilities
- PID flow control with signal generation modes
- Volume/pressure-controlled mechanical ventilation
- Gas mixing (Air/O2) with FiO2 control
- Patient triggering (flow/pressure)
- Real-time data logging at 100+ Hz
- WiFi access point with web dashboard

---

## Hardware Configuration

### Microcontroller
- **Platform**: ESP32-S3 (LilyGo T-Display S3)
- **Display**: 170x320 TFT LCD (ST7789 driver, portrait mode)
- **Display Power**: GPIO15 (must be HIGH)
- **Backlight**: GPIO38
- **User Inputs**: 
  - Interaction key: GPIO0 (WiFi control)
  - Boot button: GPIO21 (settings display)

### Communication Buses

#### I2C Bus 0 (Wire)
- **Pins**: GPIO43 (SDA), GPIO44 (SCL)
- **Clock**: 400 kHz (configurable, defined as I2C0_CLOCK_FREQ)
- **Sensors**:
  - SFM3505 flow sensor (0x2E) - Air + O2 channels
  - ABP2 high pressure sensor (0x28) *OR*
  - ABPD low pressure sensor (0x18 or 0x28)
  - MCP4725 DAC (0x60) - optional

#### I2C Bus 1 (Wire1)
- **Pins**: GPIO10 (SDA), GPIO11 (SCL)  
- **Clock**: 400 kHz (same as Bus 0)
- **Purpose**: Parallel sensor measurement (Bus 0 + Bus 1 = dual-channel capability)
- **Sensors**: Same device types as Bus 0

**Note**: Both ABP2 and ABPD use address 0x28. Only ONE can be active per bus without an I2C multiplexer. Use compile-time switches to select which pressure sensor is enabled.

#### Serial Buses

| Bus | Purpose | Pins | Baud | Protocol |
|-----|---------|------|------|----------|
| Serial (USB CDC) | Host communication | USB | Auto | ASCII commands |
| Serial1 | MUX router (actuators) | TX=17, RX=18 | 230400 | `[ADDR]<CMD>VALUE\n` |
| Serial2 | FDO2 oxygen sensor | TX=12, RX=13 | 19200 | ASCII CR-terminated |

---

## Core Architecture

### Component Hierarchy

```
main.cpp (Main Loop)
├── SensorReader (Bus 0) ──────────► I2C Wire
├── SensorReader (Bus 1) ──────────► I2C Wire1
├── ActuatorControl[6] ────────────► SerialMuxRouter ──► Serial1
├── VentilatorController ──────────► SerialMuxRouter
├── FDO2_Sensor ───────────────────► Serial2
├── CommandParser ─────────────────► Serial (USB)
├── PMixerWiFiServer ──────────────► WiFi AP
├── ImageRenderer ─────────────────► TFT Display
└── Button[2] ─────────────────────► GPIO interrupts
```

### Timing Architecture

```
Main Loop (~200 Hz)
│
├── Fast Control Loop (X command, default 100 Hz / 10ms)
│   ├── Read SFM3505 flow sensors (Bus 0 & 1)
│   ├── Read ABPD low pressure + temperature (Bus 0 & 1)
│   ├── Read ABP2 pressure async (Bus 0 & 1)
│   ├── FDO2 async check (500ms sample period = 2 Hz)
│   ├── Ventilator state machine update (if running)
│   ├── Execute actuator control (all 6 channels)
│   │   ├── PID control
│   │   ├── Signal generation (sine/step/triangle/sweep)
│   │   └── Manual valve control
│   ├── Serial MUX router update (non-blocking)
│   └── WiFi buffer update (high-speed data collection)
│
├── Slow Output Loop (T command, default 10 Hz / 100ms)
│   ├── Serial output (printf, mode-dependent)
│   ├── WiFi server status update
│   └── Display update trigger
│
├── Display Update Loop (50ms = 20 Hz)
│   ├── Update live data fields
│   └── Update O2 display (if enabled)
│
└── Async Pressure Loop (100ms = 10 Hz, independent)
    ├── Send ABP2 measurement command (0xAA 0x00 0x00)
    ├── Wait ~5ms (implicit via loop timing)
    └── Read ABP2 result (7 bytes: status + pressure + temp)
```

**Key Principle**: No blocking delays in main loop. All long-duration operations are asynchronous or time-sliced.

---

## Sensor Management

### SensorReader Class

**Purpose**: Hardware abstraction layer for I2C sensors with independent detection

**Architecture**:
- Two instances: `sensors_bus0` (Wire) and `sensors_bus1` (Wire1)
- Each instance independently detects and manages its sensors
- Detection flags: `_hasSFM3505`, `_hasABP2`, `_hasABPD`
- All reads check detection flags before accessing hardware

### Supported Sensors

#### 1. SFM3505 Flow Sensor (Sensirion)

**Specifications**:
- **I2C Address**: 0x2E
- **Channels**: 2 (Air + O2)
- **Resolution**: 24-bit (signed integer)
- **Range**: Configurable (typically ±200 slm)
- **Update Rate**: Up to 1 kHz (sampled at control loop rate)

**Data Format**:
```
9 bytes per read:
[0] Air[23:16]  [1] Air[15:8]  [2] CRC1
[3] Air[7:0]    [4] O2[23:16]  [5] CRC2
[6] O2[15:8]    [7] O2[7:0]    [8] CRC3
```

**Scaling**:
```cpp
Flow_slm = (RawValue_signed - 8388608) / 25600.0
```

**Features**:
- Continuous measurement mode
- Optional CRC-8 validation (configurable via `SFM3505_ENABLE_CRC_CHECK`)
- Non-blocking reads
- Dual-channel simultaneous measurement

**Detection**: `hasSFM3505()`

#### 2. ABP2DSNT150PG2A3XX High Pressure Sensor (Honeywell)

**Specifications**:
- **I2C Address**: 0x28
- **Type**: Absolute pressure
- **Range**: 0-150 PSI (0-1034 kPa)
- **Resolution**: 24-bit
- **Output**: Pressure + Temperature + Status

**Async Protocol**:
1. Send command: `0xAA 0x00 0x00`
2. Wait ≥5ms (handled by loop timing)
3. Read 7 bytes: [Status][Pressure MSB/MID/LSB][Temp MSB/MID/LSB]

**Status Byte** (bit flags):
- Bit 6: Power indication (1=powered)
- Bit 5: Busy flag (1=device busy)
- Bit 2: Memory integrity (1=failed)
- Bit 0: Math saturation (1=occurred)

**Scaling**:
```cpp
// 24-bit transfer function
Pressure_PSI = (counts - 1677722) × 150.0 / 13421772.0
Pressure_kPa = Pressure_PSI × 6.89476
```

**Usage Pattern**:
```cpp
// In main loop (every 100ms):
static uint32_t lastPressureTime = 0;
static bool commandSent = false;

if ((micros() - lastPressureTime) >= 100000) {
    lastPressureTime = micros();
    
    if (!commandSent) {
        sensors_bus0->startABP2Measurement();  // Send command
        commandSent = true;
    } else {
        float pressure_kpa;
        uint8_t status;
        if (sensors_bus0->readABP2Pressure(pressure_kpa, status)) {
            sensorData_bus0.supply_pressure = pressure_kpa;
        }
        commandSent = false;  // Ready for next cycle
    }
}
```

**Compile-Time Enable**: `#define USE_ABP2_PRESSURE_SENSOR`  
**Detection**: `hasABP2()`

#### 3. ABPDLNN100MG2A3 Low Pressure Sensor (Honeywell)

**Specifications**:
- **I2C Address**: 0x18 or 0x28 (⚠️ conflicts with ABP2)
- **Type**: Differential/Low pressure
- **Range**: 0-100 mbar (0-10 kPa)
- **Resolution**: 14-bit pressure + 11-bit temperature
- **Update Rate**: Continuous (read on demand)

**Data Format**:
```
4 bytes per read:
[0] Status(2) + Pressure[13:8]
[1] Pressure[7:0]
[2] Temperature[10:3]
[3] Temperature[2:0] (upper 3 bits)
```

**Scaling**:
```cpp
// 14-bit pressure
Pressure_mbar = (counts - 1638) × 100.0 / 13107.0
Pressure_kPa = Pressure_mbar × 0.1

// 11-bit temperature
Temperature_C = (counts / 2047.0) × 200.0 - 50.0
```

**Compile-Time Enable**: `#define USE_ABPD_PRESSURE_SENSOR`  
**Detection**: `hasABPD()`

**⚠️ Important**: Only ONE pressure sensor type (ABP2 or ABPD) can be enabled per bus due to address conflict. Use `#define` to select which is active.

#### 4. FDO2 Optical Oxygen Sensor (PreSens)

**Specifications**:
- **Interface**: Serial2 (UART)
- **Baud Rate**: 19200
- **Protocol**: ASCII commands, CR-terminated
- **Measurement Time**: ~500ms per sample

**Measurements**:
- O2 partial pressure (hPa)
- Temperature (°C)
- Phase shift (degrees)
- Signal intensity (mV)
- Ambient light (mV)
- Ambient pressure (mbar)
- Relative humidity (%)

**Async Workflow**:
```cpp
// Non-blocking pattern
fdo2Sensor.startMeasurementAsync(includeRaw=true);

// Later, in main loop:
if (fdo2Sensor.isResponseReady()) {
    FDO2_Sensor::MeasurementData data;
    if (fdo2Sensor.getAsyncResult(data)) {
        // Use data.oxygenPartialPressure_hPa, etc.
    }
}
```

**Status Flags**:
- Detector reduced (warning)
- Signal out of range (fatal)
- Reference out of range (fatal)
- Temperature sensor failure (fatal)
- High humidity (warning)
- Pressure/humidity sensor failure

**Conversion**:
```cpp
O2_percent = 100.0 × pO2_hPa / ambientPressure_mbar
```

**Update Rate**: 2 Hz (500ms sample period, defined as `FDO2_SAMPLE_TIME_US`)

### Default Values System

All sensor readings use **-9.9** as the invalid/unavailable indicator:
- Sensor not detected during initialization
- Communication failure
- Read timeout
- Sensor disabled by compile switch

This provides immediate visual feedback in all outputs (serial, web, display).

**Affected Values**:
```cpp
sensorData.supply_pressure = -9.9;        // ABP2
sensorData.sfm3505_air_flow = -9.9;       // SFM3505 air
sensorData.sfm3505_o2_flow = -9.9;        // SFM3505 O2
sensorData.abpd_pressure = -9.9;          // ABPD
sensorData.abpd_temperature = -9.9;       // ABPD
```

---

## Control Systems

### 1. ActuatorControl

**Purpose**: Unified interface for valve control, signal generation, and PID control

**Multi-Channel Architecture**:
```cpp
static const uint8_t NUM_MUX_CHANNELS = 6;
ActuatorControl actuators[NUM_MUX_CHANNELS];  // Independent instances
#define actuator actuators[sysConfig.mux_channel]  // Current selection
```

Each channel:
- Runs independently with its own mode and parameters
- Shares sensor data but has independent setpoints
- Sends commands via SerialMuxRouter with channel prefix

**Control Modes**:

| Mode | Value | Description |
|------|-------|-------------|
| `PID_CONTROL` | 0 | Closed-loop flow control |
| `VALVE_SET_VALUE_CONTROL` | 1 | Manual valve position (0-100%) |
| `SINE_CONTROL` | 2 | Sinusoidal signal |
| `STEP_CONTROL` | 3 | Square wave |
| `TRIANGLE_CONTROL` | 4 | Triangle wave |
| `SWEEP_CONTROL` | 5 | Frequency sweep (log/linear) |

**PID Controller**:
```cpp
// Simple PI controller (D-term = 0 by default)
error = flow_reference - flow_measured;
integrator += error;
output = p_gain × error + i_gain × integrator;
valve_signal = output + valve_offset;  // Feedforward
valve_signal = clamp(valve_signal, 0.0, 100.0);

// Anti-windup: if reference is zero, reset integrator
if (flow_reference == 0) {
    valve_signal = 0.0;
    integrator = 0.0;
}
```

**Signal Generators**:

All signals are percentage-based (0-100%):

```cpp
// Sine wave (offset ± amplitude/2)
signal = amplitude/2 × (1 + sin(2π × phase)) + offset;

// Step wave (alternating levels)
signal = (phase < 0.5) ? (offset + amplitude) : offset;

// Triangle wave (linear ramp up/down)
signal = offset + amplitude × (phase < 0.5 ? 2×phase : 2-2×phase);

// Sweep (logarithmic or linear frequency)
freq = logarithmic ? 
    f_start × (f_stop/f_start)^(t/T) :
    f_start + (f_stop - f_start) × (t/T);
signal = amplitude/2 × (1 + sin(2π × accumulated_phase)) + offset;
```

**Configuration**:
```cpp
struct SignalGeneratorConfig {
    float offset;           // 0-100%
    float amplitude;        // 0-100%
    float period_seconds;   // Period for sine/step/triangle
};

struct SweepConfig {
    float start_freq;       // Hz
    float stop_freq;        // Hz
    float sweep_time;       // seconds
    bool logarithmic;       // true=log, false=linear
};
```

**Hardware Abstraction**:
- No direct GPIO control
- All valve commands sent via `SerialMuxRouter`
- Percentage values (0-100%) converted to hardware units as needed

### 2. VentilatorController

**Purpose**: High-level ventilator state machine implementing complete breath cycle management

**State Machine**:

```
      ┌──────────────┐
      │   VENT_OFF   │◄─── stop()
      └──────┬───────┘
             │ start()
             ▼
      ┌──────────────┐
   ┌─►│ INSP_PHASE1  │ (flow control, exp valve closed)
   │  └──────┬───────┘
   │         │ timeout
   │         ▼
   │  ┌──────────────┐
   │  │ INSP_PHASE2  │ (pressure pop-off via exp valve)
   │  └──────┬───────┘
   │         │ timeout (if pause enabled)
   │         ▼
   │  ┌──────────────┐
   │  │  INSP_PAUSE  │ (no flow, measure plateau pressure)
   │  └──────┬───────┘
   │         │ timeout
   │         ▼
   │  ┌──────────────┐
   │  │ EXP_NON_TRIG │ (passive exp, PEEP control)
   │  └──────┬───────┘
   │         │ timeout (if trigger enabled)
   │         ▼
   │  ┌──────────────┐
   │  │ EXP_SYNC_WAIT│ (trigger-sensitive window)
   │  └──────┬───────┘
   │         │ trigger OR timeout
   └─────────┘
```

**Breath Cycle Timing**:

```
Cycle_Time = 60 / Resp_Rate  [seconds]

I:E Ratio Split:
  Ti = Cycle_Time × (IE_Ratio / (1 + IE_Ratio))
  Te = Cycle_Time - Ti

Inspiration Breakdown:
  Pause_Time = Ti × Pause_Fraction
  Active_Ti = Ti - Pause_Time
  
  Insp1_Time = Active_Ti × (Insp1_Frac / (Insp1_Frac + Insp2_Frac))
  Insp2_Time = Active_Ti - Insp1_Time

Expiration Breakdown:
  NonTrig_Time = Te × (NonTrig_Frac / (NonTrig_Frac + Sync_Frac))
  Sync_Time = Te - NonTrig_Time
```

**Flow Calculation (Volume Control)**:
```cpp
Active_Insp_Time_min = (Insp1_Time + Insp2_Time) / 60,000,000  [minutes]
Required_Flow_slm = (Tidal_Volume_mL / 1000) / Active_Insp_Time_min

// Clamp to max flow limit
Target_Flow = min(Required_Flow, Max_Insp_Flow)
```

**Gas Mixing Algorithm**:

Mixing air (21% O2) and pure O2 to achieve target FiO2:

```cpp
// Solving: Air×0.21 + O2×1.0 = Total×FiO2
//          Air + O2 = Total

O2_Flow = Total_Flow × (FiO2 - 0.21) / 0.79
Air_Flow = Total_Flow - O2_Flow

// Boundary cases:
if (FiO2 <= 0.21) { O2_Flow=0; Air_Flow=Total; }
if (FiO2 >= 1.0)  { O2_Flow=Total; Air_Flow=0; }
```

**State Execution**:

| State | Insp Valves | Exp Valve | Purpose |
|-------|-------------|-----------|---------|
| INSP_PHASE1 | Flow control | CLOSED | Build pressure, deliver volume |
| INSP_PHASE2 | Flow control | Pressure limit | Pop-off if P > Max |
| INSP_PAUSE | Zero flow | CLOSED | Measure plateau pressure |
| EXP_NON_TRIG | Bias flow | PEEP control | Passive expiration |
| EXP_SYNC_WAIT | Bias flow | PEEP control | Accept triggers |

**Pressure Limiting**:
```cpp
// In INSP_PHASE1: Reduce flow if pressure exceeds limit
if (airway_pressure > max_pressure) {
    pressure_error = airway_pressure - max_pressure;
    set_flow -= pressure_error × 2.0;  // Proportional reduction
    set_flow = max(set_flow, 0);
}
```

**Breath Triggering**:

Enabled only in `EXP_SYNC_WAIT` state:
```cpp
// Flow trigger: inspiratory effort detected
if (insp_flow > bias_flow + flow_trigger) { trigger = true; }

// Pressure trigger: negative pressure from patient effort
if (airway_pressure < PEEP - pressure_trigger) { trigger = true; }
```

**Alarms**:
- High pressure (exceeds `highPressureAlarm_mbar`)
- Low pressure during inspiration (below `lowPressureAlarm_mbar` after 500ms)
- Apnea (no breath within `apneaTime_s`)
- Flow limitation (calculated flow exceeds max)

**MUX Integration**:
```cpp
// Air valve (Channel 1): Flow setpoint
muxRouter.sendSetFlow(MUX_AIR_VALVE, airValveFlow_slm);

// O2 valve (Channel 2): Flow setpoint
muxRouter.sendSetFlow(MUX_O2_VALVE, o2ValveFlow_slm);

// Exp valve (Channel 3): Pressure or closed
if (expValveClosed) {
    muxRouter.sendCommand(MUX_EXP_VALVE, 'V', 0);  // Close
} else if (expValveAsPressure) {
    muxRouter.sendSetPressure(MUX_EXP_VALVE, setpoint_mbar);
} else {
    muxRouter.sendCommand(MUX_EXP_VALVE, 'V', setpoint_percent);
}
```

**Configuration Parameters**:
```cpp
struct VentilatorConfig {
    // Timing
    float respRate;              // 4-60 BPM
    float ieRatio;               // 0.1-4.0 (Ti/Te)
    float inspPauseFraction;     // 0-0.5
    float insp1Fraction;         // 0-1.0
    float insp2Fraction;         // 0-1.0
    float expNonTrigFraction;    // 0-1.0
    float expSyncFraction;       // 0-1.0
    
    // Volume/Flow
    float tidalVolume_mL;        // 50-2000 mL
    float maxInspFlow_slm;       // 5-120 slm
    float totalFlow_slm;         // 1-120 slm
    bool useVolumeControl;
    
    // Pressure
    float maxPressure_mbar;      // 5-80 mbar
    float peep_mbar;             // 0-30 mbar
    float pressureRampTime_ms;   // 0-500 ms
    
    // Gas
    float targetFiO2;            // 0.21-1.0
    
    // Triggering
    bool triggerEnabled;
    float biasFlow_slm;          // 0-20 slm
    float flowTrigger_slm;       // 0.5-20 slm
    float pressureTrigger_mbar;  // 0.5-10 mbar
    
    // Alarms
    float highPressureAlarm_mbar;
    float lowPressureAlarm_mbar;
    float apneaTime_s;
};
```

---

## Communication Protocols

### 1. Serial MUX Router

**Purpose**: Multiplexed communication protocol for managing multiple external actuators

**Protocol Format**:

```
TX (ESP32 → Actuator): [ADDR]<CMD>VALUE\n
RX (Actuator → ESP32): [ADDR]<CMD>VALUE\n

Examples:
  "3F25.5\n"  → Channel 3, Set Flow to 25.5 slm
  "3I1.25\n"  → Channel 3, Current = 1.25 A
  "V50\n"     → Channel 0 (no prefix), Set Valve to 50%
```

**Channel Addressing**:
- **Channel 0**: Direct (no prefix) - backward compatible
- **Channels 1-5**: Digit prefix (1-5)

**Command Set**:

| Direction | Command | Description | Example |
|-----------|---------|-------------|---------|
| TX | `C` | Set Current (A) | `3C1.5\n` |
| TX | `V` | Set Voltage (V) or Valve (%) | `V75\n` |
| TX | `F` | Set Flow (slm) | `1F30.5\n` |
| TX | `P` | Set Pressure (mbar) | `2P25.0\n` |
| TX | `R` | Set Blower RPM | `5R3000\n` |
| RX | `I` | Current measurement (A) | `3I1.25\n` |
| RX | `F` | Actual flow (slm) | `1F29.8\n` |
| RX | `P` | Actual pressure (mbar) | `2P24.5\n` |
| RX | `R` | Blower RPM | `5R2980\n` |

**Data Management**:
```cpp
// Per-channel storage (6 channels)
float _current[6];              // Actuator current draw
float _blowerRPM[6];            // Blower speed
float _actualFlow[6];           // Measured flow
float _actualPressure[6];       // Measured pressure
uint32_t _currentTimestamp[6];  // Last update times
// ... timestamps for each measurement type
```

**Staleness Detection**:
```cpp
bool isCurrentStale(uint8_t address, uint32_t max_age_ms = 1000) const;
// Returns true if data not received within max_age_ms
```

**Non-Blocking Operation**:
```cpp
// Called in main loop
muxRouter.update();  // Processes all available serial bytes

// No delays, no blocking reads
// Data accessed via getters with staleness checks
```

**Application-Specific Channel Assignments**:
```cpp
#define MUX_DIRECT      0  // No prefix
#define MUX_AIR_VALVE   1  // Air supply
#define MUX_O2_VALVE    2  // O2 supply
#define MUX_EXP_VALVE   3  // Expiratory valve
#define MUX_NC          4  // Reserved
#define MUX_BLOWER      5  // Blower motor
```

### 2. Command Parser

**Purpose**: Serial command interpreter for system configuration

**Command Categories**:

#### Control System Commands

| Cmd | Parameter | Description | Example |
|-----|-----------|-------------|---------|
| `T` | µs | GUI/serial output interval | `T100000` = 100ms |
| `X` | µs | Control loop interval | `X10000` = 10ms |
| `Q` | 0-9 | Quiet mode (output format) | `Q3` |
| `M` | 0-5 | MUX channel selection | `M1` |
| `!` | - | Print all settings | `!` |
| `?` | - | Help | `?` |

#### Actuator/Signal Generator Commands

| Cmd | Parameter | Description | Example |
|-----|-----------|-------------|---------|
| `C` | 0-5 | Controller mode | `C0`=PID, `C1`=Set |
| `V` | 0-100% | Manual valve control | `V50` |
| `Z` | - | Zero PID integrator | `Z` |
| `O` | 0-100% | Signal offset | `O25` |
| `A` | 0-100% | Signal amplitude | `A10` |
| `S` | seconds | Signal period | `S1.0` |
| `W` | ... | Sweep config | `W0.1,10,20,log` |

#### PID Tuning Commands

| Cmd | Parameter | Description | Example |
|-----|-----------|-------------|---------|
| `F` | L/min | Flow reference setpoint | `F50.0` |
| `P` | float | Proportional gain | `P1.5` |
| `I` | float | Integral gain | `I0.8` |
| `D` | float | Derivative gain | `D0.0` |

#### Ventilator Commands (Two-Character)

| Cmd | Parameter | Description | Example |
|-----|-----------|-------------|---------|
| `VO` | 0/1 | Ventilator ON/OFF | `VO1` |
| `VS` | - | Status query | `VS` |
| `RR` | BPM | Respiratory rate | `RR12` |
| `IE` | ratio | I:E ratio | `IE0.5` |
| `VT` | mL | Tidal volume | `VT500` |
| `PI` | mbar | Max pressure | `PI30` |
| `PE` | mbar | PEEP | `PE5` |
| `FI` | % | Target FiO2 | `FI40` |
| `TE` | 0/1 | Trigger enable | `TE1` |
| `BF` | slm | Bias flow | `BF2` |
| `FT` | slm | Flow trigger | `FT2` |
| `PT` | mbar | Pressure trigger | `PT2` |

**Command Priority**: Ventilator commands (2-char) processed before legacy commands (1-char)

---

## Web Interface

### WiFi Management

**Default State**: WiFi **disabled** on boot (power savings, EMI reduction)

**Button Control**:
- **Long press (>1s)**: Enable WiFi AP
  - SSID: `PMIXERSSID` (configurable in `main.hpp`)
  - Password: `PMIXERPWD`
  - IP: Typically 192.168.4.1 (auto-assigned)
  - Display: "Short press: disable"
- **Short press**: Disable WiFi
  - Display: "Long press: enable"

### Web Server Endpoints

| Endpoint | Method | Content-Type | Description |
|----------|--------|--------------|-------------|
| `/` | GET | text/html | Main dashboard |
| `/data` | GET | application/json | Current values (all sensors) |
| `/history` | GET | application/json | Last 100 samples |
| `/dataBuffer` | GET | application/json | High-speed buffer (batch) |
| `/ventilator` | GET | application/json | Ventilator settings/status |

### High-Speed Data Architecture

**Collection Strategy**:
```
ESP32 Side:
  └─ Control Loop (100 Hz default)
     └─ addDataPoint() called every 10ms
        └─ Data buffered in vectors (max 3000 samples)
           └─ ~20 samples accumulate per 200ms

Client Side:
  └─ Poll /dataBuffer every 200ms
     └─ Receive ~20 samples as JSON arrays
        └─ Process all samples in one batch
           └─ Update chart once (no animation)
              └─ Buffer cleared on server
```

**Buffer Structure**:
```json
{
  "count": 20,
  "mode": "PID M1",
  "timestamps": [t1, t2, ..., t20],
  "flow": [f1, f2, ..., f20],
  "pressure": [p1, p2, ..., p20],
  "lowPressure": [lp1, lp2, ..., lp20],
  "temperature": [t1, t2, ..., t20],
  "valve": [v1, v2, ..., v20],
  "current": [i1, i2, ..., i20],
  "flow2": [f21, f22, ..., f2_20],
  "pressure2": [p21, p22, ..., p2_20]
}
```

**Memory Management**:
- Display buffer: Last 100 samples (rolling window)
- High-speed buffer: Up to 3000 samples (cleared after fetch)
- Total: ~108 KB (3000 samples × 9 channels × 4 bytes)

### Chart Configuration

**Chart.js Setup**:
```javascript
// 9 datasets (dual-bus + valve + current + O2)
datasets: [
  { label: 'Flow (Bus 0)', color: '#0078D7', pointRadius: 0 },
  { label: 'Flow (Bus 1)', color: '#00BFFF', borderDash: [5,3] },
  { label: 'Pressure (Bus 0)', color: '#FF6B6B' },
  { label: 'Pressure (Bus 1)', color: '#FF4500', borderDash: [5,3] },
  { label: 'Low Pressure', color: '#FF69B4' },
  { label: 'Temperature', color: '#9370DB' },
  { label: 'Valve Signal', color: '#4ECDC4' },
  { label: 'Current', color: '#FFA500' },
  { label: 'O2', color: '#32CD32' }
]

// Performance settings
options: {
  animation: false,          // No animation for smooth updates
  pointRadius: 0,            // Lines only, no markers
  borderWidth: 2,            // 2px lines
  tension: 0.3,              // Smooth curves
  interaction: { mode: 'index', intersect: false }
}
```

**Display Configuration**:
- Max points: `PMIXER_GRAPH_DISPLAY_POINTS` (default 512)
- Update rate: 200ms fetch interval = ~5 Hz web updates
- Data rate: 100 Hz collection (configurable via `X` command)

### Data Export

**Format**: Tab-separated values (TSV)

**Filename**: `pmixer_data_YYYYMMDD_HHMMSS.txt`

**Columns**:
```
Timestamp (ms)  Flow  Pressure  Low_Pressure  Temperature  Valve  Current  Flow2  Pressure2
```

**Features**:
- Browser-side generation (no server storage)
- Includes all buffered historical data
- Automatic download via blob URL

---

## User Interface

### Display Modes

#### 1. Boot Screen (2 seconds on startup)
```
         P-Mixer
      
      Version: v2.x
      
      Build: Jan 15 2026
         12:34:56
```

#### 2. Live Data Display (default)
```
┌──────────────────────────┐
│ P-Mixer          v2.x    │
├──────────────────────────┤
│ Status                   │
│  Mode: PID M1            │
│  Flow: 25.3 slm Air      │
│  Flow2: 15.8 slm Air     │
│  HP: 45.2 LP: 12.3 22.5°C│
│  Valve: 75%              │
│  Current: 1.25 A         │
│  pO2: 210.5 hPa          │
├──────────────────────────┤
│ WiFi                     │
│  192.168.4.1             │
│  SSID: P-Mixer           │
│  Short press: disable    │
└──────────────────────────┘
```

#### 3. Ventilator Settings (Boot button long press)
```
┌──────────────────────────┐
│  Ventilator Settings     │
│                          │
│  Vent: ON  State: INSP1  │
│  RR=12.0 VT=500 IE=0.50  │
│  PI=30.0 PE=5.0 MF=60.0  │
│  #142 PkP=28.5 Vt=485mL  │
│                          │
│  Short press: back       │
└──────────────────────────┘
```

#### 4. Info Screen (I2C device scan results)
```
┌──────────────────────────┐
│        Info:             │
│                          │
│  I2C devices found       │
│  Bus0: 0x2e 0x28         │
│  Bus1: 0x2e 0x28         │
│                          │
└──────────────────────────┘
```

### Button Mapping

| Button | Action | Function |
|--------|--------|----------|
| Interaction (GPIO0) | Short press | Disable WiFi |
| Interaction (GPIO0) | Long press (>1s) | Enable WiFi |
| Boot (GPIO21) | Short press | Return to live data |
| Boot (GPIO21) | Long press (>1s) | Show ventilator settings |

### Display Update Strategy

**Optimization**: Only redraw changed values
```cpp
static String oldValue = "";
if (newValue != oldValue) {
    // Clear old text (draw in background color)
    tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
    tft.drawString(oldValue, x, y);
    
    // Draw new text
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.drawString(newValue, x, y);
    
    oldValue = newValue;
}
```

**Update Rates**:
- Live data: 50ms (20 Hz) via `SET_UI_UPDATE_TIME`
- O2 display: 500ms (2 Hz) via `FDO2_SAMPLE_TIME_US`
- Ventilator settings: Updated on mode switch only

---

## Data Structures

### SystemConfig
```cpp
struct SystemConfig {
    uint32_t delta_t;              // GUI/serial output interval (µs)
    uint32_t control_interval;     // Control loop interval (µs)
    uint32_t PressSamplTime;       // Pressure sampling interval (µs)
    int quiet_mode;                // Output format (0-9)
    float digital_flow_reference;  // Flow setpoint (L/min)
    uint8_t mux_channel;           // Selected MUX channel (0-5)
};

// Default values (in setup()):
sysConfig.delta_t = 100000;          // 100ms = 10Hz output
sysConfig.control_interval = 10000;  // 10ms = 100Hz control
sysConfig.PressSamplTime = 100000;   // 100ms pressure sampling
sysConfig.quiet_mode = 0;            // Verbose
sysConfig.digital_flow_reference = 0.0;
sysConfig.mux_channel = 0;           // Direct channel
```

### SensorData
```cpp
struct SensorData {
    float differential_pressure;   // Legacy (unused) = -9.9
    float flow;                    // Legacy (unused) = -9.9
    float supply_pressure;         // ABP2 high pressure (kPa)
    float sfm3505_air_flow;        // SFM3505 air channel (slm)
    float sfm3505_o2_flow;         // SFM3505 O2 channel (slm)
    float abpd_pressure;           // ABPD low pressure (kPa)
    float abpd_temperature;        // ABPD temperature (°C)
};

// Invalid/unavailable indicator: -9.9 for all fields
```

### PIDConfig
```cpp
struct PIDConfig {
    float p_gain;        // Proportional gain
    float i_gain;        // Integral gain
    float d_gain;        // Derivative gain (unused)
    float valve_offset;  // Feedforward term (%)
};

// Default values:
_pid_config.p_gain = 1.0;
_pid_config.i_gain = 1.0;
_pid_config.d_gain = 0.0;
_pid_config.valve_offset = 0.0;
```

### SignalGeneratorConfig
```cpp
struct SignalGeneratorConfig {
    float offset;           // 0-100%
    float amplitude;        // 0-100%
    float period_seconds;   // Period for periodic signals
};

// Default values:
_sig_gen_config.offset = 25.0;      // 25%
_sig_gen_config.amplitude = 10.0;   // 10%
_sig_gen_config.period_seconds = 1.0;
```

### SweepConfig
```cpp
struct SweepConfig {
    float start_freq;      // Hz
    float stop_freq;       // Hz
    float sweep_time;      // seconds
    bool logarithmic;      // true=log, false=linear
};

// Default values:
_sweep_config.start_freq = 0.1;     // 0.1 Hz
_sweep_config.stop_freq = 10.0;     // 10 Hz
_sweep_config.sweep_time = 10.0;    // 10 seconds
_sweep_config.logarithmic = false;  // Linear
```

### VentilatorConfig
```cpp
struct VentilatorConfig {
    // Timing
    float respRate;              // 4-60 BPM
    float ieRatio;               // 0.1-4.0 (Ti/Te)
    float inspPauseFraction;     // 0-0.5
    float insp1Fraction;         // 0-1.0
    float insp2Fraction;         // 0-1.0
    float expNonTrigFraction;    // 0-1.0
    float expSyncFraction;       // 0-1.0
    
    // Volume/Flow
    float tidalVolume_mL;        // 50-2000 mL
    float maxInspFlow_slm;       // 5-120 slm
    float totalFlow_slm;         // 1-120 slm
    bool useVolumeControl;       // true=volume, false=flow
    
    // Pressure
    float maxPressure_mbar;      // 5-80 mbar
    float peep_mbar;             // 0-30 mbar
    float pressureRampTime_ms;   // 0-500 ms (unused)
    
    // Gas
    float targetFiO2;            // 0.21-1.0
    
    // Triggering
    bool triggerEnabled;         // Enable patient triggering
    float biasFlow_slm;          // 0-20 slm
    float flowTrigger_slm;       // 0.5-20 slm
    float pressureTrigger_mbar;  // 0.5-10 mbar
    
    // Alarms
    float highPressureAlarm_mbar;
    float lowPressureAlarm_mbar;
    float apneaTime_s;
};

// Default values: See setDefaultConfig() in VentilatorController
```

### VentilatorMeasurements
```cpp
struct VentilatorMeasurements {
    float airwayPressure_mbar;      // From ABPD sensor
    float inspFlow_slm;             // From SFM3505 (Bus 0)
    float expFlow_slm;              // Reserved (sensor not connected)
    float deliveredO2_percent;      // From FDO2 sensor
};
```

### VentilatorOutputs
```cpp
struct VentilatorOutputs {
    float airValveFlow_slm;         // Air valve flow setpoint
    float o2ValveFlow_slm;          // O2 valve flow setpoint
    float expValveSetpoint;         // Exp valve setpoint (pressure or position)
    bool expValveAsPressure;        // true=pressure control, false=position
    bool expValveClosed;            // true=fully closed
};
```

### VentilatorStatus
```cpp
struct VentilatorStatus {
    VentilatorState state;          // Current state
    uint32_t breathCount;           // Total breaths delivered
    float peakPressure_mbar;        // Peak pressure this breath
    float plateauPressure_mbar;     // Plateau pressure this breath
    float measuredVt_mL;            // Measured tidal volume
    float calculatedFlow_slm;       // Calculated target flow
    float actualSetFlow_slm;        // Actual flow setpoint sent
    float cycleProgress;            // 0.0-1.0 (current position in cycle)
    bool triggered;                 // Was breath patient-triggered?
    bool flowLimited;               // Is flow clamped to max?
    bool pressureLimited;           // Is pressure limiting active?
};
```

---

## Timing & Performance

### Loop Rates

**Configurable via Commands**:
```
X<microseconds>  - Control loop interval (default 10000 = 100 Hz)
T<microseconds>  - Output loop interval (default 100000 = 10 Hz)
```

**Typical Configuration**:
- Control loop: 10ms (100 Hz) - sensor reads, PID, actuator control
- Output loop: 100ms (10 Hz) - serial output, display updates
- Display update: 50ms (20 Hz) - TFT refresh
- O2 update: 500ms (2 Hz) - FDO2 display
- Web fetch: 200ms (5 Hz) - client-side polling

### Performance Characteristics

**Execution Times** (ESP32-S3 @ 240 MHz):
- I2C sensor read: 2-5ms per sensor
- PID calculation: <1ms
- Serial MUX processing: <1ms
- Display update: 5-10ms (only changed fields)
- Web buffer update: <1ms

**Typical Control Loop Breakdown** (10ms total):
```
├─ SFM3505 read (Bus 0):           2ms
├─ SFM3505 read (Bus 1):           2ms
├─ ABPD read (Bus 0):              1ms
├─ ABPD read (Bus 1):              1ms
├─ PID + actuator control:         1ms
├─ Serial MUX update:              1ms
└─ Web buffer update:              <1ms
─────────────────────────────────────
Total:                              8ms
Margin:                             2ms (20%)
```

**CPU Usage**: ~10-20% at 100 Hz control rate (leaves headroom for other tasks)

**Memory Usage**:
- Web buffer: ~108 KB (3000 samples × 9 channels × 4 bytes)
- Display buffer: ~4 KB (100 samples × 9 channels × 4 bytes)
- Serial buffers: <1 KB
- Total RAM usage: ~150 KB (ESP32-S3 has 512 KB SRAM)

### Data Rate Capabilities

**Current**: 100 Hz collection, 200ms fetch = ~20 samples/batch

**Higher Rates** (supported by architecture):
```
200 Hz:  X5000  = 5ms control loop → ~40 samples/fetch
500 Hz:  X2000  = 2ms control loop → ~100 samples/fetch
1000 Hz: X1000  = 1ms control loop → ~200 samples/fetch
```

**Limitations**:
- I2C bus speed (400 kHz) limits max sensor read rate
- Web fetch interval should be adjusted for >500 Hz rates
- Buffer size (3000 samples) provides ~15-30 seconds at high rates

---

## Configuration

### Compile-Time Configuration

**In `SensorReader.hpp`**:
```cpp
// Pressure sensor selection (only ONE can be enabled per bus)
#define USE_ABP2_PRESSURE_SENSOR    // High pressure (0-150 PSI)
//#define USE_ABPD_PRESSURE_SENSOR  // Low pressure (0-100 mbar)

// CRC validation
#define SFM3505_ENABLE_CRC_CHECK 0  // 0=disabled, 1=enabled
```

**In `main.hpp`**:
```cpp
// Web interface
#define PMIXER_GRAPH_DISPLAY_POINTS 512  // Chart display points
#define PMIXERSSID "P-Mixer"             // WiFi AP SSID
#define PMIXERPWD "your_password"        // WiFi AP password

// Update intervals
#define SET_UI_UPDATE_TIME 50000         // Display refresh (µs)
#define FDO2_SAMPLE_TIME_US 500000       // O2 sensor rate (µs)
```

**In `PinConfig.h`**:
```cpp
// I2C Bus 0
#define I2C0_SDA_PIN 43
#define I2C0_SCL_PIN 44
#define I2C0_CLOCK_FREQ 400000

// I2C Bus 1
#define I2C1_SDA_PIN 10
#define I2C1_SCL_PIN 11

// Serial buses
#define SERIAL1_TX_PIN 17
#define SERIAL1_RX_PIN 18
#define SERIAL2_TX_PIN 12
#define SERIAL2_RX_PIN 13
```

### Runtime Configuration

**Via Serial Commands** - see [Command Parser](#2-command-parser) section

**Via Code** (in `setup()`):
```cpp
// System config
sysConfig.delta_t = 100000;          // 100ms output
sysConfig.control_interval = 10000;  // 10ms control
sysConfig.quiet_mode = 0;            // Verbose

// PID tuning
PIDConfig pid;
pid.p_gain = 1.5;
pid.i_gain = 0.8;
actuator.setPIDConfig(pid);

// Signal generator
SignalGeneratorConfig sig;
sig.offset = 30.0;
sig.amplitude = 15.0;
sig.period_seconds = 2.0;
actuator.setSignalGeneratorConfig(sig);
```

### Output Modes (Quiet Mode)

| Mode | Description | Output Format |
|------|-------------|---------------|
| 0 | Verbose | `P: 45.2  LP: 12.3  T: 22.5  Air0: 25.3  Air1: 15.8  P1: 48.1  Valve: 75.0` |
| 1 | Quiet | No output |
| 2 | Debug | `I 0.5 E 2.3 V 75.0 F 50.2 Air 25.3` |
| 3 | Special | `[PID M1] V=75.00% -> I=1.250A F=25.3 P=45.2` |
| 4 | Abbreviated | `45.2 50.2 75.0` |
| 5 | Labeled | `SP0:45.2 LP:12.3 T:22.5 Air0:25.3 Air1:15.8 SP1:48.1 V:75.0 I:1.250 O2:210.5hPa 21.0%` |
| 6 | High-speed TSV | `123456	45.2	12.3	22.5	25.3	15.8	48.1	75.0	1.250	210.5	21.0` |
| 7 | FDO2 Basic | `O2: 210.5 hPa (21.0%) | Temp: 22.5°C | Status: 0x00 OK` |
| 8 | FDO2 Extended | `O2: 210.5 hPa | T: 22.5°C | Phase: 45.3° | Signal: 250.5 mV | ...` |

**Mode 6 (High-Speed TSV)** is optimized for data logging:
- Tab-separated (minimal bandwidth)
- No labels (reduces overhead)
- All 9 critical variables
- Timestamp in milliseconds

---

## Safety Features

### Ventilator Safety

**Pressure Limiting**:
```cpp
// INSP_PHASE1: Proportional flow reduction
if (pressure > max_pressure) {
    flow = flow - (pressure - max_pressure) × 2.0;
}

// INSP_PHASE2: Pressure pop-off valve
expValve.setPressure(max_pressure);  // Releases if exceeded
```

**Alarms**:
- High pressure: Immediate detection, logged to status
- Low pressure: After 500ms in inspiration state
- Apnea: No breath within timeout (default 20s)
- Breath-by-breath monitoring: Peak P, plateau P, delivered Vt

**Fail-Safes**:
- Valve signals clamped to 0-100%
- Flow clamped to max inspiratory flow
- Cycle timeout protection
- Anti-windup (PID integrator reset on zero reference)

### General Safety

**Data Validation**:
- Sensor staleness detection (configurable timeout)
- Default -9.9 for invalid/missing data
- CRC validation (optional for SFM3505)
- ABP2 status byte monitoring (power, memory, saturation)

**Communication**:
- Serial buffer overflow protection (fixed-size buffers)
- Non-blocking I/O (no deadlocks)
- MUX command confirmation via telemetry
- Web buffer size limits (3000 samples max)

**Hardware Protection**:
- No direct GPIO valve control (all via serial MUX)
- Interrupt-based button handling (debounced)
- Display power management
- I2C error recovery (continue on single sensor failure)

---

## Extension Points

### Adding New Sensors (I2C)

1. **Add I2C address constant** to `SensorReader.hpp`:
   ```cpp
   #define I2Cadr_NEWSENSOR 0x3C
   ```

2. **Add detection flag**:
   ```cpp
   bool _hasNewSensor;
   ```

3. **Implement detection** in `initialize()`:
   ```cpp
   _wire->beginTransmission(I2Cadr_NEWSENSOR);
   if (_wire->endTransmission() == 0) {
       _hasNewSensor = true;
       // Initialize sensor...
   }
   ```

4. **Add read method**:
   ```cpp
   bool readNewSensor(float& value);
   ```

5. **Update `SensorData` structure**:
   ```cpp
   float new_sensor_value = -9.9;
   ```

6. **Call in main loop** (control or output loop as appropriate)

### Adding New MUX Channels

1. **Increase channel count** (if needed):
   ```cpp
   static const uint8_t NUM_MUX_CHANNELS = 8;  // Was 6
   ```

2. **Define channel assignment**:
   ```cpp
   #define MUX_NEW_DEVICE 6  // New channel
   ```

3. **Use like any other channel** - no code changes needed (array-based design)

### Adding New Control Modes

1. **Add enum value** to `ControllerMode`:
   ```cpp
   enum ControllerMode {
       // ...existing modes...
       CUSTOM_CONTROL
   };
   ```

2. **Implement generator method** in `ActuatorControl.cpp`:
   ```cpp
   float ActuatorControl::generateCustom(float phase) {
       // Your algorithm here
       return signal_percent;
   }
   ```

3. **Add case to state machine**:
   ```cpp
   case CUSTOM_CONTROL:
       signal = generateCustom(phase);
       break;
   ```

4. **Update command parser** (if user-accessible)

### Adding New Output Modes

1. **Add case** to `outputData()` switch in `main.cpp`:
   ```cpp
   case 9:  // New mode
       hostCom.printf("Custom format: ...\n", ...);
       break;
   ```

2. **Update help text** in `CommandParser::printHelp()`

### Future Enhancements

#### With I2C Multiplexer (TCA9548A)

When an I2C mux is added:
```cpp
// In SensorReader::initialize()
selectI2CChannel(0);  // ABP2 on channel 0
_hasABP2 = detectABP2();

selectI2CChannel(1);  // ABPD on channel 1
_hasABPD = detectABPD();

// Both sensors can now coexist!
```

Benefits:
- Use both ABP2 and ABPD simultaneously
- Support >2 identical sensors (up to 8 per mux)
- Isolate problematic sensors (prevent bus lockup)

#### Higher Data Rates

Current architecture supports:
- 500 Hz: `X2000` = 2ms control loop
- 1 kHz: `X1000` = 1ms control loop (I2C speed becomes bottleneck)

Considerations:
- Reduce sensors read per loop (e.g., alternate buses)
- Increase web fetch interval for higher rates
- Use DMA for I2C reads (ESP32-S3 supports this)

---

## Troubleshooting

### Sensor Not Detected

**Symptoms**:
- Display shows -9.9
- Serial output shows -9.9
- Boot screen shows "No I2C devices found"

**Diagnosis**:
1. Check I2C bus scan results in serial output
2. Verify sensor address matches expected (0x2E, 0x28, etc.)
3. Use multimeter to check sensor power (3.3V or 5V as needed)
4. Check SDA/SCL connections (GPIO43/44 for Bus 0, GPIO10/11 for Bus 1)
5. Verify pull-up resistors present (typically 4.7kΩ to 3.3V)

**Solutions**:
- Swap SDA/SCL if wired backwards
- Add external pull-up resistors if missing
- Check for shorts or loose connections
- Ensure sensor is powered before ESP32 boots

### Stale Data from MUX

**Symptoms**:
- Display shows "STALE data"
- Current reading is -9.9
- Mode Q3 output shows "Actuator: STALE data"

**Diagnosis**:
1. Check Serial1 wiring (TX/RX crossed between ESP32 and actuator?)
2. Verify baud rate match (230400 on both ends)
3. Enable debug output:
   ```cpp
   #define DEBUG_SERIAL_MUX_ROUTER
   ```
4. Check actuator is transmitting (use logic analyzer or oscilloscope)

**Solutions**:
- Cross TX/RX wires (ESP32 TX → Actuator RX, ESP32 RX → Actuator TX)
- Verify actuator firmware is sending `<CMD>VALUE\n` format
- Check for correct channel prefix (e.g., `3I1.25\n` for channel 3)
- Reduce staleness timeout if data is arriving but slow:
   ```cpp
   muxRouter.isCurrentStale(channel, 2000);  // 2 second timeout
   ```

### WiFi Not Starting

**Symptoms**:
- Long press has no effect
- Display still shows "WiFi OFF"
- Cannot connect to AP

**Diagnosis**:
1. Check serial output for "WiFi AP Started" message
2. Verify SSID/password in `main.hpp` (no special characters?)
3. Check button GPIO (GPIO0 for interaction key)
4. Scan for WiFi networks on phone/laptop

**Solutions**:
- Ensure long press >1 second (LED should change)
- Recompile with different SSID/password
- Check antenna connection (if external antenna model)
- Restart ESP32 after WiFi enabled

### Display Issues

**Symptoms**:
- Blank screen
- Garbled display
- Wrong colors or orientation

**Diagnosis**:
1. Check display power (GPIO15 must be HIGH)
2. Verify TFT_eSPI configuration in `User_Setup.h`
3. Check rotation setting (`setRotation(0)` for portrait)
4. Measure backlight voltage (GPIO38)

**Solutions**:
- Ensure `digitalWrite(DISPLAY_POWER_PIN, HIGH)` in setup
- Verify `TFT_eSPI` library configuration matches T-Display S3:
   ```cpp
   #define ST7789_DRIVER
   #define TFT_WIDTH  170
   #define TFT_HEIGHT 320
   ```
- Check SPI pins match board (handled by library for T-Display S3)
- Adjust backlight brightness if too dim

### ABP2 Pressure Sensor Issues

**Symptoms**:
- Pressure reading is 0 or -9.9
- Serial shows "ABP2 error: Device not powered"
- Status byte errors

**Diagnosis**:
1. Check 7-byte read (should be exactly 7)
2. Examine status byte (see bit definitions in sensor section)
3. Verify async timing (command → wait 5ms → read)
4. Check I2C address (0x28) not conflicting with ABPD

**Solutions**:
- Ensure sensor has stable power supply
- Wait ≥5ms between command and read
- If using ABPD, disable ABP2:
   ```cpp
   //#define USE_ABP2_PRESSURE_SENSOR
   #define USE_ABPD_PRESSURE_SENSOR
   ```
- Check for I2C bus errors (address NACK, data NACK)

### FDO2 Oxygen Sensor Issues

**Symptoms**:
- O2 reading is -9.9
- Serial shows "FDO2 not initialized"
- Timeout errors

**Diagnosis**:
1. Check Serial2 wiring (GPIO12 TX, GPIO13 RX)
2. Verify baud rate (19200)
3. Check sensor power (may require 5V, not 3.3V)
4. Enable verbose output (mode Q7 or Q8)

**Solutions**:
- Ensure TX/RX wires not crossed
- Wait for sensor warm-up (1+ second after power-on)
- Check for correct CR termination (`\r`, not `\n`)
- Verify sensor is PreSens FDO2 (not other O2 sensor type)

### High CPU Usage / Slow Response

**Symptoms**:
- Control loop taking >10ms
- Web interface laggy
- Display updates slow

**Diagnosis**:
1. Measure loop time (add timing code):
   ```cpp
   uint32_t start = micros();
   // ... control loop code ...