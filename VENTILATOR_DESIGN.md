# Ventilator Controller Design

## Overview

This document outlines the design for a ventilator/respirator function in the ParallelMixer system. The ventilator controller manages gas delivery (O2 and Air) and expiratory pressure (PEEP) using the existing actuator infrastructure.

---

## Technical Overview

### Hardware Platform

**MCU**: LilyGo T-Display S3 (ESP32-S3) with integrated ST7789 TFT display (170×320 pixels)

### Pin Configuration

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        T-DISPLAY S3 PIN ASSIGNMENTS                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         I2C BUS 0 (Wire)                            │   │
│  │    GPIO43 (SDA) ──┬── 4.7kΩ to 3.3V                                 │   │
│  │                   ├── SFM3505 Flow Sensor (0x2E)                    │   │
│  │                   ├── ABP2 Supply Pressure (0x28)                   │   │
│  │                   ├── ABPD Low Pressure (0x18)                      │   │
│  │                   └── MCP4725 DAC (0x60)                            │   │
│  │    GPIO44 (SCL) ──┴── 4.7kΩ to 3.3V                                 │   │
│  │    Clock: 1MHz (actual ~660-800kHz due to ESP32 limits)             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         I2C BUS 1 (Wire1)                           │   │
│  │    GPIO10 (SDA) ──┬── 4.7kΩ to 3.3V                                 │   │
│  │                   ├── SFM3505 Flow Sensor 2 (0x2E)                  │   │
│  │                   └── ABP2 Pressure Sensor 2 (0x28)                 │   │
│  │    GPIO11 (SCL) ──┴── 4.7kΩ to 3.3V                                 │   │
│  │    Clock: 1MHz (same as Bus 0)                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    SERIAL1 (MUX Router) @ 460800 baud               │   │
│  │    GPIO17 (TX) ────► Serial MUX ────► Valve Controllers (Ch 1-5)   │   │
│  │    GPIO18 (RX) ◄──── Serial MUX ◄──── Actuator Feedback            │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    SERIAL2 (FDO2 O2 Sensor) @ 19200 baud            │   │
│  │    GPIO12 (TX) ────► Pyroscience FDO2 O2 Optical Sensor            │   │
│  │    GPIO13 (RX) ◄──── O2 partial pressure, temperature, status      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         USER INTERFACE                              │   │
│  │    GPIO14 ─────── Key 1: Long=WiFi On, Short=WiFi Off              │   │
│  │    GPIO0  ─────── Key 2: Long=Show Vent Settings, Short=Back       │   │
│  │    GPIO15 ─────── Display Power Enable (set HIGH)                   │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Serial MUX Channel Mapping

| MUX Ch | Enum            | Device        | Function                    |
|--------|-----------------|---------------|-----------------------------|
| 0      | `MUX_DIRECT`    | Direct        | No prefix (legacy mode)     |
| 1      | `MUX_AIR_VALVE` | Air Valve     | Air flow control (insp)     |
| 2      | `MUX_O2_VALVE`  | O2 Valve      | O2 flow control (insp)      |
| 3      | `MUX_EXP_VALVE` | Exp Valve     | Expiratory/PEEP control     |
| 4      | `MUX_NC`        | (Reserved)    | Not connected               |
| 5      | `MUX_BLOWER`    | Blower        | Blower motor control        |

### Sensor Summary

| Sensor      | Interface | Address | Measurement               | Sample Rate |
|-------------|-----------|---------|---------------------------|-------------|
| SFM3505     | I2C Bus 0 | 0x2E    | Air/O2 Flow (slm)         | 100 Hz      |
| ABP2        | I2C Bus 0 | 0x28    | Supply Pressure (bar)     | 10-100 Hz   |
| ABPD        | I2C Bus 0 | 0x18    | Airway Pressure (mbar)    | 10-100 Hz   |
| SFM3505 #2  | I2C Bus 1 | 0x2E    | Air/O2 Flow (slm)         | 100 Hz      |
| ABP2 #2     | I2C Bus 1 | 0x28    | Supply Pressure (bar)     | 10-100 Hz   |
| FDO2        | Serial2   | -       | O2 partial pressure (hPa) | 2 Hz        |
| MCP4725     | I2C Bus 0 | 0x60    | DAC output (legacy)       | -           |

---

## Class Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              MAIN APPLICATION                               │
│                                 (main.cpp)                                  │
└─────────────────────────────────────┬───────────────────────────────────────┘
                                      │
        ┌─────────────────────────────┼─────────────────────────────┐
        │                             │                             │
        ▼                             ▼                             ▼
┌───────────────────┐   ┌───────────────────────┐   ┌───────────────────────┐
│   CommandParser   │   │  VentilatorController │   │    ActuatorControl    │
├───────────────────┤   │         (HLC)         │   │    [6 instances]      │
│ +begin()          │   ├───────────────────────┤   ├───────────────────────┤
│ +update()         │   │ -_config              │   │ -_controller_mode     │
│ +processCommands()│   │ -_status              │   │ -_pid_config          │
│ +processVent...() │   │ -_outputs             │   │ -_signal_generator    │
│                   │   │ -_muxRouter*          │   ├───────────────────────┤
│ Parses: VO,VS,RR, │   ├───────────────────────┤   │ +execute()            │
│ IE,VT,FI,PI,PE... │   │ +begin(muxRouter)     │   │ +setControllerMode()  │
└───────────────────┘   │ +start() / +stop()    │   │ +setValveSignal()     │
                        │ +update(meas, time)   │   │ +generateSignal()     │
                        │ +getOutputs()         │   └───────────┬───────────┘
                        │ +setConfig()          │               │
                        └───────────┬───────────┘               │
                                    │                           │
                                    ▼                           ▼
                        ┌───────────────────────────────────────────────────┐
                        │                  SerialMuxRouter                  │
                        ├───────────────────────────────────────────────────┤
                        │ -_serial (Serial1*)                               │
                        │ -_current[6], _blowerRPM[6], _actualFlow[6]...    │
                        ├───────────────────────────────────────────────────┤
                        │ +begin()                                          │
                        │ +update()           // Async RX processing        │
                        │ +sendSetFlow(ch, value)                           │
                        │ +sendSetPressure(ch, value)                       │
                        │ +sendCommand(ch, cmd, value)                      │
                        │ +getCurrent(ch) / +isCurrentStale(ch)             │
                        └───────────────────────────────────────────────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        │                           │                           │
        ▼                           ▼                           ▼
┌───────────────────┐   ┌───────────────────┐   ┌───────────────────────┐
│   SensorReader    │   │   ImageRenderer   │   │   PMixerWiFiServer    │
├───────────────────┤   ├───────────────────┤   ├───────────────────────┤
│ -_wire (TwoWire*) │   │ -tft (TFT_eSPI&)  │   │ -_ssid, _password     │
│ -_hasSFM3505      │   │ -labelPos, etc.   │   │ -_flowHistory[]       │
│ -_hasABP2         │   ├───────────────────┤   ├───────────────────────┤
├───────────────────┤   │ +begin()          │   │ +start() / +stop()    │
│ +initialize()     │   │ +drawStatusField()│   │ +handleClient()       │
│ +readSFM3505...() │   │ +drawFlow()       │   │ +updateFlow()         │
│ +readABP2...()    │   │ +drawPressure()   │   │ +addDataPoint()       │
│ +readABPD...()    │   │ +drawWiFiAPIP()   │   │ +generateHtmlPage()   │
└───────────────────┘   └───────────────────┘   └───────────────────────┘

┌───────────────────┐
│    FDO2_Sensor    │
├───────────────────┤
│ -_serial (Serial2)│
├───────────────────┤
│ +begin()          │
│ +startMeas...()   │
│ +getAsyncResult() │
│ +convertTo...()   │
└───────────────────┘
```

### Data Structures

```
┌─────────────────────────┐   ┌─────────────────────────┐   ┌─────────────────────────┐
│   VentilatorConfig      │   │ VentilatorMeasurements  │   │   VentilatorOutputs     │
├─────────────────────────┤   ├─────────────────────────┤   ├─────────────────────────┤
│ respRate (BPM)          │   │ airwayPressure_mbar     │   │ airValveFlow_slm        │
│ ieRatio                 │   │ inspFlow_slm            │   │ o2ValveFlow_slm         │
│ inspPauseFraction       │   │ expFlow_slm             │   │ expValveSetpoint        │
│ insp1/2Fraction         │   │ deliveredO2_percent     │   │ expValveAsPressure      │
│ expNonTrig/SyncFraction │   └─────────────────────────┘   │ expValveClosed          │
│ tidalVolume_mL          │                                 └─────────────────────────┘
│ maxInspFlow_slm         │   ┌─────────────────────────┐
│ totalFlow_slm           │   │   VentilatorStatus      │
│ maxPressure_mbar        │   ├─────────────────────────┤
│ peep_mbar               │   │ state (enum)            │
│ targetFiO2              │   │ breathCount             │
│ biasFlow_slm            │   │ cycleProgress (0-1)     │
│ flowTrigger_slm         │   │ calculatedFlow_slm      │
│ pressureTrigger_mbar    │   │ peakPressure_mbar       │
│ triggerEnabled          │   │ plateauPressure_mbar    │
│ highPressureAlarm_mbar  │   │ measuredVt_mL           │
│ lowPressureAlarm_mbar   │   │ triggered (bool)        │
│ apneaTime_s             │   └─────────────────────────┘
└─────────────────────────┘
```

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              DATA FLOW                                      │
└─────────────────────────────────────────────────────────────────────────────┘

                    SENSORS                          CONTROL                      ACTUATORS
                    ───────                          ───────                      ─────────

 ┌──────────────┐                              ┌─────────────────┐
 │   SFM3505    │─── Air Flow (slm) ──────────►│                 │
 │  Flow Sensor │─── O2 Flow (slm) ───────────►│                 │
 └──────────────┘                              │                 │
                                               │   Ventilator    │         ┌──────────────┐
 ┌──────────────┐                              │   Controller    │── F ───►│  Air Valve   │
 │     ABPD     │─── Airway Pressure ─────────►│     (HLC)       │         │   (MUX 1)    │
 │   Pressure   │    (mbar)                    │                 │         └──────────────┘
 └──────────────┘                              │                 │
                                               │  State Machine  │         ┌──────────────┐
 ┌──────────────┐                              │  ┌───────────┐  │── F ───►│   O2 Valve   │
 │     FDO2     │─── O2 Partial Pressure ─────►│  │  INSP1    │  │         │   (MUX 2)    │
 │   O2 Sensor  │    (hPa)                     │  │  INSP2    │  │         └──────────────┘
 └──────────────┘                              │  │  PAUSE    │  │
                                               │  │  EXP_NT   │  │         ┌──────────────┐
 ┌──────────────┐                              │  │  EXP_SW   │  │── P ───►│  Exp Valve   │
 │     ABP2     │─── Supply Pressure ─────────►│  └───────────┘  │         │   (MUX 3)    │
 │   Pressure   │    (bar)                     │                 │         └──────────────┘
 └──────────────┘                              └────────┬────────┘
                                                       │
                                                       │ Status
                                                       ▼
                               ┌────────────────────────────────────────────┐
                               │               USER INTERFACES              │
                               ├────────────────────────────────────────────┤
                               │                                            │
                               │  ┌──────────────┐    ┌──────────────────┐  │
                               │  │     TFT      │    │   WiFi Server    │  │
                               │  │   Display    │    │   (WebSocket)    │  │
                               │  │   170x320    │    │   http://IP/     │  │
                               │  └──────────────┘    └──────────────────┘  │
                               │                                            │
                               │  ┌──────────────────────────────────────┐  │
                               │  │          Serial Console              │  │
                               │  │    Commands: VO, VS, RR, VT, FI...   │  │
                               │  └──────────────────────────────────────┘  │
                               │                                            │
                               └────────────────────────────────────────────┘
```

### Timing Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           TIMING LOOPS                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  CONTROL LOOP (control_interval = 10ms = 100Hz)                             │
│  ├── Read SFM3505 Flow Sensors                                              │
│  ├── Read ABPD Pressure Sensor                                              │
│  ├── Update FDO2 (async state machine, 2Hz actual)                          │
│  ├── VentilatorController.update() ─► State machine + outputs               │
│  ├── Apply outputs to MUX channels (if ventilator running)                  │
│  ├── Execute ActuatorControl for all channels (if NOT ventilator)           │
│  └── WiFi data buffer update (for high-speed web graphs)                    │
│                                                                             │
│  GUI/SERIAL LOOP (delta_t = 100ms = 10Hz)                                   │
│  ├── outputData() ─► Serial console output (mode-dependent)                 │
│  └── wifiServer.update*() ─► Push values to web interface                   │
│                                                                             │
│  UI LOOP (SET_UI_UPDATE_TIME = 100ms)                                       │
│  ├── Update TFT display values                                              │
│  └── drawFlow(), drawPressure(), drawValveCtrlSignal()...                   │
│                                                                             │
│  ABP2 PRESSURE LOOP (PressSamplTime = 100ms = 10Hz)                         │
│  └── Async I2C: Send command → Wait → Read result                           │
│                                                                             │
│  CONTINUOUS (every loop iteration)                                          │
│  ├── muxRouter.update() ─► Process async Serial1 RX data                    │
│  ├── parser.update() ─► Check for serial commands                           │
│  ├── interactionKey1 ─► Button state machine                                │
│  └── wifiServer.handleClient() ─► HTTP requests                             │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## LCD Display Layout (170×320 Portrait)

```
┌──────────────────────────────────────────────────────────────────┐
│                        TFT DISPLAY                               │
│                      170 x 320 pixels                            │
│                      Portrait Mode                               │
└──────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  ┌────────────────────────────────────────────────────────────┐   │ y=0
│  │  P-Mixer                                       v0.9.1      │   │
│  └────────────────────────────────────────────────────────────┘   │ y=30
│                                                                    │
│  ┌─────────────── Status ─────────────────────────────────────┐   │ y=40
│  │                                                            │   │
│  │  Flow: 25.340 slm Air                                      │   │ y=60
│  │                                                            │   │
│  │  HP: 2.5 LP: 12.3 25.1 C                                   │   │ y=80
│  │                                                            │   │
│  │  Valve: 45.00                                              │   │ y=100
│  │                                                            │   │
│  │  Mode: Valve Set M0                                        │   │ y=120
│  │                                                            │   │
│  │  Current: 0.245 A                                          │   │ y=140
│  │                                                            │   │
│  │  pO2: 212.5 hPa                                            │   │ y=160
│  │                                                            │   │
│  └────────────────────────────────────────────────────────────┘   │ y=180
│                                                                    │
│                                                                    │
│                                                                    │
│                                                                    │
│  ┌─────────────── WiFi ───────────────────────────────────────┐   │ y=240
│  │                                                            │   │
│  │  IP: 192.168.4.1                                           │   │ y=260
│  │                                                            │   │
│  │  SSID: ParallelMixer                                       │   │ y=280
│  │                                                            │   │
│  │  Long press: enable                                        │   │ y=300
│  │                                                            │   │
│  └────────────────────────────────────────────────────────────┘   │ y=320
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
     x=5                                                       x=165

Legend:
┌────────────────────────────────────────────────────────────────────┐
│  Background: TFT_LOGOBACKGROUND (0x85BA - Light Blue)              │
│  Text:       TFT_DEEPBLUE (0x1A6F - Dark Blue)                     │
│  Rectangles: White rounded corners (10px radius)                   │
│  Fonts:      FSS9 (status), FSSB12 (header)                        │
└────────────────────────────────────────────────────────────────────┘
```

### Display Fields

| Field       | Position      | Content Example                    |
|-------------|---------------|------------------------------------|
| Label       | (10, 5)       | "P-Mixer"                          |
| Version     | (110, 12)     | "v0.9.1"                           |
| Flow        | (10, 60)      | "25.340 slm Air"                   |
| Pressure    | (10, 80)      | "HP: 2.5 LP: 12.3 25.1 C"          |
| Valve       | (10, 100)     | "45.00"                            |
| Mode        | (10, 120)     | "Valve Set M0" / "Sine M1"         |
| Current     | (10, 140)     | "0.245 A"                          |
| pO2         | (10, 160)     | "212.5 hPa"                        |
| WiFi IP     | (10, 260)     | "192.168.4.1" / "WiFi OFF"         |
| WiFi SSID   | (10, 280)     | "ParallelMixer" / "No SSID"        |
| WiFi Prompt | (10, 300)     | "Long press: enable/disable"       |

---

## Web Interface Layout

```
┌────────────────────────────────────────────────────────────────────────────┐
│                        WEB INTERFACE (http://192.168.4.1)                  │
│                             P-Mixer Monitor                                │
└────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────┐
│                                                                            │
│  ┌──────────────────────────────────────────────────────────────────────┐ │
│  │                    P-Mixer Monitor  ●                                │ │
│  │                                    (blink)                           │ │
│  └──────────────────────────────────────────────────────────────────────┘ │
│                                                                            │
│  ┌──────────────────────────────────────────────────────────────────────┐ │
│  │                     Mode: Valve Set M0                               │ │
│  │                     (green background)                               │ │
│  └──────────────────────────────────────────────────────────────────────┘ │
│                                                                            │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐     │
│  │    Flow      │ │   Pressure   │ │ Low Pressure │ │ Temperature  │     │
│  │   25.34      │ │     2.50     │ │    12.30     │ │   25.1 °C    │     │
│  │  (blue)      │ │   (blue)     │ │   (blue)     │ │   (blue)     │     │
│  └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘     │
│                                                                            │
│  ┌──────────────┐ ┌──────────────┐                                        │
│  │Valve Signal  │ │   Current    │                                        │
│  │   45.00      │ │  0.245 A     │                                        │
│  │  (blue)      │ │   (blue)     │                                        │
│  └──────────────┘ └──────────────┘                                        │
│                                                                            │
│  ┌──────────────────────────────────────────────────────────────────────┐ │
│  │                       Real-time Data                                 │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐ │ │
│  │  │                                                                 │ │ │
│  │  │  Flow ────────────────────────────────────────── (Blue)         │ │ │
│  │  │  Pressure ────────────────────────────────────── (Red)          │ │ │
│  │  │  Low Pressure ────────────────────────────────── (Pink)         │ │ │
│  │  │  Temperature ─────────────────────────────────── (Purple)       │ │ │
│  │  │  Valve Signal ────────────────────────────────── (Cyan)         │ │ │
│  │  │  Current ─────────────────────────────────────── (Orange)       │ │ │
│  │  │                                                                 │ │ │
│  │  │  ▲                    ╱╲      ╱╲                                │ │ │
│  │  │  │    ╱╲    ╱╲      ╱  ╲    ╱  ╲                               │ │ │
│  │  │  │  ╱    ╲╱    ╲  ╱    ╲  ╱    ╲                               │ │ │
│  │  │  │╱             ╲╱      ╲╱      ╲                               │ │ │
│  │  │  └─────────────────────────────────────────────────► Time (s)   │ │ │
│  │  │      10      20      30      40      50      60                 │ │ │
│  │  └─────────────────────────────────────────────────────────────────┘ │ │
│  └──────────────────────────────────────────────────────────────────────┘ │
│                                                                            │
│         ┌─────────────────┐         ┌─────────────────┐                   │
│         │    Save Data    │         │   Clear Graph   │                   │
│         │     (blue)      │         │     (blue)      │                   │
│         └─────────────────┘         └─────────────────┘                   │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘

API Endpoints:
┌───────────────────────────────────────────────────────────────────────────┐
│  GET /           → HTML page with Chart.js visualization                 │
│  GET /data       → JSON: current values (single sample)                  │
│  GET /history    → JSON: historical arrays (last 100 samples)            │
│  GET /dataBuffer → JSON: buffered samples since last fetch (cleared)     │
└───────────────────────────────────────────────────────────────────────────┘

Update Rate: 200ms polling interval (5 Hz), displays up to 512 points
Buffer: Up to 4000 samples accumulated between fetches
```

### Web Data Format

```json
// GET /data (single sample)
{
  "flow": 25.34,
  "pressure": 2.50,
  "valve": 45.00,
  "current": 0.245,
  "lowPressure": 12.30,
  "temperature": 25.1,
  "mode": "Valve Set M0",
  "timestamp": 123456789
}

// GET /dataBuffer (batch of samples)
{
  "count": 50,
  "mode": "Valve Set M0",
  "timestamps": [123456700, 123456710, ...],
  "flow": [25.32, 25.34, ...],
  "pressure": [2.49, 2.50, ...],
  "lowPressure": [12.28, 12.30, ...],
  "temperature": [25.1, 25.1, ...],
  "valve": [44.98, 45.00, ...],
  "current": [0.244, 0.245, ...]
}
```

---

## Architecture: Two-Layer Control

```
┌─────────────────────────────────────────────────────────────────┐
│                    HLC (High Level Controller)                  │
│                                                                 │
│  • Breathing cycle state machine (INSP1, INSP2, PAUSE, EXP...)  │
│  • Timing management (RR, I:E, phase fractions)                 │
│  • Setpoint calculation (target flows, pressures)               │
│  • Gas mixing (FiO2 → O2/Air flow split)                        │
│  • Patient trigger detection                                    │
│  • Alarm monitoring                                             │
│                                                                 │
│  Outputs: Target setpoints (flow_O2, flow_Air, pressure_Exp)    │
└─────────────────────────┬───────────────────────────────────────┘
                          │ Setpoints
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                    LLC (Low Level Controller)                   │
│                         [Future Layer]                          │
│                                                                 │
│  • PID control loops for each actuator                          │
│  • Real-time sensor feedback processing                         │
│  • Actuator command generation                                  │
│  • Safety limits and rate limiting                              │
│  • Fault detection at actuator level                            │
│                                                                 │
│  Inputs: Setpoints from HLC, sensor measurements                │
│  Outputs: Actuator commands (valve positions, motor speeds)     │
└─────────────────────────────────────────────────────────────────┘
```

**Current scope**: This document describes the **HLC** layer. The LLC will be added later to handle low-level actuator control with PID feedback loops.

## Actuator Mapping

| MUX Channel | Actuator | Function |
|-------------|----------|----------|
| 0 | Direct | Direct control (legacy) |
| 1 | AirValve | Controls air flow during inspiration |
| 2 | O2Valve | Controls O2 flow during inspiration |
| 3 | ExpValve | Controls expiration / PEEP pressure |
| 4 | NC | Not connected |
| 5 | Blower | Blower control (if used) |

---

## State Machine

### States

```
┌─────────────────────────────────────────────────────────────┐
│                        VENT_OFF                             │
│                   (V OFF stops ventilator)                  │
└─────────────────────────┬───────────────────────────────────┘
                          │ V ON (start)
┌─────────────────────────▼───────────────────────────────────┐
│                    VENT_INSP_PHASE1                         │
│  • Deliver O2 + Air at calculated flows                     │
│  • Exp valve CLOSED                                         │
│  • Pressure controlled by inspiratory flow only             │
│  • Reduce flow if pressure > maxPressure                    │
└─────────────────────────┬───────────────────────────────────┘
                          │ time >= insp1Time
┌─────────────────────────▼───────────────────────────────────┐
│                    VENT_INSP_PHASE2 (optional)              │
│  • Continue gas delivery                                    │
│  • Exp valve acts as POP-OFF (pressure relief)              │
│  • Exp valve opens if pressure > maxPressure                │
│  • Skip if insp1Fraction == 1.0                             │
└─────────────────────────┬───────────────────────────────────┘
                          │ time >= insp2Time
┌─────────────────────────▼───────────────────────────────────┐
│                    VENT_INSP_PAUSE                          │
│  • Stop gas flow (Air=0, O2=0)                              │
│  • Hold valves closed                                       │
│  • Measure plateau pressure                                 │
│  • Skip if inspPauseFraction == 0                           │
└─────────────────────────┬───────────────────────────────────┘
                          │ time >= inspPauseTime
┌─────────────────────────▼───────────────────────────────────┐
│                    VENT_EXP_NON_TRIG                        │
│  • Patient exhales passively                                │
│  • Exp valve controls PEEP pressure                         │
│  • NO patient triggering allowed (anti self-trigger)        │
└─────────────────────────┬───────────────────────────────────┘
                          │ time >= expNonTrigTime
┌─────────────────────────▼───────────────────────────────────┐
│                    VENT_EXP_SYNC_WAIT (optional)            │
│  • Continue PEEP control                                    │
│  • Patient trigger detection ACTIVE                         │
│  • Trigger: flow > threshold OR pressure < (PEEP-threshold) │
│  • Skip if triggerEnabled == false                          │
├─────────────────────────┬───────────────────────────────────┤
│  Trigger detected       │ Timeout (no trigger)              │
└─────────────────────────┴───────────────────────────────────┘
                          │
                          ▼
                   (New breath cycle → INSP_PHASE1)
```

### State Differences: Insp1 vs Insp2

| Aspect | INSP_PHASE1 | INSP_PHASE2 |
|--------|-------------|-------------|
| Exp Valve | Fully CLOSED | Acts as pop-off valve |
| Pressure Control | Flow reduction only | Flow + exp valve opening |
| Use Case | Normal inspiration | Pressure limiting with relief |

---

## Configuration Parameters

### Timing Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `respRate` | float | 12.0 | Respiratory rate (breaths per minute) |
| `ieRatio` | float | 0.5 | I:E ratio as decimal (0.5 = 1:2) |
| `inspPauseFraction` | float | 0.1 | Pause as fraction of total insp time (0-0.5) |
| `insp1Fraction` | float | 1.0 | Phase 1 as fraction of active insp (0-1) |
| `insp2Fraction` | float | 0.0 | Phase 2 as fraction of active insp (default 0 = disabled) |
| `expNonTrigFraction` | float | 0.5 | Non-triggerable fraction of expiration |
| `expSyncFraction` | float | 0.0 | Sync window fraction (default 0 = disabled) |

### Volume/Flow Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tidalVolume_mL` | float | 500.0 | Target tidal volume in mL |
| `maxInspFlow_slm` | float | 60.0 | Maximum inspiratory flow (L/min) |
| `totalFlow_slm` | float | 30.0 | Direct flow setting (if not using Vt) |
| `useVolumeControl` | bool | true | true = calculate flow from Vt |

### Pressure Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `maxPressure_mbar` | float | 30.0 | Peak inspiratory pressure limit |
| `peep_mbar` | float | 5.0 | PEEP pressure during expiration |
| `pressureRampTime_ms` | float | 100 | Pressure rise time (comfort) |

### Gas Mixing

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `targetFiO2` | float | 0.21 | Target O2 fraction (0.21-1.0) |

### Trigger Settings (SIMV modes)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `biasFlow_slm` | float | 2.0 | Continuous flow during expiration |
| `flowTrigger_slm` | float | 2.0 | Flow above bias to trigger |
| `pressureTrigger_mbar` | float | 2.0 | Pressure drop below PEEP to trigger |
| `triggerEnabled` | bool | false | Enable patient triggering |

### Alarm Limits

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `highPressureAlarm_mbar` | float | 40.0 | High pressure alarm threshold |
| `lowPressureAlarm_mbar` | float | 3.0 | Disconnect/low pressure alarm |
| `apneaTime_s` | float | 20.0 | Apnea alarm time |

---

## Timing Calculations

### Cycle Time
```
cycleTime_s = 60 / respRate
```

### I:E Split
```
ieFactor = ieRatio / (1 + ieRatio)
inspTotalTime = cycleTime * ieFactor
expTotalTime = cycleTime - inspTotalTime
```

Example with RR=12, IE=0.5 (1:2):
- Cycle time = 60/12 = 5.0 seconds
- ieFactor = 0.5/1.5 = 0.333
- Insp time = 5.0 × 0.333 = 1.67 seconds
- Exp time = 5.0 - 1.67 = 3.33 seconds

### Inspiration Breakdown
```
inspPauseTime = inspTotalTime × inspPauseFraction
activeInspTime = inspTotalTime - inspPauseTime
insp1Time = activeInspTime × insp1Fraction
insp2Time = activeInspTime × insp2Fraction
```
Note: `insp1Fraction + insp2Fraction` should equal 1.0 (or insp2Fraction=0 to disable phase2)

### Expiration Breakdown
```
expNonTrigTime = expTotalTime × expNonTrigFraction
expSyncTime = expTotalTime × expSyncFraction
```
Note: `expNonTrigFraction + expSyncFraction` should equal 1.0 (or expSyncFraction=0 to disable sync)

---

## Flow Calculations

### Flow from Tidal Volume
```
activeInspTime_min = (insp1Time + insp2Time) / 60
requiredFlow_slm = (tidalVolume_mL / 1000) / activeInspTime_min
```

If `requiredFlow > maxInspFlow`, use `maxInspFlow` (volume will be limited).

Example with Vt=500mL, activeInspTime=1.5s:
- activeInspTime_min = 1.5/60 = 0.025 min
- requiredFlow = 0.5L / 0.025min = 20 L/min

### Gas Mixing (O2 + Air)
```
Air contains 21% O2
Pure O2 contains 100% O2

For target FiO2 with total flow F:
  O2_flow = F × (FiO2 - 0.21) / 0.79
  Air_flow = F - O2_flow
```

Example with F=20 L/min, FiO2=0.40:
- O2_flow = 20 × (0.40 - 0.21) / 0.79 = 20 × 0.24 = 4.8 L/min
- Air_flow = 20 - 4.8 = 15.2 L/min

---

## Control Logic

### During INSP_PHASE1
```
1. Calculate target flow from Vt (or use direct setting)
2. If measured_pressure > maxPressure:
     Reduce flow proportionally (pressure limiting)
3. Calculate O2 and Air flows from total flow and FiO2
4. Set exp valve = CLOSED (0%)
```

### During INSP_PHASE2
```
1. Same flow calculation as INSP1
2. Flow continues at calculated rate
3. Exp valve operates in pressure-control mode:
     - Setpoint = maxPressure
     - Opens if airway pressure exceeds setpoint (pop-off)
```

### During INSP_PAUSE
```
1. Set Air flow = 0
2. Set O2 flow = 0
3. Hold exp valve closed
4. Record plateau pressure for compliance calculation
```

### During EXP_NON_TRIG and EXP_SYNC_WAIT
```
1. Set Air flow = 0
2. Set O2 flow = 0
3. Exp valve in pressure-control mode:
     - Setpoint = PEEP
     - Maintains PEEP throughout expiration
```

### Trigger Detection (EXP_SYNC_WAIT only)
```
During expiration, bias flow runs continuously through the circuit.

Trigger if:
  - inspFlow > (biasFlow + flowTrigger)  (patient pulling above bias)
  OR
  - airwayPressure < (PEEP - pressureTrigger_mbar)  (pressure drop)
```

### Bias Flow
- Continuous low flow during expiration (typically 2-5 L/min)
- Enables sensitive flow triggering
- Mixed O2/Air at current FiO2 setting
- Helps flush circuit and maintain fresh gas

---

## Command Interface

Two-character commands for efficient parameter setting. Parameters can be set at any time but only take effect when ventilator is running (`VO1`).

### Command Reference

| Cmd | Parameter | Unit | Default | Example | Description |
|-----|-----------|------|---------|---------|-------------|
| **Control** |
| `VO` | Vent On/Off | 0/1 | 0 | `VO1` | Start ventilator (1) or stop (0) |
| `VS` | Status | - | - | `VS` | Query ventilator status |
| **Timing** |
| `RR` | Resp Rate | BPM | 12 | `RR15` | Respiratory rate (sets cycle time) |
| `IE` | I:E Ratio | decimal | 0.5 | `IE0.5` | I:E ratio (0.5 = 1:2) |
| `IP` | Insp Pause | fraction | 0.1 | `IP0.1` | Pause as fraction of insp time |
| `I1` | Insp1 Frac | fraction | 1.0 | `I10.8` | Phase1 fraction of active insp |
| `I2` | Insp2 Frac | fraction | 0.0 | `I20.2` | Phase2 fraction (default 0 = disabled) |
| `EN` | Exp NoTrig | fraction | 0.5 | `EN0.5` | Non-triggerable exp fraction |
| `ES` | Exp Sync | fraction | 0.0 | `ES0.3` | Sync window fraction (default 0 = disabled) |
| **Volume/Flow** |
| `VT` | Tidal Vol | mL | 500 | `VT450` | Target tidal volume |
| `MF` | Max Flow | L/min | 60 | `MF40` | Maximum inspiratory flow |
| `TF` | Total Flow | L/min | 30 | `TF25` | Direct flow setting |
| `VC` | Vol Ctrl | 0/1 | 1 | `VC1` | Use Vt calc (1) or direct flow (0) |
| **Pressure** |
| `PI` | PIP | mbar | 30 | `PI25` | Peak inspiratory pressure limit |
| `PE` | PEEP | mbar | 5 | `PE8` | PEEP pressure |
| `PR` | P Ramp | ms | 100 | `PR150` | Pressure rise time |
| **Gas** |
| `FI` | FiO2 | % | 21 | `FI40` | Target O2 percentage (21-100) |
| **Trigger** |
| `TE` | Trig Enable | 0/1 | 0 | `TE1` | Enable patient triggering |
| `BF` | Bias Flow | L/min | 2.0 | `BF3` | Continuous flow during exp (for triggering) |
| `FT` | Flow Trig | L/min | 2.0 | `FT3` | Flow above bias to trigger |
| `PT` | Press Trig | mbar | 2.0 | `PT2` | Pressure drop below PEEP to trigger |
| **Alarms** |
| `AH` | High P Alarm | mbar | 40 | `AH45` | High pressure alarm |
| `AL` | Low P Alarm | mbar | 3 | `AL2` | Low/disconnect alarm |
| `AA` | Apnea Alarm | sec | 20 | `AA15` | Apnea timeout |

### Query Format
Any command without a value queries the current setting:
```
> PI
PI=30.0
> RR
RR=12.0
```

### Example Session
```
> VS
Vent: OFF  State: OFF
RR=12.0 VT=500 IE=0.50 FI=21%
PI=30.0 PE=5.0 MF=60.0

> RR15
OK
> VT450
OK
> FI35
OK
> PI25
OK

> VO1
Ventilator ON

> VS
Vent: ON  State: INSP1
RR=15.0 VT=450 IE=0.50 FI=35%
PI=25.0 PE=5.0 MF=60.0
#=12 PkP=22.3 Vt=448mL

> VO0
Ventilator OFF
```

### Example: Configuring Sub-phases
```
> IP0.15       // Set insp pause to 15% of insp time
OK
> I10.7        // Set insp phase1 to 70% of active insp
OK
> I20.3        // Set insp phase2 to 30% of active insp (enables phase2)
OK
> EN0.6        // Set exp non-trig to 60% of exp time
OK
> ES0.4        // Set exp sync to 40% (enables triggering window)
OK

> IP           // Query current setting
IP=0.15
```

### Timing Calculation Example
With RR=12, IE=0.5 (1:2):
- Cycle = 5000ms
- Insp total = 1667ms
- Exp total = 3333ms

**Default fractions:**
- inspPause = 1667 × 0.1 = 167ms
- activeInsp = 1667 - 167 = 1500ms
- insp1 = 1500 × 1.0 = 1500ms
- insp2 = 1500 × 0.0 = 0ms (disabled)
- expNonTrig = 3333 × 0.5 = 1667ms
- expSync = 3333 × 0.0 = 0ms (disabled)

**With I1=0.7, I2=0.3, ES=0.4:**
- insp1 = 1500 × 0.7 = 1050ms
- insp2 = 1500 × 0.3 = 450ms
- expNonTrig = 3333 × 0.6 = 2000ms
- expSync = 3333 × 0.4 = 1333ms (trigger window enabled)

### Implementation Notes
- Parameters persist across ON/OFF cycles
- All parameters can be changed while running (take effect next breath)
- `VO1` stops any running function generators on MUX channels 1-3 (Air, O2, Exp)
- Invalid values are rejected with error message

---

## Data Structures

### VentilatorConfig
```cpp
struct VentilatorConfig {
    // Timing
    float respRate;             // BPM (determines cycle time)
    float ieRatio;              // I:E as decimal (splits insp/exp)
    float inspPauseFraction;    // 0-0.5 (default 0.1)
    float insp1Fraction;        // 0-1 (default 1.0)
    float insp2Fraction;        // 0-1 (default 0 = disabled)
    float expNonTrigFraction;   // 0-1 (default 0.5)
    float expSyncFraction;      // 0-1 (default 0 = disabled)

    // Volume/Flow
    float tidalVolume_mL;
    float maxInspFlow_slm;
    float totalFlow_slm;
    bool useVolumeControl;

    // Pressure
    float maxPressure_mbar;
    float peep_mbar;
    float pressureRampTime_ms;

    // Gas
    float targetFiO2;

    // Trigger
    float biasFlow_slm;
    float flowTrigger_slm;
    float pressureTrigger_mbar;
    bool triggerEnabled;

    // Alarms
    float highPressureAlarm_mbar;
    float lowPressureAlarm_mbar;
    float apneaTime_s;
};
```

### VentilatorState (enum)
```cpp
enum VentilatorState {
    VENT_OFF = 0,
    VENT_INSP_PHASE1,
    VENT_INSP_PHASE2,
    VENT_INSP_PAUSE,
    VENT_EXP_NON_TRIG,
    VENT_EXP_SYNC_WAIT
};
```

### VentilatorOutputs
```cpp
struct VentilatorOutputs {
    float airValveFlow_slm;     // → MUX channel 1
    float o2ValveFlow_slm;      // → MUX channel 2
    float expValvePosition;     // → MUX channel 3
    bool expValveAsPressure;    // true = pressure setpoint mode
};
```

### VentilatorStatus
```cpp
struct VentilatorStatus {
    VentilatorState state;
    uint32_t breathCount;
    float cycleProgress;        // 0.0 - 1.0
    float calculatedFlow_slm;
    float actualSetFlow_slm;
    float peakPressure_mbar;
    float plateauPressure_mbar;
    float measuredVt_mL;
    bool pressureLimited;
    bool flowLimited;
    bool triggered;
};
```

---

## Alarm Flags

| Flag | Bit | Condition |
|------|-----|-----------|
| ALARM_HIGH_PRESSURE | 0x01 | Pressure > highPressureAlarm |
| ALARM_LOW_PRESSURE | 0x02 | Pressure < lowPressureAlarm during insp |
| ALARM_APNEA | 0x04 | No breath for apneaTime seconds |
| ALARM_LOW_VT | 0x08 | Delivered Vt < 80% of set |
| ALARM_HIGH_FIO2 | 0x10 | Measured FiO2 > set + 5% |
| ALARM_LOW_FIO2 | 0x20 | Measured FiO2 < set - 5% |

---

## Integration with Main Loop

```cpp
// In main.cpp loop()

if (ventilator.isRunning()) {
    // Gather measurements
    VentilatorMeasurements meas;
    meas.airwayPressure_mbar = sensorData_bus0.abpd_pressure;
    meas.inspFlow_slm = sensorData_bus0.sfm3505_air_flow;
    meas.deliveredO2_percent = fdo2Data.oxygenPartialPressure_hPa; // Convert as needed

    // Update ventilator state machine
    ventilator.update(meas, micros());

    // Get outputs and apply to actuators
    VentilatorOutputs out = ventilator.getOutputs();

    // Channel 1: Air valve (flow control)
    muxRouter.sendSetFlow(1, out.airValveFlow_slm);

    // Channel 2: O2 valve (flow control)
    muxRouter.sendSetFlow(2, out.o2ValveFlow_slm);

    // Channel 3: Exp valve (pressure or position control)
    if (out.expValveAsPressure) {
        muxRouter.sendSetPressure(3, out.expValvePosition);
    } else {
        muxRouter.sendCommand(3, 'V', out.expValvePosition);
    }
}
```

---

## Future Enhancements

### Display Page Toggle
- Boot button toggles between Status page and Ventilator Settings page
- Ventilator page shows: RR, Vt, FiO2, PIP, PEEP, current state, breath count

### Web Interface
- Second page on WiFi server for ventilator settings
- Real-time waveform display (pressure, flow, volume)

### PRVC Mode
- Breath-to-breath adjustment of pressure to achieve target Vt
- Requires volume measurement feedback

### Pressure Support (PS) Mode
- Patient-triggered breaths with pressure assist
- Cycle-off based on flow decay (e.g., 25% of peak flow)

---

## File Structure

```
include/
  VentilatorController.hpp    - Class declaration and structures

src/
  VentilatorController.cpp    - Implementation

Modified files:
  CommandParser.cpp           - Add 'V' command handling
  main.cpp                    - Integration in loop()
  ImageRenderer.cpp           - Ventilator display page (future)
```
