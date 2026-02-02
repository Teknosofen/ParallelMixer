# ParallelMixer Ventilator System - Presentation Outline

## Target Audience
Engineering management who are engineers but not experts in ventilator/respiratory systems.

## Presentation Tips
- **Duration:** ~20-25 minutes for main content, 5-10 for questions
- **For non-experts:** Emphasize the *why* before the *how*
- **Visuals:** Use the ASCII diagrams as basis for cleaner graphics
- **Demo:** If possible, show live system with WiFi dashboard
- **Backup slides:** Keep detailed technical specs as appendix

---

## Slide 1: Title

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│              PARALLELMIXER VENTILATOR SYSTEM                   │
│                                                                │
│              Precision Gas Mixing & Delivery                   │
│                                                                │
│              [Date] | [Your Name/Team]                         │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

---

## Slide 2: Agenda

- Problem Statement & Goals
- System Overview
- Hardware Architecture
- Software Architecture
- Ventilator Control Design
- User Interfaces
- Current Status & Roadmap

---

## Slide 3: Problem Statement

**What problem are we solving?**
- Precise control of medical gas delivery (O2 + Air)
- Automated breathing cycle management
- Real-time pressure and flow monitoring
- Safe, configurable ventilation parameters

**Key Requirements:**
- Adjustable respiratory rate, tidal volume, FiO2
- Pressure limiting and PEEP control
- Patient trigger detection (SIMV capability)
- Real-time monitoring and alarms

---

## Slide 4: System Overview (High-Level Block Diagram)

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   SENSORS   │────►│ CONTROLLER  │────►│  ACTUATORS  │
│             │     │  (ESP32-S3) │     │             │
│ • Flow      │     │             │     │ • Air Valve │
│ • Pressure  │     │ State       │     │ • O2 Valve  │
│ • O2        │     │ Machine     │     │ • Exp Valve │
└─────────────┘     └──────┬──────┘     └─────────────┘
                           │
                    ┌──────▼──────┐
                    │    USER     │
                    │ INTERFACES  │
                    │ • Display   │
                    │ • WiFi/Web  │
                    │ • Serial    │
                    └─────────────┘
```

**Key message:** Closed-loop control system with multiple user interfaces for monitoring and configuration.

---

## Slide 5: Hardware Platform

**LilyGo T-Display S3 (ESP32-S3)**

| Feature | Specification |
|---------|---------------|
| MCU | ESP32-S3 (dual-core, 240MHz) |
| Display | 170×320 TFT (ST7789) |
| Connectivity | WiFi, USB-C, 2× UART |
| I2C Buses | 2 independent buses |

**Why this platform?**
- Compact, integrated display
- Sufficient I/O for all sensors
- WiFi for remote monitoring
- Low cost, readily available

---

## Slide 6: Sensor Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    I2C BUS 0 (1 MHz) - GPIO43/44            │
├─────────────────────────────────────────────────────────────┤
│  SFM3505 (0x2E)  │  Flow measurement      │  100 Hz        │
│  ABP2 (0x28)     │  Supply pressure       │  10-100 Hz     │
│  ABPD (0x18)     │  Airway pressure       │  10-100 Hz     │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                    I2C BUS 1 (1 MHz) - GPIO10/11            │
├─────────────────────────────────────────────────────────────┤
│  SFM3505 (0x2E)  │  Flow measurement #2   │  100 Hz        │
│  ABP2 (0x28)     │  Pressure #2           │  10-100 Hz     │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                    SERIAL2 (19200 baud)                     │
├─────────────────────────────────────────────────────────────┤
│  FDO2 Pyroscience │  O2 partial pressure  │  2 Hz          │
└─────────────────────────────────────────────────────────────┘
```

**Key message:** Dual I2C buses allow sensors with same address; high-speed flow sensing (100 Hz) enables responsive control.

---

## Slide 7: Actuator Architecture (Serial MUX)

```
                    ┌─────────────────┐
     ESP32-S3       │   Serial MUX    │      Valve Controllers
    ──────────►     │   (460800 bd)   │     ──────────────────
     GPIO17 TX      │                 │
     GPIO18 RX      │  Ch1 ──────────►│  Air Valve (flow ctrl)
                    │  Ch2 ──────────►│  O2 Valve (flow ctrl)
                    │  Ch3 ──────────►│  Exp Valve (PEEP ctrl)
                    │  Ch5 ──────────►│  Blower (optional)
                    └─────────────────┘
```

**Benefits of MUX approach:**
- Single UART controls multiple actuators
- Each valve has independent PID controller
- Bidirectional: setpoints out, feedback in

---

## Slide 8: Software Architecture - Two-Layer Control

```
┌─────────────────────────────────────────────────────────────┐
│              HLC (High Level Controller)                     │
│                                                             │
│   • Breathing cycle state machine                           │
│   • Timing: RR, I:E ratio, phase fractions                  │
│   • Flow calculation from tidal volume                      │
│   • FiO2 → O2/Air split calculation                         │
│   • Patient trigger detection                               │
│   • Alarm monitoring                                        │
│                                                             │
│   OUTPUT: Flow setpoints (L/min), Pressure setpoints (mbar) │
└───────────────────────────┬─────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              LLC (Low Level Controller) - Future            │
│                                                             │
│   • PID loops per actuator                                  │
│   • Real-time feedback processing                           │
│   • Safety limits and rate limiting                         │
└─────────────────────────────────────────────────────────────┘
```

**Key message:** Clean separation between "what to deliver" (HLC) and "how to deliver it" (LLC).

---

## Slide 9: Ventilator State Machine

```
        ┌─────────┐
        │   OFF   │◄──── VO0 (stop)
        └────┬────┘
             │ VO1 (start)
             ▼
    ┌────────────────┐
    │   INSP_PHASE1  │  Gas delivery, exp valve CLOSED
    └───────┬────────┘
            ▼
    ┌────────────────┐
    │   INSP_PHASE2  │  (optional) Exp valve as pop-off
    └───────┬────────┘
            ▼
    ┌────────────────┐
    │   INSP_PAUSE   │  Hold for plateau pressure measurement
    └───────┬────────┘
            ▼
    ┌────────────────┐
    │  EXP_NON_TRIG  │  Passive exhale, no triggering allowed
    └───────┬────────┘
            ▼
    ┌────────────────┐
    │  EXP_SYNC_WAIT │  (optional) Patient trigger window
    └───────┬────────┘
            │
            └────────────► Next breath cycle
```

**Key message:** Each state has specific valve configurations and timing rules.

---

## Slide 10: Key Ventilator Parameters

| Category | Parameter | Range | Default |
|----------|-----------|-------|---------|
| **Timing** | Respiratory Rate | 4-40 BPM | 12 |
| | I:E Ratio | 1:1 to 1:4 | 1:2 |
| **Volume** | Tidal Volume | 200-1000 mL | 500 |
| | Max Flow | 10-60 L/min | 60 |
| **Pressure** | PIP Limit | 15-50 mbar | 30 |
| | PEEP | 0-20 mbar | 5 |
| **Gas** | FiO2 | 21-100% | 21% |
| **Trigger** | Flow Trigger | 0.5-5 L/min | 2 |

**Key message:** All parameters are configurable via serial commands or future GUI.

---

## Slide 11: Gas Mixing Calculation

**Problem:** Achieve target FiO2 using Air (21% O2) and pure O2

**Formula:**
```
Total Flow = Tidal Volume / Active Inspiration Time

O2 Flow  = Total × (FiO2 - 0.21) / 0.79
Air Flow = Total - O2 Flow
```

**Example:** FiO2 = 40%, Total Flow = 30 L/min
- O2 Flow = 30 × (0.40 - 0.21) / 0.79 = **7.2 L/min**
- Air Flow = 30 - 7.2 = **22.8 L/min**

**Key message:** Simple linear mixing equation ensures accurate FiO2 delivery.

---

## Slide 12: User Interface - TFT Display

```
┌─────────────────────────────────┐
│  P-Mixer              v0.9.1   │
├─────────────────────────────────┤
│  Flow: 25.340 slm Air          │  ← Bus 0 flow
│  Flow2: 12.150 slm Air         │  ← Bus 1 flow
│  HP: 2.5  LP: 12.3  25°C       │
│  Valve: 45.00%                 │
│  Mode: Valve Set M0            │
│  Current: 0.245 A              │
│  pO2: 212.5 hPa                │
├─────────────────────────────────┤
│  WiFi: 192.168.4.1             │
│  SSID: ParallelMixer           │
│  Long press: enable            │
└─────────────────────────────────┘
    170 × 320 pixels (Portrait)
```

**Features:**
- Dual I2C bus flow display (Bus 0 and Bus 1)
- Real-time sensor values
- Current operating mode
- WiFi status and connection info

**Physical Buttons:**
- Key 1 (GPIO14): Long=WiFi On, Short=WiFi Off
- Key 2 (GPIO0): Long=Show Vent Settings, Short=Back

---

## Slide 13: User Interface - Web Dashboard

```
┌─────────────────────────────────────────────────────┐
│           P-Mixer Monitor  ●                        │
├─────────────────────────────────────────────────────┤
│ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐        │
│ │Flow B0 │ │Flow B1 │ │Press B0│ │Press B1│        │
│ │ 25.34  │ │ 12.15  │ │  2.50  │ │  2.48  │        │
│ └────────┘ └────────┘ └────────┘ └────────┘        │
│ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐        │
│ │Low Pres│ │  Temp  │ │ Valve  │ │Current │        │
│ │ 12.30  │ │ 25.1°C │ │ 45.0%  │ │0.245 A │        │
│ └────────┘ └────────┘ └────────┘ └────────┘        │
├─────────────────────────────────────────────────────┤
│    ╱╲      Real-time Chart (Chart.js)              │
│  ╱    ╲     • 6 data channels graphed              │
│ ╱      ╲    • 512 data points visible              │
├─────────────────────────────────────────────────────┤
│  [Save Data]  [Clear Graph]  [Ventilator Settings] │
└─────────────────────────────────────────────────────┘
```

**Features:**
- Dual I2C bus data display (Bus 0 and Bus 1)
- Real-time multi-channel graphs
- Ventilator settings page (separate view)
- Data export to TXT file
- 200ms refresh rate
- Accessible via WiFi AP (192.168.4.1)

---

## Slide 14: Serial Command Interface

**Two-character command protocol for all settings:**

```
Examples:
  RR15     → Set respiratory rate to 15 BPM
  VT450    → Set tidal volume to 450 mL
  FI40     → Set FiO2 to 40%
  VO1      → Start ventilator
  VS       → Query status

Response:
  > VS
  Vent: ON  State: INSP1
  RR=15.0 VT=450 IE=0.50 FI=40%
  PI=30.0 PE=5.0 MF=60.0
  #=12 PkP=22.3 Vt=448mL
```

**Benefits:**
- Simple, human-readable
- Scriptable for automated testing
- Low bandwidth
- Query any parameter without value

---

## Slide 15: Safety Features

| Feature | Implementation |
|---------|----------------|
| **High Pressure Alarm** | Continuous monitoring, configurable threshold (default 40 mbar) |
| **Low Pressure Alarm** | Disconnect detection (default 3 mbar) |
| **Apnea Alarm** | Timeout if no breath detected (default 20s) |
| **Pressure Limiting** | Flow reduction + pop-off valve in INSP_PHASE2 |
| **Anti Self-Trigger** | Non-triggerable expiration phase prevents false triggers |
| **Data Logging** | WiFi export for post-hoc review and analysis |

**Key message:** Multiple layers of safety with configurable thresholds.

---

## Slide 16: Timing Architecture

```
┌────────────────────────────────────────────────────────────┐
│  CONTROL LOOP (100 Hz = 10ms)                              │
│  └─ Sensor reading, state machine, actuator commands       │
├────────────────────────────────────────────────────────────┤
│  GUI LOOP (10 Hz = 100ms)                                  │
│  └─ Display updates, serial output                         │
├────────────────────────────────────────────────────────────┤
│  CONTINUOUS                                                │
│  └─ Serial RX processing, WiFi handling, button detection  │
└────────────────────────────────────────────────────────────┘
```

**Key metrics:**
- Control latency: <10ms
- Display refresh: 100ms
- Web update: 200ms (batched for efficiency)

**Key message:** Fast control loop ensures responsive ventilation; slower loops for UI.

---

## Slide 17: Current Status

| Component | Status |
|-----------|--------|
| Hardware platform | ✅ Complete |
| Sensor integration (SFM3505, ABP2, ABPD, FDO2) | ✅ Complete |
| Serial MUX router | ✅ Complete |
| Actuator control modes (PID, Sine, Step, Sweep) | ✅ Complete |
| TFT display | ✅ Complete |
| WiFi web interface | ✅ Complete |
| Ventilator HLC (state machine) | 🔄 In Progress |
| Ventilator LLC (closed-loop control) | ⬜ Planned |
| Alarm system | ⬜ Planned |
| Clinical validation | ⬜ Future |

---

## Slide 18: Roadmap / Next Steps

### Phase 1 (Current): HLC Implementation
- Complete state machine logic
- Implement full command interface
- Basic alarm detection
- Integration testing

### Phase 2: LLC Integration
- PID tuning for each valve type
- Closed-loop flow control
- Pressure feedback control
- Response time optimization

### Phase 3: Clinical Features
- PRVC mode (pressure-regulated volume control)
- Pressure support mode
- Real-time waveform display on web interface
- Enhanced alarm management

### Phase 4: Productization
- Regulatory documentation
- Clinical validation studies
- Manufacturing design

---

## Slide 19: Demo / Questions

```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│                    LIVE DEMONSTRATION                      │
│                                                            │
│    • Serial command interface (VS, RR, VT commands)        │
│    • TFT display operation                                 │
│    • WiFi web dashboard with real-time graphs              │
│    • Signal generator modes (Sine, Step, Sweep)            │
│                                                            │
├────────────────────────────────────────────────────────────┤
│                                                            │
│                       QUESTIONS?                           │
│                                                            │
│    Contact: [Your Email]                                   │
│    Repository: [Git URL if applicable]                     │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

---

## Slide 20: Appendix - Technical Reference (Backup Slides)

### A1: Complete Pin Assignment Table

| GPIO | Function | Notes |
|------|----------|-------|
| 43 | I2C0 SDA | Flow, pressure sensors |
| 44 | I2C0 SCL | 1 MHz clock |
| 10 | I2C1 SDA | Secondary sensors |
| 11 | I2C1 SCL | |
| 17 | Serial1 TX | MUX router |
| 18 | Serial1 RX | 460800 baud |
| 12 | Serial2 TX | FDO2 sensor |
| 13 | Serial2 RX | 19200 baud |
| 14 | Key 1 | Long=WiFi On, Short=WiFi Off |
| 0 | Key 2 (Boot) | Long=Vent Settings, Short=Back |
| 15 | Display power | Must be HIGH |

### A2: Full Command Reference

| Cmd | Parameter | Unit | Default | Description |
|-----|-----------|------|---------|-------------|
| VO | Vent On/Off | 0/1 | 0 | Start/stop ventilator |
| VS | Status | - | - | Query status |
| RR | Resp Rate | BPM | 12 | Respiratory rate |
| IE | I:E Ratio | decimal | 0.5 | I:E ratio (0.5 = 1:2) |
| VT | Tidal Vol | mL | 500 | Target tidal volume |
| MF | Max Flow | L/min | 60 | Maximum inspiratory flow |
| PI | PIP | mbar | 30 | Peak pressure limit |
| PE | PEEP | mbar | 5 | PEEP pressure |
| FI | FiO2 | % | 21 | Target O2 percentage |
| TE | Trig Enable | 0/1 | 0 | Enable patient triggering |

### A3: API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | HTML dashboard page |
| `/data` | GET | Current values (JSON) |
| `/history` | GET | Last 100 samples (JSON) |
| `/dataBuffer` | GET | Buffered samples since last fetch |

### A4: Data Structures

```cpp
struct VentilatorConfig {
    float respRate;          // BPM
    float ieRatio;           // 0.5 = 1:2
    float tidalVolume_mL;    // mL
    float maxInspFlow_slm;   // L/min
    float maxPressure_mbar;  // mbar
    float peep_mbar;         // mbar
    float targetFiO2;        // 0.21-1.0
    bool triggerEnabled;
    // ... additional fields
};

struct VentilatorOutputs {
    float airValveFlow_slm;   // → MUX Ch 1
    float o2ValveFlow_slm;    // → MUX Ch 2
    float expValveSetpoint;   // → MUX Ch 3
    bool expValveClosed;
};
```

---

## Notes for Presenter

1. **Slide 3 (Problem):** If audience is unfamiliar with ventilators, briefly explain what a ventilator does (delivers controlled breaths to patients who cannot breathe adequately on their own).

2. **Slide 9 (State Machine):** Walk through one complete breath cycle to illustrate the flow.

3. **Slide 11 (Gas Mixing):** This is a key differentiator - precise FiO2 control requires accurate mixing.

4. **Slide 15 (Safety):** Emphasize that safety features are built into the architecture, not bolted on.

5. **Demo:** If doing a live demo, have a fallback (video or screenshots) in case of technical issues.

6. **Questions to anticipate:**
   - "What's the regulatory pathway?" → Research-grade prototype, regulatory TBD
   - "How does this compare to commercial ventilators?" → Focus on modularity and configurability
   - "What's the cost?" → BOM is low (~$50 MCU + sensors), development cost is the investment
