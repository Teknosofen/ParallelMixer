# The One-Mode Ventilator Control Architecture

A modular, setpoint-driven approach

## Overview

This document describes a layered control architecture for a medical ventilator system. A single generic mode architecture can implement all common ventilation modes purely through parameter configuration — there is no mode-switching logic. The architecture consists of:

1. **Layer 1**: Low-Level PID Controllers (Actuator Layer)
2. **Layer 2**: Low-Level Controller (LLC) - Breathing Phase Control
3. **Layer 3**: High-Level Controller (HLC) - Ventilation Mode Control
4. **User Interface Layer**: Parameter Conversion & Clinician Interaction (above HLC)

In addition, **Manoeuvres** (intermittent parameter changes) can be applied from any layer above the target layer to temporarily alter behaviour (e.g. recruitment manoeuvres, breath holds, sigh breaths).

The control philosophy is based on **indirect control through setpoint manipulation** — higher layers do not send explicit commands but instead modify setpoints that trigger automatic responses in lower layers.

**Separation of concerns**: The HLC operates exclusively on pre-calculated absolute state times. It does not interpret clinician-facing parameters such as respiratory rate, I:E ratio, or pause percentage. All such conversions are performed by the **User Interface Layer**, which sits above the HLC and translates user preferences into the deterministic inputs the HLC expects. This keeps the real-time control path simple and testable, while allowing different user interfaces to share the same HLC.

---

## Layer 1: Low-Level PID Controllers (Actuator Layer)

### Purpose
Provide precise, linearized control of individual actuators with sensor scaling and compensation.

### Controllers

Four independent PID controllers (or similar/more advanced controllers):

1. **O2 Flow Controller**
   - Controls oxygen flow rate
   - Handles sensor scaling and linearization for O2 flow sensor
   - Receives flow setpoint from LLC

2. **Air Flow Controller**
   - Controls air flow rate
   - Handles sensor scaling and linearization for air flow sensor
   - Receives flow setpoint from LLC

3. **Expiratory Valve Controller**
   - Controls expiratory pressure/valve position
   - Handles sensor scaling and linearization for pressure sensor
   - Receives pressure setpoint from LLC

4. **Blower Flow Controller**
   - Controls blower flow rate
   - Handles sensor scaling and linearization for blower flow sensor
   - Receives flow setpoint from LLC

### Characteristics
- Each controller operates independently at the highest loop rate
- Handles sensor and actuator non-linearities via **piecewise lookup tables**
- Provides fast, stable control loops
- No knowledge of breathing phases or ventilation modes
- All setpoints received from LLC — outputs go directly to **hardware drivers**

---

## Layer 2: Low-Level Controller (LLC) - Breathing Phase Control

### Purpose
Manage the fundamental breathing cycle alternating between insufflation (inspiration) and exsufflation (expiration).

### Flow–Pressure Control Continuum

Pressure and flow control are **end-points on a continuous scale**. The LLC can operate at any point along this continuum:

1. **Flow control** — pure flow regulation, pressure is a consequence
2. **Pressure-limited flow control** — flow-primary, but flow is reduced when pressure approaches the limit
3. **Flow-limited pressure control** — pressure-primary, but flow is capped at a maximum
4. **Pressure control** — pure pressure regulation, flow is a consequence

The operating point is determined by the combination of `PawSet` (set airway pressure) and `Max Inspiratory Flow` parameters provided by the HLC.

### States

#### Insufflation State (Inspiration)
- Controls airway pressure by inspiratory flow control using FiO₂, PawSet, and max flow limit
- Delivers pressure/flow to the patient
- Maintains target inspiratory pressure given a set flow limitation
- Controls flow using O₂ and Air flow controllers
- The expiratory valve could also be used for pressure limitation during insufflation

#### Exsufflation State (Expiration)
- Controls airway pressure using expiratory valve and bias flow
- Releases pressure/flow from the patient
- Controls pressure using expiratory valve
- Maintains PEEP (baseline pressure)

### Parameters (Set by HLC)

| Parameter | Alias | Description |
|-----------|-------|-------------|
| `FiO2` | | Fraction of inspired oxygen (0.21 - 1.0) |
| `Set Airway Pressure` | `PawSet` | Target pressure during insufflation |
| `Max Inspiratory Flow` | `FlowSet` | Maximum allowed inspiratory flow |
| `Exsufflation Pressure` | (derived from PEEP by HLC) | Target pressure during exsufflation |
| `Bias Flow` | `FlowBias` | Continuous flow maintained during exsufflation |
| `Flow Trigger Threshold` | | Flow change required to detect patient effort |
| `Pressure Trigger Threshold` | | Pressure change required to detect patient effort |

> **Note**: There may be a separate **absolute pressure limit** at LLC level that forces an immediate transition to the exsufflation state, independent of HLC-level pressure limiting. This provides an additional hardware-level safety layer.

### State Transition Mechanisms

The LLC transitions between insufflation and exsufflation based on:

1. **Patient Effort Detection**
   - Monitors flow and pressure against trigger thresholds
   - Patient inspiratory effort → triggers insufflation
   - Patient expiratory effort → triggers exsufflation

2. **Setpoint-Induced Transitions**
   - When HLC modifies setpoints, trigger criteria may suddenly be fulfilled
   - No explicit commands needed — purely reactive behavior
   - Example: HLC sets PEEP = 5 cmH₂O → LLC shifts to exsufflation to reach this pressure
   - Example: HLC sets PEEP = 100 cmH₂O → LLC enters exsufflation mode (used for inspiratory pause lock, see below)

### Communication to HLC

The LLC provides the following signals to the HLC:

- **State change events**: Signals when transitioning between insufflation ↔ exsufflation
- **Measured tidal volume**: Integrated flow delivered during inspiration phase
- **Pressure limit events**: Signals when pressure limiting has occurred

### Control Philosophy

> **No explicit commands from HLC to LLC** - all control happens through setpoint manipulation. The LLC is a purely reactive controller responding to trigger conditions.

---

## Layer 3: High-Level Controller (HLC) - Ventilation Mode Control

### Purpose
Implement complete ventilation modes using an 8-state cycle that coordinates with patient breathing effort.

### State Machine

The HLC operates as an 8-state sequential state machine divided into two phases:

#### Inspiration Sequence (4 states)

1. **Non-triggerable**
   - Fixed inspiration phase
   - Patient triggers are ignored
   - Duration set by timing parameters
   - Delivers mandatory breath

2. **Support**
   - Patient can trigger pressure support
   - If trigger detected → add inspiratory support pressure
   - Allows patient to augment mechanical breath

3. **Synch** (Synchronization)
   - Patient-synchronized state transition
   - If LLC signals exsufflation → HLC advances to next state
   - Allows patient to terminate inspiration
   - **Default duration: 0** (typically skipped unless needed)

4. **Pause** (Inspiratory Pause/Plateau)
   - Holds delivered volume without adding or releasing gas
   - Used for plateau pressure measurement
   - Duration received as absolute time from UI layer

#### Expiration Sequence (4 states)

5. **Non-triggerable**
   - Fixed expiration phase
   - Patient triggers are ignored
   - Duration set by timing parameters
   - Ensures minimum expiration time

6. **Support**
   - Patient can trigger pressure support during expiration
   - If trigger detected → add expiratory support pressure
   - Rare in typical ventilation modes

7. **Synch** (Synchronization)
   - Patient-synchronized state transition
   - If LLC signals insufflation → HLC advances to next state
   - Allows patient to initiate next breath
   - **Default duration: 0** (typically skipped unless needed)

8. **Pause** (Expiratory Pause)
   - Maintains expiratory state
   - Typically used for breath holds or measurements
   - Duration set by timing parameters

### State Transition Logic

States advance based on:

1. **Time elapsed** - Each state has a configured duration
2. **Zero-duration skipping** - States with time = 0 are immediately skipped
3. **Patient synchronization** - During Synch states, LLC state changes trigger HLC state advances:
   - **Insp Synch**: LLC exsufflation signal → advance to Insp Pause
   - **Exp Synch**: LLC insufflation signal → advance to Exp Pause

### Timing Parameters

The HLC state machine operates exclusively on **pre-calculated absolute times** for each of the 8 states. It does **not** interpret or convert user-facing parameters such as respiratory rate, I:E ratio, or pause percentage. Each state simply receives a duration in milliseconds (or seconds) and advances when that time elapses (or when a synchronization event occurs for Synch states, or immediately for zero-duration states).

| HLC State Parameter | Description |
|---------------------|-------------|
| `Time_Insp_NonTrig` | Duration of the mandatory (non-triggerable) inspiration phase |
| `Time_Insp_Support` | Duration of the inspiration support window |
| `Time_Insp_Synch` | Maximum wait time for patient-synchronized inspiration termination |
| `Time_Insp_Pause` | Duration of the inspiratory pause / plateau hold |
| `Time_Exp_NonTrig` | Duration of the mandatory (non-triggerable) expiration phase |
| `Time_Exp_Support` | Duration of the expiration support window |
| `Time_Exp_Synch` | Maximum wait time for patient-synchronized expiration termination |
| `Time_Exp_Pause` | Duration of the expiratory pause / hold |

The sum of all eight durations defines one complete breathing cycle.

**Note**: The *Active Inspiration Time* used for tidal-volume flow calculations is `Time_Insp_NonTrig + Time_Insp_Support + Time_Insp_Synch` (pause excluded because no gas flows during the hold).

> **Design principle**: By accepting only pre-calculated times, the HLC remains a simple, deterministic state machine. All interpretation of clinician-facing parameters is delegated to the User Interface Layer (see below).

### HLC Parameters

#### Pressure Parameters
| Parameter | Description |
|-----------|-------------|
| `Inspiratory Pressure` | Baseline pressure target during inspiration |
| `Pressure Limit` | Maximum allowed airway pressure (safety limit) |
| `PEEP` | Positive End-Expiratory Pressure (baseline during expiration) |
| `Insp Support Pressure` | Pressure boost added during inspiration support state |
| `Exp Support Pressure` | Pressure boost added during expiration support state |

#### Flow Parameters
| Parameter | Description |
|-----------|-------------|
| `Max Inspiratory Flow` | Maximum allowed flow during inspiration |
| `Bias Flow` | Continuous flow during expiration (typically small) |

#### Volume Parameters
| Parameter | Description |
|-----------|-------------|
| `Tidal Volume (Vt)` | Target volume to deliver per breath |

---

## Tidal Volume Control

### Initial Flow Calculation

The required insufflation flow is calculated to deliver the target tidal volume within the available time:

```
Insufflation Flow = Vt / Active Inspiration Time

where:
    Active Inspiration Time = Time(Non-triggerable) + Time(Support) + Time(Synch)
```

**Note**: Pause time is excluded because no gas flows during the pause.

### Breath-to-Breath Adaptive Control

At each **inspiration → expiration transition**, the HLC performs:

1. **Measure** actual delivered tidal volume: `Vt_measured`
2. **Calculate** volume error: `ΔVt = Vt_target - Vt_measured`
3. **Adjust** flow setpoint for next breath:
   - If `Vt_measured < Vt_target` → **gradually increase** insufflation flow setpoint
   - If `Vt_measured > Vt_target` → **gradually decrease** insufflation flow setpoint

The gradual adjustment:
- Prevents oscillations
- Allows convergence to target volume
- Compensates for changing patient compliance and resistance

### Pressure-Limited Volume Control

When delivered pressure approaches the safety limit:

1. **LLC pressure controllers** automatically reduce flow to prevent exceeding `Pressure Limit`
2. This results in `Vt_measured < Vt_target` (volume sacrifice for safety)
3. **Adaptive control** attempts to increase flow for next breath
4. If pressure limiting **persists**:
   - System operates in **pressure-limited mode**
   - Delivered volume remains below target
   - **Patient safety (pressure) is always prioritised over volume target**

This creates two operating modes:
- **Volume-controlled mode**: When pressure stays below limit
- **Pressure-limited volume-controlled mode**: When pressure reaches limit

---

## Pressure Support Mechanism

### Inspiration Support State

When in **Inspiration Support** state and patient trigger is detected:

```
LLC Insufflation Pressure = Inspiratory Pressure + Insp Support Pressure
```

- Provides additional pressure boost
- Rewards patient inspiratory effort
- Common in PSV (Pressure Support Ventilation) modes

### Expiration Support State

When in **Expiration Support** state and patient trigger is detected:

```
LLC Insufflation Pressure = PEEP + Exp Support Pressure
```

- Provides pressure boost during expiration
- Less common, but available for specific clinical scenarios

---

## Inspiratory Pause Implementation

### Purpose
Hold the delivered volume without adding or releasing gas, allowing:
- Plateau pressure measurement
- Assessment of static lung compliance
- Equilibration of airway and alveolar pressures

### Mechanism

During **Insp Pause** state, the HLC "locks" the system by manipulating setpoints:

```
Bias Flow → 0
Exsufflation Pressure (PEEP) → 100 cmH2O
```

**Effect**:
1. **Zero bias flow** → No gas is added by the insufflation controllers
2. **Very high PEEP setpoint** → LLC enters exsufflation mode
3. **High PEEP prevents valve opening** → Expiratory valve stays closed (actual pressure << 100 cmH2O)
4. **Result**: System is mechanically "locked"
   - No gas flows in (insufflation inactive)
   - No gas escapes (expiratory valve closed)
   - Volume is held constant

This elegant solution uses the existing control structure without requiring special "pause mode" logic in the LLC.

---

## HLC Control Mechanism

### Indirect Control Philosophy

The HLC never sends explicit commands to the LLC. Instead, it controls behavior by:

1. **Setting insufflation pressure** based on current state and trigger events
2. **Setting pressure limit** for safety
3. **Setting insufflation flow** based on Vt, timing, and breath-to-breath adaptation
4. **Adjusting bias flow and PEEP** to implement pause phases
5. **Adjusting trigger thresholds** based on current HLC state

### State-Dependent Setpoint Updates

| HLC State | Insufflation Pressure | PEEP | Bias Flow |
|-----------|----------------------|------|-----------|
| Insp Non-trig | Inspiratory Pressure | Normal PEEP | Normal |
| Insp Support | Insp Press + Support * | Normal PEEP | Normal |
| Insp Synch | Inspiratory Pressure | Normal PEEP | Normal |
| **Insp Pause** | **N/A** | **100 cmH2O (lock)** | **0 (lock)** |
| Exp Non-trig | N/A | PEEP | Normal |
| Exp Support | PEEP + Support * | PEEP | Normal |
| Exp Synch | N/A | PEEP | Normal |
| Exp Pause | N/A | PEEP | Normal |

\* Support pressure is only applied when a patient trigger is detected in that state.

### Setpoint Change → Automatic LLC Response

When the HLC transitions states and modifies setpoints:
- LLC trigger criteria may suddenly become fulfilled
- LLC automatically transitions states (no command needed)
- Example: Setting PEEP to 100 cmH2O forces LLC into exsufflation mode

---

## Manoeuvres

Manoeuvres are **intermittent, temporary parameter changes** applied from any layer above the target layer. They allow the system to perform special clinical actions without adding mode-switching logic to the core controllers.

Examples include:
- **Recruitment manoeuvres** — temporarily increase pressure to open collapsed lung regions
- **Breath holds** — extend pause durations for measurement purposes
- **Sigh breaths** — periodically deliver a larger-than-normal tidal volume
- **Suction procedures** — temporarily alter flow and pressure settings

Manoeuvres work by temporarily overriding specific setpoints or timing values. When the manoeuvre completes, the original parameter values are restored. Because the lower layers are purely reactive to setpoints, no special manoeuvre-aware logic is needed in the LLC or Layer 1 controllers.

---

## User Interface Layer (Parameter Conversion)

### Purpose

The **User Interface (UI) layer** sits above the HLC and is responsible for translating clinician-friendly parameters into the absolute state times and setpoints expected by the HLC. The HLC itself never performs these conversions — it only consumes the resulting values.

This separation ensures that:
- The HLC remains a **simple, deterministic state machine** operating on absolute times.
- All interpretation of user preferences is **contained in the UI layer**.
- Different user interfaces (touchscreen, remote control, automated protocols) can each implement their own parameter sets and conversion logic while driving the same HLC through a single, well-defined interface.

### User-Facing Parameters

The UI layer accepts clinician-friendly parameters such as:

| User-Facing Parameter | Description |
|----------------------|-------------|
| `Respiratory Rate (RR)` | Breaths per minute → total cycle time = 60 / RR seconds |
| `I:E Ratio` | Inspiration-to-expiration ratio → splits the cycle time between the two phases |
| `Inspiratory Pause (%)` | Fraction of cycle time allocated to inspiratory pause |
| `Expiratory Pause (%)` | Fraction of cycle time allocated to expiratory pause |
| `Non-triggerable Inspiration (%)` | Fraction of inspiration phase that is mandatory |
| `Support Window (%)` | Fraction of inspiration/expiration allocated to support |
| `Synch Window (%)` | Fraction of inspiration/expiration allocated to synchronization |
| `Tidal Volume (Vt)` | Target volume per breath |
| `FiO2` | Fraction of inspired oxygen |
| `Inspiratory Pressure` | Target pressure during inspiration |
| `PEEP` | Positive end-expiratory pressure |
| `Pressure Limit` | Maximum allowed airway pressure |

### Timing Equations

The UI layer solves the timing equations and writes the eight absolute durations to the HLC:

```
Cycle Time              = 60 / RR
Inspiration Time        = Cycle Time × I / (I + E)
Expiration Time         = Cycle Time × E / (I + E)

Time_Insp_Pause         = Inspiration Time × (Insp Pause % / 100)
Active Insp Time        = Inspiration Time − Time_Insp_Pause
Time_Insp_NonTrig       = Active Insp Time × (Non-trig % / 100)
Time_Insp_Support       = Active Insp Time × (Support % / 100)
Time_Insp_Synch         = Active Insp Time × (Synch % / 100)

Time_Exp_Pause          = Expiration Time × (Exp Pause % / 100)
Active Exp Time         = Expiration Time − Time_Exp_Pause
Time_Exp_NonTrig        = Active Exp Time × (Non-trig % / 100)
Time_Exp_Support        = Active Exp Time × (Support % / 100)
Time_Exp_Synch          = Active Exp Time × (Synch % / 100)

Insufflation Flow       = Vt / Active Insp Time
```

The UI layer also computes the initial insufflation flow setpoint from the tidal volume and active inspiration time, and passes pressure/flow setpoints alongside the timing values.

### Ventilation Mode Presets

Different ventilation modes are implemented by the UI layer as **parameter presets** — specific combinations of the user-facing parameters that produce the desired timing pattern:

| Mode | Key UI Settings |
|------|----------------|
| **Volume Control (VC)** | Non-trig = 100%, Support = 0%, Synch = 0% → fully time-triggered |
| **Assist Control (AC)** | Non-trig backup timing, Exp Synch window active for patient triggering |
| **Pressure Support (PSV)** | Non-trig minimal, Support + Synch active → patient-triggered & cycled |
| **SIMV** | Non-trig for mandatory breaths, Support windows for spontaneous breaths |

The HLC sees only the resulting absolute times and has no concept of "mode" — mode selection is entirely a UI layer concern.

### Design Benefits

1. **HLC simplicity**: The state machine has no conditional parameter logic, only countdown timers and synchronization events.
2. **Testability**: HLC can be tested with known absolute times without needing a UI.
3. **Flexibility**: New ventilation modes or parameter schemes can be added in the UI layer without changing the HLC.
4. **Multi-UI support**: Touchscreen, web interface, serial console, or automated protocol engines can all drive the same HLC.

---

## Communication Between Layers

### UI Layer → HLC (Configuration)
- Eight absolute state durations (Time_Insp_NonTrig, …, Time_Exp_Pause)
- Pressure setpoints (Inspiratory Pressure, PEEP, Pressure Limit, Support Pressures)
- Flow setpoints (Insufflation Flow, Max Inspiratory Flow, Bias Flow)
- Trigger thresholds (flow and pressure)
- Tidal Volume target

### HLC → LLC (Setpoints)
- Pressure setpoints (state-dependent, updated on each HLC state transition)
- Flow setpoints (including breath-to-breath adapted insufflation flow)
- Trigger thresholds (adjusted per HLC state)
- All parameters listed in LLC section

### LLC → HLC (Signals)
- State change events (insufflation ↔ exsufflation)
- Measured tidal volume (integrated flow)
- Pressure limit events

### LLC → Layer 1 (Setpoints)
- Individual flow controller setpoints
- Expiratory valve pressure setpoint
- Calculated from LLC parameters and current state

---

## Example Ventilation Modes

This architecture can implement various clinical ventilation modes. The **UI layer** selects the appropriate parameter preset and solves the timing equations; the **HLC** receives only the resulting absolute times.

### Volume Control (VC)
- UI sets: Non-trig = 100% of active time, Support = 0%, Synch = 0%
- HLC sees: Time_Insp_NonTrig = full active insp time, Time_Insp_Support = 0, Time_Insp_Synch = 0
- Insp Pause: Optional (UI allocates fraction for plateau measurement)
- Exp states: Fixed timing calculated by UI from RR and I:E ratio
- **Result**: Time-triggered, volume-cycled mandatory breaths

### Assist Control (AC)
- UI sets: Non-trig with backup timing, Exp Synch window > 0
- HLC sees: Time_Exp_Synch > 0, allowing patient triggering during expiration
- **Result**: Patient can trigger breaths, but gets full support

### Pressure Support Ventilation (PSV)
- UI sets: Support and Synch windows active, Non-trig minimal (safety backup)
- HLC sees: Time_Insp_Support and Time_Insp_Synch > 0, Time_Exp_Synch > 0
- **Result**: Fully patient-triggered and cycled breaths with pressure support

### SIMV (Synchronized Intermittent Mandatory Ventilation)
- UI configures combination of mandatory (Non-trig) and supported (Support) timing
- HLC sees appropriate time distribution; Synch states coordinate with patient effort

---

## Safety Considerations

1. **Pressure Limiting**: Always active in LLC to prevent barotrauma
2. **Backup Ventilation**: Non-trig states ensure minimum ventilation if patient effort fails
3. **Gradual Adaptation**: Prevents sudden changes in delivered volume
4. **Setpoint Sanity Checks**: HLC should validate parameters before sending to LLC
5. **Watchdog Timers**: Each state should have maximum duration limits

---

## Design Advantages

1. **Modularity**: Each layer has clear responsibilities
2. **No Direct Commands**: Simplifies interface and reduces coupling
3. **Extensibility**: New modes created by timing/parameter configuration
4. **Patient Synchronization**: Built into state machine architecture
5. **Safety**: Multiple layers of protection (LLC pressure limiting, HLC limits)
6. **Adaptability**: Breath-to-breath learning handles patient variability

---

## Implementation Notes

### Execution Rates

| Layer | Typical Update Rate |
|-------|--------------------|
| Layer 1 (PID controllers) | 100 – >1000 Hz |
| Layer 2 (LLC) | ~100 Hz |
| Layer 3 (HLC) | 10 – 50 Hz |
| Layer 4 (UI) | User events (asynchronous) |

### State Transitions
- Use event-driven architecture for LLC state changes
- HLC runs on fixed time base (see rates above)
- Layer 1 controllers run at the highest loop rate

### Parameter Updates
- Validate all parameter changes before applying
- Use rate limiting on parameter changes to prevent instability
- Log all setpoint changes for debugging and clinical review

### Testing
- Test each layer independently first
- Verify LLC responds correctly to setpoint changes
- Verify HLC generates correct state sequences
- Test all ventilation modes thoroughly
- Include patient simulator testing with varying compliance/resistance

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-02-01 | 1.0 | Initial documentation of three-layer architecture |
| 2026-03-23 | 1.1 | Added User Interface Layer; clarified HLC operates on absolute times only; moved timing equations to UI layer; updated communication and mode descriptions |
| 2026-03-24 | 1.2 | Fixed title to reflect layered (not three-layer) architecture; corrected pause duration description to match absolute-times model |
| 2026-03-24 | 1.3 | Synced with manually updated PPTX: "one-mode" framing; added Manoeuvres section; added flow–pressure continuum to LLC; reframed insufflation/exsufflation descriptions; added LLC absolute pressure limit note; updated parameter names/aliases; added trigger footnote to setpoint table; strengthened safety priority statement; added execution rate table; added Layer 1 lookup-table and hardware-driver details |
