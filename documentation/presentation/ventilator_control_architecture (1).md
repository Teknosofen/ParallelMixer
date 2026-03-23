# Three-Layer Ventilator Control Architecture

## Overview

This document describes a three-layer control architecture for a medical ventilator system. The architecture consists of:

1. **Layer 1**: Low-Level PID Controllers (Actuator Layer)
2. **Layer 2**: Low-Level Controller (LLC) - Breathing Phase Control
3. **Layer 3**: High-Level Controller (HLC) - Ventilation Mode Control

The control philosophy is based on **indirect control through setpoint manipulation** - higher layers do not send explicit commands but instead modify setpoints that trigger automatic responses in lower layers.

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
- Each controller operates independently
- Handles non-linearities in sensors and actuators
- Provides fast, stable control loops
- No knowledge of breathing phases or ventilation modes

---

## Layer 2: Low-Level Controller (LLC) - Breathing Phase Control

### Purpose
Manage the fundamental breathing cycle alternating between insufflation (inspiration) and exsufflation (expiration).

### States

#### Insufflation State (Inspiration)
- Delivers pressure/flow to the patient
- Controls gas delivery via O2 and Air flow controllers
- Maintains target inspiratory pressure
- **Pressure limiting**: Reduces flow when measured pressure approaches pressure limit

#### Exsufflation State (Expiration)
- Releases pressure/flow from the patient
- Controls expiratory valve to manage pressure release
- Maintains PEEP (baseline pressure)

### Parameters (Set by HLC)

| Parameter | Description |
|-----------|-------------|
| `FiO2` | Fraction of inspired oxygen (0.21 - 1.0) |
| `Insufflation Pressure` | Target pressure during insufflation |
| `Pressure Limit` | Maximum allowed airway pressure |
| `Exsufflation Pressure (PEEP)` | Target pressure during exsufflation |
| `Insufflation Flow` | Target flow rate during insufflation |
| `Bias Flow` | Continuous flow maintained during exsufflation |
| `Max Inspiratory Flow` | Maximum allowed inspiratory flow |
| `Flow Trigger Threshold` | Flow change required to detect patient effort |
| `Pressure Trigger Threshold` | Pressure change required to detect patient effort |

### State Transition Mechanisms

The LLC transitions between insufflation and exsufflation based on:

1. **Patient Effort Detection**
   - Monitors flow and pressure against trigger thresholds
   - Patient inspiratory effort → triggers insufflation
   - Patient expiratory effort → triggers exsufflation

2. **Setpoint-Induced Transitions**
   - When HLC modifies setpoints, trigger criteria may suddenly be fulfilled
   - No explicit commands needed - purely reactive behavior
   - Example: HLC sets high PEEP (100 cmH2O) → LLC shifts to exsufflation mode attempting to reach this pressure

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
   - Duration set as fraction of cycle time

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

| Parameter | Description | Calculation |
|-----------|-------------|-------------|
| `Respiratory Rate (RR)` | Breaths per minute | Cycle time = 60/RR seconds |
| `I:E Ratio` | Inspiration:Expiration ratio | Determines time split between phases |
| `Inspiratory Pause Fraction` | Plateau time as fraction of cycle | Pause time = fraction × (60/RR) |
| `Active Inspiration Time` | Time for gas delivery | Sum of Non-trig + Support + Synch times |

**Note**: Active Inspiration Time excludes Pause time for tidal volume calculations.

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
   - Patient safety prioritized over volume target

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
| Insp Support | Insp Press + Support (if triggered) | Normal PEEP | Normal |
| Insp Synch | Inspiratory Pressure | Normal PEEP | Normal |
| **Insp Pause** | **N/A** | **100 cmH2O** | **0** |
| Exp Non-trig | N/A | PEEP | Normal |
| Exp Support | PEEP + Support (if triggered) | PEEP | Normal |
| Exp Synch | N/A | PEEP | Normal |
| Exp Pause | N/A | PEEP | Normal |

### Setpoint Change → Automatic LLC Response

When the HLC transitions states and modifies setpoints:
- LLC trigger criteria may suddenly become fulfilled
- LLC automatically transitions states (no command needed)
- Example: Setting PEEP to 100 cmH2O forces LLC into exsufflation mode

---

## Communication Between Layers

### HLC → LLC (Setpoints)
- Pressure setpoints
- Flow setpoints
- Trigger thresholds
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

This architecture can implement various clinical ventilation modes by configuring the 8-state timing and parameters:

### Volume Control (VC)
- Insp Non-trig: Delivers full breath
- Insp Support: 0 time (skipped)
- Insp Synch: 0 time (skipped)
- Insp Pause: Optional (for plateau measurement)
- Exp states: Fixed timing
- **Result**: Time-triggered, volume-cycled mandatory breaths

### Assist Control (AC)
- Insp Non-trig: Backup mandatory breath timing
- Exp Synch: Active (allows patient triggering)
- **Result**: Patient can trigger breaths, but gets full support

### Pressure Support Ventilation (PSV)
- Insp Support: Active with support pressure
- Insp Synch: Active (patient cycles off)
- Exp Synch: Active (patient cycles on)
- Non-trig states: Minimal or backup only
- **Result**: Fully patient-triggered and cycled breaths with pressure support

### SIMV (Synchronized Intermittent Mandatory Ventilation)
- Combination of mandatory (Non-trig) and supported (Support) breaths
- Synch states active to coordinate with patient effort

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

### State Transitions
- Use event-driven architecture for LLC state changes
- HLC runs on fixed time base (e.g., 10-50 Hz)
- Layer 1 controllers run fastest (100-1000 Hz typical for PID loops)

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

