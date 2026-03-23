# Inspiratory Valve Linearisation & Feed-Forward — Integration Guide

This package contains the self-contained algorithm files your implementation needs.
The code is taken from the **ParallelMixer** ventilator project; you can copy it
verbatim or adapt it to your own hardware layer.

---

## Files in this package

| File | What it is |
|:-----|:-----------|
| `LocalValveController.hpp` | Header — all data structures and class declarations |
| `LocalValveController_core.cpp` | Algorithm implementations (lookup tables, PI, InspValveController) |
| `valve_table_example.cpp` | Detailed worked example: table loading, config, per-cycle call, runtime trace |
| `main_example.cpp` | Minimal self-contained main — copy this as your integration starting point |

---

## Concept in one paragraph

A proportional solenoid inspiratory valve is **nonlinear and pressure-dependent**: the
same drive percentage (V%) produces different flows at different supply pressures.
The system linearises this with a **2D PressureBandedTable**: 2–4 piecewise-linear
V%→Flow curves, each measured at a different supply pressure. At runtime an
`inverseLookup(desired_flow, live_Psupply)` call bilinearly interpolates between the
nearest two bands to produce a feedforward V%.
A pair of PI controllers (flow PI + pressure-limit PI) correct residual error. A
min-select override picks whichever controller demands less current, and an
anti-windup tracker keeps the idle controller's integrator aligned so there is no
bump on hand-off.

---

## Integration steps (summary)

> **Steps 1–3 are performed on the ParallelMixer system.**
> The ParallelMixer firmware contains the `ValveCharacterizer` that does the
> hardware sweep and generates the C++ tables.  Your colleague only needs the
> *output* of step 3 (the populated `LookupPoint[]` arrays and the
> `InspValveConfig` values) to integrate into his own system.
> Steps 4–6 are then performed entirely within his own codebase.

### 1. Measure your valve — run CC sweeps

Use the built-in `CC` serial command at 2–3 different supply pressures:

```
CC1,14,0.2,250    # air valve, regulator at ~3 bar
CC1,14,0.2,250    # same, regulator at ~4.5 bar
CC1,14,0.2,250    # same, regulator at ~6 bar
```

Each sweep prints ~71 CSV rows and a code-ready `LookupPoint[]` table.

### 2. Reduce to ≤16 points per band

Run the helper script (in `tools/`):

```bash
python tools/reduce_cc_sweep.py sweep_3bar.txt sweep_4.5bar.txt sweep_6bar.txt --normalize
```

`--normalize` corrects for supply-pressure droop during the sweep (recommended).
The script outputs ready-to-paste C++ arrays and `setFlowBand()` calls.

### 3. Load the tables at startup

```cpp
// In your init / setup function — ascending pressure order
_airValve.setFlowBand(0, 302.0f, airFlowBand0, 14);
_airValve.setFlowBand(1, 451.0f, airFlowBand1, 15);
_airValve.setFlowBand(2, 604.0f, airFlowBand2, 16);
```

Band pressures are in **kPa** (matching the ABP2 supply-pressure sensor output).

### 4. Call InspValveController::update() at 100 Hz

```cpp
InspValveSetpoints  sp  = { flowSetpoint_slm, pressureLimit_mbar };
InspValveMeasurements m = { measuredFlow_slm, supplyPressure_kPa, airwayPressure_mbar };

float current_A = _airValve.update(sp, m, dt);
// → send current_A to your valve driver / DAC
```

See `valve_table_example.cpp` for a complete self-contained example.

### 5. Calibrate the cracking current

The cracking current is the minimum current to overcome the valve spring.
It is pressure-dependent for inspiratory valves:

```
I_crack = I_base + k × P_supply [bar]
```

Measure it by slowly raising current from zero at two different supply pressures,
noting the current at first flow. Fit a line to get `I_base` and `k`, then set them
in `InspValveConfig`:

```cpp
config.crackBaseOffset_A        = 0.12f;   // A at zero pressure
config.crackPressCoeff_A_per_bar = 0.03f;  // A per bar
```

### 6. Tune PI gains

Start with the feedforward doing most of the work (`useFeedforward = true`).
The flow PI only needs to correct residual linearisation error — start conservatively:

```
kp = 0.005, ki = 0.05   (units: A per slm error)
```

The pressure PI ceiling should be set so that normal flow control never triggers it —
only a genuine pressure-limit condition should activate it.

---

## Key classes / structs (quick reference)

| Symbol | Role |
|:-------|:-----|
| `LookupPoint {x, y}` | Single table entry |
| `PiecewiseLinearTable` | 1-D linear interpolation table (≤16 pts) |
| `PressureBandedTable` | 2-D table: 2–4 bands, bilinear interpolation |
| `PIController` | PI with anti-windup integrator tracking |
| `InspValveConfig` | Configuration (gains, cracking current, limits) |
| `InspValveMeasurements` | Sensor inputs per cycle |
| `InspValveSetpoints` | Demanded flow + pressure limit per cycle |
| `InspValveStatus` | Diagnostic outputs (feedforward, PI outputs, flags) |
| `InspValveController` | Top-level: feedforward + flow PI + pressure PI |

---

## Hardware assumptions

- Supply-pressure sensor: **Honeywell ABP2**, output in **kPa**
- Airway-pressure sensor: **Sensirion ELVH**, output in **mbar**
- Flow sensor: **Sensirion SFM3505**, output in **slm**
- Valve driver: current-controlled (A), 0–max range set in config

If your sensors use different units, rescale in the `InspValveMeasurements` struct
before calling `update()`.

---

## Further reading

- `docs/VALVE_CALIBRATION.md` in the source repo — full calibration procedures,
  CC command reference, reduction strategy, worked examples.
- `docs/ARCHITECTURE.md` — design rationale for the 2D surface and override control.
