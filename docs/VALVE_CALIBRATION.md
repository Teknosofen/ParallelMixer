# Valve Calibration Guide

This document describes the steps required to populate the cracking current and valve linearization tables for the P-Mixer ventilator control system, including the automated `CC` characterization command and how to integrate the results into the software.

> See also: [ARCHITECTURE.md — Valve Characterization & Linearization](ARCHITECTURE.md#valve-characterization--linearization) for the design rationale and data-structure details.

---

## 1. Why a 2D Flow Surface?

Proportional solenoid valves are pressure-dependent: the same drive percentage (V%) produces different flows at different supply pressures. A single 1D lookup table cannot capture this. We therefore use a **PressureBandedTable** — 2–4 piecewise-linear V% → Flow tables, each measured at a different supply pressure. The feedforward interpolates between bands at runtime using the live ABP2 supply-pressure reading.

### How the 2D Surface Works (Piecewise Bilinear Interpolation)

There is no offline curve fitting, regression, or least-squares involved. The "surface" is built by storing the raw (reduced) CC sweep data directly and interpolating between sweeps at runtime.

#### Data Structure

```
PressureBandedTable
├── Band 0:  PiecewiseLinearTable  @ condition = C₀  (e.g. Psupply = 302 kPa)
├── Band 1:  PiecewiseLinearTable  @ condition = C₁  (e.g. Psupply = 451 kPa)
├── Band 2:  PiecewiseLinearTable  @ condition = C₂  (e.g. Psupply = 604 kPa)
└── (max 4 bands, max 16 points each)
```

Each band is one CC sweep, reduced to ≤16 points: `{control%, result}`.

The same `PressureBandedTable` class is reused for all three actuator types, with different physical meanings:

| | Insp Valve | Exp Valve | Blower |
|:--|:-----------|:----------|:-------|
| **Band condition** | Psupply (kPa) | Flow (slm) | Paw (mbar) |
| **Table x** | V% (control) | V% (control) | PWM% (control) |
| **Table y** | Flow (result) | Paw (result) | Flow (result) |
| **inverseLookup** | flow → V% | Paw → V% | flow → PWM% |

#### Runtime Lookup — `inverseLookup(desiredResult, liveCondition)`

Example: insp valve, "I want 50 slm at live Psupply = 500 kPa":

**Step 1 — `findBrackets(500, lo, hi, t)`:**
Find the two bands that bracket the live condition variable:

$$t = \frac{P_{live} - P_{lo}}{P_{hi} - P_{lo}} = \frac{500 - 451}{604 - 451} \approx 0.32$$

**Step 2 — Inverse lookup in each band:**
- Band 1 (451 kPa): `inverseLookup(50 slm)` → linearly interpolate between the two table points that bracket 50 slm → **V% = 8.2%**
- Band 2 (604 kPa): `inverseLookup(50 slm)` → **V% = 7.1%**

**Step 3 — Interpolate V% between bands:**

$$V\% = V\%_{lo} + t \cdot (V\%_{hi} - V\%_{lo}) = 8.2 + 0.32 \times (7.1 - 8.2) = \mathbf{7.85\%}$$

#### Extrapolation

If the live condition is outside the stored band range (e.g., below band 0 or above band N−1), the same linear formula is used with $t < 0$ or $t > 1$. This gives reasonable trends but degrades in accuracy — keep bands covering your expected operating range.

#### Exp Valve Example

Each CC3 sweep at a different LF flow rate gives one band:

1. `LF5` → `CC3` → band 0: V% → Paw at ~5 slm
2. `LF20` → `CC3` → band 1: V% → Paw at ~20 slm
3. `LF40` → `CC3` → band 2: V% → Paw at ~40 slm

At runtime: `inverseLookup(desiredPaw, measuredFlow)` → V%, with bilinear interpolation between the 2–3 bands.

---

## 2. Cracking Current Calibration

### Purpose
The cracking current is the minimum current required to open the valve against the spring and supply pressure. For inspiratory valves, it is pressure-dependent:

$$I_{crack} = I_{base} + k \cdot P_{supply}$$

- $I_{base}$: Cracking current at zero supply pressure (spring only)
- $k$: Pressure coefficient (A/bar)
- $P_{supply}$: Supply pressure in bar

### Procedure
1. **Isolate the valve** (no downstream load, measure at the valve itself).
2. **Set supply pressure** to a known value (e.g., 2 bar).
3. **Slowly increase current** from zero while monitoring flow or valve movement.
4. **Record the current** at which the valve first opens (cracking point).
5. **Repeat** for at least one other supply pressure (e.g., 5 bar).
6. **Fit a line**: $I_{crack}$ vs. $P_{supply}$ to determine $I_{base}$ and $k$.
7. **Update** `crackBaseOffset_A` and `crackPressCoeff_A_per_bar` in config (see Section 5).

---

## 3. Automated Valve Characterization (CC Command)

### Overview
The `CC` serial command runs an automated control-signal sweep on an actuator. It ramps the control signal from 0 to a configurable maximum in fixed steps, waits for the output to settle at each step, samples sensors multiple times, and prints CSV data rows plus a code-ready lookup table. Run sweeps at **different operating conditions** (supply pressure, counter pressure, or flow) to populate multiple bands of a `PressureBandedTable` 2D surface.

### Supported Actuators
| MUX Channel | Actuator | Control Signal | Primary Output | Condition Variable | Sensors Used |
|:-----------:|:--------:|:--------------:|:---------------|:-------------------|:-------------|
| 1           | Air valve | V% (MUX)     | Flow (slm)     | Supply pressure    | Bus 0 — SFM3505, ABP2, ELVH |
| 2           | O2 valve  | V% (MUX)     | Flow (slm)     | Supply pressure    | Bus 1 — SFM3505, ABP2, ELVH |
| 3           | Exp valve | V% (MUX)     | Paw (mbar)     | Flow rate          | Bus 0 — SFM3505, ABP2, ELVH |
| 4           | Blower    | PWM% (GPIO21) | Flow (slm)     | Counter pressure   | Bus 0 — SFM3505, ABP2, ELVH |

### Command Syntax

```
CC<channel>[,max[,step[,settleTime_ms]]]
```

| Parameter       | Type   | Default (valves) | Default (blower) | Description |
|:----------------|:------:|:----------------:|:----------------:|:------------|
| `channel`       | int    | —                | —                | **Required.** 1=air, 2=O2, 3=exp, 4=blower |
| `max`           | float  | 12.0             | 100.0            | Maximum control signal % |
| `step`          | float  | 0.1              | 1.0              | Increment per step % |
| `settleTime_ms` | int    | 200              | 200              | Stabilization time per step (ms) |

Samples per step is fixed at **10** (averaged).

### Examples

| Command            | Description |
|:-------------------|:------------|
| `CC1`              | Air valve, 0–12 V% in 0.1% steps, 200 ms settle |
| `CC2`              | O2 valve, same defaults |
| `CC1,14,0.2,250`   | Air valve, 0–14 V%, 0.2% step, 250 ms settle |
| `CC3,12,0.1,200`   | Exp valve, 0–12 V% (set flow with LF first) |
| `CC4`              | Blower, 0–100 PWM% in 1% steps |
| `CC4,100,2,300`    | Blower, 2% steps, 300 ms settle |

### Aborting a Sweep

```
CX
```

Immediately sets the valve V% to 0 and returns to idle.

### What Happens During a Sweep

1. The characterizer sets the control signal to 0 and begins settling.
2. At each step it:
   - Waits `settleTime_ms` for the output to stabilize.
   - Samples sensors 10 times and averages them.
   - For insp valves: computes $Cv = Q / \sqrt{P_{supply} - P_{aw}}$ as a diagnostic.
   - Prints a CSV data row to the serial console.
3. After the final step it:
   - Sets the control signal back to 0.
   - Prints `# --- Sweep complete ---`.
   - Prints a **code-ready C++ table** with `loadBand()` instructions.

### Serial Output Format

All modes follow the same column convention: **Result, Control, Modifier, (extra), Cv**.

**Inspiratory valves (ch 1, 2):**
```
# Flow(slm),    V(%),    Psupply(kPa),    Paw(mbar),    Cv
```
- *Result*: Flow (what the valve produces)
- *Control*: V% (what we sweep)
- *Modifier*: Psupply (condition variable that shifts the curve)

**Expiratory valve (ch 3):**
```
# Paw(mbar),    V(%),    Flow(slm),    Psupply(kPa),    Cv
```
- *Result*: Paw (the back-pressure the valve creates)
- *Control*: V% (what we sweep)
- *Modifier*: Flow (condition variable; varies as Paw builds)

**Blower (ch 4):**
```
# Flow(slm),    PWM(%),    Paw(mbar),    Psupply(kPa)
```
- *Result*: Flow
- *Control*: PWM%
- *Modifier*: Paw (counter pressure)

**Cv (flow coefficient):**
- Insp valves: $Cv = Q / \sqrt{P_{supply} - P_{aw}}$ (pressure drop across valve)
- Exp valve: $Cv = Q / \sqrt{P_{aw}}$ (Paw exhausting to atmosphere)
0.000,  0.001,          603.7,          -0.09,          0.00000
5.600,  0.173,          603.0,          -0.06,          0.00705
...
14.000, 193.885,        375.8,          19.01,          10.26461
```

**Code-ready table** (printed at end):
```cpp
// ============================================================
// Air valve flow table from characterization
// Avg Psupply ~ 604 kPa (6.04 bar)
// MUX channel 1, 71 points
// x = V%, y = Flow (slm)
// NOTE: Reduce to <=16 points before loading into PressureBandedTable
// ============================================================
static const LookupPoint airFlowBand_P604[] = {
    {0.000f, 0.001f},    // Psup=604 kPa, Cv=0.0000
    {5.600f, 0.173f},    // Psup=603 kPa, Cv=0.0071
    ...
};
// Usage: _valve.setFlowBand(<bandIndex>, 604.0f, airFlowBand_P604, <count>);
```

### Prerequisites
- Supply gas connected and regulated to operational pressure (for insp valve / blower).
- Downstream flow path open (patient wye disconnected or open to atmosphere).
- Serial terminal connected at host baud rate.
- **Exp valve only:** Set a fixed inspiratory flow first with `LF<flow>` (e.g., `LF10` for 10 SLM). The characterizer measures Paw as a function of exp valve V% at that flow.
- **Blower only:** Apply a fixed counter pressure / resistance to the blower outlet. Repeat the sweep at different counter pressures to build the 2D surface.

> **Automatic arbitration:** You do **not** need to disable the LLC (`LE0`) before running a characterization sweep. The control loop automatically prevents the LLC from overwriting channels owned by the characterizer or by manual `M`+`V` commands. See the [Actuator Output Ownership](#actuator-output-ownership-arbitration) section below for details.

---

## 3b. Actuator Output Ownership & Arbitration

Three subsystems can write to the same MUX channel in each 100 Hz control cycle:

| Subsystem | Code Path | What It Controls |
|:----------|:----------|:-----------------|
| **CC** (Characterizer) | `valveChar.update()` | V% or PWM% on the single channel being swept |
| **LLC** (Local Valve Controller) | `localValveCtrl.update()` | V% on air (ch1), O2 (ch2), exp (ch3); PWM on blower (ch4) |
| **ACT** (Actuator Control) | `executeActuatorControl()` | V% from `M`+`V`, signal generators, PID — all channels |

The main loop runs them in order: **CC → LLC → ACT**, with guards that prevent conflicts:

### Per-Mode Ownership Table

| Mode | Ch 1 (Air) | Ch 2 (O2) | Ch 3 (Exp) | Ch 4 (Blower) |
|:-----|:----------:|:---------:|:----------:|:-------------:|
| **Idle** (no LF, no vent) | ACT | ACT | ACT | ACT |
| **LF** (manual flow test) | LLC | LLC | ACT (`M3`+`V`) | LLC |
| **LF + CC1** | CC | LLC | ACT | LLC |
| **LF + CC3** | LLC | LLC | CC | LLC |
| **LF + CC4** | LLC | LLC | ACT | CC (PWM) |
| **Ventilator running** | LLC | LLC | LLC | LLC |

### How the Guards Work

1. **`skipMuxChannel`** — a field in the LLC setpoints struct. In LF mode it defaults to **3** (exp valve), so the LLC never sends V% to ch3. If a characterizer is sweeping ch1 or ch2, the skip shifts to that channel instead.
2. **`skipBlower`** — set to `true` when CC4 is sweeping the blower. The LLC will not zero the blower PWM.
3. **`executeActuatorControl()`** guards:
   - Ventilator running → returns immediately (LLC owns everything).
   - CC running → returns immediately (characterizer owns its channel).
   - LF active → runs **only** `actuators[3]` (exp valve), so `M3`+`V` commands persist.
   - Idle → runs all channels.

### Practical Consequences

- **`LF10` then `M3`, `V3`** — the exp valve stays at V3% while air flow is held at 10 slm by the LLC. Previously the LLC's `forceOpen()` would immediately zero the exp valve.
- **`LF10` then `CC3,12`** — the characterizer sweeps exp V%, the LLC holds air flow, and neither overwrites the other.
- **`CC4,100,2`** with `LF` running — the characterizer drives blower PWM, the LLC holds insp flow, exp valve under manual control.
- **Ventilator mode** — the LLC has exclusive control of every channel. Manual `M`+`V` commands are ignored (the LLC overwrites them next cycle).

---

## 4. Multi-Condition Characterization Procedures

### 4a. Inspiratory Valve (Air / O2) — Vary Supply Pressure

#### Goal
Capture the valve's full 2D flow surface by running sweeps at 2–3 different supply pressures.

#### Steps

1. Set the supply regulator to **~3 bar**. Run:
   ```
   CC1,14,0.2,250
   ```
   Copy the printed flow table from the serial terminal. Note the Psupply.

2. Increase the regulator to **~4.5 bar**. Run:
   ```
   CC1,14,0.25,250
   ```
   Copy the second table.

3. Increase the regulator to **~6 bar**. Run:
   ```
   CC1,14,0.2,250
   ```
   Copy the third table.

4. Repeat steps 1–3 for the **O2 valve** (`CC2`) if it has a different valve type.

#### Supply Pressure Droop

During a sweep, the supply pressure droops as flow increases (limited by regulator capacity). For example, a 6-bar sweep may see Psupply drop from 604 kPa to 376 kPa at full valve opening — a 38% reduction. This is **expected** — each data row records the actual supply pressure at that point, and the P_ref stored per band is the average supply pressure over the sweep.

However, the droop means the flow values in the active and saturation regions are **lower than they would be at the nominal (average) pressure**. This causes the feedforward to undershoot at high flows when the live supply pressure is near nominal. The effect is negligible at low flows (cracking region) where pressure is still close to the set value.

To correct for this, use the `--normalize` option in the reduction script (see Section 5c). It adjusts each flow measurement to what it would be at the band's reference pressure using the per-point Psupply data from the CSV.

### 4b. Blower — Vary Counter Pressure

#### Goal
Capture the blower's flow vs. PWM% characteristic at different counter pressures (patient circuit resistance).

#### Steps

1. Connect the blower outlet through a **low-resistance** path (open tube). Run:
   ```
   CC4,100,2,300
   ```
   Copy the printed table. The average Paw is the counter pressure for this band.

2. Add a **medium resistance** (partially occluded tube or restrictor). Run:
   ```
   CC4,100,2,300
   ```
   Copy the second table.

3. Add a **high resistance** (e.g., HEPA filter or narrow orifice). Run:
   ```
   CC4,100,2,300
   ```
   Copy the third table.

4. You now have 2–3 flow tables at different counter pressures — one per band.

#### Output Table Format
```cpp
static const LookupPoint blowerFlowBand_P5[] = {  // avg Paw ≈ 5 mbar
    {0.00f, 0.000f},
    {10.00f, 2.340f},
    ...
    {100.00f, 85.120f},
};
// Load with: blowerSurface.loadBand(<bandIdx>, 5.0f, blowerFlowBand_P5, count);
```

### 4c. Expiratory Valve — Vary Flow Rate

#### Goal
Capture the exp valve's pressure vs. V% characteristic at different flow rates.

#### Steps

1. Set a **low** inspiratory flow:
   ```
   LF5
   ```
   Wait for flow to stabilize, then sweep the exp valve:
   ```
   CC3,12,0.1,200
   ```
   Copy the printed table. The average flow is the condition variable.

2. Set a **medium** flow:
   ```
   LF20
   ```
   Run:
   ```
   CC3,12,0.1,200
   ```

3. Set a **high** flow:
   ```
   LF40
   ```
   Run:
   ```
   CC3,12,0.1,200
   ```

4. Stop the flow test: `LX`

5. You now have 2–3 pressure tables at different flows — one per band.

#### Output Table Format
```cpp
static const LookupPoint expPressureBand_F20[] = {  // avg flow ≈ 20 slm
    {0.000f, 0.12f},
    {2.000f, 1.85f},
    ...
    {12.000f, 38.50f},
};
// Load with: expSurface.loadBand(<bandIdx>, 20.0f, expPressureBand_F20, count);
```

---

## 5. Reducing Points & Selecting Table Data

### Why Reduce?

The `PiecewiseLinearTable` supports a maximum of **16 points**. A full CC sweep at 0.2% steps over 14% produces ~71 data points, and a blower sweep at 1% over 100% produces ~101 points. These raw sweeps must be reduced to ≤16 representative points per band.

The piecewise-linear interpolation between the selected points **is** the model — there is no additional curve fitting. Therefore the choice of which points to keep directly determines the accuracy of the feedforward.

### Reduction Strategy

The key insight is that interpolation error is largest where the curve has the most curvature (the "knee" region). Straight segments need fewer points; curved segments need more.

#### Step 1: Identify Regions

Plot (or visually inspect) the raw sweep. Every valve/blower sweep has these characteristic regions:

```
Result
  ▲
  │                              ┌─── Saturation (flat)
  │                         ╱────┘
  │                    ╱───╱
  │               ╱───╱        ← Active region (roughly linear)
  │          ╱───╱
  │      ╱──╱                  ← Cracking / knee (high curvature)
  │─────╱
  │                            ← Dead zone (zero output)
  └──────────────────────────────► Control %
```

| Region | Behaviour | Points to keep |
|:-------|:----------|:---------------|
| **Dead zone** | Result ≈ 0 regardless of control % | 2: the first point (0%) and the last zero-result point |
| **Cracking / knee** | Rapid rise from zero — highest curvature | 2–3 closely spaced points |
| **Active region** | Roughly linear ramp | 8–10 evenly spaced points |
| **Saturation** | Result flattens (valve fully open, sensor limit, or regulator droop) | 1–2 points |

#### Step 2: Select Points

Work through the raw CSV data (or a spreadsheet plot):

1. **Dead zone** — keep `{0.0, 0.0}` and the last row where result is still ≈ 0. These two points define the dead band the `inverseLookup()` must skip.

2. **Cracking** — keep 2–3 points right at the transition from zero to non-zero output. This is where the curve is steepest relative to the dead zone, and the interpolation error is largest if you jump straight from zero to a mid-value.

3. **Active region** — pick points at roughly equal *result* spacing (not equal *control%* spacing). Equal result spacing distributes the inverse-lookup error evenly. For example, if the sweep goes 0→200 slm, pick points near 20, 40, 60, 80, 100, 120, 140, 160, 180 slm.

4. **Saturation** — keep the last point before flattening and one point in the flat zone. If the result is still increasing at max control%, keep just the last two raw data points.

#### Step 3: Verify

After selecting your 12–16 points, check that the maximum interpolation error (vs. the raw data) is acceptable. A quick way:

1. For each raw data point, compute the piecewise-linear prediction from your reduced table.
2. The difference is the interpolation error.
3. If any error exceeds your tolerance (e.g., >2 slm for a 200 slm range), add an extra point in that region.

### Example: Reducing 71 Points to 16 (6-bar sweep)

```cpp
static const LookupPoint airFlowBand2[] = {
    { 0.00f,    0.0f},    // Dead zone
    { 5.00f,    0.0f},    // Still closed
    { 5.60f,    0.17f},   // Just cracking
    { 6.00f,    0.91f},
    { 6.40f,    4.08f},
    { 6.80f,   12.89f},
    { 7.20f,   22.48f},
    { 7.60f,   34.40f},
    { 8.00f,   48.91f},
    { 8.40f,   68.54f},
    { 8.80f,   84.81f},
    { 9.20f,  103.18f},
    { 9.60f,  127.14f},
    {10.00f,  160.04f},
    {10.60f,  194.88f},
    {11.20f,  195.50f},   // Saturated
};
```

In this example:
- **Dead zone** (2 points): 0.00% and 5.00% — both produce 0 flow.
- **Cracking** (2 points): 5.60% and 6.00% — the transition from 0.17 to 0.91 slm.
- **Active region** (10 points): 6.40% through 10.00% — roughly 15–20 slm spacing.
- **Saturation** (2 points): 10.60% and 11.20% — flow flattens at ~195 slm.

### Exp Valve Reduction Notes

For the exp valve, the *result* is Paw (mbar) and the *modifier* (flow) varies during the sweep. When reducing:

- Keep points at roughly equal **Paw spacing**, not equal V% spacing.
- The Cv annotation on each point helps identify the cracking region (Cv jumps from 0 to a finite value).
- Flow variation is normal — it's recorded per-point and the average flow is used as the band condition variable.

### Automated Reduction Script

The manual process above can be automated with the Python helper script in `tools/reduce_cc_sweep.py`. It parses raw CC serial output, detects the actuator type, applies the region-based reduction, and outputs ready-to-paste C++ code with `setFlowBand()` / `setPressureBand()` calls.

**From saved files** (one file per sweep):
```
python tools/reduce_cc_sweep.py sweep_3bar.txt sweep_4bar.txt sweep_6bar.txt
```

**Interactive** (paste each sweep when prompted):
```
python tools/reduce_cc_sweep.py
```

**Options:**
| Flag | Default | Description |
|:-----|:-------:|:------------|
| `--max-points N` | 16 | Maximum points per band |
| `--normalize` | off | Correct insp valve flows for supply pressure droop (see Section 5c) |
| `--help` | — | Show usage |

The script auto-detects insp/exp/blower from the CSV header, sorts bands by ascending condition, reports the maximum interpolation error for each band, and prints the final C++ block.

---

## 5b. From CC Sweeps to Runtime Surface — Worked Example

This section walks through the complete data pipeline: you've run 3–4 CC sweeps, now what?

### Overview

```
CC sweep 1 (low condition)  ──► raw CSV (71–101 pts) ──► reduce to ≤16 pts ──► Band 0
CC sweep 2 (mid condition)  ──► raw CSV              ──► reduce to ≤16 pts ──► Band 1
CC sweep 3 (high condition) ──► raw CSV              ──► reduce to ≤16 pts ──► Band 2
                                                                                 │
                                                     PressureBandedTable ◄───────┘
                                                     (bilinear interpolation at runtime)
```

Each CC sweep at a different operating condition becomes **one band** in the `PressureBandedTable`. The runtime feedforward interpolates between bands using the live condition variable (supply pressure, flow, or counter pressure).

### Inspiratory Valve Example (Air, 3 sweeps)

#### 1. Run the sweeps

Set the supply regulator to three different pressures and run CC1 at each:

| Sweep | Regulator Setting | Command | Avg Psupply (from CSV) |
|:-----:|:-----------------:|:--------|:----------------------:|
| A | ~3 bar | `CC1,14,0.2,250` | 302 kPa |
| B | ~4.5 bar | `CC1,14,0.2,250` | 451 kPa |
| C | ~6 bar | `CC1,14,0.2,250` | 604 kPa |

Each sweep prints ~71 CSV rows plus a code-ready table.

#### 2. Record the condition variable

The **average Psupply** reported in the table header comment becomes the band's condition value. This average accounts for regulator droop during the sweep.

```
// Avg Psupply ~ 302 kPa (3.02 bar)   ← this is the condition value for Band 0
// Avg Psupply ~ 451 kPa (4.51 bar)   ← Band 1
// Avg Psupply ~ 604 kPa (6.04 bar)   ← Band 2
```

#### 3. Reduce each sweep

Apply the Section 5 method to each of the three raw tables independently:

| Sweep | Raw Points | → | Reduced Points | Dead Zone | Cracking | Active | Saturated |
|:-----:|:----------:|:-:|:--------------:|:---------:|:--------:|:------:|:---------:|
| A (302 kPa) | 71 | → | 14 | 2 | 2 | 9 | 1 |
| B (451 kPa) | 71 | → | 15 | 2 | 2 | 10 | 1 |
| C (604 kPa) | 71 | → | 16 | 2 | 2 | 10 | 2 |

Each reduced table has its own region boundaries — the dead zone may end at a different V% at different pressures (higher pressure → more force needed to crack open → higher cracking V%).

#### 4. Assemble the surface

Load the three reduced tables as bands **in ascending condition order**:

```cpp
// ───── Band 0: 302 kPa ─────
static const LookupPoint airFlowBand0[] = {
    { 0.00f,    0.0f},
    { 5.20f,    0.0f},
    { 5.60f,    0.35f},
    { 6.00f,    2.14f},
    { 6.40f,    6.89f},
    { 6.80f,   14.02f},
    { 7.20f,   23.51f},
    { 7.60f,   35.10f},
    { 8.00f,   49.22f},
    { 8.40f,   64.70f},
    { 8.80f,   82.15f},
    { 9.20f,   98.40f},
    { 9.60f,  108.20f},
    {10.00f,  110.50f},   // Saturated
};

// ───── Band 1: 451 kPa ─────
static const LookupPoint airFlowBand1[] = {
    { 0.00f,    0.0f},
    { 5.00f,    0.0f},
    { 5.40f,    0.19f},
    { 5.80f,    1.05f},
    { 6.20f,    4.32f},
    { 6.60f,   11.08f},
    { 7.00f,   21.50f},
    { 7.40f,   34.80f},
    { 7.80f,   50.22f},
    { 8.20f,   68.90f},
    { 8.60f,   89.10f},
    { 9.00f,  112.30f},
    { 9.40f,  138.50f},
    { 9.80f,  155.20f},
    {10.20f,  156.80f},   // Saturated
};

// ───── Band 2: 604 kPa ─────
static const LookupPoint airFlowBand2[] = {
    { 0.00f,    0.0f},
    { 5.00f,    0.0f},
    { 5.60f,    0.17f},
    { 6.00f,    0.91f},
    { 6.40f,    4.08f},
    { 6.80f,   12.89f},
    { 7.20f,   22.48f},
    { 7.60f,   34.40f},
    { 8.00f,   48.91f},
    { 8.40f,   68.54f},
    { 8.80f,   84.81f},
    { 9.20f,  103.18f},
    { 9.60f,  127.14f},
    {10.00f,  160.04f},
    {10.60f,  194.88f},
    {11.20f,  195.50f},   // Saturated
};

// Load into PressureBandedTable — ascending pressure order
_airValve.setFlowBand(0, 302.0f, airFlowBand0, 14);
_airValve.setFlowBand(1, 451.0f, airFlowBand1, 15);
_airValve.setFlowBand(2, 604.0f, airFlowBand2, 16);
```

#### 5. What happens at runtime

When the ventilator requests e.g. **50 slm** and the live Psupply reads **500 kPa**:

1. `findBrackets(500)` → band 1 (451 kPa) and band 2 (604 kPa), $t = \frac{500-451}{604-451} = 0.32$
2. Band 1 `inverseLookup(50)` → interpolate between `{7.80, 50.22}` and neighbours → **V% ≈ 7.79%**
3. Band 2 `inverseLookup(50)` → interpolate between `{8.00, 48.91}` and `{8.40, 68.54}` → **V% ≈ 8.02%**
4. Blend: $V\% = 7.79 + 0.32 \times (8.02 - 7.79) = \mathbf{7.86\%}$

The LLC sends 7.86% as the feedforward, and the PI controller trims any residual error.

### Blower Example (4 sweeps)

#### 1. Run the sweeps

Apply different downstream resistances and sweep PWM%:

| Sweep | Resistance | Command | Avg Paw (from CSV) |
|:-----:|:----------:|:--------|:-------------------:|
| A | Open tube | `CC4,100,2,300` | 2 mbar |
| B | Mild restriction | `CC4,100,2,300` | 8 mbar |
| C | Medium restriction | `CC4,100,2,300` | 15 mbar |
| D | High restriction | `CC4,100,2,300` | 25 mbar |

#### 2. Reduce each sweep

The blower curve is generally smoother than a solenoid valve (no cracking threshold), so fewer points in the knee and more evenly spaced:

| Sweep | Raw Points | → | Reduced Points |
|:-----:|:----------:|:-:|:--------------:|
| A (2 mbar) | 51 | → | 12 |
| B (8 mbar) | 51 | → | 13 |
| C (15 mbar) | 51 | → | 14 |
| D (25 mbar) | 51 | → | 14 |

#### 3. Assemble and load

```cpp
_blower.setFlowBand(0,  2.0f, blowerFlowBand_P2,  12);
_blower.setFlowBand(1,  8.0f, blowerFlowBand_P8,  13);
_blower.setFlowBand(2, 15.0f, blowerFlowBand_P15, 14);
_blower.setFlowBand(3, 25.0f, blowerFlowBand_P25, 14);  // max 4 bands
```

At runtime, the feedforward uses live Paw (from ELVH) to interpolate between these bands.

### Key Points

- **Band order matters** — always load in **ascending** condition order. The `findBrackets()` algorithm assumes sorted bands.
- **Number of bands** — `PressureBandedTable` supports up to **4 bands**. Two bands give linear interpolation; three or more give better accuracy in the mid-range. Four is the maximum.
- **Bands can have different point counts** — each band is an independent `PiecewiseLinearTable`. One band can have 12 points and another 16.
- **Bands can have different V% ranges** — the dead zone and saturation V% shift with pressure. Don't force all bands to use the same x-axis values.
- **Condition value** — use the **average** from the CC header comment (e.g., "Avg Psupply ~ 604 kPa"), *not* the regulator setting. The average reflects actual conditions during the sweep including droop.
- **Normalization** — for inspiratory valves, use `--normalize` to correct for supply pressure droop before reduction (see Section 5c). This significantly improves feedforward accuracy at high flows.
- **Verification** — after loading the surface, run a quick `CC1` sweep at an intermediate pressure (e.g., 5 bar). Compare the feedforward prediction (`inverseLookup`) against the measured flow. The residual should be small enough for the PI controller to handle (typically <5% of full scale).

---

## 5c. Pressure Droop Normalization

### The Problem

During a CC sweep of an inspiratory valve, the supply pressure drops progressively as the valve opens wider and draws more flow. A typical 6-bar regulator droops from ~604 kPa at the cracking point to ~376 kPa at full opening — a 38% reduction.

This means the flow values in the raw CSV are measured at **decreasing** pressures, not at the constant reference pressure assigned to the band. The band sees:

| Region | Actual Psupply | Droop | Effect on Flow Data |
|:-------|:---------------|:------|:--------------------|
| Dead zone | ≈ nominal | ~0% | No effect (flow = 0) |
| Cracking | ≈ nominal | <2% | Negligible |
| Active (mid) | below nominal | 10–25% | Flows 5–15% low |
| Active (high) / Saturation | well below nominal | 25–40% | Flows 15–25% low |

At runtime, when the actual supply pressure is near the nominal (reference) value, the feedforward will **undershoot** because the table was learned at a lower actual pressure.

### The Fix: Per-Point Flow Normalization

The orifice (Cv) equation gives:

$$Q = C_v \sqrt{\Delta P}$$

At a given valve opening the $C_v$ is constant, so flow scales with the square root of supply pressure. We can correct each measured flow to what it would be at the reference pressure:

$$Q_{norm} = Q_{meas} \times \sqrt{\frac{P_{ref}}{P_{actual}}}$$

where:
- $Q_{meas}$ = flow measured during the sweep (from CSV column 0)
- $P_{actual}$ = supply pressure at that point (from CSV column 2)
- $P_{ref}$ = band reference pressure (average Psupply)

### Region-by-Region Impact

| Region | Correction Factor | Why |
|:-------|:------------------|:----|
| **Dead zone** | n/a | Flow is zero — 0 × anything = 0 |
| **Cracking** | ≈ 1.00 | Psupply hasn't drooped yet (nearly no flow) |
| **Active** | 1.05–1.15 | The correction this feature is built for |
| **Saturation** | 1.15–1.25 | Valve fully open = orifice; $Q \propto \sqrt{P}$ still holds |

**Cracking accuracy is preserved.** Since there is virtually no flow in the cracking region, the supply pressure is still at its set value and the correction factor is ≈ 1.0. The cracking V% and the cracking flow are unchanged after normalization. This is critical for fast valve opening — an accurate cracking point lets the feedforward jump directly past the dead zone without waiting for the integrator.

### Usage

```
python tools/reduce_cc_sweep.py --normalize sweep_3bar.txt sweep_6bar.txt
```

The script reports the number of points corrected and the magnitude of the largest correction for each sweep:

```
  Pressure droop normalization enabled (--normalize)
    Air (cond=302):  48 points corrected,  max Δ = 18.42 slm
    Air (cond=604):  55 points corrected,  max Δ = 27.31 slm
```

### When to Use — and Why Only Inspiratory Valves

The three actuator types have fundamentally different physics, so "pressure droop" means different things for each:

| Actuator | Condition Variable | What Happens During Sweep | Normalization? |
|:---------|:-------------------|:--------------------------|:---------------|
| **Insp valve** (CC1/CC2) | Psupply (kPa) | Regulator sags under load — $P_{supply}$ drops 30–40% at high flow. This is **measurement error** (the table is labelled at the average pressure, but high-flow points were measured at much lower pressure). | **Yes** — $Q_{norm} = Q_{meas} \times \sqrt{P_{ref}/P_{actual}}$ corrects each point to the band's reference pressure. The orifice equation ($Q = C_v\sqrt{\Delta P}$) is the physical basis. |
| **Blower** (CC4) | Paw (mbar) | Paw *rises* as the blower speeds up — this is the fan curve (back-pressure increases with flow through a fixed restriction). Each sweep at a different downstream restriction captures a **true operating point**, not a measurement artefact. | **No** — the Paw variation is the physics being measured. Different restriction levels give different bands; within each band, the Paw is a consequence of the flow, not a disturbance. |
| **Exp valve** (CC3) | Flow (slm) | Flow is actively held constant by the `LF` controller during the sweep. The condition variable (flow) is *regulated*, not passively drooping. | **No** — the LF controller keeps the flow setpoint; any small flow fluctuations are not systematic droop. |

The script handles this automatically: `normalize_flows()` checks the sweep type and **skips** blower and exp valve data. When you run:

```
python tools/reduce_cc_sweep.py --normalize insp_sweep.txt blower_sweep.txt exp_sweep.txt
```

the output will show:

```
  Pressure droop normalization enabled (--normalize)
    Air (cond=302):  48 points corrected,  max correction = 18.42 slm
    Blower (cond=8):  skipped (not insp valve)
    Exp (cond=20):  skipped (not insp valve)
```

### What Does *Not* Change

- **Runtime code** — `PressureBandedTable`, `inverseLookup()`, and the PI controllers are unmodified. The normalization is purely a data-preparation step.
- **Band reference pressure** — still the average Psupply from the CC header. Only the flow values are adjusted.
- **Calibration workflow** — identical to before; just add `--normalize` to the script invocation.

---

## 6. Updating the Software with Measured Values

All placeholder values live in the `LocalValveController::setDefaults()` method in `src/LocalValveController.cpp`.

### Step 1: Update Cracking Current Parameters

Find the placeholder section (around line 346):

```cpp
config.airValve.crackBaseOffset_A = 0.10f;        // PLACEHOLDER
config.airValve.crackPressCoeff_A_per_bar = 0.05f; // PLACEHOLDER
```

Replace with your measured values. Example if cracking was measured at 0.15 A at 0 bar and 0.40 A at 5 bar:

$$k = \frac{0.40 - 0.15}{5.0 - 0.0} = 0.05 \text{ A/bar}$$

```cpp
config.airValve.crackBaseOffset_A = 0.15f;
config.airValve.crackPressCoeff_A_per_bar = 0.05f;
```

Repeat for O2 valve (if gains differ from air):

```cpp
config.o2Valve.crackBaseOffset_A = 0.12f;
config.o2Valve.crackPressCoeff_A_per_bar = 0.04f;
```

And for the expiratory valve (pressure-independent):

```cpp
config.expValve.crackBaseOffset_A = 0.10f;  // PLACEHOLDER
```

### Step 2: Replace the Flow Surface Tables

Find the pressure-banded flow tables section in `setDefaults()` (search for `PRESSURE-BANDED FLOW TABLES`). Each band is a `static const LookupPoint[]` array loaded via `setFlowBand()`.

For each CC sweep you ran at a different supply pressure, create a reduced table (≤16 points) and load it as a band. Bands **must** be loaded in ascending pressure order.

```cpp
// Band 0: measured at ~3 bar (P_ref = 302 kPa)
static const LookupPoint airFlowBand0[] = {
    { 0.00f,    0.0f},    // Dead zone
    { 5.00f,    0.0f},    // Still closed
    { 5.40f,    0.23f},   // Just cracking
    { 5.80f,    1.38f},
    { 6.00f,    3.06f},
    // ... 8-10 active region points ...
    { 9.60f,  110.56f},
    {10.00f,  111.79f},   // Saturated
    {14.00f,  112.39f},   // Flat at max
};

// Band 1: measured at ~4.5 bar (P_ref = 451 kPa)
static const LookupPoint airFlowBand1[] = { /* ... */ };

// Band 2: measured at ~6 bar (P_ref = 604 kPa)
static const LookupPoint airFlowBand2[] = { /* ... */ };

_airValve.setFlowBand(0, 302.0f, airFlowBand0, count0);
_airValve.setFlowBand(1, 451.0f, airFlowBand1, count1);
_airValve.setFlowBand(2, 604.0f, airFlowBand2, count2);
```

**O2 valve**: If not yet characterized, use the same air tables:
```cpp
_o2Valve.setFlowBand(0, 302.0f, airFlowBand0, count0);
_o2Valve.setFlowBand(1, 451.0f, airFlowBand1, count1);
_o2Valve.setFlowBand(2, 604.0f, airFlowBand2, count2);
```

Once you run `CC2` sweeps, create separate O2 tables.

### Step 3: Replace the Expiratory Pressure Surface Tables

Use your `CC3` sweep results (see Section 4c). Each sweep at a different flow rate becomes one band in the expiratory `PressureBandedTable`. Find the table section (search for `expPressureTable`) and replace with reduced (≤ 16 point) tables:

```cpp
// Band 0: measured at ~5 slm
static const LookupPoint expPressureBand_F5[] = {
    {0.00f,   0.12f},
    {2.00f,   1.85f},
    // ... active region ...
    {12.00f, 38.50f},
};

// Band 1: measured at ~20 slm
static const LookupPoint expPressureBand_F20[] = { /* ... */ };

// Band 2: measured at ~40 slm
static const LookupPoint expPressureBand_F40[] = { /* ... */ };

_expValve.setPressureBand(0,  5.0f, expPressureBand_F5,  count0);
_expValve.setPressureBand(1, 20.0f, expPressureBand_F20, count1);
_expValve.setPressureBand(2, 40.0f, expPressureBand_F40, count2);
```

### Step 3b: Replace the Blower Flow Surface Tables

Use your `CC4` sweep results (see Section 4b). Each sweep at a different counter pressure becomes one band. Find the blower table section (search for `blowerFlowTable`) and replace:

```cpp
// Band 0: measured at ~5 mbar Paw
static const LookupPoint blowerFlowBand_P5[] = {
    {0.00f,   0.0f},
    {10.00f,  2.34f},
    // ... active region ...
    {100.00f, 85.12f},
};

// Band 1: measured at ~15 mbar Paw
static const LookupPoint blowerFlowBand_P15[] = { /* ... */ };

_blower.setFlowBand(0,  5.0f, blowerFlowBand_P5,  count0);
_blower.setFlowBand(1, 15.0f, blowerFlowBand_P15, count1);
```

### Step 4: Tune PI Gains

After the flow surface is populated, the PI controllers provide fine corrections around the feedforward operating point. Start with conservative gains and increase:

| Parameter | Starting Value | Description |
|:----------|:--------------:|:------------|
| `flowPI.kp` | 0.05 | Proportional gain for flow error (A/slm) |
| `flowPI.ki` | 0.5  | Integral gain for flow error (A/(slm·s)) |
| `pressurePI.kp` | 0.1 | Proportional gain for pressure error (A/mbar) |
| `pressurePI.ki` | 1.0 | Integral gain for pressure error (A/(mbar·s)) |

### Step 5: Build and Test

1. Save your changes.
2. Build: `pio run`
3. Flash: `pio run --target upload`
4. Enable the LLC: `LE1`
5. Monitor LLC status: `LS`
6. Run the ventilator and observe tracking performance.

---

## 7. Tips
- Use slow V% ramps (smaller step, longer settle) for higher accuracy.
- **Inspiratory valves:** Always run sweeps at 2–3 different supply pressures to populate the 2D flow surface.
- **Blower:** Run sweeps at 2–3 different counter pressures (use partially closed exp valve or restriction).
- **Exp valve:** Run sweeps at 2–3 different flow rates via `LF`. Stop the flow (`LX`) when done.
- Supply pressure droop during a sweep is normal — each data row records the actual pressure.
- Document ambient conditions (temperature, humidity).
- Save raw serial output to a text file for future reference.
- The `PiecewiseLinearTable` interpolates linearly between points — more points in the "knee" region improves accuracy.
- After updating tables, re-run a quick `CC` sweep to verify the feedforward matches.
- The feedforward extrapolates outside the stored pressure range — keep bands covering your expected operating pressures (typically 200–600 kPa).
- Blower characterization uses GPIO21 PWM directly — no MUX command is sent.

### Low-Pressure Extrapolation

The `PressureBandedTable` extrapolates linearly below the lowest band and above the highest band. For example, if the lowest band is at 302 kPa and the live supply pressure drops to 200 kPa, the algorithm uses bands 0 and 1 with a **negative** interpolation weight $t$:

$$t = \frac{P_{live} - P_{band0}}{P_{band1} - P_{band0}} = \frac{200 - 302}{451 - 302} \approx -0.68$$

This gives sensible results (higher V% for lower pressure — correct trend), but accuracy degrades the further you extrapolate. During the 3-bar sweep, the supply droops to ~186 kPa at full flow, where the valve is already saturated. The PI controllers will compensate for extrapolation error, but the feedforward will be less precise.

**Recommendation:** If your application regularly operates below the lowest characterized pressure (e.g., weak regulators that droop below 250 kPa under load), add a **4th band** at ~200 kPa by running `CC1` with the regulator set to ~2 bar. The `PressureBandedTable` supports up to 4 bands, and the extra band significantly improves feedforward accuracy at low pressures.

---

## 8. Quick Reference

| Command | Description |
|:--------|:------------|
| `CC1` | Characterize air valve (defaults: 0–12 V%, 0.1% step) |
| `CC2` | Characterize O2 valve (defaults: 0–12 V%, 0.1% step) |
| `CC3` | Characterize exp valve (defaults: 0–12 V%, 0.1% step) |
| `CC4` | Characterize blower (defaults: 0–100 PWM%, 1% step) |
| `CC1,14,0.2,250` | Air valve, max 14%, 0.2% step, 250ms settle |
| `CC4,100,2,300` | Blower, max 100%, 2% step, 300ms settle |
| `CC3,12,0.1,200` | Exp valve, 0.1% step (set flow with LF first) |
| `CX` | Abort sweep, output → 0 |
| `LF<air>[,<o2>]` | Set constant flow for exp valve testing |
| `LX` | Stop flow test |
| `LE1` | Enable local valve control |
| `LE0` | Disable local valve control (pass-through) |
| `LS` | Print LLC status and config |

### Files to Edit

| File | What to Change |
|:-----|:---------------|
| `src/LocalValveController.cpp` | Flow surface tables, cracking current, PI gains (in `setDefaults()`) |
| `include/LocalValveController.hpp` | `PiecewiseLinearTable`, `PressureBandedTable`, controller class definitions |
| `src/ValveCharacterizer.cpp` | Sweep state machine and output formatting |
| `include/ValveCharacterizer.hpp` | Characterizer config structs |

---

**Accurate calibration is essential for safe and precise ventilator control.**