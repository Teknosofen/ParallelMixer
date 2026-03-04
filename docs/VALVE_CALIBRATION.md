# Valve Calibration Guide

This document describes the steps required to populate the cracking current and valve linearization tables for the P-Mixer ventilator control system, including the automated `CC` characterization command and how to integrate the results into the software.

> See also: [ARCHITECTURE.md — Valve Characterization & Linearization](ARCHITECTURE.md#valve-characterization--linearization) for the design rationale and data-structure details.

---

## 1. Why a 2D Flow Surface?

Proportional solenoid valves are pressure-dependent: the same drive percentage (V%) produces different flows at different supply pressures. A single 1D lookup table cannot capture this. We therefore use a **PressureBandedTable** — 2–4 piecewise-linear V% → Flow tables, each measured at a different supply pressure. The feedforward interpolates between bands at runtime using the live ABP2 supply-pressure reading.

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
The `CC` serial command runs an automated V% sweep on an inspiratory valve. It ramps V% from 0 to a configurable maximum in fixed steps, waits for the flow to settle at each step, samples sensors multiple times, and prints CSV data rows plus a code-ready flow table. Run this sweep at **2–3 different supply pressures** to populate multiple bands of the `PressureBandedTable`.

### Supported Valves
| MUX Channel | Valve | Sensors Used |
|:-----------:|:-----:|:-------------|
| 1           | Air   | Bus 0 — SFM3505 (flow), ABP2 (supply pressure), ELVH (Paw) |
| 2           | O2    | Bus 1 — SFM3505 (flow), ABP2 (supply pressure), ELVH (Paw) |

> **Note:** The expiratory valve (MUX channel 3) is **not** supported by `CC` because it uses downstream (low) pressure rather than supply pressure for its characterization.

### Command Syntax

```
CC<channel>[,maxV[,stepV[,settleTime_ms]]]
```

| Parameter       | Type   | Default | Description |
|:----------------|:------:|:-------:|:------------|
| `channel`       | int    | —       | **Required.** 1 = air, 2 = O2 |
| `maxV`          | float  | 12.0    | Maximum V% of the sweep |
| `stepV`         | float  | 0.1     | V% increment per step |
| `settleTime_ms` | int    | 200     | Time to wait after each step before sampling (ms) |

Samples per step is fixed at **10** (averaged).

### Examples

| Command            | Description |
|:-------------------|:------------|
| `CC1`              | Air valve, 0–12% in 0.1% steps, 200 ms settle |
| `CC2`              | O2 valve, same defaults |
| `CC1,14`           | Air valve, 0–14% max |
| `CC1,14,0.2,250`   | Air valve, 0.2% steps, 250 ms settle |
| `CC2,14,0.25,250`  | O2 valve, 0.25% steps, 250 ms settle |

### Aborting a Sweep

```
CX
```

Immediately sets the valve V% to 0 and returns to idle.

### What Happens During a Sweep

1. The characterizer sends `V0.00` to the selected MUX channel and begins settling.
2. At each step it:
   - Waits `settleTime_ms` for the flow to stabilize.
   - Samples flow, supply pressure, and Paw 10 times and averages them.
   - Computes $Cv = Q / \sqrt{P_{supply} - P_{aw}}$ as a diagnostic (not used for feedforward).
   - Prints a CSV data row to the serial console.
3. After the final step it:
   - Sets V% back to 0.
   - Prints `# --- Sweep complete ---`.
   - Prints a **code-ready C++ flow table** with `setFlowBand()` instructions.

### Serial Output Format

**CSV data rows** (printed during sweep):
```
# V(V), Flow(slm),      Psupply(mbar),  Paw(mbar),      Cv(slm/sqrt(mbar))
0.000,  0.001,          603.7,          -0.09,          0.00000
5.600,  0.173,          603.0,          -0.06,          0.00705
...
14.000, 193.885,        375.8,          19.01,          10.26461
```

**Code-ready table** (printed at end):
```cpp
// ============================================================
// Air valve flow table from characterization
// P_ref ≈ 604 mbar (initial supply pressure)
// MUX channel 1, 71 points
// x = V%, y = Flow (slm)
// ============================================================
static const LookupPoint airFlowBand_P604[] = {
    {0.000f, 0.001f},    // @ 604 mbar
    {5.600f, 0.173f},    // @ 603 mbar
    ...
};
// Usage: _airValve.setFlowBand(<bandIndex>, 604.0f, airFlowBand_P604, <count>);
```

### Prerequisites
- Supply gas connected and regulated to operational pressure.
- Downstream flow path open (patient wye disconnected or open to atmosphere).
- Serial terminal connected at host baud rate.
- LLC should be **disabled** (`LE0`) so it does not interfere with the raw V% sweep.

---

## 4. Multi-Pressure Characterization Procedure

### Goal
Capture the valve's full 2D flow surface by running sweeps at 2–3 different supply pressures.

### Recommended Steps

1. Set the supply regulator to **~3 bar**. Run:
   ```
   CC1,14,0.2,250
   ```
   Copy the printed flow table from the serial terminal. Note the initial Psupply (P_ref).

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

### Supply Pressure Droop

During a sweep, the supply pressure droops as flow increases (limited by regulator capacity). This is **expected and acceptable** — each data row records the actual supply pressure at that point. The P_ref stored per band is the initial (zero-flow) supply pressure.

---

## 5. Reducing Points & Selecting Table Data

The `PiecewiseLinearTable` supports a maximum of **16 points**. A full sweep at 0.2% steps over 14% produces 71 data points. Reduce as follows:

1. **Dead zone** — all V% values where flow ≈ 0. Keep only V% = 0 (flow = 0) and the last zero-flow point (just before cracking).
2. **Cracking region** — the first 2–3 points where flow becomes significant.
3. **Active (linear) region** — pick ~8–10 evenly spaced points spanning the full stroke.
4. **Saturation** — where flow stops increasing (valve or sensor limit). Keep 1–2 points.

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

### Step 3: Replace the Expiratory Pressure Lookup Table

The expiratory valve must be characterized manually (step current, measure PEEP). Find the table (search for `expPressureTable`):

```cpp
static const LookupPoint expPressureTable[] = {
    {0.0f,  0.00f},    // Fully open
    {2.0f,  0.10f},
    ...
    {40.0f, 1.80f},    // High PEEP / pop-off
};
```

Replace with measured `(PEEP_mbar, current_A)` pairs.

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
- **Always run sweeps at 2–3 different supply pressures** to populate the 2D flow surface.
- Supply pressure droop during a sweep is normal — each data row records the actual pressure.
- Document ambient conditions (temperature, humidity).
- Save raw serial output to a text file for future reference.
- The `PiecewiseLinearTable` interpolates linearly between points — more points in the "knee" region improves accuracy.
- After updating tables, re-run a quick `CC` sweep to verify the feedforward matches.
- The feedforward extrapolates outside the stored pressure range — keep bands covering your expected operating pressures (typically 200–600 kPa).

---

## 8. Quick Reference

| Command | Description |
|:--------|:------------|
| `CC1` | Characterize air valve (defaults) |
| `CC2` | Characterize O2 valve (defaults) |
| `CC1,14,0.2,250` | Air valve, max 14%, 0.2% step, 250ms settle |
| `CX` | Abort sweep, V% → 0 |
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