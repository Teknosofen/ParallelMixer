# Valve Calibration Guide

This document describes the steps required to populate the cracking current and valve linearization tables for the P-Mixer ventilator control system, including the automated `CC` characterization command and how to integrate the results into the software.

---

## 1. Cracking Current Calibration

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

## 2. Automated Valve Characterization (CC Command)

### Overview
The `CC` serial command runs an automated voltage sweep on an inspiratory valve. It ramps voltage from 0 V to a configurable maximum in fixed steps, waits for the flow to settle at each step, samples sensors multiple times, and computes the valve's flow coefficient $Cv$ at each operating point. At the end of the sweep, it prints a code-ready lookup table that can be pasted directly into the software.

### Supported Valves
| MUX Channel | Valve | Sensors Used |
|:-----------:|:-----:|:-------------|
| 1           | Air   | Bus 0 — SFM3505 (flow), ABP2 (supply pressure), ELVH (Paw) |
| 2           | O2    | Bus 1 — SFM3505 (flow), ABP2 (supply pressure), ELVH (Paw) |

> **Note:** The expiratory valve (MUX channel 3) is **not** supported by `CC` because it uses downstream (low) pressure rather than supply pressure for its characterization.

### Command Syntax

```
CC<channel>[,maxVoltage[,stepVoltage[,settleTime_ms]]]
```

| Parameter       | Type   | Default | Description |
|:----------------|:------:|:-------:|:------------|
| `channel`       | int    | —       | **Required.** 1 = air, 2 = O2 |
| `maxVoltage`    | float  | 12.0    | Maximum voltage of the sweep (V) |
| `stepVoltage`   | float  | 0.1     | Voltage increment per step (V) |
| `settleTime_ms` | int    | 200     | Time to wait after each step before sampling (ms) |

Samples per step is fixed at **10** (averaged).

### Examples

| Command            | Description |
|:-------------------|:------------|
| `CC1`              | Air valve, 0–12 V in 0.1 V steps, 200 ms settle |
| `CC2`              | O2 valve, same defaults |
| `CC1,10`           | Air valve, 0–10 V max |
| `CC1,12,0.2,300`   | Air valve, 0.2 V steps, 300 ms settle |
| `CC2,8,0.05,500`   | O2 valve, fine steps, long settle |

### Aborting a Sweep

```
CX
```

Immediately sets the valve voltage to 0 V and returns to idle.

### What Happens During a Sweep

1. The characterizer sends `sendSetVoltage(channel, 0.0)` and begins settling.
2. At each step it:
   - Waits `settleTime_ms` for the flow to stabilize.
   - Samples flow, supply pressure, and Paw `samplesPerStep` times and averages them.
   - Computes $Cv = Q / \sqrt{P_{supply} - P_{aw}}$ (only when $\Delta P > 1\,\text{mbar}$ and $Q > 0.001\,\text{slm}$).
   - Prints a CSV data row to the serial console.
3. After the final step it:
   - Sets voltage back to 0 V.
   - Prints `# --- Sweep complete ---`.
   - Prints a **code-ready C++ lookup table** (see Section 4).

### Serial Output Format

**CSV data rows** (printed during sweep):
```
# V(V),   Flow(slm),  Psupply(mbar),  Paw(mbar),  Cv(slm/sqrt(mbar))
0.000,  0.000,      4012.3,         1.25,       0.00000
0.100,  0.000,      4010.1,         1.30,       0.00000
0.200,  0.312,      4008.5,         2.10,       0.00493
...
12.000, 58.200,     3850.0,         15.30,      0.93912
```

**Code-ready table** (printed at end):
```cpp
// ============================================================
// Air valve Cv table from characterization
// Avg Psupply ≈ 4010 mbar (4.01 bar)
// MUX channel 1, 121 points
// x = voltage (V), y = Cv (slm / sqrt(mbar))
// ============================================================
static const LookupPoint airCvTable[] = {
    {0.000f, 0.00000f},  // 0.00 slm @ 4012 mbar
    {0.100f, 0.00000f},  // 0.00 slm @ 4010 mbar
    ...
    {12.000f, 0.93912f},  // 58.20 slm @ 3850 mbar
};
// Table has 121 entries.
```

### Prerequisites
- Supply gas connected and regulated to operational pressure.
- Downstream flow path open (patient wye disconnected or open to atmosphere).
- Serial terminal connected at host baud rate.
- LLC should be **disabled** (`LE0`) so it does not interfere with the raw voltage sweep.

---

## 3. Manual Valve Characterization

### Inspiratory Valves (Air/O2)
- **Table:** Voltage (V) → Cv (slm/√mbar)
- **Procedure:**
  1. **Set supply pressure** to a typical operating value (e.g., 4 bar).
  2. **Step voltage** from 0 to max rated value in increments (e.g., 0.1 V).
  3. **Measure flow** at each voltage using a calibrated flow sensor.
  4. **Calculate Cv:** $Cv = Q / \sqrt{P_{supply} - P_{downstream}}$
  5. **Record (voltage, Cv)** pairs.
  6. **Populate** the table in code (see Section 5).

### Expiratory Valve
- **Table:** Pressure setpoint (mbar) → Current (A)
- **Procedure:**
  1. **Set up a test lung or chamber** with pressure sensor.
  2. **Step current** from just above cracking to max in increments.
  3. **Measure resulting PEEP pressure** at each current.
  4. **Record (pressure, current)** pairs.
  5. **Populate** the table in code (see Section 5).

---

## 4. Interpreting the Characterization Output

### Selecting Points for the Lookup Table

The `PiecewiseLinearTable` supports a maximum of **16 points**. A full `CC` sweep at 0.1 V steps over 12 V produces 121 data points — far more than 16. You need to select representative points:

1. **Copy** the code-ready table from the serial terminal output.
2. **Remove dead-zone rows** where Cv ≈ 0 (below cracking voltage) — keep only the first zero and the first non-zero.
3. **Remove saturation rows** where Cv stops increasing significantly — keep only the last one or two.
4. **Thin the middle region** by selecting ~10-12 evenly spaced points that capture the valve's shape (linear region, knee, saturation).
5. **Result**: 12–16 points spanning the full operating range.

### Example: Reducing 121 Points to 12

```cpp
static const LookupPoint airCvTable[] = {
    {0.000f, 0.00000f},   // Dead zone
    {1.200f, 0.00100f},   // Just opened (cracking voltage)
    {2.000f, 0.05000f},   // Low flow
    {3.000f, 0.15000f},
    {4.000f, 0.30000f},
    {5.000f, 0.48000f},
    {6.000f, 0.62000f},
    {7.000f, 0.74000f},
    {8.000f, 0.82000f},
    {9.000f, 0.88000f},   // Approaching saturation
    {10.000f, 0.92000f},
    {12.000f, 0.94000f},  // Full saturation
};
```

### Extracting the Cracking Voltage

From the CSV output, find the first row where flow becomes non-zero. The voltage at that row is the **cracking voltage**. Use this to set `crackBaseOffset_A` and `crackPressCoeff_A_per_bar` (after converting to current if your final control units are current-based) or adjust the first non-zero point in the table.

---

## 5. Updating the Software with Measured Values

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

### Step 2: Replace the Inspiratory Cv Lookup Table

Find the placeholder table (around line 381):

```cpp
// PLACEHOLDER VALVE LINEARIZATION TABLES
...
static const LookupPoint inspCvTable[] = {
    {0.00f, 0.000f},
    {0.10f, 0.050f},
    ...
    {2.00f, 2.000f},
};
_airValve.setCvTable(inspCvTable, sizeof(inspCvTable) / sizeof(inspCvTable[0]));
_o2Valve.setCvTable(inspCvTable, sizeof(inspCvTable) / sizeof(inspCvTable[0]));
```

**Option A — Same table for both valves** (if air and O2 valves are identical):

Replace the `inspCvTable[]` array contents with your reduced characterization data (≤16 points), keeping the `setCvTable()` calls the same.

**Option B — Separate tables** (if valves differ):

```cpp
static const LookupPoint airCvTable[] = {
    // Paste reduced table from CC1 output
    {0.000f, 0.00000f},
    {1.200f, 0.00100f},
    ...
};

static const LookupPoint o2CvTable[] = {
    // Paste reduced table from CC2 output
    {0.000f, 0.00000f},
    {1.100f, 0.00080f},
    ...
};

_airValve.setCvTable(airCvTable, sizeof(airCvTable) / sizeof(airCvTable[0]));
_o2Valve.setCvTable(o2CvTable, sizeof(o2CvTable) / sizeof(o2CvTable[0]));
```

### Step 3: Replace the Expiratory Pressure Lookup Table

The expiratory valve must be characterized manually (Section 3). Find the placeholder (around line 398):

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

After the lookup tables are populated, the PI controllers provide fine corrections around the feedforward operating point. Start with conservative gains and increase:

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

## 6. Tips
- Use slow voltage ramps (smaller step, longer settle) for higher accuracy.
- Repeat measurements at different supply pressures for advanced compensation.
- Document ambient conditions (temperature, humidity).
- Save raw serial output to a text file for future reference.
- The `PiecewiseLinearTable` interpolates linearly between points — more points in the "knee" region improves accuracy.
- After updating tables, re-run a quick `CC` sweep to verify the feedforward matches.

---

## 7. Quick Reference

| Command | Description |
|:--------|:------------|
| `CC1` | Characterize air valve (defaults) |
| `CC2` | Characterize O2 valve (defaults) |
| `CC1,10,0.2,300` | Air valve, max 10V, 0.2V step, 300ms settle |
| `CX` | Abort sweep, voltage → 0V |
| `LE1` | Enable local valve control |
| `LE0` | Disable local valve control (pass-through) |
| `LS` | Print LLC status and config |

### Files to Edit

| File | What to Change |
|:-----|:---------------|
| `src/LocalValveController.cpp` | Cv tables, cracking current, PI gains (in `setDefaults()`) |
| `include/LocalValveController.hpp` | Config struct definitions (if adding new parameters) |

---

**Accurate calibration is essential for safe and precise ventilator control.**