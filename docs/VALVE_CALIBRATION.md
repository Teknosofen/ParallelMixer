# Valve Calibration Guide

This document describes the steps required to populate the cracking current and valve linearization tables for the P-Mixer ventilator control system.

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
7. **Update** `crackBaseOffset_A` and `crackPressCoeff_A_per_bar` in the config.

## 2. Valve Linearization Table Calibration

### Purpose
The valve linearization table maps control current to flow (Cv) for inspiratory valves, and control current to pressure for expiratory valves. This compensates for valve nonlinearity.

### Inspiratory Valves (Air/O2)
- **Table:** Current (A) → Cv (slm/√mbar)
- **Procedure:**
  1. **Set supply pressure** to a typical operating value (e.g., 4 bar).
  2. **Step current** from just above cracking to max rated value in increments (e.g., 0.1 A).
  3. **Measure flow** at each current using a calibrated flow sensor.
  4. **Calculate Cv:** $Cv = Q / \sqrt{P_{supply} - P_{downstream}}$
  5. **Record (current, Cv)** pairs.
  6. **Repeat** for other supply pressures if needed (for advanced compensation).
  7. **Populate** the table in code or config.

### Expiratory Valve
- **Table:** Pressure setpoint (mbar) → Current (A)
- **Procedure:**
  1. **Set up a test lung or chamber** with pressure sensor.
  2. **Step current** from just above cracking to max in increments.
  3. **Measure resulting pressure** at each current.
  4. **Record (pressure, current)** pairs.
  5. **Populate** the table in code or config.

## 3. Tips
- Use slow current ramps to avoid overshoot.
- Repeat measurements for consistency.
- Document ambient conditions (temperature, humidity).
- Save raw data for future reference.

## 4. Updating the Code
- Edit the placeholder values in `LocalValveController.cpp` and `LocalValveController.hpp`.
- For runtime configuration, consider loading tables from external files or EEPROM.

---

**Accurate calibration is essential for safe and precise ventilator control.**