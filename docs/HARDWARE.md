# ParallelMixer Hardware Reference

**Last Updated**: 2026-02-17

This document consolidates all hardware wiring, circuit design, and configuration information for the ParallelMixer system.

---

## Table of Contents

1. [Pin Configuration Summary](#pin-configuration-summary)
2. [Pressure Sensor Configuration](#pressure-sensor-configuration)
3. [FDO2 Oxygen Sensor Wiring](#fdo2-oxygen-sensor-wiring)
4. [SFM3505 Pull-Up Control Circuit](#sfm3505-pull-up-control-circuit)

---

## Pin Configuration Summary

### T-Display S3 Pin Assignments

| GPIO | Function | Interface | Notes |
|------|----------|-----------|-------|
| 43 | I2C0 SDA | I2C Bus 0 | Flow + pressure sensors #1 |
| 44 | I2C0 SCL | I2C Bus 0 | 400 kHz (configurable) |
| 10 | I2C1 SDA | I2C Bus 1 | Flow + pressure sensors #2 |
| 11 | I2C1 SCL | I2C Bus 1 | Same clock as Bus 0 |
| 17 | Serial1 TX | MUX Router | 230400 baud |
| 18 | Serial1 RX | MUX Router | Actuator feedback |
| 12 | Serial2 TX | FDO2 O2 Sensor | 19200 baud |
| 13 | Serial2 RX | FDO2 O2 Sensor | |
| 14 | Key 1 | Button | Long=WiFi On, Short=WiFi Off |
| 0 | Key 2 (Boot) | Button | Long=Vent Settings, Short=Back |
| 15 | Display Power | Output | Must be set HIGH |
| 1, 2, 3, 21 | Available | GPIO/ADC | For future expansion |

### Reserved by Display (Cannot Use)

- GPIO 5-9: Display control
- GPIO 38: Display backlight
- GPIO 15: Display power enable
- GPIO 39-42, 45-48: Display data bus

### I2C Device Map

#### Bus 0 (Wire) — GPIO43/44

| Address | Device | Measurement |
|---------|--------|-------------|
| 0x2E | SFM3505 | Air + O2 flow (slm) |
| 0x28 | ABP2 | Supply pressure (kPa) |
| 0x28 | ABPD | Low pressure (mbar) + temp — **conflicts with ABP2** |
| 0x60 | MCP4725 | DAC output (legacy) |

#### Bus 1 (Wire1) — GPIO10/11

| Address | Device | Measurement |
|---------|--------|-------------|
| 0x2E | SFM3505 #2 | Air + O2 flow (slm) |
| 0x28 | ABP2 #2 | Supply pressure (kPa) |

### Serial MUX Channel Mapping

| MUX Ch | Enum | Device | Function |
|--------|------|--------|----------|
| 0 | `MUX_DIRECT` | Direct | No prefix (legacy mode) |
| 1 | `MUX_AIR_VALVE` | Air Valve | Air flow control (insp) |
| 2 | `MUX_O2_VALVE` | O2 Valve | O2 flow control (insp) |
| 3 | `MUX_EXP_VALVE` | Exp Valve | Expiratory/PEEP control |
| 4 | `MUX_NC` | (Reserved) | Not connected |
| 5 | `MUX_BLOWER` | Blower | Blower motor control |

---

## Pressure Sensor Configuration

### I2C Address Conflict

Both pressure sensors use the same I2C address (0x28):

- **ABP2DSNT150PG2A3XX** (High pressure) — 0-150 PSI (0-10.3 bar)
- **ABPDLNN100MG2A3** (Low pressure) — 0-100 mbar (0-1.45 PSI)

Until an I2C multiplexer is added, only **one** sensor can be used at a time per bus.

### Compile-Time Switch

In `include/SensorReader.hpp` (lines 12-13), uncomment exactly ONE:

```cpp
#define USE_ABP2_PRESSURE_SENSOR    // High pressure (default)
// #define USE_ABPD_PRESSURE_SENSOR    // Low pressure
```

| Option | Define | Data Fields | Read Method |
|--------|--------|-------------|-------------|
| ABP2 (default) | `USE_ABP2_PRESSURE_SENSOR` | `supply_pressure` (kPa) | Async (command → wait → read) |
| ABPD | `USE_ABPD_PRESSURE_SENSOR` | `abpd_pressure` (kPa), `abpd_temperature` (°C) | Direct read |

**Important:** You must recompile and upload after changing the switch.

### Future: Dual Sensor with I2C Multiplexer

When an I2C multiplexer (e.g., TCA9548A) is added:

1. Connect both sensors to different mux channels
2. Modify detection code in `SensorReader.cpp` to switch channels
3. Enable both `#define` statements
4. Read both sensors independently

---

## FDO2 Oxygen Sensor Wiring

### Connection Diagram

```
ESP32 T-Display S3          FDO2 Sensor (Molex 560020-0420)
==================          =============================

GPIO12 (TX) ────────────────→ Pin 3 (RXD)
GPIO13 (RX) ←──────────────── Pin 2 (TXD)
3.3V or 5V ─────────────────→ Pin 4 (VCC)
GND ────────────────────────→ Pin 1 (GND)
```

### FDO2 Connector Pinout

```
    ┌─────────────────┐
    │  FDO2 SENSOR    │
    └────┬─┬─┬─┬──────┘
         │ │ │ │
         1 2 3 4
         │ │ │ │
         │ │ │ └─── Pin 4: VCC (3.3-5.0V DC)
         │ │ └───── Pin 3: RXD (Data input, 3.0V UART, max 3.3V)
         │ └─────── Pin 2: TXD (Data output, 3.0V UART)
         └───────── Pin 1: GND (Ground)
```

### Voltage & Current

| Parameter | Value |
|-----------|-------|
| Power supply | 3.3-5.0V DC |
| Standby current | ~8 mA |
| Peak current | ~40 mA |
| UART logic level | 3.0V (max 3.3V) |

**ESP32 GPIO outputs are 3.3V — safe for direct connection.**
Do NOT connect 5V logic to the FDO2 RXD pin.

### Critical Wiring Notes

1. **TX/RX must be crossed**: ESP32 TX → FDO2 RX, and vice versa
2. **Common ground**: ESP32 GND must connect to FDO2 GND
3. **Venting capillary**: Small hole on FDO2 housing back must remain at atmospheric pressure — do not seal
4. **Wire length**: Keep <1 m; use shielded/twisted pair for longer runs

### Molex Connector Options

| Option | Part Number | Notes |
|--------|-------------|-------|
| Mating connector | 510210-0400 (housing) + 502578-0000 (crimp) | Most reliable |
| Direct solder | — | OK for prototyping, use strain relief |
| Pre-made cable | Varies | Check if sensor includes one |

---

## SFM3505 Pull-Up Control Circuit

### Problem

The SFM3505 sensor requires SDA and SCL lines to remain LOW for at least 31 ms after power-on to properly reset. Permanent pull-up resistors can partially power the sensor through ESD diodes, preventing proper initialization.

### Solution

Use GPIO21 to control an NPN transistor that switches the 4.7 kΩ pull-up resistors ON/OFF.

### Circuit Diagram

```
                        +3.3V
                          │
                    ┌─────┴─────┐
                  4.7kΩ       4.7kΩ
                 (SDA PU)   (SCL PU)
                    │           │
                    └─────┬─────┘
                      Collector
                        ──┐
                          │ NPN (2N3904 / BC547)
                       Emitter
                          │
                         GND

        GPIO21 ──── 1kΩ ──── Base

        GPIO43 (SDA) ──── SFM3505 SDA
        GPIO44 (SCL) ──── SFM3505 SCL
        3.3V ───────────── SFM3505 VDD
        GND ────────────── SFM3505 GND
```

### Components

| Component | Value | Notes |
|-----------|-------|-------|
| NPN transistor | 2N3904 / 2N2222 / BC547 | Small-signal NPN, TO-92 |
| Pull-up resistors | 2× 4.7 kΩ | SDA and SCL to 3.3V via collector |
| Base resistor | 1 kΩ | Between GPIO21 and transistor base |

### Transistor Pinout (TO-92, flat side facing you)

```
    E B C
    │ │ │
    │ │ └── to +3.3V (via pull-up resistors)
    │ └──── to GPIO21 via 1kΩ
    └────── to GND
```

### Power-On Sequence

| Phase | GPIO21 | Pull-ups | SDA/SCL | Duration |
|-------|--------|----------|---------|----------|
| Reset | LOW | Disconnected | Actively driven LOW | First 35 ms |
| Normal | HIGH | Connected to 3.3V | Released to I2C peripheral | After 35 ms |

### Verification

After building, test with a multimeter:

1. Power on ESP32 → GPIO21 should be LOW
2. Measure SDA/SCL → should be 0V (pulled LOW)
3. Wait 3 seconds (boot completes) → GPIO21 should be HIGH (3.3V)
4. Measure SDA/SCL → should be 3.3V (pulled up)

### Alternative: P-Channel MOSFET

If using a P-MOSFET (e.g., BS250):

- **Source** → +3.3V
- **Drain** → Pull-up resistors → SDA/SCL
- **Gate** → GPIO21 + 10 kΩ to 3.3V

Logic is inverted: GPIO21 LOW = pull-ups ON, GPIO21 HIGH = pull-ups OFF.

### SFM3505 Pin Reference

| Signal | ESP32 Pin | SFM3505 Pin | Wire Color |
|--------|-----------|-------------|------------|
| VDD | +3.3V | Pin 2 | Red |
| GND | GND | Pin 3 | Black |
| SDA | GPIO43 | Pin 1 | Green |
| SCL | GPIO44 | Pin 4 | Yellow |
| Pull-up Ctrl | GPIO21 | — | — |
