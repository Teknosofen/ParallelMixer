# SFM3505 Pull-Up Control Circuit

## Problem
The SFM3505 sensor requires SDA and SCL lines to remain LOW for at least 31ms after power-on to properly reset. If pull-up resistors are permanently connected to 3.3V, the sensor can be partially powered through internal ESD protection diodes, preventing proper initialization.

## Solution
Use GPIO21 to control an NPN transistor that switches the pull-up resistors ON/OFF.

## Circuit Diagram

```
                        +3.3V
                          │
                          ├─────────┐ (Sensor VDD - always on)
                          │         │
                    ┌─────┴─────┐   │
                    │           │   │
                  4.7kΩ       4.7kΩ │
                 (SDA PU)   (SCL PU)│
                    │           │   │
                    └─────┬─────┘   │
                          │         │
                      Collector     │
                        ──┐         │
                          │ NPN     │
                       Emitter    SFM3505
                          │       VDD pin
                         GND        │
                                    │
        GPIO21 ──── 1kΩ ──── Base   │
                                    │
        GPIO43 (SDA) ───────────────┤ SDA
                                    │
        GPIO44 (SCL) ───────────────┤ SCL
                                    │
        GND ────────────────────────┤ GND
```

## Components Required

1. **NPN Transistor**: 2N3904, 2N2222, BC547, or similar small-signal NPN
   - Collector: Connect to 3.3V
   - Base: Connect to GPIO21 via 1kΩ resistor
   - Emitter: Connect to GND

2. **Resistors**:
   - 2x 4.7kΩ pull-up resistors (SDA and SCL)
   - 1x 1kΩ base resistor (between GPIO21 and transistor base)

## Breadboard/PCB Connections

### Transistor Connections (e.g., 2N3904 in TO-92 package)
Looking at flat side with pins down:
```
   ┌───────┐
   │ Flat  │
   └───────┘
    │ │ │
    E B C
    │ │ │
    │ │ └── to +3.3V
    │ └──── to GPIO21 via 1kΩ
    └────── to GND
```

### Pull-Up Resistor Connections
```
+3.3V ──┬── 4.7kΩ ──┬── GPIO43 (SDA) ── SFM3505 SDA
        │           │
        └── 4.7kΩ ──┴── GPIO44 (SCL) ── SFM3505 SCL
```

### Control Circuit
```
GPIO21 ──── 1kΩ ──── Transistor Base
                     Transistor Emitter ──── GND
                     Transistor Collector ─── +3.3V
```

## Wiring Steps

1. **Connect the NPN transistor**:
   - Collector → +3.3V
   - Emitter → GND
   - Base → 1kΩ resistor → GPIO21

2. **Connect pull-up resistors**:
   - One end of each 4.7kΩ resistor → Transistor Collector (+3.3V rail)
   - Other end of one resistor → GPIO43 (SDA)
   - Other end of other resistor → GPIO44 (SCL)

3. **Connect SFM3505 sensor**:
   - SFM3505 VDD → +3.3V (direct, always on)
   - SFM3505 GND → GND
   - SFM3505 SDA → GPIO43 (also connected to pull-up)
   - SFM3505 SCL → GPIO44 (also connected to pull-up)

## How It Works

### Power-On Sequence (First 35ms):
1. GPIO21 = LOW → Transistor OFF → Pull-ups disconnected from 3.3V
2. GPIO43 and GPIO44 actively driven LOW
3. SFM3505 sees LOW on SDA/SCL for 35ms → Proper reset
4. Sensor is only powered via VDD pin, not through I2C lines

### Normal Operation (After 35ms):
1. GPIO21 = HIGH → Transistor ON → Pull-ups connected to 3.3V
2. GPIO43 and GPIO44 released to I2C peripheral
3. Wire.begin() initializes I2C with pull-ups active
4. Normal I2C communication

## Alternative: P-Channel MOSFET (High-Side Switch)

If you have a P-channel MOSFET (e.g., BS250, IRF9540):

```
                        +3.3V
                          │
                        Source
                        ──┤
                          │ P-MOSFET
                        Drain
                          │
                    ┌─────┴─────┐
                    │           │
                  4.7kΩ       4.7kΩ
                    │           │
                   SDA         SCL

Gate ──── 10kΩ ──── +3.3V (pull-up)
      └── GPIO21

When GPIO21 = LOW:  Gate pulled LOW → MOSFET ON  → Pull-ups active
When GPIO21 = HIGH: Gate at 3.3V    → MOSFET OFF → Pull-ups inactive
```

This inverts the logic but avoids the emitter-follower voltage drop of the NPN.

## Verification

After building the circuit, test with a multimeter:
1. **Power on ESP32**
2. **Immediately measure GPIO21**: Should be LOW
3. **Measure SDA/SCL**: Should be 0V (actively pulled LOW)
4. **Wait 3 seconds** (boot completes)
5. **Measure GPIO21**: Should be HIGH (3.3V)
6. **Measure SDA/SCL**: Should be 3.3V (pulled up via transistor)

## Troubleshooting

- **Sensor still not responding**: Check transistor polarity (E-B-C connections)
- **I2C not working after boot**: Verify pull-ups are enabled (GPIO21 = HIGH)
- **Weak pull-ups**: Use a lower value resistor (e.g., 2.2kΩ instead of 4.7kΩ)
- **Transistor gets hot**: Check base resistor is 1kΩ, not shorted

## Pin Reference

| Signal | ESP32 Pin | SFM3505 Pin | Notes |
|--------|-----------|-------------|-------|
| VDD | +3.3V | Pin 2 (Red) | Direct power, always on |
| GND | GND | Pin 3 (Black) | Common ground |
| SDA | GPIO43 | Pin 1 (Green) | I2C data, 4.7kΩ pull-up |
| SCL | GPIO44 | Pin 4 (Yellow) | I2C clock, 4.7kΩ pull-up |
| Pull-up Ctrl | GPIO21 | - | Controls transistor for pull-ups |
