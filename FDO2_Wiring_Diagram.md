# FDO2 Wiring Diagram for ESP32 T-Display S3

## Connection Summary

```
ESP32 T-Display S3          FDO2 Sensor (Molex Connector)
==================          =============================

GPIO12 (TX) ────────────────→ Pin 3 (RXD)
GPIO13 (RX) ←──────────────── Pin 2 (TXD)
3.3V or 5V ─────────────────→ Pin 4 (VCC)
GND ────────────────────────→ Pin 1 (GND)
```

## FDO2 Molex 560020-0420 Connector

Looking at the connector on the FDO2 sensor:

```
    ┌─────────────────┐
    │  FDO2 SENSOR    │
    │                 │
    └────┬─┬─┬─┬──────┘
         │ │ │ │
         1 2 3 4
         │ │ │ │
         │ │ │ └─── Pin 4: VCC (3.3-5.0V DC)
         │ │ └───── Pin 3: RXD (Data input, 3.0V UART, max 3.3V)
         │ └─────── Pin 2: TXD (Data output, 3.0V UART)
         └───────── Pin 1: GND (Ground)
```

## Important Notes

### Voltage Levels
- **Power Supply**: 3.3V to 5.0V DC
  - Standby current: ~8 mA
  - Peak current: ~40 mA
  - Use stable power supply (shared with ESP32 is OK)

- **UART Levels**: 3.0V logic (max 3.3V)
  - ESP32 GPIO outputs are 3.3V - **SAFE** ✅
  - DO NOT connect 5V logic to RXD pin! ⚠️

### Critical Wiring Requirements

1. **TX/RX Must Be Crossed**
   - ESP32 TX (GPIO12) connects to FDO2 RX (pin 3)
   - ESP32 RX (GPIO13) connects to FDO2 TX (pin 2)

2. **Common Ground**
   - ESP32 GND must be connected to FDO2 GND (pin 1)
   - Use the same ground reference for stable communication

3. **Venting Capillary**
   - Small hole on back of FDO2 housing must be at atmospheric pressure
   - Do not seal or block this vent!

### Wire Length
- Keep wires reasonably short (<1 meter recommended)
- Use shielded cable for longer runs to prevent noise
- Twisted pair for TX/RX is helpful

## Molex Connector Options

### Option 1: Use Mating Connector
- Molex part number: 510210-0400 (4-pin housing)
- Crimp terminals: 502578-0000
- Professional solution, most reliable

### Option 2: Direct Solder (Prototype)
- Carefully solder wires directly to FDO2 pins
- Use strain relief to prevent breaking connections
- Good for testing, not recommended for production

### Option 3: Connector Cable
- Some FDO2 sensors come with pre-made cables
- Check if your sensor includes a cable

## Testing the Connection

### Visual Check
1. Verify power LED on FDO2 (if present) lights up
2. Check all connections are secure
3. Ensure no shorts between adjacent pins

### Simple Test Program
Upload `FDO2_Simple_Test.ino` to verify communication:

Expected output:
```
=== FDO2 Sensor Test ===
Serial2: RX=GPIO13, TX=GPIO12
Initializing FDO2...
✅ FDO2 Initialized!
Device ID: 8
Firmware: 3.55
Channels: 1
```

### Troubleshooting

**No Response / Initialization Hangs**
- Check TX/RX are crossed (most common mistake!)
- Verify power supply is 3.3-5V
- Measure voltage at FDO2 VCC pin
- Try swapping TX/RX if uncertain

**Garbled Data**
- Verify baud rate is 19200
- Check for loose connections
- Ensure stable ground connection
- Check for electrical noise sources nearby

**Intermittent Communication**
- Inspect solder joints / crimps
- Check for cable damage
- Verify connector is fully seated
- Add 100nF capacitor near FDO2 VCC if power is noisy

## Electrical Specifications

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Supply Voltage | 3.3 | - | 5.0 | V DC |
| Supply Current (Standby) | - | 8 | - | mA |
| Supply Current (Peak) | - | 40 | - | mA |
| UART Logic Level | - | 3.0 | 3.3 | V |
| Baud Rate | - | 19200 | - | bps |

## Additional Considerations

### Sensor Placement
- Keep sensor away from direct sunlight (causes drift)
- Avoid condensation on sensing membrane
- Protect from mechanical shock
- Ensure adequate air circulation around sensing membrane

### Environmental
- Operating temp: -10°C to 60°C (optimal: 10-40°C)
- Non-condensing humidity on connector side
- Max pressure: 20 bar absolute, 3 bar differential

### Warm-up Time
- Allow 3 minutes warm-up for full accuracy
- Reduced accuracy during first 3 minutes after power-up

## Pin Assignment Reference

```cpp
// In your code (main.cpp or FDO2_Simple_Test.ino):
const int8_t SERIAL2_RX_PIN = 13;  // GPIO13
const int8_t SERIAL2_TX_PIN = 12;  // GPIO12

// Initialization:
fdo2Sensor.begin(19200, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
```

## Quick Reference Card

Print and keep near your workbench:

```
┌───────────────────────────────────────┐
│   FDO2 TO ESP32 QUICK REFERENCE       │
├───────────────────────────────────────┤
│ FDO2 Pin 1 (GND) → ESP32 GND          │
│ FDO2 Pin 2 (TXD) → ESP32 GPIO13 (RX)  │
│ FDO2 Pin 3 (RXD) → ESP32 GPIO12 (TX)  │
│ FDO2 Pin 4 (VCC) → ESP32 3.3V or 5V   │
├───────────────────────────────────────┤
│ Baud Rate: 19200                      │
│ Logic Level: 3.3V (MAX!)              │
│ Warm-up: 3 minutes                    │
└───────────────────────────────────────┘
```
