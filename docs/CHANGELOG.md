# ParallelMixer Changelog

All notable changes to the ParallelMixer project, condensed from individual implementation notes.

---

## Percentage-Based Interface (3-Phase Refactoring)

Migrated the entire command → actuator → hardware pipeline from raw 12-bit DAC units (0-4095) to human-readable percentages (0.0-100.0%).

### Phase 1: CommandParser Percentage Input
- Commands `O`, `A`, `V` now accept float percentages (e.g., `V33.33`)
- Input validation and clamping to 0-100%
- Help text and `!` settings display updated to show percentages
- **Files**: `src/CommandParser.cpp`, `include/CommandParser.hpp`

### Phase 2: ActuatorControl Hardware Abstraction
- `SignalGeneratorConfig` stores `float offset/amplitude` (was `uint16_t`)
- `setValveControlSignal(float percent)` / `getValveControlSignal()` return percentages
- Added private `percentToHardware()` / `hardwareToPercent()` helpers
- All signal generators (sine, step, triangle, sweep) work in percentage domain
- Hardware conversion (0-4095) now encapsulated solely in `ActuatorControl`
- **Files**: `include/ActuatorControl.hpp`, `src/ActuatorControl.cpp`

### Phase 3: Cleanup
- Removed hardware conversion code from CommandParser (was duplicated)
- Simplified query commands — no conversion needed since actuator returns percentages
- Updated `printSettings()` signature to accept float percentages
- **Design principle**: "Knowledge about resolution and communication method (PWM/I2C) is contained in the actuator methods."

---

## Serial Communication Refactoring

### SerialActuatorReader Extraction
- Extracted serial communication from `ActuatorControl` into standalone `SerialActuatorReader` class
- Non-blocking, async reception via `update()` — never blocks the main loop
- Dual storage: separate for current (`I`) and misc commands
- Timestamp tracking and staleness detection
- `ActuatorControl` forwards serial methods to `SerialActuatorReader` (backward compatible)
- **Files**: `include/SerialActuatorReader.hpp`, `src/SerialActuatorReader.cpp`, `src/ActuatorControl.cpp`

### SerialMuxRouter (Successor)
- Replaced `SerialActuatorReader` with `SerialMuxRouter` for multi-device communication
- Addresses 0-5 with optional prefix digit for MUX routing
- Per-address measurement storage: current, RPM, flow, pressure
- Per-address timestamps and stale data detection
- Generic `sendCommand(address, cmd, value)` interface
- **Files**: `include/SerialMuxRouter.hpp`, `src/SerialMuxRouter.cpp`

---

## ABP2 Asynchronous Pressure Reading

- Replaced blocking `readSupplyPressure()` (had internal delays) with async state machine
- New methods: `startABP2Measurement()`, `readABP2Pressure()`, `isABP2Busy()`
- No delays inside measurement functions — caller controls timing
- Added `PressSamplTime` to `SystemConfig` for configurable sampling rate
- State machine alternates between command and read each cycle
- Effective rate: configurable (default 10 Hz at 100 ms interval)
- **Files**: `src/main.cpp`, `src/SensorReader.cpp`, `include/SensorReader.hpp`, `include/CommandParser.hpp`

---

## SFM3505 Flow Sensor Fixes

### Byte Order Fix
- Fixed incorrect byte extraction order — CRC bytes are interspersed, not appended
- Correct order: `[Air MSB][Air MID][CRC1][Air LSB][O2 MSB][CRC2][O2 MID][O2 LSB][CRC3]`
- Simplified initialization (removed complex reset sequence)
- Matched working reference code from colleague
- No hardware changes required — purely software fix

### CRC-8 Implementation
- Added Sensirion CRC-8 validation (polynomial 0x31, init 0xFF)
- Validates 3 CRC groups per 9-byte read
- Compile-time enable/disable via `SFM3505_ENABLE_CRC_CHECK` in `include/SensorReader.hpp`
- Diagnostic output on first read and CRC errors
- **Files**: `src/SensorReader.cpp`, `include/SensorReader.hpp`

---

## SensorReader Cleanup

- Removed ~49 lines of dead/testing code from `initialize()`:
  - Commented-out I2C checks, "Akla temp override", manual 0xAA commands, 50 ms delay, obsolete diagnostic reads
- Replaced with clean I2C device detection (~11 lines)
- `update()` method deprecated for pressure reading (now handled async in `main.cpp`)
- Legacy sensors (SFM 0x40, SPD 0x25, SSC 0x58) marked as disabled
- **Files**: `src/SensorReader.cpp`

---

## Printf Refactoring

- Replaced chains of `Serial.print()` calls with `Serial.printf()` throughout
- `outputData()`: reduced from verbose multi-line prints to single `printf` per mode
- `printSettings()`: 21 lines → 11 lines
- Format specifiers explicitly show precision (`%.2f`, `%.3f`)
- **Files**: `src/main.cpp`, `src/CommandParser.cpp`

---

## ImageRenderer Cleanup

- Removed 6 unused `DisplayPos` member variables: `logoPos`, `servoIDPos`, `timePos`, `statusCOMPos`, `statusSDPos`, `phasePos`
- Removed ~18 lines of unused initialization code from `initPositions()`
- 13 remaining `DisplayPos` variables verified as actively used
- Memory savings: 48 bytes per `ImageRenderer` instance
- **Files**: `include/ImageRenderer.hpp`, `src/ImageRenderer.cpp`

---

## Control Loop Timing Analysis

Feasibility study for 2 kHz control loop operation:

| Operation | Time (µs) | % of Budget |
|-----------|-----------|-------------|
| SFM3505 read (I2C @ 400 kHz) | 250 | 94.5% |
| PID control execution | 7.5 | 2.8% |
| WiFi buffer update | 4 | 1.5% |
| Loop overhead | 3 | 1.1% |
| **Total** | **264.5** | — |

- **Available**: 500 µs (2 kHz) → **47% margin**
- **Bottleneck**: I2C sensor reads (94.5% of cycle time)
- **Optimization**: Higher I2C clock (660 kHz actual) reduces SFM3505 read to ~157 µs
- ABPD and ABP2 reads are rate-limited and have negligible average impact
- GUI/serial output loop is separate and non-interfering
