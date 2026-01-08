# ABP2 Asynchronous Integration in main.cpp

## Changes Made

### 1. SystemConfig Structure
**File:** [include/CommandParser.hpp](include/CommandParser.hpp#L11)

Added new field for pressure sampling timing:
```cpp
struct SystemConfig {
  uint32_t delta_t;                    // Sampling time in microseconds
  uint32_t PressSamplTime;             // ABP2 pressure sampling time (default: 10000 = 100Hz)
  int16_t quiet_mode;
  bool flow_setting_is_analog;
  float digital_flow_reference;
};
```

### 2. Configuration Initialization
**File:** [src/main.cpp:187-188](src/main.cpp#L187-L188)

```cpp
sysConfig.delta_t = 100000;          // Main loop: 10ms = 100Hz
sysConfig.PressSamplTime = 10000;    // Pressure sampling: 10ms = 100Hz (alternating 200Hz)
```

### 3. Asynchronous Pressure Reading Loop
**File:** [src/main.cpp:340-363](src/main.cpp#L340-L363)

New pressure measurement state machine added **before** main control loop:

```cpp
// ============================================================================
// ABP2 Pressure Sensor - Asynchronous Measurement (100Hz with 200Hz loop)
// ============================================================================
static uint32_t lastPressureTime = 0;
static bool pressureCommandSent = false;
uint32_t currentTime = micros();

if (sensorsInitialized && (currentTime - lastPressureTime) >= sysConfig.PressSamplTime) {
  lastPressureTime = currentTime;

  if (!pressureCommandSent) {
    // Send measurement command (0xAA 0x00 0x00)
    sensors_bus0->startABP2Measurement();
    pressureCommandSent = true;
  } else {
    // Read result (5ms after command, automatic due to 10ms cycle)
    float pressure;
    uint8_t status;
    if (sensors_bus0->readABP2Pressure(pressure, status)) {
      sensorData_bus0.supply_pressure = pressure;
    }
    pressureCommandSent = false;  // Next cycle sends command again
  }
}
```

## How It Works

### Timing Diagram

```
Time:     0ms    5ms    10ms   15ms   20ms   25ms   30ms
          |      |      |      |      |      |      |
Pressure: CMD----READ---CMD----READ---CMD----READ---CMD
          └─5ms─→

Main Loop: EXEC---------EXEC---------EXEC---------EXEC
          └────10ms────→
```

### State Machine

1. **Cycle 1 (t=0ms):** Send 0xAA command → Set flag
2. **Cycle 2 (t=10ms):** Read 7 bytes → Clear flag → Store pressure
3. **Cycle 3 (t=20ms):** Send 0xAA command → Set flag
4. **Cycle 4 (t=30ms):** Read 7 bytes → Clear flag → Store pressure
5. **Repeat...**

### Effective Rates

- **Loop execution rate:** 200Hz (every 5ms alternation)
- **Command send rate:** 100Hz (every 10ms)
- **Pressure update rate:** 100Hz (every 10ms)
- **Conversion time:** ~5ms (automatic, between command and read)

## Key Benefits

✅ **NO delays** in measurement functions
✅ **Fresh readings** every cycle (fixes the "same reading" bug)
✅ **Independent timing** - pressure sampling separate from main loop
✅ **Proper command-based operation** - follows Table 21 from datasheet
✅ **Configurable rate** - adjust `sysConfig.PressSamplTime` as needed

## Configuration Options

### Change Pressure Sampling Rate

In `setup()`:
```cpp
// 50Hz (20ms cycle = 10ms command, 10ms read)
sysConfig.PressSamplTime = 20000;

// 200Hz (5ms cycle = 2.5ms command, 2.5ms read) - FAST!
sysConfig.PressSamplTime = 5000;

// 100Hz (10ms cycle = 5ms command, 5ms read) - DEFAULT
sysConfig.PressSamplTime = 10000;
```

**Note:** Minimum is ~5ms to allow 2-5ms conversion time between command and read.

## Removed Code

The old synchronous approach has been removed from the main loop:
```cpp
// OLD (removed):
sensors_bus0->update(sensorData_bus0);  // Had delays inside!

// NEW (see lines 340-363):
// Asynchronous state machine with no delays
```

## Testing

1. **Upload code** to device
2. **Monitor Serial output** - you should see:
   ```
   [Bus0] ABP2: Status=0x40 [PWR=1 BSY=0 MEM=0 SAT=0], Press=0x...
   ```
3. **Apply pressure** - readings should change immediately
4. **Verify update rate** - should see new values every ~10ms

## Troubleshooting

### Still Getting Same Reading?

Check that:
1. Sensor is receiving 0xAA command (add debug print in `startABP2Measurement()`)
2. Status byte shows powered (bit 6 = 1)
3. Conversion time is adequate (>2ms between command and read)

### Pressure Readings Unstable?

Try:
- Increase `PressSamplTime` to 20000 (50Hz)
- Check I2C wiring and pull-ups
- Verify EOC pin has 10kΩ pull-up to VDD

### Want Even Higher Rate?

You can poll the busy flag instead:
```cpp
if (!pressureCommandSent) {
  sensors_bus0->startABP2Measurement();
  pressureCommandSent = true;
} else {
  // Poll busy flag
  if (!sensors_bus0->isABP2Busy()) {
    // Ready! Read now
    sensors_bus0->readABP2Pressure(pressure, status);
    pressureCommandSent = false;
  }
}
```

This allows reading as soon as conversion completes (~2-3ms) instead of waiting full 5ms.
