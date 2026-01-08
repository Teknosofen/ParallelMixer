# ABP2 Asynchronous Pressure Sensor Usage

## Problem with Old Code

The old `readSupplyPressure()` method had **delays** inside the measurement loop, which:
- Blocked the main loop
- Prevented precise timing control
- Always returned the same cached reading until reboot

## New Asynchronous API

Three new methods enable proper asynchronous operation:

### 1. `startABP2Measurement()`
Sends the Output Measurement Command (0xAA 0x00 0x00) to trigger a new conversion.

**Returns:** `true` if command sent successfully, `false` on I2C error

### 2. `readABP2Pressure(float& pressure_kpa, uint8_t& status_byte)`
Reads the 7-byte result (status + 24-bit pressure + 24-bit temperature).

**Parameters:**
- `pressure_kpa` - Output parameter for pressure in kPa
- `status_byte` - Output parameter for raw status byte

**Returns:** `true` if read successful and data valid, `false` on error

**Important:** NO DELAYS inside this function - caller controls timing!

### 3. `isABP2Busy()`
Quickly checks the busy flag (bit 5 of status byte) without reading full data.

**Returns:** `true` if sensor is busy converting, `false` if ready

## Usage Pattern in Main Loop

### Option 1: Alternating Command/Read at 200Hz Loop (100Hz Effective Sampling)

```cpp
void loop() {
  static unsigned long lastTime = 0;
  static uint8_t iteration = 0;
  unsigned long currentTime = micros();

  // 200 Hz loop (5ms = 5000us)
  if (currentTime - lastTime >= 5000) {
    lastTime = currentTime;

    if (iteration % 2 == 0) {
      // Even iterations: Start measurement
      sensorReader.startABP2Measurement();
    } else {
      // Odd iterations: Read result (5ms after command)
      float pressure;
      uint8_t status;
      if (sensorReader.readABP2Pressure(pressure, status)) {
        // Use pressure value
        Serial.printf("Pressure: %.2f kPa\n", pressure);
      }
    }

    iteration++;
  }
}
```

### Option 2: State Machine with Busy Polling

```cpp
void loop() {
  static unsigned long lastTime = 0;
  static enum { IDLE, WAIT_CONVERSION } state = IDLE;
  unsigned long currentTime = micros();

  // 200 Hz loop
  if (currentTime - lastTime >= 5000) {
    lastTime = currentTime;

    switch (state) {
      case IDLE:
        // Start new measurement
        if (sensorReader.startABP2Measurement()) {
          state = WAIT_CONVERSION;
        }
        break;

      case WAIT_CONVERSION:
        // Check if ready
        if (!sensorReader.isABP2Busy()) {
          float pressure;
          uint8_t status;
          if (sensorReader.readABP2Pressure(pressure, status)) {
            // Use pressure value
            Serial.printf("Pressure: %.2f kPa\n", pressure);
          }
          state = IDLE;
        }
        break;
    }
  }
}
```

### Option 3: Simple Time-Based (Recommended)

```cpp
void loop() {
  static unsigned long lastCmdTime = 0;
  static unsigned long lastReadTime = 0;
  unsigned long currentTime = micros();

  // Send command every 10ms (100Hz)
  if (currentTime - lastCmdTime >= 10000) {
    sensorReader.startABP2Measurement();
    lastCmdTime = currentTime;
  }

  // Read result 5ms after each command
  if (currentTime - lastCmdTime >= 5000 && currentTime - lastReadTime >= 10000) {
    float pressure;
    uint8_t status;
    if (sensorReader.readABP2Pressure(pressure, status)) {
      // Use pressure value
      Serial.printf("Pressure: %.2f kPa\n", pressure);
    }
    lastReadTime = currentTime;
  }

  // Flow sensor and valve control run at their own rates
  // ...
}
```

## Status Byte Interpretation

The status byte is parsed according to Table 20 from the datasheet:

| Bit | Name | Meaning |
|-----|------|---------|
| 7 | Always 0 | Reserved |
| 6 | Power | 1 = powered, 0 = not powered |
| 5 | Busy | 1 = conversion in progress |
| 4-3 | Always 0 | Reserved |
| 2 | Memory | 0 = integrity OK, 1 = failed |
| 1 | Always 0 | Reserved |
| 0 | Saturation | 1 = math saturation occurred |

**Normal status:** 0x40 (powered, not busy, no errors)

## Conversion Time

According to the ABP2 datasheet:
- **Typical conversion time:** 2-5ms
- **Recommended:** Wait at least 5ms between command and read
- **Alternative:** Poll `isABP2Busy()` until false

## Why This Fixes the "Same Reading" Problem

The old code:
1. Never sent the measurement command (0xAA)
2. Always read the same cached data from the sensor
3. Only got new data after power cycle (sensor auto-measures once at startup)

The new code:
1. Explicitly triggers each conversion with `startABP2Measurement()`
2. Waits for conversion (via timing in main loop)
3. Reads fresh data with `readABP2Pressure()`
4. Gets new pressure values every cycle!

## Migration from Old Code

**Old code:**
```cpp
void SensorReader::update(SensorData& data) {
  data.supply_pressure = readSupplyPressure();  // Had delays inside!
}
```

**New code in your main loop:**
```cpp
// In setup():
sensorReader.initialize();

// In loop():
static unsigned long lastPressureCmd = 0;
static unsigned long lastPressureRead = 0;

// Trigger measurement at 100Hz
if (micros() - lastPressureCmd >= 10000) {
  sensorReader.startABP2Measurement();
  lastPressureCmd = micros();
}

// Read result 5ms later
if (micros() - lastPressureCmd >= 5000 &&
    micros() - lastPressureRead >= 10000) {
  float pressure;
  uint8_t status;
  if (sensorReader.readABP2Pressure(pressure, status)) {
    sensorData.supply_pressure = pressure;
  }
  lastPressureRead = micros();
}
```

## Benefits

✅ No delays in measurement functions
✅ Main loop controls all timing
✅ Fresh pressure readings every cycle
✅ Can run flow sensor and valve control at different rates
✅ Precise timing control for control loops
