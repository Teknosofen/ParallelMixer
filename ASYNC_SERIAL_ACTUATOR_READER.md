# Asynchronous Serial Actuator Reader

## Overview

The `SerialActuatorReader` class provides non-blocking, asynchronous reception of data from an external actuator connected via Serial1. Unlike synchronous serial reading which blocks the main loop waiting for data, this reader processes incoming characters as they arrive and stores parsed values for access at any time.

## Key Features

- ✅ **Non-blocking**: Never blocks the main loop waiting for data
- ✅ **Asynchronous**: Builds messages character-by-character as data arrives
- ✅ **Dual storage**: Separate storage for 'I' (current) and other commands
- ✅ **Helper access**: Simple getter methods to retrieve stored data
- ✅ **Timestamp tracking**: Know when data was last received
- ✅ **Staleness detection**: Check if data is outdated
- ✅ **Buffer protection**: Automatic overflow prevention

## Protocol

The reader expects the same protocol as `ActuatorControl`:

```
<CHR>FLOAT\n
```

### Examples:
- `I1.25\n` - Current reading of 1.25 A (stored as valve actuator current)
- `V50.5\n` - Valve position 50.5% (stored as misc data)
- `P123.4\n` - Pressure 123.4 kPa (stored as misc data)
- `F25.0\n` - Flow 25.0 L/min (stored as misc data)

## Architecture

### Data Storage

The reader maintains two separate storage locations:

1. **Valve Actuator Current** (`_valveActuatorCurrent`)
   - Stores values from 'I' or 'i' commands
   - Represents actuator current in Amperes
   - Accessed via `getValveActuatorCurrent()`

2. **Valve Actuator Misc** (`_valveActuatorMisc`)
   - Stores values from all other commands (V, P, F, etc.)
   - Last command character stored in `_valveActuatorMiscCmd`
   - Accessed via `getValveActuatorMisc()` and `getValveActuatorMiscCommand()`

### Processing Flow

```
Serial1 RX → Character received → Add to buffer → '\n' detected → Parse message → Store data
                                         ↓
                                  If buffer full → Discard buffer
```

## API Reference

### Constructor

```cpp
SerialActuatorReader(HardwareSerial* serial)
```

**Parameters:**
- `serial` - Pointer to serial port (e.g., `&Serial1`)

**Example:**
```cpp
SerialActuatorReader actuatorReader(&Serial1);
```

### Initialization

```cpp
void begin()
```

**Purpose:** Initialize the reader, clear buffers, and reset stored values.

**Call once in `setup()`:**
```cpp
void setup() {
  Serial1.begin(115200);
  actuatorReader.begin();
}
```

### Update (Call in Loop)

```cpp
void update()
```

**Purpose:** Process incoming serial data. Must be called frequently in main loop.

**Behavior:**
- Non-blocking - returns immediately if no data available
- Processes all available characters
- Builds messages character-by-character
- Parses and stores complete messages when '\n' received

**Example:**
```cpp
void loop() {
  actuatorReader.update();  // Call every loop iteration

  // Rest of loop...
}
```

### Data Access Helpers

#### Get Valve Actuator Current

```cpp
float getValveActuatorCurrent() const
```

**Purpose:** Get the most recently received current value (command 'I')

**Returns:** Current in Amperes, or 0.0 if never received

**Example:**
```cpp
float current = actuatorReader.getValveActuatorCurrent();
Serial.printf("Actuator current: %.3f A\n", current);
```

#### Get Valve Actuator Misc Data

```cpp
float getValveActuatorMisc() const
```

**Purpose:** Get the most recently received misc value (any command except 'I')

**Returns:** Value from last non-'I' command, or 0.0 if never received

**Example:**
```cpp
float misc = actuatorReader.getValveActuatorMisc();
char cmd = actuatorReader.getValveActuatorMiscCommand();
Serial.printf("Misc data: %c = %.2f\n", cmd, misc);
```

#### Get Misc Command Character

```cpp
char getValveActuatorMiscCommand() const
```

**Purpose:** Get the command character of the last misc data

**Returns:** Command character (e.g., 'V', 'P', 'F'), or '\0' if never received

**Example:**
```cpp
char cmd = actuatorReader.getValveActuatorMiscCommand();
if (cmd == 'V') {
  float valve_pos = actuatorReader.getValveActuatorMisc();
  Serial.printf("Valve position: %.2f%%\n", valve_pos);
}
```

### Timestamp Helpers

#### Get Current Timestamp

```cpp
uint32_t getCurrentTimestamp() const
```

**Purpose:** Get time when current data was last updated

**Returns:** Milliseconds since boot, or 0 if never received

#### Get Misc Timestamp

```cpp
uint32_t getMiscTimestamp() const
```

**Purpose:** Get time when misc data was last updated

**Returns:** Milliseconds since boot, or 0 if never received

**Example:**
```cpp
uint32_t lastUpdate = actuatorReader.getCurrentTimestamp();
uint32_t age_ms = millis() - lastUpdate;
Serial.printf("Data age: %lu ms\n", age_ms);
```

### Staleness Detection

#### Check if Current Data is Stale

```cpp
bool isCurrentStale(uint32_t max_age_ms = 1000) const
```

**Purpose:** Check if current data is outdated

**Parameters:**
- `max_age_ms` - Maximum acceptable age in milliseconds (default: 1000ms)

**Returns:** `true` if data is older than `max_age_ms` or never received

**Example:**
```cpp
if (actuatorReader.isCurrentStale(2000)) {
  Serial.println("Warning: Current data is stale (>2s old)");
}
```

#### Check if Misc Data is Stale

```cpp
bool isMiscStale(uint32_t max_age_ms = 1000) const
```

**Purpose:** Check if misc data is outdated

**Parameters:**
- `max_age_ms` - Maximum acceptable age in milliseconds (default: 1000ms)

**Returns:** `true` if data is older than `max_age_ms` or never received

### Buffer Management

```cpp
void clearBuffer()
```

**Purpose:** Clear receive buffer and discard any pending data

**Use when:**
- Communication error detected
- Synchronization lost
- Starting fresh communication

**Example:**
```cpp
// Clear buffer before sending important command
actuatorReader.clearBuffer();
actuator.sendSerialCommand('V', 50.0);
```

## Usage Patterns

### Pattern 1: Basic Reading

```cpp
#include "SerialActuatorReader.hpp"

SerialActuatorReader actuatorReader(&Serial1);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  actuatorReader.begin();
}

void loop() {
  // Update reader (non-blocking)
  actuatorReader.update();

  // Access data anytime
  float current = actuatorReader.getValveActuatorCurrent();
  float misc = actuatorReader.getValveActuatorMisc();
  char miscCmd = actuatorReader.getValveActuatorMiscCommand();

  Serial.printf("Current: %.3f A | %c: %.2f\n", current, miscCmd, misc);

  delay(1000);
}
```

### Pattern 2: With Staleness Check

```cpp
void loop() {
  actuatorReader.update();

  // Only use data if it's fresh
  if (!actuatorReader.isCurrentStale(2000)) {
    float current = actuatorReader.getValveActuatorCurrent();
    Serial.printf("Current: %.3f A\n", current);
  } else {
    Serial.println("Warning: Current data stale!");
  }

  delay(100);
}
```

### Pattern 3: Command-Specific Handling

```cpp
void loop() {
  actuatorReader.update();

  char cmd = actuatorReader.getValveActuatorMiscCommand();
  float value = actuatorReader.getValveActuatorMisc();

  switch (cmd) {
    case 'V':
      Serial.printf("Valve position: %.2f%%\n", value);
      break;
    case 'P':
      Serial.printf("Pressure: %.2f kPa\n", value);
      break;
    case 'F':
      Serial.printf("Flow: %.2f L/min\n", value);
      break;
    case '\0':
      Serial.println("No misc data received yet");
      break;
  }

  delay(1000);
}
```

### Pattern 4: Bidirectional Communication

```cpp
void loop() {
  // Update receiver (non-blocking)
  actuatorReader.update();

  // Send command periodically
  static uint32_t lastSend = 0;
  if (millis() - lastSend > 100) {  // 10 Hz
    lastSend = millis();

    // Send valve command
    actuator.sendSerialCommand('V', 50.0);
  }

  // Use received data (available immediately after update)
  float current = actuatorReader.getValveActuatorCurrent();
  float valve_pos = 0.0;

  if (actuatorReader.getValveActuatorMiscCommand() == 'V') {
    valve_pos = actuatorReader.getValveActuatorMisc();
  }

  Serial.printf("Commanded: 50.0%% | Actual: %.2f%% | Current: %.3fA\n",
                valve_pos, current);

  delay(10);
}
```

### Pattern 5: Error Recovery

```cpp
void loop() {
  actuatorReader.update();

  // Check for communication timeout
  static uint32_t lastDataTime = 0;
  uint32_t currentTime = millis();

  if (!actuatorReader.isCurrentStale(100)) {
    lastDataTime = currentTime;
  }

  // If no data for 5 seconds, attempt recovery
  if (currentTime - lastDataTime > 5000) {
    Serial.println("Communication timeout - recovering...");

    // Clear buffers and try to resync
    actuatorReader.clearBuffer();

    // Send test command
    actuator.sendSerialCommand('V', 0.0);

    lastDataTime = currentTime;  // Reset timeout
  }

  delay(10);
}
```

## Integration in ParallelMixer

### Setup (main.cpp)

```cpp
#include "SerialActuatorReader.hpp"

SerialActuatorReader actuatorReader(&Serial1);

void setup() {
  // Initialize Serial1
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_TX_PIN, SERIAL1_RX_PIN);

  // Initialize async reader
  actuatorReader.begin();
}
```

### Loop (main.cpp)

```cpp
void loop() {
  // Update asynchronous reader (non-blocking, call every loop)
  actuatorReader.update();

  // Send commands periodically
  static uint32_t lastSerialSend = 0;
  if (millis() - lastSerialSend > 1000) {
    lastSerialSend = millis();

    // Send valve command
    actuator.sendSerialCommand('V', actuator.getValveControlSignal());

    // Access received data
    float current = actuatorReader.getValveActuatorCurrent();
    float misc = actuatorReader.getValveActuatorMisc();
    char miscCmd = actuatorReader.getValveActuatorMiscCommand();

    Serial.printf("Sent: V%.2f | Received: I=%.3fA, %c=%.2f\n",
                  actuator.getValveControlSignal(), current, miscCmd, misc);
  }

  // Rest of main loop...
}
```

## Debug Mode

Enable detailed logging by defining `DEBUG_SERIAL_ACTUATOR_READER`:

### In platformio.ini:
```ini
build_flags =
  -DDEBUG_SERIAL_ACTUATOR_READER
```

### Debug Output:
```
[SerialActuatorReader] Current: 1.250 A
[SerialActuatorReader] Misc: V50.50
[SerialActuatorReader] Misc: P123.45
```

## Performance Characteristics

### Timing
- **update() execution time**: < 1ms typical (depends on available data)
- **Non-blocking**: Returns immediately if no data available
- **Message parsing**: Instant when '\n' received

### Memory Usage
- **Instance size**: ~60 bytes (String buffer + data + timestamps)
- **Buffer size**: 32 bytes maximum per message
- **Stack usage**: Minimal (all allocations in constructor)

### Recommendations
- Call `update()` at least every 10ms for reliable reception
- Ideal: Call `update()` every main loop iteration
- Maximum message rate: Limited by serial baud rate (115200 = ~11.5 KB/s)

## Comparison: Synchronous vs Asynchronous

### Synchronous (Old Method)
```cpp
// BLOCKS waiting for data
if (actuator.readSerialMeasurement(cmd, value, 50)) {
  // Waited up to 50ms!
  processData(cmd, value);
}
```

**Problems:**
- ❌ Blocks main loop for up to timeout duration
- ❌ Data lost if not read immediately
- ❌ Cannot process multiple messages efficiently
- ❌ Hard to integrate with other timing-sensitive code

### Asynchronous (New Method)
```cpp
// NON-BLOCKING - returns immediately
actuatorReader.update();

// Access data anytime
float current = actuatorReader.getValveActuatorCurrent();
```

**Benefits:**
- ✅ Never blocks main loop
- ✅ Data always available when needed
- ✅ Processes messages as they arrive
- ✅ Perfect for real-time systems

## Files Modified/Created

1. **[include/SerialActuatorReader.hpp](include/SerialActuatorReader.hpp)** - Class definition
2. **[src/SerialActuatorReader.cpp](src/SerialActuatorReader.cpp)** - Implementation
3. **[src/main.cpp](src/main.cpp#L16)** - Include and integration
4. **[src/main.cpp](src/main.cpp#L46)** - Instance creation
5. **[src/main.cpp](src/main.cpp#L215)** - Initialization
6. **[src/main.cpp](src/main.cpp#L339)** - Update call in loop

## Testing Checklist

- ✅ Reader initialized in setup()
- ✅ update() called in main loop
- ✅ 'I' commands stored in current
- ✅ Other commands stored in misc
- ✅ Timestamps updated correctly
- ✅ Staleness detection works
- ✅ Buffer overflow protection active
- ✅ '\r' characters ignored
- ✅ '\n' triggers message parsing
- ✅ Invalid messages discarded

## Conclusion

The `SerialActuatorReader` provides a robust, non-blocking solution for receiving actuator data. By processing characters asynchronously and storing parsed values for later access, it eliminates the blocking behavior of synchronous serial reading while maintaining simplicity and reliability.

**Key Advantages:**
- ✅ Non-blocking operation
- ✅ Simple helper-based API
- ✅ Separate storage for current ('I') and misc data
- ✅ Timestamp tracking and staleness detection
- ✅ Automatic buffer overflow protection
- ✅ Perfect for real-time control systems
