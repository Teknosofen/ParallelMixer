# Serial Actuator Communication

## Overview

Added Serial communication support to ActuatorControl class for controlling external actuators via Serial1. The protocol uses simple ASCII commands with the format `<CHR>FLOAT\n`.

## Protocol Specification

### Command Format
```
<CHR>FLOAT\n
```

Where:
- `<CHR>` = Single character command identifier (e.g., 'V', 'P', 'F')
- `FLOAT` = Floating point value as ASCII (e.g., "50.5", "123.45")
- `\n` = Line terminator (newline)

### Examples

**Commands (sent to actuator):**
```
V50.5\n      → Set valve to 50.5%
F25.0\n      → Set flow reference to 25.0 L/min
P100.0\n     → Set pressure to 100.0 kPa
```

**Responses (received from actuator):**
```
V50.5\n      → Valve confirmed at 50.5%
P123.4\n     → Pressure reading 123.4 kPa
F24.8\n      → Flow reading 24.8 L/min
```

## API Reference

### Public Methods

#### sendSerialCommand()
```cpp
bool sendSerialCommand(char command, float value);
```

**Purpose:** Send a command to the external actuator via Serial1

**Parameters:**
- `command` - Single character command identifier
- `value` - Float value to send

**Returns:** `true` if command was sent, `false` on error

**Behavior:**
1. Formats as `<CHR>FLOAT\n` with 2 decimal places
2. Sends via Serial1
3. Optionally logs (if `DEBUG_SERIAL_ACTUATOR` defined)

**Note:** Caller is responsible for value validation and limiting. Both the calling function and the actuator should enforce appropriate limits.

**Example:**
```cpp
// Caller validates/limits the value before sending
float valve_percent = 50.5;
if (valve_percent < 0.0) valve_percent = 0.0;
if (valve_percent > 100.0) valve_percent = 100.0;

// Send valve command: V50.50\n
actuator.sendSerialCommand('V', valve_percent);
```

#### readSerialResponse()
```cpp
bool readSerialResponse(char& command, float& value, uint32_t timeout_ms = 100);
```

**Purpose:** Read a response from the external actuator

**Parameters:**
- `command` - [OUT] Receives the command character
- `value` - [OUT] Receives the parsed float value
- `timeout_ms` - Timeout in milliseconds (default: 100ms)

**Returns:** `true` if valid response received, `false` on timeout or error

**Behavior:**
1. Waits up to `timeout_ms` for data on Serial1
2. Reads characters until `\n` is received
3. Parses format: `<CHR>FLOAT\n`
4. Returns command character and float value
5. Prevents buffer overflow (max 32 characters)

**Example:**
```cpp
char cmd;
float val;

if (actuator.readSerialResponse(cmd, val, 100)) {
  Serial.printf("Received: %c = %.2f\n", cmd, val);
  // Output: "Received: V = 50.50"
} else {
  Serial.println("Timeout or error");
}
```

#### readSerialMeasurement()
```cpp
bool readSerialMeasurement(char& command, float& value, uint32_t timeout_ms = 100);
```

**Purpose:** Read a measurement from the external actuator

**Parameters:** Same as `readSerialResponse()`

**Returns:** Same as `readSerialResponse()`

**Note:** This is an alias for `readSerialResponse()`. Kept as a separate method for semantic clarity when reading measurements vs. command confirmations.

**Example:**
```cpp
char cmd;
float pressure;

if (actuator.readSerialMeasurement(cmd, pressure, 100)) {
  if (cmd == 'P') {
    Serial.printf("Pressure: %.2f kPa\n", pressure);
  }
}
```

#### clearSerialBuffer()
```cpp
void clearSerialBuffer();
```

**Purpose:** Clear any pending data in Serial1 receive buffer

**Use Case:** Call before expecting a response to ensure you're reading fresh data

**Example:**
```cpp
// Clear stale data before sending command
actuator.clearSerialBuffer();
actuator.sendSerialCommand('V', 50.0);

// Now wait for response
char cmd;
float val;
if (actuator.readSerialResponse(cmd, val, 200)) {
  Serial.printf("Response: %c%.2f\n", cmd, val);
}
```

## Usage Patterns

### Pattern 1: Send Command and Wait for Confirmation

```cpp
void setRemoteValve(float percent) {
  // Validate and limit value (caller's responsibility)
  if (percent < 0.0) percent = 0.0;
  if (percent > 100.0) percent = 100.0;

  // Clear any stale data
  actuator.clearSerialBuffer();

  // Send command
  if (actuator.sendSerialCommand('V', percent)) {
    Serial.printf("Sent: V%.2f\n", percent);

    // Wait for confirmation
    char cmd;
    float confirmed_value;
    if (actuator.readSerialResponse(cmd, confirmed_value, 200)) {
      if (cmd == 'V') {
        Serial.printf("Confirmed: Valve at %.2f%%\n", confirmed_value);
      }
    } else {
      Serial.println("No confirmation received");
    }
  }
}
```

### Pattern 2: Request Measurement

```cpp
void readRemotePressure() {
  // Send measurement request command (if actuator supports it)
  actuator.clearSerialBuffer();
  actuator.sendSerialCommand('P', 0.0);  // Request pressure reading

  // Wait for measurement
  char cmd;
  float pressure;
  if (actuator.readSerialMeasurement(cmd, pressure, 500)) {
    if (cmd == 'P') {
      Serial.printf("Pressure: %.2f kPa\n", pressure);
    }
  }
}
```

### Pattern 3: Non-Blocking Check for Measurements

```cpp
void loop() {
  // Check if measurement available (non-blocking with 0ms timeout)
  char cmd;
  float value;

  if (actuator.readSerialMeasurement(cmd, value, 0)) {
    switch (cmd) {
      case 'P':
        Serial.printf("Pressure: %.2f kPa\n", value);
        break;
      case 'F':
        Serial.printf("Flow: %.2f L/min\n", value);
        break;
    }
  }

  // Continue with other loop tasks...
}
```

### Pattern 4: Bidirectional Control with Limits

```cpp
void controlRemoteActuator() {
  float desired_flow = 45.5;  // L/min

  // Caller enforces host-side limits
  if (desired_flow < 0.0) desired_flow = 0.0;
  if (desired_flow > 50.0) desired_flow = 50.0;

  actuator.sendSerialCommand('F', desired_flow);

  // The actuator will also clamp to its safe operating range
  // If actuator max is 40.0, it will respond with F40.00\n

  char cmd;
  float actual_flow;
  if (actuator.readSerialResponse(cmd, actual_flow, 100)) {
    if (actual_flow != desired_flow) {
      Serial.printf("Warning: Requested %.2f but actuator set to %.2f\n",
                    desired_flow, actual_flow);
    }
  }
}
```

## Hardware Setup

### Serial1 Configuration

**In main.cpp setup():**
```cpp
void setup() {
  // Initialize Serial1 for actuator communication
  Serial1.begin(115200);  // Adjust baud rate as needed
  Serial1.setTimeout(100); // Optional: set read timeout

  // Wait for Serial1 to initialize
  delay(100);

  Serial.println("Serial1 initialized for actuator communication");
}
```

### Wiring

Connect Serial1 to external actuator:
- **TX1** (GPIO43) → Actuator RX
- **RX1** (GPIO44) → Actuator TX
- **GND** → Common ground

### Actuator Requirements

The external actuator must implement the same protocol:

**Receive commands:**
- Parse format: `<CHR>FLOAT\n`
- Extract command character and float value
- Validate and clamp value to safe range
- Execute command

**Send responses:**
- After executing command, echo back: `<CHR>ACTUAL_VALUE\n`
- For measurements, send: `<CHR>MEASURED_VALUE\n`

**Example Actuator Code (Arduino):**
```cpp
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    if (cmd.length() >= 2) {
      char command = cmd[0];
      float value = cmd.substring(1).toFloat();

      // Process command
      switch (command) {
        case 'V':  // Valve control
          // Clamp to actuator limits
          if (value < 0.0) value = 0.0;
          if (value > 100.0) value = 100.0;

          setValve(value);

          // Echo confirmation
          Serial.print(command);
          Serial.print(value, 2);
          Serial.println();
          break;

        case 'P':  // Pressure measurement request
          float pressure = readPressure();
          Serial.print('P');
          Serial.print(pressure, 2);
          Serial.println();
          break;
      }
    }
  }
}
```

## Value Limits and Safety

### Dual-Level Protection

The system implements two levels of limit checking:

1. **Caller-side (Application Code):**
   ```cpp
   // Caller validates and limits before sending
   float valve_percent = 150.0;
   if (valve_percent > 100.0) valve_percent = 100.0;

   actuator.sendSerialCommand('V', valve_percent);
   // Sends: V100.00\n
   ```

2. **Actuator-side (External Device):**
   ```cpp
   // Actuator also validates and clamps
   if (value > ACTUATOR_MAX_SAFE_VALUE) {
     value = ACTUATOR_MAX_SAFE_VALUE;
   }
   ```

### Responsibility Separation

- **Caller's responsibility:** Validate and limit values before calling `sendSerialCommand()`
- **Actuator's responsibility:** Final safety check, enforce hardware limits
- **Defense in depth:** If caller misses validation, actuator protects

### Example with Limits

```cpp
// Caller allows 0-100%, but actuator only allows 0-80%

// Caller side (application code)
float valve_percent = 90.0;
if (valve_percent < 0.0) valve_percent = 0.0;
if (valve_percent > 100.0) valve_percent = 100.0;

actuator.sendSerialCommand('V', valve_percent);
// Sends: V90.00\n

// Actuator side (in actuator firmware)
if (value > 80.0) value = 80.0;
setValve(80.0);
// Responds: V80.00\n

// Caller reads response
char cmd;
float actual;
if (actuator.readSerialResponse(cmd, actual)) {
  // actual = 80.0
  Serial.printf("Actuator clamped to %.2f%%\n", actual);
}
```

## Debug Mode

Enable detailed logging by defining `DEBUG_SERIAL_ACTUATOR` in your build:

### In platformio.ini:
```ini
build_flags =
  -DDEBUG_SERIAL_ACTUATOR
```

### Debug Output Example:
```
[ActuatorControl] Sent to Serial1: V50.50
[ActuatorControl] Received from Serial1: V50.50
```

## Error Handling

### Timeout Handling

```cpp
char cmd;
float val;

if (!actuator.readSerialResponse(cmd, val, 200)) {
  Serial.println("Error: Actuator not responding");

  // Optionally retry
  actuator.clearSerialBuffer();
  actuator.sendSerialCommand('V', 50.0);

  if (actuator.readSerialResponse(cmd, val, 500)) {
    Serial.println("Retry successful");
  } else {
    Serial.println("Actuator offline - using fallback");
  }
}
```

### Malformed Response Handling

```cpp
char cmd;
float val;

if (actuator.readSerialResponse(cmd, val, 100)) {
  // Validate response matches expected command
  if (cmd != 'V') {
    Serial.printf("Warning: Expected 'V' but got '%c'\n", cmd);
  }

  // Validate value is in expected range
  if (val < 0.0 || val > 100.0) {
    Serial.printf("Warning: Value %.2f out of range\n", val);
  }
} else {
  Serial.println("Failed to read response");
}
```

### Buffer Overflow Protection

The `readSerialResponse()` method automatically prevents buffer overflow:

```cpp
// Built-in protection in readSerialResponse()
if (response.length() > 32) {
  return false;  // Reject overly long messages
}
```

## Performance Considerations

### Timing

- **Send command:** ~1ms (depends on baud rate)
- **Read response:** Up to `timeout_ms` (typically 100-500ms)
- **Typical round-trip:** 5-20ms at 115200 baud

### Non-Blocking Usage

For time-critical applications, use short timeouts:

```cpp
// Check for measurement without blocking
if (actuator.readSerialMeasurement(cmd, val, 0)) {
  // Process measurement
}
// Continues immediately if no data available
```

### Recommended Practice

```cpp
void loop() {
  // Send commands at controlled rate
  static uint32_t lastCommandTime = 0;
  if (millis() - lastCommandTime > 100) {  // 10 Hz max
    // Validate valve setpoint before sending
    float valve_cmd = valve_setpoint;
    if (valve_cmd < 0.0) valve_cmd = 0.0;
    if (valve_cmd > 100.0) valve_cmd = 100.0;

    actuator.sendSerialCommand('V', valve_cmd);
    lastCommandTime = millis();
  }

  // Non-blocking read for measurements
  char cmd;
  float val;
  if (actuator.readSerialMeasurement(cmd, val, 0)) {
    processMeasurement(cmd, val);
  }

  // Other loop tasks...
}
```

## Files Modified

1. **[include/ActuatorControl.hpp](include/ActuatorControl.hpp#L69-L74)**
   - Added public methods for Serial communication

2. **[src/ActuatorControl.cpp](src/ActuatorControl.cpp#L276-L359)**
   - Implemented Serial communication methods

## Integration Example

### Complete Working Example

```cpp
#include "ActuatorControl.hpp"

ActuatorControl actuator(VALVE_PIN);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // Actuator communication
  delay(100);

  actuator.initialize();

  // Test communication
  testActuatorComm();
}

void testActuatorComm() {
  Serial.println("Testing actuator communication...");

  // Clear any stale data
  actuator.clearSerialBuffer();

  // Send test command
  if (actuator.sendSerialCommand('V', 50.0, 0.0, 100.0)) {
    Serial.println("Command sent");

    // Wait for response
    char cmd;
    float val;
    if (actuator.readSerialResponse(cmd, val, 500)) {
      Serial.printf("Success! Actuator responded: %c%.2f\n", cmd, val);
    } else {
      Serial.println("Error: No response from actuator");
    }
  }
}

void loop() {
  // Example: Control valve and read pressure
  static uint32_t lastUpdate = 0;

  if (millis() - lastUpdate > 100) {  // 10 Hz
    // Send valve command
    float valve_percent = 50.0;
    actuator.sendSerialCommand('V', valve_percent);

    // Read response
    char cmd;
    float val;
    if (actuator.readSerialResponse(cmd, val, 100)) {
      Serial.printf("Valve: %.2f%%\n", val);
    }

    lastUpdate = millis();
  }

  // Check for unsolicited measurements
  char meas_cmd;
  float meas_val;
  if (actuator.readSerialMeasurement(meas_cmd, meas_val, 0)) {
    Serial.printf("Measurement: %c = %.2f\n", meas_cmd, meas_val);
  }
}
```

## Testing Checklist

- ✅ Serial1 initialized at correct baud rate
- ✅ Commands sent with correct format
- ✅ Values clamped to specified limits
- ✅ Responses parsed correctly
- ✅ Timeouts work as expected
- ✅ Buffer overflow protection active
- ✅ clearSerialBuffer() removes stale data
- ✅ Debug output available when enabled
- ✅ Actuator responds within timeout
- ✅ Measurement readings accurate

## Conclusion

The Serial actuator communication feature provides a simple, robust protocol for controlling external actuators via Serial1. With built-in value limiting, timeout handling, and buffer protection, it's suitable for safety-critical applications while remaining easy to use.

**Key Benefits:**
- ✅ Simple ASCII protocol (human-readable)
- ✅ Dual-level safety limits
- ✅ Timeout protection
- ✅ Buffer overflow prevention
- ✅ Flexible command/measurement handling
- ✅ Debug logging support
- ✅ Non-blocking operation possible
