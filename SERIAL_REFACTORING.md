# Serial Communication Refactoring

## Overview

The serial communication methods have been refactored from `ActuatorControl` to `SerialActuatorReader`, providing better separation of concerns and allowing `ActuatorControl` to send commands while `main.cpp` can independently access received data.

## Architecture Changes

### Before Refactoring

```
ActuatorControl
├─ sendSerialCommand()      → Sends to Serial1
├─ readSerialResponse()     → Reads from Serial1 (blocking)
├─ readSerialMeasurement()  → Reads from Serial1 (blocking)
└─ clearSerialBuffer()      → Clears Serial1 buffer

Problem: ActuatorControl owns Serial1, making it hard for other parts
         of the application to access received data independently
```

### After Refactoring

```
SerialActuatorReader
├─ update()                 → Non-blocking async reception
├─ sendSerialCommand()      → Sends to Serial1
├─ readSerialResponse()     → Reads from Serial1 (blocking, for compatibility)
├─ readSerialMeasurement()  → Reads from Serial1 (blocking, for compatibility)
├─ clearBuffer()            → Clears buffers
├─ getValveActuatorCurrent() → Access received 'I' data
├─ getValveActuatorMisc()    → Access received misc data
└─ getValveActuatorMiscCommand() → Get misc command character

ActuatorControl
├─ setSerialActuatorReader() → Links to SerialActuatorReader
├─ sendSerialCommand()      → Forwards to SerialActuatorReader
├─ readSerialResponse()     → Forwards to SerialActuatorReader
├─ readSerialMeasurement()  → Forwards to SerialActuatorReader
└─ clearSerialBuffer()      → Forwards to SerialActuatorReader

Benefit: Separation of concerns - ActuatorControl sends commands,
         SerialActuatorReader manages reception, main.cpp accesses data
```

## Key Benefits

### 1. Separation of Concerns
- **ActuatorControl**: Focuses on valve control logic, sends commands
- **SerialActuatorReader**: Manages all serial communication
- **main.cpp**: Accesses received data independently

### 2. Non-Blocking Operation
- `actuatorReader.update()` processes incoming data asynchronously
- No blocking waits in main loop
- Data always available via getter methods

### 3. Backward Compatibility
- ActuatorControl still provides same serial methods
- Methods forward to SerialActuatorReader
- Existing code continues to work

### 4. Independent Data Access
- ActuatorControl can send commands
- main.cpp can access received data
- No conflicts or race conditions

## Files Modified

### 1. include/SerialActuatorReader.hpp
**Added methods:**
```cpp
// Transmission methods
bool sendSerialCommand(char command, float value);
bool readSerialResponse(char& command, float& value, uint32_t timeout_ms = 100);
bool readSerialMeasurement(char& command, float& value, uint32_t timeout_ms = 100);
```

### 2. src/SerialActuatorReader.cpp
**Added implementations:**
- `sendSerialCommand()` - Sends formatted command to serial port
- `readSerialResponse()` - Blocking read with timeout (for compatibility)
- `readSerialMeasurement()` - Alias for readSerialResponse()

### 3. include/ActuatorControl.hpp
**Changes:**
- Added forward declaration of `SerialActuatorReader`
- Added `setSerialActuatorReader()` method
- Added `_serialActuatorReader` pointer member
- Serial methods now forward to SerialActuatorReader

### 4. src/ActuatorControl.cpp
**Changes:**
- Removed direct Serial1 communication code
- Added `setSerialActuatorReader()` implementation
- Serial methods now forward to `_serialActuatorReader`
- Null pointer checks for safety

### 5. src/main.cpp
**Changes:**
- Added `actuator.setSerialActuatorReader(&actuatorReader);` in setup()
- Links ActuatorControl to SerialActuatorReader instance

## Usage Patterns

### Pattern 1: ActuatorControl Sends Commands

```cpp
// ActuatorControl can still send commands as before
actuator.sendSerialCommand('V', 50.0);

// Behind the scenes, this forwards to:
// actuatorReader.sendSerialCommand('V', 50.0);
```

### Pattern 2: main.cpp Accesses Received Data

```cpp
void loop() {
  // Update receiver (non-blocking)
  actuatorReader.update();

  // ActuatorControl sends command
  actuator.sendSerialCommand('V', actuator.getValveControlSignal());

  // main.cpp independently accesses received data
  float current = actuatorReader.getValveActuatorCurrent();
  float misc = actuatorReader.getValveActuatorMisc();
  char miscCmd = actuatorReader.getValveActuatorMiscCommand();

  Serial.printf("Current: %.3f A, %c: %.2f\n", current, miscCmd, misc);
}
```

### Pattern 3: Backward Compatible Blocking Reads

```cpp
// Old synchronous code still works (for compatibility)
char cmd;
float value;
if (actuator.readSerialResponse(cmd, value, 100)) {
  // Process response
}

// This forwards to:
// actuatorReader.readSerialResponse(cmd, value, 100);
```

## Data Flow

### Sending Commands

```
ActuatorControl::sendSerialCommand('V', 50.0)
        ↓
SerialActuatorReader::sendSerialCommand('V', 50.0)
        ↓
Serial1.print("V50.00\n")
        ↓
External Actuator
```

### Receiving Data (Asynchronous)

```
External Actuator → "I1.25\n"
        ↓
Serial1 RX Buffer
        ↓
actuatorReader.update() (called every loop)
        ↓
SerialActuatorReader processes character by character
        ↓
Message complete: "I1.25"
        ↓
Stored in _valveActuatorCurrent = 1.25
        ↓
main.cpp: actuatorReader.getValveActuatorCurrent() → 1.25
```

### Receiving Data (Synchronous - for compatibility)

```
ActuatorControl::readSerialResponse(cmd, value, 100)
        ↓
SerialActuatorReader::readSerialResponse(cmd, value, 100)
        ↓
Blocks up to 100ms waiting for Serial1 data
        ↓
Returns: cmd='I', value=1.25
```

## Integration Example

### Setup

```cpp
#include "SerialActuatorReader.hpp"
#include "ActuatorControl.hpp"

SerialActuatorReader actuatorReader(&Serial1);
ActuatorControl actuator(VALVE_PIN);

void setup() {
  // Initialize Serial1
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_TX_PIN, SERIAL1_RX_PIN);

  // Initialize reader
  actuatorReader.begin();

  // Initialize actuator
  actuator.initialize();

  // Link actuator to reader
  actuator.setSerialActuatorReader(&actuatorReader);
}
```

### Loop

```cpp
void loop() {
  // Update async receiver (non-blocking)
  actuatorReader.update();

  // ActuatorControl sends valve command
  static uint32_t lastSend = 0;
  if (millis() - lastSend > 1000) {
    lastSend = millis();

    float valveCmd = actuator.getValveControlSignal();
    actuator.sendSerialCommand('V', valveCmd);

    // main.cpp accesses received data independently
    float current = actuatorReader.getValveActuatorCurrent();
    float misc = actuatorReader.getValveActuatorMisc();
    char miscCmd = actuatorReader.getValveActuatorMiscCommand();

    Serial.printf("Sent: V%.2f | Received: I=%.3fA, %c=%.2f\n",
                  valveCmd, current, miscCmd, misc);
  }

  // Rest of main loop...
}
```

## Migration Guide

### For Existing Code Using ActuatorControl Serial Methods

**No changes needed!** The ActuatorControl serial methods continue to work as before:

```cpp
// This still works exactly as before
actuator.sendSerialCommand('V', 50.0);
char cmd;
float value;
actuator.readSerialResponse(cmd, value, 100);
```

### For Code That Needs Independent Data Access

**Use SerialActuatorReader directly:**

```cpp
// Non-blocking access to received data
float current = actuatorReader.getValveActuatorCurrent();
float misc = actuatorReader.getValveActuatorMisc();
```

## Debug Mode

Both classes support debug output:

### SerialActuatorReader Debug

Define `DEBUG_SERIAL_ACTUATOR` for transmission debug:
```ini
build_flags = -DDEBUG_SERIAL_ACTUATOR
```

Output:
```
[SerialActuatorReader] Sent: V50.00
[SerialActuatorReader] Received: I1.25
```

Define `DEBUG_SERIAL_ACTUATOR_READER` for reception debug:
```ini
build_flags = -DDEBUG_SERIAL_ACTUATOR_READER
```

Output:
```
[SerialActuatorReader] Current: 1.250 A
[SerialActuatorReader] Misc: V50.50
```

## Testing Checklist

- ✅ SerialActuatorReader created and initialized
- ✅ ActuatorControl linked to SerialActuatorReader via setSerialActuatorReader()
- ✅ ActuatorControl.sendSerialCommand() still works
- ✅ ActuatorControl methods forward to SerialActuatorReader
- ✅ actuatorReader.update() called in main loop
- ✅ main.cpp can access received data via getters
- ✅ 'I' commands stored in current
- ✅ Other commands stored in misc
- ✅ No Serial1.available() conflicts
- ✅ Backward compatibility maintained

## Advantages Summary

| Aspect | Before | After |
|--------|--------|-------|
| **Serial ownership** | ActuatorControl owns Serial1 | SerialActuatorReader owns Serial1 |
| **Data access** | Only via blocking reads | Non-blocking async access |
| **Separation** | Control + Comms mixed | Clear separation |
| **Blocking** | readSerialResponse() blocks | update() is non-blocking |
| **Data storage** | Not stored | Stored for anytime access |
| **Independence** | Tight coupling | Loose coupling |
| **Flexibility** | Limited | High - send and receive independently |

## Conclusion

The refactoring successfully separates serial communication from actuator control logic. ActuatorControl can send commands through SerialActuatorReader, while main.cpp can independently access received data via helper methods. The architecture is cleaner, more flexible, and maintains backward compatibility with existing code.
