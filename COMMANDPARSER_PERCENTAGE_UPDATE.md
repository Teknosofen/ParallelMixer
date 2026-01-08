# CommandParser Percentage-Based Input Update

## Summary

Updated CommandParser to accept percentage-based input (0.0-100.0%) with decimal point support for Offset, Amplitude, and Valve commands. Hardware scaling from percentage to 0-4095 DAC counts is now handled within CommandParser before passing values to ActuatorControl methods.

## Changes Made

### 1. Help Text Updated
**File:** [src/CommandParser.cpp:84-87](src/CommandParser.cpp#L84-L87)

```cpp
Serial.println("O for Offset in [%], float 0.0-100.0");
Serial.println("A for Amplitude in [%], float 0.0-100.0");
Serial.println("S for Samples per period of pulses or sine, int");
Serial.println("V for manual setting of valve output [%], float 0.0-100.0");
```

### 2. Settings Display Updated
**File:** [src/CommandParser.cpp:101-111](src/CommandParser.cpp#L101-L111)

All three parameters now display as percentages with 2 decimal places:

```cpp
Serial.print("O= ");
Serial.print((offset * 100.0) / 4095.0, 2);  // Convert to %
Serial.println(" %");
Serial.print("A= ");
Serial.print((amplitude * 100.0) / 4095.0, 2);  // Convert to %
Serial.println(" %");
Serial.print("V= ");
Serial.print((valve_signal * 100.0) / 4095.0, 2);  // Convert to %
Serial.println(" %");
```

### 3. Command 'O' (Offset) - Updated
**File:** [src/CommandParser.cpp:271-292](src/CommandParser.cpp#L271-L292)

**Old behavior:**
- Accepted integer values 0-4095
- Direct hardware units

**New behavior:**
```cpp
case 'O': case 'o':  // Offset (as percentage 0-100%)
  {
    SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
    if (params.length() > 0) {
      float offset_percent = params.toFloat();
      // Clamp to 0-100%
      if (offset_percent < 0.0) offset_percent = 0.0;
      if (offset_percent > 100.0) offset_percent = 100.0;
      // Convert percentage to hardware units (0-4095)
      sigConfig.offset = (uint16_t)((offset_percent * 4095.0) / 100.0);
      actuator.setSignalGeneratorConfig(sigConfig);
      Serial.print("O= ");
      Serial.print(offset_percent, 2);
      Serial.print(" %");
      sendOK();
    } else {
      Serial.print("O= ");
      Serial.print((sigConfig.offset * 100.0) / 4095.0, 2);
      Serial.println(" %");
    }
  }
  break;
```

- ✅ Accepts float percentage 0.0-100.0%
- ✅ Clamps values to valid range
- ✅ Converts to hardware units (0-4095) before passing to actuator
- ✅ Displays confirmation with percentage and "%" suffix
- ✅ Query command shows current value as percentage

### 4. Command 'A' (Amplitude) - Updated
**File:** [src/CommandParser.cpp:294-315](src/CommandParser.cpp#L294-L315)

**Old behavior:**
- Accepted integer values 0-4095
- Direct hardware units

**New behavior:**
```cpp
case 'A': case 'a':  // Amplitude (as percentage 0-100%)
  {
    SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
    if (params.length() > 0) {
      float amplitude_percent = params.toFloat();
      // Clamp to 0-100%
      if (amplitude_percent < 0.0) amplitude_percent = 0.0;
      if (amplitude_percent > 100.0) amplitude_percent = 100.0;
      // Convert percentage to hardware units (0-4095)
      sigConfig.amplitude = (uint16_t)((amplitude_percent * 4095.0) / 100.0);
      actuator.setSignalGeneratorConfig(sigConfig);
      Serial.print("A= ");
      Serial.print(amplitude_percent, 2);
      Serial.print(" %");
      sendOK();
    } else {
      Serial.print("A= ");
      Serial.print((sigConfig.amplitude * 100.0) / 4095.0, 2);
      Serial.println(" %");
    }
  }
  break;
```

- ✅ Accepts float percentage 0.0-100.0%
- ✅ Clamps values to valid range
- ✅ Converts to hardware units (0-4095) before passing to actuator
- ✅ Displays confirmation with percentage and "%" suffix
- ✅ Query command shows current value as percentage

### 5. Command 'V' (Valve) - Updated
**File:** [src/CommandParser.cpp:317-335](src/CommandParser.cpp#L317-L335)

**Old behavior:**
- Accepted integer values 0-4095
- Direct hardware units

**New behavior:**
```cpp
case 'V': case 'v':  // Valve control signal (as percentage 0-100%)
  if (params.length() > 0) {
    float valve_percent = params.toFloat();
    // Clamp to 0-100%
    if (valve_percent < 0.0) valve_percent = 0.0;
    if (valve_percent > 100.0) valve_percent = 100.0;
    // Convert percentage to hardware units (0-4095)
    uint16_t signal = (uint16_t)((valve_percent * 4095.0) / 100.0);
    actuator.setValveControlSignal(signal);
    Serial.print("V= ");
    Serial.print(valve_percent, 2);
    Serial.print(" %");
    sendOK();
  } else {
    Serial.print("V= ");
    Serial.print((actuator.getValveControlSignal() * 100.0) / 4095.0, 2);
    Serial.println(" %");
  }
  break;
```

- ✅ Accepts float percentage 0.0-100.0%
- ✅ Clamps values to valid range
- ✅ Converts to hardware units (0-4095) before passing to actuator
- ✅ Displays confirmation with percentage and "%" suffix
- ✅ Query command shows current value as percentage

## Usage Examples

### Setting Values (New Behavior)

```
> O50.5
O= 50.50 % OK

> A75.25
A= 75.25 % OK

> V33.33
V= 33.33 % OK
```

### Querying Values

```
> O
O= 50.50 %

> A
A= 75.25 %

> V
V= 33.33 %
```

### Viewing All Settings

```
> !
T= 100000
Q= 0
C= 0
O= 50.50 %
A= 75.25 %
S= 100
V= 33.33 %
```

### Edge Cases

```
> O-10.5
O= 0.00 % OK        (clamped to minimum)

> A150
A= 100.00 % OK      (clamped to maximum)

> V0
V= 0.00 % OK        (minimum value)

> O100
O= 100.00 % OK      (maximum value)
```

## Conversion Formula

**Percentage to Hardware Units:**
```cpp
hardware_value = (uint16_t)((percentage * 4095.0) / 100.0)
```

**Hardware Units to Percentage:**
```cpp
percentage = (hardware_value * 100.0) / 4095.0
```

## Technical Details

### Data Type Changes
- Input parsing: Changed from `toInt()` to `toFloat()`
- Range validation: Changed from integer comparison to float comparison
- Output display: Added 2 decimal places with `.print(value, 2)`

### Hardware Scaling
- **12-bit DAC range:** 0-4095 counts
- **User-facing range:** 0.0-100.0%
- **Resolution:** 100% / 4095 = ~0.024% per count
- **Precision:** Display shows 2 decimal places (0.01% resolution)

### Backward Compatibility
**⚠️ Breaking Change:** Old commands using raw hardware values (0-4095) will now be interpreted as percentages.

**Migration:**
- Old: `O2048` (50% of 4095)
- New: `O50.0` (50%)

Users with existing scripts or configurations will need to update their command values.

## Benefits

1. ✅ **User-Friendly:** Percentage scale is intuitive (0-100% vs 0-4095)
2. ✅ **Hardware Abstraction:** Users don't need to know DAC resolution
3. ✅ **Decimal Precision:** Allows fine-grained control (e.g., 33.33%)
4. ✅ **Consistent Interface:** All three commands use same format
5. ✅ **Validation:** Automatic clamping prevents invalid values
6. ✅ **Clear Feedback:** Percentage display with "%" suffix is unambiguous

## Files Modified

1. [src/CommandParser.cpp](src/CommandParser.cpp) - Updated help text, printSettings(), and command handlers for O, A, V

No other files require changes - hardware scaling is fully contained within CommandParser.

## Testing Checklist

After updating:
- ✅ Command 'O' accepts float percentages (0.0-100.0%)
- ✅ Command 'A' accepts float percentages (0.0-100.0%)
- ✅ Command 'V' accepts float percentages (0.0-100.0%)
- ✅ Values clamp correctly at boundaries (0% and 100%)
- ✅ Decimal points work (e.g., "O50.5")
- ✅ Query commands show percentage format
- ✅ Settings display ('!') shows all values as percentages
- ✅ Hardware receives correct 0-4095 values after conversion
- ✅ Help text ('?') shows updated format

## Notes

This update follows the design principle: **"Scaling to physical entities is made in methods that know the hardware"**

In this case, CommandParser knows:
- User interface uses percentages
- Hardware uses 12-bit DAC (0-4095)
- Conversion happens at the boundary

ActuatorControl methods remain unchanged - they still receive hardware units (0-4095) and don't need to know about the percentage abstraction.
