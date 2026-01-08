# SFM3505 CRC Validation Implementation

## What Changed

Added full CRC-8 validation for the SFM3505 sensor data to ensure data integrity.

## CRC-8 Algorithm

**Polynomial**: 0x31 (x^8 + x^5 + x^4 + 1)
**Initial value**: 0xFF
**Standard**: Sensirion CRC-8

This is the standard Sensirion CRC algorithm used across their sensor family.

## Data Format and CRC Coverage

The SFM3505 returns 9 bytes with 3 CRC bytes interspersed:

```
Byte 0: Air Flow [23:16]  ┐
Byte 1: Air Flow [15:8]   ├─ Covered by CRC1 at byte 2
Byte 2: CRC1              ┘

Byte 3: Air Flow [7:0]    (standalone byte, no CRC)

Byte 4: O2 Flow [23:16]   ──> Covered by CRC2 at byte 5
Byte 5: CRC2

Byte 6: O2 Flow [15:8]    ┐
Byte 7: O2 Flow [7:0]     ├─ Covered by CRC3 at byte 8
Byte 8: CRC3              ┘
```

## Implementation Details

### File: `include/SensorReader.hpp`
Added compile-time flag to enable/disable CRC checking:
```cpp
#define SFM3505_ENABLE_CRC_CHECK 1  // Set to 0 to disable
```

### File: `src/SensorReader.cpp`

**Function**: `readSFM3505Raw()`

1. **Reads all 9 bytes** from sensor
2. **Validates 3 CRCs**:
   - CRC1: Validates Air Flow high bytes (bytes 0-1)
   - CRC2: Validates O2 Flow high byte (byte 4)
   - CRC3: Validates O2 Flow low bytes (bytes 6-7)
3. **Returns false** if any CRC fails
4. **Extracts data** bytes into output buffer (skipping CRCs)

### CRC Validation Code

```cpp
// CRC1: Covers bytes 0-1
uint8_t crc1_calculated = calculateCRC8(&rawData[0], 2);
if (crc1_calculated != rawData[2]) {
    // CRC error - print diagnostic and return false
}

// CRC2: Covers byte 4
uint8_t crc2_calculated = calculateCRC8(&rawData[4], 1);
if (crc2_calculated != rawData[5]) {
    // CRC error
}

// CRC3: Covers bytes 6-7
uint8_t crc3_calculated = calculateCRC8(&rawData[6], 2);
if (crc3_calculated != rawData[8]) {
    // CRC error
}
```

## Diagnostic Features

### First Read Debug Output
On the first successful read, the raw 9 bytes are printed:
```
[Bus0] SFM3505 raw data (first read): 0x80 0x00 0xA2 0x00 0x80 0xA2 0x00 0x00 0x81
```

This helps verify the data format is correct.

### CRC Error Messages
If CRC validation fails, detailed error messages show:
```
[Bus0] ❌ SFM3505 CRC1 error: calculated 0xA2, received 0xB3
[Bus0]    Data: [0]=0x80 [1]=0x00
```

### CRC Success Message
When all CRCs pass:
```
[Bus0] ✅ All CRCs valid
```

## Testing with New Sensor

### Step 1: Test with CRC Enabled (Default)
Upload the code as-is. The sensor should:
1. Respond at address 0x2E during scan
2. Successfully start continuous measurement
3. Return valid CRC-protected data

**Expected output**:
```
✅ Device found at address 0x2E (SFM3505)
[Bus0] Sending SFM3505 command: 0x3603 to address 0x2E
[Bus0] ✅ Command sent successfully
[Bus0] ✅ SFM3505 initialized and measurement started
[Bus0] SFM3505 raw data (first read): 0x... (9 bytes)
[Bus0] ✅ All CRCs valid
```

### Step 2: If CRC Errors Occur
If you see CRC errors, it could mean:
1. **Data corruption on I2C bus** - Check wiring, reduce cable length, add capacitors
2. **Wrong CRC coverage** - The byte grouping might be different
3. **Timing issues** - Try reducing I2C speed to 50kHz

To debug, temporarily disable CRC checking:

**In `include/SensorReader.hpp`**, change:
```cpp
#define SFM3505_ENABLE_CRC_CHECK 0  // Disable CRC
```

This will still show the raw data but won't reject readings.

### Step 3: Verify Flow Values
Once data is reading successfully, verify the flow values are reasonable:

**At zero flow** (no air moving):
- Air flow should be near 0.0 slm
- O2 flow should be near 0.0 slm

**With air flowing**:
- Positive values for forward flow
- Negative values for reverse flow
- Typical range: -200 to +200 slm (depending on sensor variant)

## Scaling Formula

The raw 24-bit values are converted to flow using:
```cpp
float flow = ((float)(rawValue - 8388608)) / 25600.0;
```

Where:
- `8388608` = 2^23 (offset for signed 24-bit value)
- `25600` = Scale factor for SFM3505

This formula matches what your colleague uses.

## Troubleshooting

### "SFM3505 read error: expected 9 bytes, got X"
- **Sensor not responding** - Check address (should be 0x2E)
- **I2C communication failed** - Check wiring, pull-ups
- **Sensor not started** - Verify start command (0x3603) was sent successfully

### "CRC1/2/3 error"
- **Data corruption** - Shorten I2C cables, add 100nF caps near sensor
- **Wrong algorithm** - CRC-8 polynomial should be 0x31 with init 0xFF
- **Timing** - Try slower I2C clock (50kHz instead of 100kHz)

### "All CRCs valid" but wrong flow values
- **Scaling issue** - Verify formula matches your sensor variant
- **Byte order** - Should be [23:16][15:8][7:0] after removing CRCs
- **Zero offset** - Sensor might need calibration or zeroing

## Reference

**Sensirion CRC-8 Specification**:
- Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
- Initialization: 0xFF
- Bit order: MSB first
- Used by: SHT, SDP, SFM sensor families

**SFM3505 Documentation**:
- GitHub: https://github.com/Sensirion/embedded-i2c-sfm3505
- Datasheet: Available from Sensirion website
