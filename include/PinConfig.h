/*
 * T-Display S3 Pin Configuration for Dual I2C + Dual Serial
 * 
 * This file defines all the pins needed for your P-Mixer project
 * with two I2C buses and two Serial ports.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ============================================================================
// I2C BUS PINS - For sensors with same address
// ============================================================================

// I2C Bus 0 (Wire) - First sensor group (Flow + Pressure sensors #1)
#define I2C0_SDA_PIN  43
#define I2C0_SCL_PIN  44

// I2C Bus 0 Pull-up Control - Controls power to pull-up resistors
#define I2C0_PULLUP_CTRL_PIN 21  // Set HIGH to enable pull-ups, LOW to disable

// I2C Bus 0 Clock Frequency
#define I2C0_CLOCK_FREQ 400000  // 400kHz for better reliability

// I2C Bus 1 (Wire1) - Second sensor group (Flow + Pressure sensors #2)
#define I2C1_SDA_PIN 10
#define I2C1_SCL_PIN 11

// ============================================================================
// SERIAL PORT PINS - For external microcontroller communication
// ============================================================================

// Serial1 - First external microcontroller (hardware UART1)
#define SERIAL1_TX_PIN 17
#define SERIAL1_RX_PIN 18

// Serial2 - Second external microcontroller  
#define SERIAL2_TX_PIN 12
#define SERIAL2_RX_PIN 13

// ============================================================================
// USER INTERFACE PINS (Already used by T-Display S3)
// ============================================================================

// Built-in buttons (not on header, internal use only)
#define INTERACTION_BUTTON_PIN 14    // GPIO14 - Button 2
#define BOOT_BUTTON_PIN 0            // GPIO0 - Button 1

// Display power control (IMPORTANT!)
#define DISPLAY_POWER_PIN 15         // GPIO15 - Must set HIGH for display

// ============================================================================
// PIN SUMMARY - What's available on T-Display S3
// ============================================================================

/*
USED PINS (This configuration):
✅ GPIO43, 44  - I2C Bus 0 (SDA, SCL)
✅ GPIO21      - I2C Bus 0 pull-up control (for SFM3505 reset)
✅ GPIO10, 11  - I2C Bus 1 (SDA, SCL)
✅ GPIO17, 18  - Serial1 (TX, RX) - Hardware UART1
✅ GPIO12, 13  - Serial2 (TX, RX)

RESERVED BY DISPLAY (Can't use):
❌ GPIO5-9     - Display control
❌ GPIO38      - Display backlight
❌ GPIO15      - Display power enable
❌ GPIO39-42, 45-48 - Display data bus

INTERNAL BUTTONS (Not on header):
⚠️ GPIO0, 14   - User buttons

STILL AVAILABLE:
✅ GPIO1, 2, 3 - Available for other use (ADC capable if needed)

NOTES:
- GPIO43/44 also used for Serial when USB CDC is off
- All pins are 3.3V logic level
- Pull-up resistors needed for I2C (4.7kΩ typical)
*/

// ============================================================================
// ALTERNATIVE PIN CONFIGURATIONS
// ============================================================================

/*
If you don't need Serial2, you can free up GPIO12/13:

OPTION A: Use GPIO12/13 for other purposes (current configuration)
#define I2C0_SDA_PIN 43
#define I2C0_SCL_PIN 44
#define I2C1_SDA_PIN 10
#define I2C1_SCL_PIN 11
#define SERIAL1_TX_PIN 17  // Hardware UART1
#define SERIAL1_RX_PIN 18  // Hardware UART1
// GPIO1, 2, 3, 12, 13 available for other use

OPTION B: Move I2C to different pins
#define I2C0_SDA_PIN 43
#define I2C0_SCL_PIN 44
#define I2C1_SDA_PIN 1
#define I2C1_SCL_PIN 2
#define SERIAL1_TX_PIN 17  // Hardware UART1
#define SERIAL1_RX_PIN 18  // Hardware UART1
// GPIO3, 10, 11, 12, 13 available for other use
*/

// ============================================================================
// WIRING REFERENCE
// ============================================================================

/*
I2C BUS 0 (Sensors Group 1) - WITH CONTROLLED PULL-UPS FOR SFM3505:
├─ GPIO21 (Pull-up Control) ──┬── NPN transistor base (via 1kΩ resistor)
│                             └── Controls power to pull-up resistors
│
├─ NPN Transistor Collector ──── 3.3V supply for pull-ups
├─ NPN Transistor Emitter ────┬── 4.7kΩ pull-up to GPIO43 (SDA)
│                             └── 4.7kΩ pull-up to GPIO44 (SCL)
│
├─ GPIO43 (SDA) ──┬── 4.7kΩ to transistor emitter
│                 ├── SFM3505 (0x2E) - requires controlled pull-ups!
│                 ├── SFM Sensor 1 (0x40)
│                 ├── SPD Sensor 1 (0x25)
│                 └── SSC Sensor 1 (0x58)
│
└─ GPIO44 (SCL) ──┴── 4.7kΩ to transistor emitter

NOTE: SFM3505 requires pull-ups to be OFF during first 31ms after power-on!
      GPIO21 controls an NPN transistor that switches the pull-up power.
      This prevents the sensor from being partially powered via ESD diodes.

I2C BUS 1 (Sensors Group 2):
├─ GPIO10 (SDA) ──┬── 4.7kΩ to 3.3V (standard, always on)
│                 ├── SFM Sensor 2 (0x40)
│                 ├── SPD Sensor 2 (0x25)
│                 └── SSC Sensor 2 (0x58)
└─ GPIO11 (SCL) ──┴── 4.7kΩ to 3.3V

SERIAL1 (External MCU 1) - Hardware UART1:
├─ GPIO17 (TX) ───── RX of external MCU 1
└─ GPIO18 (RX) ───── TX of external MCU 1

SERIAL2 (External MCU 2):
├─ GPIO12 (TX) ───── RX of external MCU 2
└─ GPIO13 (RX) ───── TX of external MCU 2

Common connections:
└─ GND ───────────── All devices must share common ground

TRANSISTOR CIRCUIT FOR PULL-UP CONTROL:
                        +3.3V
                          │
                          │
                    ┌─────┴─────┐
                    │           │
                  4.7kΩ       4.7kΩ
                    │           │
                    ├───────────┤ (Pull-up rail)
                    │           │
                    │         Collector
                  SDA         ──┐
                    │           │ NPN (2N3904 or similar)
                  SCL         Emitter
                    │           │
                    │           GND
                    │
                  Base ─── 1kΩ ─── GPIO21
*/

#endif // PIN_CONFIG_H
