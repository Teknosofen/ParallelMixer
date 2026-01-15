# P-Mixer WiFi Server

Modern WiFi server for your P-Mixer project with real-time data visualization and CSV export.

## Features

✅ **Button-controlled WiFi** - Long press to enable, short press to disable
✅ **WiFi starts disabled** - Saves power and reduces interference by default
✅ **Real-time data push** - Updates flow, pressure, valve signals, and mode
✅ **High-speed buffering** - 50+ Hz data collection with efficient batch transfers
✅ **Live graphing** - Chart.js-based visualization with 6 data series (lines only, no points)
✅ **CSV export** - Client-side save button creates tab-separated CSV with timestamps
✅ **No SD card required** - All data stored in ESP32 memory
✅ **Clean UI** - Matches your TFT display color scheme (#84B6D6)  

## Key Differences from Old Version

| Old WifiApServer | New PMixerWiFiServer |
|------------------|----------------------|
| SD card file manager | Real-time data streaming |
| Static file downloads | Live graphing & CSV export |
| SPIFFS logo loading | Embedded HTML with Chart.js |
| Complex file management | Simple data push interface |

## Installation

1. Copy `PMixerWiFiServer.hpp` and `PMixerWiFiServer.cpp` to your project
2. Include the header: `#include "PMixerWiFiServer.hpp"`
3. Create instance: `PMixerWiFiServer wifiServer("YourSSID", "YourPassword");`

## Basic Usage

### Starting/Stopping WiFi

**WiFi starts DISABLED by default** to save power and reduce electromagnetic interference.

#### Button Control (Current Implementation)
```cpp
// WiFi is controlled via interaction button (Key1)
// - Long press (>1s): Enable WiFi
// - Short press: Disable WiFi

if (interactionKey1.wasReleased()) {
  if (interactionKey1.wasLongPress()) {
    wifiServer.start();  // Enable WiFi
    renderer.drawWiFiAPIP(wifiServer.getApIpAddress(), PMIXERSSID);
    renderer.drawWiFiPromt("Short press: disable");
  } else {
    wifiServer.stop();   // Disable WiFi
    renderer.drawWiFiAPIP("WiFi OFF", "No SSID");
    renderer.drawWiFiPromt("Long press: enable");
  }
}
```

#### Programmatic Control
```cpp
// In setup() - WiFi does NOT start automatically
// Call start() explicitly if needed
wifiServer.start();  // Start WiFi AP

// Later, to stop
wifiServer.stop();   // Stop WiFi AP

// Check status
if (wifiServer.isRunning()) {
    // WiFi is active
}
```

### Pushing Data to Web Interface

#### High-Speed Buffered Data Collection (Recommended)
```cpp
// Call addDataPoint() in fast control loop (e.g., 100Hz)
// Data is buffered and sent in batches to web client
wifiServer.addDataPoint(
  flow,           // SFM3505 air flow
  pressure,       // ABP2 pressure
  valveSignal,    // Valve control signal
  current,        // Actuator current
  lowPressure,    // ABPD low pressure
  temperature     // ABPD temperature
);

// Handle web requests (must call in loop!)
wifiServer.handleClient();
```

#### Legacy Single-Point Updates
```cpp
// Alternative: Update individual values (slower, lower data rate)
wifiServer.updateFlow(flowValue);           // Push flow data
wifiServer.updatePressure(pressureValue);   // Push pressure data
wifiServer.updateValveSignal(valveSignal);  // Push valve signal
wifiServer.updateMode("Manual");            // Push mode string

// Handle web requests (must call in loop!)
wifiServer.handleClient();
```

### Configuration

```cpp
// Set maximum data points to keep in memory (default: 100)
wifiServer.setMaxDataPoints(200);

// Get IP address to display
String ip = wifiServer.getApIpAddress();
renderer.drawWiFiAPIP(ip, "P-Mixer-AP");
```

## Web Interface

Once WiFi is started, connect to the AP and navigate to the IP address (usually `192.168.4.1`).

### Features:
- **Live Display Cards** - Show current flow, pressure, low pressure, temperature, valve signal, and current
- **Mode Status** - Green box showing current controller mode
- **Real-time Graph** - Updates every 200ms with buffered data (50+ Hz collection rate)
- **6 Data Series** - Flow, Pressure, Low Pressure, Temperature, Valve Signal, Current
- **Lines Only Display** - Smooth line graphs without individual data points for clarity
- **Save Button** - Downloads all data as tab-separated CSV with timestamps
- **Clear Button** - Clears the graph (doesn't affect history)

### API Endpoints:
- `GET /` - Main web interface
- `GET /data` - JSON with current single data point
- `GET /dataBuffer` - JSON with buffered data points (batch fetch)
- `GET /history` - JSON with all historical data

## CSV Export Format

When user clicks "Save Data to CSV", the file contains:

```
Timestamp (ms)	Flow	Pressure	Valve Signal
1234567	10.50	25.30	45.20
1235067	10.55	25.28	45.25
...
```

- **Tab-separated** (easy to import to Excel/Google Sheets)
- **Timestamps** in milliseconds since ESP32 boot
- **Automatic filename** with date/time: `pmixer_data_20241204_143052.csv`

## Memory Management

The server stores data in ESP32 RAM:
- Default: 100 data points × 3 variables = ~1.2 KB
- Configurable via `setMaxDataPoints()`
- Old data automatically trimmed when limit reached
- No SD card or SPIFFS needed

## Complete Integration Example

```cpp
#include "PMixerWiFiServer.hpp"

PMixerWiFiServer wifiServer("P-Mixer", "12345678");

void setup() {
    Serial.begin(115200);
    
    // Start WiFi
    wifiServer.start();
    Serial.println("Connect to: " + wifiServer.getApIpAddress());
}

void loop() {
    // Handle web requests
    wifiServer.handleClient();
    
    // Read your sensors
    float flow = analogRead(FLOW_PIN) * 0.1;
    float pressure = analogRead(PRESSURE_PIN) * 0.05;
    float valve = getValvePosition();
    
    // Update display
    renderer.drawFlow(String(flow, 2));
    renderer.drawPressure(String(pressure, 2));
    
    // Push to web interface
    wifiServer.updateFlow(flow);
    wifiServer.updatePressure(pressure);
    wifiServer.updateValveSignal(valve);
    wifiServer.updateMode(getCurrentMode());
    
    delay(100);
}
```

## Customization

### Change Update Rate

**Data Collection Rate**: Set by `X` command (control_interval). Default: 10ms = 100Hz
```
X10000  // 100Hz data collection
X5000   // 200Hz data collection
X2000   // 500Hz data collection
```

**Web Fetch Rate**: In `PMixerWiFiServer.cpp`, find this line in the HTML:
```javascript
setInterval(fetchData, 200);  // Change 200 to desired milliseconds (currently 200ms = 5Hz fetch)
```

Note: The system buffers data between fetches, so you can have 100Hz data collection with 200ms (5Hz) web fetches. Each fetch retrieves ~20 samples.

### Change Graph Colors

In the Chart.js configuration:
```javascript
borderColor: '#0078D7',  // Flow color
borderColor: '#FF6B6B',  // Pressure color  
borderColor: '#4ECDC4',  // Valve signal color
```

### Add More Data Series

1. Add variables in header file
2. Add to `updateXXX()` methods
3. Add dataset to Chart.js configuration
4. Include in CSV export

## Troubleshooting

**WiFi won't start:**
- Check SSID and password are not empty
- Ensure no other AP running on same SSID
- Try calling `WiFi.softAPdisconnect(true)` before start()

**No data on graph:**
- Verify `wifiServer.handleClient()` is called in loop
- Check you're calling update methods with valid data
- Open browser console (F12) for JavaScript errors

**CSV file empty:**
- Data only saved when `updateValveSignal()` is called
- Check that data points are being added to history
- Verify timestamps array has data

**Out of memory:**
- Reduce `setMaxDataPoints()` to smaller value
- Check you're not creating new server instances repeatedly

## Tips

- Call `updateValveSignal()` last as it triggers data recording
- Use `isRunning()` to conditionally enable/disable WiFi features
- The graph shows last 50 points, but CSV contains all history
- Graph updates without animation for smooth real-time performance

## License

Use freely in your P-Mixer project! 🚀
