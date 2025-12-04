# P-Mixer WiFi Server

Modern WiFi server for your P-Mixer project with real-time data visualization and CSV export.

## Features

✅ **Start/Stop WiFi from main** - Easy control via simple methods  
✅ **Real-time data push** - Updates flow, pressure, valve signals, and mode  
✅ **Live graphing** - Chart.js-based visualization with 3 data series  
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

```cpp
// In setup() or button handler
wifiServer.start();  // Start WiFi AP

// Later, to stop
wifiServer.stop();   // Stop WiFi AP

// Check status
if (wifiServer.isRunning()) {
    // WiFi is active
}
```

### Pushing Data to Web Interface

```cpp
// In your main loop, update values as they change
wifiServer.updateFlow(flowValue);           // Push flow data
wifiServer.updatePressure(pressureValue);   // Push pressure data
wifiServer.updateValveSignal(valveSignal);  // Push valve signal (also records history)
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
- **Live Display Cards** - Show current flow, pressure, and valve signal values
- **Mode Status** - Green box showing current controller mode
- **Real-time Graph** - Updates every 500ms with last 50 data points
- **Save Button** - Downloads all data as tab-separated CSV with timestamps
- **Clear Button** - Clears the graph (doesn't affect history)

### API Endpoints:
- `GET /` - Main web interface
- `GET /data` - JSON with current values
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

In `PMixerWiFiServer.cpp`, find this line in the HTML:
```javascript
setInterval(fetchData, 500);  // Change 500 to desired milliseconds
```

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
