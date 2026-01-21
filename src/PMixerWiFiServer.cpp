#include "PMixerWiFiServer.hpp"

WebServer server(80);

PMixerWiFiServer::PMixerWiFiServer(String ssid, String password)
    : _ssid(ssid), _password(password), _running(false), _maxDataPoints(100),
      _currentFlow(0.0f), _currentPressure(0.0f), _currentValveSignal(0.0f),
      _currentCurrent(0.0f), _currentLowPressure(0.0f), _currentTemperature(0.0f),
      _currentMode("Initializing")
{
    _timestamps.reserve(_maxDataPoints);
    _flowHistory.reserve(_maxDataPoints);
    _pressureHistory.reserve(_maxDataPoints);
    _valveSignalHistory.reserve(_maxDataPoints);
    _currentHistory.reserve(_maxDataPoints);
    _lowPressureHistory.reserve(_maxDataPoints);
    _temperatureHistory.reserve(_maxDataPoints);
}

void PMixerWiFiServer::start() {
    if (_running) return;
    
    WiFi.softAP(_ssid.c_str(), _password.c_str());
    setupWebServer();
    server.begin();
    _running = true;
    
    Serial.println("WiFi AP Started");
    Serial.print("SSID: ");
    Serial.println(_ssid);
    Serial.print("IP: ");
    Serial.println(getApIpAddress());
}

void PMixerWiFiServer::stop() {
    if (!_running) return;
    
    server.stop();
    WiFi.softAPdisconnect(true);
    _running = false;
    
    Serial.println("WiFi AP Stopped");
}

void PMixerWiFiServer::handleClient() {
    if (_running) {
        server.handleClient();
    }
}

String PMixerWiFiServer::getApIpAddress() const {
    return WiFi.softAPIP().toString();
}

void PMixerWiFiServer::updateFlow(float flow) {
    _currentFlow = flow;
}

void PMixerWiFiServer::updatePressure(float pressure) {
    _currentPressure = pressure;
}

void PMixerWiFiServer::updateValveSignal(float signal) {
    _currentValveSignal = signal;
    // Note: addDataPoint() is now called directly from main loop at control_interval rate
}

void PMixerWiFiServer::updateCurrent(float current) {
    _currentCurrent = current;
}

void PMixerWiFiServer::updateLowPressure(float lowPressure) {
    _currentLowPressure = lowPressure;
}

void PMixerWiFiServer::updateTemperature(float temperature) {
    _currentTemperature = temperature;
}

void PMixerWiFiServer::updateMode(const String& mode) {
    _currentMode = mode;
}

void PMixerWiFiServer::addDataPoint(float flow, float pressure, float signal, float current,
                                     float lowPressure, float temperature) {
    unsigned long timestamp = millis();

    // Add to main history
    _timestamps.push_back(timestamp);
    _flowHistory.push_back(flow);
    _pressureHistory.push_back(pressure);
    _valveSignalHistory.push_back(signal);
    _currentHistory.push_back(current);
    _lowPressureHistory.push_back(lowPressure);
    _temperatureHistory.push_back(temperature);

    // Also add to buffer for high-speed web transfer
    // Safety limit: prevent unbounded growth if client disconnects
    // 4000 samples = ~2s at 2kHz, ~4s at 1kHz, ~200KB memory
    const size_t MAX_BUFFER_SIZE = 4000;
    if (_bufferTimestamps.size() < MAX_BUFFER_SIZE) {
        _bufferTimestamps.push_back(timestamp);
        _bufferFlow.push_back(flow);
        _bufferPressure.push_back(pressure);
        _bufferValve.push_back(signal);
        _bufferCurrent.push_back(current);
        _bufferLowPressure.push_back(lowPressure);
        _bufferTemperature.push_back(temperature);
    }
    // When buffer full, new data is dropped until client fetches

    trimDataHistory();
}

void PMixerWiFiServer::trimDataHistory() {
    while (_timestamps.size() > _maxDataPoints) {
        _timestamps.erase(_timestamps.begin());
        _flowHistory.erase(_flowHistory.begin());
        _pressureHistory.erase(_pressureHistory.begin());
        _valveSignalHistory.erase(_valveSignalHistory.begin());
        _currentHistory.erase(_currentHistory.begin());
        _lowPressureHistory.erase(_lowPressureHistory.begin());
        _temperatureHistory.erase(_temperatureHistory.begin());
    }
}

void PMixerWiFiServer::setupWebServer() {
    server.on("/", [this]() { handleRoot(); });
    server.on("/data", [this]() { handleData(); });
    server.on("/history", [this]() { handleHistory(); });
    server.on("/dataBuffer", [this]() { handleDataBuffer(); });  // New endpoint for buffered data
}

void PMixerWiFiServer::handleRoot() {
    server.send(200, "text/html", generateHtmlPage());
}

void PMixerWiFiServer::handleData() {
    server.send(200, "application/json", generateDataJson());
}

void PMixerWiFiServer::handleHistory() {
    server.send(200, "application/json", generateHistoryJson());
}

void PMixerWiFiServer::handleDataBuffer() {
    server.send(200, "application/json", generateBufferJson());
}

String PMixerWiFiServer::generateDataJson() {
    String json = "{";
    json += "\"flow\":" + String(_currentFlow, 2) + ",";
    json += "\"pressure\":" + String(_currentPressure, 2) + ",";
    json += "\"valve\":" + String(_currentValveSignal, 2) + ",";
    json += "\"current\":" + String(_currentCurrent, 3) + ",";
    json += "\"lowPressure\":" + String(_currentLowPressure, 2) + ",";
    json += "\"temperature\":" + String(_currentTemperature, 1) + ",";
    json += "\"mode\":\"" + _currentMode + "\",";
    json += "\"timestamp\":" + String(millis());
    json += "}";
    return json;
}

String PMixerWiFiServer::generateHistoryJson() {
    String json = "{";
    json += "\"timestamps\":[";
    for (size_t i = 0; i < _timestamps.size(); i++) {
        if (i > 0) json += ",";
        json += String(_timestamps[i]);
    }
    json += "],\"flow\":[";
    for (size_t i = 0; i < _flowHistory.size(); i++) {
        if (i > 0) json += ",";
        json += String(_flowHistory[i], 2);
    }
    json += "],\"pressure\":[";
    for (size_t i = 0; i < _pressureHistory.size(); i++) {
        if (i > 0) json += ",";
        json += String(_pressureHistory[i], 2);
    }
    json += "],\"valve\":[";
    for (size_t i = 0; i < _valveSignalHistory.size(); i++) {
        if (i > 0) json += ",";
        json += String(_valveSignalHistory[i], 2);
    }
    json += "],\"current\":[";
    for (size_t i = 0; i < _currentHistory.size(); i++) {
        if (i > 0) json += ",";
        json += String(_currentHistory[i], 3);
    }
    json += "],\"lowPressure\":[";
    for (size_t i = 0; i < _lowPressureHistory.size(); i++) {
        if (i > 0) json += ",";
        json += String(_lowPressureHistory[i], 2);
    }
    json += "],\"temperature\":[";
    for (size_t i = 0; i < _temperatureHistory.size(); i++) {
        if (i > 0) json += ",";
        json += String(_temperatureHistory[i], 1);
    }
    json += "]}";
    return json;
}

String PMixerWiFiServer::generateBufferJson() {
    String json = "{";
    json += "\"count\":" + String(_bufferTimestamps.size()) + ",";
    json += "\"mode\":\"" + _currentMode + "\",";
    json += "\"timestamps\":[";
    for (size_t i = 0; i < _bufferTimestamps.size(); i++) {
        if (i > 0) json += ",";
        json += String(_bufferTimestamps[i]);
    }
    json += "],\"flow\":[";
    for (size_t i = 0; i < _bufferFlow.size(); i++) {
        if (i > 0) json += ",";
        json += String(_bufferFlow[i], 2);
    }
    json += "],\"pressure\":[";
    for (size_t i = 0; i < _bufferPressure.size(); i++) {
        if (i > 0) json += ",";
        json += String(_bufferPressure[i], 2);
    }
    json += "],\"lowPressure\":[";
    for (size_t i = 0; i < _bufferLowPressure.size(); i++) {
        if (i > 0) json += ",";
        json += String(_bufferLowPressure[i], 2);
    }
    json += "],\"temperature\":[";
    for (size_t i = 0; i < _bufferTemperature.size(); i++) {
        if (i > 0) json += ",";
        json += String(_bufferTemperature[i], 1);
    }
    json += "],\"valve\":[";
    for (size_t i = 0; i < _bufferValve.size(); i++) {
        if (i > 0) json += ",";
        json += String(_bufferValve[i], 2);
    }
    json += "],\"current\":[";
    for (size_t i = 0; i < _bufferCurrent.size(); i++) {
        if (i > 0) json += ",";
        json += String(_bufferCurrent[i], 3);
    }
    json += "]}";

    // Clear the buffer after sending
    _bufferTimestamps.clear();
    _bufferFlow.clear();
    _bufferPressure.clear();
    _bufferLowPressure.clear();
    _bufferTemperature.clear();
    _bufferValve.clear();
    _bufferCurrent.clear();

    return json;
}

String PMixerWiFiServer::generateHtmlPage() {
    String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <title>P-Mixer Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
    <style>
        body {
            background-color: #84B6D6;
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            color: #000;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.2);
        }
        h1 {
            text-align: center;
            color: #0078D7;
            margin-top: 0;
        }
        .status-panel {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-bottom: 20px;
        }
        .status-card {
            background-color: #f0f0f0;
            padding: 15px;
            border-radius: 8px;
            border: 2px solid #0078D7;
        }
        .status-label {
            font-size: 14px;
            color: #666;
            margin-bottom: 5px;
        }
        .status-value {
            font-size: 24px;
            font-weight: bold;
            color: #0078D7;
        }
        .mode-display {
            background-color: #e0f8e0;
            border: 2px solid #207520;
            color: #207520;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
            font-size: 20px;
            font-weight: bold;
            margin-bottom: 20px;
        }
        .chart-container {
            position: relative;
            height: 400px;
            margin-bottom: 20px;
        }
        button {
            background-color: #0078D7;
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 10px;
            cursor: pointer;
            font-size: 16px;
            box-shadow: 2px 2px 6px rgba(0,0,0,0.3);
            margin: 5px;
        }
        button:hover {
            background-color: #005FA3;
        }
        .button-container {
            text-align: center;
            margin-top: 20px;
        }
        .update-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            background-color: #00ff00;
            border-radius: 50%;
            margin-left: 10px;
            animation: blink 1s infinite;
        }
        @keyframes blink {
            0%, 50% { opacity: 1; }
            51%, 100% { opacity: 0.3; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>P-Mixer Monitor <span class="update-indicator" id="indicator"></span></h1>
        
        <div class="mode-display" id="modeDisplay">Mode: Initializing...</div>
        
        <div class="status-panel">
            <div class="status-card">
                <div class="status-label">Flow</div>
                <div class="status-value" id="flowValue">--</div>
            </div>
            <div class="status-card">
                <div class="status-label">Pressure</div>
                <div class="status-value" id="pressureValue">--</div>
            </div>
            <div class="status-card">
                <div class="status-label">Low Pressure</div>
                <div class="status-value" id="lowPressureValue">--</div>
            </div>
            <div class="status-card">
                <div class="status-label">Temperature</div>
                <div class="status-value" id="temperatureValue">--</div>
            </div>
            <div class="status-card">
                <div class="status-label">Valve Control Signal</div>
                <div class="status-value" id="valveValue">--</div>
            </div>
            <div class="status-card">
                <div class="status-label">Current</div>
                <div class="status-value" id="currentValue">--</div>
            </div>
        </div>
        
        <div class="chart-container">
            <canvas id="dataChart"></canvas>
        </div>
        
        <div class="button-container">
            <button onclick="saveDataToTxt()">💾 Save Data</button>
            <button onclick="clearData()">🗑️ Clear Graph</button>
        </div>
    </div>

    <script>
        // Data storage
        let dataHistory = {
            timestamps: [],
            flow: [],
            pressure: [],
            valve: [],
            current: [],
            lowPressure: [],
            temperature: []
        };

        // Chart setup
        const ctx = document.getElementById('dataChart').getContext('2d');
        const chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Flow',
                        data: [],
                        borderColor: '#0078D7',
                        backgroundColor: 'rgba(0, 120, 215, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Pressure',
                        data: [],
                        borderColor: '#FF6B6B',
                        backgroundColor: 'rgba(255, 107, 107, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Low Pressure',
                        data: [],
                        borderColor: '#FF69B4',
                        backgroundColor: 'rgba(255, 105, 180, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Temperature',
                        data: [],
                        borderColor: '#9370DB',
                        backgroundColor: 'rgba(147, 112, 219, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Valve Signal',
                        data: [],
                        borderColor: '#4ECDC4',
                        backgroundColor: 'rgba(78, 205, 196, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Current',
                        data: [],
                        borderColor: '#FFA500',
                        backgroundColor: 'rgba(255, 165, 0, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    mode: 'index',
                    intersect: false,
                },
                plugins: {
                    legend: {
                        position: 'top',
                    },
                    title: {
                        display: true,
                        text: 'Real-time Data'
                    }
                },
                scales: {
                    x: {
                        display: true,
                        title: {
                            display: true,
                            text: 'Time (seconds)'
                        }
                    },
                    y: {
                        display: true,
                        title: {
                            display: true,
                            text: 'Value'
                        }
                    }
                }
            }
        });

        // Fetch buffered data (batch of multiple samples for high-speed updates)
        async function fetchData() {
            try {
                const response = await fetch('/dataBuffer');
                const data = await response.json();

                // Update mode display
                document.getElementById('modeDisplay').textContent = 'Mode: ' + data.mode;

                // If no new data points, skip processing
                if (data.count === 0) {
                    return;
                }

                const maxPoints = )rawhtml" + String(PMIXER_GRAPH_DISPLAY_POINTS) + R"rawhtml(;

                // Process all buffered data points
                for (let i = 0; i < data.count; i++) {
                    // Update display values (show latest values only)
                    if (i === data.count - 1) {
                        document.getElementById('flowValue').textContent = data.flow[i].toFixed(2);
                        document.getElementById('pressureValue').textContent = data.pressure[i].toFixed(2);
                        document.getElementById('lowPressureValue').textContent = data.lowPressure[i].toFixed(2);
                        document.getElementById('temperatureValue').textContent = data.temperature[i].toFixed(1) + ' °C';
                        document.getElementById('valveValue').textContent = data.valve[i].toFixed(2);
                        document.getElementById('currentValue').textContent = data.current[i].toFixed(3) + ' A';
                    }

                    // Store all data points to history
                    const timeInSeconds = (data.timestamps[i] / 1000).toFixed(1);
                    dataHistory.timestamps.push(data.timestamps[i]);
                    dataHistory.flow.push(data.flow[i]);
                    dataHistory.pressure.push(data.pressure[i]);
                    dataHistory.valve.push(data.valve[i]);
                    dataHistory.current.push(data.current[i]);
                    dataHistory.lowPressure.push(data.lowPressure[i]);
                    dataHistory.temperature.push(data.temperature[i]);

                    // Add all points to chart
                    chart.data.labels.push(timeInSeconds);
                    chart.data.datasets[0].data.push(data.flow[i]);
                    chart.data.datasets[1].data.push(data.pressure[i]);
                    chart.data.datasets[2].data.push(data.lowPressure[i]);
                    chart.data.datasets[3].data.push(data.temperature[i]);
                    chart.data.datasets[4].data.push(data.valve[i]);
                    chart.data.datasets[5].data.push(data.current[i]);
                }

                // Trim chart to max points
                while (chart.data.labels.length > maxPoints) {
                    chart.data.labels.shift();
                    chart.data.datasets.forEach(dataset => dataset.data.shift());
                }

                // Update chart once with all new data (batch update)
                chart.update('none'); // No animation for smoother high-speed updates
            } catch (error) {
                console.error('Error fetching data:', error);
            }
        }

        // Load historical data on page load
        async function loadHistory() {
            try {
                const response = await fetch('/history');
                const history = await response.json();
                
                dataHistory.timestamps = history.timestamps;
                dataHistory.flow = history.flow;
                dataHistory.pressure = history.pressure;
                dataHistory.valve = history.valve;
                dataHistory.current = history.current;
                dataHistory.lowPressure = history.lowPressure;
                dataHistory.temperature = history.temperature;

                // Populate chart with configured number of points
                const maxPoints = )rawhtml" + String(PMIXER_GRAPH_DISPLAY_POINTS) + R"rawhtml(;
                const startIdx = Math.max(0, history.timestamps.length - maxPoints);
                for (let i = startIdx; i < history.timestamps.length; i++) {
                    const timeInSeconds = (history.timestamps[i] / 1000).toFixed(1);
                    chart.data.labels.push(timeInSeconds);
                    chart.data.datasets[0].data.push(history.flow[i]);
                    chart.data.datasets[1].data.push(history.pressure[i]);
                    chart.data.datasets[2].data.push(history.lowPressure[i]);
                    chart.data.datasets[3].data.push(history.temperature[i]);
                    chart.data.datasets[4].data.push(history.valve[i]);
                    chart.data.datasets[5].data.push(history.current[i]);
                }
                chart.update();
            } catch (error) {
                console.error('Error loading history:', error);
            }
        }

        // Save data to TXT file
        function saveDataToTxt() {
            if (dataHistory.timestamps.length === 0) {
                alert('No data to save!');
                return;
            }

            let txt = 'Timestamp (ms)\tFlow\tPressure\tLow Pressure\tTemperature (°C)\tValve Signal\tCurrent (A)\n';

            for (let i = 0; i < dataHistory.timestamps.length; i++) {
                txt += dataHistory.timestamps[i] + '\t';
                txt += dataHistory.flow[i].toFixed(2) + '\t';
                txt += dataHistory.pressure[i].toFixed(2) + '\t';
                txt += dataHistory.lowPressure[i].toFixed(2) + '\t';
                txt += dataHistory.temperature[i].toFixed(1) + '\t';
                txt += dataHistory.valve[i].toFixed(2) + '\t';
                txt += dataHistory.current[i].toFixed(3) + '\n';
            }

            // Create download
            const blob = new Blob([txt], { type: 'text/plain' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;

            // Generate filename with timestamp
            const now = new Date();
            const filename = 'pmixer_data_' +
                now.getFullYear() +
                ('0' + (now.getMonth() + 1)).slice(-2) +
                ('0' + now.getDate()).slice(-2) + '_' +
                ('0' + now.getHours()).slice(-2) +
                ('0' + now.getMinutes()).slice(-2) +
                ('0' + now.getSeconds()).slice(-2) +
                '.txt';

            a.download = filename;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            window.URL.revokeObjectURL(url);
        }

        // Clear data
        function clearData() {
            if (confirm('Clear all graphed data?')) {
                chart.data.labels = [];
                chart.data.datasets.forEach(dataset => dataset.data = []);
                chart.update();
            }
        }

        // Initialize
        loadHistory();

        // Fetch buffered data every 200ms (allows accumulation of ~10 samples at 50Hz)
        // You can adjust this interval: lower = less latency but more WiFi traffic
        setInterval(fetchData, 200);
    </script>
</body>
</html>
)rawhtml";
    return html;
}