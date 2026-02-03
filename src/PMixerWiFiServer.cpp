#include "PMixerWiFiServer.hpp"
#include "VentilatorController.hpp"
#include "chart_js_min_gz.h"

WebServer server(80);

PMixerWiFiServer::PMixerWiFiServer(String ssid, String password)
    : _ssid(ssid), _password(password), _running(false), _maxDataPoints(100), _ventController(nullptr),
      _currentFlow(0.0f), _currentPressure(0.0f), _currentValveSignal(0.0f),
      _currentCurrent(0.0f), _currentLowPressure(0.0f), _currentTemperature(0.0f),
      _currentFlow2(0.0f), _currentPressure2(0.0f),
      _currentMode("Initializing"),
      _ventRunning(false), _ventState("IDLE"),
      _ventRespRate(12.0f), _ventTidalVolume(500.0f), _ventIERatio(0.5f),
      _ventMaxPressure(30.0f), _ventPeep(5.0f), _ventMaxFlow(60.0f),
      _ventTargetFiO2(0.21f), _ventBreathCount(0),
      _ventPeakPressure(0.0f), _ventMeasuredVt(0.0f)
{
    _timestamps.reserve(_maxDataPoints);
    _flowHistory.reserve(_maxDataPoints);
    _pressureHistory.reserve(_maxDataPoints);
    _valveSignalHistory.reserve(_maxDataPoints);
    _currentHistory.reserve(_maxDataPoints);
    _lowPressureHistory.reserve(_maxDataPoints);
    _temperatureHistory.reserve(_maxDataPoints);
    _flow2History.reserve(_maxDataPoints);
    _pressure2History.reserve(_maxDataPoints);
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

void PMixerWiFiServer::updateFlow2(float flow2) {
    _currentFlow2 = flow2;
}

void PMixerWiFiServer::updatePressure2(float pressure2) {
    _currentPressure2 = pressure2;
}

void PMixerWiFiServer::updateMode(const String& mode) {
    _currentMode = mode;
}

void PMixerWiFiServer::updateVentilatorSettings(bool running, const char* state,
                                                 float respRate, float tidalVolume, float ieRatio,
                                                 float maxPressure, float peep, float maxFlow,
                                                 float targetFiO2, uint32_t breathCount,
                                                 float peakPressure, float measuredVt) {
    _ventRunning = running;
    _ventState = state;
    _ventRespRate = respRate;
    _ventTidalVolume = tidalVolume;
    _ventIERatio = ieRatio;
    _ventMaxPressure = maxPressure;
    _ventPeep = peep;
    _ventMaxFlow = maxFlow;
    _ventTargetFiO2 = targetFiO2;
    _ventBreathCount = breathCount;
    _ventPeakPressure = peakPressure;
    _ventMeasuredVt = measuredVt;
}

void PMixerWiFiServer::addDataPoint(float flow, float pressure, float signal, float current,
                                     float lowPressure, float temperature,
                                     float flow2, float pressure2) {
    unsigned long timestamp = millis();

    // Add to main history
    _timestamps.push_back(timestamp);
    _flowHistory.push_back(flow);
    _pressureHistory.push_back(pressure);
    _valveSignalHistory.push_back(signal);
    _currentHistory.push_back(current);
    _lowPressureHistory.push_back(lowPressure);
    _temperatureHistory.push_back(temperature);
    _flow2History.push_back(flow2);
    _pressure2History.push_back(pressure2);

    // Also add to buffer for high-speed web transfer
    // Safety limit: prevent unbounded growth if client disconnects
    // 3000 samples with 9 arrays = ~108KB memory (reduced from 4000 after adding Bus 1 data)
    const size_t MAX_BUFFER_SIZE = 3000;
    if (_bufferTimestamps.size() < MAX_BUFFER_SIZE) {
        _bufferTimestamps.push_back(timestamp);
        _bufferFlow.push_back(flow);
        _bufferPressure.push_back(pressure);
        _bufferValve.push_back(signal);
        _bufferCurrent.push_back(current);
        _bufferLowPressure.push_back(lowPressure);
        _bufferTemperature.push_back(temperature);
        _bufferFlow2.push_back(flow2);
        _bufferPressure2.push_back(pressure2);
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
        _flow2History.erase(_flow2History.begin());
        _pressure2History.erase(_pressure2History.begin());
    }
}

void PMixerWiFiServer::setupWebServer() {
    server.on("/", [this]() { handleRoot(); });
    server.on("/data", [this]() { handleData(); });
    server.on("/history", [this]() { handleHistory(); });
    server.on("/dataBuffer", [this]() { handleDataBuffer(); });
    server.on("/ventilator", [this]() { handleVentilatorSettings(); });
    server.on("/ventilator/set", HTTP_POST, [this]() { handleVentilatorSet(); });
    // Serve Chart.js from PROGMEM (gzip compressed) for offline operation
    server.on("/chart.min.js", []() {
        server.sendHeader("Content-Encoding", "gzip");
        server.sendHeader("Cache-Control", "public, max-age=86400");
        server.send_P(200, "application/javascript", (const char*)chart_js_gz, chart_js_gz_len);
    });
}

void PMixerWiFiServer::handleVentilatorSettings() {
    server.send(200, "application/json", generateVentilatorSettingsJson());
}

void PMixerWiFiServer::handleVentilatorSet() {
    if (!_ventController) {
        server.send(500, "application/json", "{\"error\":\"No ventilator controller\"}");
        return;
    }

    // Parse POST arguments and apply to ventilator controller
    if (server.hasArg("vo")) {
        int val = server.arg("vo").toInt();
        if (val) _ventController->start(); else _ventController->stop();
    }
    if (server.hasArg("rr")) {
        _ventController->setRespRate(server.arg("rr").toFloat());
    }
    if (server.hasArg("vt")) {
        _ventController->setTidalVolume(server.arg("vt").toFloat());
    }
    if (server.hasArg("ie")) {
        _ventController->setIERatio(server.arg("ie").toFloat());
    }
    if (server.hasArg("pi")) {
        _ventController->setMaxPressure(server.arg("pi").toFloat());
    }
    if (server.hasArg("pe")) {
        _ventController->setPEEP(server.arg("pe").toFloat());
    }
    if (server.hasArg("mf")) {
        _ventController->setMaxFlow(server.arg("mf").toFloat());
    }
    if (server.hasArg("fi")) {
        // Input is percentage, convert to fraction
        _ventController->setFiO2(server.arg("fi").toFloat() / 100.0f);
    }
    // Timing fractions
    if (server.hasArg("ip")) {
        _ventController->setInspPauseFraction(server.arg("ip").toFloat());
    }
    if (server.hasArg("i1")) {
        _ventController->setInsp1Fraction(server.arg("i1").toFloat());
    }
    if (server.hasArg("i2")) {
        _ventController->setInsp2Fraction(server.arg("i2").toFloat());
    }
    if (server.hasArg("en")) {
        _ventController->setExpNonTrigFraction(server.arg("en").toFloat());
    }
    if (server.hasArg("es")) {
        _ventController->setExpSyncFraction(server.arg("es").toFloat());
    }
    // Volume/Flow
    if (server.hasArg("tf")) {
        _ventController->setTotalFlow(server.arg("tf").toFloat());
    }
    if (server.hasArg("vc")) {
        _ventController->setUseVolumeControl(server.arg("vc").toInt() != 0);
    }
    // Pressure
    if (server.hasArg("pr")) {
        _ventController->setPressureRampTime(server.arg("pr").toFloat());
    }
    // Trigger
    if (server.hasArg("te")) {
        _ventController->setTriggerEnabled(server.arg("te").toInt() != 0);
    }
    if (server.hasArg("bf")) {
        _ventController->setBiasFlow(server.arg("bf").toFloat());
    }
    if (server.hasArg("ft")) {
        _ventController->setFlowTrigger(server.arg("ft").toFloat());
    }
    if (server.hasArg("pt")) {
        _ventController->setPressureTrigger(server.arg("pt").toFloat());
    }
    // Alarms
    if (server.hasArg("ah")) {
        VentilatorConfig cfg = _ventController->getConfig();
        cfg.highPressureAlarm_mbar = server.arg("ah").toFloat();
        _ventController->setConfig(cfg);
    }
    if (server.hasArg("al")) {
        VentilatorConfig cfg = _ventController->getConfig();
        cfg.lowPressureAlarm_mbar = server.arg("al").toFloat();
        _ventController->setConfig(cfg);
    }
    if (server.hasArg("aa")) {
        VentilatorConfig cfg = _ventController->getConfig();
        cfg.apneaTime_s = server.arg("aa").toFloat();
        _ventController->setConfig(cfg);
    }

    server.send(200, "application/json", "{\"ok\":true}");
}

String PMixerWiFiServer::generateVentilatorSettingsJson() {
    String json = "{";
    json += "\"running\":" + String(_ventRunning ? "true" : "false") + ",";
    json += "\"state\":\"" + _ventState + "\",";
    // Status (from cached values updated by main loop)
    json += "\"breathCount\":" + String(_ventBreathCount) + ",";
    json += "\"peakPressure\":" + String(_ventPeakPressure, 1) + ",";
    json += "\"measuredVt\":" + String(_ventMeasuredVt, 0) + ",";
    // Config (read directly from controller for full parameter set)
    if (_ventController) {
        VentilatorConfig cfg = _ventController->getConfig();
        // Timing
        json += "\"respRate\":" + String(cfg.respRate, 1) + ",";
        json += "\"ieRatio\":" + String(cfg.ieRatio, 2) + ",";
        json += "\"inspPauseFraction\":" + String(cfg.inspPauseFraction, 2) + ",";
        json += "\"insp1Fraction\":" + String(cfg.insp1Fraction, 2) + ",";
        json += "\"insp2Fraction\":" + String(cfg.insp2Fraction, 2) + ",";
        json += "\"expNonTrigFraction\":" + String(cfg.expNonTrigFraction, 2) + ",";
        json += "\"expSyncFraction\":" + String(cfg.expSyncFraction, 2) + ",";
        // Volume/Flow
        json += "\"tidalVolume\":" + String(cfg.tidalVolume_mL, 0) + ",";
        json += "\"maxFlow\":" + String(cfg.maxInspFlow_slm, 1) + ",";
        json += "\"totalFlow\":" + String(cfg.totalFlow_slm, 1) + ",";
        json += "\"useVolumeControl\":" + String(cfg.useVolumeControl ? "true" : "false") + ",";
        // Pressure
        json += "\"maxPressure\":" + String(cfg.maxPressure_mbar, 1) + ",";
        json += "\"peep\":" + String(cfg.peep_mbar, 1) + ",";
        json += "\"pressureRampTime\":" + String(cfg.pressureRampTime_ms, 0) + ",";
        // Gas
        json += "\"targetFiO2\":" + String(cfg.targetFiO2 * 100, 0) + ",";
        // Trigger
        json += "\"triggerEnabled\":" + String(cfg.triggerEnabled ? "true" : "false") + ",";
        json += "\"biasFlow\":" + String(cfg.biasFlow_slm, 1) + ",";
        json += "\"flowTrigger\":" + String(cfg.flowTrigger_slm, 1) + ",";
        json += "\"pressureTrigger\":" + String(cfg.pressureTrigger_mbar, 1) + ",";
        // Alarms
        json += "\"highPressureAlarm\":" + String(cfg.highPressureAlarm_mbar, 1) + ",";
        json += "\"lowPressureAlarm\":" + String(cfg.lowPressureAlarm_mbar, 1) + ",";
        json += "\"apneaTime\":" + String(cfg.apneaTime_s, 1);
    } else {
        // Fallback to cached values
        json += "\"respRate\":" + String(_ventRespRate, 1) + ",";
        json += "\"tidalVolume\":" + String(_ventTidalVolume, 0) + ",";
        json += "\"ieRatio\":" + String(_ventIERatio, 2) + ",";
        json += "\"maxPressure\":" + String(_ventMaxPressure, 1) + ",";
        json += "\"peep\":" + String(_ventPeep, 1) + ",";
        json += "\"maxFlow\":" + String(_ventMaxFlow, 1) + ",";
        json += "\"targetFiO2\":" + String(_ventTargetFiO2 * 100, 0);
    }
    json += "}";
    return json;
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
    json += "\"flow2\":" + String(_currentFlow2, 2) + ",";
    json += "\"pressure2\":" + String(_currentPressure2, 2) + ",";
    json += "\"mode\":\"" + _currentMode + "\",";
    json += "\"timestamp\":" + String(millis());
    json += "}";
    return json;
}

String PMixerWiFiServer::generateHistoryJson() {
    // Pre-reserve: ~80 bytes per sample (9 arrays * ~8 chars + timestamp ~12 chars + commas)
    String json;
    json.reserve(100 + _timestamps.size() * 85);
    json = "{";
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
    json += "],\"flow2\":[";
    for (size_t i = 0; i < _flow2History.size(); i++) {
        if (i > 0) json += ",";
        json += String(_flow2History[i], 2);
    }
    json += "],\"pressure2\":[";
    for (size_t i = 0; i < _pressure2History.size(); i++) {
        if (i > 0) json += ",";
        json += String(_pressure2History[i], 2);
    }
    json += "]}";
    return json;
}

String PMixerWiFiServer::generateBufferJson() {
    // Pre-reserve: ~85 bytes per sample (9 arrays * ~8 chars + timestamp ~12 chars + commas)
    String json;
    json.reserve(100 + _bufferTimestamps.size() * 85);
    json = "{";
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
    json += "],\"flow2\":[";
    for (size_t i = 0; i < _bufferFlow2.size(); i++) {
        if (i > 0) json += ",";
        json += String(_bufferFlow2[i], 2);
    }
    json += "],\"pressure2\":[";
    for (size_t i = 0; i < _bufferPressure2.size(); i++) {
        if (i > 0) json += ",";
        json += String(_bufferPressure2[i], 2);
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
    _bufferFlow2.clear();
    _bufferPressure2.clear();

    return json;
}

String PMixerWiFiServer::generateHtmlPage() {
    String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <title>P-Mixer Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="/chart.min.js"></script>
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
        .settings-input {
            width: 80px;
            font-size: 18px;
            font-weight: bold;
            color: #0078D7;
            border: 2px solid #ccc;
            border-radius: 6px;
            padding: 4px 8px;
            text-align: center;
        }
        .settings-input:focus {
            border-color: #0078D7;
            outline: none;
        }
        .toggle-btn {
            padding: 10px 28px;
            font-size: 18px;
            font-weight: bold;
            border-radius: 10px;
            border: none;
            cursor: pointer;
            box-shadow: 2px 2px 6px rgba(0,0,0,0.3);
        }
        .toggle-on {
            background-color: #28a745;
            color: white;
        }
        .toggle-off {
            background-color: #dc3545;
            color: white;
        }
        .send-btn {
            background-color: #FF8C00;
            color: white;
        }
        .send-btn:hover {
            background-color: #E07800;
        }
        .settings-feedback {
            display: inline-block;
            margin-left: 10px;
            font-weight: bold;
            opacity: 0;
            transition: opacity 0.3s;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>P-Mixer Monitor <span class="update-indicator" id="indicator"></span></h1>
        
        <div class="mode-display" id="modeDisplay">Mode: Initializing...</div>
        
        <div class="status-panel">
            <div class="status-card">
                <div class="status-label">Flow (Bus 0)</div>
                <div class="status-value" id="flowValue">--</div>
            </div>
            <div class="status-card">
                <div class="status-label">Flow (Bus 1)</div>
                <div class="status-value" id="flow2Value">--</div>
            </div>
            <div class="status-card">
                <div class="status-label">Press (Bus 0)</div>
                <div class="status-value" id="pressureValue">--</div>
            </div>
            <div class="status-card">
                <div class="status-label">Press (Bus 1)</div>
                <div class="status-value" id="pressure2Value">--</div>
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
        
        <!-- Main view: Chart -->
        <div id="mainView">
            <div class="chart-container">
                <canvas id="dataChart"></canvas>
            </div>

            <div class="button-container">
                <button onclick="saveDataToTxt()">Save Data</button>
                <button onclick="clearData()">Clear Graph</button>
                <button onclick="showSettings()">Ventilator Settings</button>
            </div>
        </div>

        <!-- Settings view: Ventilator configuration -->
        <div id="settingsView" style="display: none;">
            <h2 style="text-align: center; color: #0078D7;">Ventilator Settings</h2>

            <!-- On/Off toggle and status row -->
            <div style="text-align:center; margin-bottom:15px;">
                <button class="toggle-btn toggle-off" id="ventToggleBtn" onclick="toggleVentilator()">OFF</button>
                <span style="margin-left:20px; font-size:18px;">State: <b id="ventState">--</b></span>
            </div>

            <!-- Timing -->
            <h3 style="color:#0078D7; margin:10px 0 5px 5px;">Timing</h3>
            <div class="status-panel">
                <div class="status-card">
                    <div class="status-label">Resp Rate (BPM)</div>
                    <input class="settings-input" type="number" id="ventRR" step="0.1" min="1" max="60">
                </div>
                <div class="status-card">
                    <div class="status-label">I:E Ratio</div>
                    <input class="settings-input" type="number" id="ventIE" step="0.05" min="0.1" max="4.0">
                </div>
                <div class="status-card">
                    <div class="status-label">Insp Pause Frac</div>
                    <input class="settings-input" type="number" id="ventIP" step="0.01" min="0" max="0.5">
                </div>
                <div class="status-card">
                    <div class="status-label">Insp1 Fraction</div>
                    <input class="settings-input" type="number" id="ventI1" step="0.05" min="0" max="1.0">
                </div>
                <div class="status-card">
                    <div class="status-label">Insp2 Fraction</div>
                    <input class="settings-input" type="number" id="ventI2" step="0.05" min="0" max="1.0">
                </div>
                <div class="status-card">
                    <div class="status-label">Exp NoTrig Frac</div>
                    <input class="settings-input" type="number" id="ventEN" step="0.05" min="0" max="1.0">
                </div>
                <div class="status-card">
                    <div class="status-label">Exp Sync Frac</div>
                    <input class="settings-input" type="number" id="ventES" step="0.05" min="0" max="1.0">
                </div>
            </div>

            <!-- Volume / Flow -->
            <h3 style="color:#0078D7; margin:10px 0 5px 5px;">Volume / Flow</h3>
            <div class="status-panel">
                <div class="status-card">
                    <div class="status-label">Tidal Volume (mL)</div>
                    <input class="settings-input" type="number" id="ventVT" step="10" min="50" max="2000">
                </div>
                <div class="status-card">
                    <div class="status-label">Max Flow (slm)</div>
                    <input class="settings-input" type="number" id="ventMF" step="1" min="1" max="120">
                </div>
                <div class="status-card">
                    <div class="status-label">Total Flow (slm)</div>
                    <input class="settings-input" type="number" id="ventTF" step="1" min="1" max="120">
                </div>
                <div class="status-card">
                    <div class="status-label">Vol Control</div>
                    <select class="settings-input" id="ventVC" style="width:auto">
                        <option value="1">Vt-based</option>
                        <option value="0">Flow-based</option>
                    </select>
                </div>
            </div>

            <!-- Pressure -->
            <h3 style="color:#0078D7; margin:10px 0 5px 5px;">Pressure</h3>
            <div class="status-panel">
                <div class="status-card">
                    <div class="status-label">Max Pressure (mbar)</div>
                    <input class="settings-input" type="number" id="ventPI" step="1" min="5" max="80">
                </div>
                <div class="status-card">
                    <div class="status-label">PEEP (mbar)</div>
                    <input class="settings-input" type="number" id="ventPE" step="0.5" min="0" max="30">
                </div>
                <div class="status-card">
                    <div class="status-label">P Ramp (ms)</div>
                    <input class="settings-input" type="number" id="ventPR" step="10" min="10" max="500">
                </div>
            </div>

            <!-- Gas -->
            <h3 style="color:#0078D7; margin:10px 0 5px 5px;">Gas</h3>
            <div class="status-panel">
                <div class="status-card">
                    <div class="status-label">Target FiO2 (%)</div>
                    <input class="settings-input" type="number" id="ventFiO2" step="1" min="21" max="100">
                </div>
            </div>

            <!-- Trigger -->
            <h3 style="color:#0078D7; margin:10px 0 5px 5px;">Trigger</h3>
            <div class="status-panel">
                <div class="status-card">
                    <div class="status-label">Trigger Enable</div>
                    <select class="settings-input" id="ventTE" style="width:auto">
                        <option value="0">Off</option>
                        <option value="1">On</option>
                    </select>
                </div>
                <div class="status-card">
                    <div class="status-label">Bias Flow (slm)</div>
                    <input class="settings-input" type="number" id="ventBF" step="0.5" min="0" max="10">
                </div>
                <div class="status-card">
                    <div class="status-label">Flow Trigger (slm)</div>
                    <input class="settings-input" type="number" id="ventFT" step="0.5" min="0.5" max="10">
                </div>
                <div class="status-card">
                    <div class="status-label">Press Trigger (mbar)</div>
                    <input class="settings-input" type="number" id="ventPT" step="0.5" min="0.5" max="10">
                </div>
            </div>

            <!-- Alarms -->
            <h3 style="color:#0078D7; margin:10px 0 5px 5px;">Alarms</h3>
            <div class="status-panel">
                <div class="status-card">
                    <div class="status-label">High P Alarm (mbar)</div>
                    <input class="settings-input" type="number" id="ventAH" step="1" min="10" max="80">
                </div>
                <div class="status-card">
                    <div class="status-label">Low P Alarm (mbar)</div>
                    <input class="settings-input" type="number" id="ventAL" step="0.5" min="0" max="20">
                </div>
                <div class="status-card">
                    <div class="status-label">Apnea Time (s)</div>
                    <input class="settings-input" type="number" id="ventAA" step="1" min="5" max="60">
                </div>
            </div>

            <!-- Read-only measurement cards -->
            <div class="status-panel">
                <div class="status-card">
                    <div class="status-label">Breath Count</div>
                    <div class="status-value" id="ventBreaths">--</div>
                </div>
                <div class="status-card">
                    <div class="status-label">Peak Pressure (mbar)</div>
                    <div class="status-value" id="ventPkP">--</div>
                </div>
                <div class="status-card">
                    <div class="status-label">Measured Vt (mL)</div>
                    <div class="status-value" id="ventMVt">--</div>
                </div>
            </div>

            <div class="button-container">
                <button class="send-btn" onclick="sendVentSettings()">Send Settings</button>
                <span class="settings-feedback" id="settingsFeedback"></span>
                <br><br>
                <button onclick="hideSettings()">Back to Monitor</button>
            </div>
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
            temperature: [],
            flow2: [],
            pressure2: []
        };

        // Chart setup
        const ctx = document.getElementById('dataChart').getContext('2d');
        const chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Flow (Bus 0)',
                        data: [],
                        borderColor: '#0078D7',
                        backgroundColor: 'rgba(0, 120, 215, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Press (Bus 0)',
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
                    },
                    {
                        label: 'Flow (Bus 1)',
                        data: [],
                        borderColor: '#00BFFF',
                        backgroundColor: 'rgba(0, 191, 255, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        borderWidth: 2,
                        borderDash: [5, 3]
                    },
                    {
                        label: 'Press (Bus 1)',
                        data: [],
                        borderColor: '#FF4500',
                        backgroundColor: 'rgba(255, 69, 0, 0.1)',
                        tension: 0.3,
                        pointRadius: 0,
                        borderWidth: 2,
                        borderDash: [5, 3]
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
                // Fetch buffered high-speed data for graphing
                const response = await fetch('/dataBuffer');
                const data = await response.json();

                // Fetch current values including Bus 1 data
                const currentResponse = await fetch('/data');
                const currentData = await currentResponse.json();

                // Update mode display
                document.getElementById('modeDisplay').textContent = 'Mode: ' + data.mode;

                // Update Bus 1 display values (from /data endpoint)
                document.getElementById('flow2Value').textContent = currentData.flow2.toFixed(2);
                document.getElementById('pressure2Value').textContent = currentData.pressure2.toFixed(2);

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
                    dataHistory.flow2.push(data.flow2[i]);
                    dataHistory.pressure2.push(data.pressure2[i]);

                    // Add all points to chart
                    chart.data.labels.push(timeInSeconds);
                    chart.data.datasets[0].data.push(data.flow[i]);
                    chart.data.datasets[1].data.push(data.pressure[i]);
                    chart.data.datasets[2].data.push(data.lowPressure[i]);
                    chart.data.datasets[3].data.push(data.temperature[i]);
                    chart.data.datasets[4].data.push(data.valve[i]);
                    chart.data.datasets[5].data.push(data.current[i]);
                    chart.data.datasets[6].data.push(data.flow2[i]);
                    chart.data.datasets[7].data.push(data.pressure2[i]);
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
                dataHistory.flow2 = history.flow2 || [];
                dataHistory.pressure2 = history.pressure2 || [];

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
                    chart.data.datasets[6].data.push(history.flow2 ? history.flow2[i] : 0);
                    chart.data.datasets[7].data.push(history.pressure2 ? history.pressure2[i] : 0);
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

            let txt = 'Timestamp (ms)\tFlow (Bus 0)\tPress (Bus 0)\tLow Pressure\tTemperature (°C)\tValve Signal\tCurrent (A)\tFlow (Bus 1)\tPress (Bus 1)\n';

            for (let i = 0; i < dataHistory.timestamps.length; i++) {
                txt += dataHistory.timestamps[i] + '\t';
                txt += dataHistory.flow[i].toFixed(2) + '\t';
                txt += dataHistory.pressure[i].toFixed(2) + '\t';
                txt += dataHistory.lowPressure[i].toFixed(2) + '\t';
                txt += dataHistory.temperature[i].toFixed(1) + '\t';
                txt += dataHistory.valve[i].toFixed(2) + '\t';
                txt += dataHistory.current[i].toFixed(3) + '\t';
                txt += (dataHistory.flow2[i] || 0).toFixed(2) + '\t';
                txt += (dataHistory.pressure2[i] || 0).toFixed(2) + '\n';
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

        // Clear data (both graph and save buffer)
        function clearData() {
            if (confirm('Clear all data (graph and save buffer)?')) {
                // Clear chart display
                chart.data.labels = [];
                chart.data.datasets.forEach(dataset => dataset.data = []);
                chart.update();
                // Clear data history for file saving
                dataHistory.timestamps = [];
                dataHistory.flow = [];
                dataHistory.pressure = [];
                dataHistory.valve = [];
                dataHistory.current = [];
                dataHistory.lowPressure = [];
                dataHistory.temperature = [];
                dataHistory.flow2 = [];
                dataHistory.pressure2 = [];
            }
        }

        // ============================================================
        // Settings View Functions
        // ============================================================
        let settingsInterval = null;

        function showSettings() {
            document.getElementById('mainView').style.display = 'none';
            document.getElementById('settingsView').style.display = 'block';
            ventSettingsLoaded = false;  // Refresh inputs from ESP
            fetchVentilatorSettings();
            // Update settings every 1 second
            settingsInterval = setInterval(fetchVentilatorSettings, 1000);
        }

        function hideSettings() {
            document.getElementById('settingsView').style.display = 'none';
            document.getElementById('mainView').style.display = 'block';
            if (settingsInterval) {
                clearInterval(settingsInterval);
                settingsInterval = null;
            }
        }

        let ventSettingsLoaded = false;  // Only populate inputs on first load

        async function fetchVentilatorSettings() {
            try {
                const response = await fetch('/ventilator');
                const data = await response.json();

                // Update toggle button
                const btn = document.getElementById('ventToggleBtn');
                if (data.running) {
                    btn.textContent = 'ON';
                    btn.className = 'toggle-btn toggle-on';
                } else {
                    btn.textContent = 'OFF';
                    btn.className = 'toggle-btn toggle-off';
                }
                btn.dataset.running = data.running ? '1' : '0';

                document.getElementById('ventState').textContent = data.state;

                // Only populate input fields on first load (don't overwrite user edits)
                if (!ventSettingsLoaded) {
                    // Timing
                    document.getElementById('ventRR').value = data.respRate.toFixed(1);
                    document.getElementById('ventIE').value = data.ieRatio.toFixed(2);
                    document.getElementById('ventIP').value = (data.inspPauseFraction || 0).toFixed(2);
                    document.getElementById('ventI1').value = (data.insp1Fraction || 1).toFixed(2);
                    document.getElementById('ventI2').value = (data.insp2Fraction || 0).toFixed(2);
                    document.getElementById('ventEN').value = (data.expNonTrigFraction || 0.5).toFixed(2);
                    document.getElementById('ventES').value = (data.expSyncFraction || 0).toFixed(2);
                    // Volume/Flow
                    document.getElementById('ventVT').value = data.tidalVolume.toFixed(0);
                    document.getElementById('ventMF').value = data.maxFlow.toFixed(1);
                    document.getElementById('ventTF').value = (data.totalFlow || 30).toFixed(1);
                    document.getElementById('ventVC').value = data.useVolumeControl ? '1' : '0';
                    // Pressure
                    document.getElementById('ventPI').value = data.maxPressure.toFixed(1);
                    document.getElementById('ventPE').value = data.peep.toFixed(1);
                    document.getElementById('ventPR').value = (data.pressureRampTime || 100).toFixed(0);
                    // Gas
                    document.getElementById('ventFiO2').value = data.targetFiO2.toFixed(0);
                    // Trigger
                    document.getElementById('ventTE').value = data.triggerEnabled ? '1' : '0';
                    document.getElementById('ventBF').value = (data.biasFlow || 2).toFixed(1);
                    document.getElementById('ventFT').value = (data.flowTrigger || 2).toFixed(1);
                    document.getElementById('ventPT').value = (data.pressureTrigger || 2).toFixed(1);
                    // Alarms
                    document.getElementById('ventAH').value = (data.highPressureAlarm || 40).toFixed(1);
                    document.getElementById('ventAL').value = (data.lowPressureAlarm || 3).toFixed(1);
                    document.getElementById('ventAA').value = (data.apneaTime || 20).toFixed(1);
                    ventSettingsLoaded = true;
                }

                // Always update read-only fields
                document.getElementById('ventBreaths').textContent = data.breathCount;
                document.getElementById('ventPkP').textContent = data.peakPressure.toFixed(1);
                document.getElementById('ventMVt').textContent = data.measuredVt.toFixed(0);
            } catch (error) {
                console.error('Error fetching ventilator settings:', error);
            }
        }

        async function sendVentSettings() {
            const params = new URLSearchParams();
            // Timing
            params.append('rr', document.getElementById('ventRR').value);
            params.append('ie', document.getElementById('ventIE').value);
            params.append('ip', document.getElementById('ventIP').value);
            params.append('i1', document.getElementById('ventI1').value);
            params.append('i2', document.getElementById('ventI2').value);
            params.append('en', document.getElementById('ventEN').value);
            params.append('es', document.getElementById('ventES').value);
            // Volume/Flow
            params.append('vt', document.getElementById('ventVT').value);
            params.append('mf', document.getElementById('ventMF').value);
            params.append('tf', document.getElementById('ventTF').value);
            params.append('vc', document.getElementById('ventVC').value);
            // Pressure
            params.append('pi', document.getElementById('ventPI').value);
            params.append('pe', document.getElementById('ventPE').value);
            params.append('pr', document.getElementById('ventPR').value);
            // Gas
            params.append('fi', document.getElementById('ventFiO2').value);
            // Trigger
            params.append('te', document.getElementById('ventTE').value);
            params.append('bf', document.getElementById('ventBF').value);
            params.append('ft', document.getElementById('ventFT').value);
            params.append('pt', document.getElementById('ventPT').value);
            // Alarms
            params.append('ah', document.getElementById('ventAH').value);
            params.append('al', document.getElementById('ventAL').value);
            params.append('aa', document.getElementById('ventAA').value);

            try {
                const response = await fetch('/ventilator/set', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: params.toString()
                });
                const result = await response.json();
                const fb = document.getElementById('settingsFeedback');
                if (result.ok) {
                    fb.textContent = 'Sent!';
                    fb.style.color = '#28a745';
                } else {
                    fb.textContent = 'Error';
                    fb.style.color = '#dc3545';
                }
                fb.style.opacity = 1;
                setTimeout(() => { fb.style.opacity = 0; }, 2000);
                // Reload current values from ESP after sending
                ventSettingsLoaded = false;
            } catch (error) {
                console.error('Error sending settings:', error);
            }
        }

        async function toggleVentilator() {
            const btn = document.getElementById('ventToggleBtn');
            const newState = btn.dataset.running === '1' ? 0 : 1;
            try {
                await fetch('/ventilator/set', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: 'vo=' + newState
                });
            } catch (error) {
                console.error('Error toggling ventilator:', error);
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