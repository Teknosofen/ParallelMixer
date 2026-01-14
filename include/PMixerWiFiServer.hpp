#ifndef PMIXER_WIFI_SERVER_HPP
#define PMIXER_WIFI_SERVER_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <vector>

// Compile-time configuration for graph display
#ifndef PMIXER_GRAPH_DISPLAY_POINTS
#define PMIXER_GRAPH_DISPLAY_POINTS 512  // Default: 512 samples shown on graph
#endif

class PMixerWiFiServer {
public:
    PMixerWiFiServer(String ssid, String password);
    
    // WiFi control methods - call from main
    void start();
    void stop();
    void handleClient();
    bool isRunning() const { return _running; }
    String getApIpAddress() const;
    
    // Data update methods - call these to push data to clients
    void updateFlow(float flow);
    void updatePressure(float pressure);
    void updateValveSignal(float signal);
    void updateCurrent(float current);
    void updateMode(const String& mode);
    void updateLowPressure(float lowPressure);
    void updateTemperature(float temperature);

    // High-speed data buffering - call this at your control rate for high-speed web updates
    void addDataPoint(float flow, float pressure, float signal, float current, float lowPressure, float temperature);

    // Configuration
    void setMaxDataPoints(int points) { _maxDataPoints = points; }
    
private:
    String _ssid;
    String _password;
    bool _running;
    int _maxDataPoints;
    
    // Current values
    float _currentFlow;
    float _currentPressure;
    float _currentValveSignal;
    float _currentCurrent;
    float _currentLowPressure;
    float _currentTemperature;
    String _currentMode;

    // Data history for graphing
    std::vector<unsigned long> _timestamps;
    std::vector<float> _flowHistory;
    std::vector<float> _pressureHistory;
    std::vector<float> _valveSignalHistory;
    std::vector<float> _currentHistory;
    std::vector<float> _lowPressureHistory;
    std::vector<float> _temperatureHistory;

    // High-speed data buffer (for buffering data between client requests)
    std::vector<unsigned long> _bufferTimestamps;
    std::vector<float> _bufferFlow;
    std::vector<float> _bufferPressure;
    std::vector<float> _bufferValve;
    std::vector<float> _bufferCurrent;
    std::vector<float> _bufferLowPressure;
    std::vector<float> _bufferTemperature;

    // Server setup
    void setupWebServer();
    void handleRoot();
    void handleData();
    void handleHistory();
    void handleDataBuffer();  // New: Handle buffered data requests
    
    // HTML generation
    String generateHtmlPage();
    String generateDataJson();
    String generateHistoryJson();
    String generateBufferJson();  // New: Generate JSON for buffered data

    // Data management
    void trimDataHistory();
};

#endif // PMIXER_WIFI_SERVER_HPP