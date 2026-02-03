#ifndef PMIXER_WIFI_SERVER_HPP
#define PMIXER_WIFI_SERVER_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <vector>

// Forward declaration
class VentilatorController;

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
    void updateFlow2(float flow2);       // Bus 1 flow
    void updatePressure2(float pressure2); // Bus 1 pressure

    // Ventilator settings update - call from main to push ventilator data
    void updateVentilatorSettings(bool running, const char* state,
                                   float respRate, float tidalVolume, float ieRatio,
                                   float maxPressure, float peep, float maxFlow,
                                   float targetFiO2, uint32_t breathCount,
                                   float peakPressure, float measuredVt);

    // High-speed data buffering - call this at your control rate for high-speed web updates
    void addDataPoint(float flow, float pressure, float signal, float current, float lowPressure, float temperature,
                      float flow2, float pressure2);

    // Ventilator controller reference - enables web-based settings control
    void setVentilatorController(VentilatorController* vc) { _ventController = vc; }

    // Configuration
    void setMaxDataPoints(int points) { _maxDataPoints = points; }
    
private:
    String _ssid;
    String _password;
    bool _running;
    int _maxDataPoints;
    VentilatorController* _ventController;
    
    // Current values
    float _currentFlow;
    float _currentPressure;
    float _currentValveSignal;
    float _currentCurrent;
    float _currentLowPressure;
    float _currentTemperature;
    float _currentFlow2;       // Bus 1 flow
    float _currentPressure2;   // Bus 1 pressure
    String _currentMode;

    // Ventilator settings
    bool _ventRunning;
    String _ventState;
    float _ventRespRate;
    float _ventTidalVolume;
    float _ventIERatio;
    float _ventMaxPressure;
    float _ventPeep;
    float _ventMaxFlow;
    float _ventTargetFiO2;
    uint32_t _ventBreathCount;
    float _ventPeakPressure;
    float _ventMeasuredVt;

    // Data history for graphing
    std::vector<unsigned long> _timestamps;
    std::vector<float> _flowHistory;
    std::vector<float> _pressureHistory;
    std::vector<float> _valveSignalHistory;
    std::vector<float> _currentHistory;
    std::vector<float> _lowPressureHistory;
    std::vector<float> _temperatureHistory;
    std::vector<float> _flow2History;       // Bus 1 flow
    std::vector<float> _pressure2History;   // Bus 1 pressure

    // High-speed data buffer (for buffering data between client requests)
    std::vector<unsigned long> _bufferTimestamps;
    std::vector<float> _bufferFlow;
    std::vector<float> _bufferPressure;
    std::vector<float> _bufferValve;
    std::vector<float> _bufferCurrent;
    std::vector<float> _bufferLowPressure;
    std::vector<float> _bufferTemperature;
    std::vector<float> _bufferFlow2;       // Bus 1 flow
    std::vector<float> _bufferPressure2;   // Bus 1 pressure

    // Server setup
    void setupWebServer();
    void handleRoot();
    void handleData();
    void handleHistory();
    void handleDataBuffer();
    void handleVentilatorSettings();
    void handleVentilatorSet();

    // HTML generation
    String generateHtmlPage();
    String generateDataJson();
    String generateHistoryJson();
    String generateBufferJson();
    String generateVentilatorSettingsJson();

    // Data management
    void trimDataHistory();
};

#endif // PMIXER_WIFI_SERVER_HPP