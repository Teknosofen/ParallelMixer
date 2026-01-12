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
    String _currentMode;
    
    // Data history for graphing
    std::vector<unsigned long> _timestamps;
    std::vector<float> _flowHistory;
    std::vector<float> _pressureHistory;
    std::vector<float> _valveSignalHistory;
    std::vector<float> _currentHistory;
    
    // Server setup
    void setupWebServer();
    void handleRoot();
    void handleData();
    void handleHistory();
    
    // HTML generation
    String generateHtmlPage();
    String generateDataJson();
    String generateHistoryJson();
    
    // Data management
    void addDataPoint(float flow, float pressure, float signal, float current);
    void trimDataHistory();
};

#endif // PMIXER_WIFI_SERVER_HPP