/*
 * PMixerWiFiServer Usage Example
 * 
 * This shows how to integrate the WiFi server into your main.cpp
 */

#include "PMixerWiFiServer.hpp"
#include "ImageRenderer.hpp"

// Create WiFi server instance
PMixerWiFiServer wifiServer("P-Mixer-AP", "password123");

// In your setup() function:
void setup() {
    Serial.begin(115200);
    
    // ... other initialization ...
    
    // Start WiFi (or wait for user trigger)
    wifiServer.start();
    
    // Optional: configure max data points to keep in memory
    wifiServer.setMaxDataPoints(200);  // Default is 100
    
    // Display WiFi info on screen
    String apIP = wifiServer.getApIpAddress();
    renderer.drawWiFiAPIP(apIP, "P-Mixer-AP");
}

// In your loop() function:
void loop() {
    // Always call this to handle web requests
    wifiServer.handleClient();
    
    // Your existing code that reads sensors and updates display...
    float currentFlow = readFlowSensor();
    float currentPressure = readPressureSensor();
    float currentValveSignal = getValveControlSignal();
    String currentMode = getControllerMode(); // e.g., "Manual", "Auto", "PID"
    
    // Update display
    renderer.drawFlow(String(currentFlow, 2));
    renderer.drawPressure(String(currentPressure, 2));
    renderer.drawValveCtrlSignal(String(currentValveSignal, 2));
    renderer.drawControllerMode(currentMode);
    
    // Push data to WiFi clients
    wifiServer.updateFlow(currentFlow);
    wifiServer.updatePressure(currentPressure);
    wifiServer.updateValveSignal(currentValveSignal);  // This also adds data point to history
    wifiServer.updateMode(currentMode);
    
    delay(100); // Or whatever your loop timing is
}

// Optional: Functions to start/stop WiFi on demand
void startWiFiFromButton() {
    if (!wifiServer.isRunning()) {
        wifiServer.start();
        renderer.drawWiFiPromt("WiFi Started");
    }
}

void stopWiFiFromButton() {
    if (wifiServer.isRunning()) {
        wifiServer.stop();
        renderer.drawWiFiPromt("WiFi Stopped");
    }
}

// Example: Toggle WiFi with button press
void handleWiFiButton() {
    static bool lastButtonState = HIGH;
    bool buttonState = digitalRead(WIFI_BUTTON_PIN);
    
    if (buttonState == LOW && lastButtonState == HIGH) {
        if (wifiServer.isRunning()) {
            stopWiFiFromButton();
        } else {
            startWiFiFromButton();
        }
        delay(50); // Debounce
    }
    
    lastButtonState = buttonState;
}
