// DisplayUpdate.cpp - TFT display and button handling
// Extracted from main.cpp

#include "DisplayUpdate.hpp"
#include "MainGlobals.hpp"
#include "SensorPolling.hpp"  // getELVH_Pressure/Temperature

// ============================================================================
// TFT Display Initialization
// ============================================================================

void initDisplay() {
  renderer.clear();
  renderer.drawLabel();
  renderer.drawStatusField();
  renderer.drawWiFiField();
  renderer.drawWiFiStatusDot(wifiServer.hasClients());
  renderer.drawWiFiAPIP("WiFi OFF      ", "No SSID    ");
  renderer.drawWiFiPromt("Long press: enable");
}

// ============================================================================
// TFT Display Update — Live data fields and O2 at their own rates
// ============================================================================

void updateDisplay() {
  static uint32_t UILoopStartTime = micros();
  if (micros() - UILoopStartTime > SET_UI_UPDATE_TIME) {
    UILoopStartTime = micros();

    if (!showVentilatorSettings) {
      ControllerMode presentMode = actuator.getControllerMode();
      String ctrlMode = "Valve Set";
      switch (presentMode) {
        case PID_CONTROL:          ctrlMode = "PID"; break;
        case VALVE_SET_VALUE_CONTROL: ctrlMode = "Valve Set"; break;
        case SINE_CONTROL:         ctrlMode = "Sine"; break;
        case STEP_CONTROL:         ctrlMode = "Step"; break;
        case TRIANGLE_CONTROL:     ctrlMode = "Triangle"; break;
        case SWEEP_CONTROL:        ctrlMode = "Sweep"; break;
        default:                   ctrlMode = "Unknown"; break;
      }

      String modeWithChannel = ctrlMode + " M" + String(sysConfig.mux_channel);
      renderer.drawControllerMode(modeWithChannel);
      wifiServer.updateMode(modeWithChannel);

      renderer.drawFlow(String(sensorData_bus0.sfm3505_air_flow, 3) + " slm Air");
      renderer.drawFlow2(String(sensorData_bus1.sfm3505_air_flow, 3) + " slm Air");
      String pressureStr = "P0: " + String(sensorData_bus0.supply_pressure, 1) + "kPa LP: " +
                           String(getELVH_Pressure(), 1) + " " +
                           String(getELVH_Temperature(), 1) + " C";
      renderer.drawPressure(pressureStr);
      renderer.drawPressure2("P1: " + String(sensorData_bus1.supply_pressure, 1) + " kPa");
      renderer.drawValveCtrlSignal(String(actuator.getValveControlSignal()));
      renderer.drawCurrent(String(muxRouter.getCurrent(sysConfig.mux_channel), 3) + " A");
      renderer.drawWiFiStatusDot(wifiServer.hasClients());
    }
  }

  // O2 sensor display (slower rate)
  static uint32_t O2DisplayLoopStartTime = micros();
  if (micros() - O2DisplayLoopStartTime > FDO2_SAMPLE_TIME_US) {
    O2DisplayLoopStartTime = micros();

    if (!showVentilatorSettings) {
      if (fdo2Initialized) {
        renderer.drawO2(String(fdo2Data.oxygenPartialPressure_hPa, 1) + " hPa");
      } else {
        renderer.drawO2("N/A");
      }
    }
  }
}

// ============================================================================
// Button Handling — WiFi toggle + ventilator settings display
// ============================================================================

void handleButtons() {
  if (interactionKey1.wasReleased()) {
    if (interactionKey1.wasLongPress()) {
      hostCom.println("Key1 long press (>1s) - Enabling WiFi");
      wifiServer.start();
      renderer.drawWiFiAPIP(wifiServer.getApIpAddress(), PMIXERSSID);
      renderer.drawWiFiPromt("Short press: disable");
    } else {
      hostCom.println("Key1 short press - Disabling WiFi");
      wifiServer.stop();
      hostCom.println("WiFi Access Point stopped");
      renderer.drawWiFiAPIP("WiFi OFF", "No SSID");
      renderer.drawWiFiPromt("Long press: enable");
    }
  }

  if (interactionKey2.wasReleased()) {
    if (interactionKey2.wasLongPress()) {
      hostCom.println("Key2 long press - Showing ventilator settings");
      showVentilatorSettings = true;

      VentilatorConfig cfg = ventilator.getConfig();
      VentilatorStatus st = ventilator.getStatus();

      String line1 = String("Vent: ") + (ventilator.isRunning() ? "ON" : "OFF") +
                     "  " + ventilator.getStateString();
      String line2 = "RR=" + String(cfg.respRate, 1) + " VT=" + String(cfg.tidalVolume_mL, 0) +
                     " IE=" + String(cfg.ieRatio, 2);
      String line3 = "PI=" + String(cfg.maxPressure_mbar, 1) + " PE=" + String(cfg.peep_mbar, 1) +
                     " MF=" + String(cfg.maxInspFlow_slm, 1);
      String line4;
      if (ventilator.isRunning()) {
        line4 = "#" + String(st.breathCount) + " PkP=" + String(st.peakPressure_mbar, 1) +
                " Vt=" + String(st.measuredVt_mL, 0);
      } else {
        line4 = "FiO2=" + String(cfg.targetFiO2 * 100, 0) + "%";
      }

      renderer.showVentilatorSettings(line1.c_str(), line2.c_str(), line3.c_str(), line4.c_str());
    } else {
      if (showVentilatorSettings) {
        hostCom.println("Key2 short press - Returning to live data");
        showVentilatorSettings = false;

        renderer.clear();
        renderer.drawLabel();
        renderer.drawStatusField();
        renderer.drawWiFiField();
        renderer.drawWiFiStatusDot(wifiServer.hasClients());
        if (wifiServer.isRunning()) {
          renderer.drawWiFiAPIP(wifiServer.getApIpAddress(), PMIXERSSID);
          renderer.drawWiFiPromt("Short press: disable");
        } else {
          renderer.drawWiFiAPIP("WiFi OFF", "No SSID");
          renderer.drawWiFiPromt("Long press: enable");
        }
      }
    }
  }
}
