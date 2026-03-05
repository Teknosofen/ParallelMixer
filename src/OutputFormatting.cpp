// OutputFormatting.cpp - Serial output formatting and WiFi data push
// Extracted from main.cpp

#include "OutputFormatting.hpp"
#include "MainGlobals.hpp"
#include "SensorPolling.hpp"  // getELVH_Pressure/Temperature

// ============================================================================
// Serial Output — multiple quiet_mode formats
// ============================================================================

void outputData() {
  float elvh_p = getELVH_Pressure();
  float elvh_t = getELVH_Temperature();

  switch (sysConfig.quiet_mode) {
    case 0:  // Verbose: Supply Pressure, Low Pressure, Temp, Air flow (Bus0 & Bus1), Valve signal
      hostCom.printf("Paw: %.2f mbar\tT: %.1f\tFlow0: %.3f\tP0: %.1f kPa\tFlow1: %.3f\tP1: %.1f kPa\tValve: %.2f\n",
                     elvh_p,
                     elvh_t,
                     sensorData_bus0.sfm3505_air_flow,
                     sensorData_bus0.supply_pressure,
                     sensorData_bus1.sfm3505_air_flow,
                     sensorData_bus1.supply_pressure,
                     actuator.getValveControlSignal());
      break;

    case 1:  // Quiet - no output
      break;

    case 2:  // Debug: Integrator, Error, Valve, Flow, Air
      {
        ControlState state = actuator.getControlState();
        hostCom.printf("I %.1f E %.1f V %.2f F %.2f Air %.3f\n",
                       state.integrator,
                       state.error,
                       actuator.getValveControlSignal(),
                       sensorData_bus0.flow,
                       sensorData_bus0.sfm3505_air_flow);
      }
      break;

    case 3:  // Special - controller mode + setting + Serial1 actuator data
      {
        // Get controller mode
        ControllerMode mode = actuator.getControllerMode();
        String modeStr;
        switch (mode) {
          case PID_CONTROL:
            modeStr = "PID";
            break;
          case VALVE_SET_VALUE_CONTROL:
            modeStr = "Set";
            break;
          case SINE_CONTROL:
            modeStr = "Sine";
            break;
          case STEP_CONTROL:
            modeStr = "Step";
            break;
          case TRIANGLE_CONTROL:
            modeStr = "Triangle";
            break;
          case SWEEP_CONTROL:
            modeStr = "Sweep";
            break;
          default:
            modeStr = "Unknown";
            break;
        }

        // Get valve control signal and MUX router data (selected channel)
        uint8_t ch = sysConfig.mux_channel;
        float localValveCtrl = actuator.getValveControlSignal();
        float actuatorCurrent = muxRouter.getCurrent(ch);

        // Print unified output: Mode, MUX channel, Setting, and Received data
        if (!muxRouter.isCurrentStale(ch)) {
          hostCom.printf("[%s M%d] V=%.2f%% -> I=%.3fA",
                         modeStr.c_str(), ch, localValveCtrl, actuatorCurrent);

          // Show other available measurements if not stale
          if (!muxRouter.isActualFlowStale(ch)) {
            hostCom.printf(" F=%.2f", muxRouter.getActualFlow(ch));
          }
          if (!muxRouter.isActualPressureStale(ch)) {
            hostCom.printf(" P=%.2f", muxRouter.getActualPressure(ch));
          }
          if (!muxRouter.isBlowerRPMStale(ch)) {
            hostCom.printf(" R=%.1f", muxRouter.getBlowerRPM(ch));
          }
          hostCom.printf("\n");
        } else {
          hostCom.printf("[%s M%d] V=%.2f%% -> Actuator: STALE data\n",
                         modeStr.c_str(), ch, localValveCtrl);
        }
      }
      break;

    case 4:  // Abbreviated: dP, Flow, Valve signal
      hostCom.printf("%.2f %.2f %.2f\n",
                     sensorData_bus0.differential_pressure,
                     sensorData_bus0.flow,
                     actuator.getValveControlSignal());
      break;

    case 5:  // Labeled: same data as q6, with labels for readability
      {
        float localValveCtrl = actuator.getValveControlSignal();
        float actuatorCurrent = muxRouter.getCurrent(sysConfig.mux_channel);
        float o2_hPa = fdo2Initialized ? fdo2Data.oxygenPartialPressure_hPa : -9.9f;
        float o2_percent = fdo2Initialized ?
          fdo2Sensor.convertToPercentO2(fdo2Data.oxygenPartialPressure_hPa, fdo2Data.ambientPressure_mbar) : -9.9f;
        hostCom.printf("SP0:%.1fkPa LP:%.2f T:%.1f Air0:%.3f Air1:%.3f SP1:%.1fkPa V:%.2f I:%.3f O2:%.2fhPa %.2f%%\n",
                       sensorData_bus0.supply_pressure,
                       elvh_p,
                       elvh_t,
                       sensorData_bus0.sfm3505_air_flow,
                       sensorData_bus1.sfm3505_air_flow,
                       sensorData_bus1.supply_pressure,
                       localValveCtrl,
                       actuatorCurrent,
                       o2_hPa,
                       o2_percent);
      }
      break;

    // High-speed TSV: Time_ms, SupplyP0, LowP, Temp, Air0, Air1, SupplyP1, Valve%, Current, O2_hPa, O2%
    case 6:  // High-speed data logging: tab-separated, no labels (minimal bandwidth)
      {
        float localValveCtrl = actuator.getValveControlSignal();
        float actuatorCurrent = muxRouter.getCurrent(sysConfig.mux_channel);
        float o2_hPa = fdo2Initialized ? fdo2Data.oxygenPartialPressure_hPa : -9.9f;
        float o2_percent = fdo2Initialized ?
          fdo2Sensor.convertToPercentO2(fdo2Data.oxygenPartialPressure_hPa, fdo2Data.ambientPressure_mbar) : -9.9f;
        hostCom.printf("%lu\t%.2f\t%.2f\t%.1f\t%.3f\t%.3f\t%.2f\t%.2f\t%.3f\t%.2f\t%.2f\n",
                       millis(),
                       sensorData_bus0.supply_pressure,
                       elvh_p,
                       elvh_t,
                       sensorData_bus0.sfm3505_air_flow,
                       sensorData_bus1.sfm3505_air_flow,
                       sensorData_bus1.supply_pressure,
                       localValveCtrl,
                       actuatorCurrent,
                       o2_hPa,
                       o2_percent);
      }
      break;

    case 7:  // FDO2 Oxygen Sensor data
      if (fdo2Initialized) {
        hostCom.printf("O2: %.2f hPa (%.2f%%) | Temp: %.1f°C | Status: 0x%02X %s\n",
                       fdo2Data.oxygenPartialPressure_hPa,
                       fdo2Sensor.convertToPercentO2(fdo2Data.oxygenPartialPressure_hPa, 
                                                      fdo2Data.ambientPressure_mbar),
                       fdo2Data.temperature_C,
                       fdo2Data.status,
                       fdo2Sensor.getStatusString(fdo2Data.status).c_str());
      } else {
        hostCom.println("FDO2 not initialized");
      }
      break;

    case 8:  // FDO2 Extended/Raw data
      if (fdo2Initialized) {
        hostCom.printf("O2: %.2f hPa | T: %.1f°C | Phase: %.3f° | Signal: %.1f mV | Amb: %.1f mV | P: %.1f mbar | RH: %.1f%%\n",
                       fdo2Data.oxygenPartialPressure_hPa,
                       fdo2Data.temperature_C,
                       fdo2Data.phaseShift_deg,
                       fdo2Data.signalIntensity_mV,
                       fdo2Data.ambientLight_mV,
                       fdo2Data.ambientPressure_mbar,
                       fdo2Data.relativeHumidity_percent);
      } else {
        hostCom.println("FDO2 not initialized");
      }
      break;
  }
}

// ============================================================================
// Serial/GUI output and WiFi server status push (slow loop)
// ============================================================================

void updateSerialOutput() {
  if (sensorsInitialized) {
    outputData();
  }

  // WiFi server: Bus 0 data
  wifiServer.updateFlow(sensorData_bus0.sfm3505_air_flow);
  wifiServer.updatePressure(sensorData_bus0.supply_pressure);
  wifiServer.updateLowPressure(getELVH_Pressure());
  wifiServer.updateTemperature(getELVH_Temperature());
  wifiServer.updateValveSignal(actuator.getValveControlSignal());
  wifiServer.updateCurrent(muxRouter.getCurrent(sysConfig.mux_channel));

  // WiFi server: Bus 1 data
  wifiServer.updateFlow2(sensorData_bus1.sfm3505_air_flow);
  wifiServer.updatePressure2(sensorData_bus1.supply_pressure);

  // WiFi server: Ventilator settings
  VentilatorConfig cfg = ventilator.getConfig();
  VentilatorStatus st = ventilator.getStatus();
  wifiServer.updateVentilatorSettings(
      ventilator.isRunning(),
      ventilator.getStateString(),
      cfg.respRate,
      cfg.tidalVolume_mL,
      cfg.ieRatio,
      cfg.maxPressure_mbar,
      cfg.peep_mbar,
      cfg.maxInspFlow_slm,
      cfg.targetFiO2,
      st.breathCount,
      st.peakPressure_mbar,
      st.measuredVt_mL
  );
}

// ============================================================================
// Buffer high-speed data for WiFi web dashboard
// ============================================================================

void bufferWiFiData() {
  wifiServer.addDataPoint(sensorData_bus0.sfm3505_air_flow,
                         sensorData_bus0.supply_pressure,
                         actuator.getValveControlSignal(),
                         muxRouter.getCurrent(sysConfig.mux_channel),
                         getELVH_Pressure(),
                         getELVH_Temperature(),
                         sensorData_bus1.sfm3505_air_flow,
                         sensorData_bus1.supply_pressure);
}
