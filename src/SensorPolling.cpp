// SensorPolling.cpp - Sensor reading and polling functions
// Extracted from main.cpp

#include "SensorPolling.hpp"
#include "MainGlobals.hpp"

// ============================================================================
// ELVH Helpers
// ============================================================================

float getELVH_Pressure() {
  return (sensors_bus0 && sensors_bus0->hasELVH()) ? sensorData_bus0.elvh_pressure : sensorData_bus1.elvh_pressure;
}

float getELVH_Temperature() {
  return (sensors_bus0 && sensors_bus0->hasELVH()) ? sensorData_bus0.elvh_temperature : sensorData_bus1.elvh_temperature;
}

// ============================================================================
// ABP2 Pressure Sensors — Asynchronous two-phase read
// ============================================================================

void pollPressureSensors(uint32_t now) {
  // --- Bus 0 ABP2 ---
  static uint32_t lastPressureTime = 0;
  static bool pressureCommandSent = false;

  if (sensorsInitialized && sensors_bus0->hasABP2() && (now - lastPressureTime) >= sysConfig.PressSamplTime) {
    lastPressureTime = now;

    if (!pressureCommandSent) {
      sensors_bus0->startABP2Measurement();
      pressureCommandSent = true;
    } else {
      float pressure;
      uint8_t status;
      if (sensors_bus0->readABP2Pressure(pressure, status)) {
        sensorData_bus0.supply_pressure = pressure;
      }
      pressureCommandSent = false;
    }
  } else if (!sensors_bus0->hasABP2()) {
    sensorData_bus0.supply_pressure = -9.9;
  }

  // --- Bus 1 ABP2 (same async pattern) ---
  static uint32_t lastBus1PressureTime = 0;
  static bool bus1PressureCommandSent = false;

  if (sensors_bus1_initialized && sensors_bus1->hasABP2() && (now - lastBus1PressureTime) >= sysConfig.PressSamplTime) {
    lastBus1PressureTime = now;

    if (!bus1PressureCommandSent) {
      sensors_bus1->startABP2Measurement();
      bus1PressureCommandSent = true;
    } else {
      float pressure;
      uint8_t status;
      if (sensors_bus1->readABP2Pressure(pressure, status)) {
        sensorData_bus1.supply_pressure = pressure;
      }
      bus1PressureCommandSent = false;
    }
  } else if (!sensors_bus1_initialized || !sensors_bus1->hasABP2()) {
    sensorData_bus1.supply_pressure = -9.9;
  }
}

// ============================================================================
// Flow and Low-Pressure Sensors — Fast control loop
// ============================================================================

void readFastSensors(uint32_t now) {
  // SFM3505 Flow Sensor - Bus 0
  if (sensorsInitialized && sensors_bus0->hasSFM3505()) {
    float sfm_air, sfm_o2;
    if (sensors_bus0->readSFM3505AllFlows(sfm_air, sfm_o2)) {
      sensorData_bus0.sfm3505_air_flow = sfm_air;
      sensorData_bus0.sfm3505_o2_flow = sfm_o2;
    } else {
      sensorData_bus0.sfm3505_air_flow = -9.9;
      sensorData_bus0.sfm3505_o2_flow = -9.9;
    }
  } else if (sensorsInitialized && sensors_bus0->hasSFM3304()) {
    // SFM3304 Flow Sensor - Bus 0 (alternative to SFM3505)
    float flow, temp;
    uint16_t status;
    if (sensors_bus0->readSFM3304Data(flow, temp, status)) {
      sensorData_bus0.sfm3304_flow = flow;
      sensorData_bus0.sfm3304_temperature = temp;
      sensorData_bus0.sfm3304_status_word = status;
      // Mirror to sfm3505_air_flow so existing control loops work transparently
      sensorData_bus0.sfm3505_air_flow = flow;
      sensorData_bus0.sfm3505_o2_flow = -9.9;  // SFM3304 is single-gas
    } else {
      sensorData_bus0.sfm3304_flow = -9.9;
      sensorData_bus0.sfm3304_temperature = -9.9;
      sensorData_bus0.sfm3505_air_flow = -9.9;
      sensorData_bus0.sfm3505_o2_flow = -9.9;
    }
  } else {
    sensorData_bus0.sfm3505_air_flow = -9.9;
    sensorData_bus0.sfm3505_o2_flow = -9.9;
    sensorData_bus0.sfm3304_flow = -9.9;
  }

  // ELVH Low Pressure Sensor - Bus 0 (direct read, no command phase)
  static uint32_t lastELVHTime = 0;
  if (sensorsInitialized && sensors_bus0->hasELVH() && (now - lastELVHTime) >= sysConfig.PressSamplTime) {
    lastELVHTime = now;

    float elvh_pressure, elvh_temp;
    uint8_t elvh_status;
    if (sensors_bus0->readELVHPressureTemp(elvh_pressure, elvh_temp, elvh_status)) {
      sensorData_bus0.elvh_pressure = elvh_pressure;
      sensorData_bus0.elvh_temperature = elvh_temp;
    } else {
      sensorData_bus0.elvh_pressure = -9.9;
      sensorData_bus0.elvh_temperature = -9.9;
    }
  } else if (!sensors_bus0->hasELVH()) {
    sensorData_bus0.elvh_pressure = -9.9;
    sensorData_bus0.elvh_temperature = -9.9;
  }

  // SFM3505 Flow Sensor - Bus 1
  if (sensors_bus1_initialized && sensors_bus1->hasSFM3505()) {
    float sfm_air, sfm_o2;
    if (sensors_bus1->readSFM3505AllFlows(sfm_air, sfm_o2)) {
      sensorData_bus1.sfm3505_air_flow = sfm_air;
      sensorData_bus1.sfm3505_o2_flow = sfm_o2;
    } else {
      sensorData_bus1.sfm3505_air_flow = -9.9;
      sensorData_bus1.sfm3505_o2_flow = -9.9;
    }
  } else if (sensors_bus1_initialized && sensors_bus1->hasSFM3304()) {
    // SFM3304 Flow Sensor - Bus 1 (alternative to SFM3505)
    float flow, temp;
    uint16_t status;
    if (sensors_bus1->readSFM3304Data(flow, temp, status)) {
      sensorData_bus1.sfm3304_flow = flow;
      sensorData_bus1.sfm3304_temperature = temp;
      sensorData_bus1.sfm3304_status_word = status;
      sensorData_bus1.sfm3505_air_flow = flow;
      sensorData_bus1.sfm3505_o2_flow = -9.9;
    } else {
      sensorData_bus1.sfm3304_flow = -9.9;
      sensorData_bus1.sfm3304_temperature = -9.9;
      sensorData_bus1.sfm3505_air_flow = -9.9;
      sensorData_bus1.sfm3505_o2_flow = -9.9;
    }
  } else {
    sensorData_bus1.sfm3505_air_flow = -9.9;
    sensorData_bus1.sfm3505_o2_flow = -9.9;
    sensorData_bus1.sfm3304_flow = -9.9;
  }

  // ELVH Low Pressure Sensor - Bus 1 (direct read, no command phase)
  static uint32_t lastELVHTime_bus1 = 0;
  if (sensors_bus1_initialized && sensors_bus1->hasELVH() && (now - lastELVHTime_bus1) >= sysConfig.PressSamplTime) {
    lastELVHTime_bus1 = now;

    float elvh_pressure, elvh_temp;
    uint8_t elvh_status;
    if (sensors_bus1->readELVHPressureTemp(elvh_pressure, elvh_temp, elvh_status)) {
      sensorData_bus1.elvh_pressure = elvh_pressure;
      sensorData_bus1.elvh_temperature = elvh_temp;
    } else {
      sensorData_bus1.elvh_pressure = -9.9;
      sensorData_bus1.elvh_temperature = -9.9;
    }
  } else if (!sensors_bus1_initialized || !sensors_bus1->hasELVH()) {
    sensorData_bus1.elvh_pressure = -9.9;
    sensorData_bus1.elvh_temperature = -9.9;
  }
}

// ============================================================================
// FDO2 Optical Oxygen Sensor — Async non-blocking read
// ============================================================================

void pollFDO2(uint32_t now) {
  static uint32_t lastFDO2StartTime = 0;
  static bool fdo2MeasurementPending = false;

  if (!fdo2Initialized) return;

  if (fdo2MeasurementPending && fdo2Sensor.isResponseReady()) {
    if (fdo2Sensor.getAsyncResult(fdo2Data)) {
      if (fdo2Sensor.hasFatalError(fdo2Data.status)) {
        if (sysConfig.quiet_mode == 0) {
          hostCom.printf("⚠️ FDO2 Fatal Error: %s\n",
                         fdo2Sensor.getStatusString(fdo2Data.status).c_str());
        }
      } else if (sysConfig.quiet_mode == 0 && fdo2Sensor.hasWarning(fdo2Data.status)) {
        hostCom.printf("⚠️ FDO2 Warning: %s\n",
                       fdo2Sensor.getStatusString(fdo2Data.status).c_str());
      }
    }
    fdo2MeasurementPending = false;
  }

  if (!fdo2MeasurementPending && (now - lastFDO2StartTime) >= FDO2_SAMPLE_TIME_US) {
    lastFDO2StartTime = now;
    if (fdo2Sensor.startMeasurementAsync(true)) {
      fdo2MeasurementPending = true;
    }
  }
}
