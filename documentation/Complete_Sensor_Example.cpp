/*
 * Complete Sensor Integration Example
 * 
 * This example shows how to use BOTH:
 * - Legacy sensors (SPD, SFM at 0x40, SSC)
 * - New SFM3505 sensor (at 0x2E)
 * 
 * All sensors work together without conflict!
 */

#include <Arduino.h>
#include "PinConfig.h"
#include "SensorReader.hpp"
#include "main.hpp"

// Create sensor readers on separate I2C buses
SensorReader sensors_bus0(&Wire, "Bus0");
SensorReader sensors_bus1(&Wire1, "Bus1");

SensorData sensorData_bus0;
SensorData sensorData_bus1;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== Complete Sensor System Example ===\n");
  
  // Enable display power (if using T-Display S3)
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);
  delay(100);
  
  // Initialize I2C buses
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, 500000);
  Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 500000);
  
  Serial.println("I2C buses initialized\n");
  
  // Initialize ALL sensors (both legacy and SFM3505)
  sensors_bus0.initialize();
  sensors_bus1.initialize();
  
  Serial.println("\nSetup complete!\n");
  Serial.println("Reading from:");
  Serial.println("  - Legacy SFM (0x40) - differential pressure sensor");
  Serial.println("  - SPD (0x25) - differential pressure");
  Serial.println("  - SSC (0x58) - supply pressure");
  Serial.println("  - SFM3505 (0x2E) - new flow sensor\n");
}

void loop() {
  static uint32_t lastPrint = 0;
  
  // Read sensors every 100ms
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    
    Serial.println("=== Bus 0 Sensors ===");
    
    // ========================================================================
    // Method 1: Read LEGACY sensors (original functionality)
    // ========================================================================
    sensors_bus0.update(sensorData_bus0);
    
    Serial.printf("  Legacy Flow (0x40):      %.2f L/min\n", sensorData_bus0.flow);
    Serial.printf("  Diff Pressure (SPD):     %.2f mBar\n", sensorData_bus0.differential_pressure);
    Serial.printf("  Supply Pressure (SSC):   %.2f PSI\n", sensorData_bus0.supply_pressure);
    
    // ========================================================================
    // Method 2: Read NEW SFM3505 sensor
    // ========================================================================
    float sfm3505_air, sfm3505_o2;
    if (sensors_bus0.readSFM3505AllFlows(sfm3505_air, sfm3505_o2)) {
      Serial.printf("  SFM3505 Air Flow:        %.2f slm\n", sfm3505_air);
      Serial.printf("  SFM3505 O2 Flow:         %.2f slm\n", sfm3505_o2);
      
      // Store in SensorData if needed
      sensorData_bus0.sfm3505_air_flow = sfm3505_air;
      sensorData_bus0.sfm3505_o2_flow = sfm3505_o2;
    } else {
      Serial.println("  SFM3505: Not available");
    }
    
    Serial.println("\n=== Bus 1 Sensors ===");
    
    // Read Bus 1 sensors (same pattern)
    sensors_bus1.update(sensorData_bus1);
    
    Serial.printf("  Legacy Flow (0x40):      %.2f L/min\n", sensorData_bus1.flow);
    Serial.printf("  Diff Pressure (SPD):     %.2f mBar\n", sensorData_bus1.differential_pressure);
    Serial.printf("  Supply Pressure (SSC):   %.2f PSI\n", sensorData_bus1.supply_pressure);
    
    if (sensors_bus1.readSFM3505AllFlows(sfm3505_air, sfm3505_o2)) {
      Serial.printf("  SFM3505 Air Flow:        %.2f slm\n", sfm3505_air);
      Serial.printf("  SFM3505 O2 Flow:         %.2f slm\n", sfm3505_o2);
    } else {
      Serial.println("  SFM3505: Not available");
    }
    
    Serial.println("\n" + String('-', 50) + "\n");
  }
}

// ============================================================================
// Additional Examples
// ============================================================================

void exampleLegacyOnly() {
  // If you only have legacy sensors (no SFM3505)
  SensorData data;
  sensors_bus0.update(data);
  
  Serial.printf("Flow: %.2f, Pressure: %.2f, Supply: %.2f\n",
                data.flow, data.differential_pressure, data.supply_pressure);
}

void exampleSFM3505Only() {
  // If you only want to use SFM3505 (ignore legacy sensors)
  float airFlow, o2Flow;
  
  if (sensors_bus0.readSFM3505AllFlows(airFlow, o2Flow)) {
    Serial.printf("SFM3505 - Air: %.2f slm, O2: %.2f slm\n", airFlow, o2Flow);
  }
}

void exampleBothSensors() {
  // Using both legacy and SFM3505 for redundancy/comparison
  SensorData data;
  sensors_bus0.update(data);
  
  float sfm3505_air;
  sensors_bus0.readSFM3505AirFlow(sfm3505_air);
  
  // Compare legacy flow with SFM3505
  Serial.printf("Legacy Flow:  %.2f L/min\n", data.flow);
  Serial.printf("SFM3505 Flow: %.2f slm\n", sfm3505_air);
  
  // Note: Units are different! slm vs L/min
  // You may need to apply corrections based on temperature/pressure
}

void exampleSensorFusion() {
  // Advanced: Combine data from multiple sensors
  SensorData data;
  sensors_bus0.update(data);
  
  float sfm3505_air;
  sensors_bus0.readSFM3505AirFlow(sfm3505_air);
  
  // Use SFM3505 as primary, legacy as backup
  float primaryFlow = sfm3505_air;
  float backupFlow = data.flow;
  
  // Sensor validation
  if (abs(primaryFlow - backupFlow) > 5.0) {
    Serial.println("Warning: Flow sensor mismatch!");
  }
  
  // Use weighted average or select best sensor
  float fusedFlow = (primaryFlow + backupFlow) / 2.0;
}

void exampleDualBusDualSensors() {
  // Maximum configuration: 
  // Bus 0: Legacy sensors + SFM3505
  // Bus 1: Legacy sensors + SFM3505 (second set)
  
  SensorData data_bus0, data_bus1;
  float sfm_air_bus0, sfm_o2_bus0;
  float sfm_air_bus1, sfm_o2_bus1;
  
  // Read Bus 0
  sensors_bus0.update(data_bus0);
  sensors_bus0.readSFM3505AllFlows(sfm_air_bus0, sfm_o2_bus0);
  
  // Read Bus 1
  sensors_bus1.update(data_bus1);
  sensors_bus1.readSFM3505AllFlows(sfm_air_bus1, sfm_o2_bus1);
  
  Serial.println("=== All Sensors ===");
  Serial.printf("Bus 0 Legacy Flow: %.2f, SFM3505: %.2f slm\n", 
                data_bus0.flow, sfm_air_bus0);
  Serial.printf("Bus 1 Legacy Flow: %.2f, SFM3505: %.2f slm\n", 
                data_bus1.flow, sfm_air_bus1);
}
