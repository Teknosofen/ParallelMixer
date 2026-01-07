// Detailed I2C diagnostic test for SFM3505 sensor
// This file can be temporarily used to replace main.cpp for testing
// Or copy the relevant sections into your main setup()

#include <Arduino.h>
#include <Wire.h>

#define I2C0_SDA_PIN 43
#define I2C0_SCL_PIN 44
#define SFM3505_ADDRESS 0x2E
#define DISPLAY_POWER_PIN 15

void detailedI2CScan() {
  Serial.println("\n=== DETAILED I2C SCAN ===");
  Serial.println("Scanning all 128 addresses (0x00-0x7F)...\n");

  int devicesFound = 0;

  for (byte address = 0; address < 128; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("✅ Device found at 0x%02X\n", address);
      devicesFound++;
    } else if (address == SFM3505_ADDRESS) {
      // Special debug for SFM3505 address
      Serial.printf("❌ No device at 0x%02X (SFM3505 expected here) - Error: %d\n", address, error);
    }

    delay(5);  // Small delay between scans
  }

  Serial.printf("\n=== Scan complete: %d device(s) found ===\n", devicesFound);

  if (devicesFound == 0) {
    Serial.println("\n⚠️ NO I2C DEVICES FOUND!");
    Serial.println("Possible issues:");
    Serial.println("1. Check wiring connections");
    Serial.println("2. Verify power supply to sensor (3.3V)");
    Serial.println("3. Check if pull-up resistors are present (4.7kΩ)");
    Serial.println("4. Verify sensor is powered on");
    Serial.println("5. Check for loose connections");
    Serial.println("6. Try swapping SDA and SCL pins (common mistake)");
  }
}

void testSFM3505Specifically() {
  Serial.println("\n=== SFM3505 SPECIFIC TEST ===");
  Serial.printf("Testing address 0x%02X...\n", SFM3505_ADDRESS);

  // Test 1: Basic presence check
  Wire.beginTransmission(SFM3505_ADDRESS);
  byte error = Wire.endTransmission();

  Serial.printf("Test 1 - Basic I2C ACK: ");
  if (error == 0) {
    Serial.println("✅ PASS - Device acknowledged");
  } else {
    Serial.printf("❌ FAIL - Error code: %d\n", error);
    Serial.println("   Error codes: 1=too long, 2=NACK on address, 3=NACK on data, 4=other");
    return;
  }

  // Test 2: Send start measurement command
  Serial.println("\nTest 2 - Send start measurement command (0x3603):");
  Wire.beginTransmission(SFM3505_ADDRESS);
  Wire.write(0x36);  // MSB
  Wire.write(0x03);  // LSB
  error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("✅ Command sent successfully");
    delay(10);  // Wait for sensor to start

    // Test 3: Try to read data
    Serial.println("\nTest 3 - Try to read 9 bytes:");
    byte bytesRead = Wire.requestFrom(SFM3505_ADDRESS, (uint8_t)9);
    Serial.printf("   Requested 9 bytes, received: %d bytes\n", bytesRead);

    if (bytesRead > 0) {
      Serial.print("   Raw data: ");
      for (int i = 0; i < bytesRead; i++) {
        byte b = Wire.read();
        Serial.printf("%02X ", b);
      }
      Serial.println();
    }
  } else {
    Serial.printf("❌ Command failed - Error: %d\n", error);
  }
}

void checkI2CWiring() {
  Serial.println("\n=== I2C WIRING CHECK ===");
  Serial.printf("I2C Bus Configuration:\n");
  Serial.printf("  SDA Pin: GPIO%d\n", I2C0_SDA_PIN);
  Serial.printf("  SCL Pin: GPIO%d\n", I2C0_SCL_PIN);
  Serial.printf("  Clock Speed: 100kHz\n");
  Serial.printf("  Expected Device: SFM3505 at 0x%02X\n", SFM3505_ADDRESS);
  Serial.println("\nWiring should be:");
  Serial.println("  SFM3505 VDD  --> 3.3V");
  Serial.println("  SFM3505 GND  --> GND");
  Serial.println("  SFM3505 SDA  --> GPIO43 (with 4.7kΩ pull-up to 3.3V)");
  Serial.println("  SFM3505 SCL  --> GPIO44 (with 4.7kΩ pull-up to 3.3V)");
}

void setup() {
  // Initialize USB Serial
  Serial.begin();
  delay(2500);

  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("  SFM3505 I2C DIAGNOSTIC TOOL");
  Serial.println("========================================");

  // Enable display power (required for T-Display S3)
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  delay(100);

  // Print wiring info
  checkI2CWiring();

  // Initialize I2C
  Serial.println("\n=== Initializing I2C ===");
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, 100000);
  Serial.println("✅ I2C initialized at 100kHz");
  delay(100);

  // Run diagnostic tests
  detailedI2CScan();
  delay(500);

  testSFM3505Specifically();

  Serial.println("\n========================================");
  Serial.println("  DIAGNOSTIC COMPLETE");
  Serial.println("========================================\n");
}

void loop() {
  // Run diagnostic every 5 seconds
  delay(5000);
  Serial.println("\n--- Running diagnostic again ---");
  detailedI2CScan();
  testSFM3505Specifically();
}
