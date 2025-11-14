#include "SensorManager.h"

#define I2C_SFM 0x40
#define I2C_SPD 0x25
#define I2C_SSC 0x58

// commands
#define SFM_RST 0x20
#define SFM_START 0x10
#define SPD_START 0x36
#define SPD_MODE 0x08

void SensorManager::begin() {
    Wire.begin();
    Wire.setClock(1000000);

    // --- Reset SFM
    Wire.beginTransmission(I2C_SFM);
    Wire.write(SFM_RST);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(70);

    // --- Reset SPD
    Wire.beginTransmission(I2C_SPD);
    Wire.write(0x00);
    Wire.write(0x06);
    Wire.endTransmission();
    delay(10);

    // --- Start SPD continuous
    Wire.beginTransmission(I2C_SPD);
    Wire.write(SPD_START);
    Wire.write(SPD_MODE);
    Wire.endTransmission();
    delay(10);

    // --- Start SFM continuous
    Wire.beginTransmission(I2C_SFM);
    Wire.write(SFM_START);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);
}

float SensorManager::readSPD() {
    Wire.requestFrom(I2C_SPD, 3);
    if (Wire.available() < 3) return 0;

    msb = Wire.read();
    lsb = Wire.read();
    combined = (msb << 8) | lsb;

    return float(combined) * 0.95f / 240.0f;
}

float SensorManager::readSFM() {
    Wire.requestFrom(I2C_SFM, 3);
    if (Wire.available() < 3) return 0;

    msb = Wire.read();
    lsb = Wire.read();
    combined = (msb << 8) | lsb;

    return (float(combined) - 10000.0f) / 120.0f;
}

float SensorManager::readSSC() {
    uint8_t s;
    uint16_t p, t;

    Wire.requestFrom(I2C_SSC, 4);
    if (Wire.available() < 4) return 0;

    msb = Wire.read();
    lsb = Wire.read();
    s = msb >> 6;

    p = ((msb & 0x3F) << 8) | lsb;

    msb = Wire.read();
    lsb = Wire.read();
    t = ((msb << 8) | lsb) >> 5;

    return (p - 1638.0f) * 6.895f / 13107.0f;
}

float SensorManager::fuse(float diffPress, float flow) {
    float diffPressScalefactor = 10.84f;
    float diffPressExponent = 0.558f;

    float flowLowLimit = 1.0f;
    float flowHighLimit = 10.0f;

    if (diffPress < 0) diffPress = 0;

    float dpFlow = diffPressScalefactor * powf(diffPress, diffPressExponent);

    if (flow < flowLowLimit)
        return dpFlow;

    if (flow < flowHighLimit) {
        float w = (flow - flowLowLimit) / (flowHighLimit - flowLowLimit);
        return dpFlow * (1 - w) + flow * w;
    }

    return flow;
}
