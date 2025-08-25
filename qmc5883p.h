// qmc5883p.h
#pragma once
#include <Arduino.h>
#include <Wire.h>

class QMC5883P {
public:
    // Constructor: optionally different SDA/SCL pins & I²C address
    QMC5883P(uint8_t addr = 0x2C, TwoWire &bus = Wire);

    bool begin();                              // Init, true = OK
    bool readXYZ(float *xyz);                  // xyz[3] → µT, true = new data and calibrated
    float getHeadingDeg(float declDeg = 0.0f); // Heading calculation with internal data caching
    void setHardIronOffsets(float xOff, float yOff, float zOff = 0.0f);
    void setSoftIronScales(float scaleX, float scaleY, float scaleZ = 1.0f);

private:
    uint8_t _addr;
    TwoWire *_bus;

    // Calibration parameters
    float _offX, _offY, _offZ;
    float _scaleX, _scaleY, _scaleZ;

    // Buffer for raw measurement values and time
    int16_t _lastRawX, _lastRawY, _lastRawZ;
    unsigned long _lastReadTime;
    static const unsigned long _minInterval = 5; // minimum time interval between reads in ms

    bool readReg(uint8_t reg, uint8_t *buf, uint8_t len);
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRaw(); // reads raw data, updates cache only if DRDY is set and enough time has passed
};
