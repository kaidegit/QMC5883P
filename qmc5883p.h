#pragma once
#include <Arduino.h>
#include <Wire.h>

class QMC5883P {
public:
    // Konstruktor: optional andere SDA/SCL-Pins & I²C-Adresse
    QMC5883P(uint8_t addr = 0x2C, TwoWire &bus = Wire);

    bool begin();                    // Init, true = OK
    bool readXYZ(float *xyz);        // xyz[3] → µT, true = neue Daten
	float getHeadingDeg(float declDeg = 0.0f); 
	void setHardIronOffsets(float xOff, float yOff);	

private:
    uint8_t _addr;
    TwoWire *_bus;

    bool readReg(uint8_t reg, uint8_t *buf, uint8_t len);
    bool writeReg(uint8_t reg, uint8_t val);
};
