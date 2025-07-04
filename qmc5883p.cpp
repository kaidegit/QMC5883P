#include "qmc5883p.h"

/* ---------- Hard-Iron-Offsets (µT) ---------- */
static float _offX = 0.0f;
static float _offY = 0.0f;

/* Register-Adressen */
#define REG_CHIP_ID        0x00
#define REG_DATA_OUT_X_LSB 0x01
#define REG_STATUS         0x09
#define REG_CTL1           0x0A
#define REG_CTL2           0x0B

QMC5883P::QMC5883P(uint8_t addr, TwoWire &bus)
    : _addr(addr), _bus(&bus) {}

bool QMC5883P::begin()
{
    _bus->begin();           // Pins vorher im Sketch setzen (Wire.begin(SDA,SCL))

    uint8_t id;
    if (!readReg(REG_CHIP_ID, &id, 1) || id != 0x80) return false;

    writeReg(0x0D, 0x40); delay(10);
    writeReg(0x29, 0x06); delay(10);
    writeReg(REG_CTL1, 0xCF); delay(10);   // 200 Hz, Continuous
    writeReg(REG_CTL2, 0x00); delay(10);   // ±2 G
    return true;
}

bool QMC5883P::readXYZ(float *xyz)
{
    uint8_t status;
    if (!readReg(REG_STATUS, &status, 1) || !(status & 0x01)) return false;

    uint8_t raw[6];
    if (!readReg(REG_DATA_OUT_X_LSB, raw, 6)) return false;

    int16_t x = int16_t(raw[1] << 8 | raw[0]);
    int16_t y = int16_t(raw[3] << 8 | raw[2]);
    int16_t z = int16_t(raw[5] << 8 | raw[4]);

    

    xyz[0] = x / 1000.0f - _offX;
    xyz[1] = y / 1000.0f - _offY;
    xyz[2] = z / 1000.0f;
    return true;
}

float QMC5883P::getHeadingDeg(float declDeg)
{
    float v[3];
    if (!readXYZ(v)) return NAN;                  // kein neuer Datensatz

    // 1) Grundwinkel berechnen (Y, X!), liefert -π … +π
    float hdg = atan2(v[1], v[0]);

    // 2) Deklination addieren (Grad → Rad)
    hdg += declDeg * DEG_TO_RAD;                 // Arduino-Core kennt DEG_TO_RAD

    // 3) in 0 … 2π normalisieren
    if (hdg < 0)       hdg += TWO_PI;
    else if (hdg > TWO_PI) hdg -= TWO_PI;

    // 4) in Grad zurück
    return hdg * RAD_TO_DEG;
}
void QMC5883P::setHardIronOffsets(float xOff, float yOff)
{
    _offX = xOff;
    _offY = yOff;
}

/* ---------- private Low-Level ---------- */
bool QMC5883P::readReg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    if (_bus->endTransmission(false)) return false;          // Repeated-Start
    if (_bus->requestFrom(_addr, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = _bus->read();
    return true;
}

bool QMC5883P::writeReg(uint8_t reg, uint8_t val)
{
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    _bus->write(val);
    return _bus->endTransmission() == 0;
}
