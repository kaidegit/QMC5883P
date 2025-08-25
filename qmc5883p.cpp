// qmc5883p.cpp
#include "qmc5883p.h"

#include "cmath"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Register addresses */
#define REG_CHIP_ID        0x00
#define REG_DATA_OUT_X_LSB 0x01
#define REG_STATUS         0x09
#define REG_CTL1           0x0A
#define REG_CTL2           0x0B

#define TWO_PI     6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

static const char* const TAG = "QMC5883P";

QMC5883P::QMC5883P(uint8_t addr, i2c_port_num_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin)
    : _addr(addr), _i2c_port(i2c_port), _sda_pin(sda_pin), _scl_pin(scl_pin),
      _offX(0.0f), _offY(0.0f), _offZ(0.0f),
      _scaleX(1.0f), _scaleY(1.0f), _scaleZ(1.0f),
      _lastRawX(0), _lastRawY(0), _lastRawZ(0),
      _lastReadTime(0) {
}

bool QMC5883P::begin() {
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = _i2c_port,
        .sda_io_num = _sda_pin,
        .scl_io_num = _scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &_bus_handle));

    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _addr,
        .scl_speed_hz = 100 * 1000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &i2c_dev_conf, &_dev_handle));

    uint8_t id = 0;
    if (readReg(REG_CHIP_ID, &id, 1)) {
        ESP_LOGI(TAG, "QMC5883P found who am i 0x%x", id);
    } else {
        ESP_LOGE(TAG, "QMC5883P not found");
        return false;
    }

    // Default configuration: 200 Hz, Continuous, ±2G
    writeReg(0x0D, 0x40);
    vTaskDelay(10);
    writeReg(0x29, 0x06);
    vTaskDelay(10);
    writeReg(REG_CTL1, 0xCF);
    vTaskDelay(10);
    writeReg(REG_CTL2, 0x00);
    vTaskDelay(10);

    _lastReadTime = 0;
    return true;
}

bool QMC5883P::readRaw() {
    unsigned long now = esp_timer_get_time() / 1000;
    if (now - _lastReadTime < _minInterval) {
        // Not enough time has passed yet, use cache
        return false;
    }

    uint8_t status;
    if (!readReg(REG_STATUS, &status, 1) || !(status & 0x01)) {
        // No new data
        return false;
    }

    uint8_t buf[6];
    if (!readReg(REG_DATA_OUT_X_LSB, buf, 6)) return false;

    _lastRawX = int16_t(buf[1] << 8 | buf[0]);
    _lastRawY = int16_t(buf[3] << 8 | buf[2]);
    _lastRawZ = int16_t(buf[5] << 8 | buf[4]);
    _lastReadTime = now;
    return true;
}

bool QMC5883P::readXYZ(float *xyz) {
    if (!readRaw()) return false; // No new data

    // Raw data → µT and apply calibration (Hard- + Soft-Iron)
    float x = _lastRawX / 1000.0f;
    float y = _lastRawY / 1000.0f;
    float z = _lastRawZ / 1000.0f;

    xyz[0] = (x - _offX) * _scaleX;
    xyz[1] = (y - _offY) * _scaleY;
    xyz[2] = (z - _offZ) * _scaleZ;
    return true;
}

float QMC5883P::getHeadingDeg(float declDeg) {
    // Try to read new data, otherwise use last cache
    readRaw();

    // Same conversion as in readXYZ, without return value
    float x = (_lastRawX / 1000.0f - _offX) * _scaleX;
    float y = (_lastRawY / 1000.0f - _offY) * _scaleY;

    // 1) Base angle (-π … π)
    float hdg = atan2(y, x);
    // 2) Add declination (deg → rad)
    hdg += declDeg * DEG_TO_RAD;
    // 3) Normalize to 0 … 2π
    if (hdg < 0) hdg += TWO_PI;
    else if (hdg > TWO_PI) hdg -= TWO_PI;
    // 4) Convert to degrees
    return hdg * RAD_TO_DEG;
}

void QMC5883P::setHardIronOffsets(float xOff, float yOff, float zOff) {
    _offX = xOff;
    _offY = yOff;
    _offZ = zOff;
}

void QMC5883P::setSoftIronScales(float scaleX, float scaleY, float scaleZ) {
    _scaleX = scaleX;
    _scaleY = scaleY;
    _scaleZ = scaleZ;
}

bool QMC5883P::readReg(uint8_t reg, uint8_t *buf, uint8_t len) {
    return ESP_OK == i2c_master_transmit_receive(
        _dev_handle, &reg, 1, buf, len, -1
    );
}

bool QMC5883P::writeReg(uint8_t reg, uint8_t val) {
    return ESP_OK == i2c_master_transmit(_dev_handle, &reg, 1,  -1);
}
