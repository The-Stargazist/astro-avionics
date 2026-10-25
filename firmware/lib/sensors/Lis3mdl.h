#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "Vec3.h"

// LIS3MDL magnetometer over I2C: +/-4 gauss, ultra-high-performance mode on
// all axes, 80 Hz continuous conversion, block data update.
class Lis3mdl {
public:
    explicit Lis3mdl(TwoWire& wire, uint8_t address = 0x1E);

    bool begin();
    bool read();

    bool configured() const { return configured_; }
    const Vec3& field() const { return field_; }  // gauss, sensor frame
    float temperatureC() const { return temperatureC_; }

private:
    bool readRegisters(uint8_t reg, uint8_t* data, size_t length);
    bool writeRegister(uint8_t reg, uint8_t value);

    TwoWire& wire_;
    uint8_t address_;
    bool configured_ = false;
    Vec3 field_;
    float temperatureC_ = 0.0f;
};
