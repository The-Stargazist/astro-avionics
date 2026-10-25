#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "Vec3.h"

// ICM-20649 wide-range IMU over I2C.
// Configured for +/-30 g and +/-2000 dps with both digital low-pass filters
// at ~50 Hz. (+/-4 g, the previous setting, saturates during motor burn.)
class Icm20649 {
public:
    explicit Icm20649(TwoWire& wire, uint8_t address = 0x68);

    bool begin();
    bool read();

    bool configured() const { return configured_; }
    const Vec3& accel() const { return accel_; }  // m/s^2, sensor frame
    const Vec3& gyro() const { return gyro_; }    // deg/s, sensor frame
    float temperatureC() const { return temperatureC_; }

private:
    bool selectBank(uint8_t bank);
    bool readRegisters(uint8_t reg, uint8_t* data, size_t length);
    bool writeRegister(uint8_t reg, uint8_t value);

    TwoWire& wire_;
    uint8_t address_;
    bool configured_ = false;
    Vec3 accel_;
    Vec3 gyro_;
    float temperatureC_ = 0.0f;
};
