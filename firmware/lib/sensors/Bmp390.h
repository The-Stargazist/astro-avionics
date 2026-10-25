#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "Bmp390Compensation.h"

// BMP390 barometer over I2C, free-running in normal mode at 50 Hz with 8x
// pressure / 1x temperature oversampling and IIR coefficient 1.
//
// read() never blocks on a conversion: it polls the status register and
// returns true only when the sensor has produced a new sample, so callers can
// poll as often as they like and use the return value as the sample clock.
class Bmp390 {
public:
    explicit Bmp390(TwoWire& wire, uint8_t address = 0x77);

    // Soft-resets and configures the sensor, then waits for the first sample.
    bool begin();
    // Polls for a new sample. Returns true when one was read; always false
    // until begin() has succeeded.
    bool read();

    bool configured() const { return configured_; }
    float pressurePa() const { return pressurePa_; }
    float temperatureC() const { return temperatureC_; }
    // ISA altitude for a 1013.25 hPa datum. Only differences are meaningful;
    // flight logic works on altitude above the pad.
    float altitudeM() const;

private:
    bool readRegisters(uint8_t reg, uint8_t* data, size_t length);
    bool writeRegister(uint8_t reg, uint8_t value);

    TwoWire& wire_;
    uint8_t address_;
    bool configured_ = false;
    bmp390::Calibration calibration_ = {};
    float pressurePa_ = 0.0f;
    float temperatureC_ = 0.0f;
};
