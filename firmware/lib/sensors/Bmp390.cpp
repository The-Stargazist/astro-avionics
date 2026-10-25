#include "Bmp390.h"

#include <math.h>

namespace {

constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_ERR = 0x02;
constexpr uint8_t REG_STATUS = 0x03;      // followed by DATA_0..DATA_5 (0x04..0x09)
constexpr uint8_t REG_PWR_CTRL = 0x1B;
constexpr uint8_t REG_OSR = 0x1C;
constexpr uint8_t REG_ODR = 0x1D;
constexpr uint8_t REG_CONFIG = 0x1F;
constexpr uint8_t REG_CALIBRATION = 0x31;
constexpr uint8_t REG_CMD = 0x7E;

constexpr uint8_t CHIP_ID_BMP390 = 0x60;
constexpr uint8_t CHIP_ID_BMP388 = 0x50;
constexpr uint8_t CMD_SOFT_RESET = 0xB6;

constexpr uint8_t STATUS_DRDY_PRESS = 1 << 5;
constexpr uint8_t ERR_ANY = 0x07;          // fatal_err | cmd_err | conf_err

constexpr uint8_t PWR_NORMAL_PT = 0x33;    // mode=normal, press_en, temp_en
constexpr uint8_t OSR_P8_T1 = 0x03;        // osr_p = x8, osr_t = x1
constexpr uint8_t ODR_50HZ = 0x02;         // valid for x8/x1 (~19 ms conversion)
constexpr uint8_t IIR_COEF_1 = 0x02;       // iir_filter = coef 1

constexpr float SEA_LEVEL_PA = 101325.0f;

}  // namespace

Bmp390::Bmp390(TwoWire& wire, uint8_t address) : wire_(wire), address_(address) {}

bool Bmp390::readRegisters(uint8_t reg, uint8_t* data, size_t length) {
    wire_.beginTransmission(address_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) {
        return false;
    }
    if (wire_.requestFrom(address_, length) != length) {
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        data[i] = static_cast<uint8_t>(wire_.read());
    }
    return true;
}

bool Bmp390::writeRegister(uint8_t reg, uint8_t value) {
    wire_.beginTransmission(address_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
}

bool Bmp390::begin() {
    configured_ = false;

    uint8_t id = 0;
    if (!readRegisters(REG_CHIP_ID, &id, 1) || (id != CHIP_ID_BMP390 && id != CHIP_ID_BMP388)) {
        return false;
    }
    if (!writeRegister(REG_CMD, CMD_SOFT_RESET)) {
        return false;
    }
    delay(10);  // datasheet start-up time is 2 ms

    uint8_t raw[bmp390::CALIBRATION_LENGTH];
    if (!readRegisters(REG_CALIBRATION, raw, sizeof(raw))) {
        return false;
    }
    calibration_ = bmp390::parseCalibration(raw);

    // Settings must be written in sleep mode (the state after reset).
    if (!writeRegister(REG_OSR, OSR_P8_T1) || !writeRegister(REG_ODR, ODR_50HZ) ||
        !writeRegister(REG_CONFIG, IIR_COEF_1) || !writeRegister(REG_PWR_CTRL, PWR_NORMAL_PT)) {
        return false;
    }

    // An invalid OSR/ODR combination sets conf_err and leaves the sensor asleep.
    uint8_t err = 0;
    uint8_t power = 0;
    if (!readRegisters(REG_ERR, &err, 1) || (err & ERR_ANY) != 0 ||
        !readRegisters(REG_PWR_CTRL, &power, 1) || power != PWR_NORMAL_PT) {
        return false;
    }

    configured_ = true;
    const uint32_t start = millis();
    while (millis() - start < 100) {
        if (read()) {
            return true;
        }
        delay(2);
    }
    configured_ = false;
    return false;
}

bool Bmp390::read() {
    uint8_t data[7];  // STATUS, then pressure and temperature, 24-bit little-endian
    if (!configured_ || !readRegisters(REG_STATUS, data, sizeof(data))) {
        return false;
    }
    if ((data[0] & STATUS_DRDY_PRESS) == 0) {
        return false;  // no new sample since the last read
    }
    const uint32_t rawPressure = data[1] | (data[2] << 8) | (static_cast<uint32_t>(data[3]) << 16);
    const uint32_t rawTemperature = data[4] | (data[5] << 8) | (static_cast<uint32_t>(data[6]) << 16);

    const double temperature = bmp390::compensateTemperature(calibration_, rawTemperature);
    const double pressure = bmp390::compensatePressure(calibration_, rawPressure, temperature);
    if (!(pressure > 30000.0 && pressure < 125000.0)) {
        return false;  // outside the sensor's range: corrupt read
    }
    temperatureC_ = static_cast<float>(temperature);
    pressurePa_ = static_cast<float>(pressure);
    return true;
}

float Bmp390::altitudeM() const {
    return 44330.0f * (1.0f - powf(pressurePa_ / SEA_LEVEL_PA, 0.190295f));
}
