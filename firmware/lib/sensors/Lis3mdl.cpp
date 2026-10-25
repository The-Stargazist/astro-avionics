#include "Lis3mdl.h"

namespace {

constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t REG_CTRL_REG1 = 0x20;
constexpr uint8_t REG_CTRL_REG2 = 0x21;
constexpr uint8_t REG_CTRL_REG3 = 0x22;
constexpr uint8_t REG_CTRL_REG4 = 0x23;
constexpr uint8_t REG_CTRL_REG5 = 0x24;
constexpr uint8_t REG_OUT_X_L = 0x28;  // X, Y, Z, then TEMP_OUT at 0x2E
constexpr uint8_t AUTO_INCREMENT = 0x80;

constexpr uint8_t WHO_AM_I_LIS3MDL = 0x3D;

constexpr uint8_t CTRL1_TEMP_UHP_80HZ = 0xFC;  // TEMP_EN | OM=UHP | DO=80 Hz
constexpr uint8_t CTRL2_4_GAUSS = 0x00;
constexpr uint8_t CTRL3_CONTINUOUS = 0x00;
constexpr uint8_t CTRL4_Z_UHP = 0x0C;
constexpr uint8_t CTRL5_BDU = 0x40;            // registers update only after both bytes are read

constexpr float LSB_PER_GAUSS = 6842.0f;       // +/-4 gauss
constexpr float TEMP_LSB_PER_C = 8.0f;
constexpr float TEMP_OFFSET_C = 25.0f;

int16_t le16(const uint8_t* p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); }

}  // namespace

Lis3mdl::Lis3mdl(TwoWire& wire, uint8_t address) : wire_(wire), address_(address) {}

bool Lis3mdl::readRegisters(uint8_t reg, uint8_t* data, size_t length) {
    wire_.beginTransmission(address_);
    wire_.write(length > 1 ? (reg | AUTO_INCREMENT) : reg);
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

bool Lis3mdl::writeRegister(uint8_t reg, uint8_t value) {
    wire_.beginTransmission(address_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
}

bool Lis3mdl::begin() {
    configured_ = false;
    uint8_t id = 0;
    if (!readRegisters(REG_WHO_AM_I, &id, 1) || id != WHO_AM_I_LIS3MDL) {
        return false;
    }
    const bool written = writeRegister(REG_CTRL_REG1, CTRL1_TEMP_UHP_80HZ) &&
                         writeRegister(REG_CTRL_REG2, CTRL2_4_GAUSS) &&
                         writeRegister(REG_CTRL_REG4, CTRL4_Z_UHP) &&
                         writeRegister(REG_CTRL_REG5, CTRL5_BDU) &&
                         writeRegister(REG_CTRL_REG3, CTRL3_CONTINUOUS);  // start last
    if (!written) {
        return false;
    }
    delay(20);
    configured_ = true;
    if (!read()) {
        configured_ = false;
    }
    return configured_;
}

bool Lis3mdl::read() {
    uint8_t buf[8];  // OUT_X_L .. TEMP_OUT_H
    if (!configured_ || !readRegisters(REG_OUT_X_L, buf, sizeof(buf))) {
        return false;
    }
    field_.x = le16(&buf[0]) / LSB_PER_GAUSS;
    field_.y = le16(&buf[2]) / LSB_PER_GAUSS;
    field_.z = le16(&buf[4]) / LSB_PER_GAUSS;
    temperatureC_ = le16(&buf[6]) / TEMP_LSB_PER_C + TEMP_OFFSET_C;
    return true;
}
