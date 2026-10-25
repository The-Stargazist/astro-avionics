#include "Icm20649.h"

namespace {

constexpr uint8_t REG_BANK_SEL = 0x7F;  // present in every bank

// Bank 0
constexpr uint8_t REG_WHO_AM_I = 0x00;
constexpr uint8_t REG_PWR_MGMT_1 = 0x06;
constexpr uint8_t REG_PWR_MGMT_2 = 0x07;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x2D;  // accel(6), temp(2), gyro(6)

// Bank 2
constexpr uint8_t REG_GYRO_SMPLRT_DIV = 0x00;
constexpr uint8_t REG_GYRO_CONFIG_1 = 0x01;
constexpr uint8_t REG_ACCEL_SMPLRT_DIV_1 = 0x10;
constexpr uint8_t REG_ACCEL_SMPLRT_DIV_2 = 0x11;
constexpr uint8_t REG_ACCEL_CONFIG = 0x14;

constexpr uint8_t WHO_AM_I_ICM20649 = 0xE1;
constexpr uint8_t PWR_RESET = 0x80;
constexpr uint8_t PWR_CLK_AUTO = 0x01;

// CONFIG layout: DLPFCFG[5:3] | FS_SEL[2:1] | FCHOICE[0]
constexpr uint8_t GYRO_CONFIG = (3 << 3) | (2 << 1) | 1;   // DLPF ~51 Hz, +/-2000 dps
constexpr uint8_t ACCEL_CONFIG = (3 << 3) | (3 << 1) | 1;  // DLPF ~50 Hz, +/-30 g

constexpr float GRAVITY = 9.80665f;
constexpr float ACCEL_LSB_PER_G = 1024.0f;   // +/-30 g
constexpr float GYRO_LSB_PER_DPS = 16.4f;    // +/-2000 dps
constexpr float TEMP_LSB_PER_C = 333.87f;
constexpr float TEMP_OFFSET_C = 21.0f;

int16_t be16(const uint8_t* p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }

}  // namespace

Icm20649::Icm20649(TwoWire& wire, uint8_t address) : wire_(wire), address_(address) {}

bool Icm20649::readRegisters(uint8_t reg, uint8_t* data, size_t length) {
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

bool Icm20649::writeRegister(uint8_t reg, uint8_t value) {
    wire_.beginTransmission(address_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
}

bool Icm20649::selectBank(uint8_t bank) {
    return writeRegister(REG_BANK_SEL, static_cast<uint8_t>(bank << 4));
}

bool Icm20649::begin() {
    configured_ = false;

    uint8_t id = 0;
    if (!selectBank(0) || !readRegisters(REG_WHO_AM_I, &id, 1) || id != WHO_AM_I_ICM20649) {
        return false;
    }
    if (!writeRegister(REG_PWR_MGMT_1, PWR_RESET)) {
        return false;
    }
    delay(100);
    // The reset also returns the bank select to bank 0.
    if (!writeRegister(REG_PWR_MGMT_1, PWR_CLK_AUTO) || !writeRegister(REG_PWR_MGMT_2, 0x00)) {
        return false;
    }
    delay(10);

    const bool written = selectBank(2) &&
                         writeRegister(REG_GYRO_SMPLRT_DIV, 0) &&      // 1.1 kHz
                         writeRegister(REG_GYRO_CONFIG_1, GYRO_CONFIG) &&
                         writeRegister(REG_ACCEL_SMPLRT_DIV_1, 0) &&   // 1.125 kHz
                         writeRegister(REG_ACCEL_SMPLRT_DIV_2, 0) &&
                         writeRegister(REG_ACCEL_CONFIG, ACCEL_CONFIG);
    if (!selectBank(0) || !written) {
        return false;
    }
    delay(20);  // let the filters settle
    configured_ = true;
    if (!read()) {
        configured_ = false;
    }
    return configured_;
}

bool Icm20649::read() {
    uint8_t buf[14];
    if (!configured_ || !readRegisters(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
        return false;
    }
    constexpr float accelScale = GRAVITY / ACCEL_LSB_PER_G;
    constexpr float gyroScale = 1.0f / GYRO_LSB_PER_DPS;

    accel_.x = be16(&buf[0]) * accelScale;
    accel_.y = be16(&buf[2]) * accelScale;
    accel_.z = be16(&buf[4]) * accelScale;
    temperatureC_ = be16(&buf[6]) / TEMP_LSB_PER_C + TEMP_OFFSET_C;
    gyro_.x = be16(&buf[8]) * gyroScale;
    gyro_.y = be16(&buf[10]) * gyroScale;
    gyro_.z = be16(&buf[12]) * gyroScale;
    return true;
}
