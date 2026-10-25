#pragma once

#include <stdint.h>

// BMP390 floating-point compensation, BMP390 datasheet section 8.4/8.5
// (same maths as Bosch's BMP3_FLOAT_COMPENSATION). Hardware-independent so
// it can be checked against the reference implementation on the host.
namespace bmp390 {

constexpr uint8_t CALIBRATION_LENGTH = 21;  // NVM_PAR_T1 (0x31) .. NVM_PAR_P11 (0x45)

struct Calibration {
    double t1, t2, t3;
    double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};

inline Calibration parseCalibration(const uint8_t raw[CALIBRATION_LENGTH]) {
    auto u16 = [&](int i) { return static_cast<uint16_t>(raw[i] | (raw[i + 1] << 8)); };
    auto s16 = [&](int i) { return static_cast<int16_t>(u16(i)); };
    auto s8 = [&](int i) { return static_cast<int8_t>(raw[i]); };

    Calibration c;
    c.t1 = u16(0) / 0.00390625;                  // / 2^-8
    c.t2 = u16(2) / 1073741824.0;                // / 2^30
    c.t3 = s8(4) / 281474976710656.0;            // / 2^48
    c.p1 = (s16(5) - 16384) / 1048576.0;         // (x - 2^14) / 2^20
    c.p2 = (s16(7) - 16384) / 536870912.0;       // (x - 2^14) / 2^29
    c.p3 = s8(9) / 4294967296.0;                 // / 2^32
    c.p4 = s8(10) / 137438953472.0;              // / 2^37
    c.p5 = u16(11) / 0.125;                      // / 2^-3
    c.p6 = u16(13) / 64.0;                       // / 2^6
    c.p7 = s8(15) / 256.0;                       // / 2^8
    c.p8 = s8(16) / 32768.0;                     // / 2^15
    c.p9 = s16(17) / 281474976710656.0;          // / 2^48
    c.p10 = s8(19) / 281474976710656.0;          // / 2^48
    c.p11 = s8(20) / 36893488147419103232.0;     // / 2^65
    return c;
}

// Linearised temperature in degC. Pressure compensation needs this value.
inline double compensateTemperature(const Calibration& c, uint32_t rawTemperature) {
    const double partial1 = static_cast<double>(rawTemperature) - c.t1;
    const double partial2 = partial1 * c.t2;
    return partial2 + partial1 * partial1 * c.t3;
}

// Pressure in Pa.
inline double compensatePressure(const Calibration& c, uint32_t rawPressure, double temperature) {
    const double t = temperature;
    const double p = static_cast<double>(rawPressure);
    const double out1 = c.p5 + c.p6 * t + c.p7 * t * t + c.p8 * t * t * t;
    const double out2 = p * (c.p1 + c.p2 * t + c.p3 * t * t + c.p4 * t * t * t);
    const double out3 = p * p * (c.p9 + c.p10 * t) + p * p * p * c.p11;
    return out1 + out2 + out3;
}

}  // namespace bmp390
