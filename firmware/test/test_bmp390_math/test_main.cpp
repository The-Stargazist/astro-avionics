#include <unity.h>

// Header-only and hardware-independent; included by path so the native
// build does not pull in the Arduino drivers of the `sensors` library.
#include "../../lib/sensors/Bmp390Compensation.h"

namespace {

// Expected values produced by Bosch's reference implementation (bmp3.c,
// BMP3_DOUBLE_PRECISION_COMPENSATION) for the same calibration bytes and
// raw samples. A 20k-case randomized comparison over the sensor's physical
// range agreed to within 0.003 Pa.
struct Vector {
    uint8_t calibration[bmp390::CALIBRATION_LENGTH];
    uint32_t rawPressure;
    uint32_t rawTemperature;
    double temperatureC;
    double pressurePa;
};

const Vector VECTORS[] = {
    {{0xFD, 0xED, 0x84, 0xB3, 0xC6, 0x1F, 0xFC, 0xEF, 0xD8, 0xE1, 0xEC, 0x2D, 0x53, 0xB7, 0xB3,
      0xE9, 0x9B, 0xDE, 0xF7, 0x5B, 0x29},
     0x4F6C21, 0xF49DA0, 18.55064783, 92726.72245},
    {{0x58, 0x99, 0xEC, 0x0F, 0x0D, 0x8B, 0x78, 0x96, 0x62, 0xA0, 0x0E, 0xFD, 0x1A, 0x66, 0xF1,
      0x4C, 0x53, 0x7C, 0x4B, 0x0C, 0x40},
     0x0D65CA, 0xD06035, 14.29158848, 81518.32154},
    {{0x14, 0x08, 0xCD, 0x32, 0x1E, 0x3E, 0x2B, 0x62, 0xB6, 0x6F, 0x92, 0xE8, 0x40, 0x8D, 0x3D,
      0x3C, 0xCF, 0xB6, 0x79, 0x0B, 0x2A},
     0x6B1BD5, 0x301A9E, 32.50437453, 96490.50866},
};

}  // namespace

void setUp() {}
void tearDown() {}

void test_matches_bosch_reference() {
    for (const Vector& v : VECTORS) {
        const bmp390::Calibration c = bmp390::parseCalibration(v.calibration);
        const double t = bmp390::compensateTemperature(c, v.rawTemperature);
        const double p = bmp390::compensatePressure(c, v.rawPressure, t);
        TEST_ASSERT_DOUBLE_WITHIN(1e-6, v.temperatureC, t);
        TEST_ASSERT_DOUBLE_WITHIN(0.01, v.pressurePa, p);
    }
}

void test_calibration_byte_order_and_signs() {
    uint8_t raw[bmp390::CALIBRATION_LENGTH] = {};
    raw[0] = 0x34;  // T1 = 0x1234 (unsigned, little-endian)
    raw[1] = 0x12;
    raw[4] = 0xFF;  // T3 = -1 (signed byte)
    raw[5] = 0x00;  // P1 = 0x4000 -> (x - 2^14) = 0
    raw[6] = 0x40;
    const bmp390::Calibration c = bmp390::parseCalibration(raw);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0x1234 * 256.0, c.t1);
    TEST_ASSERT_TRUE(c.t3 < 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, c.p1);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_matches_bosch_reference);
    RUN_TEST(test_calibration_byte_order_and_signs);
    return UNITY_END();
}
