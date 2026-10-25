#include <unity.h>

#include <stdio.h>
#include <string.h>

#include <string>

#include "NmeaParser.h"

namespace {

// "$<body>*<checksum>\r\n"
std::string sentence(const char* body) {
    unsigned checksum = 0;
    for (const char* p = body; *p != '\0'; ++p) {
        checksum ^= static_cast<unsigned char>(*p);
    }
    char tail[8];
    snprintf(tail, sizeof(tail), "*%02X\r\n", checksum);
    return std::string("$") + body + tail;
}

// Feeds text and returns how many sentences were accepted.
int feed(NmeaParser& parser, const std::string& text) {
    int accepted = 0;
    for (char c : text) {
        accepted += parser.feed(c) ? 1 : 0;
    }
    return accepted;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_gga_reference_sentence() {
    NmeaParser parser;
    TEST_ASSERT_EQUAL(1, feed(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"));
    const GpsFix& fix = parser.fix();
    TEST_ASSERT_TRUE(fix.valid);
    TEST_ASSERT_TRUE(fix.everValid);
    // ddmm.mmm must be converted to decimal degrees, not divided by 100.
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 48.0 + 7.038 / 60.0, fix.latitudeDeg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 11.0 + 31.0 / 60.0, fix.longitudeDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 545.4f, fix.altitudeM);
    TEST_ASSERT_EQUAL_UINT8(8, fix.satellites);
    TEST_ASSERT_EQUAL_UINT8(12, fix.hour);
    TEST_ASSERT_EQUAL_UINT8(35, fix.minute);
    TEST_ASSERT_EQUAL_UINT8(19, fix.second);
}

void test_rmc_reference_sentence() {
    NmeaParser parser;
    TEST_ASSERT_EQUAL(1, feed(parser, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"));
    const GpsFix& fix = parser.fix();
    TEST_ASSERT_TRUE(fix.valid);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 48.1173, fix.latitudeDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 22.4f * 0.514444f, fix.speedMps);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 84.4f, fix.courseDeg);
}

// u-blox M8N default output uses the GN (multi-constellation) talker.
void test_gn_talker_is_accepted() {
    NmeaParser parser;
    const std::string text =
        sentence("GNGGA,082155.00,2822.12345,N,07535.54321,E,1,11,0.85,301.2,M,-41.0,M,,");
    TEST_ASSERT_EQUAL(1, feed(parser, text));
    TEST_ASSERT_TRUE(parser.fix().valid);
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, 28.0 + 22.12345 / 60.0, parser.fix().latitudeDeg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, 75.0 + 35.54321 / 60.0, parser.fix().longitudeDeg);
    TEST_ASSERT_EQUAL_UINT8(11, parser.fix().satellites);
}

void test_southern_and_western_hemispheres_are_negative() {
    NmeaParser parser;
    feed(parser, sentence("GPGGA,000001,3351.000,S,15112.000,W,1,05,1.2,10.0,M,0.0,M,,"));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -(33.0 + 51.0 / 60.0), parser.fix().latitudeDeg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -(151.0 + 12.0 / 60.0), parser.fix().longitudeDeg);
}

// Without a fix most fields are empty. strtok() would merge the commas and
// read the wrong fields; the parser must keep field positions.
void test_empty_fields_keep_field_positions() {
    NmeaParser parser;
    feed(parser, sentence("GNGGA,082155.00,2822.12345,N,07535.54321,E,1,11,0.85,301.2,M,-41.0,M,,"));
    TEST_ASSERT_EQUAL(1, feed(parser, sentence("GNGGA,082156.00,,,,,0,00,99.99,,,,,,")));
    const GpsFix& fix = parser.fix();
    TEST_ASSERT_FALSE(fix.valid);
    TEST_ASSERT_TRUE(fix.everValid);
    TEST_ASSERT_EQUAL_UINT8(0, fix.satellites);
    TEST_ASSERT_EQUAL_UINT8(56, fix.second);
    // Last known position is kept for recovery.
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, 28.0 + 22.12345 / 60.0, fix.latitudeDeg);
}

void test_rmc_void_status_clears_validity() {
    NmeaParser parser;
    feed(parser, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    feed(parser, sentence("GPRMC,123520,V,,,,,,,230394,,,N"));
    TEST_ASSERT_FALSE(parser.fix().valid);
    TEST_ASSERT_TRUE(parser.fix().everValid);
}

void test_bad_or_missing_checksum_is_rejected() {
    NmeaParser parser;
    TEST_ASSERT_EQUAL(0, feed(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48\r\n"));
    TEST_ASSERT_EQUAL(0, feed(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,\r\n"));
    TEST_ASSERT_FALSE(parser.fix().valid);
    TEST_ASSERT_EQUAL_UINT32(2, parser.checksumErrors());
    TEST_ASSERT_EQUAL_UINT32(0, parser.sentences());
}

void test_other_sentences_and_garbage_are_ignored() {
    NmeaParser parser;
    std::string text = sentence("GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00");
    text += std::string(300, 'x') + "\r\n";                // noise with no '$'
    text += "$" + std::string(200, 'A') + "\r\n";          // overlong sentence
    text += "$GPGGA,123519,4807.0";                        // truncated by a new '$'
    text += "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n";
    TEST_ASSERT_EQUAL(1, feed(parser, text));
    TEST_ASSERT_TRUE(parser.fix().valid);
    TEST_ASSERT_EQUAL_UINT32(1, parser.sentences());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_gga_reference_sentence);
    RUN_TEST(test_rmc_reference_sentence);
    RUN_TEST(test_gn_talker_is_accepted);
    RUN_TEST(test_southern_and_western_hemispheres_are_negative);
    RUN_TEST(test_empty_fields_keep_field_positions);
    RUN_TEST(test_rmc_void_status_clears_validity);
    RUN_TEST(test_bad_or_missing_checksum_is_rejected);
    RUN_TEST(test_other_sentences_and_garbage_are_ignored);
    return UNITY_END();
}
