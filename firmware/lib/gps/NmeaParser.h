#pragma once

#include <stddef.h>
#include <stdint.h>

struct GpsFix {
    bool valid = false;         // receiver currently reports a position fix
    bool everValid = false;     // a fix was seen at least once (lat/lon are meaningful)
    double latitudeDeg = 0.0;   // +north; last known value is kept when the fix drops
    double longitudeDeg = 0.0;  // +east
    float altitudeM = 0.0f;     // above mean sea level (GGA)
    float speedMps = 0.0f;      // over ground (RMC)
    float courseDeg = 0.0f;     // true course over ground (RMC)
    uint8_t satellites = 0;     // used in the fix (GGA)
    uint8_t hour = 0;           // UTC
    uint8_t minute = 0;
    uint8_t second = 0;
};

// Incremental NMEA 0183 parser for GGA and RMC sentences from any talker
// (GP, GN, GL, GA, ...; a u-blox M8N defaults to GN). Sentences without a
// valid "*hh" checksum are dropped. Hardware-independent.
class NmeaParser {
public:
    // Feed one received character. Returns true when it completed a valid
    // GGA or RMC sentence (fix() was updated).
    bool feed(char c);

    const GpsFix& fix() const { return fix_; }
    uint32_t sentences() const { return sentences_; }
    uint32_t checksumErrors() const { return checksumErrors_; }

private:
    static constexpr size_t MAX_SENTENCE = 100;  // NMEA allows 82 characters
    static constexpr size_t MAX_FIELDS = 24;

    bool processSentence();
    void parseGga(char* const* field, size_t count);
    void parseRmc(char* const* field, size_t count);

    char buffer_[MAX_SENTENCE + 1] = {};
    size_t length_ = 0;
    bool inSentence_ = false;
    GpsFix fix_;
    uint32_t sentences_ = 0;
    uint32_t checksumErrors_ = 0;
};
