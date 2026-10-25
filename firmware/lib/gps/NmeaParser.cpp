#include "NmeaParser.h"

#include <stdlib.h>
#include <string.h>

namespace {

constexpr float KNOTS_TO_MPS = 0.514444f;

int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// "ddmm.mmmm" / "dddmm.mmmm" plus hemisphere -> signed decimal degrees.
bool parseCoordinate(const char* value, const char* hemisphere, double* out) {
    if (value[0] == '\0' || hemisphere[0] == '\0') {
        return false;
    }
    char* end = nullptr;
    const double raw = strtod(value, &end);
    if (end == value) {
        return false;
    }
    const double degrees = static_cast<double>(static_cast<long>(raw / 100.0));
    double decimal = degrees + (raw - degrees * 100.0) / 60.0;
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
        decimal = -decimal;
    }
    *out = decimal;
    return true;
}

// "hhmmss.ss" -> UTC time of day.
void parseTime(const char* value, GpsFix* fix) {
    for (int i = 0; i < 6; ++i) {
        if (value[i] < '0' || value[i] > '9') {
            return;
        }
    }
    fix->hour = static_cast<uint8_t>((value[0] - '0') * 10 + (value[1] - '0'));
    fix->minute = static_cast<uint8_t>((value[2] - '0') * 10 + (value[3] - '0'));
    fix->second = static_cast<uint8_t>((value[4] - '0') * 10 + (value[5] - '0'));
}

}  // namespace

bool NmeaParser::feed(char c) {
    if (c == '$') {
        inSentence_ = true;
        length_ = 0;
        return false;
    }
    if (!inSentence_) {
        return false;
    }
    if (c == '\r' || c == '\n') {
        inSentence_ = false;
        buffer_[length_] = '\0';
        return processSentence();
    }
    if (length_ >= MAX_SENTENCE) {
        inSentence_ = false;  // overlong: garbage, wait for the next '$'
        return false;
    }
    buffer_[length_++] = c;
    return false;
}

bool NmeaParser::processSentence() {
    // buffer_ holds everything between '$' and the line ending: "body*hh".
    char* star = strrchr(buffer_, '*');
    if (star == nullptr || star[1] == '\0' || star[2] == '\0') {
        ++checksumErrors_;
        return false;
    }
    const int high = hexValue(star[1]);
    const int low = hexValue(star[2]);
    uint8_t checksum = 0;
    for (const char* p = buffer_; p < star; ++p) {
        checksum ^= static_cast<uint8_t>(*p);
    }
    if (high < 0 || low < 0 || checksum != ((high << 4) | low)) {
        ++checksumErrors_;
        return false;
    }
    *star = '\0';

    // Split in place on ',' keeping empty fields: strtok() would merge
    // consecutive commas and shift every field after an empty one.
    char* field[MAX_FIELDS];
    size_t count = 0;
    field[count++] = buffer_;
    for (char* p = buffer_; *p != '\0' && count < MAX_FIELDS; ++p) {
        if (*p == ',') {
            *p = '\0';
            field[count++] = p + 1;
        }
    }

    // Sentence id is a 2-letter talker followed by the type, e.g. "GNGGA".
    const char* id = field[0];
    if (strlen(id) != 5) {
        return false;
    }
    if (strcmp(id + 2, "GGA") == 0) {
        parseGga(field, count);
    } else if (strcmp(id + 2, "RMC") == 0) {
        parseRmc(field, count);
    } else {
        return false;
    }
    ++sentences_;
    return true;
}

// $xxGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,geoid,M,age,station
void NmeaParser::parseGga(char* const* field, size_t count) {
    if (count < 10) {
        return;
    }
    parseTime(field[1], &fix_);
    fix_.valid = atoi(field[6]) > 0;
    fix_.satellites = static_cast<uint8_t>(atoi(field[7]));
    if (!fix_.valid) {
        return;
    }
    double lat = 0.0;
    double lon = 0.0;
    if (parseCoordinate(field[2], field[3], &lat) && parseCoordinate(field[4], field[5], &lon)) {
        fix_.latitudeDeg = lat;
        fix_.longitudeDeg = lon;
        fix_.everValid = true;
    }
    if (field[9][0] != '\0') {
        fix_.altitudeM = static_cast<float>(atof(field[9]));
    }
}

// $xxRMC,time,status,lat,N,lon,E,speed_knots,course,date,magvar,E[,mode]
void NmeaParser::parseRmc(char* const* field, size_t count) {
    if (count < 9) {
        return;
    }
    parseTime(field[1], &fix_);
    fix_.valid = field[2][0] == 'A';
    if (!fix_.valid) {
        return;
    }
    double lat = 0.0;
    double lon = 0.0;
    if (parseCoordinate(field[3], field[4], &lat) && parseCoordinate(field[5], field[6], &lon)) {
        fix_.latitudeDeg = lat;
        fix_.longitudeDeg = lon;
        fix_.everValid = true;
    }
    if (field[7][0] != '\0') {
        fix_.speedMps = static_cast<float>(atof(field[7])) * KNOTS_TO_MPS;
    }
    if (field[8][0] != '\0') {
        fix_.courseDeg = static_cast<float>(atof(field[8]));
    }
}
