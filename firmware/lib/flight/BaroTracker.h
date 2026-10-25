#pragma once

#include <stdint.h>

#include "AltitudeFilter.h"
#include "FlightComputer.h"

// Turns absolute barometric altitude samples into AGL altitude, vertical
// velocity and flight phase.
//
// - The pad reference is the median of the first GROUND_SAMPLES samples, so
//   one bad reading cannot shift it. The flight computer only runs after
//   that, so a sensor that comes up late still gets a reference.
// - A sample further than OUTLIER_GATE_M from the filter's prediction is
//   dropped: a corrupted read that still decodes to a plausible pressure can
//   be off by a kilometre, and fed to the filter it would fake a velocity of
//   hundreds of m/s. After MAX_CONSECUTIVE_OUTLIERS in a row the filter
//   re-acquires on the measurement (keeping its velocity) instead of
//   ignoring the sensor forever.
class BaroTracker {
public:
    static constexpr uint16_t GROUND_SAMPLES = 50;  // 1 s at 50 Hz
    static constexpr float OUTLIER_GATE_M = 50.0f;  // real motion is < 10 m per 20 ms sample
    static constexpr uint8_t MAX_CONSECUTIVE_OUTLIERS = 5;

    explicit BaroTracker(const FlightConfig& config = FlightConfig()) : flight_(config) {}

    // Feed one absolute altitude sample (metres, any fixed datum).
    FlightEvent update(uint32_t nowMs, float altitudeM);

    bool groundReady() const { return groundCount_ >= GROUND_SAMPLES; }
    float groundAltitudeM() const { return groundAltitudeM_; }

    float rawAltitudeAglM() const { return rawAglM_; }
    float altitudeAglM() const { return filter_.altitude(); }
    float velocityMps() const { return filter_.velocity(); }
    float maxAltitudeAglM() const { return flight_.maxAltitudeM(); }
    FlightPhase phase() const { return flight_.phase(); }
    uint32_t outliers() const { return outliers_; }

private:
    AltitudeFilter filter_;
    FlightComputer flight_;
    float groundSamples_[GROUND_SAMPLES] = {};
    uint16_t groundCount_ = 0;
    float groundAltitudeM_ = 0.0f;
    float rawAglM_ = 0.0f;
    uint32_t lastMs_ = 0;
    uint8_t consecutiveOutliers_ = 0;
    uint32_t outliers_ = 0;
};
