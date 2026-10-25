#pragma once

#include <stdint.h>

// Flight phase detection from filtered AGL altitude and vertical velocity.
// Pure logic: no hardware access, so it runs unchanged in host unit tests.
//
//   Ground --(AGL > launchAltitude)--> Flight
//   Flight --(velocity < apogeeVelocity)--> Descent   [event Apogee: fire main]
//   Descent --(velocity < -overspeed)--> (stays)       [event Overspeed: fire backup, once]
//   Descent --(|velocity| < landedVelocity)--> Landed
//
// Every transition requires its condition to hold continuously for a confirm
// time, so one noisy sample cannot trigger it.

enum class FlightPhase : uint8_t {
    Ground = 0,   // on the pad, waiting for launch
    Flight = 1,   // launch detected, waiting for apogee
    Descent = 2,  // past apogee, main deployed
    Landed = 3,
};

enum class FlightEvent : uint8_t {
    None,
    Launch,
    Apogee,
    Overspeed,
    Landed,
};

const char* toString(FlightPhase phase);
const char* toString(FlightEvent event);

struct FlightConfig {
    // Launch: AGL altitude above this (mission requirement: 200 m).
    float launchAltitudeM = 200.0f;
    uint32_t launchConfirmMs = 100;

    // Apogee: velocity below this. Slightly positive because the filter
    // velocity lags low-going during coast.
    float apogeeVelocityMps = 2.0f;
    uint32_t apogeeConfirmMs = 100;

    // Overspeed: descending faster than this (a positive speed).
    float overspeedMps = 20.0f;
    uint32_t overspeedConfirmMs = 200;
    // Ignore overspeed for this long after apogee: the main ejection charge
    // pressurises the avionics bay and corrupts baro readings for a moment.
    uint32_t deployLockoutMs = 2000;

    // Landed: |velocity| below this for this long.
    float landedVelocityMps = 2.0f;
    uint32_t landedConfirmMs = 20000;
};

class FlightComputer {
public:
    explicit FlightComputer(const FlightConfig& config = FlightConfig())
        : config_(config) {}

    // Feed one filtered sample. altitudeAglM is metres above the pad,
    // velocityMps is positive upwards. Returns the event this sample caused.
    FlightEvent update(uint32_t nowMs, float altitudeAglM, float velocityMps);

    FlightPhase phase() const { return phase_; }
    float maxAltitudeM() const { return maxAltitudeM_; }
    const FlightConfig& config() const { return config_; }

private:
    // True once `condition` has held on every sample for at least holdMs.
    class Debounce {
    public:
        bool update(bool condition, uint32_t nowMs, uint32_t holdMs) {
            if (!condition) {
                active_ = false;
                return false;
            }
            if (!active_) {
                active_ = true;
                sinceMs_ = nowMs;
            }
            return nowMs - sinceMs_ >= holdMs;
        }

    private:
        bool active_ = false;
        uint32_t sinceMs_ = 0;
    };

    FlightConfig config_;
    FlightPhase phase_ = FlightPhase::Ground;
    Debounce launch_;
    Debounce apogee_;
    Debounce overspeed_;
    Debounce landed_;
    uint32_t apogeeMs_ = 0;
    bool overspeedReported_ = false;
    float maxAltitudeM_ = 0.0f;
};
