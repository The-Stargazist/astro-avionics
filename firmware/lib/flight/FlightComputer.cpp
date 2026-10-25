#include "FlightComputer.h"

#include <math.h>

const char* toString(FlightPhase phase) {
    switch (phase) {
        case FlightPhase::Ground: return "GROUND";
        case FlightPhase::Flight: return "FLIGHT";
        case FlightPhase::Descent: return "DESCENT";
        case FlightPhase::Landed: return "LANDED";
    }
    return "?";
}

const char* toString(FlightEvent event) {
    switch (event) {
        case FlightEvent::None: return "NONE";
        case FlightEvent::Launch: return "LAUNCH";
        case FlightEvent::Apogee: return "APOGEE";
        case FlightEvent::Overspeed: return "OVERSPEED";
        case FlightEvent::Landed: return "LANDED";
    }
    return "?";
}

FlightEvent FlightComputer::update(uint32_t nowMs, float altitudeAglM, float velocityMps) {
    // NaN inputs make every comparison below false, which resets the
    // debouncers instead of confirming anything.
    if (altitudeAglM > maxAltitudeM_) {
        maxAltitudeM_ = altitudeAglM;
    }

    switch (phase_) {
        case FlightPhase::Ground:
            if (launch_.update(altitudeAglM > config_.launchAltitudeM, nowMs,
                               config_.launchConfirmMs)) {
                phase_ = FlightPhase::Flight;
                return FlightEvent::Launch;
            }
            break;

        case FlightPhase::Flight:
            if (apogee_.update(velocityMps < config_.apogeeVelocityMps, nowMs,
                               config_.apogeeConfirmMs)) {
                phase_ = FlightPhase::Descent;
                apogeeMs_ = nowMs;
                return FlightEvent::Apogee;
            }
            break;

        case FlightPhase::Descent: {
            const bool lockedOut = nowMs - apogeeMs_ < config_.deployLockoutMs;
            const bool tooFast = !lockedOut && velocityMps < -config_.overspeedMps;
            if (overspeed_.update(tooFast, nowMs, config_.overspeedConfirmMs) &&
                !overspeedReported_) {
                overspeedReported_ = true;
                return FlightEvent::Overspeed;
            }
            if (landed_.update(fabsf(velocityMps) < config_.landedVelocityMps, nowMs,
                               config_.landedConfirmMs)) {
                phase_ = FlightPhase::Landed;
                return FlightEvent::Landed;
            }
            break;
        }

        case FlightPhase::Landed:
            break;
    }
    return FlightEvent::None;
}
