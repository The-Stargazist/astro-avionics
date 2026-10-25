#include "BaroTracker.h"

#include <math.h>

#include <algorithm>

FlightEvent BaroTracker::update(uint32_t nowMs, float altitudeM) {
    if (!isfinite(altitudeM)) {
        return FlightEvent::None;
    }

    if (!groundReady()) {
        groundSamples_[groundCount_++] = altitudeM;
        if (groundReady()) {
            float* middle = groundSamples_ + GROUND_SAMPLES / 2;
            std::nth_element(groundSamples_, middle, groundSamples_ + GROUND_SAMPLES);
            groundAltitudeM_ = *middle;
            rawAglM_ = altitudeM - groundAltitudeM_;
            filter_.reset(rawAglM_);
            lastMs_ = nowMs;
        }
        return FlightEvent::None;
    }

    rawAglM_ = altitudeM - groundAltitudeM_;  // logged even when rejected below
    const float dtSeconds = static_cast<float>(nowMs - lastMs_) * 1e-3f;
    if (fabsf(rawAglM_ - filter_.predict(dtSeconds)) > OUTLIER_GATE_M) {
        ++outliers_;
        if (++consecutiveOutliers_ <= MAX_CONSECUTIVE_OUTLIERS) {
            return FlightEvent::None;  // lastMs_ unchanged: the next dt spans the gap
        }
        filter_.reset(rawAglM_, filter_.velocity());
    } else {
        filter_.update(rawAglM_, dtSeconds);
    }
    consecutiveOutliers_ = 0;
    lastMs_ = nowMs;
    return flight_.update(nowMs, filter_.altitude(), filter_.velocity());
}
