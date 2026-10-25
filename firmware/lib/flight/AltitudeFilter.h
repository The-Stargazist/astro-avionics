#pragma once

#include <math.h>

// Alpha-beta (g-h) filter for barometric altitude.
//
// Differentiating raw baro altitude at 50 Hz turns ~0.1 m of sensor noise
// into several m/s of velocity noise, enough to trip apogee or landing
// detection on a single bad sample. This filter tracks altitude and vertical
// velocity with a constant-velocity model and fixed gains.
//
// With the default gains at a 20 ms sample period:
//   - velocity noise is ~1.7x the altitude noise (~0.15 m/s for a BMP390 at
//     8x oversampling),
//   - the velocity estimate lags a constant acceleration a by ~0.19 s * a,
//     i.e. it reads ~2 m/s high during coast, so apogee is reported ~0.2 s
//     late rather than early.
class AltitudeFilter {
public:
    explicit AltitudeFilter(float alpha = 0.2f, float beta = 0.02f)
        : alpha_(alpha), beta_(beta) {}

    void reset(float altitude, float velocity = 0.0f) {
        altitude_ = altitude;
        velocity_ = velocity;
    }

    // Altitude the model expects dtSeconds after the last update.
    float predict(float dtSeconds) const { return altitude_ + velocity_ * dtSeconds; }

    // dtSeconds is the time since the previous measurement.
    void update(float measuredAltitude, float dtSeconds) {
        if (!isfinite(measuredAltitude) || !(dtSeconds > 0.0f)) {
            return;
        }
        const float predicted = altitude_ + velocity_ * dtSeconds;
        const float residual = measuredAltitude - predicted;
        altitude_ = predicted + alpha_ * residual;
        velocity_ += (beta_ / dtSeconds) * residual;
    }

    float altitude() const { return altitude_; }
    float velocity() const { return velocity_; }

private:
    float alpha_;
    float beta_;
    float altitude_ = 0.0f;
    float velocity_ = 0.0f;
};
