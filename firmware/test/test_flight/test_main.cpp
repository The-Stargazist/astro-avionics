#include <unity.h>

#include <math.h>

#include <random>
#include <vector>

#include "AltitudeFilter.h"
#include "BaroTracker.h"
#include "FlightComputer.h"

namespace {

constexpr float PAD_ALTITUDE_M = 300.0f;  // datum offset, removed by the ground reference
constexpr uint32_t BOOT_MS = 10000;       // tracker sees millis() starting here
constexpr uint32_t SAMPLE_MS = 20;        // BMP390 at 50 Hz
constexpr float GRAVITY = 9.81f;

struct Event {
    float timeS;  // since liftoff
    FlightEvent event;
};

struct Glitch {
    float timeS;
    float offsetM;
};

struct Flight {
    std::vector<Event> events;
    float apogeeTimeS = 0.0f;
    float apogeeAltitudeM = 0.0f;
    float touchdownTimeS = -1.0f;
};

// Point-mass flight: 5 s on the pad, 2.5 s boost at 70 m/s^2, coast with
// drag, then either a working main (descent settles at -6 m/s) or a failed
// main (free fall towards a -45 m/s terminal velocity). Baro noise is
// Gaussian; glitches add one-sample offsets.
Flight fly(bool mainWorks, float noiseM, uint32_t seed, const std::vector<Glitch>& glitches = {},
           float durationS = 300.0f) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.0f, noiseM);
    BaroTracker tracker;
    Flight result;

    double h = 0.0;
    double v = 0.0;
    bool pastApogee = false;
    const double step = 0.001;
    const float padS = 5.0f;
    uint32_t nextSampleMs = BOOT_MS;

    for (double t = -padS; t < durationS; t += step) {
        double a = 0.0;
        if (t >= 0.0) {
            if (t < 2.5) {
                a = 70.0;
            } else if (!pastApogee) {
                a = -GRAVITY - 0.0005 * v * fabs(v);
            } else if (mainWorks) {
                a = v > -6.0 ? -GRAVITY : 0.0;
                if (v < -6.0) v = -6.0;
            } else {
                a = -GRAVITY * (1.0 - (v * v) / (45.0 * 45.0));
            }
        }
        v += a * step;
        h += v * step;
        if (t > 2.5 && !pastApogee && v <= 0.0) {
            pastApogee = true;
            result.apogeeTimeS = static_cast<float>(t);
            result.apogeeAltitudeM = static_cast<float>(h);
        }
        if (h <= 0.0 && t > 1.0) {
            if (result.touchdownTimeS < 0.0f) result.touchdownTimeS = static_cast<float>(t);
            h = 0.0;
            v = 0.0;
        }

        const uint32_t nowMs = BOOT_MS + static_cast<uint32_t>(lround((t + padS) * 1000.0));
        if (nowMs < nextSampleMs) {
            continue;
        }
        nextSampleMs += SAMPLE_MS;
        float measured = PAD_ALTITUDE_M + static_cast<float>(h) + noise(rng);
        for (const Glitch& g : glitches) {
            if (fabs(t - g.timeS) < 0.5 * SAMPLE_MS / 1000.0) measured += g.offsetM;
        }
        const FlightEvent event = tracker.update(nowMs, measured);
        if (event != FlightEvent::None) {
            result.events.push_back({static_cast<float>(t), event});
        }
    }
    return result;
}

int count(const Flight& f, FlightEvent e) {
    int n = 0;
    for (const Event& x : f.events) n += x.event == e ? 1 : 0;
    return n;
}

float timeOf(const Flight& f, FlightEvent e) {
    for (const Event& x : f.events) {
        if (x.event == e) return x.timeS;
    }
    return NAN;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_filter_holds_constant_altitude() {
    std::mt19937 rng(1);
    std::normal_distribution<float> noise(0.0f, 0.2f);
    AltitudeFilter filter;
    filter.reset(0.0f);
    float worst = 0.0f;
    for (int i = 0; i < 3000; ++i) {
        filter.update(100.0f + noise(rng), 0.02f);
        if (i > 500) worst = fmaxf(worst, fabsf(filter.velocity()));
    }
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.0f, filter.altitude());
    TEST_ASSERT_TRUE_MESSAGE(worst < 1.5f, "velocity noise must stay far below the 2 m/s thresholds");
}

void test_filter_tracks_constant_velocity() {
    AltitudeFilter filter;
    filter.reset(0.0f);
    for (int i = 1; i <= 500; ++i) filter.update(50.0f * i * 0.02f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, filter.velocity());
}

void test_filter_ignores_bad_input() {
    AltitudeFilter filter;
    filter.reset(10.0f, 1.0f);
    filter.update(NAN, 0.02f);
    filter.update(20.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, filter.altitude());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, filter.velocity());
}

void test_nominal_flight_sequence() {
    const Flight f = fly(true, 0.15f, 42);
    TEST_ASSERT_EQUAL_INT(1, count(f, FlightEvent::Launch));
    TEST_ASSERT_EQUAL_INT(1, count(f, FlightEvent::Apogee));
    TEST_ASSERT_EQUAL_INT(0, count(f, FlightEvent::Overspeed));
    TEST_ASSERT_EQUAL_INT(1, count(f, FlightEvent::Landed));
    TEST_ASSERT_EQUAL(FlightEvent::Launch, f.events[0].event);
    TEST_ASSERT_EQUAL(FlightEvent::Apogee, f.events[1].event);
    TEST_ASSERT_EQUAL(FlightEvent::Landed, f.events[2].event);
}

void test_apogee_is_detected_near_true_apogee() {
    for (uint32_t seed = 1; seed <= 20; ++seed) {
        const Flight f = fly(true, 0.15f, seed);
        const float error = timeOf(f, FlightEvent::Apogee) - f.apogeeTimeS;
        // Never meaningfully early (rocket still climbing fast), at most ~1 s late.
        TEST_ASSERT_TRUE_MESSAGE(error > -0.4f, "apogee detected too early");
        TEST_ASSERT_TRUE_MESSAGE(error < 1.0f, "apogee detected too late");
    }
}

void test_landed_only_after_touchdown() {
    const Flight f = fly(true, 0.15f, 7);
    TEST_ASSERT_TRUE(f.touchdownTimeS > 0.0f);
    const float landed = timeOf(f, FlightEvent::Landed);
    TEST_ASSERT_TRUE_MESSAGE(landed > f.touchdownTimeS, "landed reported while still descending");
    TEST_ASSERT_TRUE(landed < f.touchdownTimeS + 25.0f);
}

void test_failed_main_triggers_backup_once_after_lockout() {
    const Flight f = fly(false, 0.15f, 3);
    TEST_ASSERT_EQUAL_INT(1, count(f, FlightEvent::Overspeed));
    const float apogee = timeOf(f, FlightEvent::Apogee);
    const float overspeed = timeOf(f, FlightEvent::Overspeed);
    const float lockoutS = FlightConfig().deployLockoutMs / 1000.0f;
    TEST_ASSERT_TRUE(overspeed >= apogee + lockoutS);
    TEST_ASSERT_TRUE_MESSAGE(overspeed < apogee + 6.0f, "backup should fire within seconds");
}

void test_pad_noise_and_glitches_do_not_launch() {
    std::vector<Glitch> glitches;
    for (float t = -4.0f; t < 0.0f; t += 0.5f) glitches.push_back({t, (t < -2.0f) ? 1500.0f : -800.0f});
    // Stay on the pad: the simulated motor never lights within the window.
    const Flight f = fly(true, 0.5f, 11, glitches, -0.001f);
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(f.events.size()));
}

// A single corrupted sample during fast coast must not fake an apogee.
void test_coast_glitch_does_not_trigger_early_apogee() {
    const std::vector<Glitch> glitches = {{6.0f, -1200.0f}, {7.0f, -60.0f}, {8.0f, 900.0f}};
    const Flight f = fly(true, 0.15f, 5, glitches);
    TEST_ASSERT_EQUAL_INT(1, count(f, FlightEvent::Apogee));
    TEST_ASSERT_TRUE(timeOf(f, FlightEvent::Apogee) > f.apogeeTimeS - 0.4f);
}

void test_ground_reference_ignores_glitch_during_calibration() {
    BaroTracker tracker;
    for (uint16_t i = 0; i < BaroTracker::GROUND_SAMPLES; ++i) {
        tracker.update(BOOT_MS + i * SAMPLE_MS, i == 10 ? 5000.0f : PAD_ALTITUDE_M);
    }
    TEST_ASSERT_TRUE(tracker.groundReady());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, PAD_ALTITUDE_M, tracker.groundAltitudeM());
}

void test_launch_needs_sustained_altitude() {
    FlightComputer fc;
    uint32_t t = 0;
    for (int i = 0; i < 4; ++i, t += SAMPLE_MS) {  // 60 ms above threshold, then back down
        TEST_ASSERT_EQUAL(FlightEvent::None, fc.update(t, 250.0f, 50.0f));
    }
    TEST_ASSERT_EQUAL(FlightEvent::None, fc.update(t, 150.0f, 50.0f));
    TEST_ASSERT_EQUAL(FlightPhase::Ground, fc.phase());
    FlightEvent last = FlightEvent::None;
    for (int i = 0; i < 10 && last == FlightEvent::None; ++i, t += SAMPLE_MS) {
        last = fc.update(t, 250.0f, 50.0f);
    }
    TEST_ASSERT_EQUAL(FlightEvent::Launch, last);
}

void test_nan_input_never_confirms() {
    FlightComputer fc;
    for (uint32_t t = 0; t < 5000; t += SAMPLE_MS) {
        TEST_ASSERT_EQUAL(FlightEvent::None, fc.update(t, NAN, NAN));
    }
    TEST_ASSERT_EQUAL(FlightPhase::Ground, fc.phase());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_filter_holds_constant_altitude);
    RUN_TEST(test_filter_tracks_constant_velocity);
    RUN_TEST(test_filter_ignores_bad_input);
    RUN_TEST(test_nominal_flight_sequence);
    RUN_TEST(test_apogee_is_detected_near_true_apogee);
    RUN_TEST(test_landed_only_after_touchdown);
    RUN_TEST(test_failed_main_triggers_backup_once_after_lockout);
    RUN_TEST(test_pad_noise_and_glitches_do_not_launch);
    RUN_TEST(test_coast_glitch_does_not_trigger_early_apogee);
    RUN_TEST(test_ground_reference_ignores_glitch_during_calibration);
    RUN_TEST(test_launch_needs_sustained_altitude);
    RUN_TEST(test_nan_input_never_confirms);
    return UNITY_END();
}
