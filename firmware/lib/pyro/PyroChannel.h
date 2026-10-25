#pragma once

#include <Arduino.h>

// One pyro channel on the VADA board:
//   TRIG  drives the CPC1020N solid-state relay that connects +BATT to the e-match
//   CONT  sources a small test current (33k) into the e-match terminal
//   SENSE reads that terminal back: LOW with CONT high means the e-match is connected
//
// Main and backup TRIG/CONT lines are diode-OR'd between Pico A and Pico B and
// both Picos read the same SENSE net, so either board can test or fire them.
//
// fire() is non-blocking; call update() every loop to end the pulse.
class PyroChannel {
public:
    static constexpr uint8_t NO_PIN = 0xFF;

    PyroChannel(const char* name, uint8_t triggerPin, uint8_t continuityPin, uint8_t sensePin)
        : name_(name), triggerPin_(triggerPin), continuityPin_(continuityPin), sensePin_(sensePin) {}

    // Drives TRIG and CONT low. Call before anything else in setup().
    void begin();

    // Blocks for ~10 ms. Returns true when an e-match is connected.
    bool checkContinuity();

    void fire(uint32_t nowMs, uint32_t durationMs);
    void update(uint32_t nowMs);

    bool firing() const { return firing_; }
    bool fired() const { return fired_; }
    const char* name() const { return name_; }

private:
    const char* name_;
    uint8_t triggerPin_;
    uint8_t continuityPin_;
    uint8_t sensePin_;
    bool firing_ = false;
    bool fired_ = false;
    uint32_t fireStartMs_ = 0;
    uint32_t fireDurationMs_ = 0;
};
