#include "PyroChannel.h"

void PyroChannel::begin() {
    if (triggerPin_ != NO_PIN) {
        pinMode(triggerPin_, OUTPUT);
        digitalWrite(triggerPin_, LOW);
    }
    if (continuityPin_ != NO_PIN) {
        pinMode(continuityPin_, OUTPUT);
        digitalWrite(continuityPin_, LOW);
    }
    if (sensePin_ != NO_PIN) {
        pinMode(sensePin_, INPUT);
    }
}

bool PyroChannel::checkContinuity() {
    if (continuityPin_ == NO_PIN || sensePin_ == NO_PIN) {
        return false;
    }
    digitalWrite(continuityPin_, HIGH);
    delay(10);
    const bool connected = digitalRead(sensePin_) == LOW;
    digitalWrite(continuityPin_, LOW);
    return connected;
}

void PyroChannel::fire(uint32_t nowMs, uint32_t durationMs) {
    if (triggerPin_ == NO_PIN || firing_) {
        return;
    }
    digitalWrite(triggerPin_, HIGH);
    firing_ = true;
    fired_ = true;
    fireStartMs_ = nowMs;
    fireDurationMs_ = durationMs;
}

void PyroChannel::update(uint32_t nowMs) {
    if (firing_ && nowMs - fireStartMs_ >= fireDurationMs_) {
        digitalWrite(triggerPin_, LOW);
        firing_ = false;
    }
}
