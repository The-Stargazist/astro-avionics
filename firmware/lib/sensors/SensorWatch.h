#pragma once

#include <stdint.h>

// Health tracking for one sensor: healthy while it produced data within the
// timeout. Once it has been silent for longer, shouldRetry() returns true at
// most once per retry interval so the caller can re-run begin() (which
// recovers a sensor that browned out and lost its configuration).
class SensorWatch {
public:
    explicit SensorWatch(uint32_t timeoutMs = 500, uint32_t retryMs = 1000)
        : timeoutMs_(timeoutMs), retryMs_(retryMs) {}

    void good(uint32_t nowMs) {
        lastGoodMs_ = nowMs;
        everGood_ = true;
    }

    bool healthy(uint32_t nowMs) const { return everGood_ && nowMs - lastGoodMs_ <= timeoutMs_; }

    bool shouldRetry(uint32_t nowMs) {
        if (healthy(nowMs) || (retried_ && nowMs - lastRetryMs_ < retryMs_)) {
            return false;
        }
        retried_ = true;
        lastRetryMs_ = nowMs;
        return true;
    }

private:
    uint32_t timeoutMs_;
    uint32_t retryMs_;
    uint32_t lastGoodMs_ = 0;
    uint32_t lastRetryMs_ = 0;
    bool everGood_ = false;
    bool retried_ = false;
};
