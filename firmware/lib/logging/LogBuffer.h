#pragma once

#include <stddef.h>
#include <stdint.h>

#include <atomic>

#include "pico/critical_section.h"

// Double-buffered text log shared between producers (any core or task) and
// one consumer that writes to the SD card / USB serial.
//
// Producers only copy into RAM, so a slow SD card never stalls flight logic.
// The consumer calls take() periodically: it swaps buffers and hands back the
// filled one, which stays valid until the next take(). If the consumer falls
// a full buffer behind, new lines are dropped and counted.
class LogBuffer {
public:
    // Per buffer. At 50 Hz this absorbs an SD stall of ~1.5 s (Pico B rows)
    // to ~5 s (Pico A rows).
    static constexpr size_t CAPACITY = 16384;

    // Call once before producing. Calls made before begin() are no-ops, so the
    // consumer may already be polling take() on the other core.
    void begin();

    void write(const char* line);
    void printf(const char* format, ...) __attribute__((format(printf, 2, 3)));

    // Consumer only. Returns the number of bytes at *data (0 when empty).
    size_t take(const char** data);

    uint32_t droppedLines() const { return dropped_; }

private:
    char buffers_[2][CAPACITY];
    uint8_t filling_ = 0;
    size_t used_ = 0;
    volatile uint32_t dropped_ = 0;
    critical_section_t lock_;
    std::atomic<bool> initialized_{false};
};
