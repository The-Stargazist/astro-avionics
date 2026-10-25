#include "LogBuffer.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void LogBuffer::begin() {
    if (!initialized_) {
        critical_section_init(&lock_);
        initialized_ = true;
    }
}

void LogBuffer::write(const char* line) {
    if (!initialized_) {
        return;
    }
    const size_t length = strlen(line);
    critical_section_enter_blocking(&lock_);
    if (used_ + length <= CAPACITY) {
        memcpy(&buffers_[filling_][used_], line, length);
        used_ += length;
    } else {
        dropped_ = dropped_ + 1;
    }
    critical_section_exit(&lock_);
}

void LogBuffer::printf(const char* format, ...) {
    char line[256];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length <= 0) {
        return;
    }
    if (static_cast<size_t>(length) >= sizeof(line)) {
        line[sizeof(line) - 2] = '\n';  // truncated: keep the line terminated
    }
    write(line);
}

size_t LogBuffer::take(const char** data) {
    if (!initialized_) {
        *data = nullptr;
        return 0;
    }
    critical_section_enter_blocking(&lock_);
    *data = buffers_[filling_];
    const size_t length = used_;
    filling_ ^= 1;
    used_ = 0;
    critical_section_exit(&lock_);
    return length;
}
