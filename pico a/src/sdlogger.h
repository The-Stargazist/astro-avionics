#pragma once

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <cstring>

// ===============================
// SD SPI Pin Configuration
// ===============================

#define SD_MISO 16
#define SD_MOSI 19
#define SD_SCK  18
#define SD_CS   17


// ===============================
// Shared flight data
// ===============================

extern volatile float altitude;
extern volatile float velocity;

enum class FlightState : int;
extern volatile FlightState flightState;


// ===============================
// Logger Class
// ===============================

class SDLogger {
public:
    bool begin();

    void logSample();
    void logString(const char* msg);
    void logContinuity(bool mcont, bool bcont);

    void flush();
    void stop();

private:
    static const int BUFFER_SIZE = 2048;

    File logFile;

    char bufferA[BUFFER_SIZE];
    char bufferB[BUFFER_SIZE];

    char* activeBuffer = bufferA;
    char* flushBuffer  = nullptr;

    volatile int activeIndex = 0;
    volatile int flushSize = 0;

    bool isLogging = false;

    void append(const char* data);
};

extern SDLogger logger;