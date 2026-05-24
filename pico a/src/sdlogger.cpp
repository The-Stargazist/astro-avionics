#include "sdlogger.h"
#include "pico/critical_section.h"

SDLogger logger;

// Critical section for cross-core safety
static critical_section_t buffer_cs;

bool SDLogger::begin()
{
    // Init critical section
    critical_section_init(&buffer_cs);

    // Configure SPI pins
    SPI.setRX(SD_MISO);
    SPI.setTX(SD_MOSI);
    SPI.setSCK(SD_SCK);

    if (!SD.begin(SD_CS, SPI))
        return false;

    char filename[32];
    int fileIndex = 1;

    bool isNewFile = false;

    // Find next available file
    while (true)
    {
        snprintf(filename, sizeof(filename), "flight%03d.csv", fileIndex);

        if (!SD.exists(filename)) {
            isNewFile = true;
            break;
        }

        fileIndex++;
    }

    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile)
        return false;

    // Write header if new file
    if (isNewFile)
    {
        logFile.println("time,altitude,velocity,state");
    }

    isLogging = true;   // ✅ ENABLE LOGGING

    return true;
}

void SDLogger::append(const char* data)
{
    if (!isLogging)
        return;

    int len = strlen(data);

    if (len <= 0 || len >= BUFFER_SIZE)
        return;

    critical_section_enter_blocking(&buffer_cs);

    if (activeIndex + len >= BUFFER_SIZE)
    {
        if (flushBuffer == nullptr)
        {
            flushBuffer = activeBuffer;
            flushSize = activeIndex;

            activeBuffer = (activeBuffer == bufferA) ? bufferB : bufferA;
            activeIndex = 0;
        }
        else
        {
            // Both buffers busy → drop safely
            critical_section_exit(&buffer_cs);
            return;
        }
    }

    memcpy(&activeBuffer[activeIndex], data, len);
    activeIndex += len;

    critical_section_exit(&buffer_cs);
}

void SDLogger::logSample()
{
    if (!isLogging)
        return;

    char line[96];

    unsigned long t = millis();
    int state = static_cast<int>(flightState);

    float alt = altitude;
    float vel = velocity;

    snprintf(line, sizeof(line),
             "%lu,%.3f,%.3f,%d\n",
             t,
             alt,
             vel,
             state);

    append(line);
}

void SDLogger::logString(const char* msg)
{
    if (!isLogging)
        return;

    char line[128];

    snprintf(line, sizeof(line),
             "%lu,%s\n",
             millis(),
             msg);

    append(line);
}

void SDLogger::logContinuity(bool mcont, bool bcont)
{
    if (!isLogging)
        return;

    char line[128];

    const char* mainStatus   = mcont ? "TRUE" : "FALSE";
    const char* backupStatus = bcont ? "TRUE" : "FALSE";

    snprintf(line, sizeof(line),
             "%lu,Continuity => Main: %s, Backup: %s\n",
             millis(),
             mainStatus,
             backupStatus);

    append(line);
}

void SDLogger::flush()
{
    if (!logFile || !isLogging)
        return;

    char* buf = nullptr;
    int size = 0;

    // Atomically grab buffer
    critical_section_enter_blocking(&buffer_cs);

    if (flushBuffer != nullptr)
    {
        buf = flushBuffer;
        size = flushSize;
        flushBuffer = nullptr;
    }

    critical_section_exit(&buffer_cs);

    // Validate before write
    if (buf == nullptr || size <= 0 || size > BUFFER_SIZE)
        return;

    logFile.write((uint8_t*)buf, size);
    logFile.flush();
}

void SDLogger::stop()
{
    if (!logFile || !isLogging)
        return;

    // Flush remaining data
    flush();

    // Close file
    logFile.close();

    // Disable logging
    isLogging = false;
}