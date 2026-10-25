// Pico A: deployment flight computer.
//
// Flies on its own barometer (BMP390 "B" on I2C rail B), fires MAIN at
// apogee and BACKUP (plus a second MAIN pulse) on descent overspeed, and logs
// to SD card A. Core 0 runs sensing and flight logic and never blocks on the
// SD card or USB; core 1 drains the log buffer to the card and USB serial.
//
// Status LED: 1 Hz blink = ready, solid = main/backup continuity missing,
// 5 Hz blink = barometer or SD card fault.

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include <atomic>

#include "BaroTracker.h"
#include "Bmp390.h"
#include "LogBuffer.h"
#include "LogFile.h"
#include "PyroChannel.h"
#include "SensorWatch.h"
#include "VadaPins.h"

namespace {

constexpr uint32_t MAIN_PULSE_MS = 500;
constexpr uint32_t BACKUP_PULSE_MS = 1000;
constexpr uint32_t MAIN_RETRY_PULSE_MS = 1000;

constexpr uint32_t BARO_POLL_MS = 2;          // sensor produces a sample every 20 ms
constexpr uint32_t CONTINUITY_INTERVAL_MS = 1000;  // only while on the ground
constexpr uint32_t LOG_SERVICE_MS = 100;      // SD write + flush at 10 Hz

constexpr const char* LOG_HEADER = "time_ms,pressure_pa,temp_c,alt_raw_m,alt_m,vel_mps,phase\n";

PyroChannel mainPyro("main", pins::MAIN_TRIG, pins::MAIN_CONT, pins::MAIN_SENSE);
PyroChannel backupPyro("backup", pins::BACKUP_TRIG, pins::BACKUP_CONT, pins::BACKUP_SENSE);
// Not used by this firmware, but their outputs are held low and continuity is reported.
PyroChannel motorPyro("motor", pins::MOTOR_TRIG, pins::MOTOR_CONT, pins::MOTOR_SENSE);
PyroChannel stagePyro("stage", pins::STAGE_TRIG, pins::STAGE_CONT, pins::STAGE_SENSE);
PyroChannel checkLoop("check", PyroChannel::NO_PIN, pins::CHECK_CONT, pins::CHECK_SENSE);

Bmp390 baro(Wire);
SensorWatch baroWatch;  // fault after 500 ms without a sample, re-init every 1 s
BaroTracker tracker;
LogBuffer logBuffer;

// Owned by core 1 once sdReady is set; core 0 only touches it during setup().
File logFile;
std::atomic<bool> sdReady{false};
std::atomic<bool> closeLogRequested{false};

bool baroHealthy = false;
bool continuityOk = false;
bool groundReported = false;
bool mainRetryPending = false;
uint32_t lastContinuityMs = 0;

void logEvent(uint32_t nowMs, const char* message) {
    logBuffer.printf("# %lu %s\n", static_cast<unsigned long>(nowMs), message);
}

void checkContinuity(uint32_t nowMs, bool always) {
    const bool main = mainPyro.checkContinuity();
    const bool backup = backupPyro.checkContinuity();
    const bool ok = main && backup;
    if (always || ok != continuityOk) {
        logBuffer.printf("# %lu continuity main=%d backup=%d motor=%d stage=%d check=%d\n",
                         static_cast<unsigned long>(nowMs), main, backup,
                         motorPyro.checkContinuity(), stagePyro.checkContinuity(),
                         checkLoop.checkContinuity());
    }
    continuityOk = ok;
    lastContinuityMs = nowMs;
}

void handleFlightEvent(uint32_t nowMs, FlightEvent event) {
    switch (event) {
        case FlightEvent::None:
            break;
        case FlightEvent::Launch:
            logEvent(nowMs, "LAUNCH detected");
            break;
        case FlightEvent::Apogee:
            mainPyro.fire(nowMs, MAIN_PULSE_MS);
            logBuffer.printf("# %lu APOGEE at %.1f m AGL, firing main\n",
                             static_cast<unsigned long>(nowMs), tracker.maxAltitudeAglM());
            break;
        case FlightEvent::Overspeed:
            backupPyro.fire(nowMs, BACKUP_PULSE_MS);
            mainRetryPending = true;  // second main pulse once the backup pulse ends
            logBuffer.printf("# %lu OVERSPEED %.1f m/s, firing backup\n",
                             static_cast<unsigned long>(nowMs), tracker.velocityMps());
            break;
        case FlightEvent::Landed:
            logEvent(nowMs, "LANDED, closing log file");
            closeLogRequested = true;
            break;
    }
}

void servicePyros(uint32_t nowMs) {
    mainPyro.update(nowMs);
    backupPyro.update(nowMs);
    if (mainRetryPending && !backupPyro.firing()) {
        mainRetryPending = false;
        mainPyro.fire(nowMs, MAIN_RETRY_PULSE_MS);
        logEvent(nowMs, "firing main again");
    }
}

void serviceBarometer(uint32_t nowMs) {
    if (baro.read()) {
        baroWatch.good(nowMs);
        const FlightEvent event = tracker.update(nowMs, baro.altitudeM());
        if (tracker.groundReady() && !groundReported) {
            groundReported = true;
            logBuffer.printf("# %lu ground reference %.2f m\n", static_cast<unsigned long>(nowMs),
                             tracker.groundAltitudeM());
        }
        logBuffer.printf("%lu,%.1f,%.2f,%.2f,%.2f,%.2f,%u\n", static_cast<unsigned long>(nowMs),
                         baro.pressurePa(), baro.temperatureC(), tracker.rawAltitudeAglM(),
                         tracker.altitudeAglM(), tracker.velocityMps(),
                         static_cast<unsigned>(tracker.phase()));
        handleFlightEvent(nowMs, event);
    } else if (baroWatch.shouldRetry(nowMs) && baro.begin()) {
        baroWatch.good(nowMs);
    }

    const bool healthy = baroWatch.healthy(nowMs);
    if (healthy != baroHealthy) {
        baroHealthy = healthy;
        logEvent(nowMs, healthy ? "barometer OK" : "barometer lost");
    }
}

void updateStatusLed(uint32_t nowMs) {
    const bool sdFault = !sdReady && tracker.phase() != FlightPhase::Landed;
    bool on;
    if (!baroHealthy || sdFault) {
        on = (nowMs / 100) % 2;
    } else if (!continuityOk) {
        on = true;
    } else {
        on = (nowMs / 500) % 2;
    }
    digitalWrite(pins::STATUS_LED, on ? HIGH : LOW);
}

}  // namespace

void setup() {
    // Pyro outputs low before anything else can go wrong.
    mainPyro.begin();
    backupPyro.begin();
    motorPyro.begin();
    stagePyro.begin();
    checkLoop.begin();

    pinMode(pins::STATUS_LED, OUTPUT);
    Serial.begin(115200);
    logBuffer.begin();

    const uint32_t now = millis();
    logEvent(now, "Pico A boot");

    SPI.setRX(pins::SPI_MISO);
    SPI.setTX(pins::SPI_MOSI);
    SPI.setSCK(pins::SPI_SCK);
    char fileName[16] = "";
    if (SD.begin(pins::SPI_CS_SD) &&
        openNextLogFile(logFile, "flight", LOG_HEADER, fileName, sizeof(fileName))) {
        sdReady = true;
        logBuffer.printf("# %lu logging to %s\n", static_cast<unsigned long>(now), fileName);
    } else {
        logEvent(now, "SD card not available");
    }

    Wire.setSDA(pins::I2C_SDA);
    Wire.setSCL(pins::I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);
    if (baro.begin()) {
        baroWatch.good(millis());
    } else {
        logEvent(now, "barometer not found");
    }

    checkContinuity(millis(), true);
}

void loop() {
    const uint32_t now = millis();
    servicePyros(now);
    serviceBarometer(now);
    if (tracker.phase() == FlightPhase::Ground && now - lastContinuityMs >= CONTINUITY_INTERVAL_MS) {
        checkContinuity(now, false);
    }
    updateStatusLed(now);
    delay(BARO_POLL_MS);
}

// Core 1: the only code that touches the SD card or USB serial after setup().
// arduino-pico starts core 1 before setup() runs on core 0; until logBuffer
// and sdReady are set up this loop just finds nothing to do.
void setup1() {}

void loop1() {
    // Read the request before draining so lines logged before it are written.
    const bool closeRequested = closeLogRequested;
    const char* data = nullptr;
    const size_t length = logBuffer.take(&data);
    if (length > 0) {
        if (sdReady) {
            const size_t written = logFile.write(reinterpret_cast<const uint8_t*>(data), length);
            logFile.flush();
            if (written != length) {
                sdReady = false;  // card removed or full: show the fault on the LED
                logBuffer.printf("# %lu SD write failed\n", static_cast<unsigned long>(millis()));
            }
        }
        Serial.write(reinterpret_cast<const uint8_t*>(data), length);
    }
    if (closeRequested && sdReady) {
        logFile.close();
        sdReady = false;
    }
    delay(LOG_SERVICE_MS);
}
