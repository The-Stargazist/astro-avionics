// Pico B: sensor, telemetry and logging computer.
//
// Reads everything on I2C rail A (ICM-20649, LIS3MDL, BMP390 "A") plus the
// GPS, tracks flight phase from its own barometer, monitors main/backup
// continuity, logs to SD card B and sends LoRa telemetry. It does not fire
// pyro channels (deployment is Pico A's job); its TRIG outputs are held low.
//
// FreeRTOS tasks (priority):
//   SENSOR (3) every 5 ms: barometer poll (50 Hz samples), IMU at 100 Hz, mag at 25 Hz
//   GPS    (2) every 20 ms: UART -> NMEA parser
//   SAMPLE (2) every 20 ms: snapshot -> CSV row in the log buffer, latest -> radio
//   CONT   (1) every 1 s: main/backup continuity
//   LOGGER (1) every 100 ms: log buffer -> SD card
//   RADIO  (1) every 250 ms: one LoRa packet
//   loop()     every 1 s: status LED and a USB status line
// SD card and LoRa share SPI0 behind spiMutex; everything on I2C is in SENSOR.
//
// Status LED: 1 Hz blink = ready, solid = main/backup continuity missing,
// 5 Hz blink = a sensor, the SD card or the radio is not working.

#include <Arduino.h>
#include <LoRa.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include "BaroTracker.h"
#include "Bmp390.h"
#include "Icm20649.h"
#include "Lis3mdl.h"
#include "LogBuffer.h"
#include "LogFile.h"
#include "NmeaParser.h"
#include "PyroChannel.h"
#include "SensorWatch.h"
#include "VadaPins.h"

namespace {

constexpr long LORA_FREQUENCY_HZ = 433E6;
constexpr int LORA_SPREADING_FACTOR = 7;
constexpr int LORA_TX_POWER_DBM = 17;
constexpr uint32_t GPS_BAUD = 9600;

constexpr uint32_t SENSOR_PERIOD_MS = 5;
constexpr uint32_t IMU_EVERY_N_TICKS = 2;   // 100 Hz
constexpr uint32_t MAG_EVERY_N_TICKS = 8;   // 25 Hz
constexpr uint32_t GPS_PERIOD_MS = 20;
constexpr uint32_t SAMPLE_PERIOD_MS = 20;
constexpr uint32_t CONTINUITY_PERIOD_MS = 1000;
constexpr uint32_t LOGGER_PERIOD_MS = 100;
constexpr uint32_t RADIO_PERIOD_MS = 250;   // ~115 ms airtime per packet at SF7/125 kHz
constexpr uint32_t STATUS_PERIOD_MS = 1000;

constexpr const char* LOG_HEADER =
    "time_ms,phase,pressure_pa,baro_temp_c,alt_raw_m,alt_m,vel_mps,"
    "ax,ay,az,gx,gy,gz,imu_temp_c,mx,my,mz,lat,lon,gps_alt_m,gps_sats,gps_fix,flags\n";

// Health bits, logged as `flags` and sent in telemetry.
enum Flag : uint16_t {
    FLAG_BARO = 1 << 0,
    FLAG_IMU = 1 << 1,
    FLAG_MAG = 1 << 2,
    FLAG_GPS_FIX = 1 << 3,
    FLAG_SD = 1 << 4,
    FLAG_RADIO = 1 << 5,
    FLAG_MAIN_CONTINUITY = 1 << 6,
    FLAG_BACKUP_CONTINUITY = 1 << 7,
};

struct Snapshot {
    uint32_t timeMs = 0;
    FlightPhase phase = FlightPhase::Ground;
    float pressurePa = 0.0f;
    float baroTempC = 0.0f;
    float rawAltitudeM = 0.0f;  // AGL, unfiltered
    float altitudeM = 0.0f;     // AGL, filtered
    float velocityMps = 0.0f;
    float maxAltitudeM = 0.0f;
    Vec3 accel;
    Vec3 gyro;
    float imuTempC = 0.0f;
    Vec3 mag;
    GpsFix gps;
    uint16_t flags = 0;
};

PyroChannel mainPyro("main", pins::MAIN_TRIG, pins::MAIN_CONT, pins::MAIN_SENSE);
PyroChannel backupPyro("backup", pins::BACKUP_TRIG, pins::BACKUP_CONT, pins::BACKUP_SENSE);

Bmp390 baro(Wire1);
Icm20649 imu(Wire1);
Lis3mdl mag(Wire1);
NmeaParser gps;
LogBuffer logBuffer;
File logFile;

SemaphoreHandle_t stateMutex;  // guards `state`
SemaphoreHandle_t spiMutex;    // guards SPI0 (SD card and LoRa)
QueueHandle_t radioMailbox;    // length 1: always holds the newest snapshot
Snapshot state;

bool sdReady = false;     // set in setup(), afterwards only LOGGER touches it
bool radioReady = false;  // written in setup() only

void setFlag(uint16_t flag, bool on) {
    state.flags = on ? (state.flags | flag) : (state.flags & ~flag);
}

class StateLock {
public:
    StateLock() { xSemaphoreTake(stateMutex, portMAX_DELAY); }
    ~StateLock() { xSemaphoreGive(stateMutex); }
};

void sensorTask(void*) {
    BaroTracker tracker;
    SensorWatch baroWatch;
    SensorWatch imuWatch;
    SensorWatch magWatch;
    const uint32_t start = millis();
    if (baro.configured()) baroWatch.good(start);
    if (imu.configured()) imuWatch.good(start);
    if (mag.configured()) magWatch.good(start);

    TickType_t wake = xTaskGetTickCount();
    for (uint32_t tick = 0;; ++tick) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
        const uint32_t now = millis();

        FlightEvent event = FlightEvent::None;
        const bool baroSample = baro.read();
        if (baroSample) {
            baroWatch.good(now);
            event = tracker.update(now, baro.altitudeM());
        } else if (baroWatch.shouldRetry(now) && baro.begin()) {
            baroWatch.good(now);
        }

        const bool imuSample = tick % IMU_EVERY_N_TICKS == 0 && imu.read();
        if (imuSample) {
            imuWatch.good(now);
        } else if (imuWatch.shouldRetry(now) && imu.begin()) {
            imuWatch.good(now);
        }

        const bool magSample = tick % MAG_EVERY_N_TICKS == 0 && mag.read();
        if (magSample) {
            magWatch.good(now);
        } else if (magWatch.shouldRetry(now) && mag.begin()) {
            magWatch.good(now);
        }

        {
            StateLock lock;
            if (baroSample) {
                state.phase = tracker.phase();
                state.pressurePa = baro.pressurePa();
                state.baroTempC = baro.temperatureC();
                state.rawAltitudeM = tracker.rawAltitudeAglM();
                state.altitudeM = tracker.altitudeAglM();
                state.velocityMps = tracker.velocityMps();
                state.maxAltitudeM = tracker.maxAltitudeAglM();
            }
            if (imuSample) {
                state.accel = imu.accel();
                state.gyro = imu.gyro();
                state.imuTempC = imu.temperatureC();
            }
            if (magSample) {
                state.mag = mag.field();
            }
            setFlag(FLAG_BARO, baroWatch.healthy(now));
            setFlag(FLAG_IMU, imuWatch.healthy(now));
            setFlag(FLAG_MAG, magWatch.healthy(now));
        }

        if (event != FlightEvent::None) {
            logBuffer.printf("# %lu %s alt=%.1f vel=%.1f max=%.1f\n",
                             static_cast<unsigned long>(now), toString(event),
                             tracker.altitudeAglM(), tracker.velocityMps(),
                             tracker.maxAltitudeAglM());
        }
    }
}

void gpsTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(GPS_PERIOD_MS));
        bool updated = false;
        while (Serial2.available() > 0) {
            updated |= gps.feed(static_cast<char>(Serial2.read()));
        }
        if (updated) {
            StateLock lock;
            state.gps = gps.fix();
            setFlag(FLAG_GPS_FIX, gps.fix().valid);
        }
    }
}

void continuityTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    bool lastMain = false;
    bool lastBackup = false;
    for (bool first = true;; first = false) {
        const bool main = mainPyro.checkContinuity();
        const bool backup = backupPyro.checkContinuity();
        {
            StateLock lock;
            setFlag(FLAG_MAIN_CONTINUITY, main);
            setFlag(FLAG_BACKUP_CONTINUITY, backup);
        }
        if (first || main != lastMain || backup != lastBackup) {
            logBuffer.printf("# %lu continuity main=%d backup=%d\n",
                             static_cast<unsigned long>(millis()), main, backup);
        }
        lastMain = main;
        lastBackup = backup;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTINUITY_PERIOD_MS));
    }
}

void sampleTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        Snapshot s;
        {
            StateLock lock;
            state.timeMs = millis();
            s = state;
        }
        xQueueOverwrite(radioMailbox, &s);

        logBuffer.printf(
            "%lu,%u,%.1f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,"
            "%.7f,%.7f,%.1f,%u,%u,%u\n",
            static_cast<unsigned long>(s.timeMs), static_cast<unsigned>(s.phase), s.pressurePa,
            s.baroTempC, s.rawAltitudeM, s.altitudeM, s.velocityMps, s.accel.x, s.accel.y,
            s.accel.z, s.gyro.x, s.gyro.y, s.gyro.z, s.imuTempC, s.mag.x, s.mag.y, s.mag.z,
            s.gps.latitudeDeg, s.gps.longitudeDeg, s.gps.altitudeM,
            static_cast<unsigned>(s.gps.satellites), static_cast<unsigned>(s.gps.valid),
            static_cast<unsigned>(s.flags));
    }
}

void loggerTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOGGER_PERIOD_MS));
        const char* data = nullptr;
        const size_t length = logBuffer.take(&data);
        if (length == 0 || !sdReady) {
            continue;
        }
        xSemaphoreTake(spiMutex, portMAX_DELAY);
        const size_t written = logFile.write(reinterpret_cast<const uint8_t*>(data), length);
        logFile.flush();
        xSemaphoreGive(spiMutex);
        if (written != length) {
            sdReady = false;  // card removed or full: report it instead of retrying
            StateLock lock;
            setFlag(FLAG_SD, false);
        }
    }
}

// Telemetry: VADA,time_ms,phase,alt_m,vel_mps,max_alt_m,lat,lon,sats,flags_hex
void radioTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(RADIO_PERIOD_MS));
        Snapshot s;
        if (xQueuePeek(radioMailbox, &s, 0) != pdTRUE) {
            continue;
        }
        char packet[96];
        snprintf(packet, sizeof(packet), "VADA,%lu,%u,%.1f,%.1f,%.1f,%.6f,%.6f,%u,%04X",
                 static_cast<unsigned long>(s.timeMs), static_cast<unsigned>(s.phase),
                 s.altitudeM, s.velocityMps, s.maxAltitudeM, s.gps.latitudeDeg,
                 s.gps.longitudeDeg, static_cast<unsigned>(s.gps.satellites),
                 static_cast<unsigned>(s.flags));

        xSemaphoreTake(spiMutex, portMAX_DELAY);
        // beginPacket() returns 0 while the previous packet is still on air;
        // writing the FIFO then would corrupt it, so skip this slot.
        if (LoRa.beginPacket()) {
            LoRa.print(packet);
            LoRa.endPacket(true);  // async: do not hold the SPI bus during airtime
        }
        xSemaphoreGive(spiMutex);
    }
}

bool initRadio() {
    LoRa.setSPI(SPI);
    LoRa.setPins(pins::SPI_CS_LORA, pins::LORA_RESET, pins::LORA_DIO0);
    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        return false;
    }
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setTxPower(LORA_TX_POWER_DBM);
    LoRa.enableCrc();
    return true;
}

void startTask(TaskFunction_t task, const char* name, uint32_t stackWords, UBaseType_t priority) {
    if (xTaskCreate(task, name, stackWords, nullptr, priority, nullptr) != pdPASS) {
        logBuffer.printf("# %lu failed to start task %s\n", static_cast<unsigned long>(millis()),
                         name);
    }
}

}  // namespace

void setup() {
    // This board never fires; hold its pyro outputs low.
    mainPyro.begin();
    backupPyro.begin();

    pinMode(pins::STATUS_LED, OUTPUT);
    Serial.begin(115200);
    logBuffer.begin();
    stateMutex = xSemaphoreCreateMutex();
    spiMutex = xSemaphoreCreateMutex();
    radioMailbox = xQueueCreate(1, sizeof(Snapshot));

    logBuffer.printf("# %lu Pico B boot\n", static_cast<unsigned long>(millis()));

    Wire1.setSDA(pins::I2C_SDA);
    Wire1.setSCL(pins::I2C_SCL);
    Wire1.begin();
    Wire1.setClock(400000);
    const bool baroOk = baro.begin();
    const bool imuOk = imu.begin();
    const bool magOk = mag.begin();
    logBuffer.printf("# %lu sensors baro=%d imu=%d mag=%d\n", static_cast<unsigned long>(millis()),
                     baroOk, imuOk, magOk);

    Serial2.setTX(pins::GPS_UART_TX);
    Serial2.setRX(pins::GPS_UART_RX);
    Serial2.setFIFOSize(512);  // 9600 baud fills the default 32 bytes in ~33 ms
    Serial2.begin(GPS_BAUD);

    // Both chip selects have pull-ups on the board; drive them high anyway
    // before the first SPI transfer so neither device sees the other's traffic.
    pinMode(pins::SPI_CS_SD, OUTPUT);
    digitalWrite(pins::SPI_CS_SD, HIGH);
    pinMode(pins::SPI_CS_LORA, OUTPUT);
    digitalWrite(pins::SPI_CS_LORA, HIGH);
    SPI.setRX(pins::SPI_MISO);
    SPI.setTX(pins::SPI_MOSI);
    SPI.setSCK(pins::SPI_SCK);

    char fileName[16] = "";
    sdReady = SD.begin(pins::SPI_CS_SD) &&
              openNextLogFile(logFile, "telem", LOG_HEADER, fileName, sizeof(fileName));
    logBuffer.printf("# %lu sd=%d %s\n", static_cast<unsigned long>(millis()), sdReady, fileName);

    radioReady = initRadio();
    logBuffer.printf("# %lu radio=%d\n", static_cast<unsigned long>(millis()), radioReady);

    setFlag(FLAG_SD, sdReady);
    setFlag(FLAG_RADIO, radioReady);

    // The scheduler is already running, so tasks start as soon as they are
    // created: create them only once everything they use is initialised.
    startTask(sensorTask, "SENSOR", 2048, 3);
    startTask(gpsTask, "GPS", 1024, 2);
    startTask(sampleTask, "SAMPLE", 2048, 2);
    startTask(continuityTask, "CONT", 1024, 1);
    startTask(loggerTask, "LOGGER", 2048, 1);
    if (radioReady) {
        startTask(radioTask, "RADIO", 1024, 1);
    }
}

void loop() {
    Snapshot s;
    {
        StateLock lock;
        s = state;
    }
    const uint16_t required = FLAG_BARO | FLAG_IMU | FLAG_MAG | FLAG_SD | FLAG_RADIO;
    const uint16_t continuity = FLAG_MAIN_CONTINUITY | FLAG_BACKUP_CONTINUITY;

    // Blink pattern over one second: 5 Hz on fault, solid without continuity, else 1 Hz.
    for (uint32_t elapsed = 0; elapsed < STATUS_PERIOD_MS; elapsed += 100) {
        bool on;
        if ((s.flags & required) != required) {
            on = (elapsed / 100) % 2;
        } else if ((s.flags & continuity) != continuity) {
            on = true;
        } else {
            on = elapsed < STATUS_PERIOD_MS / 2;
        }
        digitalWrite(pins::STATUS_LED, on ? HIGH : LOW);
        delay(100);
    }

    Serial.printf("%s alt=%.1f vel=%.1f max=%.1f gps=%d sats=%u flags=0x%04X dropped=%lu\n",
                  toString(s.phase), s.altitudeM, s.velocityMps, s.maxAltitudeM, s.gps.valid,
                  static_cast<unsigned>(s.gps.satellites), static_cast<unsigned>(s.flags),
                  static_cast<unsigned long>(logBuffer.droppedLines()));
}
