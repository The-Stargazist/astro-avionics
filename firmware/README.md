# VADA flight computer firmware

Firmware for the two Raspberry Pi Pico 2 boards on the VADA PCB (`../vada`).
One PlatformIO project builds both:

| Environment | Board  | Role |
|-------------|--------|------|
| `pico_a`    | Pico A | **Deployment computer.** Own barometer (BMP390 "B"), fires MAIN at apogee and BACKUP on descent overspeed, logs to SD card A. Bare-metal dual core: core 0 flies, core 1 writes the SD card. |
| `pico_b`    | Pico B | **Sensor, telemetry and logging computer.** IMU, magnetometer, BMP390 "A", GPS, continuity monitoring, LoRa telemetry, logs to SD card B. FreeRTOS. Never fires a pyro channel. |

The two boards run independently: each has its own barometer, I2C bus and SD
card, so a failure on one does not take down the other.

## Build, flash, test

```bash
pio run -e pico_a -t upload
```

```bash
pio run -e pico_b -t upload
```

```bash
pio test -e native
```

`pio test -e native` runs the host unit tests (flight logic against simulated
flights, NMEA parsing, BMP390 compensation against Bosch's reference values).
It needs `gcc`/`g++` on `PATH`; on Windows MSYS2 or MinGW works. CI runs the
tests and both firmware builds on every push (`.github/workflows/firmware.yml`).

## Board setup

**Solder jumpers JP1, JP2, JP3 and JP4 must all be bridged.** The PCB routes:

- I2C rail A (ICM-20649, LIS3MDL, BMP390 "A") to Pico B GPIO10/11 through JP1/JP2,
- I2C rail B (BMP390 "B") to Pico A GPIO20/21 through JP3/JP4.

Each Pico masters exactly one rail and leaves the other rail's pins as inputs,
so no bus ever has two masters. The full pin map, read from
`vada/VADA.kicad_pcb`, is in [`lib/board/VadaPins.h`](lib/board/VadaPins.h).

Pyro channels (MAIN = J2 "Pyro 3", BACKUP = J3 "Pyro 4") have their TRIG and
CONT lines diode-OR'd between the two Picos and a shared SENSE net, so either
board can test them. MOTOR (J5) and STAGE (J6) are wired to Pico A only; this
firmware holds them low and only reports their continuity.

## Flight logic (both boards)

`lib/flight` is hardware-independent and unit-tested on the host.

1. **Ground reference:** median of the first 50 baro samples (1 s).
2. **Filtering:** an alpha-beta filter turns AGL altitude into smoothed
   altitude and vertical velocity. Raw differentiation of a 50 Hz barometer
   gives several m/s of velocity noise; the filter gets that to ~0.15 m/s.
   Samples more than 50 m from the prediction (corrupt reads) are dropped.
3. **Phases**, each transition confirmed over time, not one sample:

| From → To | Condition (defaults in `FlightConfig`) | Pico A action |
|-----------|----------------------------------------|---------------|
| GROUND → FLIGHT | AGL > 200 m for 100 ms | log |
| FLIGHT → DESCENT | velocity < 2 m/s for 100 ms (apogee) | fire MAIN 500 ms |
| DESCENT (once) | velocity < −20 m/s for 200 ms, ignored for 2 s after apogee | fire BACKUP 1 s, then MAIN again 1 s |
| DESCENT → LANDED | \|velocity\| < 2 m/s for 20 s | close log file |

The 2 s lockout after apogee exists because the main ejection charge
pressurises the avionics bay and briefly corrupts baro readings.

Pico B runs the same logic only to report the phase in its log and
telemetry. Pico A re-checks continuity every second while on the ground, so
e-matches connected after power-up are picked up.

## Status LED (both boards)

| Pattern | Meaning |
|---------|---------|
| 1 Hz blink | Ready: sensors, SD (and radio on Pico B) working, MAIN and BACKUP continuity present |
| Solid on | MAIN or BACKUP continuity missing |
| 5 Hz blink | A sensor, the SD card or (Pico B) the radio is not working |

## Logs

Each boot opens the next free file: `flightNNN.csv` (Pico A) or
`telemNNN.csv` (Pico B). Data is flushed to the card at 10 Hz. Lines starting
with `#` are events (`# <time_ms> <message>`); read the files with
`pandas.read_csv(path, comment="#")`.

Pico A, 50 Hz:

```
time_ms,pressure_pa,temp_c,alt_raw_m,alt_m,vel_mps,phase
```

Pico B, 50 Hz (accel m/s², gyro deg/s, mag gauss, all in the sensor frame):

```
time_ms,phase,pressure_pa,baro_temp_c,alt_raw_m,alt_m,vel_mps,ax,ay,az,gx,gy,gz,imu_temp_c,mx,my,mz,lat,lon,gps_alt_m,gps_sats,gps_fix,flags
```

`phase`: 0 GROUND, 1 FLIGHT, 2 DESCENT, 3 LANDED. Altitudes are metres above
the pad; `alt_raw_m` is unfiltered. Pico A also echoes its log on USB serial;
Pico B prints a one-line status every second.

## Telemetry (Pico B, LoRa 433 MHz, SF7, CRC on, 4 Hz)

ASCII CSV, one packet per line:

```
VADA,time_ms,phase,alt_m,vel_mps,max_alt_m,lat,lon,sats,flags
```

`flags` is hex: bit 0 baro, 1 IMU, 2 magnetometer, 3 GPS fix, 4 SD card,
5 radio, 6 MAIN continuity, 7 BACKUP continuity. Latitude/longitude keep the
last known fix when the GPS loses lock.

## Layout

```
lib/board     VadaPins.h: pin map for both Picos (from the PCB)
lib/flight    AltitudeFilter, BaroTracker, FlightComputer (pure C++)
lib/gps       NmeaParser: GGA/RMC, any talker, checksum-validated (pure C++)
lib/sensors   BMP390, ICM-20649, LIS3MDL drivers, SensorWatch
lib/pyro      PyroChannel: continuity test and non-blocking firing
lib/logging   LogBuffer (non-blocking cross-core log), log file naming
src/pico_a    Pico A application
src/pico_b    Pico B application
test/         host unit tests
tools/        MicroPython bring-up script for Pico B (not flight code)
```

## Hardware notes from the PCB review

These come from reading `vada/VADA.kicad_pcb`; check them on the bench.

- **Pyro SSR drive current looks far too low.** Each TRIG line reaches its
  CPC1020N LED through 33 kΩ (R12/R21/R22/R23) with another 33 kΩ
  (R18/R19/R13/R20) across the LED. From a 3.3 V GPIO that leaves roughly
  10–40 µA of LED current, while the CPC1020N needs milliamps to switch on.
  If those values are what is fitted, the channels may never fire.
  **Bench-test every channel with an e-match substitute before flying.**
- The barometer footprints are `BMP-390L` but their schematic values say
  `BMP585_A/B`. The firmware expects a BMP390 (chip ID 0x60).
- The Pico-to-Pico link is UART0 (GPIO0/1, crossed), not I2C. The firmware
  does not use it yet.
- J4 "ConnCheck" is a loop between Pico A GPIO11 and GPIO12. Pico A logs its
  state at boot; nothing depends on it.
