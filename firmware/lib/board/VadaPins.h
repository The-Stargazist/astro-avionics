#pragma once

#include <stdint.h>

// VADA board pin map, read from the netlist in vada/VADA.kicad_pcb.
// Each PlatformIO environment defines exactly one of BOARD_PICO_A / BOARD_PICO_B.
//
// I2C rails and solder jumpers:
//   Rail A: ICM-20649, LIS3MDL, BMP390 "A". Native to Pico A GPIO26/27 (I2C1),
//           reaches Pico B GPIO10/11 (I2C1) through JP1/JP2.
//   Rail B: BMP390 "B". Native to Pico B GPIO20/21 (I2C0),
//           reaches Pico A GPIO20/21 (I2C0) through JP3/JP4.
// This firmware expects all four jumpers bridged, with Pico B mastering rail A
// and Pico A mastering rail B. Each board leaves the other rail's pins as
// inputs, so there is never more than one master per rail and each flight
// computer has its own barometer.
//
// Pyro channels (connector, SSR):
//   MAIN   J2 "Pyro 3", U10    BACKUP J3 "Pyro 4", U11   (both Picos)
//   MOTOR  J5 "Pyro 1", U13    STAGE  J6 "Pyro 2", U12   (Pico A only)

namespace pins {

#if defined(BOARD_PICO_A)

constexpr uint8_t I2C_SDA = 20;  // rail B via JP3/JP4
constexpr uint8_t I2C_SCL = 21;

constexpr uint8_t SPI_MISO = 16;  // SPI0, SD card "SD_A"
constexpr uint8_t SPI_CS_SD = 17;
constexpr uint8_t SPI_SCK = 18;
constexpr uint8_t SPI_MOSI = 19;

constexpr uint8_t STATUS_LED = 22;

constexpr uint8_t LINK_TX = 0;  // UART0 to Pico B (unused)
constexpr uint8_t LINK_RX = 1;

constexpr uint8_t MOTOR_TRIG = 2;
constexpr uint8_t MOTOR_CONT = 3;
constexpr uint8_t MOTOR_SENSE = 4;
constexpr uint8_t STAGE_TRIG = 5;
constexpr uint8_t STAGE_CONT = 6;
constexpr uint8_t STAGE_SENSE = 7;
constexpr uint8_t MAIN_TRIG = 8;
constexpr uint8_t MAIN_CONT = 9;
constexpr uint8_t MAIN_SENSE = 10;
constexpr uint8_t CHECK_CONT = 11;  // J4 "ConnCheck" loop
constexpr uint8_t CHECK_SENSE = 12;
constexpr uint8_t BACKUP_CONT = 13;
constexpr uint8_t BACKUP_TRIG = 14;
constexpr uint8_t BACKUP_SENSE = 15;

#elif defined(BOARD_PICO_B)

constexpr uint8_t I2C_SDA = 10;  // I2C1, rail A via JP1/JP2
constexpr uint8_t I2C_SCL = 11;

constexpr uint8_t SPI_SCK = 2;  // SPI0, shared by LoRa and SD card "SD_B"
constexpr uint8_t SPI_MOSI = 3;
constexpr uint8_t SPI_MISO = 4;
constexpr uint8_t SPI_CS_LORA = 5;
constexpr uint8_t LORA_RESET = 6;
constexpr uint8_t LORA_DIO0 = 7;
constexpr uint8_t SPI_CS_SD = 26;

// UART1. Board nets: GPIO8 -> GPS RX pad, GPIO9 <- GPS TX pad (verified on the PCB).
constexpr uint8_t GPS_UART_TX = 8;
constexpr uint8_t GPS_UART_RX = 9;

constexpr uint8_t STATUS_LED = 18;

constexpr uint8_t LINK_TX = 0;  // UART0 to Pico A (unused)
constexpr uint8_t LINK_RX = 1;

constexpr uint8_t MAIN_SENSE = 13;  // shared with Pico A GPIO10
constexpr uint8_t MAIN_TRIG = 14;
constexpr uint8_t MAIN_CONT = 15;
constexpr uint8_t BACKUP_TRIG = 16;
constexpr uint8_t BACKUP_SENSE = 17;  // shared with Pico A GPIO15
constexpr uint8_t BACKUP_CONT = 22;

#else
#error "Define BOARD_PICO_A or BOARD_PICO_B (see platformio.ini)"
#endif

}  // namespace pins
