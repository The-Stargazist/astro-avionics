# Code for Pico B acting as only MASTER

This code is for PICO B acting as the sole MASTER on VADA PCB.

THE I2C Rail A is shorted to give Pico B access to all the I2C devices on the PCB. and PICO A merely acts as a slave to PICO B for continuity check.

Pico B is connected to lora and sdcard via spi, GPS via uart.

The I2C rail A contains the following devices:

- BMP390 (Barometeric Altimeter)
- ICM20649 (IMU)
- LIS3DML (Magnetometer)
- PICO A as slave for continuity check and redundant loggig to SD card

The I2C rail B contains the following devices:

- BMP390 (Barometeric Altimeter)

## Functions of the code

Primary function of code is to deploy parachute at right altitude.

The Secondary function for the code is to get readings from all the sensors and log them to the SD card. It also has to send the data to PICO A for redundancy in datalogging and continuity check. The code also has to send the data to lora for transmission.


i also want to implement a sd card so can you add a global stack of logs which we keep adding things to and sd card write happens every 10hz