#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

/*
Usage Example:

// Setup I2C
Wire.setSDA(SDA0);
Wire.setSCL(SCL0);
Wire.begin();
Wire.setClock(400000); // 100kHz / 400kHz / 1MHz

// Create object
BMP bmp(Wire);

// Initialize
if (!bmp.begin()) {
    // handle error
}

// In loop / task
bmp.read();

float alt = bmp.getRelativeAltitude();
float vel = bmp.getVelocity();
*/

class BMP {
public:
    BMP(TwoWire& wire, uint8_t addr = 0x77);

    bool begin();     // Initializes sensor + sets ground altitude (call once)
    void read();      // Call as fast as you want (internally limited to ~50Hz)

    void setSeaLevelPressure(float p);  // Set this before begin() for better accuracy

    float getTemperature();       // °C
    float getPressure();          // hPa
    float getAltitude();          // Absolute altitude (WARNING: not ground-relative)
    float getRelativeAltitude();  // Altitude above launch point
    float getVelocity();          // m/s (filtered)

private:
    TwoWire* _wire;
    uint8_t _addr;

    Adafruit_BMP3XX bmp;

    float seaLevelPressure = 1013.25;  // Change based on weather for accuracy

    float temperature = 0;
    float pressure = 1013.25;

    float altitude = 0;
    float groundAltitude = 0;

    float velocity = 0;
    float lastAltitude = 0;

    unsigned long lastTime = 0;
};