#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

class BMP{
    public:
    bool begin(TwoWire* bus);
    bool getAltitude(float &altitude);
    bool getVelnAlt(float &velocity, float &altitude);
    float groundalt = 0;

    private:
    Adafruit_BMP3XX bmp;
    float lastalt = 0;
    unsigned long lasttime = 0;
};
