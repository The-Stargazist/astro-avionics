#include "bmp_lib.h"

#define SEALEVELPRESSURE_HPA 1013.25

/*
Example implementation in main.cpp

// Start the wire for all the sensors on an i2c line
Wire.setSDA(SDA0); // Wire1.setSDA(SDA1) for i2c_1
Wire.setSCL(SCL0);
Wire.begin();
Wire.setClock(400000); // check if system is stable in 400Khz, Available modes: 100Khz, 400Khz, 1000Khz

// Create object (Use objects for different i2c line bmps)
BMP bmp1;

// Initialize the bmp
bmp1.begin(&Wire);
*/

bool BMP::begin(TwoWire* bus){
    if(!bmp.begin_I2C(0x77, bus)){
        return false;
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);  // Operating at 50Hz

    if(!bmp.performReading()) return false;
    groundalt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    lastalt = groundalt;
    lasttime = millis();

    return true;
}

bool BMP::getAltitude(float &altitude){  // WARNING: returns absolute ALTITUDE! 
    if(!bmp.performReading()) return false; // use groundalt + flightmode_alt to check for liftoff
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);  // PILANI GROUNDALT ~ 300m
    return true;
}

bool BMP::getVelnAlt(float &velocity, float &altitude){
    if(!bmp.performReading()) return false;
    
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);  // WARNING: returns absolute ALTITUDE!
    unsigned long time = millis();  // use groundalt + flightmode_alt to check for liftoff
    if((time - lasttime) < 20) return false;  // PILANI GROUNDALT ~ 300m

    velocity = (altitude - lastalt)*1000.0f / (time - lasttime);
    lastalt = altitude;
    lasttime = time;
    return true;
}
