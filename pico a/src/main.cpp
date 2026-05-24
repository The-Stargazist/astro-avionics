#include <Arduino.h>
#include "continuity.h"
#include "BMP.h"
#include "sdlogger.h"

// Pindefs
#define STATUS_LED 22
#define MAIN_TRIG 8
#define BAK_TRIG 14
#define SDA0 20
#define SCL0 21

// Constants
#define VEL_THRES 2
#define DES_THRES 20
#define LAN_THRES 2
#define FLIGHTMODE_ALT 200
#define BMP_DELAY 20

// BMP def
BMP bmp(Wire)

// Flightmode enum def
enum class FlightState {
    GROUND,
    FLIGHT,
    DESCENT,
    LANDED
};

volatile FlightState flightState = FlightState::GROUND;

// Shared data def
volatile float altitude = 0;
volatile float velocity = 0;
volatile bool mcont = false;
volatile bool bcont = false;
volatile bool cont = false;

inline void groundSwitch(){
    // Check if height is less than 200m relative otherwise stay in this mode continue logging
    // Switch to FLIGHT if rel height > 200
    // Store the millis at which we switched to FLIGHT
    bmp.read();
    altitude = bmp.getRelativeAltitude();
    velocity = bmp.getVelocity();
    logger.logSample();

    if(altitude > FLIGHTMODE_ALT){
        flightState = FlightState::FLIGHT;
        logger.logString("Altitude > 200m! Switching to FLIGHT MODE");
    }

    delay(BMP_DELAY);
}

inline void flightSwitch(){
    // Check if apogee is being reached ie. velocity < 0 for 5 consecutive readings, while logging
    // Deploy main chute at apogee
    // Switch to DESCENT if apogee is reached
    static int fcounter = 0;

    bmp.read();
    altitude = bmp.getRelativeAltitude();
    velocity = bmp.getVelocity();
    logger.logSample();

    velocity < VEL_THRES ? fcounter++ : (fcounter > 0 ? fcounter-- : 0);
    
    if(fcounter > 5){
        logger.logString("Apogee detected! Deploying main parachute!");
        digitalWrite(MAIN_TRIG, HIGH);

        flightState = FlightState::DESCENT;
        logger.logString("Switching to DESCENT MODE");

        delay(500);
        digitalWrite(MAIN_TRIG, LOW);
    }

    delay(BMP_DELAY);
}

inline void descentSwitch(){
    // Check if descent rate < DESC_THRESH, if thats the case deploy backup chute
    // continue logging
    // if after X millis since switch to FLIGHT or velocity has been < 2m/s for a few mins
    // stop logging and close the file
    static int dcounter = 0;
    static int vcounter = 0;

    bmp.read();
    altitude = bmp.getRelativeAltitude();
    velocity = bmp.getVelocity();
    logger.logSample();

    velocity < DES_THRES ? dcounter++ : (dcounter > 0 ? dcounter-- : 0);
    velocity < LAN_THRES ? vcounter++ : (vcounter > 0 ? vcounter-- : 0);

    if(dcounter > 5){
        logger.logString("Overspeed detected! Deploying backup parachute!");

        digitalWrite(BAK_TRIG, HIGH); // Deploy backup with a longer relay on time
        delay(1000);
        digitalWrite(BAK_TRIG, LOW);

        digitalWrite(MAIN_TRIG, HIGH); // Try main chute again with longer relay on time
        delay(1000);
        digitalWrite(MAIN_TRIG, LOW);

        dcounter = 0;
    }

    if(vcounter > 1000){ // Check either flight time or 20 seconds of no descent
        flightState = FlightState::LANDED;
        logger.logString("Landed! Shutting down logging");
        logger.stop();
    }

    delay(BMP_DELAY);
}
void setup(){
    // init sd card
    // init bmp
    // check continuity and other stuff as well before starting
    // maybe define status led such that it only blinks if continuity worked and its actually logging
    // and it stays static on if cont failed
    // but dont make it not go into flight logic if cont failed, eggtimer also has cont checks

    Wire.setSDA(SDA0);
    Wire.setSCL(SCL0);
    Wire.begin();
    Wire.setClock(400000);

    contSetup();
    pinMode(MAIN_TRIG, OUTPUT);
    pinMode(BAK_TRIG, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);

    logger.begin();
    
    mcont = contCheckMain();
    bcont = contCheckBackup();
    logger.logContinuity(mcont, bcont);
    if(mcont && bcont){
        cont = true;
    }

    if(!cont){  // Set STATUS_LED to high if there is no continuity and dont blink
        digitalWrite(STATUS_LED, HIGH);
    }

}

void loop(){ // flight logic (core 0)
    static int ledCounter = 0;
    static bool ledState = false;

    if(ledCounter > 25){ // blink only if there is continuity
        if(cont){
            ledState = !ledState;
            digitalWrite(STATUS_LED, ledState);
        }

        ledCounter = 0;
    }

    ledCounter++;

    switch(flightState){

        case FlightState::GROUND:
        groundSwitch();
        break;

        case FlightState::FLIGHT:
        flightSwitch();
        break;

        case FlightState::DESCENT:
        descentSwitch();
        break;

        case FlightState::LANDED:
        delay(1000);
        break;
    }
}

void setup1(){ // core 1 setup

}

void loop1(){ // SD card write (core 1)

    logger.flush();

    delay(200);
}