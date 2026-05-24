#pragma once

#include <Arduino.h>

/*
PICO A
MMAIN_CONT  9   mainPower
MMAIN_SENS  10  mainDetect
MMAIN_TRIG  8

MBAK_CONT   13  backupPower
MBAK_SENS   15  backupDetect
MBAK_TRIG   14

MMOTOR_CONT 3
MMOTOR_SENS 4
MMOTOR_IGN  2

MCHECK_CONT 11
MCHECK_SENS 12


PICO B
SMAIN_CONT  15  mainPower
SMAIN_SENS  13  mainDetect
SMAIN_TRIG  14

SBAK_CONT   22  backupPower
SBAK_SENS   17  backupDetect
SBAK_TRIG   16 
*/

// PICO A - definitions (uncomment the one that is being used for compilation)

constexpr int mainPower   = 9;
constexpr int mainDetect  = 10;
constexpr int backupPower = 13;
constexpr int backupDetect = 15;

// PICO B - definitions

// constexpr int mainPower   = 15;
// constexpr int mainDetect  = 13;
// constexpr int backupPower = 22;
// constexpr int backupDetect = 17;


void contSetup();

bool getContinuityStatus(int powerPin, int detectPin); // Returns bool indicating cont

bool contCheckMain();
bool contCheckBackup();