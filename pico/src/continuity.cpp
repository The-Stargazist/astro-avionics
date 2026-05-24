#include "continuity.h"

void contSetup() {
    pinMode(mainPower, OUTPUT);
    pinMode(mainDetect, INPUT);

    pinMode(backupPower, OUTPUT);
    pinMode(backupDetect, INPUT);

    digitalWrite(mainPower, LOW);
    digitalWrite(backupPower, LOW);
}

bool getContinuityStatus(int powerPin, int detectPin) { // Returns bool indicating cont
    digitalWrite(powerPin, HIGH);
    delay(10);

    bool status = digitalRead(detectPin) == LOW;

    digitalWrite(powerPin, LOW);

    return status;
}

bool contCheckMain() {
    return getContinuityStatus(mainPower, mainDetect);
}

bool contCheckBackup() {
    return getContinuityStatus(backupPower, backupDetect);
}