#include <Arduino.h>
#include <Utils.h>
#include <math.h>

const float stepsPerRev = 800.0;

long getDelayFromSpeed(long s) {
    // Converting ticks per second into a delay timing
    long microseconds = 1000000 / s;
    return microseconds;
}

void waitForSerialInput() {
    while (Serial.available() == 0) {
        delay(50);
    }
}

void writeMsg(String msg) {
    Serial.print("MSG: ");
    Serial.println(msg);
}

int degreeToSteps(float degree) {
    // spr / 360 = steps per degree * degree
    return floorf((stepsPerRev / 360.0) * degree);
}