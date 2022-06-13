#include <Arduino.h>
#include <Utils.h>

long getDelayFromSpeed(long s) {
    // Here we are converting ticks per second into a delay timing
    // ticks / second *
    // writeInfo(String(s));
    long microseconds = 1000000 / s;
    // writeInfo(String(microseconds));
    // writeDebug("Speed: " + String(microseconds) + " Delay");
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