#ifndef Utils_h
#define Utils_h
#include <Arduino.h>
void initSerialMonitor();

long getDelayFromSpeed(long s);

void waitForSerialInput();

void print(String msg);

// Steps per revolution (set on driver)
// Steps per revolution (SPR) -> angle (Degrees)
// SPR / 360 = steps per degree
int degreeToSteps(float degree);

#endif