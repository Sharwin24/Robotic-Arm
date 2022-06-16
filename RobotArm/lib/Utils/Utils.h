#ifndef Utils_h
#define Utils_h
#include <Arduino.h>
#include <Position.h>
#include <Vector.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>

const int stepsPerRev = 800;

void initSerialMonitor();

long getDelayFromSpeed(long s);

// Steps per revolution (set on driver)
// Steps per revolution (SPR) -> angle (Degrees)
// SPR / 360 = steps per degree
float degreeToSteps(float degree);

RotationMatrix multiply(RotationMatrix R1, RotationMatrix R2);

RotationMatrix transpose(RotationMatrix R);

HomogeneousTransform multiply(HomogeneousTransform A1, HomogeneousTransform A2);

HomogeneousTransform transpose(HomogeneousTransform A);

#endif