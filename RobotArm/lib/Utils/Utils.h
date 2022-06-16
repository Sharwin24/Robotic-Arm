#ifndef Utils_h
#define Utils_h
#include <Arduino.h>
#include <Position.h>
#include <Vector.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>

const int stepsPerRev = 6400;
const float RPM = 75.0;  // Link Speed

void initSerialMonitor();

long getDelayFromSpeed(float s);

float degreeToSteps(float degree);

RotationMatrix multiply(RotationMatrix R1, RotationMatrix R2);

RotationMatrix transpose(RotationMatrix R);

HomogeneousTransform multiply(HomogeneousTransform A1, HomogeneousTransform A2);

HomogeneousTransform transpose(HomogeneousTransform A);

#endif