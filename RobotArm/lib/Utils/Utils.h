#ifndef Utils_h
#define Utils_h
#include <Arduino.h>
#include <Position.h>
#include <Vector.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>

// Change Constants here
const int stepsPerRev = 6400;  // Set according to motor-driver
const float RPM = 75.0;        // Link Speed
// Link 1
const int L1 = 125;  // length is in mm
const int link1DirPin = 3;
const int link1StepPin = 2;
const int link1LimitSwitchPin = -1;
// Link 2
const int L2 = 100;
const int link2DirPin = 5;
const int link2StepPin = 6;
const int link2LimitSwitchPin = -1;
// Link 3
const int L3 = 40;
const int link3DirPin = 7;
const int link3StepPin = 8;
const int link3LimitSwitchPin = -1;
// End-Effector
const int endEffectorLength = 10;

void initSerialMonitor();

long getDelayFromSpeed(float s);

float degreeToSteps(float degree);

RotationMatrix multiply(RotationMatrix R1, RotationMatrix R2);

RotationMatrix transpose(RotationMatrix R);

HomogeneousTransform multiply(HomogeneousTransform A1, HomogeneousTransform A2);

HomogeneousTransform transpose(HomogeneousTransform A);

#endif