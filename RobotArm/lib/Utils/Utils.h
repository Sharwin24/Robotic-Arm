#ifndef Utils_h
#define Utils_h
#include <Arduino.h>
#include <Position.h>
#include <Vector.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>

// Change Constants here
const float stepsPerRev = 8000.0;  // Set according to motor-driver
const float RPM = 75.0;            // Link Speed
const int DECIMALPRECISION = 4;    // Decimal Precision when printing
// Link 1
const int L1 = 125;  // length is in mm
const int link1StepPin = 2;
const int link1DirPin = 3;
const int link1LimitSwitchPin = -1;  // 8
// Link 2
const int L2 = 100;
const int link2StepPin = 4;
const int link2DirPin = 5;
const int link2LimitSwitchPin = -1;  // 12
// Link 3
const int L3 = 40;
const int link3StepPin = 6;
const int link3DirPin = 7;
const int link3LimitSwitchPin = -1;  // 13
// End-Effector
const int endEffectorLength = 10;
const int servoLPin = -1;  // TODO: Wire servo motors
const int servoRPin = -1;

void initSerialMonitor();

long getDelayFromSpeed(float s);

float degreeToSteps(float degree);

void printLink(int linkNumber, float q1);
void printLink1Link2(float q1, float q2);
void printLink1Link2Link3(float q1, float q2, float q3);

// Matrix Methods
RotationMatrix multiply(RotationMatrix R1, RotationMatrix R2);

RotationMatrix transpose(RotationMatrix R);

void printMatrix(RotationMatrix R);

HomogeneousTransform multiply(HomogeneousTransform A1, HomogeneousTransform A2);

HomogeneousTransform transpose(HomogeneousTransform A);

void printMatrix(HomogeneousTransform A);

// Joint position methods
float getJoint2Height();
float getJoint3Height();

#endif