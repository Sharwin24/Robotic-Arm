#ifndef Manipulator_h
#define Manipulator_h
#include <Arduino.h>
#include <LinkMotor.h>
#include <Utils.h>
#include <Position.h>
#include <JointAngles.h>

// TODO: Wire Diagram

// Link 1
const int L1 = 125;  // length is in mm
const int link1DirPin = 3;
const int link1StepPin = 2;
const int link1LimitSwitchPin = 4;
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

// TODO: Docs

class Manipulator {
   private:
    int numLinks;
    LinkMotor Link1 = LinkMotor(1, link1StepPin, link1DirPin, link1LimitSwitchPin);
    LinkMotor Link2 = LinkMotor(2, link2StepPin, link2DirPin, link2LimitSwitchPin);
    LinkMotor Link3 = LinkMotor(3, link3StepPin, link3DirPin, link3LimitSwitchPin);
    float link1Length;
    float link2Length;
    float link3Length;
    // LinkMotor links[3]; // FIXME: Build issue

   public:
    Manipulator() {
        // Default Constructor uses default link lengths
        Manipulator(L1, L2, L3);
    }
    Manipulator(float _link1Length, float _link2Length, float _link3Length) {
        Serial.println("Initializing 3-Link Manipulator");
        numLinks = 3;
        link1Length = _link1Length;
        link2Length = _link2Length;
        link3Length = _link3Length;
        // LinkMotor links[] = {Link1, Link2, Link3};
        Serial.println("Initializing Motors");
        Link1.init();
        Link2.init();
        Link3.init();
        Serial.println("Calibrating Motors");
        Link1.calibrate();
        Link2.calibrate();
        Link3.calibrate();
        Serial.println("Manipulator Setup Complete");
        delay(500);
    }

    // Math Functions
    Position ForwardKinematics(float q1, float q2, float q3);

    JointAngles InverseKinematics(float xTarget, float yTarget);

    // Movement Functions
    void moveToAngles(float q1, float q2, float q3);

    void moveToJointAngles(JointAngles jointAngleTargets);

    void moveToXY(float xTarget, float yTarget);

    void moveToPosition(Position positionTarget);

    // Individual Link functions
    void link1ToAngle(float degrees);

    void link2ToAngle(float degrees);

    void link3ToAngle(float degrees);
};
#endif