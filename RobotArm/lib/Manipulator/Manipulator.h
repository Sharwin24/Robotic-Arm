#ifndef Manipulator_h
#define Manipulator_h
#include <Arduino.h>
#include <LinkMotor.h>
#include <Position.h>
#include <JointAngles.h>
#include <Utils.h>

// TODO: Wire Diagram

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

// TODO: Docs

class Manipulator {
   private:
    int numLinks;
    LinkMotor Link1 = LinkMotor(1, link1StepPin, link1DirPin, link1LimitSwitchPin, 0.2);
    LinkMotor Link2 = LinkMotor(2, link2StepPin, link2DirPin, link2LimitSwitchPin, 0.2);
    LinkMotor Link3 = LinkMotor(3, link3StepPin, link3DirPin, link3LimitSwitchPin);
    float link1Length;
    float link2Length;
    float link3Length;
    Position EEPos;

   public:
    Manipulator() {
        Manipulator(L1, L2, L3);
    }
    Manipulator(float _link1Length, float _link2Length, float _link3Length) {
        numLinks = 3;
        link1Length = _link1Length;
        link2Length = _link2Length;
        link3Length = _link3Length;
        EEPos = ForwardKinematics(0, 0, 0);
    }

    void init() {
        Link1.init();
        Link2.init();
        Link3.init();
        Link1.calibrate();
        Link2.calibrate();
        Link3.calibrate();
    }

    Position getEEPos() {
        return EEPos;
    }

    // Math Functions
    Position ForwardKinematics(float q1, float q2, float q3);

    JointAngles InverseKinematics(float xTarget, float yTarget);

    // Movement Functions
    void moveToAngles(float q1, float q2, float q3);

    void moveToJointAngles(JointAngles jointAngleTargets);

    void moveToXY(float xTarget, float yTarget);

    void moveToPosition(Position positionTarget);

    void updateLinks();

    // Individual Link functions
    void link1ToAngle(float degrees);

    void link2ToAngle(float degrees);

    void link3ToAngle(float degrees);
};
#endif