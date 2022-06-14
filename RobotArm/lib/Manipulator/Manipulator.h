#ifndef Manipulator_h
#include <Arduino.h>
#include <LinkMotor.h>
#include <Utils.h>
#include <Position.h>
#include <JointAngles.h>
#define Manipulator_h

// TODO: Docs

class Manipulator {
   private:
    int numLinks;

   public:
    Manipulator() {
        writeMsg("Initializing 3-Link Manipulator");
        // Link 1
        const int link1DirPin = 3;
        const int link1StepPin = 2;
        const int link1LimitSwitchPin = -1;
        LinkMotor Link1 = LinkMotor(1, 17, link1StepPin, link1DirPin, link1LimitSwitchPin);
        // Link 2
        const int link2DirPin = -1;
        const int link2StepPin = -1;
        LinkMotor Link2 = LinkMotor(2, 14, link2StepPin, link2DirPin, -1);
        // Link 3
        const int link3DirPin = -1;
        const int link3StepPin = -1;
        LinkMotor Link3 = LinkMotor(3, 11, link3StepPin, link3DirPin, -1);
        LinkMotor links[] = {Link1, Link2, Link3};
        numLinks = sizeof(links) / sizeof(LinkMotor);
        writeMsg("Initializing Motors");
        Link1.init();
        Link2.init();
        Link3.init();
        writeMsg("Calibrating Motors");
        Link1.calibrate();
        Link2.calibrate();
        Link3.calibrate();
        writeMsg("Manipulator Setup Complete");
        delay(500);
    }

    // Math Functions
    Position ForwardKinematics(float link1Angle, float link2Angle, float link3Angle);

    JointAngles InverseKinematics(float xTarget, float yTarget);

    // Movement Functions
    void moveToAngles(float link1Angle, float link2Angle, float link3Angle);

    void moveToJointAngles(JointAngles jointAngleTargets);

    void moveToXY(float xTarget, float yTarget);

    void moveToPosition(Position positionTarget);
};
#endif