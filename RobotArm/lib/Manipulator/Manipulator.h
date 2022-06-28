#ifndef Manipulator_h
#define Manipulator_h
#include <Arduino.h>
#include <LinkMotor.h>
#include <Gripper.h>
#include <Position.h>
#include <JointAngles.h>
#include <Utils.h>

// TODO: Docs

class Manipulator {
   private:
    int numLinks;
    LinkMotor Link1 = LinkMotor(1, link1StepPin, link1DirPin, link1LimitSwitchPin, 5.0);
    LinkMotor Link2 = LinkMotor(2, link2StepPin, link2DirPin, link2LimitSwitchPin, 5.0);
    LinkMotor Link3 = LinkMotor(3, link3StepPin, link3DirPin, link3LimitSwitchPin);
    Gripper EE = Gripper(servoLPin, servoRPin);
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
        EE.init();
    }

    void closeEE() {
        EE.close();
    }

    void openEE() {
        EE.open();
    }

    Position getEEPos() {
        return EEPos;
    }

    void updateEEPos(float q1, float q2, float q3) {
        EEPos = ForwardKinematics(q1, q2, q3);
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
    void setLink1Target(int targetStep);
    void setLink2Target(int targetStep);
    void setLink3Target(int targetStep);

    void link1ToAngle(float degrees);
    void link2ToAngle(float degrees);
    void link3ToAngle(float degrees);

    float getLink1GR();
    float getLink2GR();
    float getLink3GR();

    // Joint position methods
    Position getJoint1Position();
    Position getJoint2Position();
    Position getJoint3Position();
};
#endif