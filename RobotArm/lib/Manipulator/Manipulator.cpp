#include <Manipulator.h>
#include <Position.h>
#include <JointAngles.h>

// Math Functions
Position Manipulator::ForwardKinematics(float link1Angle, float link2Angle, float link3Angle) {
    return Position(0, 0);
}

JointAngles Manipulator::InverseKinematics(float xTarget, float yTarget) {
    return JointAngles(0, 0, 0);
}

// Movement Functions
void Manipulator::moveToAngles(float link1Angle, float link2Angle, float link3Angle) {
}

void Manipulator::moveToJointAngles(JointAngles jointAngleTargets);

void Manipulator::moveToXY(float xTarget, float yTarget) {
}

void Manipulator::moveToPosition(Position positionTarget) {
}