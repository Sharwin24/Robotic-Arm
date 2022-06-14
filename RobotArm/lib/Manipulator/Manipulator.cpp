#include <Manipulator.h>
#include <Position.h>
#include <JointAngles.h>
#include <math.h>

// Math Functions
Position Manipulator::ForwardKinematics(float q1, float q2, float q3) {
    return Position(0, 0);
}

JointAngles Manipulator::InverseKinematics(float xTarget, float yTarget) {
    return JointAngles(0, 0, 0);
}

// Movement Functions
void Manipulator::moveToAngles(float q1, float q2, float q3) {
}

void Manipulator::moveToJointAngles(JointAngles jointAngleTargets) {
    moveToAngles(jointAngleTargets.q1, jointAngleTargets.q2, jointAngleTargets.q3);
}

void Manipulator::moveToXY(float xTarget, float yTarget) {
}

void Manipulator::moveToPosition(Position positionTarget) {
    moveToXY(positionTarget.x, positionTarget.y);
}

// Individual Link functions
void Manipulator::link1ToAngle(float degrees) {
    Link1.jointAngle(degrees);
}

void Manipulator::link2ToAngle(float degrees) {
    Link2.jointAngle(degrees);
}

void Manipulator::link3ToAngle(float degrees) {
    Link3.jointAngle(degrees);
}