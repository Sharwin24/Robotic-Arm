#include <Manipulator.h>
#include <Position.h>
#include <JointAngles.h>
#include <math.h>
#include <Vector.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>
// #include <Eigen.h> // Build issues

// Math Functions
Position Manipulator::ForwardKinematics(float q1, float q2, float q3) {
    // Obtaining angles in radians
    float q1R = radians(q1);
    float q2R = radians(q2);
    float q3R = radians(q3);
    // Frame 0 -> 1
    Vector r01 = Vector(link1Length * cosf(q1R), link1Length * sinf(q1R), 0.0);
    RotationMatrix R01 = RotationMatrix('z', q1R);
    HomogeneousTransform A01 = HomogeneousTransform(R01, r01);
    // Frame 1 -> 2
    Vector r12 = Vector(link2Length * cosf(q2R), link2Length * sinf(q2R), 0.0);
    RotationMatrix R12 = RotationMatrix('z', q2R);
    HomogeneousTransform A12 = HomogeneousTransform(R12, r12);
    // Frame 2 -> 3
    Vector r23 = Vector(link3Length * cosf(q3R), link3Length * sinf(q3R), 0.0);
    RotationMatrix R23 = RotationMatrix('z', q3R);
    HomogeneousTransform A23 = HomogeneousTransform(R23, r23);
    // Frame 3 -> EE
    Vector r3e = Vector(endEffectorLength, 0.0, 0.0);
    float R3e0[] = {0.0, 0.0, 1.0};
    float R3e1[] = {0.0, -1.0, 0.0};
    float R3e2[] = {1.0, 0.0, 0.0};
    RotationMatrix R3e = RotationMatrix(R3e0, R3e1, R3e2);
    HomogeneousTransform A3e = HomogeneousTransform(R3e, r3e);

    // Find Transformation
    HomogeneousTransform A02 = multiply(A01, A12);
    HomogeneousTransform A2e = multiply(A23, A3e);
    HomogeneousTransform A0e = multiply(A02, A2e);

    return Position(A0e.get(0, 3), A0e.get(1, 3));
}

JointAngles Manipulator::InverseKinematics(float xTarget, float yTarget) {
    float ell = sqrtf(sq(xTarget) + sq(yTarget));
    float q2D = -acosf(
        (sq(ell) - sq(link1Length) - sq(link2Length)) /
        (2 * link1Length * link2Length));
    float beta = acosf(
        (sq(ell) + sq(link1Length) - sq(link2Length)) /
        (2 * ell * link1Length));
    float alpha = atan2f(yTarget, xTarget);
    float q1D = alpha + beta;  // + for elbow up, - for elbow down
    float phi = 270.0;
    float q3D = phi - q1D - q2D;
    return JointAngles(q1D, q2D, q3D);
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