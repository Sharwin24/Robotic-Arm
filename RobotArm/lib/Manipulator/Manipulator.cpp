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
    RotationMatrix R01 = RotationMatrix('z', q1);
    HomogeneousTransform A01 = HomogeneousTransform(R01, r01);
    // Frame 1 -> 2
    Vector r12 = Vector(link2Length * cosf(q2R), link2Length * sinf(q2R), 0.0);
    RotationMatrix R12 = RotationMatrix('z', q2);
    HomogeneousTransform A12 = HomogeneousTransform(R12, r12);
    // Frame 2 -> 3
    Vector r23 = Vector(link3Length * cosf(q3R), link3Length * sinf(q3R), 0.0);
    RotationMatrix R23 = RotationMatrix('z', q3);
    HomogeneousTransform A23 = HomogeneousTransform(R23, r23);
    // Frame 3 -> EE
    Vector r3e = Vector(endEffectorLength, 0.0, 0.0);
    float R3eMatrix[3][3] = {
        {0.0, 0.0, 1.0},
        {0.0, -1.0, 0.0},
        {1.0, 0.0, 0.0}};
    // Ry(90) -> Rz(180)
    RotationMatrix R3e = RotationMatrix(R3eMatrix);
    HomogeneousTransform A3e = HomogeneousTransform(R3e, r3e);

    // Find Transformation
    HomogeneousTransform A02 = multiply(A01, A12);
    HomogeneousTransform A2e = multiply(A23, A3e);
    HomogeneousTransform A0e = multiply(A02, A2e);
    return Position(A0e.get(0, 3), A0e.get(1, 3));
}

JointAngles Manipulator::InverseKinematics(float xTarget, float yTarget) {
    float ell = sqrt(sq(xTarget) + sq(yTarget));
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
    // Obtain target angles as steps
    int q1Steps = floorf(degreeToSteps(q1));
    int q2Steps = floorf(degreeToSteps(q2));
    int q3Steps = floorf(degreeToSteps(q3));
    // Set the target step for each Link
    Link1.setTarget(q1Steps);
    Link2.setTarget(q2Steps);
    Link3.setTarget(q3Steps);
    // Update all Links until target is acheived
    updateLinks();
    // Update EE pos
    updateEEPos(q1, q2, q3);
}

void Manipulator::moveToJointAngles(JointAngles jointAngleTargets) {
    moveToAngles(jointAngleTargets.q1, jointAngleTargets.q2, jointAngleTargets.q3);
}

void Manipulator::moveToXY(float xTarget, float yTarget) {
    JointAngles Q = InverseKinematics(xTarget, yTarget);
    moveToAngles(Q.q1, Q.q2, Q.q3);
}

void Manipulator::moveToPosition(Position positionTarget) {
    moveToXY(positionTarget.x, positionTarget.y);
}

void Manipulator::updateLinks() {
    // We need to move Link 1 first because
    // we obtain invalid positions when Link 1 is at
    // 0 (or low enough) and Link 2 needs to acheive a negative angle
    while (Link1.isMoving()) {
        Link1.update();
    }
    while (Link2.isMoving() || Link3.isMoving()) {
        Link2.update();
        Link3.update();
    }
}

// Individual Link functions
void Manipulator::link1ToAngle(float degrees) {
    Link1.moveToAngle(degrees);
    updateEEPos(degrees, Link2.getAngle(), Link3.getAngle());
}

void Manipulator::setLink1Target(int targetStep) {
    Link1.setTarget(targetStep);
}

void Manipulator::link2ToAngle(float degrees) {
    Link2.moveToAngle(degrees);
    updateEEPos(Link1.getAngle(), degrees, Link3.getAngle());
}

void Manipulator::setLink2Target(int targetStep) {
    Link2.setTarget(targetStep);
}

void Manipulator::link3ToAngle(float degrees) {
    Link3.moveToAngle(degrees);
    updateEEPos(Link1.getAngle(), Link2.getAngle(), degrees);
}

void Manipulator::setLink3Target(int targetStep) {
    Link3.setTarget(targetStep);
}