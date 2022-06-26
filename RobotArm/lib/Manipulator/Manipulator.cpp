#include <Manipulator.h>
#include <Position.h>
#include <JointAngles.h>
#include <math.h>
#include <Vector.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>

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
    float ell = sqrtf(sq(xTarget) + sq(yTarget));
    float q2R = -acosf(
        (sq(ell) - sq(link1Length) - sq(link2Length)) /
        (2 * link1Length * link2Length));
    float beta = acosf(
        (sq(ell) + sq(link1Length) - sq(link2Length)) /
        (2 * ell * link1Length));
    float alpha = atan2f(yTarget, xTarget);
    float q1R = alpha + beta;  // + for elbow up, - for elbow down
    float phi = 270.0;
    float q3R = phi - q1R - q2R;
    float q1D = degrees(q1R);
    float q2D = degrees(q2R);
    float q3D = degrees(q3R);
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
    // For all possible movement combinations, determine order to move joints
    // TODO: Test and Verify
    float q1 = Link1.getAngle();
    float q2 = Link2.getAngle();
    float q3 = Link3.getAngle();
    float t1 = Link1.getTarget();
    float t2 = Link2.getTarget();
    float t3 = Link3.getTarget();
    bool CW1 = q1 > t1;
    bool CW2 = q2 > t2;
    bool CW3 = q3 > t3;
    // CCW = 0, CW = 1
    if (CW1 && CW2 && CW3) {
        // 1 1 1
        while (Link1.isMoving() || Link2.isMoving() || Link3.isMoving()) {
            Link1.update();
            Link2.update();
            Link3.update();
        }
    } else if (CW1 && CW2 && !CW3) {
        // 1 1 0
        while (Link1.isMoving() || Link2.isMoving() || Link3.isMoving()) {
            Link1.update();
            Link2.update();
            Link3.update();
        }
    } else if (!CW1 && CW2 && CW3) {
        // 0 1 1
        while (Link1.isMoving()) {
            Link1.update();
        }
        while (Link2.isMoving() || Link3.isMoving()) {
            Link2.update();
            Link3.update();
        }
    } else if (CW1 && !CW2 && !CW3) {
        // 1 0 0
        while (Link2.isMoving() || Link3.isMoving()) {
            Link2.update();
            Link3.update();
        }
        while (Link1.isMoving()) {
            Link1.update();
        }
    } else if (!CW1 && CW2 && !CW3) {
        // 0 1 0
        while (Link2.isMoving() || Link3.isMoving()) {
            Link2.update();
            Link3.update();
        }
        while (Link1.isMoving()) {
            Link1.update();
        }
    } else if (!CW1 && !CW2 && CW3) {
        // 0 0 1
        while (Link1.isMoving() || Link2.isMoving()) {
            Link1.update();
            Link2.update();
        }
        while (Link3.isMoving()) {
            Link3.update();
        }
    } else if (CW1 && !CW2 && CW3) {
        // 1 0 1
        while (Link1.isMoving()) {
            Link1.update();
        }
        while (Link2.isMoving() || Link3.isMoving()) {
            Link2.update();
            Link3.update();
        }
    } else {  // 0 0 0
        while (Link1.isMoving() || Link2.isMoving() || Link3.isMoving()) {
            Link1.update();
            Link2.update();
            Link3.update();
        }
    }
}

// Individual Link functions
void Manipulator::setLink1Target(int targetStep) {
    Link1.setTarget(targetStep);
}

void Manipulator::setLink2Target(int targetStep) {
    // For some reason, Link 2 always travels half its target
    // Math has been checked, Gear ratio has been checked,
    // SPR on driver has been checked
    // Potential Reasons: Nema 14 has a different steps setting?
    Link2.setTarget(targetStep * 2.0);  // !!Temporary Fix!!
}

void Manipulator::setLink3Target(int targetStep) {
    Link3.setTarget(targetStep);
}

void Manipulator::link1ToAngle(float degrees) {
    // Link1.moveToAngle(degrees);
    int steps = round(degreeToSteps(degrees));
    setLink1Target(steps);
    Link1.waitToStop();
    updateEEPos(degrees, Link2.getAngle(), Link3.getAngle());
}

void Manipulator::link2ToAngle(float degrees) {
    // Link2.moveToAngle(degrees);
    int steps = round(degreeToSteps(degrees));
    setLink2Target(steps);
    Link2.waitToStop();
    updateEEPos(Link1.getAngle(), degrees, Link3.getAngle());
}

void Manipulator::link3ToAngle(float degrees) {
    // Link3.moveToAngle(degrees);
    int steps = round(degreeToSteps(degrees));
    setLink3Target(steps);
    Link3.waitToStop();
    updateEEPos(Link1.getAngle(), Link2.getAngle(), degrees);
}

float Manipulator::getLink1GR() {
    return Link1.getGR();
}

float Manipulator::getLink2GR() {
    return Link2.getGR();
}

float Manipulator::getLink3GR() {
    return Link3.getGR();
}

// Joint position methods
float getJoint1Position() {
    float x = 0;
    float y = 0;
    return Position(x, y);
}
float getJoint2Position() {
    float x = L1 * sinf(radians(Link1.getAngle()));
    float y = L1 * cosf(radians(Link1.getAngle()));
    return Position(x, y);
}
float getJoint3Height() {
    // TODO: Verify
    float x = L1 * sinf(radians(Link1.getAngle())) + L2 * sinf(radians(Link2.getAngle()));
    float y = L1 * cosf(radians(Link1.getAngle())) + L2 * cosf(radians(Link2.getAngle()));
    return Position(x, y);
}