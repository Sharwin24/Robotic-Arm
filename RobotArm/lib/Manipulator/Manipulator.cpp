#include <Manipulator.h>
#include <Position.h>
#include <JointAngles.h>
#include <math.h>
#include <Vector.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>
#include <Utils.h>

/**
 * @brief Using Forward Kinematics for this 3R-Planar Manipulator, obtains the
 * position of the end-effector given the joint angles.
 *
 * @param q1 Link 1 angle [deg]
 * @param q2 Link 2 angle [deg]
 * @param q3 Link 3 angle [deg]
 * @param debug boolean to display debugging info, false by default
 * @return Position An object coupling (x,y) position
 */
Position Manipulator::ForwardKinematics(float q1, float q2, float q3, bool debug) {
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
    Vector r3e = Vector(Le, 0.0, 0.0);
    RotationMatrix R3e = RotationMatrix();  // No rotation -> Identity
    HomogeneousTransform A3e = HomogeneousTransform(R3e, r3e);
    // Find Transformation
    HomogeneousTransform A02 = multiply(A01, A12);
    HomogeneousTransform A2e = multiply(A23, A3e);
    HomogeneousTransform A0e = multiply(A02, A2e);
    if (debug) {
        printTransform(A01, "01");
        printTransform(A12, "12");
        printTransform(A23, "23");
        printTransform(A3e, "3e");
        printTransform(A0e, "0e");
    }
    return Position(A0e.get(0, 3), A0e.get(1, 3));
}

/**
 * @brief Using Inverse Kinematics for this 3R-Planar Manipulator, obtains the
 * joint angles given an end-effector Position.
 *
 * @param xTarget End-Effector X position [mm]
 * @param yTarget End-Effector Y position [mm]
 * @param debug boolean to display debugging info, false by default
 * @return JointAngles an object coupling the joint angles together (q1,q2,q3)
 */
JointAngles Manipulator::InverseKinematics(float xTarget, float yTarget, bool debug) {
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
    if (debug) {
        printLink(1, q1D);
        printLink(2, q2D);
        printLink(3, q3D);
        Serial.println("IK Values:");
        Serial.print("ell -> ");
        Serial.println(ell, DECIMALPRECISION);
        Serial.print("beta -> ");
        Serial.println(beta, DECIMALPRECISION);
        Serial.print("alpha -> ");
        Serial.println(alpha, DECIMALPRECISION);
        Serial.println();
    }
    return JointAngles(q1D, q2D, q3D);
}

/**
 * @brief Rotates each joint to their given angle with concurrent motion.
 * If a given angle is not within the respective link's range-of-motion,
 * no motion will occur.
 *
 * @param q1 Link 1 angle [deg]
 * @param q2 Link 2 angle [deg]
 * @param q3 Link 3 angle [deg]
 */
void Manipulator::moveToAngles(float q1, float q2, float q3) {
    if (!Link1.withinROM(q1) || !Link2.withinROM(q2) || !Link3.withinROM(q3)) {
        return;
    }
    // Obtain target angles as steps
    int q1Steps = round(degreeToSteps(q1));
    int q2Steps = round(degreeToSteps(q2));
    int q3Steps = round(degreeToSteps(q3));
    // Set the target step for each Link
    Link1.setTarget(q1Steps);
    Link2.setTarget(q2Steps);
    Link3.setTarget(q3Steps);
    // Update all Links until target is acheived
    updateLinks();
    // Update EE pos
    updateEEPos(q1, q2, q3);
}

/**
 * @brief Rotates each joint to the given angle targets with concurrent motion
 *
 *
 * @param jointAngleTargets an object coupling the joint angles together
 */
void Manipulator::moveToJointAngles(JointAngles jointAngleTargets) {
    moveToAngles(jointAngleTargets.q1, jointAngleTargets.q2, jointAngleTargets.q3);
}

/**
 * @brief Using InverseKinematics, given a (x,y) position,
 * the arm will move such that the End-Effector is at the given location.
 * TODO: Implement ROM so if the motion is not possible, the arm will not move.
 *
 * @param xTarget desired End-Effector X position [mm]
 * @param yTarget desired End-Effector Y position [mm]
 */
void Manipulator::moveToXY(float xTarget, float yTarget) {
    JointAngles Q = InverseKinematics(xTarget, yTarget);
    moveToAngles(Q.q1, Q.q2, Q.q3);
}

/**
 * @brief Using InverseKinematics, given a Position object,
 * the arm will move such that the End-Effector is at the given location.
 * If the motion is not possible, the arm will not move.
 *
 * @param positionTarget a Position object coupling (x,y)
 * for the desired EE position
 */
void Manipulator::moveToPosition(Position positionTarget) {
    moveToXY(positionTarget.x, positionTarget.y);
}

/**
 * @brief This method is meant to be run internally after targets have been
 * set for each Link. Using a decision tree for each Link's required direction,
 * this method will handle the order of concurrency for moving each link.
 * In most cases, all 3 links will move simultaneously, however some cases
 * require moving a specific link first.
 *
 */
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
    // TODO: Collapse and validate decision tree
    // CCW = 0, CW = 1
    if ((CW1 && CW2 && CW3) || (CW1 && CW2 && !CW3)) {
        // 111 || 110
        while (Link1.isMoving() || Link2.isMoving() || Link3.isMoving()) {
            Link1.update();
            Link2.update();
            Link3.update();
        }
    } else if ((!CW1 && CW2 && CW3) || (CW1 && !CW2 && CW3)) {
        // 011 || 101
        while (Link1.isMoving()) {
            Link1.update();
        }
        while (Link2.isMoving() || Link3.isMoving()) {
            Link2.update();
            Link3.update();
        }
    } else if ((CW1 && !CW2 && !CW3) || (!CW1 && CW2 && !CW3)) {
        // 100 || 010
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
    } else {  // 0 0 0
        while (Link1.isMoving() || Link2.isMoving() || Link3.isMoving()) {
            Link1.update();
            Link2.update();
            Link3.update();
        }
    }
}

/**
 * @brief Sets the target for Link 1
 *
 * @param targetStep the desired target as integer steps
 */
void Manipulator::setLink1Target(int targetStep) {
    Link1.setTarget(targetStep);
}

/**
 * @brief Sets the target for Link 2. NOTE: Check in-line comments
 *
 * @param targetStep the desired target as integer steps
 */
void Manipulator::setLink2Target(int targetStep) {
    // For some reason, Link 2 always travels half its target
    // Math has been checked, Gear ratio has been checked,
    // SPR on driver has been checked
    // Potential Reasons:
    // - Nema 14 has a different steps setting?
    // - Motor Driver is faulty? (unlikely)
    Link2.setTarget(targetStep * 2.0);  // !!Temporary Fix!!
}

/**
 * @brief Sets the target for Link 3.
 *
 * @param targetStep the desired target as integer steps
 */
void Manipulator::setLink3Target(int targetStep) {
    Link3.setTarget(targetStep);
}

/**
 * @brief Sets the target for the given link number and updates the link
 * to complete the motion
 *
 * @param degrees target angle for the link
 * @param linkNumber the number for which link to move [1,2,3]
 */
void Manipulator::linkToAngle(float degrees, int linkNumber) {
    if (linkNumber == 1 && Link1.withinROM(degrees)) {
        link1ToAngle(degrees);
    } else if (linkNumber == 2 && Link2.withinROM(degrees)) {
        link2ToAngle(degrees);
    } else if (linkNumber == 3 && Link3.withinROM(degrees)) {
        link3ToAngle(degrees);
    } else {
        Serial.print("Invalid Link Number given");
    }
}

/**
 * @brief Sets the target for Link 1 using degrees and updates the link
 * to complete the motion.
 *
 * @param degrees target angle for the link
 */
void Manipulator::link1ToAngle(float degrees) {
    // Link1.moveToAngle(degrees);
    int steps = round(degreeToSteps(degrees));
    setLink1Target(steps);
    Link1.waitToStop();
    updateEEPos(degrees, Link2.getAngle(), Link3.getAngle());
}

/**
 * @brief Sets the target for Link 2 using degrees and updates the link
 * to complete the motion.
 *
 * @param degrees target angle for the link
 */
void Manipulator::link2ToAngle(float degrees) {
    // Link2.moveToAngle(degrees);
    int steps = round(degreeToSteps(degrees));
    setLink2Target(steps);
    Link2.waitToStop();
    updateEEPos(Link1.getAngle(), degrees, Link3.getAngle());
}

/**
 * @brief Sets the target for Link 3 using degrees and updates the link
 * to complete the motion.
 *
 * @param degrees target angle for the link
 */
void Manipulator::link3ToAngle(float degrees) {
    // Link3.moveToAngle(degrees);
    int steps = round(degreeToSteps(degrees));
    setLink3Target(steps);
    Link3.waitToStop();
    updateEEPos(Link1.getAngle(), Link2.getAngle(), degrees);
}

// Getter methods for outputGearRatio (GR)
float Manipulator::getLink1GR() { return Link1.getGR(); }
float Manipulator::getLink2GR() { return Link2.getGR(); }
float Manipulator::getLink3GR() { return Link3.getGR(); }

/*
  Y ^
    |
    *--> X
 */

/**
 * @brief Obtains the 2D Position of Joint 1, which Link 1 rotates about
 *
 * @return Position object coupling (x,y)
 */
Position Manipulator::getJoint1Position() {
    float x = 0;
    float y = 0;
    return Position(x, y);
}

/**
 * @brief Obtains the 2D Position of Joint 2, which Link 2 rotates about
 *
 * @return Position object coupling (x,y)
 */
Position Manipulator::getJoint2Position() {
    float x = L1 * cosf(radians(Link1.getAngle()));
    float y = L1 * sinf(radians(Link1.getAngle()));
    return Position(x, y);
}

/**
 * @brief Obtains the 2D Position of Joint 3, which Link 3 rotates about
 *
 * @return Position object coupling (x,y)
 */
Position Manipulator::getJoint3Position() {
    float x = L1 * cosf(radians(Link1.getAngle())) + L2 * cosf(radians(Link2.getAngle()));
    float y = L1 * sinf(radians(Link1.getAngle())) + L2 * sinf(radians(Link2.getAngle()));
    return Position(x, y);
}