#ifndef TestBench_h
#define TestBench_h
#include <Arduino.h>
#include <Manipulator.h>
#include <Position.h>
#include <JointAngles.h>
#include <Vector.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>
#include <TestBench.h>
#include <Utils.h>

class TestBench {
   public:
    TestBench() {}

    void link1ToAngle(Manipulator RobotArm, float q1, long d = 1000) {
        while (true) {
            printLink(1, q1);
            RobotArm.link1ToAngle(q1);
            delay(d);
            RobotArm.link1ToAngle(0.0);
            printLink(1, 0.0);
            delay(d);
        }
    }

    void link2ToAngle(Manipulator RobotArm, float q2, long d = 1000) {
        while (true) {
            printLink(2, q2);
            RobotArm.link2ToAngle(q2);
            delay(d);
            RobotArm.link2ToAngle(0.0);
            printLink(2, 0.0);
            delay(d);
        }
    }

    void link3ToAngle(Manipulator RobotArm, float q3, long d = 1000) {
        while (true) {
            printLink(3, q3);
            RobotArm.link3ToAngle(q3);
            delay(d);
            RobotArm.link3ToAngle(0.0);
            printLink(3, 0.0);
            delay(d);
        }
    }

    void concurrentLink1Link2(Manipulator RobotArm, float q1, float q2, long d = 1000) {
        int q1Steps = round(degreeToSteps(q1));
        int q2Steps = round(degreeToSteps(q2));
        while (true) {
            RobotArm.setLink1Target(q1Steps);
            RobotArm.setLink2Target(q2Steps);
            printLink1Link2(q1, q2);
            RobotArm.updateLinks();
            delay(d);
            RobotArm.setLink1Target(0.0);
            RobotArm.setLink2Target(0.0);
            printLink1Link2(0.0, 0.0);
            RobotArm.updateLinks();
            delay(d);
        }
    }

    void concurrentLink1Link2Link3(Manipulator RobotArm, float q1, float q2, float q3, long d = 1000) {
        int q1Steps = round(degreeToSteps(q1));
        int q2Steps = round(degreeToSteps(q2));
        int q3Steps = round(degreeToSteps(q3));
        while (true) {
            RobotArm.setLink1Target(q1Steps);
            RobotArm.setLink2Target(q2Steps);
            RobotArm.setLink3Target(q3Steps);
            printLink1Link2Link3(q1, q2, q3);
            RobotArm.updateLinks();
            delay(d);
            RobotArm.setLink1Target(0.0);
            RobotArm.setLink2Target(0.0);
            RobotArm.setLink3Target(0.0);
            printLink1Link2Link3(0.0, 0.0, 0.0);
            RobotArm.updateLinks();
            delay(d);
        }
    }

    void concurrentMovementTest(Manipulator RobotArm, float xTarget, float yTarget, long d = 1000) {
        Position currPos = RobotArm.getEEPos();
        while (true) {
            RobotArm.moveToXY(currPos.x, currPos.y);
            delay(d);
            RobotArm.moveToXY(xTarget, yTarget);
            delay(d);
        }
    }

    void FKToPosition(Manipulator RobotArm, float q1, float q2, float q3) {
        Position p = RobotArm.ForwardKinematics(q1, q2, q3);
        Serial.print("Moving to Position: (");
        Serial.print(p.x, DECIMALPRECISION);
        Serial.print(", ");
        Serial.print(p.y, DECIMALPRECISION);
        Serial.println(")");
        Serial.println("Ensure Links are zeroed");
        RobotArm.setLink1Target(q1);
        RobotArm.setLink2Target(q2);
        RobotArm.setLink3Target(q3);
        RobotArm.updateLinks();
        Serial.println("Verify that EE position and joint angles are correct");
        Serial.println("Computed EE Position: ");
        Position EE = RobotArm.getEEPos();
        Serial.print("(");
        Serial.print(EE.x);
        Serial.print(", ");
        Serial.print(EE.y);
        Serial.println(")");
    }

    void IKToAngles(Manipulator RobotArm, float xTarget, float yTarget) {
        JointAngles Q = RobotArm.InverseKinematics(xTarget, yTarget);
        Serial.print("Moving to angles: [");
        Serial.print(Q.q1, DECIMALPRECISION);
        Serial.print(", ");
        Serial.print(Q.q2, DECIMALPRECISION);
        Serial.print(", ");
        Serial.print(Q.q3, DECIMALPRECISION);
        Serial.println("]");
        RobotArm.setLink1Target(Q.q1);
        RobotArm.setLink2Target(Q.q2);
        RobotArm.setLink3Target(Q.q3);
        RobotArm.updateLinks();
        Serial.println("Verify that EE position and joint angles are correct");
        Serial.println("Computed EE Position: ");
        Position EE = RobotArm.getEEPos();
        Serial.print("(");
        Serial.print(EE.x);
        Serial.print(", ");
        Serial.print(EE.y);
        Serial.println(")");
    }

    void printForwardKinematics(float q1, float q2, float q3) {
        // RRRManipulator.ForwardKinematics(15, 30, 45);
        // r01 = [L1*C1, L1*S1, 0], R01 = Rz(q1)
        // r12 = [L2*C1, L2*S1, 0], R12 = Rz(q2)
        // r23 = [L3*C1, L3*S1, 0], R23 = Rz(q3)
        // r3e = [Le, 0, 0], R3e = Ry(90) * Rz(180)
        float q1R = radians(q1);
        float q2R = radians(q2);
        float q3R = radians(q3);
        Vector r01 = Vector(L1 * cosf(q1R), L1 * sinf(q1R), 0);
        Vector r12 = Vector(L2 * cosf(q2R), L2 * sinf(q2R), 0);
        Vector r23 = Vector(L3 * cosf(q3R), L3 * sinf(q3R), 0);
        Vector r3e = Vector(endEffectorLength, 0, 0);

        RotationMatrix R01 = RotationMatrix('z', q1);
        RotationMatrix R12 = RotationMatrix('z', q2);
        RotationMatrix R23 = RotationMatrix('z', q3);
        RotationMatrix R3e = multiply(RotationMatrix('y', 90), RotationMatrix('z', 180));
        HomogeneousTransform A01 = HomogeneousTransform(R01, r01);
        HomogeneousTransform A12 = HomogeneousTransform(R12, r12);
        HomogeneousTransform A23 = HomogeneousTransform(R23, r23);
        HomogeneousTransform A3e = HomogeneousTransform(R3e, r3e);
        Serial.println("A01 -->");
        printMatrix(A01);
        Serial.println("A12 -->");
        printMatrix(A12);
        Serial.println("A23 -->");
        printMatrix(A23);
        Serial.println("A3e -->");
        printMatrix(A3e);
    }
};

#endif