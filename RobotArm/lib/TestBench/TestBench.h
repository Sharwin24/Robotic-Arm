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

    void linkToAngle(Manipulator RobotArm, int linkNumber, float q1, long d = 1000) {
        while (true) {
            printLink(linkNumber, q1);
            RobotArm.linkToAngle(q1, linkNumber);
            delay(d);
            RobotArm.linkToAngle(0.0, linkNumber);
            printLink(linkNumber, 0.0);
            delay(d);
        }
    }

    void concurrentMotionFK(Manipulator RobotArm, float q1, float q2, float q3, long d = 1000) {
        while (true) {
            RobotArm.moveToAngles(q1, q2, q3);
            printLink1Link2Link3(q1, q2, q3);
            delay(d);
            RobotArm.moveToAngles(0, 0, 0);
            printLink1Link2Link3(0, 0, 0);
            delay(d);
        }
    }

    void concurrentMotionIK(Manipulator RobotArm, float xTarget, float yTarget, long d = 1000) {
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
        RobotArm.moveToAngles(q1, q2, q3);
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
        RobotArm.moveToXY(xTarget, yTarget);
        Serial.println("Verify that EE position and joint angles are correct");
        Serial.println("Computed EE Position: ");
        Position EE = RobotArm.getEEPos();
        Serial.print("(");
        Serial.print(EE.x);
        Serial.print(", ");
        Serial.print(EE.y);
        Serial.println(")");
    }

    void printManipulatorFK(Manipulator m, float q1, float q2, float q3) {
        m.ForwardKinematics(q1, q2, q3, true);
    }

    void printForwardKinematics(float q1, float q2, float q3) {
        // r01 = [L1*C1, L1*S1, 0], R01 = Rz(q1)
        // r12 = [L2*C1, L2*S1, 0], R12 = Rz(q2)
        // r23 = [L3*C1, L3*S1, 0], R23 = Rz(q3)
        // r3e = [Le, 0, 0], R3e = I_3
        float q1R = radians(q1);
        float q2R = radians(q2);
        float q3R = radians(q3);
        Vector r01 = Vector(L1 * cosf(q1R), L1 * sinf(q1R), 0);
        Vector r12 = Vector(L2 * cosf(q2R), L2 * sinf(q2R), 0);
        Vector r23 = Vector(L3 * cosf(q3R), L3 * sinf(q3R), 0);
        Vector r3e = Vector(Le, 0, 0);
        RotationMatrix R01 = RotationMatrix('z', q1);
        RotationMatrix R12 = RotationMatrix('z', q2);
        RotationMatrix R23 = RotationMatrix('z', q3);
        RotationMatrix R3e = RotationMatrix();
        HomogeneousTransform A01 = HomogeneousTransform(R01, r01);
        HomogeneousTransform A12 = HomogeneousTransform(R12, r12);
        HomogeneousTransform A23 = HomogeneousTransform(R23, r23);
        HomogeneousTransform A3e = HomogeneousTransform(R3e, r3e);
        HomogeneousTransform A02 = multiply(A01, A12);
        HomogeneousTransform A03 = multiply(A02, A23);
        HomogeneousTransform A0e = multiply(A03, A3e);
        printTransform(A01, "01");
        printTransform(A12, "12");
        printTransform(A23, "23");
        printTransform(A3e, "3e");
        printTransform(A02, "02");
        printTransform(A03, "03");
        printTransform(A0e, "0e");
    }
};

#endif