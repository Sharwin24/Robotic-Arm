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
        int q1Steps = round(degreeToSteps(q1));
        while (true) {
            RobotArm.setLink1Target(q1Steps);
            printLink(1, q1);
            RobotArm.updateLinks();
            delay(d);
            RobotArm.setLink1Target(0.0);
            printLink(1, 0.0);
            RobotArm.updateLinks();
            delay(d);
        }
    }

    void link2ToAngle(Manipulator RobotArm, float q2, long d = 1000) {
        int q2Steps = round(degreeToSteps(q2));
        while (true) {
            RobotArm.setLink2Target(q2Steps);
            printLink(2, q2);
            RobotArm.updateLinks();
            delay(d);
            RobotArm.setLink2Target(0.0);
            printLink(2, 0.0);
            RobotArm.updateLinks();
            delay(d);
        }
    }

    void link3ToAngle(Manipulator RobotArm, float q3, long d = 1000) {
        int q3Steps = round(degreeToSteps(q3));
        while (true) {
            RobotArm.setLink3Target(q3Steps);
            printLink(3, q3);
            RobotArm.updateLinks();
            delay(d);
            RobotArm.setLink3Target(0.0);
            printLink(3, 0.0);
            RobotArm.updateLinks();
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

    void printMatricesTest() {
        // RRRManipulator.ForwardKinematics(15, 30, 45);
        // r01 = [L1*C1, L1*S1, 0], R01 = Rz(q1)
        // r12 = [L2*C1, L2*S1, 0], R12 = Rz(q2)
        // r23 = [L3*C1, L3*S1, 0], R23 = Rz(q3)
        // r3e = [Le, 0, 0], R3e = Ry(90) * Rz(180)

        RotationMatrix Rz = RotationMatrix('z', 45);
        RotationMatrix Rx = RotationMatrix('x', 30);
        printMatrix(Rz);
        printMatrix(Rx);
        HomogeneousTransform A0 = HomogeneousTransform(Rz, Vector(1, 2, 3));
        HomogeneousTransform A1 = HomogeneousTransform(Rx, Vector(3, 4, 5));
        printMatrix(A0);
        printMatrix(A1);
        HomogeneousTransform A01 = multiply(A0, A1);
        printMatrix(A01);
    }
};

#endif