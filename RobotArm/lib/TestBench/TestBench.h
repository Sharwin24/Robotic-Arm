#ifndef TestBench_h
#define TestBench_h
#include <Arduino.h>
#include <Manipulator.h>
#include <TestBench.h>
#include <Utils.h>

class TestBench {
   public:
    TestBench() {}
    void link1TwoAngles(Manipulator RobotArm, float angle1, float angle2, long d = 1000) {
        while (true) {
            Serial.print("Going to Angle 1: ");
            Serial.println(angle1, DECIMALPRECISION);
            RobotArm.link1ToAngle(angle1);
            delay(d);
            Serial.print("Going to Angle 2: ");
            Serial.println(angle2, DECIMALPRECISION);
            RobotArm.link1ToAngle(angle2);
            delay(d);
        }
    }

    void link2TwoAngles(Manipulator RobotArm, float angle1, float angle2, long d = 1000) {
        while (true) {
            Serial.print("Going to Angle 1: ");
            Serial.println(angle1, DECIMALPRECISION);
            RobotArm.link2ToAngle(angle1);
            delay(d);
            Serial.print("Going to Angle 2: ");
            Serial.println(angle2, DECIMALPRECISION);
            RobotArm.link2ToAngle(angle2);
            delay(d);
        }
    }

    void link3TwoAngles(Manipulator RobotArm, float angle1, float angle2, long d = 1000) {
        while (true) {
            Serial.print("Going to Angle 1: ");
            Serial.println(angle1, DECIMALPRECISION);
            RobotArm.link3ToAngle(angle1);
            delay(d);
            Serial.print("Going to Angle 2: ");
            Serial.println(angle2, DECIMALPRECISION);
            RobotArm.link3ToAngle(angle2);
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

    void concurrentLink1Link2(Manipulator RobotArm, float q1, float q2, long d = 1000) {
        Position currPos = RobotArm.getEEPos();
        JointAngles currAngles = RobotArm.InverseKinematics(currPos.x, currPos.y);
        int q1Steps = floorf(degreeToSteps(q1));
        int q2Steps = floorf(degreeToSteps(q2));
        while (true) {
            RobotArm.setLink1Target(q1Steps);
            RobotArm.setLink2Target(q2Steps);
            printLink1Link2(q1, q2);
            RobotArm.updateLinks();
            RobotArm.setLink1Target(floorf(degreeToSteps(currAngles.q1)));
            RobotArm.setLink2Target(floorf(degreeToSteps(currAngles.q2)));
            printLink1Link2(currAngles.q1, currAngles.q2);
            RobotArm.updateLinks();
        }
    }

    void printMatricesTest() {
        RotationMatrix R = RotationMatrix('z', 45);
        printMatrix(R);
        HomogeneousTransform A = HomogeneousTransform(R, Vector(1, 2, 3));
        printMatrix(A);
    }
};

#endif