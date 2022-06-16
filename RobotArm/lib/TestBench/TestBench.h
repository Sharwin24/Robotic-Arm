#ifndef TestBench_h
#define TestBench_h
#include <Arduino.h>
#include <Manipulator.h>
#include <TestBench.h>
class TestBench {
   public:
    TestBench() {}
    void twoAngleTest(Manipulator RobotArm, float angle1, float angle2, long d = 1000) {
        while (true) {
            Serial.print("Going to Angle 1: ");
            Serial.println(angle1);
            RobotArm.link1ToAngle(angle1);
            delay(d);
            Serial.print("Going to Angle 2: ");
            Serial.println(angle2);
            RobotArm.link1ToAngle(angle2);
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
};

#endif