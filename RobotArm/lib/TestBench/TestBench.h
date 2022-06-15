#ifndef TestBench_h
#define TestBench_h
#include <Arduino.h>
#include <Manipulator.h>
#include <LinkMotor.h>
#include <TestBench.h>
class TestBench {
   private:
    Manipulator RobotArm;  // = Manipulator();

   public:
    TestBench() {
        Serial.println("Initializing TestBench using Default Manipulator");
        RobotArm = Manipulator();
    }

    TestBench(Manipulator m) {
        Serial.println("Initializing TestBench using Given Manipulator");
        RobotArm = m;
    }

    void twoAngleTest(float angle1, float angle2, long d = 1000) {
        RobotArm.link1ToAngle(angle1);
        delay(d);
        RobotArm.link1ToAngle(angle2);
        delay(d);
        RobotArm.link1ToAngle(angle1);
        delay(d);
    }
};

#endif