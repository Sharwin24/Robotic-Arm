#ifndef TestBench_h
#define TestBench_h
#include <Arduino.h>
#include <Manipulator.h>
#include <LinkMotor.h>
#include <TestBench.h>
class TestBench {
   private:
    Manipulator RobotArm = Manipulator();

   public:
    TestBench(Manipulator m) {
        Serial.println("Initializing TestBench");
        RobotArm = m;
    }

    void testLinkMotors() {
        RobotArm.link1ToAngle(0);
        delay(1000);
        RobotArm.link1ToAngle(90);
        delay(1000);
        RobotArm.link1ToAngle(115);
        delay(1000);
        RobotArm.link1ToAngle(0);
        delay(1000);
    }
};

#endif