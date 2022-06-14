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