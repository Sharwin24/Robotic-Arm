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
        print("Initializing TestBench");
        RobotArm = m;
    }

    void testLinkMotors();
};

#endif