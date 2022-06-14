#include <Arduino.h>
#include <TestBench.h>

void TestBench::testLinkMotors() {
    RobotArm.link1ToAngle(0);
    delay(1000);
    RobotArm.link1ToAngle(90);
    delay(1000);
    RobotArm.link1ToAngle(115);
    delay(1000);
    RobotArm.link1ToAngle(0);
    delay(1000);
}
