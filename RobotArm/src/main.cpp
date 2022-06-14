#include <Arduino.h>
#include <Manipulator.h>
#include <Position.h>
#include <JointAngles.h>
#include <Utils.h>

Manipulator RobotArm = Manipulator();
void setup() {
    Serial.begin(9600);
    Serial.flush();
}

void loop() {
    RobotArm.link1ToAngle(0);
    delay(1000);
    RobotArm.link1ToAngle(90);
    delay(1000);
    RobotArm.link1ToAngle(115);
    delay(1000);
    RobotArm.link1ToAngle(0);
    delay(1000);
}