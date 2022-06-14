#include <Arduino.h>
#include <Manipulator.h>
#include <Position.h>
#include <JointAngles.h>
#include <Utils.h>

void setup() {
    Serial.begin(9600);
    Serial.flush();
    Manipulator RobotArm = Manipulator();
}

void loop() {
    // Link3.moveTo(0);
    // delay(1000);
    // Link3.moveTo(1500);
    // delay(1000);
    // Link3.moveTo(2000);
    // delay(1000);
    // Link3.moveTo(0);
    // delay(1000);
}