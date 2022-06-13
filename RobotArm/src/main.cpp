#include <Arduino.h>
#include <LinkMotor.h>
#include <Utils.h>

// Link 1
const int link1DirPin = -1;
const int link1StepPin = -1;
const int link1LimitSwitchPin = -1;
LinkMotor Link1 = LinkMotor(1, 17, link1StepPin, link1DirPin, link1LimitSwitchPin);

// Link 2
const int link2DirPin = -1;
const int link2StepPin = -1;
LinkMotor Link2 = LinkMotor(2, 14, link2StepPin, link2DirPin, -1);

// Link 3
const int link3DirPin = 3;
const int link3StepPin = 2;
LinkMotor Link3 = LinkMotor(3, 11, link3StepPin, link3DirPin, -1);

// LinkMotor links[] = {Link1, Link2, Link3};
// int numLinks = sizeof(links) / sizeof(LinkMotor);

void setup() {
    Serial.begin(9600);
    Serial.flush();
    writeMsg("Initializing Motors");
    Link1.init();
    Link2.init();
    Link3.init();
    writeMsg("Calibrating Motors");
    Link1.calibrate();
    Link2.calibrate();
    Link3.calibrate();
    writeMsg("Setup Complete");
    delay(1000);
}

void loop() {
    Link3.moveTo(0);
    delay(1000);
    Link3.moveTo(1500);
    delay(1000);
    Link3.moveTo(2000);
    delay(1000);
    Link3.moveTo(0);
    delay(1000);
}