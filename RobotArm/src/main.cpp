#include <Arduino.h>
#include <Position.h>
#include <JointAngles.h>
#include <Utils.h>
#include <Manipulator.h>
#include <TestBench.h>
// Construct a TestBench to run pre-made tests on your Manipulator
// NOTE: Most TestBench methods are blocking
TestBench tb = TestBench();
// Construct a 3-Revolute Link Planar Manipulator with default link lengths
// Default constants (link lengths, link speed, motor/sensor pins, motor-driver settings, etc.) are in Utils.h
Manipulator RRRManipulator = Manipulator();

void setup() {
    initSerialMonitor();
    RRRManipulator.init();
    Serial.println("Manipulator Initialized");
    Serial.println("Ensure Power is connected and links are zeroed");
    Serial.println("Disconnect power if mechanical failure occurs");
    Serial.println("Executing Control Loop in 3 seconds");
    delay(3000);
    tb.printForwardKinematics(15, 30, 90);
}

void loop() {
    tb.concurrentMotionFK(RRRManipulator, 15, 30, 90);
}
