#include <Arduino.h>
#include <Position.h>
#include <JointAngles.h>
#include <Utils.h>
#include <Manipulator.h>
#include <TestBench.h>

// Construct a TestBench to run pre-made tests on your Manipulator
TestBench tb = TestBench();
// Construct a 3-Revolute Link Planar Manipulator with default link lengths
// Default constants (link lengths, link speed, motor/sensor pins, motor-driver settings, etc.) are in Utils.h
Manipulator RRRManipulator = Manipulator();

void setup() {
    initSerialMonitor();
    RRRManipulator.init();
    Serial.println("Manipulator Initialized");
    Serial.println("Ensure [ower is connected and links are zeroed");
    Serial.println("Disconnect power if mechanical failure occurs");
    delay(3000);
    Serial.println("Executing Control Loop");
}

void loop() {
    // NOTE: TestBench methods are blocking
    // tb.link1TwoAngles(RRRManipulator, 0, 90);
    // tb.printMatricesTest();
}