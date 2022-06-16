#include <Arduino.h>
#include <Position.h>
#include <JointAngles.h>
#include <Utils.h>
#include <Manipulator.h>
#include <TestBench.h>

// Construct a 3-Revolute Link Planar Manipulator with default link lengths
// Default constants (link lengths, motor/sensor pins) are in Manipulator.h
// Construct a TestBench to run pre-made tests on your Manipulator
TestBench tb = TestBench();
Manipulator RRRManipulator = Manipulator(L1, L2, L3);

void setup() {
    initSerialMonitor();
    RRRManipulator.init();
    Serial.println("Manipulator Initialized");
    Serial.println("Ensure Power is connected and Links are zeroed");
    delay(2000);
}

void loop() {
    // NOTE: TestBench methods are blocking
    tb.twoAngleTest(RRRManipulator, 0, 90);
}