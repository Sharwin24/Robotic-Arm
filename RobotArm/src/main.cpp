#include <Arduino.h>
#include <Position.h>
#include <JointAngles.h>
#include <Utils.h>
#include <Manipulator.h>
#include <TestBench.h>

// Construct a 3-Revolute Link Planar Manipulator with default link lengths
// Default constants (link lengths, motor/sensor pins) are in Manipulator.h
// Construct a TestBench to run pre-made tests on your Manipulator
Manipulator RRRManipulator = Manipulator();
TestBench tb = TestBench(RRRManipulator);

void setup() {
    initSerialMonitor();
}

void loop() {
    tb.twoAngleTest(0, 90);
}