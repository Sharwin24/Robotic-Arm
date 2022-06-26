#ifndef Gripper_h
#define Gripper_h
#include <Arduino.h>
#include <Servo.h>

class Gripper {
   private:
    // Constructor Arguments
    int leftServoPin;
    int rightServoPin;
    // Servo objects and angles
    Servo servoL;
    Servo servoR;
    const int leftOpenAngle = -1;  // TODO: Find and set
    const int rightOpenAngle = -1;
    const int leftCloseAngle = -1;
    const int rightCloseAngle = -1;

   public:
    Gripper(int _leftServoPin, int _rightServoPin) {
        leftServoPin = _leftServoPin;
        rightServoPin = _rightServoPin;
    }

    void init() {
        servoL.attach(leftServoPin);
        servoR.attach(rightServoPin);
    }

    void close() {
        servoL.write(leftCloseAngle);
        servoR.write(rightCloseAngle);
    }

    void open() {
        servoL.write(leftOpenAngle);
        servoR.write(rightOpenAngle);
    }
};
#endif