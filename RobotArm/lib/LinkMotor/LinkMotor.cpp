#include <Arduino.h>
#include <LinkMotor.h>
#include <Utils.h>

// Docs in header file

void LinkMotor::init() {
    current = 0;
    currentAngle = 0.0;  // current % stepsPerRev;
    target = 0;
    // currentSpeed = 1000;  // Speed is measured in steps per second
    // steps per rev * revs per min = steps per min * 1 min per 60 seconds = steps per seconds
    if (linkNumber == 1 || linkNumber == 2) {
        currentSpeed = stepsPerRev * RPM * (1 / 60.0);
    } else {
        currentSpeed = stepsPerRev * (RPM / 2.0) * (1 / 60.0);
    }
    currentDelay = getDelayFromSpeed(currentSpeed);
    previousChangeTime = micros();
    currentlyRunning = false;

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    if (limitSwitchPin != -1) {
        pinMode(limitSwitchPin, INPUT_PULLUP);
    }
    delay(500);
}

// Sets the speed (steps per sec) and calculates the required pulse delay
void LinkMotor::setSpeed(float speed) {
    currentSpeed = speed;
    currentDelay = getDelayFromSpeed(speed);
}

void LinkMotor::setDirection(bool CW) {
    if (CW) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
    currentDir = CW;
}

void LinkMotor::setTarget(int targetStep) {
    if (current != target) {
        return;  // Motor is currently moving
    }
    previous = current;
    target = targetStep;
    setDirection(current < target);  // True -> CW, False -> CCW
}

float LinkMotor::getSpeed() { return currentSpeed; }

int LinkMotor::getDelay() { return currentDelay; }

float LinkMotor::getAngle() { return currentAngle; }

void LinkMotor::updateAngle() {
    currentAngle = current % stepsPerRev;
    if (current < 0) {
        currentAngle *= -1
    }
}

int LinkMotor::getLimitSwitch() {
    // If there is a valid limit switch pin, read it, otherwise return -1
    return limitSwitchPin != -1 ? digitalRead(limitSwitchPin) : -1;
}

// Pulse the motor with the current delay value calculated from current speed
void LinkMotor::stepMotor() {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
}

void LinkMotor::moveTo(int targetStep) {
    setDirection(current < targetStep);  // True -> CW, False -> CCW
    int stepDifference = abs(targetStep - current);
    for (int i = 0; i < stepDifference; i++) {
        stepMotor();
    }
    current = targetStep;
    updateAngle();
}

void LinkMotor::moveToAngle(float degrees) {
    float steps = degreeToSteps(degrees) * outputGearRatio;
    int stepsInt = floorf(steps);
    moveTo(stepsInt);
}

int LinkMotor::calibrate() {
    if (limitSwitchPin == -1) {
        return -1;
    }

    int steps = 0;
    bool limitSwitchDir = true;
    setDirection(limitSwitchDir);
    while (getLimitSwitch() == 1) {
        stepMotor();
        steps++;
    }

    setDirection(!limitSwitchDir);
    while (getLimitSwitch() == 0) {
        stepMotor();
    }
    return steps;
}

void LinkMotor::update() {
    if (current == target) {
        return;
    }
    // So this function will get called many times per second, we want to trigger steps based on the speed
    // We can keep track of the current time of the program and the last time we did a step, notably we should not have
    // any delays here
    // int stepsMoved = abs(current - previous);
    // int totalSteps = abs(target - previous);
    // We can go ahead and assume that the current speed is updated, then every time we finish a step we will go ahead
    // and update it
    long currentTime = micros();
    long timeSinceLastStep = currentTime - previousChangeTime;
    if (timeSinceLastStep > currentDelay) {
        currentlyRunning = !currentlyRunning;
        if (currentlyRunning) {  // Step if currently running
            digitalWrite(stepPin, HIGH);
        } else {
            digitalWrite(stepPin, LOW);
        }
        previousChangeTime = currentTime;
        // Speed should already have been set
        // long s = getSpeedCurve(stepsMoved, totalSteps);
        // setSpeed(s);
        if (!currentlyRunning) {
            // increment steps in off cycle
            if (current < target) {
                current++;
            } else {
                current--;
            }
            updateAngle();
        }
    }
}

bool LinkMotor::isMoving() { return current != target; }

bool LinkMotor::waitToStop() {
    if (current == target) {
        return true;
    }
    while (current != target) {
        update();
    }
    return true;
}
