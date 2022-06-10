#include <Arduino.h>
#include <LinkMotor.h>

// Docs in header file

void LinkMotor::init() {
    current = 0;
    target = 0;
    currentSpeed = 500;
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

void LinkMotor::setSpeed(long speed) {
    currentSpeed = speed;
    currentDelay = getDelayFromSpeed(speed);
}

int LinkMotor::getSpeed() { return currentSpeed; }

int LinkMotor::getDelay() { return currentDelay; }

int LinkMotor::getLimitSwitch() {
    return limitSwitchPin != -1 ? digitalRead(limitSwitchPin) : -1;
}

void LinkMotor::setDirection(bool CW) {
    // TODO: Test CW and CCW
    if (CW) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
    currentDir = CW;
}

void LinkMotor::stepMotor() {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
}

void LinkMotor::moveTo(int targetStep) {
    if (targetStep < 0) {
        Serial.print("Target step must be positive");
        return;
    }
    if (current > targetStep) {
        setDirection(false);
    } else {
        setDirection(true);
    }
    int stepDifference = abs(targetStep - current);
    for (int i = 0; i < stepDifference; i++) {
        stepMotor();
    }
    current = targetStep;
}

int LinkMotor::calibrate() {
    if (limitSwitchPin == -1) {
        return -1;
    }

    int steps = 0;
    limitSwitchDir = true;  // TODO: Tune direction
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
    if (current = target) {
        return;
    }
    // So this function will get called many times per second, we want to trigger steps based on the speed
    // We can keep track of the current time of the program and the last time we did a step, notably we should not have
    // any delays here
    int stepsMoved = abs(current - previous);
    int totalSteps = abs(target - previous);
    // We can go ahead and assume that the current speed is updated, then every time we finish a step we will go ahead
    // and update it
    long currentTime = micros();
    long timeSinceLastStep = currentTime - previousChangeTime;
    if (timeSinceLastStep > currentDelay) {
        currentlyRunning = !currentlyRunning;
        writeMotor(currentlyRunning);
        previousChangeTime = currentTime;
        // Speed should already have been set
        // long s = getSpeed();
        // setSpeed(s);
        if (!currentlyRunning) {
            // increment steps in off cycle
            if (current < target) {
                current++;
            } else {
                current--;
            }
        }
    }
}

void LinkMotor::setTarget(int targetStep) {
    if (current != target) {
        return;
    }
    if (targetStep < 0) {
        return;
    }
    previous = current;
    target = targetStep;
    if (current > target) {
        setDirection(false);  // TODO: Tune directions
    } else {
        setDirection(true);
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
