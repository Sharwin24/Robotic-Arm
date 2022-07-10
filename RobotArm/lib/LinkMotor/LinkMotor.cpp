#include <Arduino.h>
#include <LinkMotor.h>
#include <Utils.h>

/**
 * @brief Initializes the LinkMotor Object and sets default values for:
 * current, target, currentSpeed, currentDelay. Also sets up pin-modes for motor/sensor pins
 *
 */
void LinkMotor::init() {
    current = 0;
    currentAngle = 0.0;  // current % stepsPerRev;
    target = 0;
    currentSpeed = stepsPerRev * RPM * (1.0 / 60.0);  // Speed is measured in steps per second
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

/**
 * @brief Calibrates the LinkMotor to start at 'zero' position. When called, this method will move the link towards the limit switch indicating 'zero' until triggered. 
 * If the link does not support a limit switch, this method has no operation.
 *
 * @return int returns -1 on no-op, otherwise returns number of steps taken to calibrate Link
 */
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

/**
 * @brief Sets the speed (steps per sec) and calculates the required pulse delay
 *
 * @param speed speed in steps per second
 */
void LinkMotor::setSpeed(float speed) {
    currentSpeed = speed;
    currentDelay = getDelayFromSpeed(speed);
}

/**
 * @brief Sets the direction given a bool representing direction
 *
 * @param CW  True -> CW, False -> CCW
 */
void LinkMotor::setDirection(bool CW) {
    if (CW) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
    currentDir = CW;
}

/**
 * @brief Sets the target to the given target step.
 * Also sets the direction.
 * This method will not execute if the link is still in motion.
 *
 * @param targetStep
 */
void LinkMotor::setTarget(int targetStep) {
    if (current != target) {
        return;  // Motor is currently moving
    }
    previous = current;
    target = targetStep * outputGearRatio;
    setDirection(current < target);  // True -> CW, False -> CCW
}

// Getter methods for class variables
float LinkMotor::getTarget() { return target; }
float LinkMotor::getSpeed() { return currentSpeed; }
long LinkMotor::getDelay() { return currentDelay; }
float LinkMotor::getAngle() { return currentAngle; }
float LinkMotor::getGR() { return outputGearRatio; }
int LinkMotor::getLimitSwitch() { return limitSwitchPin != -1 ? digitalRead(limitSwitchPin) : -1; }

/**
 * @brief Updates the current angle of the link using the current step of the LinkMotor.
 *
 */
void LinkMotor::updateAngle() {
    currentAngle = abs(current % round(stepsPerRev));
    currentAngle /= (stepsPerRev / 360.0);
    if (current < 0) {
        currentAngle *= -1;
    }
}

/**
 * @brief Provides a pulse to the motor using the current delay calculated from the current speed
 *
 */
void LinkMotor::stepMotor() {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
}

/**
 * @brief Given a target step (negative steps allowed), calculates the number of steps and direction, and steps the motor towards the target.
 * This is a blocking function and will only allow for this movement to be executed.
 *
 * @param targetStep the target to acheive. Represents an absolute position
 */
void LinkMotor::moveTo(int targetStep) {
    setDirection(current < targetStep);  // True -> CW, False -> CCW
    int stepDifference = abs(targetStep - current);
    for (int i = 0; i < stepDifference; i++) {
        stepMotor();
    }
    current = targetStep;
    updateAngle();
}

/**
 * @brief A more practical moveTo() function taking in a target angle.
 * This method calls moveTo(int targetStep) after translating the given angle to steps.
 *
 * @param targetAngle the target angle in degrees
 */
void LinkMotor::moveToAngle(float targetAngle) {
    float steps = round(degreeToSteps(targetAngle) * outputGearRatio);
    moveTo(steps);
}

/**
 * @brief Primary function to be used for non-blocking motion.
 * This method will be called multiple times per second and will
 * determine when to step the motor given the current time of the program
 * and the time the motor was last pulsed. Updates current and currentAngle
 * as motion occurs.
 *
 */
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

/**
 * @brief Calls update() until the current step has met the target step
 *
 * @return true on function termination
 */
bool LinkMotor::waitToStop() {
    if (current == target) {
        return true;
    }
    while (current != target) {
        update();
    }
    return true;
}
