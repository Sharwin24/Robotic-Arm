#include <Arduino.h>
#include <Utils.h>
#include <math.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>

void initSerialMonitor() {
    Serial.begin(9600);
    Serial.flush();
    Serial.println("Serial Monitor Initialized");
}

long getDelayFromSpeed(float s) {
    // Converting ticks per second into a delay timing
    long microseconds = 1000000 / s;
    return microseconds;
}

float degreeToSteps(float degree) {
    // Steps per revolution (set on driver)
    // Steps per revolution (SPR) -> angle (Degrees)
    // SPR * 1 / 360 = steps per degree * degree
    return (stepsPerRev / 360.0) * degree;
}

void printLink1Link2(float q1, float q2) {
    Serial.println();
    Serial.print("Link 1 -> ");
    Serial.print(q1, DECIMALPRECISION);
    Serial.print(" | ");
    Serial.print("Link 2 -> ");
    Serial.println(q2, DECIMALPRECISION);
}

// Rotation Matrix Methods
RotationMatrix multiply(RotationMatrix R1, RotationMatrix R2) {
    float matrix[3][3] = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}};
    for (int m = 0; m < 3; m++) {
        for (int r = 0; r < 3; r++) {
            for (int k = 0; k < 3; k++) {
                matrix[m][r] += R1.get(m, k) * R2.get(k, r);
            }
        }
    }
    return RotationMatrix(matrix);
}

RotationMatrix transpose(RotationMatrix R) {
    float matrix[3][3] = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}};
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            matrix[c][r] = R.get(r, c);
        }
    }
    return RotationMatrix(matrix);
}

void printMatrix(RotationMatrix R) {
    String msg = "R";
    msg.concat(R.getRotType());
    msg.concat("(");
    msg.concat(R.getTheta());
    msg.concat(") -> ");
    Serial.println(msg);
    for (int r = 0; r < 3; r++) {
        Serial.print("|");
        for (int c = 0; c < 3; c++) {
            Serial.print(" ");
            Serial.print(R.get(r, c), DECIMALPRECISION);
            Serial.print(" ");
        }
        Serial.println("|");
    }
}

// Homogeneous Transform Matrix Methods
HomogeneousTransform multiply(HomogeneousTransform A1, HomogeneousTransform A2) {
    float matrix[4][4] = {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0}};
    for (int m = 0; m < 4; m++) {
        for (int r = 0; r < 4; r++) {
            for (int k = 0; k < 4; k++) {
                matrix[m][r] += A1.get(m, k) * A2.get(k, r);
            }
        }
    }
    return HomogeneousTransform(matrix);
}

HomogeneousTransform transpose(HomogeneousTransform A) {
    float matrix[4][4] = {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0}};

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            matrix[c][r] = A.get(r, c);
        }
    }
    return HomogeneousTransform(matrix);
}

void printMatrix(HomogeneousTransform A) {
    Serial.println("A -> ");
    for (int r = 0; r < 4; r++) {
        Serial.print("|");
        for (int c = 0; c < 4; c++) {
            Serial.print(" ");
            Serial.print(A.get(r, c), DECIMALPRECISION);
            Serial.print(" ");
        }
        Serial.println("|");
    }
}