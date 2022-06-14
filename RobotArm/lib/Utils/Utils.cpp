#include <Arduino.h>
#include <Utils.h>
#include <math.h>
#include <RotationMatrix.h>
#include <HomogeneousTransform.h>

const float stepsPerRev = 800.0;

void initSerialMonitor() {
    Serial.begin(9600);
    Serial.flush();
}

long getDelayFromSpeed(long s) {
    // Converting ticks per second into a delay timing
    long microseconds = 1000000 / s;
    return microseconds;
}

void waitForSerialInput() {
    while (Serial.available() == 0) {
        delay(50);
    }
}

int degreeToSteps(float degree) {
    // spr / 360 = steps per degree * degree
    return floorf((stepsPerRev / 360.0) * degree);
}

RotationMatrix multiply(RotationMatrix R1, RotationMatrix R2) {
    float r0[] = {0.0, 0.0, 1.0};
    float r1[] = {0.0, -1.0, 0.0};
    float r2[] = {1.0, 0.0, 0.0};
    for (int m = 0; m < 3; i++) {
        for (int r = 0; r < 3; j++) {
            for (int k = 0; k < 3; k++) {
                if (m == 0) {
                    r0[j] += R1.get(m, k) * R2.get(k, r);
                } else if (m == 1) {
                    r1[j] += R1.get(m, k) * R2.get(k, r);
                } else if (m == 2) {
                    r2[j] += R1.get(m, k) * R2.get(k, r);
                }
            }
        }
    }
    return RotationMatrix(r0, r1, r2);
}

HomogeneousTransform multiply(HomogeneousTransform A1, HomogeneousTransform A2) {
    float r0[] = {0.0, 0.0, 0.0, 0.0};
    float r1[] = {0.0, 0.0, 0.0, 0.0};
    float r2[] = {0.0, 0.0, 0.0, 0.0};
    float r3[] = {0.0, 0.0, 0.0, 0.0};
    for (int m = 0; m < 4; i++) {
        for (int r = 0; r < 4; j++) {
            for (int k = 0; k < 4; k++) {
                if (m == 0) {
                    r0[j] += A1.get(m, k) * A2.get(k, r);
                } else if (m == 1) {
                    r1[j] += A1.get(m, k) * A2.get(k, r);
                } else if (m == 2) {
                    r2[j] += A1.get(m, k) * A2.get(k, r);
                } else if (m == 3) {
                    r3[j] += A1.get(m, k) * A2.get(k, r);
                }
            }
        }
    }
    return HomogeneousTransform(r0, r1, r2, r3);
}