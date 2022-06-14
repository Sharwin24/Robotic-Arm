#ifndef RotationMatrix_h
#define RotationMatrix_h
#include <Arduino.h>
#include <math.h>
#include <Utils.h>
class RotationMatrix {
   private:
    char rotationType;
    float theta;
    float row0[3];
    float row1[3];
    float row2[3];

   public:
    RotationMatrix(float r0[3], float r1[3], float r2[3]) {
        for (int i = 0; i < 3; i++) {
            row0[i] = r0[i];
            row1[i] = r1[i];
            row2[i] = r2[i];
        }
    }
    RotationMatrix(char rotation, float _theta) {
        rotationType = rotation;
        theta = _theta;
        if (rotationType == 'x') {
            float r0[] = {1.0, 0.0, 0.0};
            float r1[] = {0.0, cosf(theta), -sinf(theta)};
            float r2[] = {0.0, sinf(theta), cosf(theta)};
            for (int i = 0; i < 3; i++) {
                row0[i] = r0[i];
                row1[i] = r1[i];
                row2[i] = r2[i];
            }
        } else if (rotationType == 'y') {
            float d0[] = {cosf(theta), 0.0, sinf(theta)};
            float d1[] = {0.0, 1.0, 0.0};
            float d2[] = {-sinf(theta), 0.0, cosf(theta)};
            for (int i = 0; i < 3; i++) {
                row0[i] = d0[i];
                row1[i] = d1[i];
                row2[i] = d2[i];
            }
        } else if (rotationType == 'z') {
            float x0[] = {cosf(theta), -sinf(theta), 0.0};
            float x1[] = {sinf(theta), cosf(theta), 0.0};
            float x2[] = {0.0, 0.0, 1.0};
            for (int i = 0; i < 3; i++) {
                row0[i] = x0[i];
                row1[i] = x1[i];
                row2[i] = x2[i];
            }
        } else {
            Serial.println("Invalid rotation type given");
        }
    }

    float get(int r, int c) {
        if (r == 0) {
            return row0[c];
        } else if (r == 1) {
            return row1[c];
        } else if (r == 2) {
            return row2[c];
        } else {
            Serial.println("Invalid matrix index given");
            return INFINITY;
        }
    }

    void printMatrix() {
        String msg = "R";
        msg.concat(rotationType);
        msg.concat("( ");
        msg.concat(theta);
        msg.concat(") -> ");
        Serial.println(msg);
        for (int r = 0; r < 3; r++) {
            Serial.print("|");
            for (int c = 0; c < 3; c++) {
                Serial.print(get(r, c));
            }
            Serial.println("|");
        }
    }
};
#endif