#ifndef RotationMatrix_h
#define RotationMatrix_h
#include <Arduino.h>
#include <math.h>
#include <Utils.h>
class RotationMatrix {
   private:
    char rotationType;
    float theta;
    float m[3][3];

   public:
    RotationMatrix(float _m[3][3]) {
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                m[r][c] = _m[r][c];
            }
        }
    }

    RotationMatrix(char rotation, float _theta) {
        rotationType = rotation;
        // theta = _theta;
        theta = radians(_theta);
        if (rotationType == 'x') {
            float matrix1[3][3] = {
                {1.0, 0.0, 0.0},
                {0.0, cosf(theta), -sinf(theta)},
                {0.0, sinf(theta), cosf(theta)}};
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    m[r][c] = matrix1[r][c];
                }
            }
        } else if (rotationType == 'y') {
            float matrix2[3][3] = {
                {cosf(theta), 0.0, sinf(theta)},
                {0.0, 1.0, 0.0},
                {-sinf(theta), 0.0, cosf(theta)}};
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    m[r][c] = matrix2[r][c];
                }
            }
        } else if (rotationType == 'z') {
            float matrix3[3][3] = {
                {cosf(theta), -sinf(theta), 0.0},
                {sinf(theta), cosf(theta), 0.0},
                {0.0, 0.0, 1.0}};
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    m[r][c] = matrix3[r][c];
                }
            }

        } else {
            Serial.println("Invalid rotation type given");
        }
    }

    float get(int r, int c) {
        if (r < 0 || r > 2 || c < 0 || c > 2) {
            Serial.println("Invalid matrix index given");
            return INFINITY;
        }
        return m[r][c];
    }

    char getRotType() { return rotationType; }
    float getTheta() { return degrees(theta); }
};
#endif