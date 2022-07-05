#ifndef RotationMatrix_h
#define RotationMatrix_h
#include <Arduino.h>
#include <math.h>
#include <Utils.h>

/**
 * @brief Class definition for a 3x3 matrix representing the
 * Rotation Matrix describing a rotation around an axis
 *
 */
class RotationMatrix {
   private:
    char rotationType;  // valid types -> 'x', 'y', 'z'
    float theta;        // theta is stored in radians
    float m[3][3];

   public:
    /**
     * @brief Default Constructor creates Identity Matrix
     *
     */
    RotationMatrix() {
        float I3[3][3] = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}};
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                m[r][c] = I3[r][c];
            }
        }
    }

    /**
     * @brief Construct a Rotation Matrix using a given 3x3 matrix
     *
     * @param _m 3x3 matrix of type float
     */
    RotationMatrix(float _m[3][3]) {
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                m[r][c] = _m[r][c];
            }
        }
    }
    /**
     * @brief Construct a Rotation Matrix using a rotation type
     * and a given angle
     *
     * @param rotation character representing the axis of rotation
     * @param _theta angle in degrees
     */
    RotationMatrix(char rotation, float _theta) {
        rotationType = rotation;
        // theta = _theta;
        theta = radians(_theta);
        if (rotationType == 'x') {
            float Rx[3][3] = {
                {1.0, 0.0, 0.0},
                {0.0, cosf(theta), -sinf(theta)},
                {0.0, sinf(theta), cosf(theta)}};
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    m[r][c] = Rx[r][c];
                }
            }
        } else if (rotationType == 'y') {
            float Ry[3][3] = {
                {cosf(theta), 0.0, sinf(theta)},
                {0.0, 1.0, 0.0},
                {-sinf(theta), 0.0, cosf(theta)}};
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    m[r][c] = Ry[r][c];
                }
            }
        } else if (rotationType == 'z') {
            float Rz[3][3] = {
                {cosf(theta), -sinf(theta), 0.0},
                {sinf(theta), cosf(theta), 0.0},
                {0.0, 0.0, 1.0}};
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    m[r][c] = Rz[r][c];
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