#ifndef HomogeneousTransform_h
#define HomogeneousTransform_h
#include <Arduino.h>
#include <RotationMatrix.h>
#include <Vector.h>
#include <Utils.h>

/**
 * @brief Class definition for a 4x4 matrix representing the
 * Homogeneous Transformation matrix built with a 3x3 Rotation Matrix and a 3x1 distance vector
 *
 */
class HomogeneousTransform {
   private:
    float m[4][4];

   public:
    HomogeneousTransform(float _m[4][4]) {
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                m[r][c] = _m[r][c];
            }
        }
    }

    HomogeneousTransform(RotationMatrix R, Vector r) {
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                m[r][c] = R.get(r, c);
            }
        }
        m[0][3] = r.x;
        m[1][3] = r.y;
        m[2][3] = r.z;
        m[3][3] = 1.0;
    }

    float get(int r, int c) {
        if (r < 0 || r > 3 || c < 0 || c > 3) {
            Serial.println("Invalid matrix index given");
            return INFINITY;
        }
        return m[r][c];
    }
};

#endif