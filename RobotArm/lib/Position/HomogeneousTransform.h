#ifndef HomogeneousTransform_h
#define HomogeneousTransform_h
#include <RotationMatrix.h>
#include <DistanceVector.h>
#include <math.h>
#include <Arduino.h>
#include <Utils.h>
class HomogeneousTransform {
   private:
    float row0[4];
    float row1[4];
    float row2[4];
    float row3[4];

   public:
    HomogeneousTransform(float r0[], float r1[], float r2[], float r3[]) {
        for (int i = 0; i < 4; i++) {
            row0[i] = r0[i];
            row1[i] = r1[i];
            row2[i] = r2[i];
            row3[i] = r3[i];
        }
    }
    HomogeneousTransform(RotationMatrix R, DistanceVector r) {
        for (int c = 0; c < 2; c++) {
            row0[c] = R.get(0, c);
            row1[c] = R.get(1, c);
            row2[c] = R.get(2, c);
            row3[c] = 0.0;
        }
        row0[3] = r.x;
        row1[3] = r.y;
        row2[3] = r.z;
        row3[3] = 1.0;
    }

    float get(int r, int c) {
        if (r == 0) {
            return row0[c];
        } else if (r == 1) {
            return row1[c];
        } else if (r == 2) {
            return row2[c];
        } else if (r == 3) {
            return row3[c];
        } else {
            Serial.println("Invalid matrix index given");
            return INFINITY;
        }
    }

    void printMatrix() {
        Serial.println("A -> ");
        for (int r = 0; r < 4; r++) {
            Serial.print("|");
            for (int c = 0; c < 4; c++) {
                Serial.print(get(r, c));
            }
            Serial.println("|");
        }
    }
};

#endif