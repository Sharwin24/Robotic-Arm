#ifndef Vector_h
#define Vector_h
class Vector {
   public:
    float x;
    float y;
    float z;
    Vector(float _x, float _y, float _z) {
        x = _x;
        y = _y;
        z = _z;
    }
};
#endif