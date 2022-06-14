#ifndef DistanceVector_h
#define DistanceVector_h
class DistanceVector {
   public:
    float x;
    float y;
    float z;
    DistanceVector(float _x, float _y, float _z) {
        x = _x;
        y = _y;
        z = _z;
    }
};
#endif