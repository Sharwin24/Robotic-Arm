#ifndef JointAngles_h
#define JointAngles_h

// Class to group each joint angle together (in degrees)
class JointAngles {
   public:
    float q1;
    float q2;
    float q3;
    JointAngles(float _q1, float _q2, float _q3) {
        q1 = _q1;
        q2 = _q2;
        q3 = _q3;
    }
};
#endif
