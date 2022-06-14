#ifndef LinkMotor_h
#define LinkMotor_h

// TODO: Docs

class LinkMotor {
   private:
    // Constructor Arguments
    int nemaMotorType;
    int linkNumber;
    int stepPin;
    int dirPin;
    int limitSwitchPin;
    // Control Variables
    bool currentDir;  // True -> CW, False -> CCW
    int current;
    int target;
    int previous;
    long previousChangeTime;
    bool currentlyRunning;
    long maxSpeed;
    int currentSpeed;
    long currentDelay;

   public:
    LinkMotor(int _linkNumber, int _nemaMotorType, int _stepPin, int _dirPin, int _limitSwitchPin) {
        linkNumber = _linkNumber;
        nemaMotorType = _nemaMotorType;
        stepPin = _stepPin;
        dirPin = _dirPin;
        limitSwitchPin = _limitSwitchPin;
    }

    void init();

    int calibrate();

    int getSpeed();

    int getDelay();

    int getLimitSwitch();

    void setSpeed(long speed);

    void setDirection(bool CW);

    void setTarget(int targetStep);

    void stepMotor();

    void moveTo(int targetStep);

    void jointAngle(float degrees);

    void update();

    bool isMoving();

    bool waitToStop();
};
#endif