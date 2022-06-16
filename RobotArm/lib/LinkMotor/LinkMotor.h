#ifndef LinkMotor_h
#define LinkMotor_h

// TODO: Docs

class LinkMotor {
   private:
    // Constructor Arguments
    int linkNumber;
    int stepPin;
    int dirPin;
    int limitSwitchPin;
    float outputGearRatio;
    // Control Variables
    bool currentDir;  // True -> CW, False -> CCW
    int current;
    float currentAngle;
    int target;
    int previous;
    long previousChangeTime;
    bool currentlyRunning;
    long maxSpeed;
    float currentSpeed;
    long currentDelay;

   public:
    LinkMotor(int _linkNumber, int _stepPin, int _dirPin, int _limitSwitchPin, float _outputGearRatio) {
        linkNumber = _linkNumber;
        stepPin = _stepPin;
        dirPin = _dirPin;
        limitSwitchPin = _limitSwitchPin;
        outputGearRatio = _outputGearRatio;
    }

    LinkMotor(int _linkNumber, int _stepPin, int _dirPin, int _limitSwitchPin = -1) {
        linkNumber = _linkNumber;
        stepPin = _stepPin;
        dirPin = _dirPin;
        limitSwitchPin = _limitSwitchPin;
        outputGearRatio = 1.0;
    }

    void init();

    int calibrate();

    float getSpeed();

    int getDelay();

    int getLimitSwitch();

    float getAngle();

    void setSpeed(float speed);

    void setDirection(bool CW);

    void setTarget(int targetStep);

    void stepMotor();

    void moveTo(int targetStep);

    void moveToAngle(float degrees);

    void update();

    bool isMoving();

    bool waitToStop();
};
#endif