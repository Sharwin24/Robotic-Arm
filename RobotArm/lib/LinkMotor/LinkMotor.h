#ifndef LinkMotor_h
#define LinkMotor_h

// Documentaiton for methods in LinkMotor.cpp
// TODO: Implement ROM (Range-of-Motion)

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
    float minROM;  // Range-of-Motion min/max in degrees
    float maxROM;

   public:
    // Constructors for creating a LinkMotor object. Needs to be assigned to a
    // link number, step/direction pins, a limit switch pin, and output gear ratio
    LinkMotor(int _linkNumber, int _stepPin, int _dirPin, int _limitSwitchPin, float _outputGearRatio) {
        linkNumber = _linkNumber;
        stepPin = _stepPin;
        dirPin = _dirPin;
        limitSwitchPin = _limitSwitchPin;
        outputGearRatio = _outputGearRatio;
        minROM = 0;
        maxROM = 0;
    }

    LinkMotor(int _linkNumber, int _stepPin, int _dirPin, int _limitSwitchPin = -1) {
        linkNumber = _linkNumber;
        stepPin = _stepPin;
        dirPin = _dirPin;
        limitSwitchPin = _limitSwitchPin;
        outputGearRatio = 1.0;
        minROM = 0;
        maxROM = 0;
    }

    LinkMotor(int _linkNumber, int _stepPin, int _dirPin, int _limitSwitchPin, float _outputGearRatio, float _minROM, float _maxROM) {
        linkNumber = _linkNumber;
        stepPin = _stepPin;
        dirPin = _dirPin;
        limitSwitchPin = _limitSwitchPin;
        outputGearRatio = _outputGearRatio;
        minROM = _minROM;
        maxROM = _maxROM;
    }

    void init();

    int calibrate();

    float getTarget();
    float getSpeed();
    long getDelay();
    float getGR();
    float getAngle();
    int getLimitSwitch();

    void updateAngle();

    void setSpeed(float speed);

    void setDirection(bool CW);

    void setTarget(int targetStep);

    void stepMotor();

    void moveTo(int targetStep);

    void moveToAngle(float targetAngle);

    void update();

    bool isMoving();

    bool waitToStop();
};
#endif