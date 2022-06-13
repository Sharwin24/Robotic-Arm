#ifndef Utils_h
#define Utils_h
long getDelayFromSpeed(long s);

void waitForSerialInput();

void writeMsg(String msg);

// Steps per revolution: 400 (set on driver)
// Steps per revolution (SPR) -> angle (Degrees)
// 400 / 360 = 1.11 steps per degree -> approx 1 step per degree

#endif