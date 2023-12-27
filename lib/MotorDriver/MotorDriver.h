#ifndef MotorDriver_h
#define MotorDriver_h

#include "Encoder.h"

class MotorDriver {
public:
    MotorDriver(int motorPin1, int motorPin2, Encoder* encoder);
    void initialize();
    void setSpeed(int speed);
    void update();

private:
    int _motorPin1, _motorPin2;
    Encoder* _encoder;
};

#endif
