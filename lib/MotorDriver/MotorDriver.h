#ifndef MotorDriver_h
#define MotorDriver_h



class MotorDriver {
public:
    MotorDriver(int motorPin1, int motorPin2, int motorSpeedPin, float wheelCircumference);
    void initialize(float pGain);
    void setSpeed(int speed);
    void updateSpeed(float speed);
    void setSpeedSetPoint(int speed);

private:
    int _motorPin1, _motorPin2, _motorSpeedPin;
    float _wheelCircumference, _wheelSpeed, _pGain = 0;
    
};

#endif
