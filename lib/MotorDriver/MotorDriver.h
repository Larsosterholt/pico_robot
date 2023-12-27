#ifndef MotorDriver_h
#define MotorDriver_h



class MotorDriver {
public:
    MotorDriver(int motorPin1, int motorPin2, int motorSpeedPin, float wheelCircumference);
    void initialize();
    void setSpeed(float speed);
    void updateSpeed(float speed);

private:
    int _motorPin1, _motorPin2, _motorSpeedPin;
    float _wheelCircumference, _wheelSpeed;
    
};

#endif
