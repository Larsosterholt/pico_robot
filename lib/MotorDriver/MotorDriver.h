#ifndef MotorDriver_h
#define MotorDriver_h



class MotorDriver {
public:
    MotorDriver(int motorPin1, int motorPin2, int motorSpeedPin, float wheelCircumference);
    void initialize(float pGain, float iGain, float dGain);
    void setSpeed(int speed);
    void updateSpeed(float speed);
    void setSpeedSetPoint(int speed);
    float getCurrentSpeed();
    void setProportionalGain(float gain);
    float getProportionalGain();
    void setIntegralGain(float gain);
    float getIntegralGain();
    void setDerivativeGain(float gain);
    float getDerivativeGain();

private:
    int _motorPin1, _motorPin2, _motorPWMPin;
    float _wheelCircumference, _wheelSpeed, _pGain = 0,_iGain = 0, _integral  = 0, maxIntegral = 80;
    float _dGain = 0;  // Derivative gain
float _lastError = 0;  // Last error for derivative calculation

    
};

#endif
