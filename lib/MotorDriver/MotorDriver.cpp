#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(int motorPin1, int motorPin2, int motorPWMPin, float wheelCircumference)
    : _motorPin1(motorPin1),
      _motorPin2(motorPin2),
      _motorPWMPin(motorPWMPin),
      _wheelCircumference(wheelCircumference) {}

void MotorDriver::initialize(float pGain, float iGain, float dGain) {

    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    digitalWrite(_motorPin1, HIGH);
    digitalWrite(_motorPin2, LOW);

    pinMode(_motorPWMPin, OUTPUT);
    analogWrite(_motorPWMPin, 0);
    _pGain = pGain;
    _iGain = iGain;
        _dGain = dGain;

}

void MotorDriver::setSpeed(int speed)
{

    if (speed >= 0)
    {
        digitalWrite(_motorPin1, HIGH);
        digitalWrite(_motorPin2, LOW);
        analogWrite(_motorPWMPin, abs(speed));
    }
    else
    {
        digitalWrite(_motorPin1, !HIGH);
        digitalWrite(_motorPin2, !LOW);
        analogWrite(_motorPWMPin, abs(speed));
    }

    // Serial.print("Speed set to");
    // Serial.println(speed);
}

void MotorDriver::updateSpeed(float speed)
{
    _wheelSpeed = speed;
}

void MotorDriver::setSpeedSetPoint(int speed) {
    float error = speed - _wheelSpeed;
    _integral += error;
    
    // Anti-windup: Clamp the integral term
    if (_integral > maxIntegral) _integral = maxIntegral;
    if (_integral < -maxIntegral) _integral = -maxIntegral;

    float derivative = error - _lastError;
    float control = _pGain * error + _iGain * _integral + _dGain * derivative;
    setSpeed((int)control);

    _lastError = error;
}

float MotorDriver::getCurrentSpeed()
{
    return _wheelSpeed;
}

void MotorDriver::setProportionalGain(float gain)
{
    _pGain = gain;
}

float MotorDriver::getProportionalGain()
{
    return _pGain;
}

void MotorDriver::setIntegralGain(float gain) {
    _iGain = gain;
}

float MotorDriver::getIntegralGain() {
    return _iGain;
}

void MotorDriver::setDerivativeGain(float gain) {
    _dGain = gain;
}

float MotorDriver::getDerivativeGain() {
    return _dGain;
}