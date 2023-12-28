#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(int motorPin1, int motorPin2, int motorSpeedPin, float wheelCircumference)
: _motorPin1(motorPin1), 
_motorPin2(motorPin2), 
_motorSpeedPin(motorSpeedPin), 
_wheelCircumference(wheelCircumference) {}

void MotorDriver::initialize(float pGain) {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    digitalWrite(_motorPin1, HIGH);
    digitalWrite(_motorPin2, LOW);

    pinMode(_motorSpeedPin, OUTPUT);
    _pGain = pGain;
}

void MotorDriver::setSpeed(int speed) {

    if(speed >= 0){
        digitalWrite(_motorPin1, HIGH);
        digitalWrite(_motorPin2, LOW);
        analogWrite(_motorSpeedPin, abs(speed));  
    } 
    else {
        digitalWrite(_motorPin1, !HIGH);
        digitalWrite(_motorPin2, !LOW);
        analogWrite(_motorSpeedPin, abs(speed));  
    }

    //Serial.print("Speed set to");
    //Serial.println(speed);

}

void MotorDriver::updateSpeed(float speed) {
    _wheelSpeed = speed;
}


void MotorDriver::setSpeedSetPoint(int speed){
    float diff = (speed - _wheelSpeed);
    int pwmSetSetPoint = (int)(diff*_pGain);
    
    setSpeed(pwmSetSetPoint);
}