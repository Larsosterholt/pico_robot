#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(int motorPin1, int motorPin2, int motorSpeedPin, float wheelCircumference)
: _motorPin1(motorPin1), 
_motorPin2(motorPin2), 
_motorSpeedPin(motorSpeedPin), 
_wheelCircumference(wheelCircumference) {}

void MotorDriver::initialize() {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    digitalWrite(_motorPin1, HIGH);
    digitalWrite(_motorPin2, LOW);

    pinMode(_motorSpeedPin, OUTPUT);
}

void MotorDriver::setSpeed(float speed) {


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

    Serial.print("Speed set to");
    Serial.println(speed);

}

void MotorDriver::updateSpeed(float speed) {
    _wheelSpeed = speed;
}
