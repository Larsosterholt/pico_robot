#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(int motorPin1, int motorPin2, Encoder* encoder)
: _motorPin1(motorPin1), _motorPin2(motorPin2), _encoder(encoder) {}

void MotorDriver::initialize() {
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
    // Initialize other motor-related setups
}

void MotorDriver::setSpeed(int speed) {
    Serial.print("Speed set to");
    Serial.println(speed);
}

void MotorDriver::update() {
    // You can use _encoder->getCount() here to get the encoder count
    // and use it for feedback control or other purposes
}
