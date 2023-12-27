#include "Encoder.h"
#include <Arduino.h>

Encoder::Encoder(int encoderPinA, int encoderPinB)
: _encoderPinA(encoderPinA), _encoderPinB(encoderPinB), _count(0), _lastCount(0), _lastTime(0), _speed(0.0) {}


Encoder& Encoder::getInstance(int encoderPinA, int encoderPinB) {
    static Encoder instance(encoderPinA, encoderPinB);
    return instance;
}

void Encoder::initialize() {
    pinMode(_encoderPinA, INPUT_PULLUP);
    pinMode(_encoderPinB, INPUT_PULLUP);
}

//void Encoder::attachInterrupt() {
//    ::attachInterrupt(digitalPinToInterrupt(_encoderPinA), encoderISR, CHANGE);
//}


void Encoder::handleInterrupt() {
    if (digitalRead(_encoderPinA) == digitalRead(_encoderPinB)) {
        increment();
        Serial.println("Incriment");
    } else {
        decrement();
        Serial.println("decrement");
    }
}

void Encoder::updateSpeed() {
    unsigned long currentTime = millis();
    long currentCount = _count;

    // Calculate time difference and count difference
    unsigned long timeDiff = currentTime - _lastTime;
    long countDiff = currentCount - _lastCount;

    // Avoid division by zero
    if (timeDiff > 0) {
        _speed = (float)countDiff / (float)timeDiff * 1000.0; // Converts to counts per second
    }

    // Update last time and count
    _lastTime = currentTime;
    _lastCount = currentCount;
}

float Encoder::getSpeed() {
    return _speed;
}
