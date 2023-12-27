#include <Arduino.h>
#include "MotorDriver.h"

// /media/lars/RPI-RP2

// Right motor global varibles
const int rightEncoderPinA = 19;
const int rightEncoderPinB = 20;
volatile long rightEncoderPos = 0;
bool lightLastEncoded = LOW;

// Left motor global varibles
const int leftEncoderPinA = 21;
const int leftEncoderPinB = 22;
volatile long leftEncoderPos = 0;
bool leftLastEncoded = LOW;

// "Defines"
const int countsPerRef = 12;
//const int loopTime = 10;
const float wheelCircumference = 3.1415*(65);

void RightUpdateEncoderISR() {
  int MSB = digitalRead(rightEncoderPinA); // MSB = most significant bit
  int LSB = digitalRead(rightEncoderPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; 
  int sum = (lightLastEncoded << 2) | encoded; 

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) rightEncoderPos++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) rightEncoderPos--;

  lightLastEncoded = encoded; 
}

void LeftUpdateEncoderISR() {
  int MSB = digitalRead(leftEncoderPinA); // MSB = most significant bit
  int LSB = digitalRead(leftEncoderPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; 
  int sum = (leftLastEncoded << 2) | encoded; 

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) leftEncoderPos++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) leftEncoderPos--;

  leftLastEncoded = encoded; 
}


void setup() {
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), RightUpdateEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinB), RightUpdateEncoderISR, CHANGE);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), LeftUpdateEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinB), LeftUpdateEncoderISR, CHANGE);
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  Serial.begin(9600);
}





void loop() {
  unsigned long lastTime = 0;
  const unsigned long loopInterval = 50;
  volatile long rightLastEncoderPos = 0;
  volatile long leftLastEncoderPos = 0;

  //Serial.print("Right wheel: ");
  //Serial.println(rightEncoderPos);
  //Serial.print("Left wheel: ");
  //Serial.println(leftEncoderPos);
  //delay(100);

  MotorDriver rightMotor(16, 17, 18, wheelCircumference);
  rightMotor.initialize();

  MotorDriver leftMotor(14, 15, 13, wheelCircumference);
  leftMotor.initialize();

  if (millis() - lastTime >= loopInterval) {
    long interval = millis() - lastTime;

    // Right motor speed calculation
    float rightSpeed = (1000.0 * (rightEncoderPos - rightLastEncoderPos) / interval) * wheelCircumference;
    rightMotor.updateSpeed(rightSpeed);
    rightLastEncoderPos = rightEncoderPos;
    Serial.print("Right wheel speed: ");
    Serial.println(rightSpeed, 5);

    // Left motor speed calculation
    float leftSpeed = (1000.0 * (leftEncoderPos - leftLastEncoderPos) / interval) * wheelCircumference;
    leftMotor.updateSpeed(leftSpeed);
    leftLastEncoderPos = leftEncoderPos;

    lastTime = millis();
  }

}
