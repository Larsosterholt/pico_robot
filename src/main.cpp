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
const int countsPerRev = 12 * 90;
const float wheelCircumference = 3.1415 * (65);
unsigned long lastTime = 0;
const unsigned long loopInterval = 50;
volatile long rightLastEncoderPos = 0;
volatile long leftLastEncoderPos = 0;

MotorDriver rightMotor(16, 17, 18, wheelCircumference);
MotorDriver leftMotor(14, 15, 13, wheelCircumference);

int targetLeftSpeed = 0;
int targetRightSpeed = 0;

float alpha = 0.05;  // Example value, adjust as needed for your filter strength
float lastFilteredRightSpeed = 0;
float lastFilteredLeftSpeed = 0;

void RightUpdateEncoderISR()
{
  int MSB = digitalRead(rightEncoderPinA); // MSB = most significant bit
  int LSB = digitalRead(rightEncoderPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB;
  int sum = (lightLastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    rightEncoderPos++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    rightEncoderPos--;

  lightLastEncoded = encoded;
}

void LeftUpdateEncoderISR()
{
  int MSB = digitalRead(leftEncoderPinA); // MSB = most significant bit
  int LSB = digitalRead(leftEncoderPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB;
  int sum = (leftLastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    leftEncoderPos++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    leftEncoderPos--;

  leftLastEncoded = encoded;
}

void parseCommand(String command) {

  command.trim();
  if (command.startsWith("L") && command.indexOf("R") > -1) {
    int rIndex = command.indexOf("R");
    targetLeftSpeed = command.substring(1, rIndex).toInt();
    targetRightSpeed = command.substring(rIndex + 1).toInt();
  }
  else if (command.startsWith("G")) {
    float controllerGain = command.substring(1).toFloat();

    rightMotor.setProportionalGain(controllerGain);
    leftMotor.setProportionalGain(controllerGain);
  }
  else if (command.equals("GETSPEED"))
  {
    Serial.print("L");
    Serial.print(leftMotor.getCurrentSpeed()); // Assuming your motor driver can return current speed
    Serial.print(" R");
    Serial.println(rightMotor.getCurrentSpeed());
  }
}

void updateMotorSpeeds()
{
  // Here you might need to convert integer speed to the motor's speed unit if necessary
  leftMotor.setSpeedSetPoint(targetLeftSpeed);
  rightMotor.setSpeedSetPoint(targetRightSpeed);
}

void updateWheelSpeed() {
  long interval = millis() - lastTime;

  // Right motor speed calculation
  float rightSpeedRaw = (1000.0 * (rightEncoderPos - rightLastEncoderPos) / interval) * wheelCircumference / countsPerRev;
  float filteredRightSpeed = alpha * rightSpeedRaw + (1 - alpha) * lastFilteredRightSpeed;
  rightMotor.updateSpeed(filteredRightSpeed);
  rightLastEncoderPos = rightEncoderPos;
  lastFilteredRightSpeed = filteredRightSpeed;

  // Left motor speed calculation
  float leftSpeedRaw = (1000.0 * (leftEncoderPos - leftLastEncoderPos) / interval) * wheelCircumference / countsPerRev;
  float filteredLeftSpeed = alpha * leftSpeedRaw + (1 - alpha) * lastFilteredLeftSpeed;
  leftMotor.updateSpeed(filteredLeftSpeed);
  leftLastEncoderPos = leftEncoderPos;
  lastFilteredLeftSpeed = filteredLeftSpeed;

  lastTime = millis();
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Setup");

  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), RightUpdateEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinB), RightUpdateEncoderISR, CHANGE);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), LeftUpdateEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinB), LeftUpdateEncoderISR, CHANGE);
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);

  leftMotor.initialize(4.0);
  rightMotor.initialize(4.0);
}

void loop()
{

  if (millis() - lastTime >= loopInterval)
  {
    updateWheelSpeed();


/*
  Serial.print(">Right_speed: ");
  Serial.println(rightMotor.getCurrentSpeed());

  Serial.print(">Left_speed: ");
  Serial.println(leftMotor.getCurrentSpeed());

  Serial.print(">Left_SP: ");
  Serial.println(targetLeftSpeed);

  Serial.print(">Right_SP: ");
  Serial.println(targetRightSpeed);

  Serial.print(">P_gain: ");
  Serial.println(rightMotor.getProportionalGain());
*/
  Serial.println(".");
  }

  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
  }

  updateMotorSpeeds();


}
