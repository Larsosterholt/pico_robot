#include "Encoder.h"
#include "MotorDriver.h"

void encoderISR() {
    Encoder::getInstance().handleInterrupt();
}

  const int RightMotorPin_1 = 1;
  const int RightMotorPin_2 = 2;

  const int RightEncoderPin_1 = 21;
  const int RightEncoderPin_2 = 22;


void setup() {
    Encoder& encoder = Encoder::getInstance(RightEncoderPin_1, RightEncoderPin_2);
    encoder.initialize();
    //encoder.attachInterrupt();
    
    MotorDriver motorDriver(RightMotorPin_1, RightMotorPin_2, &encoder);
    motorDriver.initialize();
    // Other setup code...
}

void loop() {

  // Update encoder speed regularly
  Encoder& encoderR = Encoder::getInstance();

  MotorDriver motorR(RightMotorPin_1, RightMotorPin_2, &encoderR);
  motorR.initialize();

  motorR.setSpeed(55);
}
