/**
   @file Robot_Controller.ino
   @author Darshan Shah
   @brief This code takes target distance in terms of encoder counts and moves the robot wheels 
          accordingly using PID feedback control.
   @version 0.1
   @date 2024-08-04

   @copyright Copyright (c) 2024

*/

#include <Motor.h>
#include <PID_Controller.h>
#include <util/atomic.h>

// Global Variables
long prevT = 0;
volatile int posInt[] = {0, 0};

// Pin Configuration
const int leftEncoderPinA = 21;
const int leftEncoderPinB = 20;

const int leftMotorIn1 = 22;
const int leftMotorIn2 = 23;
const int leftMotorEnable = 7;

const int rightEncoderPinA = 3;
const int rightEncoderPinB = 2;

const int rightMotorIn1 = 24;
const int rightMotorIn2 = 25;
const int rightMotorEnable = 6;

const int encoderPinA[] = {leftEncoderPinA, rightEncoderPinA};

PID_Controller leftPID;
PID_Controller rightPID;
// TODO: Read these from configs.
Motor leftMotor(leftMotorIn1, leftMotorIn2, leftMotorEnable, leftEncoderPinA, leftEncoderPinB);
Motor rightMotor(rightMotorIn1, rightMotorIn2, rightMotorEnable, rightEncoderPinA, rightEncoderPinB);

void setup() {
  Serial.begin(9600);

  leftPID.setParams(1.25, 0.025, 0, 255);
  rightPID.setParams(1.25, 0.025, 0, 255);

  attachInterrupt(digitalPinToInterrupt(encoderPinA[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[1]), readEncoder<1>, RISING);

}

void loop() {
  int targetL = 1000;
  int targetR = 1000;

  // Estimate time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  int posL;
  int posR;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posL = posInt[0];
    posR = posInt[1];
  }

  int powerL = 0;
  int powerR = 0;
  int dirL = 0;
  int dirR = 0;

  leftPID.evaluate(posL, targetL, deltaT, powerL, dirL);
  rightPID.evaluate(posR, targetR, deltaT, powerR, dirR);

  leftMotor.setMotor(dirL, powerL);
  rightMotor.setMotor(dirR, powerR);

  Serial.print(targetL);
  Serial.print(" ");
  Serial.print(posL);
  Serial.print(" ");
  Serial.print(targetR);
  Serial.print(" ");
  Serial.print(posR);
  Serial.println();

}

template <int j>
void readEncoder() {
  int b = digitalRead(encoderPinA[j]);
  if (b > 0) {
    posInt[j]++;
  }
  else {
    posInt[j]--;
  }
}
