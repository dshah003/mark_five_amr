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


// Global Variables
long prevT = 0;

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


// Constants
const float WHEEL_RADIUS = 0.055;  // 5 cm radius
const float WHEEL_BASE = 0.142;     // 20 cm distance between wheels
const float TICKS_PER_REV = 540;  // Number of encoder ticks per revolution
const float WHEEL_CIRCUMFERENCE = 2 * 3.14159 * WHEEL_RADIUS;  // 2pir

Motor leftMotor(leftMotorIn1, leftMotorIn2, leftMotorEnable, leftEncoderPinA, leftEncoderPinB);
Motor rightMotor(rightMotorIn1, rightMotorIn2, rightMotorEnable, rightEncoderPinA, rightEncoderPinB);

// Variables to store previous encoder positions and times
long prevLeftTicks = 0;
long prevRightTicks = 0;
unsigned long prevTime = 0;

int targetSpeedLeft = 0;
int targetSpeedRight = 0;
float linearVel = 0.0;
float angularVel = 0.0;

void setup() {
  Serial.begin(9600);

  leftMotor.setPIDParams(1.25, 0.025, 0);
  rightMotor.setPIDParams(1.25, 0.025, 0);
}

void loop() {

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert to seconds

  // Read the current encoder positions
  long leftTicks = leftMotor.getEncoderCount();
  long rightTicks = rightMotor.getEncoderCount();

  Serial.print("Left Tick: "); Serial.print(leftTicks);

  // Calculate the change in ticks
  long deltaLeftTicks = leftTicks - prevLeftTicks;
  long deltaRightTicks = rightTicks - prevRightTicks;

  Serial.print(" deltaLeftTicks: "); Serial.print(deltaLeftTicks);

  // Calculate wheel speeds (ticks per second)
  float leftSpeedTicksPerSec = deltaLeftTicks / deltaTime;
  float rightSpeedTicksPerSec = deltaRightTicks / deltaTime;

  Serial.print(" leftSpeedTicksPerSec: "); Serial.print(leftSpeedTicksPerSec);
  // Convert to linear velocity
  float actualSpeedLeft = ticksToVelocity(leftSpeedTicksPerSec);
  float actualSpeedRight = ticksToVelocity(rightSpeedTicksPerSec);

  Serial.print("\n  actualSpeed "); Serial.print(actualSpeedLeft);


  if (Serial.available()) 
  {
    String input = Serial.readStringUntil('\n');  // Read input until newline character
    input.trim();  // Remove any leading/trailing whitespace

    // Split the input string by the comma
    int commaIndex = input.indexOf(',');
    if (commaIndex != -1) {
      String firstValue = input.substring(0, commaIndex);
      String secondValue = input.substring(commaIndex + 1);

      // Convert strings to float
      linearVel = firstValue.toFloat();
      angularVel = secondValue.toFloat();

      // Calculate Wheel velocity
      float vRight = linearVel + (angularVel * (WHEEL_BASE / 2.0)); 
      float vLeft = linearVel - (angularVel * (WHEEL_BASE / 2.0)); 
      Serial.print(" vLeft: "); Serial.print(vLeft);

      // Convert to wheel speeds (ticks per second)
      targetSpeedLeft = velocityToTicksPerSecond(vLeft);
      targetSpeedRight = velocityToTicksPerSecond(vRight);

      Serial.print(" targetSpeedLeft: "); Serial.print(targetSpeedLeft);
 
      // Serial.print(rightWheelSpeed);
      // Serial.print(" ");
      // Serial.print(leftWheelSpeed);
        // Serial.println();
    } 
  }

  // Adjust PWM based on actual speed and target speed (Simple P controller)
  int pwmLeft = adjustPWM(targetSpeedLeft, leftSpeedTicksPerSec);
  int pwmRight = adjustPWM(targetSpeedRight, rightSpeedTicksPerSec);

  Serial.print(" pwmLeft: "); Serial.println(pwmLeft);
  // Set motor directions and speeds
  leftMotor.setMotorSpeed(pwmLeft);
  rightMotor.setMotorSpeed(pwmRight);

  // Update previous values for the next loop
  prevLeftTicks = leftTicks;
  prevRightTicks = rightTicks;
  prevTime = currentTime;

  // Optionally, print out debug information
  Serial.print("Left Speed: "); Serial.print(actualSpeedLeft);
  Serial.print(" m/s, Right Speed: "); Serial.println(actualSpeedRight);

  delay(1000);  // Delay for stability
  
  
  // rightMotor.setMotorSpeedWithPID(rightWheelSpeed);
  // leftMotor.setMotorSpeedWithPID(leftWheelSpeed);
  // Serial.print(leftMotor.getCurrentVelocity());
  // Serial.print(" ");
  // Serial.println(rightMotor.getCurrentVelocity());
  // delay(100);

}

// Function to convert velocity to ticks per second
int velocityToTicksPerSecond(float velocity) {
  return (velocity / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV;
}

// Function to convert ticks per second to linear velocity
float ticksToVelocity(float ticksPerSecond) {
  return (ticksPerSecond / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
}

// Simple P-controller for PWM adjustment
float adjustPWM(float targetSpeed, float actualSpeed) {
  float error = targetSpeed - actualSpeed;
  float Kp = 0.5;  // Proportional gain, tune this value
  int pwm = error * Kp;
  return constrain(pwm, -255, 255);
}