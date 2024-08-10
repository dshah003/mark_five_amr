
#include <Encoder.h>

// Define motor control pins
const int leftMotor1 = 2;
const int leftMotor2 = 3;
const int leftEnable = A0;
const int rightMotor1 = 5;
const int rightMotor2 = 7;
const int rightEnable = A1;

// Constants for motor control
const int motorSpeed = 100;  // Set your desired motor speed
const float wheelDiameter = 0.055;  // Set your wheel diameter in meters
const float encoderCountsPerRev = 360.0;  // Set your encoder counts per revolution
const float trackWidth = 0.0145;  // Set the distance between the two tracks in meters
const float desiredAngle = 40.0;  // Desired turn angle in degrees

void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("Motor test!");
    pinMode(encoderPinLeft, INPUT);
    pinMode(encoderPinRight, INPUT);

    pinMode(leftMotor1, OUTPUT);
    pinMode(leftMotor2, OUTPUT);
    pinMode(rightMotor1, OUTPUT);
    pinMode(rightMotor2, OUTPUT);
    pinMode(leftEnable, OUTPUT);
    pinMode(rightEnable, OUTPUT);
    
    Encoder leftEncoder(encoderPin1, encoderPin2);
    Encoder rightEncoder(encoderPin3, encoderPin4);

    leftEncoder.write(0);
    rightEncoder.write(0);

}

// Function to move the robot forward (straight)
void moveForward() {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(leftEnable, abs(motorSpeed));
  analogWrite(rightEnable, abs(motorSpeed));
}

// Function to move the robot backward (reverse)
void moveReverse() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(leftEnable, abs(motorSpeed));
  analogWrite(rightEnable, abs(motorSpeed));
}

// Function to turn the robot left
void turnLeft() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(leftEnable, abs(motorSpeed));
  analogWrite(rightEnable, abs(motorSpeed));
}

// Function to turn the robot right
void turnRight() {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(leftEnable, abs(motorSpeed));
  analogWrite(rightEnable, abs(motorSpeed));
}

// Function to stop the robot
void stopRobot() {
  analogWrite(leftEnable, 0);
  analogWrite(rightEnable, 0);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
}

int getEncoderCountsForDistance(float distance) {
    return (distance / (PI * wheelDiameter)) * encoderCountsPerRev;
}

int getEncoderCountsForAngle(float angle) {
    return (PI * trackWidth * angle) / 180 * encoderCountsPerRev;
}

void goStraight(float distance) {
    int counts = getEncoderCountsForDistance(abs(distance));
    Serial.print(counts)
    Serial.println();
    // Reset encoders
    leftEncoder.write(0);
    rightEncoder.write(0);
    
    if (distance > 0) {
        // FORWARD Logic
        void moveForward();

        while (leftEncoder.read() < counts && rightEncoder.read() < counts) {
        // Wait for the robot to reach the desired distance
        }

        stopRobot();

    } else {
        // REVERSE LOGIC
        void moveReverse()
        while (leftEncoder.read() < counts && rightEncoder.read() < counts) {
        // Wait for the robot to reach the desired distance
        }

        stopRobot();
    }
    leftEncoder.write(0);
    rightEncoder.write(0);

    return;
}


void turn(float angle) {
    long counts = getEncoderCountsForAngle(abs(angle));
    // Reset encoders
    leftEncoder.write(0);
    rightEncoder.write(0);
    
    if (angle > 0) {
        // RIGHT Logic
        turnRight();

        while (leftEncoder.read() < counts && rightEncoder.read() < counts) {
        // Wait for the robot to reach the desired distance
        }

        stopRobot();

    } else {
        // LEFT LOGIC
        turnLeft();

        while (leftEncoder.read() < counts && rightEncoder.read() < counts) {
        // Wait for the robot to reach the desired distance
        }

        stopRobot();
    }
    leftEncoder.write(0);
    rightEncoder.write(0);

    return;
}

void loop() {
  Serial.print("\nFORWARD");
  goStraight(2);
     
  Serial.print("\t BACKWARD");

  goStraight(-2);
//   delay(5000);

  Serial.print("\t STOP");
  stopRobot();
  delay(2000);
  
}
