#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>
#include <PID_v1.h>

// Define motor and encoder pins
const int leftMotorPin1 = 22;
const int leftMotorPin2 = 23;
const int rightMotorPin1 = 24;
const int rightMotorPin2 = 25;

const int leftEncoderPinA = 21;
const int leftEncoderPinB = 20;
const int rightEncoderPinA = 3;
const int rightEncoderPinB = 2;

// Motor and encoder parameters
const float wheel_diameter = 0.055; // meters
const float wheel_base = 0.0145;     // meters
const int encoder_counts_per_revolution = 540;

// Encoder objects
Encoder leftEncoder(leftEncoderPinA, leftEncoderPinB);
Encoder rightEncoder(rightEncoderPinA, rightEncoderPinB);

// PID parameters
double leftSetpoint, rightSetpoint;
double leftInput, rightInput;
double leftOutput, rightOutput;

// Initialize PID control
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, 2, 5, 1, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, 2, 5, 1, DIRECT);

// ROS node handle
ros::NodeHandle nh;

// Velocity command callback
void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  float linear_x = cmd_msg.linear.x;  // m/s
  float angular_z = cmd_msg.angular.z;  // rad/s

  // Calculate wheel speeds
  leftSetpoint = linear_x - (angular_z * wheel_base / 2.0);
  rightSetpoint = linear_x + (angular_z * wheel_base / 2.0);

//   // Update PID targets
//   leftPID.SetPoint(leftSetpoint);
//   rightPID.SetPoint(rightSetpoint);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);

// Function to set motor speeds using PWM
void setMotorSpeeds() {
  analogWrite(leftMotorPin1, leftOutput);
  analogWrite(leftMotorPin2, 0);
  analogWrite(rightMotorPin1, rightOutput);
  analogWrite(rightMotorPin2, 0);
}

void setup() {
  // Initialize ROS
  nh.initNode();
  nh.subscribe(sub);

  // Set motor pins as outputs
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Initialize PID controllers
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);

  // Set PID output limits (adjust as necessary)
  leftPID.SetOutputLimits(50, 255);
  rightPID.SetOutputLimits(50, 255);
}

void loop() {
  // Read encoder values
  long leftCounts = leftEncoder.read();
  long rightCounts = rightEncoder.read();

  // Convert encoder counts to speed (you need to adjust this based on your encoder specs)
  leftInput = (float)leftCounts / encoder_counts_per_revolution; // wheel revolutions per second
  rightInput = (float)rightCounts / encoder_counts_per_revolution;

  // Compute PID output
  leftPID.Compute();
  rightPID.Compute();

  // Set motor speeds
  setMotorSpeeds();

  // Reset encoders after each loop (if necessary)
  leftEncoder.write(0);
  rightEncoder.write(0);

  // Spin ROS
  nh.spinOnce();
  delay(5);  // Adjust the delay as needed
}
