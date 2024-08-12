/**
 * @file Robot_Node.ino
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Motor.h>

// Define Pins here
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

// Define Robot Constants
const float wheelDiameter = 0.055;  // Set your wheel diameter in meters
const float encoderCountsPerRev = 360.0;  // Set your encoder counts per revolution
const float trackWidth = 0.0145;  // Set the distance between the two tracks in meters

// Create Motor Objects
Motor leftMotor(leftMotorIn1, leftMotorIn2, leftMotorEnable, leftEncoderPinA, leftEncoderPinB);
Motor rightMotor(rightMotorIn1, rightMotorIn2, rightMotorEnable, rightEncoderPinA, rightEncoderPinB);

// Configure ROS node
ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel;

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);

// Create ROS callback the updates motor speed
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  linearVel = msg.linear.x;
  angularVel = msg.angular.z;
  // Calculate Motor Speed
  int left_wheel_speed = (linearVel - angularVel) * 100; // Convert float to int.
  int right_wheel_speed = (linearVel + angularVel) * 100; // Convert float to int.

  // Set Motor Speed
  leftMotor.setMotor(left_wheel_speed);
  rightMotor.setMotor(right_wheel_speed);
}

void setup() {
  nh.initNode();
  nh.subscribe(cmd_vel_sub);

  leftMotor.stop();
  rightMotor.stop();
}

void loop() {
    while (!nh.connected()) {
        nh.spinOnce();
    }
    nh.spinOnce();
    delay(1);
}