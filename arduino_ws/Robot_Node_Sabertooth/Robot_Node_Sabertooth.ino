/*
 * Robot_Node_Sabertooth.ino
 *
 * Description: ROS node for Mark Five AMR using Sabertooth 2x12 motor driver.
 *              Publishes encoder ticks (/right_ticks, /left_ticks) and
 *              subscribes to /cmd_vel for differential drive control.
 *
 * Motor Driver: Sabertooth 2x12 v1.00 (Packetized Serial Mode)
 * Library: Dimension Engineering Sabertooth Arduino Library
 *
 * DIP Switches: OFF OFF ON ON ON ON (Packetized Serial, Address 128)
 *
 * Wiring:
 *   Pin 18 (TX1) --> Sabertooth S1
 *   GND --> Sabertooth 0V
 *   Encoders on pins 2, 3, 20, 21
 *
 * Based on original Robot_Node.ino for L293DNE H-Bridge.
 * Reference: Automatic Addison, Practical Robotics in C++
 */

#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <Sabertooth.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

////////////////// Sabertooth Configuration ///////////////////////////////////

// Sabertooth at address 128, using Serial1 (Pin 18 TX on Mega)
Sabertooth ST(128, Serial1);

////////////////// Tick Data Publishing Variables and Constants ///////////////

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 21
#define ENC_IN_RIGHT_A 3

// Other encoder output to Arduino to keep track of wheel direction
#define ENC_IN_LEFT_B 20
#define ENC_IN_RIGHT_B 2

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Min/max values for 16-bit integers (range of 65,535)
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

////////////////// Motor Controller Variables and Constants ///////////////////

// Number of ticks per wheel revolution
const int TICKS_PER_REVOLUTION = 540;

// Wheel radius in meters
const double WHEEL_RADIUS = 0.055;

// Distance from center of left tire to center of right tire in meters
const double WHEEL_BASE = 0.14;

// Number of ticks a wheel makes moving a linear distance of 1 meter
const double TICKS_PER_METER = 3125;

// Proportional constant for PWM-Linear Velocity relationship
const int K_P = 278;

// Y-intercept for PWM-Linear Velocity relationship
const int b = 52;

// Correction multiplier for drift
const int DRIFT_MULTIPLIER = 120;

// Turning PWM output (Sabertooth uses -127 to 127)
const int PWM_TURN = 60;

// Set minimum and maximum limits for the PWM values
const int PWM_MIN = 30;  // Minimum to overcome friction
const int PWM_MAX = 80;  // Limit max speed

// Velocity and PWM variables for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

/////////////////////// Tick Data Publishing Functions ////////////////////////

void right_wheel_tick() {
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false; // Reverse
  } else {
    Direction_right = true;  // Forward
  }

  if (Direction_right) {
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    } else {
      right_wheel_tick_count.data++;
    }
  } else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    } else {
      right_wheel_tick_count.data--;
    }
  }
}

void left_wheel_tick() {
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = true;  // Reverse
  } else {
    Direction_left = false; // Forward
  }

  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    } else {
      left_wheel_tick_count.data++;
    }
  } else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    } else {
      left_wheel_tick_count.data--;
    }
  }
}

/////////////////////// Motor Controller Functions ////////////////////////////

void calc_vel_left_wheel() {
  static double prevTime = 0;
  static int prevLeftCount = 0;

  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;

  if (numOfTicks > 10000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  velLeftWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
  prevLeftCount = left_wheel_tick_count.data;
  prevTime = (millis() / 1000.0);
}

void calc_vel_right_wheel() {
  static double prevTime = 0;
  static int prevRightCount = 0;

  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;

  if (numOfTicks > 10000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  velRightWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
  prevRightCount = right_wheel_tick_count.data;
  prevTime = (millis() / 1000.0);
}

void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  lastCmdVelReceived = (millis() / 1000.0);

  // Scale linear velocity to PWM (-127 to 127 for Sabertooth)
  pwmLeftReq = K_P * cmdVel.linear.x + b;
  pwmRightReq = K_P * cmdVel.linear.x + b;

  if (cmdVel.angular.z != 0.0) {
    if (cmdVel.angular.z > 0.0) {  // Turn left
      pwmLeftReq = -PWM_TURN;
      pwmRightReq = PWM_TURN;
    } else {  // Turn right
      pwmLeftReq = PWM_TURN;
      pwmRightReq = -PWM_TURN;
    }
  } else {  // Go straight - apply drift correction
    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = velLeftWheel - velRightWheel;
    double avgDifference = (prevDiff + prevPrevDiff + currDifference) / 3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;

    pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
    pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
  }

  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;
  }
}

void set_pwm_values() {
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;

  // Stop before switching direction
  if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
      (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  // Increase PWM if robot is not moving but should be
  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }

  // Gradually adjust output PWM
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += 1;
  } else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= 1;
  }

  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += 1;
  } else if (abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= 1;
  }

  // Limit PWM output (Sabertooth range: -127 to 127)
  pwmLeftOut = constrain(pwmLeftOut, 0, PWM_MAX);
  pwmRightOut = constrain(pwmRightOut, 0, PWM_MAX);

  // Apply direction sign
  int leftWithSign = (pwmLeftReq >= 0) ? pwmLeftOut : -pwmLeftOut;
  int rightWithSign = (pwmRightReq >= 0) ? pwmRightOut : -pwmRightOut;

  // Send commands to Sabertooth using library
  // Note: Swap motor(1) and motor(2) if wheels are reversed
  ST.motor(1, leftWithSign);
  ST.motor(2, rightWithSign);
}

// ROS subscriber to velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);

void setup() {
  // Initialize Sabertooth serial communication
  Serial1.begin(9600);
  ST.autobaud();

  delay(100);

  // Stop both motors on startup
  ST.motor(1, 0);
  ST.motor(2, 0);

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
}

void loop() {
  nh.spinOnce();

  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // Publish tick counts
    leftPub.publish(&left_wheel_tick_count);
    rightPub.publish(&right_wheel_tick_count);

    // Calculate wheel velocities
    calc_vel_right_wheel();
    calc_vel_left_wheel();
  }

  // Stop if no cmd_vel messages received for 1 second
  if ((millis() / 1000.0) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  set_pwm_values();
}
