/*
 * Robot_Node_Sabertooth.ino (Simple Serial Protocol)
 *
 * Description: Arduino node for Mark Five AMR using Sabertooth 2x12 motor driver.
 *              Uses simple serial protocol instead of ros2arduino (memory efficient).
 *              Publishes encoder ticks and receives velocity commands via serial.
 *
 * Motor Driver: Sabertooth 2x12 v1.00 (Packetized Serial Mode)
 * Library: Dimension Engineering Sabertooth Arduino Library
 *
 * DIP Switches: OFF OFF ON ON ON ON (Packetized Serial, Address 128)
 *
 * Serial Protocol:
 *   TX (Arduino sends):  "t,<left_ticks>,<right_ticks>\n"
 *   RX (Arduino receives): "v,<linear_x>,<angular_z>\n"
 *
 * Wiring:
 *   Pin 18 (TX1) --> Sabertooth S1
 *   GND --> Sabertooth 0V
 *   Encoders on pins 2, 3, 20, 21
 *
 * ROS2 Bridge: Run the serial_bridge node on the host to convert to ROS2 topics.
 *
 * Based on original Robot_Node.ino for L293DNE H-Bridge.
 * Reference: Automatic Addison, Practical Robotics in C++
 */

#include <Sabertooth.h>

// Serial configuration
#define SERIAL_BAUD 115200
#define PUBLISH_INTERVAL 30  // ms

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
volatile int16_t right_wheel_tick_count = 0;
volatile int16_t left_wheel_tick_count = 0;

// Time interval for measurements in milliseconds
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

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
const int PWM_TURN = 40;

// Set minimum and maximum limits for the PWM values
const int PWM_MIN = 30;  // Minimum to overcome friction
const int PWM_MAX = 50;  // Limit max speed

// Velocity and PWM variables for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;

// Record the time that the last velocity command was received
unsigned long lastCmdVelReceived = 0;

// Serial receive buffer
String inputString = "";
boolean stringComplete = false;

/////////////////////// Tick Data Publishing Functions ////////////////////////

void right_wheel_tick() {
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false; // Reverse
  } else {
    Direction_right = true;  // Forward
  }

  if (Direction_right) {
    if (right_wheel_tick_count == encoder_maximum) {
      right_wheel_tick_count = encoder_minimum;
    } else {
      right_wheel_tick_count++;
    }
  } else {
    if (right_wheel_tick_count == encoder_minimum) {
      right_wheel_tick_count = encoder_maximum;
    } else {
      right_wheel_tick_count--;
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
    if (left_wheel_tick_count == encoder_maximum) {
      left_wheel_tick_count = encoder_minimum;
    } else {
      left_wheel_tick_count++;
    }
  } else {
    if (left_wheel_tick_count == encoder_minimum) {
      left_wheel_tick_count = encoder_maximum;
    } else {
      left_wheel_tick_count--;
    }
  }
}

/////////////////////// Motor Controller Functions ////////////////////////////

void calc_vel_left_wheel() {
  static unsigned long prevTime = 0;
  static int prevLeftCount = 0;

  int numOfTicks = (65535 + left_wheel_tick_count - prevLeftCount) % 65535;

  if (numOfTicks > 10000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  unsigned long currentTime = millis();
  if (currentTime > prevTime) {
    velLeftWheel = (double)numOfTicks / TICKS_PER_METER / ((currentTime - prevTime) / 1000.0);
  }
  prevLeftCount = left_wheel_tick_count;
  prevTime = currentTime;
}

void calc_vel_right_wheel() {
  static unsigned long prevTime = 0;
  static int prevRightCount = 0;

  int numOfTicks = (65535 + right_wheel_tick_count - prevRightCount) % 65535;

  if (numOfTicks > 10000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  unsigned long currentTime = millis();
  if (currentTime > prevTime) {
    velRightWheel = (double)numOfTicks / TICKS_PER_METER / ((currentTime - prevTime) / 1000.0);
  }
  prevRightCount = right_wheel_tick_count;
  prevTime = currentTime;
}

void processCmdVel(double linear_x, double angular_z) {
  lastCmdVelReceived = millis();

  // Scale linear velocity to PWM (-127 to 127 for Sabertooth)
  // Apply offset 'b' with proper sign for forward/backward motion
  if (linear_x >= 0) {
    pwmLeftReq = K_P * linear_x + b;
    pwmRightReq = K_P * linear_x + b;
  } else {
    pwmLeftReq = K_P * linear_x - b;
    pwmRightReq = K_P * linear_x - b;
  }

  if (angular_z != 0.0) {
    if (angular_z > 0.0) {  // Turn left
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
  // Accelerate slowly (+1), but decelerate quickly (-5) for safety
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += 1;
  } else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= 5;  // Faster deceleration for quick stops
    if (pwmLeftOut < 0) pwmLeftOut = 0;
  }

  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += 1;
  } else if (abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= 5;  // Faster deceleration for quick stops
    if (pwmRightOut < 0) pwmRightOut = 0;
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

/////////////////////// Serial Communication //////////////////////////////////

void parseCommand() {
  if (inputString.startsWith("v,")) {
    // Parse velocity command: "v,<linear_x>,<angular_z>\n"
    int firstComma = inputString.indexOf(',');
    int secondComma = inputString.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      double linear_x = inputString.substring(firstComma + 1, secondComma).toFloat();
      double angular_z = inputString.substring(secondComma + 1).toFloat();
      processCmdVel(linear_x, angular_z);
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void publishTicks() {
  // Send tick counts: "t,<left_ticks>,<right_ticks>\n"
  Serial.print("t,");
  Serial.print(left_wheel_tick_count);
  Serial.print(",");
  Serial.println(right_wheel_tick_count);
}

/////////////////////// Setup and Loop ////////////////////////////////////////

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

  // USB Serial for communication with ROS2 bridge
  Serial.begin(SERIAL_BAUD);
  inputString.reserve(64);
}

void loop() {
  currentMillis = millis();

  // Process any received serial commands
  if (stringComplete) {
    parseCommand();
    inputString = "";
    stringComplete = false;
  }

  // Publish ticks at regular intervals
  if (currentMillis - previousMillis > PUBLISH_INTERVAL) {
    previousMillis = currentMillis;
    publishTicks();

    // Calculate wheel velocities
    calc_vel_right_wheel();
    calc_vel_left_wheel();
  }

  // Stop if no cmd_vel messages received for 1 second
  if (millis() - lastCmdVelReceived > 1000) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  set_pwm_values();
}
