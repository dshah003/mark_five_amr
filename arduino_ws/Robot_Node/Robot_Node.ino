/*
 * Robot_Node.ino (Simple Serial Protocol)
 *
 * Description: Arduino node for Mark Five AMR using L293DNE H-Bridge motor driver.
 *              Uses simple serial protocol instead of ros2arduino (memory efficient).
 *              Publishes encoder ticks and receives velocity commands via serial.
 *
 * Motor Driver: L293DNE H-Bridge
 *
 * Serial Protocol:
 *   TX (Arduino sends):  "t,<left_ticks>,<right_ticks>\n"
 *   RX (Arduino receives): "v,<linear_x>,<angular_z>\n"
 *
 * ROS2 Bridge: Run the serial_bridge node on the host to convert to ROS2 topics.
 *
 * Author: Automatic Addison (original), modified for simple serial protocol
 * Website: https://automaticaddison.com
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 */

// Serial configuration
#define SERIAL_BAUD 115200
#define PUBLISH_INTERVAL 30  // ms

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

// Left Motor connections
const int enA = 7;
const int in1 = 22;
const int in2 = 23;

// Right Motor connections
const int enB = 6;
const int in3 = 24;
const int in4 = 25;

// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

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

// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 80;

// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 50;   // about 0.1 m/s
const int PWM_MAX = 100;  // about 0.172 m/s

// Set linear velocity and PWM variable values for each wheel
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

  // Calculate the PWM value given the desired velocity
  pwmLeftReq = K_P * linear_x + b;
  pwmRightReq = K_P * linear_x + b;

  // Check if we need to turn
  if (angular_z != 0.0) {
    // Turn left
    if (angular_z > 0.0) {
      pwmLeftReq = -PWM_TURN;
      pwmRightReq = PWM_TURN;
    }
    // Turn right
    else {
      pwmLeftReq = PWM_TURN;
      pwmRightReq = -PWM_TURN;
    }
  }
  // Go straight
  else {
    // Remove any differences in wheel velocities
    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = velLeftWheel - velRightWheel;
    double avgDifference = (prevDiff + prevPrevDiff + currDifference) / 3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;

    // Correct PWM values of both wheels to make the vehicle go straight
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

  // If the required PWM is of opposite sign as the output PWM, stop first
  if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
      (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  // Set the direction of the motors
  if (pwmLeftReq > 0) {  // Left wheel forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwmLeftReq < 0) {  // Left wheel reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {  // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (pwmRightReq > 0) {  // Right wheel forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (pwmRightReq < 0) {  // Right wheel reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {  // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  // Increase the required PWM if the robot is not moving
  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }

  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  } else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;
  }

  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  } else if (abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  }

  // Limit PWM output
  pwmLeftOut = constrain(pwmLeftOut, 0, PWM_MAX);
  pwmRightOut = constrain(pwmRightOut, 0, PWM_MAX);

  // Set the PWM value on the pins
  analogWrite(enA, pwmLeftOut);
  analogWrite(enB, pwmRightOut);
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
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  // Motor control pins are outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);

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
