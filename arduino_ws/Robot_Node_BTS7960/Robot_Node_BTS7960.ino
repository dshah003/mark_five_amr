/*
 * Robot_Node_BTS7960.ino (Simple Serial Protocol)
 *
 * Description: Arduino node for Mark Five AMR using 2x BTS7960 motor driver modules.
 *              Uses simple serial protocol instead of ros2arduino (memory efficient).
 *              Publishes encoder ticks and receives velocity commands via serial.
 *
 * Motor Driver: 2x BTS7960 (IBT-2) H-Bridge modules
 *               - One module per motor
 *               - Direct PWM control (no library needed)
 *               - PWM range 0-255 per direction
 *
 * Serial Protocol:
 *   TX (Arduino sends):  "t,<left_ticks>,<right_ticks>\n"
 *   RX (Arduino receives): "v,<linear_x>,<angular_z>\n"
 *
 * Wiring (see README.md for full diagram):
 *   Left motor  BTS7960: RPWM→4, LPWM→5, R_EN→6, L_EN→7
 *   Right motor BTS7960: RPWM→8, LPWM→9, R_EN→10, L_EN→11
 *   Encoders: left A/B→21/20, right A/B→3/2
 *
 * ROS2 Bridge: Run the serial_bridge node on the host to convert to ROS2 topics.
 */

// Serial configuration
#define SERIAL_BAUD 115200
#define PUBLISH_INTERVAL 30  // ms

// BTS7960 pin definitions - Left motor
#define RPWM_LEFT   4   // Forward PWM
#define LPWM_LEFT   5   // Reverse PWM
#define R_EN_LEFT   6   // Enable (right half-bridge)
#define L_EN_LEFT   7   // Enable (left half-bridge)

// BTS7960 pin definitions - Right motor
#define RPWM_RIGHT  8   // Forward PWM
#define LPWM_RIGHT  9   // Reverse PWM
#define R_EN_RIGHT  10  // Enable (right half-bridge)
#define L_EN_RIGHT  11  // Enable (left half-bridge)

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

// Wheel radius in meters (circumference 230mm measured → r = 230/(2π) = 36.6mm)
const double WHEEL_RADIUS = 0.0366;

// Distance from center of left tire to center of right tire in meters
const double WHEEL_BASE = 0.36;

// Number of ticks a wheel makes moving a linear distance of 1 meter
// 540 ticks/rev ÷ 0.23m circumference = 2348 — recalibrate with measured drive
const double TICKS_PER_METER = 2348;

// Proportional constant for PWM-Linear Velocity relationship
// BTS7960 uses 0-255 scale (vs -127..127 for Sabertooth) — retune after first test
const int K_P = 278;

// Y-intercept for PWM-Linear Velocity relationship
const int b = 52;

// Correction multiplier for drift
const int DRIFT_MULTIPLIER = 120;

// Turning PWM output (0-255 scale)
const int PWM_TURN = 80;

// Set minimum and maximum limits for the PWM values (0-255 scale)
const int PWM_MIN = 50;   // Minimum to overcome friction
const int PWM_MAX = 160;  // Max speed (increase from 100 once motors are confirmed working)

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

  // Map linear velocity to PWM. Apply offset 'b' with correct sign.
  // Explicitly zero when linear_x == 0 — the >= case would give pwmReq = b = 52, which
  // doesn't stop the motors since 52 > PWM_MIN.
  if (linear_x > 0) {
    pwmLeftReq = K_P * linear_x + b;
    pwmRightReq = K_P * linear_x + b;
  } else if (linear_x < 0) {
    pwmLeftReq = K_P * linear_x - b;
    pwmRightReq = K_P * linear_x - b;
  } else {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  if (angular_z != 0.0) {
    // Note: Signs inverted to match physical motor/encoder configuration
    if (angular_z > 0.0) {  // Turn left
      pwmLeftReq = PWM_TURN;
      pwmRightReq = -PWM_TURN;
    } else {  // Turn right
      pwmLeftReq = -PWM_TURN;
      pwmRightReq = PWM_TURN;
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

  // Zero out requests below minimum (deadband)
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;
  }
}

void set_motor(int rpwm_pin, int lpwm_pin, int signed_pwm) {
  // Drive one BTS7960 module.
  // signed_pwm > 0 → forward (RPWM active), < 0 → reverse (LPWM active)
  int magnitude = constrain(abs(signed_pwm), 0, 255);
  if (signed_pwm >= 0) {
    analogWrite(rpwm_pin, magnitude);
    analogWrite(lpwm_pin, 0);
  } else {
    analogWrite(rpwm_pin, 0);
    analogWrite(lpwm_pin, magnitude);
  }
}

void set_pwm_values() {
  // Direct PWM — no ramp. The ramp (+N per iteration) ran at ~100kHz Arduino loop speed,
  // completing in microseconds with no real smoothing effect and causing kickstart oscillation.
  // Smooth accel/decel can be added back time-gated to the 30ms tick interval once basic
  // motion is confirmed working.
  int leftOut  = constrain((int)abs(pwmLeftReq),  0, PWM_MAX);
  int rightOut = constrain((int)abs(pwmRightReq), 0, PWM_MAX);

  int leftWithSign  = (pwmLeftReq  >= 0) ? leftOut  : -leftOut;
  int rightWithSign = (pwmRightReq >= 0) ? rightOut : -rightOut;

  set_motor(RPWM_LEFT,  LPWM_LEFT,  leftWithSign);
  set_motor(RPWM_RIGHT, LPWM_RIGHT, rightWithSign);
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
  // Swapped to correct RViz rotation direction
  Serial.print("t,");
  Serial.print(right_wheel_tick_count);
  Serial.print(",");
  Serial.println(left_wheel_tick_count);
}

/////////////////////// Setup and Loop ////////////////////////////////////////

void setup() {
  // BTS7960 enable pins — set HIGH to activate both half-bridges on each module
  pinMode(R_EN_LEFT,  OUTPUT); digitalWrite(R_EN_LEFT,  HIGH);
  pinMode(L_EN_LEFT,  OUTPUT); digitalWrite(L_EN_LEFT,  HIGH);
  pinMode(R_EN_RIGHT, OUTPUT); digitalWrite(R_EN_RIGHT, HIGH);
  pinMode(L_EN_RIGHT, OUTPUT); digitalWrite(L_EN_RIGHT, HIGH);

  // Stop both motors on startup
  set_motor(RPWM_LEFT,  LPWM_LEFT,  0);
  set_motor(RPWM_RIGHT, LPWM_RIGHT, 0);

  // Encoder pin setup
  pinMode(ENC_IN_LEFT_A,  INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B,  INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A),  left_wheel_tick,  RISING);
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

    calc_vel_right_wheel();
    calc_vel_left_wheel();
  }

  // Stop if no cmd_vel received for 1 second
  if (millis() - lastCmdVelReceived > 1000) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  set_pwm_values();
}
