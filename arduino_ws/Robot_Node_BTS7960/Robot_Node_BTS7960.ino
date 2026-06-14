/*
 * Robot_Node_BTS7960.ino  —  Simplified open-loop differential drive
 *
 * No velocity feedback or drift correction. If ROS says go straight, motors
 * go straight. Tune K_P and b to get the desired speed range.
 *
 * Serial Protocol:
 *   RX: "v,<linear_x>,<angular_z>\n"   (from ROS serial_bridge)
 *   TX: "t,<left_ticks>,<right_ticks>\n" (to ROS serial_bridge)
 *
 * Wiring:
 *   Left  BTS7960: RPWM→4, LPWM→5, R_EN→6,  L_EN→7
 *   Right BTS7960: RPWM→8, LPWM→9, R_EN→10, L_EN→11
 *   Left  encoder: A→21, B→20
 *   Right encoder: A→3,  B→2
 */

#define SERIAL_BAUD      115200
#define PUBLISH_INTERVAL 30     // ms between tick publishes

// ── Pin definitions ───────────────────────────────────────────────────────────

#define RPWM_LEFT   4
#define LPWM_LEFT   5
#define R_EN_LEFT   6
#define L_EN_LEFT   7

#define RPWM_RIGHT  8
#define LPWM_RIGHT  9
#define R_EN_RIGHT  10
#define L_EN_RIGHT  11

#define ENC_IN_LEFT_A   21
#define ENC_IN_LEFT_B   20
#define ENC_IN_RIGHT_A  3
#define ENC_IN_RIGHT_B  2

// ── Tunable parameters ────────────────────────────────────────────────────────

const double WHEEL_BASE = 0.36;   // meters, center-to-center of wheels

// PWM = K_P * |vel_m_s| + b, then clamped to [PWM_MIN, PWM_MAX]
const int K_P     = 278;
const int b       = 52;
const int PWM_MIN = 60;   // below this the motors stall — raise if needed
const int PWM_MAX = 100;

// ── Encoder state (modified by ISRs) ─────────────────────────────────────────

volatile int16_t left_wheel_tick_count  = 0;
volatile int16_t right_wheel_tick_count = 0;

// ── Runtime state ─────────────────────────────────────────────────────────────

int pwmLeftReq  = 0;   // signed: positive = forward
int pwmRightReq = 0;

unsigned long lastCmdVelReceived = 0;
unsigned long previousMillis     = 0;

String  inputString    = "";
boolean stringComplete = false;

// ── Encoder ISRs ──────────────────────────────────────────────────────────────

void right_wheel_tick() {
  boolean forward = (digitalRead(ENC_IN_RIGHT_B) != LOW);
  if (forward) {
    right_wheel_tick_count = (right_wheel_tick_count == 32767) ? -32768 : right_wheel_tick_count + 1;
  } else {
    right_wheel_tick_count = (right_wheel_tick_count == -32768) ? 32767 : right_wheel_tick_count - 1;
  }
}

void left_wheel_tick() {
  // Left encoder direction is physically inverted (motor faces opposite way)
  boolean forward = (digitalRead(ENC_IN_LEFT_B) == LOW);
  if (forward) {
    left_wheel_tick_count = (left_wheel_tick_count == 32767) ? -32768 : left_wheel_tick_count + 1;
  } else {
    left_wheel_tick_count = (left_wheel_tick_count == -32768) ? 32767 : left_wheel_tick_count - 1;
  }
}

// ── Motor helpers ─────────────────────────────────────────────────────────────

void set_motor(int rpwm_pin, int lpwm_pin, int signed_pwm) {
  int mag = constrain(abs(signed_pwm), 0, 255);
  if (signed_pwm >= 0) {
    analogWrite(rpwm_pin, mag);
    analogWrite(lpwm_pin, 0);
  } else {
    analogWrite(rpwm_pin, 0);
    analogWrite(lpwm_pin, mag);
  }
}

// Map a signed velocity (m/s) to a signed PWM value.
// Returns 0 if velocity is below the movement threshold.
int velToPwm(double vel) {
  if (fabs(vel) < 0.01) return 0;
  int mag = constrain((int)(K_P * fabs(vel) + b), PWM_MIN, PWM_MAX);
  return (vel > 0) ? mag : -mag;
}

// ── Velocity command ──────────────────────────────────────────────────────────

void processCmdVel(double linear_x, double angular_z) {
  lastCmdVelReceived = millis();

  // Standard differential-drive kinematics.
  // angular_z > 0 = CCW = turn left → left wheel backward, right wheel forward.
  double leftVel  = linear_x - (WHEEL_BASE / 2.0) * angular_z;
  double rightVel = linear_x + (WHEEL_BASE / 2.0) * angular_z;

  pwmLeftReq  = velToPwm(leftVel);
  pwmRightReq = velToPwm(rightVel);
}

// ── Serial ────────────────────────────────────────────────────────────────────

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

void parseCommand() {
  if (inputString.startsWith("v,")) {
    int c1 = inputString.indexOf(',');
    int c2 = inputString.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      double linear_x  = inputString.substring(c1 + 1, c2).toFloat();
      double angular_z = inputString.substring(c2 + 1).toFloat();
      processCmdVel(linear_x, angular_z);
    }
  }
}

void publishTicks() {
  noInterrupts();
  int16_t lt = left_wheel_tick_count;
  int16_t rt = right_wheel_tick_count;
  interrupts();
  // rt/lt swapped here to match expected ROS odometry frame
  Serial.print("t,");
  Serial.print(rt);
  Serial.print(",");
  Serial.println(lt);
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────

void setup() {
  // Enable both half-bridges on each BTS7960
  pinMode(R_EN_LEFT,  OUTPUT); digitalWrite(R_EN_LEFT,  HIGH);
  pinMode(L_EN_LEFT,  OUTPUT); digitalWrite(L_EN_LEFT,  HIGH);
  pinMode(R_EN_RIGHT, OUTPUT); digitalWrite(R_EN_RIGHT, HIGH);
  pinMode(L_EN_RIGHT, OUTPUT); digitalWrite(L_EN_RIGHT, HIGH);

  set_motor(RPWM_LEFT,  LPWM_LEFT,  0);
  set_motor(RPWM_RIGHT, LPWM_RIGHT, 0);

  // INPUT_PULLUP on all encoder pins reduces electrical noise
  pinMode(ENC_IN_LEFT_A,  INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B,  INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A),  left_wheel_tick,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  Serial.begin(SERIAL_BAUD);
  inputString.reserve(64);
}

void loop() {
  if (stringComplete) {
    parseCommand();
    inputString    = "";
    stringComplete = false;
  }

  if (millis() - previousMillis > PUBLISH_INTERVAL) {
    previousMillis = millis();
    publishTicks();
  }

  // Safety stop: zero motors if no command received for 1 second
  if (millis() - lastCmdVelReceived > 1000) {
    pwmLeftReq  = 0;
    pwmRightReq = 0;
  }

  set_motor(RPWM_LEFT,  LPWM_LEFT,  pwmLeftReq);
  set_motor(RPWM_RIGHT, LPWM_RIGHT, pwmRightReq);
}
